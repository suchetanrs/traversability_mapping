/* Traversability Mapping - A global and local traversability mapping algorithm.
 * Copyright (C) 2024 Suchetan Saravanan and Damien Vivet
 *
 * Moment-fused GLOBAL traversability node.
 *
 * NOTE: the file/executable is still named "local_traversability" for build
 * compatibility, but it now owns a single fixed-frame, growing GLOBAL grid_map
 * whose layers hold the fused moments across all keyframes. Each cell's plane
 * normal is therefore fitted over ALL points that fall in its vicinity, not
 * per-keyframe-then-fused.
 *
 * Pipeline (additions, GT-driven):
 *   KeyFrameAdditions -> transform cloud to map -> filter -> bin into
 *   cell-local moments on the absolute lattice -> ADD into the grid moment
 *   layers -> recompute hazards over the dilated dirty set. A 1 Hz timer
 *   publishes the whole map. KeyFrameUpdates (PGO) subtract the old
 *   contribution, transform it, and re-add (frozen partition).
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <traversability_msgs/msg/key_frame_additions.hpp>
#include <traversability_msgs/msg/key_frame_updates.hpp>

#include "traversability_mapping/KeyFrame.hpp"
#include "traversability_mapping/Moments.hpp"
#include "traversability_mapping/TraversabilityMetrics.hpp"
#include "traversability_mapping/Helpers.hpp"
#include "traversability_mapping/Parameters.hpp"

#include "common.hpp"

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <memory>
#include <cmath>

namespace tmap = traversability_mapping;

class GlobalTraversabilityNode : public rclcpp::Node
{
public:
    GlobalTraversabilityNode() : Node("local_traversability_node")
    {
        // --- ROS params (node name + keys match traversability_gt_ros_params.yaml) ---
        declare_parameter("additions_topic_name", rclcpp::ParameterValue("traversability_keyframe_additions"));
        get_parameter("additions_topic_name", additions_topic_);
        declare_parameter("updates_topic_name", rclcpp::ParameterValue("traversability_keyframe_updates"));
        get_parameter("updates_topic_name", updates_topic_);
        declare_parameter("slam_frame", rclcpp::ParameterValue("base_footprint"));
        get_parameter("slam_frame", slam_frame_);
        declare_parameter("robot_base_frame", rclcpp::ParameterValue("base_footprint"));
        get_parameter("robot_base_frame", robot_base_frame_);
        declare_parameter("lidar_frame", rclcpp::ParameterValue("lidar_link"));
        get_parameter("lidar_frame", lidar_frame_);
        declare_parameter("map_frame", rclcpp::ParameterValue("map"));
        get_parameter("map_frame", map_frame_);

        std::string parameter_file_path;
        declare_parameter("parameter_file_path", rclcpp::ParameterValue(""));
        get_parameter("parameter_file_path", parameter_file_path);
        tmap::ParameterHandler::getInstance(parameter_file_path);

        // --- Cache traversability params ---
        res_ = parameterInstance.getValue<double>("resolution_local_map");
        ground_clearance_ = parameterInstance.getValue<double>("ground_clearance");
        max_slope_ = parameterInstance.getValue<double>("max_slope");
        robot_height_ = parameterInstance.getValue<double>("robot_height");
        max_range_base_frame_ = parameterInstance.getValue<double>("max_range_base_frame");
        min_range_base_frame_ = parameterInstance.getValue<double>("min_range_base_frame");
        security_distance_ = parameterInstance.getValue<double>("security_distance");
        min_vicinity_points_ = static_cast<unsigned int>(parameterInstance.getValue<int>("min_vicinity_points"));
        min_occupied_fraction_ = parameterInstance.getValue<double>("min_occupied_fraction");
        // Vicinity radius in cells (symmetric window of side 2*delta_ind_+1).
        delta_ind_ = std::max(1, static_cast<int>(std::ceil((security_distance_ / 2.0) / res_)));

        // --- Fixed global lattice ---
        const double cx = parameterInstance.getValue<double>("grid_center_x");
        const double cy = parameterInstance.getValue<double>("grid_center_y");
        lattice_ = tmap::Lattice(cx, cy, res_);

        // --- Extrinsics from TF ---
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
        populateTransforms(slam_frame_, robot_base_frame_, lidar_frame_, get_clock(), get_logger(),
                           tf_buffer_, Tsv_, Tbs_);
        Tbv_ = Tbs_ * Tsv_;  // base <- lidar

        // --- Global grid_map ---
        layers_ = {"N", "sx", "sy", "sz", "sx2", "sy2", "sz2", "sxy", "sxz", "syz",
                   "hazard", "elevation", "slope_haz", "step_haz", "roughness_haz"};
        const double half = parameterInstance.getValue<double>("half_size_traversability");
        gridMap_ = freshMap(half, half);

        // --- ROS I/O ---
        additions_sub_ = create_subscription<traversability_msgs::msg::KeyFrameAdditions>(
            additions_topic_, 10,
            std::bind(&GlobalTraversabilityNode::additionsCallback, this, std::placeholders::_1));
        updates_sub_ = create_subscription<traversability_msgs::msg::KeyFrameUpdates>(
            updates_topic_, 10,
            std::bind(&GlobalTraversabilityNode::updatesCallback, this, std::placeholders::_1));

        occupancy_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
            "global_traversability_map", rclcpp::QoS(1).transient_local());
        gridmap_pub_ = create_publisher<grid_map_msgs::msg::GridMap>(
            "global_traversability_gridmap", rclcpp::QoS(1).transient_local());

        const double rate = parameterInstance.getValue<double>("publish_rate_hz");
        publish_timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / std::max(0.1, rate)),
            std::bind(&GlobalTraversabilityNode::publish, this));

        RCLCPP_INFO(get_logger(), "Global traversability node ready (res=%.3f, vicinity=%d).", res_, delta_ind_);
    }

private:
    // ---- grid_map construction / growth ------------------------------------

    // Build a fresh grid centred on the lattice origin with an ODD cell count
    // per axis so cell centres land exactly on the lattice. Position stays the
    // lattice origin forever; growth is symmetric -> cells never shift off the
    // absolute lattice, so partials keyed by absolute id remain valid.
    grid_map::GridMap freshMap(double halfX, double halfY)
    {
        const int kx = static_cast<int>(std::ceil(halfX / res_));
        const int ky = static_cast<int>(std::ceil(halfY / res_));
        grid_map::GridMap m(layers_);
        m.setFrameId(map_frame_);
        m.setGeometry(grid_map::Length((2 * kx + 1) * res_, (2 * ky + 1) * res_), res_,
                      grid_map::Position(lattice_.x0, lattice_.y0));
        for (const auto &l : layers_)
            m[l].setConstant(std::numeric_limits<float>::quiet_NaN());
        return m;
    }

    // Grow (symmetrically, preserving the lattice) so [minx,maxx]x[miny,maxy]
    // fits with a one-cell margin. Copies existing data across by world position.
    void growToInclude(double minx, double maxx, double miny, double maxy)
    {
        const double margin = res_;
        const double curHalfX = gridMap_.getLength().x() / 2.0;
        const double curHalfY = gridMap_.getLength().y() / 2.0;
        const double needX = std::max(std::abs(maxx - lattice_.x0), std::abs(minx - lattice_.x0)) + margin;
        const double needY = std::max(std::abs(maxy - lattice_.y0), std::abs(miny - lattice_.y0)) + margin;
        if (needX <= curHalfX && needY <= curHalfY)
            return;

        const double extend = parameterInstance.getValue<double>("extend_length_every_resize_by");
        double newHalfX = curHalfX, newHalfY = curHalfY;
        while (newHalfX < needX) newHalfX += extend;
        while (newHalfY < needY) newHalfY += extend;

        grid_map::GridMap old = gridMap_;
        gridMap_ = freshMap(newHalfX, newHalfY);
        for (grid_map::GridMapIterator it(old); !it.isPastEnd(); ++it)
        {
            grid_map::Position p;
            old.getPosition(*it, p);
            if (!gridMap_.isInside(p))
                continue;
            for (const auto &l : layers_)
            {
                const float v = old.at(l, *it);
                if (!std::isnan(v))
                    gridMap_.atPosition(l, p) = v;
            }
        }
        RCLCPP_INFO(get_logger(), "Grew global grid to %.1f x %.1f m.",
                    gridMap_.getLength().x(), gridMap_.getLength().y());
    }

    // ---- moment <-> grid layer helpers -------------------------------------

    grid_map::Position cellPos(int ci, int cj) const
    {
        const Eigen::Vector2d c = lattice_.centerOf(ci, cj);
        return grid_map::Position(c.x(), c.y());
    }

    inline void addToLayer(const std::string &l, const grid_map::Position &p, double v)
    {
        float &cell = gridMap_.atPosition(l, p);
        if (std::isnan(cell)) cell = 0.f;
        cell += static_cast<float>(v);
    }

    void addPartialToGrid(std::uint64_t cellId, const tmap::NodeMetaData &m, double sign)
    {
        int ci, cj;
        tmap::Lattice::unkey(cellId, ci, cj);
        const grid_map::Position p = cellPos(ci, cj);
        if (!gridMap_.isInside(p))
            return;
        addToLayer("N", p, sign * m.N);
        addToLayer("sx", p, sign * m.sx);   addToLayer("sy", p, sign * m.sy);   addToLayer("sz", p, sign * m.sz);
        addToLayer("sx2", p, sign * m.sx2); addToLayer("sy2", p, sign * m.sy2); addToLayer("sz2", p, sign * m.sz2);
        addToLayer("sxy", p, sign * m.sxy); addToLayer("sxz", p, sign * m.sxz); addToLayer("syz", p, sign * m.syz);
        // If a subtraction emptied the cell, blank its moment + derived layers.
        if (sign < 0 && std::lround(gridMap_.atPosition("N", p)) <= 0)
            blankCell(p);
    }

    void blankCell(const grid_map::Position &p)
    {
        for (const auto &l : layers_)
            gridMap_.atPosition(l, p) = std::numeric_limits<float>::quiet_NaN();
    }

    bool readCellMoment(int ci, int cj, tmap::NodeMetaData &out) const
    {
        const grid_map::Position p = cellPos(ci, cj);
        if (!gridMap_.isInside(p))
            return false;
        const float n = gridMap_.atPosition("N", p);
        if (std::isnan(n) || n < 1.f)
            return false;
        out.N = static_cast<unsigned int>(std::lround(n));
        out.sx = gridMap_.atPosition("sx", p);   out.sy = gridMap_.atPosition("sy", p);   out.sz = gridMap_.atPosition("sz", p);
        out.sx2 = gridMap_.atPosition("sx2", p); out.sy2 = gridMap_.atPosition("sy2", p); out.sz2 = gridMap_.atPosition("sz2", p);
        out.sxy = gridMap_.atPosition("sxy", p); out.sxz = gridMap_.atPosition("sxz", p); out.syz = gridMap_.atPosition("syz", p);
        return true;
    }

    // ---- recompute ----------------------------------------------------------

    std::unordered_set<std::uint64_t> dilate(const std::unordered_set<std::uint64_t> &touched) const
    {
        std::unordered_set<std::uint64_t> out;
        out.reserve(touched.size() * (2 * delta_ind_ + 1) * (2 * delta_ind_ + 1));
        for (auto id : touched)
        {
            int ci, cj;
            tmap::Lattice::unkey(id, ci, cj);
            for (int di = -delta_ind_; di <= delta_ind_; ++di)
                for (int dj = -delta_ind_; dj <= delta_ind_; ++dj)
                    out.insert(tmap::Lattice::key(ci + di, cj + dj));
        }
        return out;
    }

    void recomputeDirty(const std::unordered_set<std::uint64_t> &dirty)
    {
        for (auto id : dirty)
        {
            int ci, cj;
            tmap::Lattice::unkey(id, ci, cj);
            const grid_map::Position qp = cellPos(ci, cj);
            if (!gridMap_.isInside(qp))
                continue;

            tmap::NodeMetaData qd;
            if (!readCellMoment(ci, cj, qd))
                continue;  // query cell unobserved -> nothing to write

            tmap::CellMoment query{qd, Eigen::Vector3d(qp.x(), qp.y(), 0.0)};
            std::vector<tmap::CellMoment> occupied;
            int total = 0;
            for (int i = ci - delta_ind_; i <= ci + delta_ind_; ++i)
                for (int j = cj - delta_ind_; j <= cj + delta_ind_; ++j)
                {
                    ++total;
                    tmap::NodeMetaData d;
                    if (readCellMoment(i, j, d))
                    {
                        const Eigen::Vector2d c2 = lattice_.centerOf(i, j);
                        occupied.push_back({d, Eigen::Vector3d(c2.x(), c2.y(), 0.0)});
                    }
                }

            const auto haz = tmap::computeGoodness(query, occupied, total, ground_clearance_,
                                                   max_slope_, min_vicinity_points_, min_occupied_fraction_);

            if (!std::isnan(haz[tmap::HAZ_ELEVATION]))
                gridMap_.atPosition("elevation", qp) = static_cast<float>(haz[tmap::HAZ_ELEVATION]);

            if (std::isnan(haz[tmap::HAZ_OVERALL]))
            {
                gridMap_.atPosition("hazard", qp) = std::numeric_limits<float>::quiet_NaN();
                gridMap_.atPosition("slope_haz", qp) = std::numeric_limits<float>::quiet_NaN();
                gridMap_.atPosition("step_haz", qp) = std::numeric_limits<float>::quiet_NaN();
                gridMap_.atPosition("roughness_haz", qp) = std::numeric_limits<float>::quiet_NaN();
                continue;
            }
            gridMap_.atPosition("hazard", qp) = static_cast<float>(haz[tmap::HAZ_OVERALL]);
            gridMap_.atPosition("slope_haz", qp) = static_cast<float>(haz[tmap::HAZ_SLOPE]);
            gridMap_.atPosition("step_haz", qp) = static_cast<float>(haz[tmap::HAZ_STEP]);
            gridMap_.atPosition("roughness_haz", qp) = static_cast<float>(haz[tmap::HAZ_ROUGHNESS]);
        }
    }

    // ---- callbacks ----------------------------------------------------------

    static Eigen::Affine3f poseToAffine(const geometry_msgs::msg::Pose &pose)
    {
        Eigen::Translation3f t(pose.position.x, pose.position.y, pose.position.z);
        Eigen::Quaternionf q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        return t * q;
    }

    void additionsCallback(const traversability_msgs::msg::KeyFrameAdditions::SharedPtr msg)
    {
        for (const auto &kf : msg->keyframes)
            processAddition(kf);
    }

    void processAddition(const traversability_msgs::msg::KeyFrame &kf_msg)
    {
        if (keyframes_.count(kf_msg.kf_id))
        {
            RCLCPP_WARN(get_logger(), "Duplicate keyframe id %lu ignored.", (unsigned long)kf_msg.kf_id);
            return;
        }

        const Eigen::Affine3f Tmb = poseToAffine(kf_msg.kf_pose);
        const Eigen::Affine3f Tmv = Tmb * Tbv_;  // map <- lidar

        pcl::PointCloud<pcl::PointXYZ> cloud_lidar;
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(kf_msg.kf_pointcloud, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, cloud_lidar);

        // Pass 1: filter (ego + height) and transform to map; track AABB.
        std::vector<Eigen::Vector3f> map_pts;
        map_pts.reserve(cloud_lidar.size());
        double minx = 1e18, maxx = -1e18, miny = 1e18, maxy = -1e18;
        for (const auto &p : cloud_lidar)
        {
            if (p.x == 0.f && p.y == 0.f)
                continue;  // ego / invalid return
            const Eigen::Vector3f p_base = Tbv_ * Eigen::Vector3f(p.x, p.y, p.z);
            if (p_base.z() > static_cast<float>(robot_height_))
                continue;  // ceiling / overhang
            const float range = p_base.norm();
            if (range > static_cast<float>(max_range_base_frame_) ||
                range < static_cast<float>(min_range_base_frame_))
                continue;  // out of range
            const Eigen::Vector3f p_map = Tmb * p_base;
            map_pts.push_back(p_map);
            minx = std::min(minx, (double)p_map.x()); maxx = std::max(maxx, (double)p_map.x());
            miny = std::min(miny, (double)p_map.y()); maxy = std::max(maxy, (double)p_map.y());
        }
        if (map_pts.empty())
            return;

        growToInclude(minx, maxx, miny, maxy);

        // Pass 2: bin into cell-local moments held by the keyframe.
        auto kf = std::make_shared<tmap::KeyFrame>(kf_msg.kf_id, Tmb);
        for (const auto &p_map : map_pts)
        {
            int ci, cj;
            lattice_.cellOf(p_map.x(), p_map.y(), ci, cj);
            const Eigen::Vector2d c = lattice_.centerOf(ci, cj);
            kf->addPoint(tmap::Lattice::key(ci, cj),
                         p_map.x() - c.x(), p_map.y() - c.y(), p_map.z());  // z about 0
        }

        // Fuse the keyframe's partials into the global moment layers.
        std::unordered_set<std::uint64_t> touched;
        for (const auto &kv : kf->partials())
        {
            addPartialToGrid(kv.first, kv.second, +1.0);
            touched.insert(kv.first);
        }
        keyframes_[kf_msg.kf_id] = kf;

        recomputeDirty(dilate(touched));
    }

    void updatesCallback(const traversability_msgs::msg::KeyFrameUpdates::SharedPtr msg)
    {
        // PGO pose corrections. Frozen partition: each partial is transformed in
        // place about its own cell centre, subtracted then re-added; the cell
        // membership never changes. (Exercised once a real SLAM provides updates.)
        std::unordered_set<std::uint64_t> touched;
        for (const auto &kf_msg : msg->keyframes)
        {
            auto it = keyframes_.find(kf_msg.kf_id);
            if (it == keyframes_.end())
                continue;
            auto &kf = it->second;

            const Eigen::Affine3f newPose = poseToAffine(kf_msg.kf_pose);
            const Eigen::Affine3f delta = newPose * kf->pose().inverse();  // map-frame delta T_d
            const Eigen::Matrix3d Rd = delta.linear().cast<double>();
            const Eigen::Vector3d td = delta.translation().cast<double>();

            for (auto &kv : kf->partials())
            {
                int ci, cj;
                tmap::Lattice::unkey(kv.first, ci, cj);
                const Eigen::Vector2d c = lattice_.centerOf(ci, cj);
                const Eigen::Vector3d c3(c.x(), c.y(), 0.0);

                addPartialToGrid(kv.first, kv.second, -1.0);                 // remove old
                const Eigen::Vector3d t_eff = Rd * c3 + td - c3;             // delta about cell centre
                kv.second.transform(Rd, t_eff);                             // transform partial
                addPartialToGrid(kv.first, kv.second, +1.0);                 // re-add transformed
                touched.insert(kv.first);
            }
            kf->setPose(newPose);
        }
        if (!touched.empty())
            recomputeDirty(dilate(touched));
    }

    // ---- publish ------------------------------------------------------------

    void publish()
    {
        if (occupancy_pub_->get_subscription_count() > 0)
        {
            nav_msgs::msg::OccupancyGrid og;
            tmap::gridMapToOccupancyGrid(gridMap_, "hazard", 0.f, 1.f, og);  // sets frame + origin
            og.header.stamp = now();
            occupancy_pub_->publish(og);
        }

        if (gridmap_pub_->get_subscription_count() > 0)
        {
            auto gm = grid_map::GridMapRosConverter::toMessage(gridMap_);
            gm->header.frame_id = map_frame_;
            gm->header.stamp = now();
            gridmap_pub_->publish(*gm);
        }
    }

    // ---- members ------------------------------------------------------------
    std::string additions_topic_, updates_topic_;
    std::string slam_frame_, robot_base_frame_, lidar_frame_, map_frame_;

    tmap::Lattice lattice_;
    std::vector<std::string> layers_;
    grid_map::GridMap gridMap_;
    std::unordered_map<std::uint64_t, std::shared_ptr<tmap::KeyFrame>> keyframes_;

    Eigen::Affine3f Tsv_, Tbs_, Tbv_;

    double res_, ground_clearance_, max_slope_, robot_height_, security_distance_, min_occupied_fraction_;
    double max_range_base_frame_, min_range_base_frame_;
    unsigned int min_vicinity_points_;
    int delta_ind_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<traversability_msgs::msg::KeyFrameAdditions>::SharedPtr additions_sub_;
    rclcpp::Subscription<traversability_msgs::msg::KeyFrameUpdates>::SharedPtr updates_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_pub_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridmap_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalTraversabilityNode>());
    rclcpp::shutdown();
    return 0;
}
