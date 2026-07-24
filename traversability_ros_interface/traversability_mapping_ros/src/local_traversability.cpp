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
 *   KeyFrameAdditions -> filter + transform cloud to the robot BASE frame ->
 *   store the pruned base-frame cloud in the keyframe -> the keyframe bins it
 *   into cell-local moments on the absolute lattice (rebin) -> ADD into the grid
 *   moment layers -> recompute hazards over the dilated dirty set. A timer
 *   publishes two subscription-gated outputs: (1) a SPARSE update to nav (only
 *   the cells touched since the last tick, just the nav_layers_, keyed by
 *   absolute lattice id; first publish / any (re)connect sends a full snapshot),
 *   and (2) the full nav grid_map for visualization.
 *
 *   KeyFrameUpdates (PGO) carry a corrected pose only: the node subtracts the
 *   keyframe's previous contribution, re-transforms its stored base-frame cloud
 *   by the new pose and RE-BINS from scratch (the partition is NOT frozen; cell
 *   membership is recomputed every update, however small), then re-adds. Hazards
 *   are recomputed over the dilated union of the cells left and newly occupied.
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
#include <traversability_msgs/msg/traversability_sparse_update.hpp>

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
#include <utility>
#include <chrono>
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
                   "hazard", "elevation", "slope_haz", "step_haz", "roughness_haz",
                   "normal_x", "normal_y", "normal_z"};
        // Subset of layers_ published to navigation. This is the ONLY thing nav
        // receives; edit this list (entries must exist in layers_) to add/remove
        // a nav layer without touching the internal map.
        nav_layers_ = {"normal_x", "normal_y", "normal_z", "slope_haz",
                       "step_haz", "elevation", "roughness_haz", "hazard"};
        const double half = parameterInstance.getValue<double>("half_size_traversability");
        gridMap_ = freshMap(half, half);

        // --- ROS I/O ---
        additions_sub_ = create_subscription<traversability_msgs::msg::KeyFrameAdditions>(
            additions_topic_, 10,
            std::bind(&GlobalTraversabilityNode::additionsCallback, this, std::placeholders::_1));
        updates_sub_ = create_subscription<traversability_msgs::msg::KeyFrameUpdates>(
            updates_topic_, 10,
            std::bind(&GlobalTraversabilityNode::updatesCallback, this, std::placeholders::_1));

        // Full nav grid_map, for visualization only (RViz grid_map plugin).
        gridmap_pub_ = create_publisher<grid_map_msgs::msg::GridMap>(
            "global_traversability_gridmap", rclcpp::QoS(1).transient_local());

        // Flattened occupancy view of the hazard layer, for costmap/RViz.
        occupancy_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
            "global_traversability_occupancy", rclcpp::QoS(1).transient_local());

        // Sparse updates for navigation. Reliable, ordered, volatile: deltas must
        // arrive in order and must not be replayed stale (transient_local would
        // resend only the last delta). Late joiners are served a full snapshot on
        // (re)connect instead.
        sparse_pub_ = create_publisher<traversability_msgs::msg::TraversabilitySparseUpdate>(
            "global_traversability_updates", rclcpp::QoS(10).reliable());

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
        // Mark it dirty so nav is told the cell is now cleared (NaN).
        if (sign < 0 && std::lround(gridMap_.atPosition("N", p)) <= 0)
        {
            blankCell(p);
            dirty_for_nav_.insert(cellId);
        }
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

    // Recompute one query cell's derived (nav) layers from the FUSED moments in
    // its vicinity. Reads only moment layers (never written here) + neighbour
    // cells; writes only THIS cell's derived layers. Because distinct cells write
    // disjoint elements and the moment layers are read-only during recompute,
    // this is safe to run concurrently across cells. Returns true iff the cell's
    // nav layers were (re)written (values or NaN) -> caller flags it for nav.
    bool recomputeCell(std::uint64_t id)
    {
        int ci, cj;
        tmap::Lattice::unkey(id, ci, cj);
        const grid_map::Position qp = cellPos(ci, cj);
        if (!gridMap_.isInside(qp))
            return false;

        tmap::NodeMetaData qd;
        if (!readCellMoment(ci, cj, qd))
            return false;  // query cell unobserved -> nothing to write

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
            gridMap_.atPosition("normal_x", qp) = std::numeric_limits<float>::quiet_NaN();
            gridMap_.atPosition("normal_y", qp) = std::numeric_limits<float>::quiet_NaN();
            gridMap_.atPosition("normal_z", qp) = std::numeric_limits<float>::quiet_NaN();
            return true;
        }
        gridMap_.atPosition("hazard", qp) = static_cast<float>(haz[tmap::HAZ_OVERALL]);
        gridMap_.atPosition("slope_haz", qp) = static_cast<float>(haz[tmap::HAZ_SLOPE]);
        gridMap_.atPosition("step_haz", qp) = static_cast<float>(haz[tmap::HAZ_STEP]);
        gridMap_.atPosition("roughness_haz", qp) = static_cast<float>(haz[tmap::HAZ_ROUGHNESS]);
        gridMap_.atPosition("normal_x", qp) = static_cast<float>(haz[tmap::HAZ_NORMAL_X]);
        gridMap_.atPosition("normal_y", qp) = static_cast<float>(haz[tmap::HAZ_NORMAL_Y]);
        gridMap_.atPosition("normal_z", qp) = static_cast<float>(haz[tmap::HAZ_NORMAL_Z]);
        return true;
    }

    // Recompute the dirty set in parallel (one cell per task; disjoint writes).
    // dirty_for_nav_ is not thread-safe, so flags are gathered per cell and
    // merged serially afterwards.
    void recomputeDirty(const std::unordered_set<std::uint64_t> &dirty)
    {
        const std::vector<std::uint64_t> cells(dirty.begin(), dirty.end());
        const std::size_t n = cells.size();
        std::vector<char> wrote(n, 0);

        #pragma omp parallel for schedule(dynamic, 64)
        for (std::size_t k = 0; k < n; ++k)
            wrote[k] = recomputeCell(cells[k]) ? 1 : 0;

        for (std::size_t k = 0; k < n; ++k)
            if (wrote[k])
                dirty_for_nav_.insert(cells[k]);
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
            processKeyframe(kf, /*isUpdate=*/false);
    }

    void updatesCallback(const traversability_msgs::msg::KeyFrameUpdates::SharedPtr msg)
    {
        // PGO pose corrections. Pose-only: re-bin the stored cloud at the new
        // pose. (Exercised once a real SLAM provides updates; GT emits none.)
        RCLCPP_WARN(get_logger(), "[UPDATE] batch received: %zu keyframe(s) to re-bin.",
                    msg->keyframes.size());
        for (size_t i = 0; i < msg->keyframes.size(); ++i)
        {
            const auto &kf = msg->keyframes[i];
            RCLCPP_WARN(get_logger(), "[UPDATE] (%zu/%zu) processing keyframe id %lu.",
                        i + 1, msg->keyframes.size(), (unsigned long)kf.kf_id);
            processKeyframe(kf, /*isUpdate=*/true);
            // Run the full publishing routine after EACH keyframe so RViz shows the
            // map shift/correct in real time (full grid_map + occupancy + sparse),
            // instead of waiting for the 1 Hz timer.
            publish();
        }
    }

    // Filter the lidar cloud (ego return, ceiling/overhang, range) and express
    // the survivors in the robot BASE frame. The base frame is rigidly attached
    // to the robot, so this pruned cloud stays valid across any pose correction
    // and is what the keyframe re-bins from. All three gates are pose-invariant.
    std::vector<Eigen::Vector3f> pruneToBase(const sensor_msgs::msg::PointCloud2 &ros_cloud) const
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_lidar;
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(ros_cloud, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, cloud_lidar);

        std::vector<Eigen::Vector3f> cloud_base;
        cloud_base.reserve(cloud_lidar.size());
        for (const auto &p : cloud_lidar)
        {
            if (p.x == 0.f && p.y == 0.f)
                continue;  // ego / invalid return (lidar frame)
            const Eigen::Vector3f p_base = Tbv_ * Eigen::Vector3f(p.x, p.y, p.z);
            if (p_base.z() > static_cast<float>(robot_height_))
                continue;  // ceiling / overhang
            const float range = p_base.norm();
            if (range > static_cast<float>(max_range_base_frame_) ||
                range < static_cast<float>(min_range_base_frame_))
                continue;  // out of range
            cloud_base.push_back(p_base);
        }
        return cloud_base;
    }

    // Grow the grid (if needed) so every cell the keyframe contributes to fits.
    // AABB is taken from the partials' cell centres; growToInclude's one-cell
    // margin covers the <=1/2-res gap between a cell centre and its points.
    void growToIncludeCells(const std::unordered_map<std::uint64_t, tmap::NodeMetaData> &partials)
    {
        if (partials.empty())
            return;
        double minx = 1e18, maxx = -1e18, miny = 1e18, maxy = -1e18;
        for (const auto &kv : partials)
        {
            int ci, cj;
            tmap::Lattice::unkey(kv.first, ci, cj);
            const Eigen::Vector2d c = lattice_.centerOf(ci, cj);
            minx = std::min(minx, c.x()); maxx = std::max(maxx, c.x());
            miny = std::min(miny, c.y()); maxy = std::max(maxy, c.y());
        }
        growToInclude(minx, maxx, miny, maxy);
    }

    // Shared addition/update path. Additions create a keyframe from the pruned
    // base-frame cloud; updates back out the old contribution first. Both then
    // re-bin from the stored cloud at the current pose and re-add. Hazards are
    // recomputed over the dilated union of cells left and newly occupied.
    void processKeyframe(const traversability_msgs::msg::KeyFrame &kf_msg, bool isUpdate)
    {
        const auto t_start = std::chrono::steady_clock::now();
        const char *tag = isUpdate ? "UPDATE" : "ADD";
        const Eigen::Affine3f Tmb = poseToAffine(kf_msg.kf_pose);  // map <- base

        std::shared_ptr<tmap::KeyFrame> kf;
        std::unordered_set<std::uint64_t> touched;  // M_old (subtracted) U M_new (added)
        size_t n_subtracted = 0;

        if (isUpdate)
        {
            auto it = keyframes_.find(kf_msg.kf_id);
            if (it == keyframes_.end())
            {
                RCLCPP_WARN(get_logger(), "[%s] keyframe id %lu unknown; ignored.",
                            tag, (unsigned long)kf_msg.kf_id);
                return;
            }
            kf = it->second;
            // Back out the OLD contribution. Must read partials_ BEFORE rebin
            // clears it. Old cells are always already inside the map, so this is
            // safe before any grow.
            for (const auto &kv : kf->partials())
            {
                addPartialToGrid(kv.first, kv.second, -1.0);
                touched.insert(kv.first);
            }
            n_subtracted = kf->partials().size();
            kf->setPose(Tmb);
        }
        else
        {
            if (keyframes_.count(kf_msg.kf_id))
            {
                RCLCPP_WARN(get_logger(), "[%s] keyframe id %lu duplicate; ignored.",
                            tag, (unsigned long)kf_msg.kf_id);
                return;
            }
            std::vector<Eigen::Vector3f> cloud_base = pruneToBase(kf_msg.kf_pointcloud);
            if (cloud_base.empty())
            {
                RCLCPP_WARN(get_logger(), "[%s] keyframe id %lu pruned to empty; not stored.",
                            tag, (unsigned long)kf_msg.kf_id);
                return;
            }
            kf = std::make_shared<tmap::KeyFrame>(kf_msg.kf_id, Tmb, std::move(cloud_base));
            keyframes_[kf_msg.kf_id] = kf;
        }

        // (Re)bin the stored base-frame cloud at the current pose, grow to fit,
        // then add the fresh contribution.
        kf->rebin(lattice_);
        growToIncludeCells(kf->partials());
        for (const auto &kv : kf->partials())
        {
            addPartialToGrid(kv.first, kv.second, +1.0);
            touched.insert(kv.first);
        }
        const size_t n_added = kf->partials().size();

        recomputeDirty(dilate(touched));

        const double ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - t_start).count();
        RCLCPP_WARN(get_logger(),
                    "[%s] keyframe id %lu done in %.2f ms (subtracted %zu, added %zu, "
                    "touched %zu cells). Total keyframes: %zu.",
                    tag, (unsigned long)kf_msg.kf_id, ms, n_subtracted, n_added,
                    touched.size(), keyframes_.size());
    }

    // ---- publish ------------------------------------------------------------

    // Serialize the given cells' nav layers into a sparse-update message.
    // Cells whose nav layers are all NaN are still emitted (they tell nav to
    // clear that cell). keys must be valid absolute lattice ids inside the map.
    void fillMessage(traversability_msgs::msg::TraversabilitySparseUpdate &msg,
                     const std::vector<std::uint64_t> &keys, bool full) const
    {
        msg.header.frame_id = map_frame_;
        msg.resolution = res_;
        msg.origin_x = lattice_.x0;
        msg.origin_y = lattice_.y0;
        msg.layers = nav_layers_;
        msg.is_full_snapshot = full;
        msg.cell_keys.reserve(keys.size());
        msg.values.reserve(keys.size() * nav_layers_.size());
        for (auto id : keys)
        {
            int ci, cj;
            tmap::Lattice::unkey(id, ci, cj);
            const grid_map::Position p = cellPos(ci, cj);
            if (!gridMap_.isInside(p))
                continue;
            msg.cell_keys.push_back(id);
            for (const auto &l : nav_layers_)
                msg.values.push_back(gridMap_.atPosition(l, p));
        }
    }

    // Every occupied cell currently in the map (for a full snapshot).
    std::vector<std::uint64_t> allOccupiedKeys() const
    {
        std::vector<std::uint64_t> keys;
        for (grid_map::GridMapIterator it(gridMap_); !it.isPastEnd(); ++it)
        {
            const float n = gridMap_.at("N", *it);
            if (std::isnan(n) || n < 1.f)
                continue;
            grid_map::Position p;
            gridMap_.getPosition(*it, p);
            int ci, cj;
            lattice_.cellOf(p.x(), p.y(), ci, cj);
            keys.push_back(tmap::Lattice::key(ci, cj));
        }
        return keys;
    }

    void publish()
    {
        publishFullMap();
        publishOccupancyMap();
        publishSparse();
    }

    // Full nav grid_map for visualization. Independent of the sparse path.
    void publishFullMap()
    {
        if (gridmap_pub_->get_subscription_count() == 0)
            return;
        auto gm = grid_map::GridMapRosConverter::toMessage(gridMap_, nav_layers_);
        gm->header.frame_id = map_frame_;
        gm->header.stamp = now();
        gridmap_pub_->publish(*gm);
    }

    // Flattened 2D occupancy view of the "hazard" layer, for consumers that want
    // a plain nav_msgs/OccupancyGrid (e.g. costmap/RViz). hazard is in [0,1] ->
    // [0,100]; unobserved cells (NaN) map to -1 (unknown). Subscriber-gated.
    void publishOccupancyMap()
    {
        if (occupancy_pub_->get_subscription_count() == 0)
            return;
        nav_msgs::msg::OccupancyGrid og;
        grid_map::GridMapRosConverter::toOccupancyGrid(gridMap_, "hazard", 0.0, 1.0, og);
        og.header.frame_id = map_frame_;
        og.header.stamp = now();
        occupancy_pub_->publish(og);
    }

    void publishSparse()
    {
        // No consumer: drop accumulated deltas and arm a full snapshot so the
        // next subscriber starts from a consistent state.
        if (sparse_pub_->get_subscription_count() == 0)
        {
            dirty_for_nav_.clear();
            need_full_snapshot_ = true;
            return;
        }

        traversability_msgs::msg::TraversabilitySparseUpdate msg;
        if (need_full_snapshot_)
        {
            fillMessage(msg, allOccupiedKeys(), /*full=*/true);
            need_full_snapshot_ = false;
            dirty_for_nav_.clear();
        }
        else
        {
            if (dirty_for_nav_.empty())
                return;
            const std::vector<std::uint64_t> keys(dirty_for_nav_.begin(), dirty_for_nav_.end());
            fillMessage(msg, keys, /*full=*/false);
            dirty_for_nav_.clear();
        }
        msg.header.stamp = now();
        sparse_pub_->publish(msg);
    }

    // ---- members ------------------------------------------------------------
    std::string additions_topic_, updates_topic_;
    std::string slam_frame_, robot_base_frame_, lidar_frame_, map_frame_;

    tmap::Lattice lattice_;
    std::vector<std::string> layers_;       // all internal layers (moments + hazards)
    std::vector<std::string> nav_layers_;   // subset published to navigation
    grid_map::GridMap gridMap_;
    std::unordered_map<std::uint64_t, std::shared_ptr<tmap::KeyFrame>> keyframes_;

    // Cells whose nav layers changed since the last publish (sparse update set).
    std::unordered_set<std::uint64_t> dirty_for_nav_;
    bool need_full_snapshot_ = true;        // send a full snapshot on first publish

    Eigen::Affine3f Tsv_, Tbs_, Tbv_;

    double res_, ground_clearance_, max_slope_, robot_height_, security_distance_, min_occupied_fraction_;
    double max_range_base_frame_, min_range_base_frame_;
    unsigned int min_vicinity_points_;
    int delta_ind_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<traversability_msgs::msg::KeyFrameAdditions>::SharedPtr additions_sub_;
    rclcpp::Subscription<traversability_msgs::msg::KeyFrameUpdates>::SharedPtr updates_sub_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridmap_pub_;          // full map, viz only
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_pub_;       // flattened hazard, viz/costmap
    rclcpp::Publisher<traversability_msgs::msg::TraversabilitySparseUpdate>::SharedPtr sparse_pub_;  // sparse updates, nav
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalTraversabilityNode>());
    rclcpp::shutdown();
    return 0;
}
