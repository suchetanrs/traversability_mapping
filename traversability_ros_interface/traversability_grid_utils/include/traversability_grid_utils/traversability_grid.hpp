#pragma once

#include <string>
#include <vector>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traversability_msgs/msg/traversability_sparse_update.hpp>

namespace traversability_mapping
{
class TraversabilityGrid
{
public:
    using Update = traversability_msgs::msg::TraversabilitySparseUpdate;

    // Where the grid contents come from.
    enum class Source
    {
        SparseUpdate,
        FullGridMap
    };

    static constexpr double kDefaultGrowStep = 5.0;  // metres per growth step (sparse mode)
    static constexpr const char *kGridMapTopic = "/global_traversability_gridmap";
    static constexpr const char *kSparseTopic = "/global_traversability_updates";

    explicit TraversabilityGrid(rclcpp::Node *node,
                                Source source = Source::FullGridMap,
                                double grow_step_m = kDefaultGrowStep);

    const grid_map::GridMap *map() const { return initialized_ ? &map_ : nullptr; }

    Source source() const { return source_; }
    bool initialized() const { return initialized_; }
    double resolution() const { return map_.getResolution(); }
    const grid_map::Position &origin() const { return map_.getPosition(); }
    const grid_map::Size &size() const { return map_.getSize(); }
    const grid_map::Length &length() const { return map_.getLength(); }
    const std::string &frameId() const { return map_.getFrameId(); }
    const std::vector<std::string> &layers() const { return map_.getLayers(); }

private:
    // Subscription callbacks.
    void onGridMap(const grid_map_msgs::msg::GridMap::ConstSharedPtr msg);
    void onSparse(const Update::ConstSharedPtr msg) { update(*msg); }

    // Sparse-update path.
    bool update(const Update &msg);
    grid_map::GridMap freshMap(double half_x, double half_y) const;
    void resetFrom(const Update &msg);
    void growToInclude(double minx, double maxx, double miny, double maxy);

    Source source_;
    rclcpp::Logger logger_;
    double grow_step_ = kDefaultGrowStep;

    grid_map::GridMap map_;
    std::vector<std::string> layers_;
    std::string frame_id_;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    double resolution_ = 0.0;
    bool initialized_ = false;

    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr gridmap_sub_;
    rclcpp::Subscription<Update>::SharedPtr sparse_sub_;
};

}  // namespace traversability_mapping
