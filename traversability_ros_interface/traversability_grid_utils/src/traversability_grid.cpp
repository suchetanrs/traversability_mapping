#include "traversability_grid_utils/traversability_grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include <grid_map_ros/GridMapRosConverter.hpp>

namespace traversability_mapping
{

TraversabilityGrid::TraversabilityGrid(rclcpp::Node *node, Source source, double grow_step_m)
    : source_(source),
      logger_(node->get_logger().get_child("traversability_grid")),
      grow_step_(grow_step_m)
{
    if (source_ == Source::FullGridMap)
    {
        gridmap_sub_ = node->create_subscription<grid_map_msgs::msg::GridMap>(
            kGridMapTopic, rclcpp::QoS(1).transient_local(),
            [this](const grid_map_msgs::msg::GridMap::ConstSharedPtr msg) { onGridMap(msg); });
        RCLCPP_INFO(logger_, "Consuming full grid_map on '%s'.", kGridMapTopic);
    }
    else
    {
        sparse_sub_ = node->create_subscription<Update>(
            kSparseTopic, rclcpp::QoS(10).reliable(),
            [this](const Update::ConstSharedPtr msg) { onSparse(msg); });
        RCLCPP_INFO(logger_, "Consuming sparse traversability updates on '%s'.", kSparseTopic);
    }
}

void TraversabilityGrid::onGridMap(const grid_map_msgs::msg::GridMap::ConstSharedPtr msg)
{
    if (!grid_map::GridMapRosConverter::fromMessage(*msg, map_))
    {
        RCLCPP_WARN(logger_, "Failed to convert incoming grid_map message.");
        return;
    }
    initialized_ = true;
}

namespace
{
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

// Absolute lattice key packing, byte-compatible with the sender (see
// TraversabilitySparseUpdate.msg): hi 32 bits = ci, lo 32 bits = cj.
inline void unkey(std::uint64_t k, int &ci, int &cj)
{
    ci = static_cast<int>(static_cast<std::uint32_t>(k >> 32));
    cj = static_cast<int>(static_cast<std::uint32_t>(k & 0xffffffffu));
}
}  // namespace

grid_map::GridMap TraversabilityGrid::freshMap(double half_x, double half_y) const
{
    const int kx = std::max(0, static_cast<int>(std::ceil(half_x / resolution_)));
    const int ky = std::max(0, static_cast<int>(std::ceil(half_y / resolution_)));
    grid_map::GridMap m(layers_);
    m.setFrameId(frame_id_);
    m.setGeometry(grid_map::Length((2 * kx + 1) * resolution_, (2 * ky + 1) * resolution_),
                  resolution_, grid_map::Position(origin_x_, origin_y_));
    for (const auto &l : layers_)
        m[l].setConstant(kNaN);
    return m;
}

void TraversabilityGrid::resetFrom(const Update &msg)
{
    origin_x_ = msg.origin_x;
    origin_y_ = msg.origin_y;
    resolution_ = msg.resolution;
    layers_.assign(msg.layers.begin(), msg.layers.end());
    frame_id_ = msg.header.frame_id;
    map_ = freshMap(grow_step_, grow_step_);
    initialized_ = true;
}

void TraversabilityGrid::growToInclude(double minx, double maxx, double miny, double maxy)
{
    const double margin = resolution_;
    const double cur_half_x = map_.getLength().x() / 2.0;
    const double cur_half_y = map_.getLength().y() / 2.0;
    const double need_x = std::max(std::abs(maxx - origin_x_), std::abs(minx - origin_x_)) + margin;
    const double need_y = std::max(std::abs(maxy - origin_y_), std::abs(miny - origin_y_)) + margin;
    if (need_x <= cur_half_x && need_y <= cur_half_y)
        return;

    double new_half_x = cur_half_x;
    double new_half_y = cur_half_y;
    while (new_half_x < need_x) new_half_x += grow_step_;
    while (new_half_y < need_y) new_half_y += grow_step_;

    grid_map::GridMap old = map_;
    map_ = freshMap(new_half_x, new_half_y);
    for (grid_map::GridMapIterator it(old); !it.isPastEnd(); ++it)
    {
        grid_map::Position p;
        old.getPosition(*it, p);
        if (!map_.isInside(p))
            continue;
        for (const auto &l : layers_)
        {
            const float v = old.at(l, *it);
            if (!std::isnan(v))
                map_.atPosition(l, p) = v;
        }
    }
}

bool TraversabilityGrid::update(const Update &msg)
{
    const std::size_t n_layers = msg.layers.size();
    if (n_layers == 0 || msg.resolution <= 0.0)
        return false;
    if (msg.values.size() != msg.cell_keys.size() * n_layers)
        return false;

    const bool geom_changed =
        !initialized_ ||
        msg.is_full_snapshot ||
        msg.resolution != resolution_ ||
        msg.origin_x != origin_x_ ||
        msg.origin_y != origin_y_ ||
        !std::equal(layers_.begin(), layers_.end(), msg.layers.begin(), msg.layers.end());
    if (geom_changed)
        resetFrom(msg);

    if (msg.cell_keys.empty())
        return true;

    // World AABB of incoming cells, so we grow once up front.
    double minx = std::numeric_limits<double>::infinity();
    double maxx = -std::numeric_limits<double>::infinity();
    double miny = std::numeric_limits<double>::infinity();
    double maxy = -std::numeric_limits<double>::infinity();
    for (const std::uint64_t k : msg.cell_keys)
    {
        int ci, cj;
        unkey(k, ci, cj);
        const double x = origin_x_ + ci * resolution_;
        const double y = origin_y_ + cj * resolution_;
        minx = std::min(minx, x); maxx = std::max(maxx, x);
        miny = std::min(miny, y); maxy = std::max(maxy, y);
    }
    growToInclude(minx, maxx, miny, maxy);

    // Stitch each cell's per-layer values onto the grid (NaN clears a cell+layer).
    for (std::size_t i = 0; i < msg.cell_keys.size(); ++i)
    {
        int ci, cj;
        unkey(msg.cell_keys[i], ci, cj);
        const grid_map::Position p(origin_x_ + ci * resolution_, origin_y_ + cj * resolution_);
        if (!map_.isInside(p))
            continue;
        const std::size_t base = i * n_layers;
        for (std::size_t l = 0; l < n_layers; ++l)
            map_.atPosition(layers_[l], p) = msg.values[base + l];
    }
    return true;
}

}  // namespace traversability_mapping
