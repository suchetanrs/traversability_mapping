#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <traversability_msgs/msg/traversability_sparse_update.hpp>

#include "traversability_grid_utils/grid_introspector.hpp"
#include "traversability_grid_utils/grid_reconstructor.hpp"

namespace traversability_mapping
{

class TraversabilityGridClient
{
public:
    using Update = traversability_msgs::msg::TraversabilitySparseUpdate;
    using UpdateCallback = std::function<void(const grid_map::GridMap &)>;

    class ReadAccess
    {
    public:
        ReadAccess(std::shared_mutex &m, const grid_map::GridMap &map)
            : lock_(m), view_(map) {}

        const GridIntrospector *operator->() const { return &view_; }
        const GridIntrospector &operator*() const { return view_; }
        const GridIntrospector &view() const { return view_; }
        const grid_map::GridMap &map() const { return view_.map(); }

    private:
        std::shared_lock<std::shared_mutex> lock_;
        GridIntrospector view_;
    };

    TraversabilityGridClient(rclcpp::Node *node,
                             const std::string &topic,
                             const rclcpp::QoS &qos = rclcpp::QoS(rclcpp::KeepLast(10)),
                             double grow_step_m = 5.0);

    ReadAccess read() const { return ReadAccess(mutex_, recon_.map()); }

    bool initialized() const;

    void setUpdateCallback(UpdateCallback cb);

private:
    void onUpdate(const Update::ConstSharedPtr msg);

    rclcpp::Subscription<Update>::SharedPtr sub_;
    rclcpp::Logger logger_;
    mutable std::shared_mutex mutex_;
    GridReconstructor recon_;
    UpdateCallback cb_;
};

}  // namespace traversability_mapping
