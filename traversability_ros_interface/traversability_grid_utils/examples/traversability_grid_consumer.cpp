#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "traversability_grid_utils/traversability_grid.hpp"

class GridConsumer : public rclcpp::Node
{
public:
    GridConsumer() : Node("traversability_grid_consumer")
    {
        const auto source = traversability_mapping::TraversabilityGrid::Source::FullGridMap;
        // const auto source = traversability_mapping::TraversabilityGrid::Source::SparseUpdate;

        grid_ = std::make_unique<traversability_mapping::TraversabilityGrid>(this, source);

        timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { report(); });

        RCLCPP_INFO(get_logger(), "Consumer up");
    }

private:

    void report()
    {
        // initialized()
        RCLCPP_INFO(get_logger(), "initialized=%s",
                    grid_->initialized() ? "true" : "false");

        if (!grid_->initialized())
        {
            RCLCPP_INFO(get_logger(), "  (no grid received yet)");
            return;
        }

        // --- metadata getters ---------------------------------------------
        const grid_map::Position &o = grid_->origin();
        const grid_map::Length &len = grid_->length();
        const grid_map::Size &sz = grid_->size();

        RCLCPP_INFO(get_logger(), "  frame_id   : %s", grid_->frameId().c_str());
        RCLCPP_INFO(get_logger(), "  resolution : %.3f m", grid_->resolution());
        RCLCPP_INFO(get_logger(), "  origin     : (%.2f, %.2f)", o.x(), o.y());
        RCLCPP_INFO(get_logger(), "  length     : %.2f x %.2f m", len.x(), len.y());
        RCLCPP_INFO(get_logger(), "  size       : %d x %d cells", sz(0), sz(1));

        std::string layer_list;
        for (const auto &l : grid_->layers())
            layer_list += (layer_list.empty() ? "" : ", ") + l;
        RCLCPP_INFO(get_logger(), "  layers     : [%s]", layer_list.c_str());

        const grid_map::GridMap *m = grid_->map();
        if (m == nullptr)
            return;
    }

    std::unique_ptr<traversability_mapping::TraversabilityGrid> grid_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GridConsumer>());
    rclcpp::shutdown();
    return 0;
}
