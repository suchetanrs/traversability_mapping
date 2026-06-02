#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class TraversabilityThresholdNode : public rclcpp::Node
{
public:
    TraversabilityThresholdNode()
        : Node("traversability_threshold_node")
    {
        // Subscription to the input occupancy map
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "global_traversability_map", 10, std::bind(&TraversabilityThresholdNode::callback, this, std::placeholders::_1)
        );

        this->declare_parameter("lethal_obstacle_threshold", rclcpp::ParameterValue(85));
        this->get_parameter("lethal_obstacle_threshold", lethal_threshold_);

        // Publisher for the thresholded occupancy map
        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("traversability_thresholded", rclcpp::QoS(rclcpp::KeepLast(10)).transient_local());
    }

private:
    void callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        // Create a new occupancy grid message to hold the thresholded data
        auto thresholded_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>(*msg);
        thresholded_msg->data.clear();

        // Trinarize the occupancy grid
        for (auto& value : msg->data)
        {
            if (value >= lethal_threshold_)
            {
                thresholded_msg->data.push_back(100);  // Lethal obstacle
            }
            else if (value == -1)
            {
                thresholded_msg->data.push_back(-1);  // Unknown
            }
            else
            {
                thresholded_msg->data.push_back(0);   // Free space
            }
        }

        // Publish the thresholded occupancy grid
        publisher_->publish(*thresholded_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    int lethal_threshold_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TraversabilityThresholdNode>());
    rclcpp::shutdown();
    return 0;
}
