#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <fstream>

class LineMarkerPublisher : public rclcpp::Node
{
public:
    LineMarkerPublisher() : Node("line_marker_publisher")
    {
        // Open CSV file in append mode.
        csv_file_.open("points.csv", std::ios::out | std::ios::app);
        if (!csv_file_.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file for writing.");
        }
        else
        {
            // Optionally, write header (if file is new)
            csv_file_ << "x1,y1,z1,x2,y2,z2\n";
        }

        // Create subscriber for clicked points.
        point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "clicked_point", 10,
            std::bind(&LineMarkerPublisher::pointCallback, this, std::placeholders::_1));

        // Create publisher for the visualization marker.
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

        // Set up the marker.
        marker_.header.frame_id = "map"; // Change as needed.
        marker_.ns = "line_marker";
        marker_.id = 0;
        marker_.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker_.action = visualization_msgs::msg::Marker::ADD;
        // Set line width.
        marker_.scale.x = 0.05;
        // Set color (red in this example).
        marker_.color.r = 1.0;
        marker_.color.g = 0.0;
        marker_.color.b = 0.0;
        marker_.color.a = 1.0;
        // Keep marker indefinitely.
        marker_.lifetime = rclcpp::Duration(0, 0);

        RCLCPP_INFO(this->get_logger(), "LineMarkerPublisher node initialized.");
    }

    ~LineMarkerPublisher()
    {
        if (csv_file_.is_open())
        {
            csv_file_.close();
        }
    }

private:
    void pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received clicked point: (%.2f, %.2f, %.2f)",
                    msg->point.x, msg->point.y, msg->point.z);

        // If no previous point, store the current point.
        if (!has_previous_point_)
        {
            previous_point_ = msg->point;
            has_previous_point_ = true;
            RCLCPP_INFO(this->get_logger(), "Stored first point. Waiting for the second point to form a pair.");
        }
        else
        {
            // We already have a previous point; now form a pair.
            geometry_msgs::msg::Point current_point = msg->point;

            // Write the pair of points to CSV (format: x1,y1,z1,x2,y2,z2).
            if (csv_file_.is_open())
            {
                csv_file_ << previous_point_.x << "," << previous_point_.y << "," << previous_point_.z << ","
                          << current_point.x << "," << current_point.y << "," << current_point.z << "\n";
                csv_file_.flush();
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "CSV file not open, cannot write points.");
            }

            // Add the two points to the marker.
            marker_.points.push_back(previous_point_);
            marker_.points.push_back(current_point);

            // Update the marker header time and publish the marker.
            marker_.header.stamp = this->now();
            marker_pub_->publish(marker_);
            RCLCPP_INFO(this->get_logger(), "Published a line marker between the two points.");

            // Reset for the next pair.
            has_previous_point_ = false;
        }
    }

    // Subscriber and publisher.
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    // Marker to display lines between pairs.
    visualization_msgs::msg::Marker marker_;

    // CSV file stream.
    std::ofstream csv_file_;

    // To store the previous point.
    geometry_msgs::msg::Point previous_point_;
    bool has_previous_point_{false};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LineMarkerPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}