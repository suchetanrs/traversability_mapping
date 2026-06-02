#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class MultiMapMerge : public rclcpp::Node
{
public:
    MultiMapMerge() : Node("multi_map_merge")
    {
        // Subscribe to the first occupancy grid
        subscription1_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/scout_1/traversability_grid", 10,
            std::bind(&MultiMapMerge::occupancyGridCallback1, this, std::placeholders::_1));

        // Subscribe to the second occupancy grid
        subscription2_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/scout_2/traversability_grid", 10,
            std::bind(&MultiMapMerge::occupancyGridCallback2, this, std::placeholders::_1));

        // Publish the merged occupancy grid
        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/traversability_thresholded", rclcpp::QoS(10).transient_local().reliable());
    }

private:
    inline double getMaxPositionX()
    {
        return std::max(occupancyGrid1_->info.width * occupancyGrid1_->info.resolution + occupancyGrid1_->info.origin.position.x, occupancyGrid2_->info.width * occupancyGrid2_->info.resolution + occupancyGrid2_->info.origin.position.x);
    }

    inline double getMaxPositionY()
    {
        return std::max(occupancyGrid1_->info.height * occupancyGrid1_->info.resolution + occupancyGrid1_->info.origin.position.y, occupancyGrid2_->info.height * occupancyGrid2_->info.resolution + occupancyGrid2_->info.origin.position.y);
    }

    bool setMergedMapInfo()
    {
        if (occupancyGrid1_ != nullptr && occupancyGrid2_ != nullptr)
        {
            if (occupancyGrid1_->info.resolution != occupancyGrid2_->info.resolution)
            {
                throw std::runtime_error("Cannot merge maps. Resolution is not same");
            }
            mergedGrid_.info.resolution = occupancyGrid1_->info.resolution;
            mergedGrid_.header.frame_id = "map";
            if (mergedGrid_.info.origin.position.x != std::min(occupancyGrid1_->info.origin.position.x, occupancyGrid2_->info.origin.position.x) || mergedGrid_.info.origin.position.y != std::min(occupancyGrid1_->info.origin.position.y, occupancyGrid2_->info.origin.position.y))
            {
                RCLCPP_WARN_STREAM(this->get_logger(), "Merged Grid pos x old:" << mergedGrid_.info.origin.position.x);
                RCLCPP_WARN_STREAM(this->get_logger(), "Merged Grid pos y old:" << mergedGrid_.info.origin.position.y);
                mergedGrid_.info.origin.position.x = std::min(occupancyGrid1_->info.origin.position.x, occupancyGrid2_->info.origin.position.x);
                mergedGrid_.info.origin.position.y = std::min(occupancyGrid1_->info.origin.position.y, occupancyGrid2_->info.origin.position.y);
                mergedGrid_.data.clear();
                mergedGrid_.info.width = (getMaxPositionX() - mergedGrid_.info.origin.position.x) / mergedGrid_.info.resolution;
                mergedGrid_.info.height = (getMaxPositionY() - mergedGrid_.info.origin.position.y) / mergedGrid_.info.resolution;
                mergedGrid_.data.resize(mergedGrid_.info.width * mergedGrid_.info.height, -1);
                RCLCPP_WARN_STREAM(this->get_logger(), "Merged Grid pos x new:" << mergedGrid_.info.origin.position.x);
                RCLCPP_WARN_STREAM(this->get_logger(), "Merged Grid pos y new:" << mergedGrid_.info.origin.position.y);
            }
        }
        else
            return false;
        // TODO: Add check for greatest position along with origins.
        if (mergedGrid_.info.width == std::max(occupancyGrid1_->info.width, occupancyGrid2_->info.width) && mergedGrid_.info.height == std::max(occupancyGrid1_->info.height, occupancyGrid2_->info.height))
        {
            return true;
        }
        // resize if the origin remains same but height and width changes.
        mergedGrid_.info.width = (getMaxPositionX() - mergedGrid_.info.origin.position.x) / mergedGrid_.info.resolution;
        mergedGrid_.info.height = (getMaxPositionY() - mergedGrid_.info.origin.position.y) / mergedGrid_.info.resolution;
        mergedGrid_.data.resize(mergedGrid_.info.width * mergedGrid_.info.height, -1);
        return true;
    }

    void occupancyGridCallback1(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        RCLCPP_DEBUG_STREAM(this->get_logger(), "OC1 CB**************");
        occupancyGrid1_ = msg;
        // Print origin
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Origin: (" << msg->info.origin.position.x << ", " << msg->info.origin.position.y << ")");
        // Print size (width and height)
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Size: " << msg->info.width << " x " << msg->info.height);
        // Print resolution
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Resolution: " << msg->info.resolution);
        mergeOccupancyGrids(occupancyGrid1_);
        RCLCPP_DEBUG_STREAM(this->get_logger(), "OC1 CB!!!**************");
    }

    void occupancyGridCallback2(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        RCLCPP_DEBUG_STREAM(this->get_logger(), "OC2 CB**************");
        occupancyGrid2_ = msg;
        // Print origin
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Origin: (" << msg->info.origin.position.x << ", " << msg->info.origin.position.y << ")");
        // Print size (width and height)
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Size: " << msg->info.width << " x " << msg->info.height);
        // Print resolution
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Resolution: " << msg->info.resolution);
        mergeOccupancyGrids(occupancyGrid2_);
        RCLCPP_DEBUG_STREAM(this->get_logger(), "OC2 CB!!!**************");
    }

    void mergeOccupancyGrids(nav_msgs::msg::OccupancyGrid::SharedPtr interestMap)
    {
        RCLCPP_DEBUG(this->get_logger(), "===================================");
        RCLCPP_DEBUG(this->get_logger(), "Merge CB");
        auto infoSetFlag = setMergedMapInfo();
        if (infoSetFlag)
        {
            mergedGrid_.header.stamp = interestMap->header.stamp;
            // Print origin
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Origin: (" << mergedGrid_.info.origin.position.x << ", " << mergedGrid_.info.origin.position.y << ")");
            // Print size (width and height)
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Size w: " << mergedGrid_.info.width << " Size h:  " << mergedGrid_.info.height);
            RCLCPP_DEBUG_STREAM(this->get_logger(), "DataSize: " << mergedGrid_.data.size());
            // Print resolution
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Resolution: " << mergedGrid_.info.resolution);
            int offsetY_ = 0;
            int offsetX_ = 0;
            offsetX_ = (interestMap->info.origin.position.x - mergedGrid_.info.origin.position.x) / interestMap->info.resolution;
            offsetY_ = (interestMap->info.origin.position.y - mergedGrid_.info.origin.position.y) / interestMap->info.resolution;
            RCLCPP_DEBUG_STREAM(this->get_logger(), "OffsetX: " << offsetX_ << " OffsetY: " << offsetY_);

            for (int x = 0; x < interestMap->info.width; ++x)
            {
                for (int y = 0; y < interestMap->info.height; ++y)
                {
                    int idxMerged = (x + offsetX_) + (y + offsetY_) * mergedGrid_.info.width;
                    int idxMap = x + y * interestMap->info.width;
                    if (idxMap >= interestMap->data.size())
                        throw std::runtime_error("IDx more than data size interest map, this should not happen.");
                    else if (idxMerged >= mergedGrid_.data.size())
                        throw std::runtime_error("IDx more than data size merged map, this should not happen.");
                    if (static_cast<int>(interestMap->data[idxMap]) != -1)
                    {
                        auto addingData = static_cast<int>(interestMap->data[idxMap]) > 60 ? static_cast<int8_t>(100) : static_cast<int8_t>(0);
                        // mergedGrid_.data[idxMerged] = std::max(mergedGrid_.data[idxMerged], addingData);
                        mergedGrid_.data[idxMerged] = addingData;
                    }
                }
            }

            // Publish the merged occupancy grid
            publisher_->publish(mergedGrid_);
        }
        else
            RCLCPP_WARN(this->get_logger(), "Waiting for set info to be true");
        RCLCPP_DEBUG(this->get_logger(), "!!!===================================");
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr occupancyGrid1_;
    nav_msgs::msg::OccupancyGrid::SharedPtr occupancyGrid2_;
    nav_msgs::msg::OccupancyGrid mergedGrid_;
    double maxWorldPosX_;
    double maxWorldPosY_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription1_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription2_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiMapMerge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
