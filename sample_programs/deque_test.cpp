#include <iostream>
#include <deque>
#include <algorithm>
#include <memory>

class PointCloudBufferNode
{
public:
    PointCloudBufferNode()
    {
        // Initialize the buffer
        buffer_.clear();
    }

    void pointCloudCallback(double timestamp, int val)
    {
        // Store incoming point cloud in the buffer
        buffer_.push_back(std::make_pair(timestamp, val));
    }

    // Function to find the closest point cloud to the queried timestamp
    int getClosestPointCloud(double time)
    {
        auto closest_cloud_iter = std::min_element(buffer_.begin(), buffer_.end(),
                                                   [&time](const auto &a, const auto &b)
                                                   {
                                                       return std::abs(a.first - time) < std::abs(b.first - time);
                                                   });

        return closest_cloud_iter->second;
    }

    // Function to delete all points before the queried timestamp in the buffer
    void deletePointsBefore(double time)
    {
        buffer_.erase(std::remove_if(buffer_.begin(), buffer_.end(),
                                     [&time](const auto &entry)
                                     {
                                         return entry.first < time;
                                     }),
                      buffer_.end());
    }

    void printSize()
    {
        std::cout << "size is " << buffer_.size() << std::endl;
    }

    void printQueue()
    {
        std::cout << "Queue is: " << std::endl;
        for (auto value : buffer_)
        {
            std::cout << value.second << std::endl;
        }
    }

private:
    std::deque<std::pair<double, int>> buffer_;
};

int main(int argc, char **argv)
{
    auto node = std::make_shared<PointCloudBufferNode>();
    node->pointCloudCallback(0.1, 0);
    node->pointCloudCallback(1.0, 1);
    node->pointCloudCallback(2.0, 2);
    node->pointCloudCallback(3.0, 3);
    node->pointCloudCallback(4.0, 4);
    node->pointCloudCallback(5.0, 5);
    node->pointCloudCallback(6.0, 6);
    std::cout << node->getClosestPointCloud(-1.1) << std::endl;
    node->printSize();
    node->deletePointsBefore(0.0);
    node->printSize();
    node->printQueue();
    return 0;
}