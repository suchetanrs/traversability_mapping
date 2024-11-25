#ifndef SYSTEM_HPP_
#define SYSTEM_HPP_

#include "traversability_mapping/LocalMap.hpp"
#include "traversability_mapping/PointCloudBuffer.hpp"
#include "traversability_mapping/KeyFrame.hpp"
#include "traversability_mapping/Helpers.hpp"
#include "traversability_mapping_common/type_conversion.hpp"
#include "traversability_mapping/PointCloudBufferROS.hpp"
#include <iostream>
namespace traversability_mapping
{
    class System
    {
    public:
        System();

        void addNewKeyFrameTsDouble(const double timestamp,
                                    long unsigned int kfID,
                                    long unsigned int mapID);

        void addNewKeyFrameTsULong(const unsigned long long timestamp,
                                   long unsigned int kfID,
                                   long unsigned int mapID);

        // Use this function if you dont want the buffer. Push own custom pointcloud msg for the KF.
        void addNewKeyFrameWithPCL(const unsigned long long timestamp,
                                   int kfID,
                                   long unsigned int mapID,
                                   std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensorPointCloud);

        void informLoopClosure();

        void updateKeyFrame(long unsigned int kfID,
                            Sophus::SE3f &poseSLAM,
                            long unsigned int numConnections);

        void updateKeyFrame(unsigned long long kfID,
                            Eigen::Affine3d &poseAffine,
                            long unsigned int numConnections);

        void deleteKeyFrame(unsigned long long kfID);

        void addNewLocalMap(long unsigned int mapID);

        void pushToBuffer(double timestamp,
                          std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &pcl2);
        
        void pushToBuffer(double timestamp,
                          pcl::PCLPointCloud2& pcl_pc2);

#ifdef WITH_ROS2_SENSOR_MSGS
        void pushToBuffer(sensor_msgs::msg::PointCloud2::SharedPtr pcl2);
#endif

        std::shared_ptr<LocalMap> getLocalMap();

        void updateKFMap(long unsigned int kfID,
                         long unsigned int mapID);

    private:
        void addNewKeyFrameToMap(const double timestamp,
                                 long unsigned int kfID,
                                 long unsigned int mapID,
                                 std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensorPointCloud);

        void setCurrentMap(long unsigned int mapID);

        void makeAllMapsInactive();

        void setLocalMap(std::shared_ptr<LocalMap> pLocalMap);

        std::shared_ptr<PointCloudBuffer> pointCloudBuffer_ = nullptr;
        bool useROSBuffer_;
        std::shared_ptr<PointCloudBufferROS> pointCloudBufferROS_ = nullptr;
        std::recursive_mutex localMapMutex_;
        std::unordered_map<long unsigned int, std::shared_ptr<LocalMap>> localMapsSet_;
        std::unordered_map<long unsigned int, long unsigned int> allKeyFramesSet_;
        std::shared_ptr<LocalMap> localMap_ = nullptr;

        std::mutex updateQueueMutex_;
        std::shared_ptr<UpdateQueue> keyFrameUpdateQueue_;
    };
}

#endif // SYSTEM_HPP_
