#ifndef SYSTEM_HPP_
#define SYSTEM_HPP_

#include "traversability_mapping/LocalMap.hpp"
#include "traversability_mapping/PointCloudBuffer.hpp"
#include "traversability_mapping/KeyFrame.hpp"
#include "traversability_mapping_common/type_conversion.hpp"
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
                                   sensor_msgs::msg::PointCloud2 sensorPointCloud);

        void updateKeyFrame(long unsigned int kfID,
                            Sophus::SE3f &poseSLAM);

        void updateKeyFrame(unsigned long long kfID,
                            Eigen::Affine3d &poseAffine);

        void addNewLocalMap(long unsigned int mapID);

        void pushToBuffer(double timestamp,
                          sensor_msgs::msg::PointCloud2 &pcl2);

        void pushToBuffer(sensor_msgs::msg::PointCloud2 &pcl2);

        std::shared_ptr<LocalMap> getLocalMap();

        void updateKFMap(long unsigned int kfID,
                         long unsigned int mapID);

    private:
        void addNewKeyFrameToMap(const double timestamp,
                                 long unsigned int kfID,
                                 long unsigned int mapID,
                                 sensor_msgs::msg::PointCloud2 &sensorPointCloud);

        void setCurrentMap(long unsigned int mapID);

        void makeAllMapsInactive();

        void setLocalMap(std::shared_ptr<LocalMap> pLocalMap);

        std::shared_ptr<PointCloudBuffer> pointCloudBuffer_ = nullptr;

        std::recursive_mutex localMapMutex_;
        std::unordered_map<long unsigned int, std::shared_ptr<LocalMap>> localMapsSet_;
        std::unordered_map<long unsigned int, long unsigned int> allKeyFramesSet_;
        std::shared_ptr<LocalMap> localMap_ = nullptr;

        std::mutex updateQueueMutex_;
        std::shared_ptr<UpdateQueue> keyFrameUpdateQueue_;
    };
}

#endif // SYSTEM_HPP_
