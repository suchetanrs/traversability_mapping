/* Traversability Mapping - A global and local traversability mapping algorithm.
 * Copyright (C) 2024 Suchetan Saravanan and Damien Vivet
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.  If not, see
 * <https://www.gnu.org/licenses/>.
 */
#include "traversability_mapping/System.hpp"

namespace traversability_mapping
{

    System::System()
    {
        std::cout << "Called traversability system constructor" << std::endl;
        const std::string yaml_base_path = ament_index_cpp::get_package_share_directory("traversability_mapping");
        std::string yaml_path;
        YAML::Node yaml_node;
        yaml_path = yaml_base_path + "/params/traversabilityParams.yaml";
        YAML::Node loaded_node = YAML::LoadFile(yaml_path);
        if (loaded_node["use_pointcloud_buffer"].as<bool>())
        {
            pointCloudBuffer_ = std::make_shared<PointCloudBuffer>();
            useROSBuffer_ = parameterInstance.getValue<bool>("use_ros_buffer");
            pointCloudBufferROS_ = std::make_shared<PointCloudBufferROS>();
        }
        keyFrameUpdateQueue_ = std::make_shared<UpdateQueue>();
    }

    void System::setExtrinsicParameters(Eigen::Affine3f &tf_SlamToLidar, Eigen::Affine3f &tf_BaseToSlam)
    {
        parameterInstance.setValue<Eigen::Affine3f>("T_SLAMFrameToLidarFrame", tf_SlamToLidar);
        parameterInstance.setValue<Eigen::Affine3f>("T_BasefootprintToSLAM", tf_BaseToSlam);
    }

    void System::addNewKeyFrameToMap(const double timestamp,
                                     long unsigned int kfID,
                                     long unsigned int mapID,
                                     std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensorPointCloud)
    {
        // std::cout << "addNewKeyFrameToMap" << std::endl;
        // std::cout << "traversability_mapping::System::addNewKeyFrameToMap with id: " << kfID << " and ts: " << timestamp << std::endl;
        // Add the keyframe to the local map
        std::lock_guard<std::recursive_mutex> lock(localMapMutex_);
        if (localMapsSet_.find(mapID) != localMapsSet_.end())
        {
            if (allKeyFramesSet_.find(kfID) == allKeyFramesSet_.end())
            {
                allKeyFramesSet_[kfID] = mapID;
                localMapsSet_[mapID]->addNewKeyFrame(timestamp, kfID, *sensorPointCloud, mapID);
                // std::cout << "ADDING KEYFRAME TO MAP!!!!" << mapID << std::endl;
                if (localMapsSet_[mapID] != localMap_)
                    setCurrentMap(mapID);
            }
            else
            {
                std::cout << "They keyframe with this ID: " << kfID << " has already been added. Not adding again" << std::endl;
            }
        }
        else
        {
            std::cout << "Local Map not initialized. Not adding the keyframe." << std::endl;
        }
        // std::cout << "Added new KeyFrame with ID: " << kfID << std::endl;
    }

    void System::addNewKeyFrameTsDouble(const double timestamp,
                                        long unsigned int kfID,
                                        long unsigned int mapID)
    {
        // std::cout << "addNewKeyFrame2" << std::endl;
        if (kfID < 0)
        {
            std::string errorMsg = "The given KF ID was negative. Please make sure the keyFrame IDs are positive. KF ID: " + std::to_string(kfID);
            throw std::runtime_error(errorMsg);
        }
        long unsigned int kfIDlong = static_cast<long unsigned int>(kfID);
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensorPointCloud;
#ifdef WITH_ROS2_SENSOR_MSGS
        if(useROSBuffer_)
            sensorPointCloud = pointCloudBufferROS_->getClosestPointCloud(timestamp);
        else
            sensorPointCloud = pointCloudBuffer_->getClosestPointCloud(timestamp);
#else
        sensorPointCloud = pointCloudBuffer_->getClosestPointCloud(timestamp);
#endif
        // std::cout << "Removing pcl before " << timestamp - 20.0 << std::endl;
        // pointCloudBuffer_->deletePointsBefore(timestamp - 20.0);
        if (sensorPointCloud == nullptr)
            return;
        // auto pcl_sec = sensorPointCloud.header.stamp.sec + (sensorPointCloud.header.stamp.nanosec * 1e-9);
        // std::cout << "pcl ts is: " << pcl_sec << std::endl;
        // std::cout << "Difference in closest pcl ts is: " << timestamp - pcl_sec << std::endl;
        // std::cout << "Got closest pointcloud for: " << kfID << std::endl;
        // auto pcl_sec = sensorPointCloud.header.stamp.sec + (sensorPointCloud.header.stamp.nanosec * pow(10, -9));
        // std::cout << "Difference in closest pcl ts is: " << timestamp - pcl_sec << std::endl;
        addNewKeyFrameToMap(timestamp, kfIDlong, mapID, sensorPointCloud);
    }

    void System::addNewKeyFrameTsULong(const unsigned long long timestamp,
                                       long unsigned int kfID,
                                       long unsigned int mapID)
    {
        // std::cout << "addNewKeyFrame3" << std::endl;
        // std::cout << "TS input:" << timestamp << std::endl;
        const unsigned long long timestampSecond = timestamp / 1e9;
        const unsigned long long timestampNanoSec = timestamp % (unsigned long long)1e9;
        double timestampDouble = timestampSecond + (timestampNanoSec * 1e-9);
        // std::cout << "TS double:" << timestampDouble << std::endl;
        if (kfID < 0)
        {
            std::string errorMsg = "The given KF ID was negative. Please make sure the keyFrame IDs are positive. KF ID: " + std::to_string(kfID);
            throw std::runtime_error(errorMsg);
        }
        long unsigned int kfIDlong = static_cast<long unsigned int>(kfID);
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensorPointCloud;
#ifdef WITH_ROS2_SENSOR_MSGS
        if(useROSBuffer_)
            sensorPointCloud = pointCloudBufferROS_->getClosestPointCloud(timestampDouble);
        else
            sensorPointCloud = pointCloudBuffer_->getClosestPointCloud(timestampDouble);
#else
        sensorPointCloud = pointCloudBuffer_->getClosestPointCloud(timestampDouble);
#endif
        // std::cout << "Removing pcl before " << timestampDouble - 20.0 << std::endl;
        // pointCloudBuffer_->deletePointsBefore(timestampDouble - 20.0);
        if (sensorPointCloud == nullptr)
            return;
        // auto pcl_sec = sensorPointCloud.header.stamp.sec + (sensorPointCloud.header.stamp.nanosec * 1e-9);
        // std::cout << "pcl ts is: " << pcl_sec << std::endl;
        // std::cout << "Difference in closest pcl ts is: " << timestampDouble - pcl_sec << std::endl;
        // std::cout << "Got closest pointcloud for: " << kfID << std::endl;
        // auto pcl_sec = sensorPointCloud.header.stamp.sec + (sensorPointCloud.header.stamp.nanosec * pow(10, -9));
        // std::cout << "Difference in closest pcl ts is: " << timestamp - pcl_sec << std::endl;
        addNewKeyFrameToMap(timestampDouble, kfIDlong, mapID, sensorPointCloud);
    }

    void System::addNewKeyFrameWithPCL(const unsigned long long timestamp,
                                       int kfID,
                                       long unsigned int mapID,
                                       std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensorPointCloud)
    {
        // std::cout << "addNewKeyFrame4" << std::endl;
        const unsigned long long timestampSecond = timestamp / 1e9;
        const unsigned long long timestampNanoSec = timestamp % (unsigned long long)1e9;
        double timestampDouble = timestampSecond + (timestampNanoSec * 1e-9);
        if (kfID < 0)
        {
            std::string errorMsg = "The given KF ID was negative. Please make sure the keyFrame IDs are positive. KF ID: " + std::to_string(kfID);
            throw std::runtime_error(errorMsg);
        }
        long unsigned int kfIDlong = static_cast<long unsigned int>(kfID);
        addNewKeyFrameToMap(timestampDouble, kfIDlong, mapID, sensorPointCloud);
    }

    void System::informLoopClosure()
    {
        localMapsSet_[0]->clearEntireMap();
    }

    void System::updateKeyFrame(long unsigned int kfID,
                                Sophus::SE3f &poseSLAM,
                                long unsigned int numConnections)
    {
        // std::cout << "traversability_mapping::System::updateKeyFrame with id: " << kfID << std::endl;
        // Add the keyframe to the local map
        std::lock_guard<std::recursive_mutex> lock(localMapMutex_);
        if (allKeyFramesSet_.find(kfID) != allKeyFramesSet_.end())
        {
            // std::cout << "System::updateKeyFrame kfid: " << kfID << std::endl;
            updateQueueMutex_.lock();
            keyFrameUpdateQueue_->push(KeyFrameUpdateData(kfID, poseSLAM, 0));
            updateQueueMutex_.unlock();
            // std::cout << "UPDATING KEYFRAME TO MAP!!!!" << mapID << std::endl;
        }
        else
        {
            std::cout << "KeyFrame not initialized. Not updating the keyframe." << std::endl;
        }
        // std::cout << "updated KeyFrame with ID: " << kfID << std::endl;

        /**
        // TEST UPDATE
        std::cout << "Test UPDATE" << std::endl;
        traversability_mapping::TraversabilityTypeConversions typeC_;
        // eigen result in in orb coords.

        std::cout << "rotation original3 se3" << std::endl;
        std::cout << poseSLAM.rotationMatrix() << std::endl;
        std::cout << "translation original23 se3" << std::endl;
        std::cout << poseSLAM.translation() << std::endl;
        Eigen::Affine3d eigenPoseSLAM = typeC_.se3ToAffine<Eigen::Affine3d>(poseSLAM, true);
        std::cout << "rotation original23 eigen" << std::endl;
        std::cout << eigenPoseSLAM.rotation() << std::endl;
        std::cout << "translation original23 eigen" << std::endl;
        std::cout << eigenPoseSLAM.translation() << std::endl;
        updateKeyFrame(static_cast<unsigned long long>(kfID), eigenPoseSLAM);
        */
    }

    void System::updateKeyFrame(unsigned long long kfID,
                                Eigen::Affine3d &poseAffine,
                                long unsigned int numConnections)
    {
        Sophus::SE3f poseSLAM(poseAffine.cast<float>().rotation(), poseAffine.cast<float>().translation());

        // std::cout << "traversability_mapping::System::updateKeyFrame with id: " << static_cast<long unsigned int>(kfID) << std::endl;
        // Add the keyframe to the local map
        std::lock_guard<std::recursive_mutex> lock(localMapMutex_);
        if (allKeyFramesSet_.find(static_cast<long unsigned int>(kfID)) != allKeyFramesSet_.end())
        {
            // std::cout << "System::updateKeyFrame kfid: " << static_cast<long unsigned int>(kfID) << std::endl;
            updateQueueMutex_.lock();
            keyFrameUpdateQueue_->push(KeyFrameUpdateData(static_cast<long unsigned int>(kfID), poseSLAM, 0));
            updateQueueMutex_.unlock();
            // std::cout << "UPDATING KEYFRAME TO MAP!!!!" << mapID << std::endl;
        }
        else
        {
            std::cout << "KeyFrame not initialized. Not updating the keyframe." << std::endl;
        }
        // std::cout << "updated KeyFrame with ID: " << static_cast<long unsigned int>(kfID) << std::endl;
    }

    void System::updateKFMap(long unsigned int kfID,
                             long unsigned int mapID)
    {
        // std::cout << "Incoming: kf: " << kfID << " mapID: " << mapID << std::endl;
        // std::cout << "Exisiting: kf: " << kfID << " mapID: " << allKeyFramesSet_[kfID] << std::endl;
        std::lock_guard<std::recursive_mutex> lock(localMapMutex_);
        if (allKeyFramesSet_.find(kfID) != allKeyFramesSet_.end())
        {
            if (localMapsSet_.find(mapID) != localMapsSet_.end())
            {
                if (allKeyFramesSet_[kfID] != mapID)
                {
                    // to access keyframe from old map localMapsSet_[allKeyFramesSet_[kfID]]
                    // to access keyframe from new map localMapsSet_[mapID]
                    auto keyFramePtr = localMapsSet_[allKeyFramesSet_[kfID]]->getKeyFramesMap().at(kfID);
                    keyFramePtr->setMap(mapID, localMapsSet_[mapID]->getGridMap(), localMapsSet_[mapID]->getGridMapMutex());
                    localMapsSet_[mapID]->addAlreadyDeclaredKF(keyFramePtr);
                    localMapsSet_[allKeyFramesSet_[kfID]]->deleteKeyFrame(kfID);
                    allKeyFramesSet_[kfID] = mapID;
                    // push the same pose again so that the traversability is marked again on the new map.
                    keyFramePtr->updateGridAfterMapChange();
                    // keyFrameUpdateQueue_->push_back(std::make_pair(kfID, keyFramePtr->getSLAMPose()));
                }
                else
                {
                    std::cout << "Not moving KF: " << kfID << std::endl;
                }
            }
            else
            {
                std::cout << "The map ID " << mapID << " has not been init yet. What are you moving to?" << std::endl;
            }
        }
        else
        {
            std::cout << "The keyframe with ID " << kfID << " is not made yet. What are you moving? It may also be deleted." << std::endl;
        }
    }

    void System::deleteKeyFrame(unsigned long long kfID)
    {
        std::lock_guard<std::recursive_mutex> lock(localMapMutex_);
        auto kf_c = static_cast<long unsigned int>(kfID);
        if (allKeyFramesSet_.find(kf_c) != allKeyFramesSet_.end())
        {
            localMapsSet_[allKeyFramesSet_[kf_c]]->deleteKeyFrame(kf_c);
            allKeyFramesSet_.erase(kf_c);
            spatialHashGridInstance_.deleteKeyframe(kf_c);
        }
        else
        {
            std::cout << "Cannot delete KF" << kf_c << ". Not found.." << std::endl;
        }
    }

    void System::addNewLocalMap(long unsigned int mapID)
    {
        std::lock_guard<std::recursive_mutex> lock(localMapMutex_);
        if (localMapsSet_.find(mapID) == localMapsSet_.end())
        {
            std::cout << "Added new local map with ID: " << mapID << std::endl;
            auto newLocalMap = std::make_shared<LocalMap>(mapID, keyFrameUpdateQueue_, updateQueueMutex_);
            std::thread t(&LocalMap::RunUpdateQueue, newLocalMap);
            t.detach();
            std::thread t2(&LocalMap::RunTraversability, newLocalMap);
            t2.detach();
            std::thread t3(&LocalMap::RunLocalKeyFrames, newLocalMap);
            t3.detach();
            localMapsSet_[mapID] = newLocalMap;
            setCurrentMap(mapID);
        }
        else
        {
            std::cout << "Local map with the ID: " << mapID << " has already been made. Not making again" << std::endl;
        }
    }

    void System::setCurrentMap(long unsigned int mapID)
    {
        std::lock_guard<std::recursive_mutex> lock(localMapMutex_);
        makeAllMapsInactive();
        if (localMapsSet_.find(mapID) != localMapsSet_.end())
        {
            localMapsSet_[mapID]->setActiveMap(true);
            setLocalMap(localMapsSet_[mapID]);
        }
        else
        {
            std::cout << "Local Map not initialized. Not setting active map." << std::endl;
        }
    }

    void System::pushToBuffer(double timestamp,
                              std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &pcl2)
    {
        pointCloudBuffer_->addPointCloud(pcl2, timestamp);
    }

    void System::pushToBuffer(double timestamp,
                                pcl::PCLPointCloud2& pcl_pc2)
    {
        throw std::runtime_error("This pushToBuffer function is not implemented yet.");
    }

#ifdef WITH_ROS2_SENSOR_MSGS
    void System::pushToBuffer(sensor_msgs::msg::PointCloud2::SharedPtr pcl2)
    {
        // pcl::PCLPointCloud2 pcl_pc2;
        // toPCL(*pcl2, pcl_pc2);
        // std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pointcloudInput = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        // pcl::fromPCLPointCloud2(pcl_pc2, *pointcloudInput);
        double seconds = pcl2->header.stamp.sec + (pcl2->header.stamp.nanosec * pow(10, -9));
        pointCloudBufferROS_->addPointCloud(pcl2, seconds);
    }
#endif

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> System::getGlobalPointCloud(float voxel_size_x, float voxel_size_y, float voxel_size_z)
    {
        std::lock_guard<std::recursive_mutex> lock(localMapMutex_);
        return localMap_->getStitchedPointCloud(voxel_size_x, voxel_size_y, voxel_size_z);
    }

    std::shared_ptr<LocalMap> System::getLocalMap()
    {
        std::lock_guard<std::recursive_mutex> lock(localMapMutex_);
        return localMap_;
    }

    void System::setLocalMap(std::shared_ptr<LocalMap> pLocalMap)
    {
        std::lock_guard<std::recursive_mutex> lock(localMapMutex_);
        localMap_ = pLocalMap;
    }

    void System::makeAllMapsInactive()
    {
        for (auto &pair : localMapsSet_)
        {
            pair.second->setActiveMap(false);
        }
    }
}

// int main()
// {
//     // Create a LocalMap object
//     traversability_mapping::System system;

//     // Create a dummy ORB_SLAM3::KeyFrame object
//     ORB_SLAM3::KeyFrame keyFrame;

//     // Add the key frame to the local map
//     system.addKeyFrame(&keyFrame);

//     // Update the pose of the key frame
//     system.updateKeyFrame(&keyFrame);

//     return 0;
// }