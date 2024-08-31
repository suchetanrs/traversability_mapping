#include "traversability_mapping/System.hpp"
// check pose of 0th frame continously.
// mutex for localmaps's gridmap.

namespace traversability_mapping
{

    System::System()
    {
        std::cout << "Called traversability system constructor" << std::endl;
        YAML::Node loaded_node = YAML::LoadFile("/usr/local/params/traversabilityParams.yaml");
        if (loaded_node["use_pointcloud_buffer"].as<bool>())
            pointCloudBuffer_ = std::make_shared<PointCloudBuffer>();
        keyFrameUpdateQueue_ = std::make_shared<UpdateQueue>();
    }

    void System::addNewKeyFrameToMap(const double timestamp,
                                     long unsigned int kfID,
                                     long unsigned int mapID,
                                     sensor_msgs::msg::PointCloud2 &sensorPointCloud)
    {
        std::cout << "addNewKeyFrameToMap" << std::endl;
        // std::cout << "traversability_mapping::System::addNewKeyFrameToMap with id: " << kfID << " and ts: " << timestamp << std::endl;
        // Add the keyframe to the local map
        if (localMapsSet_.find(mapID) != localMapsSet_.end())
        {
            if (allKeyFramesSet_.find(kfID) == allKeyFramesSet_.end())
            {
                allKeyFramesSet_[kfID] = mapID;
                localMapsSet_[mapID]->addNewKeyFrame(timestamp, kfID, sensorPointCloud, mapID);
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
        std::cout << "addNewKeyFrame2" << std::endl;
        sensor_msgs::msg::PointCloud2 sensorPointCloud = pointCloudBuffer_->getClosestPointCloud(timestamp);
        std::cout << "Removing pcl before " << timestamp - 20.0 << std::endl;
        pointCloudBuffer_->deletePointsBefore(timestamp - 20.0);
        if (sensorPointCloud.data.size() == 0)
            return;
        // std::cout << "Got closest pointcloud for: " << kfID << std::endl;
        auto pcl_sec = sensorPointCloud.header.stamp.sec + (sensorPointCloud.header.stamp.nanosec * pow(10, -9));
        std::cout << "pcl ts is: " << pcl_sec << std::endl;
        std::cout << "Difference in closest pcl ts is: " << timestamp - pcl_sec << std::endl;
        addNewKeyFrameToMap(timestamp, kfID, mapID, sensorPointCloud);

        /**
        //test
        std::cout << "Testing ADD" << std::endl;
        unsigned long long sec = static_cast<unsigned long long>(std::floor(timestamp));
        unsigned long long nanosec = static_cast<unsigned long long>((timestamp - sec) * pow(10, 9));
        addNewKeyFrame(sec + nanosec, static_cast<int>(kfID), mapID);
        */
    }

    void System::addNewKeyFrameTsULong(const unsigned long long timestamp,
                                       long unsigned int kfID,
                                       long unsigned int mapID)
    {
        std::cout << "addNewKeyFrame3" << std::endl;
        const unsigned long long timestampSecond = timestamp / 1e9;
        const unsigned long long timestampNanoSec = timestamp % (unsigned long long)1e9;
        double timestampDouble = timestampSecond + (timestampNanoSec * 1e-9);
        if (kfID < 0)
        {
            std::string errorMsg = "The given KF ID was negative. Please make sure the keyFrame IDs are positive. KF ID: " + std::to_string(kfID);
            throw std::runtime_error(errorMsg);
        }
        long unsigned int kfIDlong = static_cast<long unsigned int>(kfID);

        sensor_msgs::msg::PointCloud2 sensorPointCloud = pointCloudBuffer_->getClosestPointCloud(timestampDouble);
        std::cout << "Removing pcl before " << timestampDouble - 20.0 << std::endl;
        pointCloudBuffer_->deletePointsBefore(timestampDouble - 20.0);
        if (sensorPointCloud.data.size() == 0)
            return;
        auto pcl_sec = sensorPointCloud.header.stamp.sec + (sensorPointCloud.header.stamp.nanosec * 1e-9);
        std::cout << "pcl ts is: " << pcl_sec << std::endl;
        std::cout << "Difference in closest pcl ts is: " << timestampDouble - pcl_sec << std::endl;
        // std::cout << "Got closest pointcloud for: " << kfID << std::endl;
        // auto pcl_sec = sensorPointCloud.header.stamp.sec + (sensorPointCloud.header.stamp.nanosec * pow(10, -9));
        // std::cout << "Difference in closest pcl ts is: " << timestamp - pcl_sec << std::endl;
        addNewKeyFrameToMap(timestampDouble, kfIDlong, mapID, sensorPointCloud);
    }

    void System::addNewKeyFrameWithPCL(const unsigned long long timestamp,
                                       int kfID,
                                       long unsigned int mapID,
                                       sensor_msgs::msg::PointCloud2 sensorPointCloud)
    {
        std::cout << "addNewKeyFrame4" << std::endl;
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

    void System::updateKeyFrame(long unsigned int kfID,
                                Sophus::SE3f &poseSLAM)
    {
        // std::cout << "traversability_mapping::System::updateKeyFrame with id: " << kfID << std::endl;
        // Add the keyframe to the local map
        if (allKeyFramesSet_.find(kfID) != allKeyFramesSet_.end())
        {
            // std::cout << "System::updateKeyFrame kfid: " << kfID << std::endl;
            std::lock_guard<std::mutex> lock(updateQueueMutex_);
            keyFrameUpdateQueue_->push_back(std::make_pair(kfID, poseSLAM));
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
                                Eigen::Affine3d &poseAffine)
    {

        Sophus::SE3f poseSLAM(poseAffine.cast<float>().rotation(), poseAffine.cast<float>().translation());

        // std::cout << "traversability_mapping::System::updateKeyFrame with id: " << static_cast<long unsigned int>(kfID) << std::endl;
        // Add the keyframe to the local map
        if (allKeyFramesSet_.find(static_cast<long unsigned int>(kfID)) != allKeyFramesSet_.end())
        {
            // std::cout << "System::updateKeyFrame kfid: " << static_cast<long unsigned int>(kfID) << std::endl;
            std::lock_guard<std::mutex> lock(updateQueueMutex_);
            keyFrameUpdateQueue_->push_back(std::make_pair(static_cast<long unsigned int>(kfID), poseSLAM));
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
        if (allKeyFramesSet_.count(kfID) > 0)
        {
            if (localMapsSet_.count(mapID) > 0)
            {
                if (allKeyFramesSet_[kfID] != mapID)
                {
                    // to access keyframe from old map localMapsSet_[allKeyFramesSet_[kfID]]
                    // to access keyframe from new map localMapsSet_[mapID]
                    auto keyFramePtr = localMapsSet_[allKeyFramesSet_[kfID]]->getKeyFramesMap()[kfID];
                    keyFramePtr->setMap(mapID, localMapsSet_[mapID]->getGridMap());
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
            std::cout << "The keyframe with ID " << kfID << " is not made yet. What are you moving?" << std::endl;
        }
    }

    void System::addNewLocalMap(long unsigned int mapID)
    {
        if (localMapsSet_.find(mapID) == localMapsSet_.end())
        {
            std::cout << "Added new local map with ID: " << mapID << std::endl;
            auto newLocalMap = std::make_shared<LocalMap>(mapID, keyFrameUpdateQueue_, updateQueueMutex_);
            std::thread t(&LocalMap::Run, newLocalMap);
            t.detach();
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
                              sensor_msgs::msg::PointCloud2 &pcl2)
    {
        pointCloudBuffer_->addPointCloud(pcl2, timestamp);
    }

    void System::pushToBuffer(sensor_msgs::msg::PointCloud2 &pcl2)
    {
        double seconds = pcl2.header.stamp.sec + (pcl2.header.stamp.nanosec * pow(10, -9));
        pointCloudBuffer_->addPointCloud(pcl2, seconds);
    }

    std::shared_ptr<LocalMap> System::getLocalMap()
    {
        std::lock_guard<std::mutex> lock(localMapMutex_);
        return localMap_;
    }

    void System::setLocalMap(std::shared_ptr<LocalMap> pLocalMap)
    {
        std::lock_guard<std::mutex> lock(localMapMutex_);
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