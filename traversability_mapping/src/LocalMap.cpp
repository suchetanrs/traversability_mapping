#include "traversability_mapping/LocalMap.hpp"
#include "traversability_mapping/Helpers.hpp"

namespace traversability_mapping
{
    LocalMap::LocalMap(long unsigned int mapID, std::shared_ptr<UpdateQueue> keyFrameUpdateQueue, std::mutex &updateQueueMutex)
        : updateQueueMutex_(updateQueueMutex),
          mapID_(mapID),
          keyFrameUpdateQueue_(keyFrameUpdateQueue)
    {
        // Load YAML file and retrieve parameters
        YAML::Node loaded_node = YAML::LoadFile("/usr/local/params/traversabilityParams.yaml");
        
        // Traversability Params
        half_size_gridmap_ = loaded_node["half_size_local_map"].as<double>();
        
        loadedKFParams_.resolution_ = loaded_node["resolution_local_map"].as<double>();
        resolution_ = loadedKFParams_.resolution_;
        loadedKFParams_.half_size_traversability_ = loaded_node["half_size_traversability"].as<double>();
        loadedKFParams_.security_distance_ = loaded_node["security_distance"].as<double>();
        loadedKFParams_.ground_clearance_ = loaded_node["ground_clearance"].as<double>();
        loadedKFParams_.max_slope_ = loaded_node["max_slope"].as<double>();
        loadedKFParams_.robot_height_ = loaded_node["robot_height"].as<double>();
        loadedKFParams_.translation_change_threshold_ = loaded_node["translation_change_threshold"].as<double>();
        loadedKFParams_.rotation_change_threshold_ = loaded_node["rotation_change_threshold"].as<double>();
        loadedKFParams_.is_kf_optimization_enabled_ = loaded_node["is_kf_optimization_enabled"].as<bool>();

        // Other params
        Eigen::Translation3f translation(
            loaded_node["T_SLAMFrameToLidarFrame"]["translation"]["x"].as<float>(),
            loaded_node["T_SLAMFrameToLidarFrame"]["translation"]["y"].as<float>(),
            loaded_node["T_SLAMFrameToLidarFrame"]["translation"]["z"].as<float>());
        Eigen::Quaternion<float> quaternion(
            loaded_node["T_SLAMFrameToLidarFrame"]["quaternion"]["w"].as<float>(),
            loaded_node["T_SLAMFrameToLidarFrame"]["quaternion"]["x"].as<float>(),
            loaded_node["T_SLAMFrameToLidarFrame"]["quaternion"]["y"].as<float>(),
            loaded_node["T_SLAMFrameToLidarFrame"]["quaternion"]["z"].as<float>());
        Tbv_ = translation * quaternion;

        SLAMSystem_ = loaded_node["SLAM_System"].as<std::string>();

        typeConversion_ = std::make_shared<TraversabilityTypeConversions>();
        grid_map::GridMap gridMap_({"hazard", "step_haz", "roughness_haz", "slope_haz", "border_haz", "elevation", "kfid"});
        gridMap_.setFrameId("map");
        gridMap_.setGeometry(grid_map::Length(2. * half_size_gridmap_, 2. * half_size_gridmap_), resolution_);
        Eigen::Vector2d slamPosition;
        slamPosition.x() = 120.0;
        slamPosition.y() = 70.0;
        gridMap_.setPosition(slamPosition);
        pGridMap_ = std::make_shared<grid_map::GridMap>(gridMap_);
    }

    LocalMap::~LocalMap() {}

    void LocalMap::Run()
    {
        // Loop runs forever.
        while (1)
        {
            // std::cout << "Keys in the map: " << mapID_ << ": ";
            // for (const auto &entry : keyFramesMap_)
            // {
            //     std::cout << entry.first << " ";
            // }
            // std::cout << std::endl
            //           << "Total size: " << keyFramesMap_.size() << std::endl;
            processUpdateQueue();
            // if map set to active, the cache is recomputed continously.
            // TODO: Run this only once when update is recieved Instead of every 500 milli.
            while (activeMap_)
            {
                // std::cout << "Keys in the map: " << mapID_ << ": ";
                // for (const auto &entry : keyFramesMap_)
                // {
                //     std::cout << entry.first << " ";
                // }
                // std::cout << std::endl
                //           << "Total size: " << keyFramesMap_.size() << std::endl;
                processUpdateQueue();
                // std::cout << "Currently running active map id: " << mapID_ << " and the update queue size is: " << keyFrameUpdateQueue_->size() << std::endl;
                for (auto &pair : keyFramesMap_)
                {
                    auto keyFramePtr = pair.second;

                    // Check if the pointer is valid before calling recomputeCache
                    if (keyFramePtr)
                    {
                        keyFramePtr->recomputeCache();
                    }
                }
                nav_msgs::msg::OccupancyGrid occupancyGrid_msg;
                traversability_mapping::gridMapToOccupancyGrid(*pGridMap_, "hazard", 0., 1., occupancyGrid_msg);
                gridMapOccupancy_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(occupancyGrid_msg);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                // auto end_time = std::chrono::high_resolution_clock::now();
                // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                // std::cout << "Processing time: " << duration.count() / 1e3 << " seconds.";
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    std::shared_ptr<KeyFrame> LocalMap::addNewKeyFrame(double timestamp,
                                                       long unsigned int kfID,
                                                       sensor_msgs::msg::PointCloud2 &pointCloud,
                                                       long unsigned int mapID)
    {
        std::shared_ptr<KeyFrame> keyFrame = std::make_shared<KeyFrame>(timestamp, kfID, pointCloud, pGridMap_, mapID, Tbv_, loadedKFParams_);
        std::lock_guard<std::mutex> lock(keyFramesMapMutex);
        keyFramesMap_[kfID] = keyFrame;
        return keyFrame;
    }

    void LocalMap::addAlreadyDeclaredKF(std::shared_ptr<KeyFrame> keyFrame)
    {
        std::lock_guard<std::mutex> lock(keyFramesMapMutex);
        keyFramesMap_[keyFrame->getKfID()] = keyFrame;
    }

    void LocalMap::deleteKeyFrame(long unsigned int kfID)
    {
        std::lock_guard<std::mutex> lock(keyFramesMapMutex);
        auto it = keyFramesMap_.find(kfID);
        if (it != keyFramesMap_.end())
        {
            keyFramesMap_.erase(it); // Erase the key-value pair
            // std::cout << "Value associated with key " << kfID << " deleted from map: " << mapID_ << std::endl;
        }
        else
        {
            std::cout << "Key " << kfID << " not found." << std::endl;
        }
    }

    // void LocalMap::updateKeyFrame(long unsigned int kfID,
    //                               Sophus::SE3f &poseSLAM)
    // {
    //     /**
    //      * LOGIC 1: Update on the go.
    //      * This resulted in an older pose being updated later than the latest pose in the queue so not being used.
    //      */
    //     // if (keyFramesMap_.count(kfID) > 0)
    //     // {
    //     //     keyFramesMap_[kfID]->setSLAMPose(poseSLAM);
    //     //     keyFramesMap_[kfID]->setPose(typeConversion_->se3ToAffine<Eigen::Affine3f>(poseSLAM));
    //     // }
    //     // else
    //     // {
    //     //     updateQueueLock_.lock();
    //     //     keyFrameUpdateQueue_.push_back(std::make_pair(kfID, poseSLAM));
    //     //     updateQueueLock_.unlock();
    //     //     std::cout << "Cant find element: " << kfID << " in map, so pushing to queue." << std::endl;
    //     // }

    //     /** LOGIC 2: Only push to queue everytime
    //      *Will be cleared automatically and the latest element is always in the end of the vector.
    //      */
    //     // updateQueueLock_.lock();
    //     // keyFrameUpdateQueue_->push_back(std::make_pair(kfID, poseSLAM));
    //     // updateQueueLock_.unlock();
    //     // std::cout << "Element: " << kfID << " in map, so pushing to queue." << std::endl;
    // }

    std::unordered_map<long unsigned int, std::shared_ptr<KeyFrame>> &LocalMap::getKeyFramesMap()
    {
        std::lock_guard<std::mutex> lock(keyFramesMapMutex);
        return keyFramesMap_;
    }

    const std::shared_ptr<nav_msgs::msg::OccupancyGrid> &LocalMap::getOccupancyMap()
    {
        return gridMapOccupancy_;
    }

    std::shared_ptr<grid_map::GridMap> LocalMap::getGridMap()
    {
        return pGridMap_;
    }

    void LocalMap::processUpdateQueue()
    {
        // std::cout << mapID_ << " Processing queue with size: " << keyFrameUpdateQueue_->size() << std::endl;
        for (auto it = keyFrameUpdateQueue_->begin(); it != keyFrameUpdateQueue_->end();)
        {
            // Check if the long unsigned int value is equal to 100
            std::lock_guard<std::mutex> lock(keyFramesMapMutex);
            if (keyFramesMap_.count(it->first) > 0)
            {
                keyFramesMap_[it->first]->setSLAMPose(it->second);

                std::cout << "NEW CONVERSION!" << std::endl;

                if (SLAMSystem_ == "ORB3")
                {
                    keyFramesMap_[it->first]->setPose(typeConversion_->se3ToAffine<Eigen::Affine3f>(it->second, true));
                }
                else if (SLAMSystem_ == "ISAE")
                {
                    keyFramesMap_[it->first]->setPose(typeConversion_->se3ToAffine<Eigen::Affine3f>(it->second, false));
                }
                // updateQueueLock_.lock();
                std::lock_guard<std::mutex> lock(updateQueueMutex_);
                it = keyFrameUpdateQueue_->erase(it);
                // std::cout << "Removing element: " << it->first << " from queue." << std::endl;
                // updateQueueLock_.unlock();
            }
            else
            {
                // Move to the next element
                ++it;
                // std::cout << "Cant find element: " << it->first << " in map," << mapID_ << "going to remain in queue." << std::endl;
            }
        }
    }
}

// make pcl transformer gpt queried
// invlove gridmap after this and making of local map.