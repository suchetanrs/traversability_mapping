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
#include "traversability_mapping/LocalMap.hpp"
#include "traversability_mapping/Helpers.hpp"

namespace traversability_mapping
{
    LocalMap::LocalMap(long unsigned int mapID, std::shared_ptr<UpdateQueue> keyFrameUpdateQueue, std::mutex &updateQueueMutex)
        : updateQueueMutex_(updateQueueMutex),
          mapID_(mapID),
          keyFrameUpdateQueue_(keyFrameUpdateQueue)
    {
        Tsv_ = parameterInstance.getValue<Eigen::Affine3f>("T_SLAMFrameToLidarFrame");
        Tbs_ = parameterInstance.getValue<Eigen::Affine3f>("T_BasefootprintToSLAM");

        typeConversion_ = std::make_shared<TraversabilityTypeConversions>();
        grid_map::GridMap gridMap_({"num_additions", "hazard", "step_haz", "roughness_haz", "slope_haz", "border_haz", "elevation", "kfid"});
        gridMap_.setFrameId("map");
        gridMap_.setGeometry(grid_map::Length(2. * parameterInstance.getValue<double>("half_size_local_map"), 2. * parameterInstance.getValue<double>("half_size_local_map")), parameterInstance.getValue<double>("resolution_local_map"));
        Eigen::Vector2d slamPosition;
        slamPosition.x() = parameterInstance.getValue<double>("grid_center_x");
        slamPosition.y() = parameterInstance.getValue<double>("grid_center_y");
        gridMap_.setPosition(slamPosition);
        pGridMap_ = std::make_shared<grid_map::GridMap>(gridMap_);
        masterGridMapMutex_ = std::make_shared<std::mutex>();
        markVirtualBoundary("/usr/local/params/virtual_boundary.csv");
    }

    LocalMap::~LocalMap() {}

    void LocalMap::markVirtualBoundary(const std::string &csvFilePath)
    {
        if(!parameterInstance.getValue<bool>("use_virtual_boundary"))
        {
            return;
        }
        // 1) Add "virtual_boundary" layer if it doesn't exist already.
        if (!pGridMap_->exists("virtual_boundary"))
        {
            std::lock_guard<std::mutex> lock(*masterGridMapMutex_);
            // Initialize the entire layer to 0.0
            pGridMap_->add("virtual_boundary", 0.0);
        }

        // 2) Read line segments from CSV
        std::vector<LineSegment2D> segments;
        {
            std::ifstream file(csvFilePath);
            if (!file.is_open())
            {
                throw std::runtime_error("Cannot open CSV file: " + csvFilePath);
            }

            std::string line;
            while (std::getline(file, line))
            {
                if (line.empty())
                    continue;
                std::stringstream ss(line);

                float x1, y1, z1, x2, y2, z2;
                char comma;
                // CSV format: x1,y1,z1,x2,y2,z2
                if (!(ss >> x1 >> comma >> y1 >> comma >> z1 >> comma >> x2 >> comma >> y2 >> comma >> z2))
                {
                    throw std::runtime_error("Invalid CSV row: " + line);
                }

                segments.push_back({x1, y1, x2, y2});
            }
        }

        // 3) Mark each line segment by sampling points along it
        const int numSamples = 100; // or adjust as needed
        for (const auto &seg : segments)
        {
            for (int i = 0; i < numSamples; ++i)
            {
                float alpha = static_cast<float>(i) / (numSamples - 1);
                float x = seg.x1 + alpha * (seg.x2 - seg.x1);
                float y = seg.y1 + alpha * (seg.y2 - seg.y1);

                grid_map::Position pos(x, y);

                // Check if within map bounds, then mark "virtual_boundary" = 1.0
                if (pGridMap_->isInside(pos))
                {
                    std::lock_guard<std::mutex> lock(*masterGridMapMutex_);
                    pGridMap_->atPosition("virtual_boundary", pos) = 1.0f;
                }
            }
        }
    }

    void LocalMap::RunUpdateQueue()
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
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                // std::cout << "Currently running active map id: " << mapID_ << " and the update queue size is: " << keyFrameUpdateQueue_->size() << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    void LocalMap::RunTraversability()
    {
        int sleep = parameterInstance.getValue<int>("global_adjustment_sleep");
        while (1)
        {
            while (activeMap_)
            {
                std::unordered_map<long unsigned int, std::shared_ptr<KeyFrame>> keyFramesMapCopy_;
                // create copy to avoid concurrent access and maintain RT mapping.
                {
                    std::lock_guard<std::mutex> lock(keyFramesMapMutex);
                    keyFramesMapCopy_ = keyFramesMap_;
                }
                for (auto &pair : keyFramesMapCopy_)
                {
                    if (!activeMap_)
                    {
                        globalMappingRunning_ = false;
                        break;
                    }
                    auto keyFramePtr = pair.second;

                    // Check if the pointer is valid before calling recomputeCache
                    if (keyFramePtr)
                    {
                        keyFramePtr->recomputeCache(true);
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(sleep));
                }
                std::cout << "**** Global adjustment complete ****" << std::endl;
                std::cout << "**** Will take \"atleast\" " << sleep * keyFramesMap_.size() << " ms for next adjustment****" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                // auto end_time = std::chrono::high_resolution_clock::now();
                // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                // std::cout << "Processing time: " << duration.count() / 1e3 << " seconds.";
            }
            globalMappingRunning_ = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }

    void LocalMap::RunLocalKeyFrames()
    {
        while (1)
        {
            while (1)
            {
                localKeyFramesMutex.lock();
                if (mLocalKeyFrames_.size() == 0 || !activeMap_)
                {
                    localKeyFramesMutex.unlock();
                    localMappingRunning_ = false;
                    break;
                }
                auto kfPtr = mLocalKeyFrames_.back();
                mLocalKeyFrames_.pop_back();
                if (mLocalKeyFrames_.size() > parameterInstance.getValue<double>("num_local_keyframes"))
                {
                    // Remove the oldest element
                    mLocalKeyFrames_.pop_front();
                }
                localKeyFramesMutex.unlock();
                kfPtr->recomputeCache(false);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            std::cout << "No Local Keyframes present ..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }

    void LocalMap::clearEntireMap()
    {
        activeMap_ = false;
        globalMappingRunning_ = true;
        localMappingRunning_ = true;
        while (globalMappingRunning_ || localMappingRunning_)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            std::cout << "Waiting for local (" << localMappingRunning_ << ") and global mapping (" << globalMappingRunning_ << ")to stop" << std::endl;
        }
        std::unordered_map<long unsigned int, std::shared_ptr<KeyFrame>> keyFramesMapCopy_;
        // create copy to avoid concurrent access and maintain RT mapping.
        {
            std::lock_guard<std::mutex> lock(keyFramesMapMutex);
            keyFramesMapCopy_ = keyFramesMap_;
        }
        for (auto &pair : keyFramesMapCopy_)
        {
            auto keyFramePtr = pair.second;

            // Check if the pointer is valid before calling recomputeCache
            if (keyFramePtr)
            {
                keyFramePtr->clearStrayValuesInGrid();
            }
        }
        activeMap_ = true;
    }

    std::shared_ptr<KeyFrame> LocalMap::addNewKeyFrame(double timestamp,
                                                       long unsigned int kfID,
                                                       pcl::PointCloud<pcl::PointXYZ> &pointCloud,
                                                       long unsigned int mapID)
    {
        std::shared_ptr<KeyFrame> keyFrame = std::make_shared<KeyFrame>(timestamp, kfID, pointCloud, pGridMap_, masterGridMapMutex_, mapID, Tsv_, Tbs_);
        std::lock_guard<std::mutex> lock(keyFramesMapMutex);
        keyFramesMap_[kfID] = keyFrame;
        localKeyFramesMutex.lock();
        mLocalKeyFrames_.push_back(keyFrame);
        localKeyFramesMutex.unlock();
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
            // it->second->clearStrayValuesInGrid();
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

    const std::unordered_map<long unsigned int, std::shared_ptr<KeyFrame>> &LocalMap::getKeyFramesMap() const
    {
        return keyFramesMap_;
    }
#ifdef WITH_ROS2_SENSOR_MSGS
    const std::shared_ptr<nav_msgs::msg::OccupancyGrid> LocalMap::getOccupancyMap()
    {
        nav_msgs::msg::OccupancyGrid occupancyGrid_msg;
        traversability_mapping::gridMapToOccupancyGrid(*pGridMap_, "hazard", 0., 1., occupancyGrid_msg);
        gridMapOccupancy_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(occupancyGrid_msg);
        return gridMapOccupancy_;
    }
#endif
    const std::shared_ptr<grid_map::GridMap> LocalMap::getGridMap() const
    {
        return pGridMap_;
    }

    void LocalMap::processUpdateQueue()
    {
        // std::cout << mapID_ << " Processing queue with size: " << keyFrameUpdateQueue_->size() << std::endl;
        while (1)
        {
            updateQueueMutex_.lock();
            if (keyFrameUpdateQueue_->size() == 0)
            {
                updateQueueMutex_.unlock();
                break;
            }
            auto kfPoseToUpdate = keyFrameUpdateQueue_->front();
            keyFrameUpdateQueue_->pop();
            updateQueueMutex_.unlock();
            std::shared_ptr<KeyFrame> kfPtrToUpdate = nullptr;
            {
                std::lock_guard<std::mutex> lock(keyFramesMapMutex);
                if (keyFramesMap_.find(kfPoseToUpdate.id) != keyFramesMap_.end())
                {
                    kfPtrToUpdate = keyFramesMap_[kfPoseToUpdate.id];
                }
            }
            if (kfPtrToUpdate)
            {
                kfPtrToUpdate->setSLAMPose(kfPoseToUpdate.pose);
                kfPtrToUpdate->setConnections(kfPoseToUpdate.numConnections);

                // std::cout << "NEW CONVERSION!" << std::endl;

                if (parameterInstance.getValue<std::string>("SLAM_System") == "ORB3")
                {
                    kfPtrToUpdate->setPose(typeConversion_->se3ToAffine<Eigen::Affine3f>(kfPoseToUpdate.pose, true));
                }
                else if (parameterInstance.getValue<std::string>("SLAM_System") == "ISAE")
                {
                    kfPtrToUpdate->setPose(typeConversion_->se3ToAffine<Eigen::Affine3f>(kfPoseToUpdate.pose, false));
                }
            }
        }
    }
}