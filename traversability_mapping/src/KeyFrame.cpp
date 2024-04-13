#include "traversability_mapping/KeyFrame.hpp"

namespace traversability_mapping
{
    KeyFrame::KeyFrame(double timestamp,
                       long unsigned int kfID,
                       sensor_msgs::msg::PointCloud2 &pointCloud,
                       std::shared_ptr<grid_map::GridMap> gridMap,
                       long unsigned int mapID,
                       Eigen::Affine3f Tbv)
        : timestamp_(timestamp),
          kfID_(kfID),
          pointCloudLidar_(pointCloud),
          pGridMap_(gridMap),
          parentMapID_(mapID),
          Tbv_(Tbv)
    {
        // TODO: Make this a parameter
        // auto translation = Eigen::Translation3f(
        //     static_cast<float>(0.527),
        //     static_cast<float>(0.0),
        //     static_cast<float>(0.662));
        // auto quaternion = Eigen::Quaternion<float>(
        //     static_cast<float>(0.924), // w
        //     static_cast<float>(0.0),   // x
        //     static_cast<float>(0.381), // y
        //     static_cast<float>(0.0));  // z
        // Tbv_ = translation * quaternion;
    }

    const double &KeyFrame::getTimestamp()
    {
        return timestamp_;
    }

    const long unsigned int &KeyFrame::getKfID()
    {
        return kfID_;
    }

    const std::shared_ptr<sensor_msgs::msg::PointCloud2> &KeyFrame::getPointCloud()
    {
        return pointCloudMap_;
    }

    const Eigen::Affine3f &KeyFrame::getPose()
    {
        std::lock_guard<std::mutex> lock(poseMutex_);
        if (!poseTraversabilityCoord_)
        {
            throw std::runtime_error("The requested pose in traversability coordinates is null. Did you set it?");
        }
        return *poseTraversabilityCoord_;
    }

    const Sophus::SE3f &KeyFrame::getSLAMPose()
    {
        std::lock_guard<std::mutex> lock(poseMutex_);
        if (!poseSLAMCoord_)
        {
            throw std::runtime_error("The requested pose in SLAM coordinates is null. Did you set it?");
        }
        return *poseSLAMCoord_;
    }

    void KeyFrame::setPose(const Eigen::Affine3f &pose)
    {
        std::lock_guard<std::mutex> lock(poseMutex_);
        if (!poseTraversabilityCoord_)
            poseTraversabilityCoord_ = std::make_unique<Eigen::Affine3f>(pose);
        else
            *poseTraversabilityCoord_ = pose;
        poseUpdateQueueMutex_.lock();
        poseUpdates_.push_back(pose);
        poseUpdateQueueMutex_.unlock();
    }

    void KeyFrame::setSLAMPose(const Sophus::SE3f &pose)
    {
        std::lock_guard<std::mutex> lock(poseMutex_);
        if (!poseSLAMCoord_)
            poseSLAMCoord_ = std::make_unique<Sophus::SE3f>(pose);
        else
            *poseSLAMCoord_ = pose;
    }

    void KeyFrame::setMap(long unsigned int mapID, std::shared_ptr<grid_map::GridMap> gridMap)
    {
        std::lock_guard<std::mutex> lock(gridMapMutex_);
        parentMapID_ = mapID;
        pGridMap_ = gridMap;
    }

    void KeyFrame::computeLocalTraversability(sensor_msgs::msg::PointCloud2 &kFpcl)
    {
        // TODO: Load from paramter file.
        double resolution_ = 0.25;
        double half_size_traversability_ = 5.5;
        const double security_distance_ = 0.6;
        const double ground_clearance_ = 0.2;
        const double max_slope_ = 0.8;
        double robot_height_ = 1.5;
        Eigen::Vector2d slamPosition;
        slamPosition.x() = poseTraversabilityCoord_->translation().x();
        slamPosition.y() = poseTraversabilityCoord_->translation().y();
        auto traversabilityMap = std::make_shared<traversabilityGrid>(resolution_, Eigen::Vector2d(half_size_traversability_, half_size_traversability_), slamPosition);
        // Fill traversability structure
        traversabilityMap->reset();
        for (sensor_msgs::PointCloud2ConstIterator<float> it(kFpcl, "x"); it != it.end(); ++it)
        {
            Eigen::Vector3d pt3(it[0], it[1], it[2]);
            if (((it[0] == 0) && (it[1] == 0)) || (it[2] > robot_height_))
                continue;
            traversabilityMap->insert_data(pt3);
        }

        // Publish as grid map
        // Create grid map.
        // map.move(slamPosition);
        // map.setPosition(slamPosition);
        int grid_count = 0;
        // for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
        // {
        //     grid_map::Position position;
        //     map.getPosition(*it, position);
        //     Eigen::Vector4d haz = traversabilityMap->get_goodness_m(
        //         Eigen::Vector2d(position.x(), position.y()),
        //         security_distance_, ground_clearance_, max_slope_);
        //     ++grid_count;
        //     if (haz(0) < 0.)
        //         continue;

        //     // map.at("hazard", *it) = haz(0);
        //     // map.at("step_haz", *it) = haz(1);
        //     // map.at("roughness_haz", *it) = haz(2);
        //     // map.at("slope_haz", *it) = haz(3);
        //     map.atPosition("hazard", position) = haz(0);
        //     // std::cout << "Hazard value is: " << haz(0) << std::endl;
        //     // map.at("border_haz", *it) = haz(4);
        //     // map.at("elevation", *it) = haz(5);
        // }
        auto travGrid = traversabilityMap->getGrid();
        for (size_t i = 0; i < travGrid.size(); ++i)
        {
            for (size_t j = 0; j < travGrid[i].size(); ++j)
            {
                Eigen::Vector4d haz = traversabilityMap->get_goodness(
                    Eigen::Vector2d(i, j),
                    security_distance_, ground_clearance_, max_slope_);
                ++grid_count;
                if (haz(0) < 0.)
                    continue;
                std::lock_guard<std::mutex> lock(gridMapMutex_);
                pGridMap_->atPosition("hazard", traversabilityMap->ind2meter(Eigen::Vector2d(i, j))) = haz(0);
                pGridMap_->atPosition("step_haz", traversabilityMap->ind2meter(Eigen::Vector2d(i, j))) = haz(1);
                // original
                // pGridMap_->atPosition("roughness_haz", traversabilityMap->ind2meter(Eigen::Vector2d(i, j))) = haz(2);
                // made now for visualization
                pGridMap_->atPosition("elevation", traversabilityMap->ind2meter(Eigen::Vector2d(i, j))) = haz(2);
                pGridMap_->atPosition("slope_haz", traversabilityMap->ind2meter(Eigen::Vector2d(i, j))) = haz(3);
                // the latest updating kf's id is stored in this position of the gridmap.
                pGridMap_->atPosition("kfid", traversabilityMap->ind2meter(Eigen::Vector2d(i, j))) = static_cast<float>(kfID_);
                markedCells_.push_back(traversabilityMap->ind2meter(Eigen::Vector2d(i, j)));
            }
        }
        // std::cout << "Grid count: " << grid_count << " KF ID: " << kfID_;
        // map.setPosition(slamPosition);
    }

    void KeyFrame::recomputeCache()
    {
        poseUpdateQueueMutex_.lock();
        if (poseUpdates_.size() == 0)
        {
            // std::cout << "Pose queue empty!!! for kf id: " << kfID_ << std::endl;
            poseUpdateQueueMutex_.unlock();
            return;
        }
        auto Tmb_ = poseUpdates_[poseUpdates_.size() - 1];
        poseUpdates_.clear();
        poseUpdateQueueMutex_.unlock();
        auto start_time = std::chrono::high_resolution_clock::now();
        // std::cout << "Recompute cache for KF with ID: " << kfID_ << std::endl;
        // Transform the pointCloudLidar_ to map frame.
        auto Tmv_ = Tmb_ * Tbv_;
        sensor_msgs::msg::PointCloud2 pointCloudCorrected_;
        traversability_mapping::doTransformPCL(pointCloudLidar_, pointCloudCorrected_, Tmv_);
        pointCloudMap_ = std::make_shared<sensor_msgs::msg::PointCloud2>(pointCloudCorrected_);
        pointCloudMap_->header.frame_id = "map";

        // clear stray values
        clearStrayValuesInGrid();
        // recompute values and mark on map.
        computeLocalTraversability(*pointCloudMap_);
    }

    void KeyFrame::updateGridAfterMapChange()
    {
        poseUpdateQueueMutex_.lock();
        if (poseUpdates_.size() == 0)
        {
            poseUpdates_.push_back(getPose());
        }
        poseUpdateQueueMutex_.unlock();
    }

    void KeyFrame::clearStrayValuesInGrid()
    {
        std::lock_guard<std::mutex> lock(gridMapMutex_);
        for (const auto &cell : markedCells_)
        {
            if (pGridMap_->atPosition("kfid", cell) == static_cast<float>(kfID_))
            {
                pGridMap_->atPosition("hazard", cell) = std::numeric_limits<float>::quiet_NaN();
                pGridMap_->atPosition("step_haz", cell) = std::numeric_limits<float>::quiet_NaN();
                pGridMap_->atPosition("roughness_haz", cell) = std::numeric_limits<float>::quiet_NaN();
                pGridMap_->atPosition("slope_haz", cell) = std::numeric_limits<float>::quiet_NaN();
                pGridMap_->atPosition("elevation", cell) = std::numeric_limits<float>::quiet_NaN();

                pGridMap_->atPosition("kfid", cell) = std::numeric_limits<float>::quiet_NaN();
            }
        }
        markedCells_.clear();
    }
}