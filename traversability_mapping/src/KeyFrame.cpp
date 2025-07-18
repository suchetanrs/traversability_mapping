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
#include "traversability_mapping/KeyFrame.hpp"

namespace traversability_mapping
{
    KeyFrame::KeyFrame(double timestamp,
                       long unsigned int kfID,
                       pcl::PointCloud<pcl::PointXYZ> &pointCloud,
                       std::shared_ptr<grid_map::GridMap> gridMap,
                       std::shared_ptr<std::mutex> masterGridMapMutex,
                       long unsigned int mapID,
                       Eigen::Affine3f Tsv,
                       Eigen::Affine3f Tbs)
        : timestamp_(timestamp),
          kfID_(kfID),
          pointCloudLidar_(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(pointCloud)),
          pGridMap_(gridMap),
          masterGridMapMutex_(masterGridMapMutex),
          parentMapID_(mapID),
          Tsv_(Tsv),
          Tsb_(Tbs.inverse())
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
        // Tsv_ = translation * quaternion;
        numConnections_ = 0;
    }

    KeyFrame::KeyFrame(long unsigned int kfID,
                       std::shared_ptr<grid_map::GridMap> gridMap,
                       std::shared_ptr<std::mutex> masterGridMapMutex,
                       Eigen::Affine3f Tsv,
                       Eigen::Affine3f Tbs)
        : kfID_(kfID),
          pGridMap_(gridMap),
          masterGridMapMutex_(masterGridMapMutex),
          Tsv_(Tsv),
          Tsb_(Tbs.inverse())
    {
        numConnections_ = 0;
    }

    KeyFrame::~KeyFrame()
    {
        std::cout << "keyframe with Id: " << kfID_ << " destructed" << std::endl;
        // clearStrayValuesInGrid();
    }

    const double &KeyFrame::getTimestamp()
    {
        return timestamp_;
    }

    const long unsigned int &KeyFrame::getKfID()
    {
        return kfID_;
    }

    // const std::shared_ptr<sensor_msgs::msg::PointCloud2> &KeyFrame::getPointCloud()
    // {
    //     return pointCloudMap_;
    // }

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

    const long unsigned int &KeyFrame::getConnections()
    {
        std::lock_guard<std::mutex> lock(connectionMutex_);
        return numConnections_;
    }

    void KeyFrame::setPose(const Eigen::Affine3f &pose)
    {
        std::lock_guard<std::mutex> lock(poseMutex_);
        bool newPose = false;
        if (!poseTraversabilityCoord_)
        {
            poseTraversabilityCoord_ = std::make_unique<Eigen::Affine3f>(pose);
            newPose = true;
        }
        else
        {
            // // Measure if the pose has significantly changed wrt the the thresholds.
            // Eigen::Vector3f eulerAnglesDiff = (poseTraversabilityCoord_->rotation() * pose.rotation().transpose()).eulerAngles(2, 0, 1);
            // Eigen::Vector3f translationDiff = poseTraversabilityCoord_->translation() - pose.translation();
            // // std::cout << "Max angle:" << eulerAnglesDiff.maxCoeff() << std::endl;
            // // std::cout << "Max adist:" << translationDiff.maxCoeff() << std::endl;
            // if (eulerAnglesDiff.maxCoeff() > parameterInstance.getValue<double>("rotation_change_threshold") ||
            //     translationDiff.maxCoeff() > parameterInstance.getValue<double>("translation_change_threshold"))
            // {
            // }

            *poseTraversabilityCoord_ = pose;
            newPose = true;
        }
        if (newPose)
        {
            poseUpdateQueueMutex_.lock();
            poseUpdates_.push_back(pose);
            poseUpdateQueueMutex_.unlock();
        }
    }

    void KeyFrame::setSLAMPose(const Sophus::SE3f &pose)
    {
        try
        {
            std::lock_guard<std::mutex> lock(poseMutex_);
            if (!poseSLAMCoord_)
                poseSLAMCoord_ = std::make_unique<Sophus::SE3f>(pose);
            else
                *poseSLAMCoord_ = pose;
        }
        catch (const std::exception &e)
        {
            throw std::runtime_error("Code should crash now");
        }
    }

    void KeyFrame::setMap(long unsigned int mapID, std::shared_ptr<grid_map::GridMap> gridMap, std::shared_ptr<std::mutex> masterGridMapMutex)
    {
        std::lock_guard<std::mutex> lock(gridMapMutex_);
        masterGridMapMutex_ = masterGridMapMutex;
        parentMapID_ = mapID;
        pGridMap_ = gridMap;
    }

    void KeyFrame::setConnections(long unsigned int numConnections)
    {
        std::lock_guard<std::mutex> lock(connectionMutex_);
        numConnections_ = numConnections;
    }

    void KeyFrame::computeLocalTraversability(pcl::PointCloud<pcl::PointXYZ> &kFpcl, Eigen::Affine3f &traversabilityPose)
    {
        double resolution_ = parameterInstance.getValue<double>("resolution_local_map");
        double half_size_traversability_ = parameterInstance.getValue<double>("half_size_traversability");
        const double security_distance_ = parameterInstance.getValue<double>("security_distance");
        const double ground_clearance_ = parameterInstance.getValue<double>("ground_clearance");
        const double max_slope_ = parameterInstance.getValue<double>("max_slope");
        double robot_height_ = parameterInstance.getValue<double>("robot_height");
        Eigen::Vector2d traversabilityPose2D;
        traversabilityPose2D.x() = traversabilityPose.translation().x();
        traversabilityPose2D.y() = traversabilityPose.translation().y();
        auto traversabilityMap = std::make_shared<traversabilityGrid>(resolution_, Eigen::Vector2d(half_size_traversability_, half_size_traversability_), traversabilityPose2D);
        // Fill traversability structure
        traversabilityMap->reset();
        auto T_robot_to_map = traversabilityPose.inverse();
        Eigen::Vector3f pRobot;
        Eigen::Vector3f pGlobal;
        for (const auto &point : kFpcl)
        {
            pGlobal = {point.x, point.y, point.z};
            pRobot = T_robot_to_map * pGlobal;
            if ((point.x == 0 && point.y == 0) || pRobot.z() > robot_height_)
                continue;

            traversabilityMap->insert_data(point.x, point.y, point.z, pRobot.x(), pRobot.y(), pRobot.z());
        }
        // Publish as grid map
        // Create grid map.
        // map.move(traversabilityPose2D);
        // map.setPosition(traversabilityPose2D);
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
        std::lock_guard<std::mutex> lock(gridMapMutex_);
        std::lock_guard<std::mutex> lock2(*masterGridMapMutex_);
        for (size_t i = 0; i < travGrid.size(); ++i)
        {
            for (size_t j = 0; j < travGrid[i].size(); ++j)
            {
                float mx, my;
                traversabilityMap->ind2meterOpt(i, j, mx, my);
                Eigen::Vector2d meterValue(mx, my);
                Eigen::Vector4d haz = traversabilityMap->get_goodness(
                    i, j, security_distance_, ground_clearance_, max_slope_);
                ++grid_count;
                if (haz(0) < 0.)
                    continue;
                if (parameterInstance.getValue<bool>("use_averaging"))
                {
                    auto num_additions = 0.0;
                    if (std::isnan(pGridMap_->atPosition("num_additions", meterValue)))
                    {
                        pGridMap_->atPosition("num_additions", meterValue) = 0.0;
                        pGridMap_->atPosition("hazard", meterValue) = 0.0;
                    }
                    num_additions = pGridMap_->atPosition("num_additions", meterValue);
                    if (haz(0) > pGridMap_->atPosition("hazard", meterValue))
                    {
                        if (num_additions > parameterInstance.getValue<double>("average_persistence"))
                        {
                            pGridMap_->atPosition("num_additions", meterValue) = 1.0;
                            num_additions = 1.0;
                            pGridMap_->atPosition("hazard", meterValue) = haz(0);
                        }
                        pGridMap_->atPosition("num_additions", meterValue) += 1.0;
                        pGridMap_->atPosition("hazard", meterValue) = ((pGridMap_->atPosition("hazard", meterValue) * num_additions) + haz(0)) / (num_additions + 1.0);
                    }
                }
                else if (parameterInstance.getValue<bool>("use_probabilistic_update"))
                {
                    // in this case, num additions becomes the log odds.
                    // TODO: rename the key in the gridmap.
                    if (haz(0) == 0)
                        haz(0) = 0.1;
                    if (haz(0) == 1)
                        haz(0) = 0.9;
                    double updated_probability = updateCellLogOdds(pGridMap_->atPosition("num_additions", meterValue), haz(0));
                    pGridMap_->atPosition("hazard", meterValue) = updated_probability;
                }
                else
                {
                    pGridMap_->atPosition("hazard", meterValue) = haz(0);
                }
                if (parameterInstance.getValue<bool>("use_virtual_boundary"))
                    pGridMap_->atPosition("hazard", meterValue) = std::max(pGridMap_->atPosition("virtual_boundary", meterValue), pGridMap_->atPosition("hazard", meterValue));
                pGridMap_->atPosition("step_haz", meterValue) = haz(1);
                // original
                // pGridMap_->atPosition("roughness_haz", meterValue) = haz(2);
                // made now for visualization
                pGridMap_->atPosition("elevation", meterValue) = haz(2);
                pGridMap_->atPosition("slope_haz", meterValue) = haz(3);
                // the latest updating kf's id is stored in this position of the gridmap.
                pGridMap_->atPosition("kfid", meterValue) = static_cast<float>(kfID_);
                markedCells_.push_back(meterValue);
            }
        }
        // std::cout << "Grid count: " << grid_count << " KF ID: " << kfID_;
        // map.setPosition(traversabilityPose2D);
    }

    void KeyFrame::recomputeCache(bool useHashGrid)
    {
        poseUpdateQueueMutex_.lock();
        if (poseUpdates_.size() == 0)
        {
            // std::cout << "Pose queue empty!!! for kf id: " << kfID_ << std::endl;
            poseUpdateQueueMutex_.unlock();
            return;
        }
        auto Tms_ = poseUpdates_[poseUpdates_.size() - 1];
        poseUpdates_.clear();
        poseUpdateQueueMutex_.unlock();

        if (!spatialHashGridInstance_.updateKeyframe(kfID_, Tms_, getConnections()))
        {
            if (useHashGrid)
            {
                if (!parameterInstance.getValue<bool>("is_kf_optimization_enabled"))
                {
                    pointCloudLidar_.reset();
                }
                return;
            }
        }

        auto start_time = std::chrono::high_resolution_clock::now();
        // std::cout << "Recompute cache for KF with ID: " << kfID_ << std::endl;
        // Transform the pointCloudLidar_ to map frame.
        // transform from map frame to slam frame * transform from slam frame to velodyne (lidar) frame.
        auto Tmv = Tms_ * Tsv_;
        auto Tmb = Tms_ * Tsb_;
        pcl::PointCloud<pcl::PointXYZ> pointCloudCorrected_;
        // correct the pointcloud from velodyne frame to map frame.
        if (pointCloudLidar_ == nullptr)
        {
            std::cerr << "POINTCLOUD LIDAR IS NULL" << std::endl;
            return;
        }
        traversability_mapping::doTransformPCL(*pointCloudLidar_, pointCloudCorrected_, Tmv);

        // clear stray values
        // clearStrayValuesInGrid();
        // recompute values and mark on map.
        computeLocalTraversability(pointCloudCorrected_, Tmb);
        if (!parameterInstance.getValue<bool>("is_kf_optimization_enabled"))
        {
            pointCloudLidar_.reset();
        }
    }

    void KeyFrame::updateGridAfterMapChange()
    {
        clearStrayValuesInGrid();
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
        std::lock_guard<std::mutex> lock2(*masterGridMapMutex_);
        for (const auto &cell : markedCells_)
        {
            if (pGridMap_->atPosition("kfid", cell) == static_cast<float>(kfID_))
            {
                pGridMap_->atPosition("num_additions", cell) = std::numeric_limits<float>::quiet_NaN();
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