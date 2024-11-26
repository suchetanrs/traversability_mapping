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

#ifndef TRAV_WS_TYPE_CONVERSIONS_HPP_
#define TRAV_WS_TYPE_CONVERSIONS_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"
#include <iostream>
#ifdef WITH_ROS2_SENSOR_MSGS
#include <sensor_msgs/msg/point_cloud2.hpp>
#endif
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

namespace traversability_mapping
{
    class TraversabilityTypeConversions
    {
    public:
        // **************************************DATA TYPE CONVERSIONS*************************************
        /**
         * @brief Converts a ROS timestamp to seconds.
         * @param stamp The ROS timestamp to convert.
         * @return The corresponding time in seconds.
         */
        // double stampToSec(builtin_interfaces::msg::Time stamp);

        /**
         * @brief Converts seconds to a ROS timestamp.
         * @param seconds The time in seconds.
         * @return The corresponding ROS timestamp.
         */
        // builtin_interfaces::msg::Time secToStamp(double seconds);

        /**
         * @brief Converts an Eigen vector to a geometry_msgs Point message.
         * @param e The Eigen vector to convert.
         * @return The corresponding Point message.
         */
        // geometry_msgs::msg::Point eigenToPointMsg(Eigen::Vector3f &e);

        /**
         * @brief Converts an Eigen quaternion to a geometry_msgs Quaternion message.
         * @param e The Eigen quaternion to convert.
         * @return The corresponding Quaternion message.
         */
        // geometry_msgs::msg::Quaternion eigenToQuaternionMsg(Eigen::Quaternionf &e);

        /**
         * @note Essentially, this converts the ORB coordinates pose of the world frame in the camera frame (Tcw) 
         * to camera frame in world frame of ROS coordinates.
         * @brief Converts a Sophus SE3f transform to a Affine Transform.
         * @param s The Sophus SE3f transform.
         * @return The corresponding Affine Transform in world frame of ROS coordinates.
         * @attention Use only with Tcw. Will not work with Twc.
         */
        Eigen::Affine3f se3ORBToROS(const Sophus::SE3f &s);

        /**
         * @brief This function converts a vector in the world frame of ORB coordinates 
         * to a vector in the world frame of ROS coordinates.
         * @param s The Vector transform.
         * @return The corresponding vector after transform.
         */
        Eigen::Vector3f vector3fORBToROS(const Eigen::Vector3f &s);

        /**
         * @brief Converts a Sophus SE3f transform to an Eigen Affine3d transform.
         * @param s The Sophus SE3f transform.
         * @return The corresponding Eigen Affine3d transform.
         */
        template <typename T>
        T se3ToAffine(const Sophus::SE3f &s, bool ORBSLAM);

        /**
         * @brief Converts a Sophus SE3f transform to a geometry_msgs Pose message.
         * @param s The Sophus SE3f transform.
         * @return The corresponding Pose message.
         */
        // geometry_msgs::msg::Pose se3ToPoseMsg(const Sophus::SE3f &s);

        // sensor_msgs::msg::PointCloud2 MapPointsToPCL(std::vector<Eigen::Vector3f>& mapPoints);

        // **************************************TRANSFORMATIONS*************************************
        /**
         * @brief Transforms a pose using a reference pose and SE3 transform.
         * @param referencePose Reference pose.
         * @param s SE3 transform.
         * @return Transformed pose.
         */
        // template <typename T>
        // T transformPoseWithReference(Eigen::Affine3d &, Sophus::SE3f &);

        /**
         * @brief Transforms a pose using a reference pose and SE3 transform.
         * @param referencePose Reference pose.
         * @param s SE3 transform.
         * @return Transformed pose.
         */
        // template <typename T>
        // T transformPointWithReference(Eigen::Affine3d &referencePose, Eigen::Vector3f &s);
    };

#ifdef WITH_ROS2_SENSOR_MSGS
    inline
    void toPCL(const sensor_msgs::msg::PointField &pf, pcl::PCLPointField &pcl_pf)
    {
        pcl_pf.name = pf.name;
        pcl_pf.offset = pf.offset;
        pcl_pf.datatype = pf.datatype;
        pcl_pf.count = pf.count;
    }

    inline
    void toPCL(const std::vector<sensor_msgs::msg::PointField> &pfs, std::vector<pcl::PCLPointField> &pcl_pfs)
    {
        pcl_pfs.resize(pfs.size());
        std::vector<sensor_msgs::msg::PointField>::const_iterator it = pfs.begin();
        size_t i = 0;
        for(; it != pfs.end(); ++it, ++i) {
        toPCL(*(it), pcl_pfs[i]);
        }
    }

    inline
    void copyPointCloud2MetaData(const sensor_msgs::msg::PointCloud2 &pc2, pcl::PCLPointCloud2 &pcl_pc2)
    {
        pcl_pc2.height = pc2.height;
        pcl_pc2.width = pc2.width;
        toPCL(pc2.fields, pcl_pc2.fields);
        pcl_pc2.is_bigendian = pc2.is_bigendian;
        pcl_pc2.point_step = pc2.point_step;
        pcl_pc2.row_step = pc2.row_step;
        pcl_pc2.is_dense = pc2.is_dense;
    }

    inline
    void toPCL(const sensor_msgs::msg::PointCloud2 &pc2, pcl::PCLPointCloud2 &pcl_pc2)
    {
        copyPointCloud2MetaData(pc2, pcl_pc2);
        pcl_pc2.data = pc2.data;
    }
#endif
}

#endif