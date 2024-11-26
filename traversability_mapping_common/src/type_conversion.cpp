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

#include "traversability_mapping_common/type_conversion.hpp"
#include "sophus/se3.hpp"

namespace traversability_mapping
{
    // double TraversabilityTypeConversions::stampToSec(builtin_interfaces::msg::Time stamp)
    // {
    //     double seconds = stamp.sec + (stamp.nanosec * pow(10, -9));
    //     return seconds;
    // }

    // builtin_interfaces::msg::Time TraversabilityTypeConversions::secToStamp(double seconds)
    // {
    //     builtin_interfaces::msg::Time stamp;
    //     stamp.sec = static_cast<int32_t>(std::floor(seconds));
    //     stamp.nanosec = static_cast<uint32_t>((seconds - stamp.sec) * pow(10, 9));
    //     return stamp;
    // }

    // geometry_msgs::msg::Point TraversabilityTypeConversions::eigenToPointMsg(Eigen::Vector3f &e)
    // {
    //     geometry_msgs::msg::Point p;
    //     p.x = e.x();
    //     p.y = e.y();
    //     p.z = e.z();
    //     return p;
    // }

    // geometry_msgs::msg::Quaternion TraversabilityTypeConversions::eigenToQuaternionMsg(Eigen::Quaternionf &e)
    // {
    //     geometry_msgs::msg::Quaternion q;
    //     q.w = e.w();
    //     q.x = e.x();
    //     q.y = e.y();
    //     q.z = e.z();
    //     return q;
    // }

    Eigen::Affine3f TraversabilityTypeConversions::se3ORBToROS(const Sophus::SE3f &s)
    {
        Eigen::Matrix3f tfCameraRotation = s.rotationMatrix();
        Eigen::Vector3f tfCameraTranslation = s.translation();

        Eigen::Matrix3f tfORBToROS;
        tfORBToROS << 0, 0, 1,
            -1, 0, 0,
            0, -1, 0;

        // Transform from orb coordinate system to ros coordinate system on camera coordinates
        Eigen::Matrix3f tfCameraRotationTemp = tfORBToROS * tfCameraRotation;
        Eigen::Vector3f tfCameraTranslationTemp = tfORBToROS * tfCameraTranslation;

        // Inverse matrix (Tcw -> Twc, converts to map frame of ROS coordinates)
        Eigen::Matrix3f tfCameraRotationInv = tfCameraRotationTemp.transpose();
        Eigen::Vector3f tfCameraTranslationInv = -(tfCameraRotationInv * tfCameraTranslationTemp);

        // Transform from orb coordinate system to ros coordinate system on map coordinates
        tfCameraRotation = tfORBToROS * tfCameraRotationInv;
        tfCameraTranslation = tfORBToROS * tfCameraTranslationInv;

        Eigen::Affine3f affineMatrix = Eigen::Affine3f::Identity();
        affineMatrix.rotate(tfCameraRotation);
        affineMatrix.translation() = tfCameraTranslation;

        return affineMatrix;
    }

    Eigen::Vector3f TraversabilityTypeConversions::vector3fORBToROS(const Eigen::Vector3f &s)
    {
        Eigen::Matrix3f tfCameraRotation = Eigen::Matrix3f::Identity();
        Eigen::Vector3f tfCameraTranslation = s;

        // Coordinate transformation matrix from orb coordinate system to ros coordinate system
        Eigen::Matrix3f tfORBToROS;
        tfORBToROS << 0, 0, 1,
            -1, 0, 0,
            0, -1, 0;

        // Transform from orb coordinate system to ros coordinate system on camera coordinates
        Eigen::Matrix3f tfCameraRotationTemp = tfORBToROS * tfCameraRotation;
        Eigen::Vector3f tfCameraTranslationTemp = tfORBToROS * tfCameraTranslation;

        tfCameraRotation = tfCameraRotationTemp;
        tfCameraTranslation = tfCameraTranslationTemp;

        return tfCameraTranslation;
    }

    template <>
    Eigen::Affine3d TraversabilityTypeConversions::se3ToAffine(const Sophus::SE3f &s, bool ORBSLAM)
    {
        if (ORBSLAM)
        {
            Eigen::Affine3d affineTf = se3ORBToROS(s).cast<double>();
            return affineTf;
        }
        Eigen::Affine3d eigenSe3Pose = Eigen::Affine3d::Identity();
        eigenSe3Pose.rotate(s.rotationMatrix().cast<double>());
        eigenSe3Pose.translation() = s.translation().cast<double>();
        return eigenSe3Pose;
    }

    template <>
    Eigen::Affine3f TraversabilityTypeConversions::se3ToAffine(const Sophus::SE3f &s, bool ORBSLAM)
    {
        if (ORBSLAM)
        {
            Eigen::Affine3f affineTf = se3ORBToROS(s);
            return affineTf;
        }
        Eigen::Affine3f eigenSe3Pose = Eigen::Affine3f::Identity();
        eigenSe3Pose.rotate(s.rotationMatrix());
        eigenSe3Pose.translation() = s.translation();
        return eigenSe3Pose;
    }

    // geometry_msgs::msg::Pose TraversabilityTypeConversions::se3ToPoseMsg(const Sophus::SE3f &s)
    // {
    //     Eigen::Affine3d poseTransform = se3ORBToROS(s).cast<double>();
    //     geometry_msgs::msg::Pose pose = tf2::toMsg(poseTransform);
    //     return pose;
    // }

    // sensor_msgs::msg::PointCloud2 TraversabilityTypeConversions::MapPointsToPCL(std::vector<Eigen::Vector3f>& mapPoints)
    // {
    //     const int numChannels = 3; // x y z

    //     if (mapPoints.size() == 0)
    //     {
    //         std::cout << "Map point vector is empty!" << std::endl;
    //     }

    //     sensor_msgs::msg::PointCloud2 cloud;

    //     // cloud.header.stamp = current_frame_time;
    //     cloud.header.frame_id = "map";
    //     cloud.height = 1;
    //     cloud.width = mapPoints.size();
    //     cloud.is_bigendian = false;
    //     cloud.is_dense = true;
    //     cloud.point_step = numChannels * sizeof(float);
    //     cloud.row_step = cloud.point_step * cloud.width;
    //     cloud.fields.resize(numChannels);

    //     std::string channel_id[] = {"x", "y", "z"};

    //     for (int i = 0; i < numChannels; i++)
    //     {
    //         cloud.fields[i].name = channel_id[i];
    //         cloud.fields[i].offset = i * sizeof(float);
    //         cloud.fields[i].count = 1;
    //         cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    //     }

    //     cloud.data.resize(cloud.row_step * cloud.height);

    //     unsigned char *cloud_data_ptr = &(cloud.data[0]);

    //     for (unsigned int i = 0; i < cloud.width; i++)
    //     {
    //         Eigen::Vector3f point_translation = mapPoints[i];

    //         float data_array[numChannels] = {
    //             point_translation.x(), point_translation.y(), point_translation.z()};

    //         memcpy(cloud_data_ptr + (i * cloud.point_step), data_array,
    //                 numChannels * sizeof(float));
    //     }
    //     return cloud;
    // }

    // template <>
    // geometry_msgs::msg::Pose TraversabilityTypeConversions::transformPoseWithReference(Eigen::Affine3d &affineMapToRef, Sophus::SE3f &transform)
    // {
    //     // Convert SE3 to affine.
    //     auto affine_map_to_pose = affineMapToRef * se3ToAffine<Eigen::Affine3d>(transform);
    //     return tf2::toMsg(affine_map_to_pose);
    // }

    // template <>
    // Eigen::Affine3d TraversabilityTypeConversions::transformPoseWithReference(Eigen::Affine3d &affineMapToRef, Sophus::SE3f &transform)
    // {
    //     // Convert SE3 to affine.
    //     auto affineMapToPose = affineMapToRef * se3ToAffine<Eigen::Affine3d>(transform);
    //     return affineMapToPose;
    // }

    // template <>
    // geometry_msgs::msg::Point TraversabilityTypeConversions::transformPointWithReference(Eigen::Affine3d &affineMapToRef, Eigen::Vector3f &point)
    // {
    //     auto affine_map_to_pose = affineMapToRef.cast<float>() * point;
    //     return eigenToPointMsg(affine_map_to_pose);
    // }

    // template <>
    // Eigen::Vector3f TraversabilityTypeConversions::transformPointWithReference(Eigen::Affine3d &affineMapToRef, Eigen::Vector3f &point)
    // {
    //     auto affine_map_to_pose = affineMapToRef.cast<float>() * point;
    //     return affine_map_to_pose;
    // }
}