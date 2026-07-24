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
}