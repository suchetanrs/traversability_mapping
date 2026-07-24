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

#ifndef KEYFRAME_HPP_
#define KEYFRAME_HPP_


#include <cstdint>
#include <unordered_map>
#include <vector>
#include <mutex>
#include <Eigen/Geometry>

#include "traversability_mapping/Moments.hpp"

namespace traversability_mapping
{
    /**
     * @brief A single keyframe: its pose, its retained base-frame cloud, and its
     *        per-cell moment contribution to the global grid.
     *
     * The partition is not frozen: on a PGO pose correction the stored base-frame cloud
     * is re-transformed and re-binned from scratch via rebin(). The owner (System /
     * LocalMap) performs all grid arithmetic; the keyframe never touches the grid and is
     * kept PCL-free (Eigen only) so it can be unit-tested in isolation.
     */
    class KeyFrame
    {
    public:
        /// @brief Primary constructor.
        /// @param kfID unique keyframe id. @param timestamp acquisition time (s).
        /// @param Tm_base initial pose (map <- base_footprint).
        /// @param cloudBase already-pruned base-frame cloud, moved in and retained for re-binning.
        /// @param parentMapID map this keyframe currently belongs to.
        KeyFrame(std::uint64_t kfID, double timestamp, const Eigen::Affine3f &Tm_base,
                 std::vector<Eigen::Vector3f> &&cloudBase, std::uint64_t parentMapID = 0);

        /// @brief Convenience overload (timestamp = 0, parentMapID = 0); prefer the primary ctor.
        /// @param kfID unique keyframe id. @param Tm_base initial pose. @param cloudBase base-frame cloud.
        KeyFrame(std::uint64_t kfID, const Eigen::Affine3f &Tm_base,
                 std::vector<Eigen::Vector3f> &&cloudBase);

        /// @brief Unique keyframe id.
        const std::uint64_t &getKfID() const { return kfID_; }
        /// @brief Acquisition time (seconds).
        const double &getTimestamp() const { return timestamp_; }

        /// @brief Record a new parent map id (moving partials between grids is the caller's job).
        /// @param parentMapID new owning map id.
        void setMap(std::uint64_t parentMapID) { parentMapID_ = parentMapID; }

        /// @brief Store the newest pose and flag the keyframe as needing a rebin.
        /// @param p new pose (map <- base_footprint). Non-blocking, thread-safe against getPendingPose.
        void setPose(const Eigen::Affine3f &p);

        /// @brief Consume the pending pose, if any.
        /// @param out [out] receives the pending pose when true.
        /// @return true if a pose was pending (flag cleared); false otherwise.
        bool getPendingPose(Eigen::Affine3f &out);

        /// @brief Latest known pose (map <- base_footprint).
        /// @return a non-blocking copy of the pose.
        Eigen::Affine3f getPose() const;

        /// @brief Re-bin the base-frame cloud at @p pose into cell-local moments, replacing the partials.
        /// @param lattice cell lattice to bin onto. @param pose pose to transform the cloud by.
        void rebin(const Lattice &lattice, const Eigen::Affine3f &pose);

        /// @brief Mutable per-cell moments (cellId -> cell-local moments).
        std::unordered_map<std::uint64_t, NodeMetaData> &partials() { return partials_; }
        /// @brief Read-only per-cell moments.
        const std::unordered_map<std::uint64_t, NodeMetaData> &partials() const { return partials_; }

        /// @brief Retained pruned cloud in the robot base frame (empty once dropped).
        const std::vector<Eigen::Vector3f> &cloudBase() const { return cloudBase_; }

        /// @brief Whether the base-frame cloud is still retained (false disables re-binning).
        bool hasCloud() const { return !cloudBase_.empty(); }
        /// @brief Drop the retained cloud to reclaim memory (the keyframe can no longer be re-binned).
        void dropCloud() { cloudBase_.clear(); cloudBase_.shrink_to_fit(); }

    private:
        /// @name Set once in the constructor; read-only afterwards (no locking needed).
        /// @{
        std::uint64_t kfID_;
        double timestamp_ = 0.0;
        std::uint64_t parentMapID_ = 0;
        std::vector<Eigen::Vector3f> cloudBase_;                     ///< pruned cloud, base frame
        std::unordered_map<std::uint64_t, NodeMetaData> partials_;   ///< cellId -> cell-local moments
        /// @}

        /// @name Pose state machine, guarded by poseMutex_.
        /// latestPose_ and hasPending_ are written together in setPose and consumed
        /// together in getPendingPose; "has a pending pose" IS the "needs rebin" signal.
        /// @{
        mutable std::mutex poseMutex_;
        Eigen::Affine3f latestPose_;                                 ///< map <- base_footprint
        bool hasPending_ = false;                                    ///< latestPose_ awaits a rebin
        /// @}
    };
}  // namespace traversability_mapping

#endif  // KEYFRAME_HPP_
