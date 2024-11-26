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
#ifndef SPATIAL_HASH_HPP
#define SPATIAL_HASH_HPP

#include <tuple>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <cmath>
#include <memory>
#include <Eigen/Dense>
#include <mutex>

class SpatialHash {
public:
    struct Bucket
    {
        float x;
        float y;
        float z;
        float roll;
        float pitch;
        float yaw;

        Bucket(float x_, float y_, float z_, float roll_, float pitch_, float yaw_)
            : x(x_), y(y_), z(z_), roll(roll_), pitch(pitch_), yaw(yaw_) {}
    
        Bucket() : x(0), y(0), z(0), roll(0), pitch(0), yaw(0) {}
        
        bool operator==(const Bucket& other) const {
            return x == other.x &&
                y == other.y &&
                z == other.z &&
                roll == other.roll &&
                pitch == other.pitch &&
                yaw == other.yaw;
        }
    };

    struct BucketHash
    {
        size_t operator()(const Bucket &key) const
        {
            // Calculate hash based on some combination of member variables
            size_t hash = 0;
            hash = std::hash<float>()(key.x) ^
                    std::hash<float>()(key.y) << 1 ^
                    std::hash<float>()(key.z) << 2 ^
                    std::hash<float>()(key.roll) << 3 ^
                    std::hash<float>()(key.pitch) << 4 ^
                    std::hash<float>()(key.yaw) << 5;
            return hash;
        }
    };

    struct BucketEntity
    {
        long unsigned int kfID;
        long unsigned int numConnections;

        BucketEntity() : kfID(0), numConnections(0) {}
                
        BucketEntity(long unsigned int id, long unsigned int connections) 
            : kfID(id), numConnections(connections) {}
    };

    SpatialHash();

    static SpatialHash& getInstance()
    {
        std::lock_guard<std::mutex> lock(instanceMutex_);
        if(spatialHashGridPtr_ == nullptr)
            spatialHashGridPtr_.reset(new SpatialHash());
        return *spatialHashGridPtr_;
    }

    // Update a keyframe
    bool updateKeyframe(long unsigned int kfID, Eigen::Affine3f pose, long unsigned int numConnections);

    // Delete a keyframe
    void deleteKeyframe(long unsigned int kfID);

    // // Get a list of all active keyframes
    // std::vector<KeyFrame> getActiveKeyframes() const;

private:
    SpatialHash(const SpatialHash&) = delete;
    SpatialHash& operator=(const SpatialHash&) = delete;
    static std::unique_ptr<SpatialHash> spatialHashGridPtr_;
    static std::mutex instanceMutex_;

    std::unordered_map<Bucket, std::vector<BucketEntity>, BucketHash> buckets;
    std::unordered_map<long unsigned int, Bucket> keyframeToBucket;
    std::mutex bucketsMutex_;

    long unsigned int updateActiveKeyframe(const Bucket &bucket);
};

inline SpatialHash& spatialHashGridInstance_ = SpatialHash::getInstance();

#endif // SPATIAL_HASH_HPP
