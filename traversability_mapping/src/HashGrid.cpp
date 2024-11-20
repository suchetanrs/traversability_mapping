#include "traversability_mapping/HashGrid.hpp"

std::unique_ptr<SpatialHash> SpatialHash::spatialHashGridPtr_ = nullptr;
std::mutex SpatialHash::instanceMutex_;

// Constructor
SpatialHash::SpatialHash()
{
    
}

// Update an existing keyframe
bool SpatialHash::updateKeyframe(long unsigned int kfID, Eigen::Affine3f pose, long unsigned int numConnections) {
    std::lock_guard<std::mutex> lock(bucketsMutex_);
    auto translation = pose.translation();
    auto eulerAngles = pose.rotation().eulerAngles(2, 1, 0);
    Bucket newBucket = {
        std::floor(translation.x()),
        std::floor(translation.y()),
        std::floor(translation.z()),
        std::floor(eulerAngles[0]),
        std::floor(eulerAngles[1]),
        std::floor(eulerAngles[2])
    };

    if(buckets.find(newBucket) == buckets.end()) {
        buckets[newBucket] = std::vector<BucketEntity>();
    }
    
    if(keyframeToBucket.find(kfID) == keyframeToBucket.end()) {
        keyframeToBucket[kfID] = newBucket;
        buckets[newBucket].push_back(BucketEntity(kfID, numConnections));
    }
    else
    {
        // remove it from the old bucket
        auto oldBucket = keyframeToBucket[kfID];
        buckets[oldBucket].erase(
                std::remove_if(buckets[oldBucket].begin(), buckets[oldBucket].end(),
                    [kfID](const BucketEntity& entity) { return entity.kfID == kfID; }),
                buckets[oldBucket].end());

        // add it to the new bucket
        keyframeToBucket[kfID] = newBucket;
        buckets[newBucket].push_back(BucketEntity(kfID, numConnections));
    }


    // // Update the active keyframe for the new bucket
    auto activeKF = updateActiveKeyframe(newBucket);
    if(kfID == activeKF)
        return true;
    else
        return false;
}

// Delete a keyframe
void SpatialHash::deleteKeyframe(long unsigned int kfID) {
    std::lock_guard<std::mutex> lock(bucketsMutex_);
    auto oldBucket = keyframeToBucket[kfID];
    buckets[oldBucket].erase(
            std::remove_if(buckets[oldBucket].begin(), buckets[oldBucket].end(),
                [kfID](const BucketEntity& entity) { return entity.kfID == kfID; }),
            buckets[oldBucket].end());
    keyframeToBucket.erase(kfID);
}

// Update the active keyframe in a bucket
long unsigned int SpatialHash::updateActiveKeyframe(const Bucket &bucket) {
    auto &keyframeList = buckets[bucket];

    if (keyframeList.empty()) {
        throw std::runtime_error("No keyframes in the bucket. How did this happen?");
    }

    // Find the keyframe with the highest weight
    auto maxWeightIt = std::max_element(keyframeList.begin(), keyframeList.end(),
                                        [](const BucketEntity &a, const BucketEntity &b) {
                                            return a.numConnections < b.numConnections;
                                        });
    
    return maxWeightIt->kfID;
}