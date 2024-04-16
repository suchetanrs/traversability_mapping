uint64 kf_timestamp_in_nanosec # convert your seconds to nanoseconds.
uint64 kf_id
geometry_msgs/Pose kf_pose # in map frame
sensor_msgs/PointCloud2 kf_pointcloud # in sensor frame. Fill this field only if you are adding a new keyframe. Not needed for pose updates.
uint64 map_id # set it to 0 in case your SLAM is just odometry or a single map SLAM.