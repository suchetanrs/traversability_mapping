uint64 kf_timestamp_in_nanosec # convert your seconds to nanoseconds.
geometry_msgs/Pose kf_pose # in map frame
sensor_msgs/PointCloud2 kf_pointcloud # in sensor frame. Fill this field only if you are adding a new keyframe. Not needed for pose updates.