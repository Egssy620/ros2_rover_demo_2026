# Per-keyframe landmark observations for Bundle Adjustment
# frame_id   : keyframe index (matches raw_frame_N.pcd)
# robot_pose_map : LIO-SAM pose in map frame (initial value for BA)
# landmark_ids   : detected landmark IDs in this frame
# positions_sensor : LM centroid in LiDAR/sensor frame (observation value)
# positions_map    : LM position projected to map frame (T_map * p_sensor)
int32 frame_id
geometry_msgs/Pose robot_pose_map
string[] landmark_ids
geometry_msgs/Point[] positions_sensor
geometry_msgs/Point[] positions_map
