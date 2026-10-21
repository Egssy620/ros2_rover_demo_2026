# Landmark map with shared frame
# frame_id: common frame (e.g. map)
# Each pose corresponds index-wise to ids[]
std_msgs/Header header
string[] ids
geometry_msgs/Pose[] poses
