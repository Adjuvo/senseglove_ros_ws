# SenseGloveState.msg
# A message for streaming SenseGlove Hand State
std_msgs/Header header

string[] joint_names
float32[] position                         
float32[] absolute_velocity              

KinematicsVect3D[] hand_position
KinematicsVect3D[] finger_tip_positions

float32[] normalized_input
geometry_msgs/Quaternion imu_orientation
