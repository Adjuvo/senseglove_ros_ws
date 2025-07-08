#!/usr/bin/env python
import rospy
import tf
import math
import numpy as np
from senseglove_msgs.msg import SenseGloveState  # Adjust import as necessary

br = tf.TransformBroadcaster()

# Based on the hand parameter
child_frame = None
topic_name = None

def correct_imu_orientation(q):

    # Convert SenseGlove IMU orientation to ROS convention
    # TO-DO:  Not Entirely sure of the conversion yet, need more testing and findig a solution

    # Normalize input quaternion
    norm = np.linalg.norm(q)
    if norm == 0:
        rospy.logwarn("Received a zero-length quaternion; cannot normalize.")
        return [0, 0, 0, 1]

    # Normalize the quaternion
    q_in = np.array(q) / norm

    # (Unity → ROS transformation)
    unity_to_ros_q = np.array([q_in[2], -q_in[1], q_in[0], q_in[3]])  # Swaps X <-> Z, negates Y? Does not work as of now

    # Apply Rotation 
    q_correction = tf.transformations.quaternion_from_euler(0, 0, math.pi / 2)
    q_ros = tf.transformations.quaternion_multiply(q_correction, unity_to_ros_q)

    return q_ros

def state_callback(msg):
    # Extract IMU quaternion
    q = [msg.imu_orientation.x, msg.imu_orientation.y, msg.imu_orientation.z, msg.imu_orientation.w]
    q_ros = correct_imu_orientation(q)

    # Translation (adjust via ROS parameter)
    translation = rospy.get_param("~translation", [0.5, 0.0, 0.5])

    # Broadcast the transform from "world" to the appropriate glove hub frame
    br.sendTransform(translation, tuple(q_ros), rospy.Time.now(), child_frame, "world")

def main():
    global child_frame, topic_name
    rospy.init_node('imu_tf_broadcaster')

    # Get the "hand" parameter (default is "rh" for right-hand)
    hand = rospy.get_param("~hand", "rh").lower()
    if hand == "rh":
        topic_name = "/senseglove/sg0/rh/senseglove_states"
        child_frame = "r_glove_hub"
    elif hand == "lh":
        topic_name = "/senseglove/sg0/lh/senseglove_states"
        child_frame = "l_glove_hub"
    else:
        rospy.logerr("Invalid hand parameter: {}. Use 'rh' or 'lh'.".format(hand))
        return

    rospy.Subscriber(topic_name, SenseGloveState, state_callback)
    rospy.loginfo("SenseGlove TF broadcaster node started for hand: %s", hand)
    rospy.spin()

if __name__ == '__main__':
    main()
