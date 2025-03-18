#!/usr/bin/env python
import rospy
import tf
import math
from senseglove_shared_resources.msg import SenseGloveState  # Adjust import as necessary

br = tf.TransformBroadcaster()

def state_callback(msg):
    # Extract the IMU quaternion from the message
    q = [msg.imu_orientation.x, msg.imu_orientation.y, msg.imu_orientation.z, msg.imu_orientation.w]
    
    # Normalize the quaternion manually
    norm = math.sqrt(q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)
    if norm == 0:
        rospy.logwarn("Received a zero-length quaternion; cannot normalize.")
        return
    q_norm = [val / norm for val in q]
    
    # Define translation (adjust these values to match your URDF)
    translation = (0.5, 0.0, 0.5)
    
    # Broadcast the transform from "world" to "r_glove_hub"
    br.sendTransform(
        translation,
        tuple(q_norm),
        rospy.Time.now(),
        "r_glove_hub",
        "world"
    )

def main():
    rospy.init_node('imu_tf_broadcaster')
    rospy.Subscriber("/senseglove/0/rh/senseglove_states", SenseGloveState, state_callback)
    rospy.loginfo("SenseGlove TF broadcaster node started.")
    rospy.spin()

if __name__ == '__main__':
    main()
