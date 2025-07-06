#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, quaternion_multiply

from geometry_msgs.msg import TransformStamped
from senseglove_msgs.msg import SenseGloveState


class ImuTfBroadcaster(Node):
    def __init__(self):
        super().__init__('imu_tf_broadcaster')

        # Declare parameters, with defaults
        self.declare_parameter('hand', 'rh')
        self.declare_parameter('translation', [0.5, 0.0, 0.5])

        hand = self.get_parameter('hand').get_parameter_value().string_value.lower()
        if hand == 'rh':
            topic = '/senseglove/sg0/rh/senseglove_states'
            self.child_frame = 'r_glove_hub'
        elif hand == 'lh':
            topic = '/senseglove/sg0/lh/senseglove_states'
            self.child_frame = 'l_glove_hub'
        else:
            self.get_logger().error(f"Invalid hand parameter: {hand}, must be 'rh' or 'lh'")
            rclpy.shutdown()
            return

        # Read translation once
        self.translation = self.get_parameter('translation').get_parameter_value().double_array_value

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber
        self.subscription = self.create_subscription(
            SenseGloveState,
            topic,
            self.state_callback,
            10
        )
        self.get_logger().info(f"Started IMU TF broadcaster on {topic} (child frame: {self.child_frame})")

    # TODO: Still needs to be correct
    def correct_imu_orientation(self, q):
        # Normalize
        q = np.array(q, dtype=float)
        norm = np.linalg.norm(q)
        if norm == 0:
            self.get_logger().warn("Zero-length quaternion received; using identity")
            return [0.0, 0.0, 0.0, 1.0]
        q_in = q / norm

        # Unity to ROS
        unity_to_ros_q = [q_in[2], -q_in[1], q_in[0], q_in[3]]

        # Apply a 90° rotation about Z
        q_corr = quaternion_from_euler(0.0, 0.0, math.pi / 2.0)
        q_ros = quaternion_multiply(q_corr, unity_to_ros_q)

        return q_ros

    def state_callback(self, msg: SenseGloveState):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.child_frame

        tx, ty, tz = self.translation
        t.transform.translation.x = float(tx)
        t.transform.translation.y = float(ty)
        t.transform.translation.z = float(tz)

        q = [
            msg.imu_orientation.x,
            msg.imu_orientation.y,
            msg.imu_orientation.z,
            msg.imu_orientation.w
        ]
        q_ros = self.correct_imu_orientation(q)
        t.transform.rotation.x = float(q_ros[0])
        t.transform.rotation.y = float(q_ros[1])
        t.transform.rotation.z = float(q_ros[2])
        t.transform.rotation.w = float(q_ros[3])

        # Send Transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ImuTfBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()