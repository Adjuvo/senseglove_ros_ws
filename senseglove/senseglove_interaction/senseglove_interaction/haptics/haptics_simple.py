#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient as ParameterClient
from rclpy.duration import Duration as RclDuration

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class HapticsSimpleNode(Node):
    def __init__(self):
        super().__init__('senseglove_haptics_node_simple')
        self.get_logger().info('Initializing simple haptics node')

        # Node with parameters
        self.declare_parameter('controller_node', '/senseglove/sg0/rh/controller')
        controller_node = self.get_parameter('controller_node').value

        # ParameterClient
        client = ParameterClient(self, controller_node)
        got = client.wait_for_services(timeout_sec=2.0)
        if not got:
            self.get_logger().error(f'Could not contact {controller_node} parameter service, using defaults')
            self.joint_list = ['empty']
            self.publish_rate = 1.0
        else:
            # Fetch parameters
            future = client.get_parameters(['trajectory.joints', 'hand_state.publish_rate'])
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            result = future.result()
            if result is None or len(result.values) < 2:
                self.get_logger().warn('Bad parameter reply, using defaults')
                self.joint_list = ['empty']
                self.publish_rate = 1.0
            else:
                self.joint_list = result.values[0].string_array_value
                self.publish_rate = result.values[1].double_value
                self.get_logger().info(f'Got joints={self.joint_list}, publish_rate={self.publish_rate}')

        # Publisher
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/senseglove/sg0/rh/controller/trajectory/command',
            10
        )
        
        # Timer
        period = (2.0 / self.publish_rate) if self.publish_rate > 0 else 1.0
        self.timer = self.create_timer(period, self._on_timer)

    def _on_timer(self):
        traj = JointTrajectory()
        traj.header = Header()
        now = self.get_clock().now()
        future = now + RclDuration(seconds=0.1)
        traj.header.stamp = future.to_msg()
        traj.joint_names = list(self.joint_list)

        pt = JointTrajectoryPoint()
        pt.positions = [0.0] * len(self.joint_list)
        pt.time_from_start = RclDuration(seconds=0.05).to_msg()
        traj.points.append(pt)

        self.publisher_.publish(traj)

def main(args=None):
    rclpy.init(args=args)
    node = HapticsSimpleNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
