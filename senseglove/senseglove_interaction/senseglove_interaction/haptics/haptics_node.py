#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient as ParameterClient
from std_msgs.msg import Float64MultiArray

class HapticsNode(Node):
    def __init__(self):
        super().__init__('senseglove_haptics_node')
        self.get_logger().info('Initializing haptics node...')

        # Parameters
        self.declare_parameter('controller_node', '/senseglove/glove0/rh/haptics_controller')
        self.declare_parameter('publish_topic', '/senseglove/glove0/rh/haptics_controller/commands')
        self.declare_parameter('publish_rate', 60)
        self.declare_parameter('hold_time', 2.0) 
        self.declare_parameter('default_efforts', [20.0, 20.0, 20.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.controller_node = self.get_parameter('controller_node').value
        self.publish_topic = self.get_parameter('publish_topic').value
        self.publish_rate = int(self.get_parameter('publish_rate').value)
        self.hold_time = float(self.get_parameter('hold_time').value)
        self.current_efforts = list(self.get_parameter('default_efforts').value)

        # Fetch joint list from controller
        self.joint_names = self._fetch_joint_list()

        # Publisher and Subscriber
        self.pub = self.create_publisher(Float64MultiArray, self.publish_topic, 10)
        self.sub = self.create_subscription(Float64MultiArray, '/haptics_commands', self._callback, 10)

        # Timer
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self._on_timer)
        self.last_command_time = time.time()

        self.get_logger().info(f"Publishing to {self.publish_topic}")
        self.get_logger().info("Joints:\n  " + "\n  ".join(self.joint_names))

        self.get_logger().info(
            f"Playing default efforts for {self.hold_time:.1f} seconds if no matching subscriber call"
        )        

    def _fetch_joint_list(self):
        client = ParameterClient(self, self.controller_node)
        if not client.wait_for_services(timeout_sec=2.0):
            self.get_logger().error(f"Could not reach {self.controller_node} for joints param")
            return ['dummy']

        future = client.get_parameters(['joints'])
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        result = future.result()
        if result and result.values and result.values[0].string_array_value:
            return list(result.values[0].string_array_value)
        else:
            self.get_logger().warn(f"Controller {self.controller_node} has no 'joints' parameter")
            return ['dummy']
        
    def _callback(self, msg: Float64MultiArray):
        if len(msg.data) != len(self.joint_names):
            self.get_logger().warn(f"Expected {len(self.joint_names)} effort values, got {len(msg.data)}")
            return
        self.current_efforts = list(msg.data)
        self.last_command_time = time.time()

    def _on_timer(self):
        elapsed = time.time() - self.last_command_time
        if elapsed > self.hold_time:
            efforts = [0.0] * len(self.joint_names)
        else:
            efforts = self.current_efforts
        self._apply_efforts(efforts)

    def _apply_efforts(self, efforts):
        msg = Float64MultiArray(data=[float(x) for x in efforts])
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HapticsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
