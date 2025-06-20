#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient as ParameterClient
from rclpy.parameter import Parameter
from rclpy.duration import Duration as RclDuration

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class HapticsDynamicNode(Node):
    FEEDBACK_PARAMS = [
        'thumb_ffb', 'index_ffb', 'middle_ffb', 'ring_ffb', 'pinky_ffb',
        'thumb_buzz', 'index_buzz', 'middle_buzz', 'ring_buzz', 'pinky_buzz',
        'thumper_buzz', 'palm_index_buzz', 'palm_pinky_buzz', 'palm_strap'
    ]

    def __init__(self):
        super().__init__('senseglove_haptics_node_dynamic')
        self.get_logger().info('Initializing dynamic haptics node')

        # Node with parameters
        self.declare_parameter('controller_node', '/senseglove/sg0/rh/controller')
        controller = self.get_parameter('controller_node').value

        # ParameterClient to fetch parameters
        client = ParameterClient(self, controller)
        if not client.wait_for_services(timeout_sec=2.0):
            self.get_logger().error(f'Could not contact {controller} parameter service; using defaults')
            self.joint_list = ['empty']
            self.publish_rate = 1.0
        else:
            # Fetch parameters
            future = client.get_parameters(['trajectory.joints', 'hand_state.publish_rate'])
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            res = future.result()
            if not res or len(res.values) < 2:
                self.get_logger().warn('Bad parameter response; using defaults')
                self.joint_list = ['empty']
                self.publish_rate = 1.0
            else:
                self.joint_list = res.values[0].string_array_value
                self.publish_rate = res.values[1].double_value
                self.get_logger().info(f'Got joints={self.joint_list}, publish_rate={self.publish_rate}')

        # Declare dynamic feedback parameters and reset flag
        for p in self.FEEDBACK_PARAMS:
            self.declare_parameter(p, 0.0)
        self.declare_parameter('reset', False)

        # Load the initial values into attributes
        self._load_feedback_params()
        self.reset_flag = self.get_parameter('reset').value

        # Publisher
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/senseglove/sg0/rh/controller/trajectory/command',
            10
        )

        # Watch for live updates like dynamic_reconfigure in ROS1
        self.add_on_set_parameters_callback(self._on_parameter_update)

        # Timer
        period = (2.0 / self.publish_rate) if self.publish_rate > 0 else 1.0
        self.timer = self.create_timer(period, self._publish_trajectory)

    def _load_feedback_params(self):
        for p in self.FEEDBACK_PARAMS:
            setattr(self, p, self.get_parameter(p).value)

    def _on_parameter_update(self, params):
        need_timer_reset = False

        for param in params:
            name = param.name
            if name == 'publish_rate':
                self.publish_rate = param.value
                need_timer_reset = True
            elif name == 'joints':
                self.joint_list = param.value
            elif name == 'reset':
                self.reset_flag = param.value
            elif name in self.FEEDBACK_PARAMS:
                setattr(self, name, param.value)

        # Rebuild timer if publish_rate is changed
        if need_timer_reset:
            new_period = (2.0 / self.publish_rate) if self.publish_rate > 0 else 1.0
            self.timer.cancel()
            self.timer = self.create_timer(new_period, self._publish_trajectory)
            self.get_logger().info(f'publish_rate updated to {self.publish_rate}, timer period {new_period:.3f}s')

        return rclpy.node.ParameterEventHandlerResult.SUCCESS

    def _publish_trajectory(self):
        # If reset requested, zero all feedback, clear flag, and skip one cycle
        if self.reset_flag:
            for p in self.FEEDBACK_PARAMS:
                self.set_parameters([Parameter(p, Parameter.Type.DOUBLE, 0.0)])
            self.set_parameters([Parameter('reset', Parameter.Type.BOOL, False)])
            self.reset_flag = False
            self.get_logger().info('Feedback parameters reset to zero')
            return

        traj = JointTrajectory()
        traj.header = Header()
        now = self.get_clock().now()
        traj.header.stamp = (now + RclDuration(seconds=0.1)).to_msg()
        traj.joint_names = list(self.joint_list)

        pt = JointTrajectoryPoint()
        pt.positions = [float(getattr(self, p)) for p in self.FEEDBACK_PARAMS]
        pt.time_from_start = RclDuration(seconds=0.05).to_msg()

        if pt.positions:
            traj.points.append(pt)
            self.publisher_.publish(traj)
        else:
            self.get_logger().warn('No feedback positions to publish; skipping.')

    def destroy_node(self):
        self.get_logger().info('Shutting down dynamic haptics node')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HapticsDynamicNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()