#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from senseglove_msgs.msg import SenseGloveState, FingerDistanceFloats
from senseglove_msgs.srv import Calibrate
from senseglove_interaction.finger_distance.finger_distance_calibration import Calibration
from senseglove_interaction.finger_distance.finger_tip_vector import FingerTipVector

class FingerTipHandler(Node):
    def __init__(self, finger_nrs=None, use_best_effort_qos=True):
        super().__init__('finger_tip_distance_node')
        self.get_logger().info(f"Initializing: {self.get_name()}")

        # Parameters for calibration
        default_list = [0.0] * 4
        self.declare_parameter('pinch_calibration_min', default_list)
        self.declare_parameter('pinch_calibration_max', default_list)
        self.declare_parameter('calib_mode', 'nothing')
        self.declare_parameter('publish_rate', 1000.0)

        # Initialize state
        self.finger_nrs = finger_nrs or [0, 1, 2, 3, 4]  # thumb, index, middle, ring, pinky
        self.calib_mode = self.get_parameter('calib_mode').value
        self.finger_field_names = ['th_ff', 'th_mf', 'th_rf', 'th_lf']

        # Data
        self.finger_tip_positions = [FingerTipVector() for _ in self.finger_nrs]

        # QoS profile
        self.use_best_effort_qos = use_best_effort_qos
        if self.use_best_effort_qos:
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                durability=DurabilityPolicy.VOLATILE
            )
        else:
            qos_profile = QoSProfile(depth=10)   # default reliable QoS


        # Publisher and Subscriber
        self.pub = self.create_publisher(FingerDistanceFloats, "finger_distances", 1)
        self.sub = self.create_subscription(SenseGloveState, "senseglove_states", self.callback, qos_profile)

        # Calibration
        self.setup_calibration()
        self.add_on_set_parameters_callback(self._on_set_parameters)

    def setup_calibration(self):
        self.get_logger().info(f"Setting up Calibration for {self.get_fully_qualified_name()}")
        self.calibration = Calibration()

        min_vals = self.get_parameter('pinch_calibration_min').value
        max_vals = self.get_parameter('pinch_calibration_max').value

        if any(min_vals) or any(max_vals):
            self.calibration.pinch_calibration_min = min_vals
            self.calibration.pinch_calibration_max = max_vals
            self.get_logger().info("Loaded calibration from parameters")
            self.get_logger().info(f"Calibration Min values: {self.calibration.pinch_calibration_min}")
            self.get_logger().info(f"Calibration Max values: {self.calibration.pinch_calibration_max}")
            
        else:
            self.get_logger().warn("No calibration data found, using defaults")

    def _on_set_parameters(self, params: list[Parameter]) -> SetParametersResult:
        try:
            for p in params:
                if p.name == 'pinch_calibration_min' and p.type_ == Parameter.Type.DOUBLE_ARRAY:
                    self.calibration.pinch_calibration_min = list(map(float, p.value))
                    self.get_logger().info(f"Updated pinch_calibration_min: {self.calibration.pinch_calibration_min}")

                elif p.name == 'pinch_calibration_max' and p.type_ == Parameter.Type.DOUBLE_ARRAY:
                    self.calibration.pinch_calibration_max = list(map(float, p.value))
                    self.get_logger().info(f"Updated pinch_calibration_max: {self.calibration.pinch_calibration_max}")

                elif p.name == 'calib_mode' and p.type_ in (Parameter.Type.STRING,):
                    self.calib_mode = str(p.value)
                    self.get_logger().info(f"Updated calib_mode: {self.calib_mode}")

            return SetParametersResult(successful=True)
        except Exception as e:
            self.get_logger().error(f"Parameter update rejected: {e}")
            return SetParametersResult(successful=False, reason=str(e))

    def apply_calib(self, pinch_value, pinch_combination, mode):
        if mode == 'nothing':
            return pinch_value
        if mode == 'minimum':
            return pinch_value - self.calibration.pinch_calibration_min[pinch_combination]
        elif mode == 'normalized':
            # Return normalized finger distance value between 0 and 100
            min_val = self.calibration.pinch_calibration_min[pinch_combination]
            max_val = self.calibration.pinch_calibration_max[pinch_combination]

            normalized = (pinch_value - min_val) / (max_val - min_val)
            clamped = max(0.0, min(1.0, normalized))
            return clamped * 100.0

    def distance_publish(self):
        msg = FingerDistanceFloats()
        # Apply calibration and publish distances
        msg.th_ff.data = self.apply_calib((self.finger_tip_positions[0] - self.finger_tip_positions[1]).magnitude(), 0, self.calib_mode)
        msg.th_mf.data = self.apply_calib((self.finger_tip_positions[0] - self.finger_tip_positions[2]).magnitude(), 1, self.calib_mode)
        msg.th_rf.data = self.apply_calib((self.finger_tip_positions[0] - self.finger_tip_positions[3]).magnitude(), 2, self.calib_mode)
        msg.th_lf.data = self.apply_calib((self.finger_tip_positions[0] - self.finger_tip_positions[4]).magnitude(), 3, self.calib_mode)
        self.pub.publish(msg)

    def callback(self, msg: SenseGloveState):
        # Update fingertip positions
        for i in range(len(self.finger_nrs)):
            pt = msg.finger_tip_position[i]
            self.finger_tip_positions[i].x = float(pt.x)
            self.finger_tip_positions[i].y = float(pt.y)
            self.finger_tip_positions[i].z = float(pt.z)
                        
        self.distance_publish()

def main(args=None):
    rclpy.init(args=args)
    node = FingerTipHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
