#!/usr/bin/env python3

import sys
import subprocess
from math import sqrt, pow

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from senseglove_msgs.msg import SenseGloveState, FingerDistanceFloats
from senseglove_msgs.srv import Calibrate
from senseglove_interaction.finger_distance.finger_distance_calibration import Calibration

class FingerTipVector:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, other):
        return FingerTipVector(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return FingerTipVector(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, other):
        return self.x * other.x + self.y * other.y + self.z * other.z

    def magnitude(self):
        return sqrt(pow(self.x, 2) + pow(self.y, 2) + pow(self.z, 2))


class FingerTipHandler(Node):
    def __init__(self, glove_nr="1", calib_mode='nothing', finger_nrs=None):
        super().__init__('senseglove_finger_distance_node')

        # Parameters for calibration
        default_list = [0.0] * 4
        self.declare_parameter('pinch_calibration_min', default_list)
        self.declare_parameter('pinch_calibration_max', default_list)

        # Initialize state
        self.finger_nrs = finger_nrs or [3, 7, 11, 15, 19]
        self.calib_mode = calib_mode
        self.finger_tips = [FingerTipVector() for _ in self.finger_nrs]
        self.glove_nr = glove_nr
        self.handedness = ["/lh", "/rh"]
        self.senseglove_ns = f"/senseglove/sg{int(int(glove_nr)/2)}{self.handedness[int(int(glove_nr) % 2)]}"

        # Publisher and Subscriber
        self._create_publishers()
        self._create_subscriber()

        # Calibration
        self.add_on_set_parameters_callback(self._on_params_changed)
        self._setup_calibration()

        # Calibration Service
        self.create_service(
            Calibrate,
            f'{self.senseglove_ns}/Calibrate',
            self.calibrate_service
            )
        self.get_logger().info(f"Calibration service 'Calibrate' ready at {self.senseglove_ns}")

    def _on_params_changed(self, params):
        for p in params:
            if p.name == 'pinch_calibration_min':
                self.calibration.pinch_calibration_min = p.value
            elif p.name == 'pinch_calibration_max':
                self.calibration.pinch_calibration_max = p.value
        return SetParametersResult(successful=True, reason='')

    def _create_publishers(self):   
        self.pub = self.create_publisher(FingerDistanceFloats, f"{self.senseglove_ns}/finger_distances", 1)

    def _create_subscriber(self):
        self.create_subscription(SenseGloveState,f"{self.senseglove_ns}/senseglove_states", self.callback, 1)

    def _setup_calibration(self):
        self.get_logger().info(f"Setting up Calibration for {self.senseglove_ns}")
        self.calibration = Calibration(name='default')

        min_vals = self.get_parameter('pinch_calibration_min').value
        max_vals = self.get_parameter('pinch_calibration_max').value

        if any(min_vals) or any(max_vals):
            self.calibration = Calibration('from_param_server')
            self.calibration.pinch_calibration_min = min_vals
            self.calibration.pinch_calibration_max = max_vals
            self.get_logger().info("Loaded calibration from parameters")
        else:
            self.get_logger().warn("No calibration data found, using defaults")

    def calibrate_service(self, request, response):
        self.get_logger().info("Executing calibration service via GUI")
        # Launch the calibration process
        proc = subprocess.Popen([
            'ros2', 'run', 'senseglove_interaction',
            'finger_distance_calibration', self.glove_nr, request.name
        ])
        proc.wait()

        # Check updated parameters
        min_vals = self.get_parameter('pinch_calibration_min').value
        max_vals = self.get_parameter('pinch_calibration_max').value
        if any(min_vals) and any(max_vals):
            self.get_logger().info("Calibration completed successfully")
            response.success = True
        else:
            self.get_logger().warn("Calibration failed; parameters not updated")
            response.success = False
        return response

    def apply_calib(self, pinch_value, pinch_combination, mode):
        if mode == 'nothing':
            return pinch_value
        if mode == 'minimum':
            return pinch_value - self.calibration.pinch_calibration_min[pinch_combination]
        elif mode == 'normalized':
            # Return normalized finger distance value between 0 and 1
            minv = self.calibration.pinch_calibration_min[pinch_combination]
            maxv = self.calibration.pinch_calibration_max[pinch_combination]
            return (pinch_value - minv) / maxv if maxv else 0.0

    def callback(self, msg: SenseGloveState):
        # Update fingertip positions
        for i in range(len(self.finger_nrs)):
            pt = msg.finger_tip_positions[i]
            self.finger_tips[i].x = float(pt.x)
            self.finger_tips[i].y = float(pt.y)
            self.finger_tips[i].z = float(pt.z)
        self.distance_publish()

    def distance_publish(self):
        msg = FingerDistanceFloats()
        # Apply calibration and publish distances
        msg.th_ff.data = self.apply_calib((self.finger_tips[0] - self.finger_tips[1]).magnitude(), 0, self.calib_mode)
        msg.th_mf.data = self.apply_calib((self.finger_tips[0] - self.finger_tips[2]).magnitude(), 1, self.calib_mode)
        msg.th_rf.data = self.apply_calib((self.finger_tips[0] - self.finger_tips[3]).magnitude(), 2, self.calib_mode)
        msg.th_lf.data = (self.finger_tips[0] - self.finger_tips[4]).magnitude()
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    # Parse command-line args: glove_nr and calib_mode
    glove_nr = sys.argv[1] if len(sys.argv) > 1 else '1'
    calib_mode = sys.argv[2] if len(sys.argv) > 2 else 'nothing'

    node = FingerTipHandler(glove_nr=glove_nr, calib_mode=calib_mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
