#!/usr/bin/env python3
import sys
import argparse
import subprocess

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters

from senseglove_msgs.srv import Calibrate

def parse_args(argv):
    p = argparse.ArgumentParser(
        prog='calibration_manager',
        description='Run fingertip-distance calibration for a target node.'
    )
    p.add_argument('--target-ns', required=True,
                   help='FQN namespaceof the finger-distance node, e.g. /senseglove/glove0/rh')
    p.add_argument('--call-service', action='store_true',
                   help='If set, automatically call the calibration service on startup')
    return p.parse_args(argv)

class CalibrationManager(Node):
    def __init__(self, target_ns: str, call_service: bool):
        super().__init__('calibration_manager')
        self.get_logger().info(f"Initializing: {self.get_name()}")

        # Parameters for calibration
        self.target_node = f"{target_ns.rstrip('/')}/finger_tip_distance_node"

        self.srv_calibrate = self.create_service(Calibrate, "Calibrate", self.handle_calibration)
        self.get_logger().info(f"Calibration service ready!")

        self._auto_called = False
        self._timer = None
        if call_service:
            self.create_timer(1.0, self.auto_call_service)
        self._auto_called = False

    def auto_call_service(self):
        if self._auto_called:
            return
        self._auto_called = True
        req = Calibrate.Request()
        resp = Calibrate.Response()
        self.handle_calibration(req, resp)

    def handle_calibration(self, request, response):
        self.get_logger().info("Executing calibration service via GUI")

        ret = subprocess.call([
            'ros2', 'run', 'senseglove_interaction',
            'finger_tip_distance_calibration',
            self.target_node
        ])

        ok = False
        if ret == 0:
            try:
                ok = self.check_remote_parameters(self.target_node)
            except Exception as e:
                self.get_logger().warn(f"Could not verify params: {e}")

        response.success = ok
        self.get_logger().info("Calibration SUCCESS" if ok else "Calibration FAILED")
        return response

    def check_remote_parameters(self, target_node: str) -> bool:
        client = self.create_client(GetParameters, f"{target_node}/get_parameters")
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"{target_node}/get_parameters not available")
        
        req = GetParameters.Request(names=['pinch_calibration_min', 'pinch_calibration_max'])
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if not res or len(res.values) != 2:
            return False

        def arr(v):
            return list(v.double_array_value) if v.type == v.TYPE_DOUBLE_ARRAY else []
        mn = arr(res.values[0]) 
        mx = arr(res.values[1])
        return (any(x != 0.0 for x in mn) and any(x != 0.0 for x in mx))

def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]
    args = parse_args(argv)

    rclpy.init(args=argv)
    node = CalibrationManager(args.target_ns, args.call_service)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()