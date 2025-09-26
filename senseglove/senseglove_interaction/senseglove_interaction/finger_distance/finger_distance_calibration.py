#!/usr/bin/env python3

import sys
import os
import yaml
from collections import deque
from PyQt5 import QtWidgets, QtCore

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter as RclParameter
from rcl_interfaces.srv import SetParameters
from ament_index_python.packages import get_package_share_directory

from senseglove_msgs.msg import FingerDistanceFloats

class Calibration:
    def __init__(self):

        # Defaults
        self.pinch_calibration_min = [0.0, 0.0, 0.0, 0.0]  # [index, middle, ring, pinky] in mm
        self.pinch_calibration_max = [100.0, 100.0, 100.0, 100.0]

        self.avg_open_flat = [0.0, 0.0, 0.0, 0.0]   # distances between thumb&index thumb&middle thumb&ring thumb&pinky
        self.avg_thumb_index_pinch = [0.0, 0.0, 0.0, 0.0]
        self.avg_thumb_middle_pinch = [0.0, 0.0, 0.0, 0.0]
        self.avg_thumb_ring_pinch = [0.0, 0.0, 0.0, 0.0]
        self.avg_thumb_pinky_pinch = [0.0, 0.0, 0.0, 0.0]

        self.finished_open_flat = False
        self.finished_thumb_index_pinch = False
        self.finished_thumb_middle_pinch = False
        self.finished_thumb_ring_pinch = False
        self.finished_thumb_pinky_pinch = False

        self.calib_time = 2  # seconds to gather data
        self.databuffer = deque(maxlen=10)

        # Flag to indicate cancellation
        self.cancelled = False

    def log(self, msg: str):
        print(f"[Calibration] {msg}")

    def callback(self, finger_distance_msg: FingerDistanceFloats):
        self.databuffer.appendleft(finger_distance_msg)

    def get_avg_finger_distances(self) -> FingerDistanceFloats:
        avg_positions_msg = FingerDistanceFloats()
        n = len(self.databuffer)
        if n == 0:
            self.log("Warning: No data received during calibration step.")
            return avg_positions_msg
        sum_ff = sum(x.th_ff.data for x in self.databuffer)
        sum_mf = sum(x.th_mf.data for x in self.databuffer)
        sum_rf = sum(x.th_rf.data for x in self.databuffer)
        sum_lf = sum(x.th_lf.data for x in self.databuffer)
        avg_positions_msg.th_ff.data = sum_ff / n
        avg_positions_msg.th_mf.data = sum_mf / n
        avg_positions_msg.th_rf.data = sum_rf / n
        avg_positions_msg.th_lf.data = sum_lf / n
        return avg_positions_msg

    def set_open_flat(self, avg_positions_msg: FingerDistanceFloats):
        self.avg_open_flat = [
            avg_positions_msg.th_ff.data,
            avg_positions_msg.th_mf.data,
            avg_positions_msg.th_rf.data,
            avg_positions_msg.th_lf.data
        ]
        self.finished_open_flat = True
        self.log(f"Open-flat set: {self.avg_open_flat}")


    def set_thumb_index_pinch(self, avg_positions_msg: FingerDistanceFloats):
        if not self.finished_open_flat:
            self.log("Error: calibrate open-flat first.")
            return
        vals = [
            avg_positions_msg.th_ff.data,
            avg_positions_msg.th_mf.data,
            avg_positions_msg.th_rf.data,
            avg_positions_msg.th_lf.data
        ]
        if vals == self.avg_open_flat:
            self.log("Warning: identical to open-flat; check glove connection.")
            return
        self.avg_thumb_index_pinch = vals
        self.finished_thumb_index_pinch = True
        self.log(f"Thumb-Index pinch set: {self.avg_thumb_index_pinch}")

    def set_thumb_middle_pinch(self, avg_positions_msg: FingerDistanceFloats):
        if not self.finished_open_flat:
            self.log("Error: calibrate open-flat first.")
            return
        vals = [
            avg_positions_msg.th_ff.data,
            avg_positions_msg.th_mf.data,
            avg_positions_msg.th_rf.data,
            avg_positions_msg.th_lf.data
        ]
        if vals == self.avg_open_flat:
            self.log("Warning: identical to open-flat; check glove connection.")
            return
        self.avg_thumb_middle_pinch = vals
        self.finished_thumb_middle_pinch = True
        self.log(f"Thumb-Middle pinch set: {self.avg_thumb_middle_pinch}")

    def set_thumb_ring_pinch(self, avg_positions_msg: FingerDistanceFloats):
        if not self.finished_open_flat:
            self.log("Error: calibrate open-flat first.")
            return
        vals = [
            avg_positions_msg.th_ff.data,
            avg_positions_msg.th_mf.data,
            avg_positions_msg.th_rf.data,
            avg_positions_msg.th_lf.data
        ]
        if vals == self.avg_open_flat:
            self.log("Warning: identical to open-flat; check glove connection.")
            return
        self.avg_thumb_ring_pinch = vals
        self.finished_thumb_ring_pinch = True
        self.log(f"Thumb-Ring pinch set: {self.avg_thumb_ring_pinch}")

    def set_thumb_pinky_pinch(self, avg_positions_msg: FingerDistanceFloats):
        if not self.finished_open_flat:
            self.log("Error: calibrate open-flat first.")
            return
        vals = [
            avg_positions_msg.th_ff.data,
            avg_positions_msg.th_mf.data,
            avg_positions_msg.th_rf.data,
            avg_positions_msg.th_lf.data
        ]
        if vals == self.avg_open_flat:
            self.log("Warning: identical to open-flat; check glove connection.")
            return
        self.avg_thumb_pinky_pinch = vals
        self.finished_thumb_pinky_pinch = True
        self.log(f"Thumb-Pinky pinch set: {self.avg_thumb_pinky_pinch}")

    def is_calibrated(self) -> bool:
        if self.cancelled:
            return False
        return (
            self.finished_open_flat and 
            self.finished_thumb_index_pinch and
            self.finished_thumb_middle_pinch and 
            self.finished_thumb_ring_pinch and
            self.finished_thumb_pinky_pinch
        )

    def save_to_yaml(self, node_key: str):

        new_block = {
            "ros__parameters": {
                "pinch_calibration_min": [
                    float(self.avg_thumb_index_pinch[0]),
                    float(self.avg_thumb_middle_pinch[1]),
                    float(self.avg_thumb_ring_pinch[2]),
                    float(self.avg_thumb_pinky_pinch[3]),
                ],
                "pinch_calibration_max": [float(x) for x in self.avg_open_flat],
            }
        }

        # Locate the bringup package
        pkg_share = get_package_share_directory("senseglove_bringup")
        calib_file = os.path.join(pkg_share, "config", "calibration.yaml")
        os.makedirs(os.path.dirname(calib_file), exist_ok=True)

        # Load existing file
        existing = {}
        if os.path.exists(calib_file):
            with open(calib_file, "r") as f:
                loaded = yaml.safe_load(f) or {}
                if isinstance(loaded, dict):
                    existing = loaded

        # Update just this block
        existing[node_key] = new_block

        with open(calib_file, "w") as f:
            yaml.safe_dump(existing, f, sort_keys=False)

        self.log(f"Calibration params saved to: {calib_file} under key {node_key}")


# -----------------------------------------------------------------------------
# Qt5 GUI for calibration steps
# -----------------------------------------------------------------------------
class CalibrationGUI(QtWidgets.QWidget):
    def __init__(self, calibration: Calibration, node: Node):
        super().__init__()
        self.calibration = calibration
        self.node = node

        self.setWindowTitle("Finger Distance Calibration")
        self.resize(600, 500)

        self._build_ui()
        self._subscribe_to_topic()
        self._start_ros_spin_timer()

    def _build_ui(self):
        self.setStyleSheet("""
            QWidget {
                background-color: #f7f7f7;
                font-family: "Segoe UI", sans-serif;
                font-size: 12pt;
            }
            QPushButton {
                background-color: #007ACC;
                color: white;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #005A9E;
            }
            QTextEdit {
                background-color: white;
                border: 1px solid #ccc;
                padding: 5px;
            }
            QGroupBox {
                font-weight: bold;
                border: 1px solid #007ACC;
                border-radius: 5px;
                margin-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 3px 0 3px;
            }
        """)

        main_layout = QtWidgets.QVBoxLayout(self)

        header = QtWidgets.QLabel("SenseGlove Calibration")
        header.setAlignment(QtCore.Qt.AlignCenter)
        header.setStyleSheet("font-size: 16pt; font-weight: bold; margin: 10px;")
        main_layout.addWidget(header)

        # Group box
        steps_group = QtWidgets.QGroupBox("Calibration Steps")
        steps_layout = QtWidgets.QGridLayout()

        self.btn_open_flat    = QtWidgets.QPushButton("Step 1: Maintain an Open-Flat-Hand (represents #5)")
        self.btn_open_flat.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.btn_thumb_index  = QtWidgets.QPushButton("Step 2: Pinch Thumb-Index")
        self.btn_thumb_middle = QtWidgets.QPushButton("Step 3: Pinch Thumb-Middle")
        self.btn_thumb_ring   = QtWidgets.QPushButton("Step 4: Pinch Thumb-Ring")
        self.btn_thumb_pinky  = QtWidgets.QPushButton("Step 5: Pinch Thumb-Pinky")
        steps_layout.addWidget(self.btn_open_flat,    0,0,1,2)
        steps_layout.addWidget(self.btn_thumb_index,  1,0)
        steps_layout.addWidget(self.btn_thumb_middle, 1,1)
        steps_layout.addWidget(self.btn_thumb_ring,   2,0)
        steps_layout.addWidget(self.btn_thumb_pinky,  2,1)
        steps_group.setLayout(steps_layout)
        main_layout.addWidget(steps_group)

        # Save and Cancel buttons.
        button_layout = QtWidgets.QHBoxLayout()
        self.btn_save = QtWidgets.QPushButton("Save Calibration")
        self.btn_cancel = QtWidgets.QPushButton("Cancel Calibration")
        button_layout.addWidget(self.btn_save)
        button_layout.addWidget(self.btn_cancel)
        main_layout.addLayout(button_layout)

        # log text
        log_group = QtWidgets.QGroupBox("Log")
        log_layout = QtWidgets.QVBoxLayout()
        self.log_text = QtWidgets.QTextEdit()
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        main_layout.addWidget(log_group)

        self.setLayout(main_layout)

        self.btn_open_flat.clicked.connect(lambda: self._run_step('open_flat', "Open flat hand"))
        self.btn_thumb_index.clicked.connect(lambda: self._run_step('thumb_index', "Thumb-Index pinch"))
        self.btn_thumb_middle.clicked.connect(lambda: self._run_step('thumb_middle', "Thumb-Middle pinch"))
        self.btn_thumb_ring.clicked.connect(lambda: self._run_step('thumb_ring', "Thumb-Ring pinch"))
        self.btn_thumb_pinky.clicked.connect(lambda: self._run_step('thumb_pinky', "Thumb-Pinky pinch"))
        self.btn_save.clicked.connect(self._on_save)
        self.btn_cancel.clicked.connect(self._on_cancel)

    def _subscribe_to_topic(self):
        target = getattr(self.node, 'target_node', '')

        if not target:
            self._log("Error: No target node FQN provided; aborting calibration.")
            self._abort(2)
            return

        try:
            ns = target.rsplit('/', 1)[0]
            topic = f"{ns}/finger_distances"
        except Exception as e:
            self._log(f"Error: invalid target node FQN '{target}': {e}. Aborting.")
            self._abort(2)
            return
        
        names = dict(self.node.get_topic_names_and_types())
        if topic not in names:
            self._log(f"Warning: topic '{topic}' not currently advertised.")

        self.node.create_subscription(
            FingerDistanceFloats, topic, self.calibration.callback, 1)
        self._log(f"Subscribed to: {topic}")

    def _start_ros_spin_timer(self):
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0.01))
        self._timer.start(100)  # every 100 ms

    def _run_step(self, step_key: str, description: str):
        # open_flat first
        if step_key != 'open_flat' and not self.calibration.finished_open_flat:
            self._log("Error: complete Step 1 (open flat) first.")
            return
        self._log(f"Gathering data for '{description}' ({self.calibration.calib_time}s)...")

        # Clear buffer
        self.calibration.databuffer.clear()

        # Disable buttons during capture
        for btn in [self.btn_open_flat, self.btn_thumb_index, self.btn_thumb_middle,
                    self.btn_thumb_ring, self.btn_thumb_pinky, self.btn_save]:
            btn.setEnabled(False)

        # After calib_time seconds, finish step
        QtCore.QTimer.singleShot(int(self.calibration.calib_time * 1000),
                                 lambda: self._finish_step(step_key, description))
        
    def _finish_step(self, step_key: str, description: str):
        avg_msg = self.calibration.get_avg_finger_distances()
        if step_key == 'open_flat':
            self.calibration.set_open_flat(avg_msg)
        elif step_key == 'thumb_index':
            self.calibration.set_thumb_index_pinch(avg_msg)
        elif step_key == 'thumb_middle':
            self.calibration.set_thumb_middle_pinch(avg_msg)
        elif step_key == 'thumb_ring':
            self.calibration.set_thumb_ring_pinch(avg_msg)
        elif step_key == 'thumb_pinky':
            self.calibration.set_thumb_pinky_pinch(avg_msg)

        vals = []
        if step_key == 'open_flat':
            vals = self.calibration.avg_open_flat
        else:
            attr = {
                'thumb_index': self.calibration.avg_thumb_index_pinch,
                'thumb_middle': self.calibration.avg_thumb_middle_pinch,
                'thumb_ring': self.calibration.avg_thumb_ring_pinch,
                'thumb_pinky': self.calibration.avg_thumb_pinky_pinch
            }.get(step_key, [])
            vals = attr
        self._log(f"Step '{description}' done. Values: {vals}")
        
        # Re-enable buttons
        for btn in [self.btn_open_flat, self.btn_thumb_index, self.btn_thumb_middle,
                    self.btn_thumb_ring, self.btn_thumb_pinky, self.btn_save]:
            btn.setEnabled(True)

    def _on_save(self):
        if not self.calibration.is_calibrated():
            self._log("Error: Calibration not complete! Please finish all steps before saving.")
            return
        # Compute parameters
        # pinch_calibration_min: [thumb-index, thumb-middle, thumb-ring, thumb-pinky]
        self.calibration.pinch_calibration_min = [
            self.calibration.avg_thumb_index_pinch[0],
            self.calibration.avg_thumb_middle_pinch[1],
            self.calibration.avg_thumb_ring_pinch[2],
            self.calibration.avg_thumb_pinky_pinch[3]
        ]
        self.calibration.pinch_calibration_max = self.calibration.avg_open_flat
        self._log(f"Calibration parameters computed: min={self.calibration.pinch_calibration_min}, max={self.calibration.pinch_calibration_max}")

        # Push to target node
        if getattr(self.node, 'target_node', ''):
            target = self.node.target_node
            self._log(f"Pushing params to target node: {target}")

        # Create a client to the target node's parameter service
        client = self.node.create_client(SetParameters, f"{target}/set_parameters")
        if not client.wait_for_service(timeout_sec=5.0):
            self._log("Error: target parameter service not available.")
        else:
            req = SetParameters.Request()
            req.parameters = [
                RclParameter(
                    'pinch_calibration_min',
                    RclParameter.Type.DOUBLE_ARRAY,
                    list(map(float, self.calibration.pinch_calibration_min))
                ).to_parameter_msg(),
                RclParameter(
                    'pinch_calibration_max',
                    RclParameter.Type.DOUBLE_ARRAY,
                    list(map(float, self.calibration.pinch_calibration_max))
                ).to_parameter_msg(),
            ]

            future = client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)
            if future.result() and all([r.successful for r in future.result().results]):
                self._log("Remote parameters set successfully.")
            else:
                self._log("Warning: Failed to set remote parameters.")
        
        # Save YAML profile
        target_ns = os.path.dirname(target) + "/**"
        self.calibration.save_to_yaml(target_ns)
        self._log("Calibration saved. Closing GUI.")
        QtWidgets.QApplication.quit()

    def _on_cancel(self):
        self._log("Calibration cancelled.")
        self.calibration.cancelled = True
        QtWidgets.QApplication.quit()

    def _abort(self, code: int = 2):
        self._log("Calibration cancelled.")
        self.calibration.cancelled = True
        QtCore.QTimer.singleShot(0, lambda: QtWidgets.QApplication.exit(code))

    def _log(self, msg: str):
        self.log_text.append(msg)
        print(f"[GUI] {msg}")

# -----------------------------------------------------------------------------
# Main entry
# -----------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = Node('finger_tip_distance_calibration')

    node.declare_parameter('pinch_calibration_min', [0.0, 0.0, 0.0, 0.0])
    node.declare_parameter('pinch_calibration_max', [100.0, 100.0, 100.0, 100.0])

    target_node = sys.argv[1] if len(sys.argv) > 1 else ''

    calibration = Calibration()
    node.target_node = target_node

    # Launch Qt GUI (blocks until closed)
    app = QtWidgets.QApplication(sys.argv)
    gui = CalibrationGUI(calibration, node)
    gui.show()
    app.exec_()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
