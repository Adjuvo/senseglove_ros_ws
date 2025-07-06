#!/usr/bin/env python3

import sys
import os
import yaml
from collections import deque

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from PyQt5 import QtWidgets, QtCore

from senseglove_msgs.msg import FingerDistanceFloats

# -----------------------------------------------------------------------------
# Calibration data
# -----------------------------------------------------------------------------
class Calibration:
    """
    Class used by a finger distance controller to calibrate fingertip distances.
    """        
    def __init__(self, glove_nr=1, name="default"):
        """
        :param glove_nr: integer >=0; even = left hand, odd = right hand, every 2 increments is new glove index.
        :param name: Calibration profile name (used for saving YAML).
        """
        self.glove_nr = glove_nr
        self.name = name
        self.handedness_list = ["/lh", "/rh"]

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

    def senseglove_callback(self, finger_distance_msg: FingerDistanceFloats):
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

    def save_to_yaml(self):
        data = {
            'pinch_calibration_min': [
                self.avg_thumb_index_pinch[0],
                self.avg_thumb_middle_pinch[1],
                self.avg_thumb_ring_pinch[2],
                self.avg_thumb_pinky_pinch[3]
            ],
            'pinch_calibration_max': self.avg_open_flat
        }
        home = os.path.expanduser('~')
        calib_dir = os.path.join(home, '.ros', 'senseglove', 'calibration')
        os.makedirs(calib_dir, exist_ok=True)
        filename = os.path.join(calib_dir, f"{self.name}.yaml")
        with open(filename, 'w') as f:
            yaml.dump(data, f)
        self.log(f"Calibration data saved to: {filename}")

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
        topic = f"/senseglove/sg{str(int(self.calibration.glove_nr/2))}{self.calibration.handedness_list[self.calibration.glove_nr % 2]}/finger_distances"
        self.node.create_subscription(
            FingerDistanceFloats, 
            topic,
            self.calibration.senseglove_callback,
            1
        )
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
        # Set ROS 2 node parameters so other nodes can pick up immediately
        if self.node:
            params = [
                rclpy.parameter.Parameter('pinch_calibration_min', rclpy.Parameter.Type.DOUBLE_ARRAY, self.calibration.pinch_calibration_min),
                rclpy.parameter.Parameter('pinch_calibration_max', rclpy.Parameter.Type.DOUBLE_ARRAY, self.calibration.pinch_calibration_max),
            ]
            self.node.set_parameters(params)
            self._log("Parameters set on node parameter server.")
        # Save YAML profile
        self.calibration.save_to_yaml()
        self._log("Calibration saved. Closing GUI.")
        QtWidgets.QApplication.quit()

    def _on_cancel(self):
        self._log("Calibration cancelled by user.")
        self.calibration.cancelled = True
        QtWidgets.QApplication.quit()

    def _log(self, msg: str):
        self.log_text.append(msg)
        print(f"[GUI] {msg}")

# -----------------------------------------------------------------------------
# Main entry
# -----------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = Node('finger_distance_calibration')

    node.declare_parameter('pinch_calibration_min', [0.0, 0.0, 0.0, 0.0])
    node.declare_parameter('pinch_calibration_max', [100.0, 100.0, 100.0, 100.0])

    glove_nr = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    profile  = sys.argv[2] if len(sys.argv) > 2 else 'default'

    calibration = Calibration(glove_nr=glove_nr, name=profile)

    # Launch Qt GUI (blocks until closed)
    app = QtWidgets.QApplication(sys.argv)
    gui = CalibrationGUI(calibration, node)
    gui.show()
    app.exec_()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
