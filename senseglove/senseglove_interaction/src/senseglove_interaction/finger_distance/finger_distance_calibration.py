#!/usr/bin/env python3

import sys
import os
from os.path import isdir, exists
import rospy
import rospkg
import rosparam
from collections import deque
from PyQt5 import QtWidgets, QtCore
from senseglove_msgs.msg import FingerDistanceFloats

class Calibration:
    """
    Class used by a finger distance controller to calibrate the distances between the fingertips of the user.
    The objects of this class are used as an interface to execute calibrating commands.
    """
        
    def __init__(self, glove_nr=1, name="default"):
        """
        Initializes an object of the class Calibration.
        :param glove_nr: a value bigger or equal to 0. Even numbers are left hands and right hands get an uneven number. Every increment of 2 results in a new set of gloves. 0 & 1 are a set and so are 2 & 3.
        :param name: The name of the calibration, useful when saving calibration data.
        """

        self.glove_nr = glove_nr

        self.name = name  # Calibration profile name
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

    def set_open_flat(self, avg_positions_msg):
        """
        Call when user holds a flat hand
        """
        self.avg_open_flat = [
            avg_positions_msg.th_ff.data,
            avg_positions_msg.th_mf.data,
            avg_positions_msg.th_rf.data,
            avg_positions_msg.th_lf.data
        ]
        self.finished_open_flat = True

    def set_thumb_index_pinch(self, avg_positions_msg):
        """
        Call when user pinches index finger and thumb
        """
        if not self.finished_open_flat:
            rospy.loginfo("First calibrate the open flat hand, then the pinching position!")
            return        
        self.avg_thumb_index_pinch = [
            avg_positions_msg.th_ff.data,
            avg_positions_msg.th_mf.data,
            avg_positions_msg.th_rf.data,
            avg_positions_msg.th_lf.data
        ]
        if self.avg_thumb_index_pinch == self.avg_open_flat:
            rospy.logwarn("Identical measurements! Cannot calibrate. Is your glove still connected?")
            return
        self.finished_thumb_index_pinch = True

    def set_thumb_middle_pinch(self, avg_positions_msg):
        """
        Call when user pinches middle finger and thumb
        """
        if not self.finished_open_flat:
            rospy.loginfo("First calibrate the open flat hand, then the pinching position!")
            return
        self.avg_thumb_middle_pinch = [
            avg_positions_msg.th_ff.data,
            avg_positions_msg.th_mf.data,
            avg_positions_msg.th_rf.data,
            avg_positions_msg.th_lf.data
        ]
        if self.avg_thumb_middle_pinch == self.avg_open_flat:
            rospy.logwarn("Identical measurements! Cannot calibrate. Is your glove still connected?")
            return
        self.finished_thumb_middle_pinch = True

    def set_thumb_ring_pinch(self, avg_positions_msg):
        """
        Call when user pinches ring finger and thumb
        """
        if not self.finished_open_flat:
            rospy.loginfo("First calibrate the open flat hand, then the pinching position!")
            return
        self.avg_thumb_ring_pinch = [
            avg_positions_msg.th_ff.data,
            avg_positions_msg.th_mf.data,
            avg_positions_msg.th_rf.data,
            avg_positions_msg.th_lf.data
        ]
        if self.avg_thumb_ring_pinch == self.avg_open_flat:
            rospy.logwarn("Identical measurements! Cannot calibrate. Is your glove still connected?")
            return
        self.finished_thumb_ring_pinch = True

    def set_thumb_pinky_pinch(self, avg_positions_msg):
        """
        Call when user pinches pinky finger and thumb
        """
        if not self.finished_open_flat:
            rospy.loginfo("First calibrate the open flat hand, then the pinching position!")
            return
        self.avg_thumb_pinky_pinch = [
            avg_positions_msg.th_ff.data,
            avg_positions_msg.th_mf.data,
            avg_positions_msg.th_rf.data,
            avg_positions_msg.th_lf.data
        ]
        if self.avg_thumb_pinky_pinch == self.avg_open_flat:
            rospy.logwarn("Identical measurements! Cannot calibrate. Is your glove still connected?")
            return
        self.finished_thumb_pinky_pinch = True

    def is_calibrated(self):
        if self.cancelled:
            return False
        return (self.finished_open_flat and 
                self.finished_thumb_index_pinch and
                self.finished_thumb_middle_pinch and 
                self.finished_thumb_ring_pinch and
                self.finished_thumb_pinky_pinch)

    def senseglove_callback(self, finger_distance_msg):
        self.databuffer.appendleft(finger_distance_msg)

    def get_avg_finger_distances(self):
        avg_positions_msg = FingerDistanceFloats()

        if len(self.databuffer) == 0:
            rospy.logwarn("No data received!")
        else:
            avg_positions_msg.th_ff.data = sum([x.th_ff.data for x in self.databuffer]) / len(self.databuffer)
            avg_positions_msg.th_mf.data = sum([x.th_mf.data for x in self.databuffer]) / len(self.databuffer)
            avg_positions_msg.th_rf.data = sum([x.th_rf.data for x in self.databuffer]) / len(self.databuffer)
            avg_positions_msg.th_lf.data = sum([x.th_lf.data for x in self.databuffer]) / len(self.databuffer)

        return avg_positions_msg

# ===== The PyQt5 GUI for Calibration =====
class CalibrationGUI(QtWidgets.QWidget):
    def __init__(self, calibration):
        """
        Pass in a Calibration instance so that the GUI operates on it.
        """
        super(CalibrationGUI, self).__init__()
        self.calibration = calibration
        self.setWindowTitle("SenseGlove Calibration")
        self.resize(600, 500)
        self.init_ui()
        self.subscribe_to_sensor()

    def init_ui(self):
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

        main_layout = QtWidgets.QVBoxLayout()

        header = QtWidgets.QLabel("SenseGlove Calibration")
        header.setAlignment(QtCore.Qt.AlignCenter)
        header.setStyleSheet("font-size: 16pt; font-weight: bold; margin: 10px;")
        main_layout.addWidget(header)

        # Group box
        steps_group = QtWidgets.QGroupBox("Calibration Steps")
        steps_layout = QtWidgets.QGridLayout()

        self.btn_open_flat = QtWidgets.QPushButton("Step 1: Maintain an Open-Flat-Hand (represents #5)")
        self.btn_open_flat.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.btn_thumb_index = QtWidgets.QPushButton("Step 2: Pinch Thumb-Index")
        self.btn_thumb_middle = QtWidgets.QPushButton("Step 3: Pinch Thumb-Middle")
        self.btn_thumb_ring = QtWidgets.QPushButton("Step 4: Pinch Thumb-Ring")
        self.btn_thumb_pinky = QtWidgets.QPushButton("Step 5: Pinch Thumb-Pinky")
        steps_layout.addWidget(self.btn_open_flat, 0, 0, 1, 2)
        steps_layout.addWidget(self.btn_thumb_index, 1, 0)
        steps_layout.addWidget(self.btn_thumb_middle, 1, 1)
        steps_layout.addWidget(self.btn_thumb_ring, 2, 0)
        steps_layout.addWidget(self.btn_thumb_pinky, 2, 1)
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

        self.btn_open_flat.clicked.connect(self.on_calibrate_open_flat)
        self.btn_thumb_index.clicked.connect(self.on_calibrate_thumb_index)
        self.btn_thumb_middle.clicked.connect(self.on_calibrate_thumb_middle)
        self.btn_thumb_ring.clicked.connect(self.on_calibrate_thumb_ring)
        self.btn_thumb_pinky.clicked.connect(self.on_calibrate_thumb_pinky)
        self.btn_save.clicked.connect(self.on_save_calibration)
        self.btn_cancel.clicked.connect(self.on_cancel_calibration)

    def subscribe_to_sensor(self):
        """
        Subscribe to the sensor topic so that sensor data is fed into the calibration.
        """
        topic_name = ("/senseglove/sg" + str(int(self.calibration.glove_nr / 2)) + self.calibration.handedness_list[self.calibration.glove_nr % 2] + "/finger_distances")
        rospy.Subscriber(topic_name, FingerDistanceFloats, self.calibration.senseglove_callback, queue_size=1)
        self.log("Subscribed to: " + topic_name)

    def log(self, message):
        self.log_text.append(message)
        rospy.loginfo(message)

    def gather_data(self):
        self.calibration.databuffer.clear()
        # self.log("Gathering sensor data for {} seconds...".format(self.calibration.calib_time))
        rospy.sleep(self.calibration.calib_time)
        self.log("Data gathering complete.")

    def on_calibrate_open_flat(self):
        self.gather_data()
        avg_msg = self.calibration.get_avg_finger_distances()
        self.calibration.set_open_flat(avg_msg)
        self.log("Step 1 complete! Open flat measurements: {}".format(self.calibration.avg_open_flat))

    def on_calibrate_thumb_index(self):
        if not self.calibration.finished_open_flat:
            self.log("Error: Please complete the open flat calibration first!")
            return
        self.gather_data()
        avg_msg = self.calibration.get_avg_finger_distances()
        self.calibration.set_thumb_index_pinch(avg_msg)
        self.log("Step 2 complete! Thumb-index measurements: {}".format(self.calibration.avg_thumb_index_pinch))

    def on_calibrate_thumb_middle(self):
        if not self.calibration.finished_open_flat:
            self.log("Error: Please complete the open flat calibration first!")
            return
        self.gather_data()
        avg_msg = self.calibration.get_avg_finger_distances()
        self.calibration.set_thumb_middle_pinch(avg_msg)
        self.log("Step 3 complete! Thumb-middle measurements: {}".format(self.calibration.avg_thumb_middle_pinch))

    def on_calibrate_thumb_ring(self):
        if not self.calibration.finished_open_flat:
            self.log("Error: Please complete the open flat calibration first!")
            return
        self.gather_data()
        avg_msg = self.calibration.get_avg_finger_distances()
        self.calibration.set_thumb_ring_pinch(avg_msg)
        self.log("Step 4 complete! Thumb-ring measurements: {}".format(self.calibration.avg_thumb_ring_pinch))

    def on_calibrate_thumb_pinky(self):
        if not self.calibration.finished_open_flat:
            self.log("Error: Please complete the open flat calibration first!")
            return
        self.gather_data()
        avg_msg = self.calibration.get_avg_finger_distances()
        self.calibration.set_thumb_pinky_pinch(avg_msg)
        self.log("Step 5 complete! Thumb-pinky measurements: {}".format(self.calibration.avg_thumb_pinky_pinch))

    def on_save_calibration(self):
        if not self.calibration.is_calibrated():
            self.log("Error: Calibration not complete! Please finish all steps before saving.")
            return
        self.calibration.pinch_calibration_min = [
            self.calibration.avg_thumb_index_pinch[0],
            self.calibration.avg_thumb_middle_pinch[1],
            self.calibration.avg_thumb_ring_pinch[2],
            self.calibration.avg_thumb_pinky_pinch[3]
        ]
        self.calibration.pinch_calibration_max = self.calibration.avg_open_flat

        self.log("Calibration parameters computed:")
        self.log("  Min: {}".format(self.calibration.pinch_calibration_min))
        self.log("  Max: {}".format(self.calibration.pinch_calibration_max))
        rospy.set_param('~pinch_calibration_min', self.calibration.pinch_calibration_min)
        rospy.set_param('~pinch_calibration_max', self.calibration.pinch_calibration_max)
        self.log("Parameters set on the ROS parameter server.")

        # Use the calibration name for the file.
        config_folder = os.path.join(rospkg.RosPack().get_path('senseglove_msgs'), "calibration")
        if not isdir(config_folder):
            self.log("Warning: Calibration folder {} not found; not saving to file.".format(config_folder))
        else:
            # Here the filename is built from the dynamic calibration name.
            filename = os.path.join(config_folder, self.calibration.name + ".yaml")
            if exists(filename):
                self.log("Warning: Overwriting existing file {}".format(filename))
            rosparam.dump_params(filename, rospy.get_name())
            self.log("Calibration data saved to file: {}".format(filename))
        
        self.log("Success!")
        QtWidgets.QApplication.quit()

    def on_cancel_calibration(self):
        self.log("Calibration cancelled by user.")
        self.calibration.cancelled = True
        QtWidgets.QApplication.quit()

# ===== Helper Function =====
def run_gui_calibration(calibration_instance):
    """
    Launch the calibration GUI with the provided Calibration instance.
    Blocks until the GUI is closed.
    Returns True if calibration was successful, False otherwise.
    """
    app = QtWidgets.QApplication([])
    gui = CalibrationGUI(calibration_instance)
    gui.show()
    app.exec_()
    return calibration_instance.is_calibrated()

if __name__ == '__main__':
    rospy.init_node('finger_distance_calibration')
    glove_nr = sys.argv[1] if len(sys.argv) > 2 else "1"
    file_name = sys.argv[2] if len(sys.argv) > 1 else "default"

    calib = Calibration(glove_nr= int(glove_nr), name=file_name)
    run_gui_calibration(calib)

