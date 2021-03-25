from __future__ import print_function
from collections import deque
import rospy
import rosparam
import sys
from os.path import isdir, exists
import rospkg
import numpy as np

from senseglove_shared_resources.msg import FingerDistances

class Calibration:

    def __init__(self, glove_nr, name):
        self.glove_nr = glove_nr
        self.name = name  # Calibration profile name

        # Defaults
        self.offset_calibration = [[-6.8, -36.7, 14.4],
                                   [-6.8, -29.7, 18.7],
                                   [-6.8, -25.8, 9.1]]  # [index, middle, ring][x, y, z]
        self.length_calibration = [51.3, 70.6, 63.6]  # [index, middle, ring]

        self.avg_open_flat = [0.0, 0.0, 0.0]  # distances between thumb&index thumb&middle thumb&ring
        self.avg_thumb_index_pinch = [0.0, 0.0, 0.0]
        self.avg_thumb_middle_pinch = [0.0, 0.0, 0.0]
        self.avg_thumb_ring_pinch = [0.0, 0.0, 0.0]

        self.finished_open_flat = False
        self.finished_thumb_index_pinch = False
        self.finished_thumb_middle_pinch = False
        self.finished_thumb_ring_pinch = False

        self.calib_time = 2  # sec

        self.databuffer = deque(maxlen=10)

    def set_open_flat(self, avg_positions_msg):
        """
        Call when user holds a flat hand
        """
        for i, position in enumerate([avg_positions_msg.index, avg_positions_msg.middle, avg_positions_msg.ring]):
            self.avg_open_flat = [position.x, position.y, position.z]

        self.finished_open_flat = True

    def set_thumb_index_pinch(self, avg_positions_msg):
        """
        Call when user holds fingers in 90 deg bent position
        """
        if not self.finished_open_flat:
            print("First calibrate the flat hand, then the bent position!")
            return

        self.avg_thumb_index_pinch = [avg_positions_msg.thumb_index, avg_positions_msg.thumb_middle, avg_positions_msg.thumb_ring]
        if self.avg_thumb_index_pinch == self.avg_open_flat:
            rospy.logwarn("Identical measurements! Cannot calibrate. Is your glove still connected?")
            return

        self.finished_thumb_index_pinch = True

    def set_thumb_middle_pinch(self, avg_positions_msg):
        """
        Call when user holds fingers in 90 deg bent position
        """
        if not self.finished_open_flat:
            print("First calibrate the flat hand, then the bent position!")
            return

        self.avg_thumb_middle_pinch = [avg_positions_msg.thumb_index, avg_positions_msg.thumb_middle, avg_positions_msg.thumb_ring]
        if self.avg_thumb_middle_pinch == self.avg_open_flat:
            rospy.logwarn("Identical measurements! Cannot calibrate. Is your glove still connected?")
            return

        self.finished_thumb_middle_pinch = True

    def set_thumb_ring_pinch(self, avg_positions_msg):
        """
        Call when user holds fingers in 90 deg bent position
        """
        if not self.finished_open_flat:
            print("First calibrate the flat hand, then the bent position!")
            return

        self.avg_thumb_ring_pinch = [avg_positions_msg.thumb_index, avg_positions_msg.thumb_middle, avg_positions_msg.thumb_ring]
        if self.avg_thumb_ring_pinch == self.avg_open_flat:
            rospy.logwarn("Identical measurements! Cannot calibrate. Is your glove still connected?")
            return

        self.finished_thumb_ring_pinch = True

    def is_calibrated(self):
        return self.finished_open_flat and self.finished_thumb_index_pinch and self.finished_thumb_middle_pinch and self.finished_thumb_ring_pinch

    def run_interactive_calibration(self):
        """
        Run an interactive (CLI) session for calibration.
        """

        rospy.Subscriber('senseglove_' + str(self.glove_nr) + '/finger_distances', FingerDistances, callback=self.senseglove_callback)

        rospy.loginfo("Calibration of senseglove started, please flatten your hand.")
        rospy.loginfo("Type [y] + [Enter] when ready, or [q] + [Enter] to quit.")

        self.key_press_interface()
        self.log_finger_distances()

        # Set average values for flat hand
        self.set_open_flat(self.get_avg_finger_distances())

        rospy.loginfo("Step 1 done.")

        rospy.loginfo("Calibration step 2, please pinch with your index finger and thumb.")
        rospy.loginfo("Type [y] + [Enter] when ready, or [q] + [Enter] to quit.")

        self.key_press_interface()
        self.log_finger_distances()

        # Set average values for bent fingers
        self.set_thumb_index_pinch(self.get_avg_finger_distances())
        if not self.finished_thumb_index_pinch:
            rospy.logerr("Could not finish thumb to index pinch calibration, calibration failed")
            return False

        rospy.loginfo("Step 2 done")

        rospy.loginfo("Calibration step 3, please pinch with your middle finger and thumb.")
        rospy.loginfo("Type [y] + [Enter] when ready, or [q] + [Enter] to quit.")

        self.key_press_interface()
        self.log_finger_distances()

        # Set average values for bent fingers
        self.set_thumb_middle_pinch(self.get_avg_finger_distances())
        if not self.finished_thumb_middle_pinch:
            rospy.logerr("Could not finish thumb to middle pinch calibration, calibration failed")
            return False

        rospy.loginfo("Step 3 done")

        rospy.loginfo("Calibration step 4, please pinch with your ring finger and thumb.")
        rospy.loginfo("Type [y] + [Enter] when ready, or [q] + [Enter] to quit.")

        self.key_press_interface()
        self.log_finger_distances()

        # Set average values for bent fingers
        self.set_thumb_ring_pinch(self.get_avg_finger_distances())
        if not self.finished_thumb_ring_pinch:
            rospy.logerr("Could not finish thumb to index pinch calibration, calibration failed")
            return False

        rospy.loginfo("Step 4 done")



        rospy.loginfo("Computing calibration parameters...")

        """
        Calibration data:
        - finger offset: measured offset between human hand and senseglove. The senseglove sees the base of the glove's 
        first finger as (0,0,0). With these offsets applied, the base of the human's first finger becomes (0,0,0). 
        For instance, the base of the human's middle finger then is measured at location (x, 0, 0) with x the distance 
        between the attachments of the first and middle finger.
        - finger length: to determine the proper scaling that should be applied between human and shadow hand 
        coordinates based on their differences in size.
        """

        for i, _ in enumerate(['index', 'middle', 'ring']):
            # offset for every finger = [x-offset first finger, y-offset current finger, z-offset current finger]
            self.offset_calibration[i] = [self.avg_open_flat[0][0], self.avg_open_flat[i][1], self.avg_pinch[i][2]]
            # length: flat z-position - bent (90 deg) z-position of finger
            self.length_calibration[i] = self.avg_open_flat[i][2] - self.avg_pinch[i][2]
            if self.length_calibration[i] == 0.0:
                rospy.logwarn("Got finger length zero. Is your glove still connected?")
                return False


        rospy.loginfo("The calibration for '%s' is done. These are the numbers:" % self.name)
        rospy.loginfo("Finger offsets: %s\n" % self.offset_calibration)
        rospy.loginfo("Finger lengths: %s\n" % self.length_calibration)
        rospy.loginfo("Type [y] + [Enter] when OK, or [q] + [Enter] to discard and quit.")

        self.key_press_interface()

        rospy.loginfo("Calibration successful!")
        rospy.loginfo("Setting on param server and saving to file...")

        # Set parameters
        rospy.set_param('~offset_calibration', self.offset_calibration)
        rospy.set_param('~length_calibration', self.length_calibration)
        config_folder = rospkg.RosPack().get_path('senseglove_ros') + "/calib"

        if not isdir(config_folder):
            rospy.logwarn("Could not locate calibration folder %s, not saving." % config_folder)
        else:
            filename = config_folder + "/" + self.name + ".yaml"
            if exists(filename):
                rospy.logwarn("Overwriting %s" % filename)
            rosparam.dump_params(filename, rospy.get_name())
            rospy.loginfo("Done!")

        return True

    def senseglove_callback(self, finger_distance_msg):
        self.databuffer.appendleft(finger_distance_msg)

    def get_avg_finger_distances(self):

        avg_positions_msg = FingerDistances()

        thumb_indexdata = [x.thumb_index for x in self.databuffer]
        if len(thumb_indexdata) == 0:
            rospy.logwarn("No data received! Is your glove still connected?")
        else:
            avg_positions_msg.thumb_index = sum(thumb_indexdata) / len(thumb_indexdata)

        thumb_middledata = [x.thumb_middle for x in self.databuffer]
        if len(thumb_middledata) == 0:
            rospy.logwarn("No data received! Is your glove still connected?")
        else:
            avg_positions_msg.thumb_middle = sum(thumb_middledata) / len(thumb_middledata)

        thumb_ringdata = [x.thumb_ring for x in self.databuffer]
        if len(thumb_ringdata) == 0:
            rospy.logwarn("No data received! Is your glove still connected?")
        else:
            avg_positions_msg.thumb_ring = sum(thumb_ringdata) / len(thumb_ringdata)

        return avg_positions_msg

    def key_press_interface(self):
        k = raw_input()

        while not (k == 'q' or k == 'y'):
            rospy.loginfo("Not valid: %s. Type [y] + [Enter] when ready, or [q] + [Enter] to quit." % k)
            k = raw_input()

        if k == "q":
            rospy.loginfo("Calibration aborted!")
            return False

    def log_finger_distances(self):
        self.databuffer.clear() # Start with a fresh buffer
        for i in range(int(self.calib_time/0.05)):
            print(".", end="")
            sys.stdout.flush()
            rospy.sleep(0.05)
        print()
