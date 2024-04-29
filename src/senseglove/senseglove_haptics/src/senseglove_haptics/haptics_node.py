#!/usr/bin/env python3

"""@package docstring
ROS Node to send simple Force/Vibration feedback on SenseGlove
"""

import rospy
import time
import numpy as np

from std_msgs.msg import Float64MultiArray

from dynamic_reconfigure.server import Server
from senseglove_haptics.cfg import HapticSliderConfig

HAPTIC_TOPIC = "/senseglove/0/rh/senseglove_haptics"
ACTIVE_STRAP_TOPIC = "/senseglove/0/rh/active_strap_haptics"

class SGHapticFeedback:

    def __init__(self):
         # Initialize ROS
        rospy.init_node('senseglove_haptic_feedback', anonymous=False)
        rospy.loginfo("Starting Haptics")
        self._create_publisher()

        self.ffb_vibro_points = Float64MultiArray()  # init without feedback
        self.active_strap_points = Float64MultiArray()  # init without feedback

    def _create_publisher(self):      
        self.hap_pub = rospy.Publisher(HAPTIC_TOPIC, Float64MultiArray, queue_size=1)
        self.strap_pub = rospy.Publisher(ACTIVE_STRAP_TOPIC, Float64MultiArray, queue_size=1)

    def callback(self):

        # while not rospy.is_shutdown():
        if self.reset == False:
            self.ffb_vibro_points.data = [self.thumb_ffb, self.index_ffb, self.middle_ffb, self.ring_ffb, self.pinky_ffb,
                                     self.thumb_buzz, self.index_buzz, self.middle_buzz, self.ring_buzz, self.pinky_buzz]
            
            self.active_strap_points.data = [self.active_squeeze, self.palm_index_buzz, self.palm_pinky_buzz]

        else:
            self.ffb_vibro_points.data = [0.0, 0.0, 0.0, 0.0, 0.0, 
                                          0.0, 0.0, 0.0, 0.0, 0.0]
            self.active_strap_points.data = [0.0, 0.0, 0.0]

            self.reset_parameters()            
           
        self.hap_pub.publish(self.ffb_vibro_points)
        self.strap_pub.publish(self.active_strap_points)


    def reset_parameters(self):
        rospy.set_param('~thumb_ffb', 0.0)
        rospy.set_param('~index_ffb', 0.0)
        rospy.set_param('~middle_ffb', 0.0)
        rospy.set_param('~ring_ffb', 0.0)
        rospy.set_param('~pinky_ffb', 0.0)

        rospy.set_param('~thumb_buzz', 0.0)
        rospy.set_param('~index_buzz', 0.0)
        rospy.set_param('~middle_buzz', 0.0)
        rospy.set_param('~ring_buzz', 0.0)
        rospy.set_param('~pinky_buzz', 0.0)

        rospy.set_param('~active_squeeze', 0.0)
        rospy.set_param('~palm_index_buzz', 0.0)
        rospy.set_param('~palm_pinky_buzz', 0.0)


    def _dyn_config_callback(self, config, level):
        self.thumb_ffb = config["thumb_ffb"]
        self.index_ffb = config["index_ffb"]
        self.middle_ffb = config["middle_ffb"]
        self.ring_ffb = config["ring_ffb"]
        self.pinky_ffb = config["pinky_ffb"]

        self.thumb_buzz = config["thumb_buzz"]
        self.index_buzz = config["index_buzz"]
        self.middle_buzz = config["middle_buzz"]
        self.ring_buzz = config["ring_buzz"]
        self.pinky_buzz = config["pinky_buzz"]

        self.active_squeeze = config["active_squeeze"]
        self.palm_index_buzz = config["palm_index_buzz"]
        self.palm_pinky_buzz = config["palm_pinky_buzz"]

        self.reset = config["reset"]

        return config        

    def run(self):
        # Init dynamic config before starting the callback/subscriber
        srv = Server(HapticSliderConfig, self._dyn_config_callback) 
        r = rospy.Rate(30)

        while not rospy.is_shutdown():
            self.callback()
            r.sleep()


if __name__== '__main__':
    node = SGHapticFeedback()
    node.run()  