#!/usr/bin/env python3

"""@package docstring
ROS Node to send simple Force/Vibration feedback on SenseGlove
"""

import rospy
import numpy as np

from dynamic_reconfigure.server import Server
from senseglove_haptics.cfg import HapticSliderConfig

from std_msgs.msg import Time
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


HAPTIC_TOPIC = "/senseglove/0/rh/controller/trajectory/command"

class SGHapticFeedback:

    def __init__(self):
         # Initialize ROS
        rospy.init_node('senseglove_haptics_node_dynamic')
        rospy.loginfo("initialize haptics node")

        self._create_publisher()

        self.joint_list = ['empty']
        if rospy.has_param('/senseglove/0/rh/controller/trajectory/joints'):
            self.joint_list = rospy.get_param('/senseglove/0/rh/controller/trajectory/joints')
        
        self.publish_rate = 1
        if rospy.has_param('/senseglove/0/rh/controller/hand_state/publish_rate'):
            self.publish_rate = rospy.get_param('/senseglove/0/rh/controller/hand_state/publish_rate')

        self.hap_pub = rospy.Publisher('/senseglove/0/rh/controller/trajectory/command', JointTrajectory, queue_size=1)
    

        
    def _create_publisher(self):      
        self.hap_pub = rospy.Publisher(HAPTIC_TOPIC, JointTrajectory, queue_size=1)
    

    def callback(self):        
        
        self.trajectory = JointTrajectory()
        self.trajectory.header = Header()
        self.trajectory.joint_names = self.joint_list
        self.trajectory.header.stamp = rospy.Time.now()

        self.point = JointTrajectoryPoint()        
        self.point.time_from_start = rospy.Duration.from_sec(0.1)

        if self.reset:
            self.reset_parameters()   

        # self.point.positions = [self.thumb_ffb, self.index_ffb, self.middle_ffb, self.ring_ffb, self.pinky_ffb,
        #                     self.thumb_buzz, self.index_buzz, self.middle_buzz, self.ring_buzz, self.pinky_buzz, self.thumper_buzz]
        
                
        # self.point.positions = [self.thumb_ffb, self.index_ffb, self.middle_ffb, self.ring_ffb,
        #                         self.thumb_buzz, self.index_buzz, 
        #                         self.thumper_buzz]

        self.point.positions = [self.thumb_ffb, self.index_ffb, self.middle_ffb, self.ring_ffb,
                                self.thumb_buzz, self.index_buzz, 
                                self.palm_index_buzz, self.palm_pinky_buzz, self.palm_strap]


        self.trajectory.points.append(self.point)
        self.hap_pub.publish(self.trajectory)


    def reset_parameters(self):
        # Reset all haptic feedback parameters to zero
        feedback_params = [
            'thumb_ffb', 'index_ffb', 'middle_ffb', 'ring_ffb', 'pinky_ffb',
            'thumb_buzz', 'index_buzz', 'middle_buzz', 'ring_buzz', 'pinky_buzz',
            'thumper_buzz', 'palm_index_buzz', 'palm_pinky_buzz', 'palm_strap'
        ]
        for param in feedback_params:
            rospy.set_param(f'~{param}', 0.0)
            setattr(self, param, 0.0)
        self.reset = False

    def _dyn_config_callback(self, config, level):
        # Update haptic feedback parameters from dynamic reconfigure
        feedback_params = [
            'thumb_ffb', 'index_ffb', 'middle_ffb', 'ring_ffb', 'pinky_ffb',
            'thumb_buzz', 'index_buzz', 'middle_buzz', 'ring_buzz', 'pinky_buzz',
            'thumper_buzz', 'palm_index_buzz', 'palm_pinky_buzz', 'palm_strap'
        ]
        for param in feedback_params:
            setattr(self, param, config[param])

        self.reset = config['reset']
        return config 
     
    def run(self):
        # Init dynamic config before starting the callback/subscriber
        srv = Server(HapticSliderConfig, self._dyn_config_callback) 
        r = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            self.callback() 
            r.sleep() 

        if rospy.is_shutdown():
            return

                
if __name__== '__main__':
    node = SGHapticFeedback()
    node.run()  