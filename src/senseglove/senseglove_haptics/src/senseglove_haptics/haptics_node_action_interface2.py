#!/usr/bin/env python3

"""@package docstring
ROS Node to send simple Force/Vibration feedback on SenseGlove
"""

import rospy
import sys
from std_msgs.msg import Header
import actionlib
import numpy as np

from dynamic_reconfigure.server import Server
from senseglove_haptics.cfg import HapticSliderConfig

from copy import copy

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

class Trajectory(object):
    def __init__(self, ns='', joint_names=['empty'], goal_time_tol=0.01, timeout=0.001):
        self.wait_for_goal_timeout = timeout
        self._joint_names = joint_names
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(goal_time_tol)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear()

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=0):
        if timeout == 0:
            self._client.wait_for_result(timeout=rospy.Duration(self.wait_for_goal_timeout))
        else:
            self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = self._joint_names


class SGHapticFeedback:

    def __init__(self):
         # Initialize ROS
        rospy.init_node('senseglove_haptics_node')
        rospy.loginfo("initialize haptics node")

        self.joint_list = ['empty']
        ns = '/senseglove/0/rh/controller/'
        self.action_ns = ns + 'trajectory/'
        if rospy.has_param(self.action_ns + 'joints'):
            self.joint_list = rospy.get_param(self.action_ns + 'joints')
        publish_rate = 1
        if rospy.has_param(ns + 'hand_state/publish_rate'):
            publish_rate = rospy.get_param(ns + 'hand_state/publish_rate')

        self.rate = rospy.Rate(publish_rate)
        self.n_sec = 0.01

        self.rand_traj_points = [0.0, 0.0, 0.0, 0.0, 0.0, 
                            0.0, 0.0, 0.0, 0.0, 0.0]

    def callback(self):
        self.traj = Trajectory(ns=self.action_ns, joint_names=self.joint_list, goal_time_tol=1.0, timeout=0.001)
        self.rand_traj_points = [self.thumb_ffb, self.index_ffb, self.middle_ffb, self.ring_ffb, self.pinky_ffb,
                            self.thumb_buzz, self.index_buzz, self.middle_buzz, self.ring_buzz, self.pinky_buzz]
        self.traj.add_point(self.rand_traj_points, self.n_sec)
        self.traj.start()
        self.traj.wait()
        self.traj.clear()

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

        self.reset = config["reset"]

        return config    
     
    def run(self):
        # Init dynamic config before starting the callback/subscriber
        srv = Server(HapticSliderConfig, self._dyn_config_callback) 
        r = rospy.Rate(30)

        while not rospy.is_shutdown():
            self.callback()
            r.sleep()  

        if rospy.is_shutdown():
            self.traj.stop()
            return
        
if __name__== '__main__':
    node = SGHapticFeedback()
    node.run()  