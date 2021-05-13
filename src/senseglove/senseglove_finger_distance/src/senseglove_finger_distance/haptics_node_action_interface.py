import rospy
import sys
from std_msgs.msg import Header
import actionlib
import numpy as np

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


def main():
    rospy.init_node('senseglove_haptics_node')
    rospy.loginfo("initialize haptics node")
    joint_list = ['empty']
    ns = '/senseglove/0/lh/controller/'
    action_ns = ns + 'trajectory/'
    if rospy.has_param(action_ns + 'joints'):
        joint_list = rospy.get_param(action_ns + 'joints')
    publish_rate = 1
    if rospy.has_param(ns + 'hand_state/publish_rate'):
        publish_rate = rospy.get_param(ns + 'hand_state/publish_rate')

    rate = rospy.Rate(publish_rate)
    n_sec = 0.01

    i = 0
    f = 10  # Hz
    amp = 50  # percentage
    wave = np.linspace(0, np.pi * f, 201)
    rand_traj_points = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # what you will!
    while not rospy.is_shutdown():
        if i >= 201:
            i = 0
        traj = Trajectory(ns=action_ns, joint_names=joint_list, goal_time_tol=1.0, timeout=0.001)
        rand_traj_points[0] = amp * np.sin(wave[i]) + amp
        traj.add_point(rand_traj_points, n_sec)
        traj.start()
        traj.wait()
        rate.sleep()
        traj.clear()
        i += 1

        # [thumb_brake, index_brake, middle_brake, ring_brake, pinky_brake, thumb_cmc, index_mcp,
        #   middle_mcp, ring_mcp, pinky_mcp]
        #   middle_mcp, ring_mcp, pinky_mcp]
    if rospy.is_shutdown():
        traj.stop()
        return

    rospy.spin()
