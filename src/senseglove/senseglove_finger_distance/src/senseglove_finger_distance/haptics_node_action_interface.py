import rospy
import sys
from std_msgs.msg import Header
import actionlib

from copy import copy

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)


class Trajectory(object):
    def __init__(self, ns='', joint_names=['empty'], goal_time_tol=1):
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

    def wait(self, timeout=15.0):
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

    rand_traj_points = [100, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # what you will!
    while not rospy.is_shutdown():
        traj = Trajectory(action_ns, joint_list, 0.01)
        traj.add_point(rand_traj_points, n_sec)
        traj.start()
        traj.wait()
        rate.sleep()
        # [thumb_brake, index_brake, middle_brake, ring_brake, pinky_brake, thumb_cmc, index_mcp,
        #   middle_mcp, ring_mcp, pinky_mcp]
    if rospy.is_shutdown():
        traj.stop()
        return

    rospy.spin()
