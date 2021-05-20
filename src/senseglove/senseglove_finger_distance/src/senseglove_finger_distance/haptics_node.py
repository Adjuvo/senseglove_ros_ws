import rospy
from senseglove_shared_resources.msg import SenseGloveState, FingerDistanceFloats
import time
from std_msgs.msg import Time
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


def main():
    rospy.init_node('senseglove_haptics_node')
    rospy.loginfo("initialize haptics node")
    hap_pub = rospy.Publisher('/senseglove/0/lh/controller/trajectory/command', JointTrajectory, queue_size=5)
    joint_list = ['empty']
    if rospy.has_param('/senseglove/0/lh/controller/trajectory/joints'):
        joint_list = rospy.get_param('/senseglove/0/lh/controller/trajectory/joints')
    publish_rate = 1
    if rospy.has_param('/senseglove/0/lh/controller/hand_state/publish_rate'):
        publish_rate = rospy.get_param('/senseglove/0/lh/controller/hand_state/publish_rate')

    rate = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():
        hap_cmd = JointTrajectory()
        hap_cmd.header = Header()
        hap_cmd.header.stamp = rospy.Time.now()
        hap_cmd.joint_names = joint_list
        print("joint list: ", hap_cmd.header.stamp)
        point = JointTrajectoryPoint()
        point.positions = [100, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # what you will!
        point.time_from_start = rospy.Duration.from_sec(0.001)
        hap_cmd.points.append(point)
        hap_pub.publish(hap_cmd)
        rate.sleep()
        # [thumb_brake, index_brake, middle_brake, ring_brake, pinky_brake, thumb_cmc, index_mcp,
        #   middle_mcp, ring_mcp, pinky_mcp]
    if rospy.is_shutdown():
        return

    rospy.spin()
