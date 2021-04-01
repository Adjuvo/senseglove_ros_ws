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
    hap_pub = rospy.Publisher('/senseglove/0/lh/joint_trajectory_controller/command', JointTrajectory, queue_size=5)
    if rospy.has_param('/senseglove/0/lh/controller/trajectory/joints'):
        joint_list = rospy.get_param('/senseglove/0/lh/controller/trajectory/joints')

    while not rospy.is_shutdown():
        hap_cmd = JointTrajectory()
        hap_cmd.header = Header()
        hap_cmd.header.stamp = rospy.Time.now()
        hap_cmd.joint_names = joint_list
        print("joint list: ", joint_list[0], ", ", joint_list[1], ", ", joint_list[2], ", ", joint_list[3], ", ", joint_list[4], ", ", len(joint_list))
        point = JointTrajectoryPoint()
        point.effort = [100, 100, 100, 100, 100, 0, 0, 0, 0, 0]# what you will!
        hap_cmd.points.append(point)
        hap_pub.publish(hap_cmd)
        rospy.sleep(0.5)
        # [thumb_brake, index_brake, middle_brake, ring_brake, pinky_brake, thumb_cmc, index_mcp,
        #   middle_mcp, ring_mcp, pinky_mcp]
    if rospy.is_shutdown():
        return

    rospy.spin()