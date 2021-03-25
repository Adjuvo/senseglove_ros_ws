import rospy
from senseglove_shared_resources.msg import SenseGloveState, FingerDistances
from finger_distance_calibration import Calibration
from math import sqrt, pow

class FingerTipHandler:
    def __init__(self, glove_nr=1, finger_nrs=[3, 7, 11, 15, 19]):
        self.finger_nrs = finger_nrs
        self.finger_tips = [FingerTipVector() for i in self.finger_nrs]
        rospy.Subscriber("/senseglove_" + str(glove_nr) + "/senseglove_states", SenseGloveState,
                         callback=self.callback)
        self.pub = rospy.Publisher("senseglove_" + str(glove_nr) + "/finger_distances", FingerDistances, queue_size=10)

    def distance_publish(self):
        self.calibration = Calibration("default")

        # If calibration already on param server, load it
        if rospy.has_param('~offset_calibration') and rospy.has_param('~length_calibration'):
            rospy.loginfo("Found calibration data on server")
            self.calibration = Calibration("from_param_server")
            self.calibration.offset_calibration = rospy.get_param('~offset_calibration')
            self.calibration.length_calibration = rospy.get_param('~length_calibration')
        else:
            rospy.logwarn("No calibration data found, using defaults")

        finger_distance_message = FingerDistances()
        finger_distance_message.thumb_index = (self.finger_tips[0] - self.finger_tips[1]).magnitude()
        finger_distance_message.thumb_middle = (self.finger_tips[0] - self.finger_tips[2]).magnitude()
        finger_distance_message.thumb_ring = (self.finger_tips[0] - self.finger_tips[3]).magnitude()
        finger_distance_message.thumb_pinky = (self.finger_tips[0] - self.finger_tips[4]).magnitude()
        self.pub.publish(finger_distance_message)

    def callback(self, data):

        for i in range(len(self.finger_nrs)):
            self.finger_tips[i].x = data.finger_tip_positions[i].x
            self.finger_tips[i].y = data.finger_tip_positions[i].y
            self.finger_tips[i].z = data.finger_tip_positions[i].z
        self.distance_publish()


class FingerTipVector:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, other):
        return FingerTipVector(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return FingerTipVector(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, other):
        return self.x * other.x + self.y * other.y + self.z * other.z

    def magnitude(self):
        return sqrt(pow(self.x, 2) + pow(self.y, 2) + pow(self.z, 2))


def main(glove_nr):
    rospy.init_node('senseglove_finger_distance_node')
    rospy.loginfo("initialize finger distance node")
    FingerTipHandler(glove_nr=glove_nr)

    while not rospy.is_shutdown():
        rospy.sleep(0.5)

    if rospy.is_shutdown():
        return

    rospy.spin()
