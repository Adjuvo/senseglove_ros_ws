import rospy
from senseglove_shared_resources.msg import SenseGloveState, FingerDistanceFloats
from finger_distance_calibration import Calibration
from math import sqrt, pow


class FingerTipHandler:
    def __init__(self, glove_nr=1, finger_nrs=[3, 7, 11, 15, 19]):
        self.finger_nrs = finger_nrs
        self.finger_tips = [FingerTipVector() for i in self.finger_nrs]
        rospy.Subscriber("/senseglove_" + str(glove_nr) + "/senseglove_states", SenseGloveState,
                         callback=self.callback)
        self.pub = rospy.Publisher("senseglove_" + str(glove_nr) + "/finger_distances", FingerDistanceFloats, queue_size=10)

        self.calibration = Calibration("default")

    def apply_calib(self, pinch_value=0.0, pinch_combination=1, mode='normalized'):
        if mode == 'minimum':
            # Return the values so that pinching your fingers results in a finger distance of zero
            return pinch_value - self.calibration.pinch_calibration_min[pinch_combination]
        elif mode == 'normalized':
            # Return normalized finger distance value between 0 and 1
            return (pinch_value - self.calibration.pinch_calibration_min[pinch_combination]) / self.calibration.pinch_calibration_max[pinch_combination]

    def distance_publish(self):
        finger_distance_message = FingerDistanceFloats()
        finger_distance_message.th_ff.data = self.apply_calib((self.finger_tips[0] - self.finger_tips[1]).magnitude(), 0, 'minimum')
        finger_distance_message.th_mf.data = self.apply_calib((self.finger_tips[0] - self.finger_tips[2]).magnitude(), 1, 'minimum')
        finger_distance_message.th_rf.data = self.apply_calib((self.finger_tips[0] - self.finger_tips[3]).magnitude(), 2, 'minimum')
        finger_distance_message.th_lf.data = (self.finger_tips[0] - self.finger_tips[4]).magnitude()
        self.pub.publish(finger_distance_message)

    def callback(self, data):
        if not self.calibration.is_calibrated():
            # If calibration on param server, load it
            if rospy.has_param('~pinch_calibration_min') and rospy.has_param('~pinch_calibration_max'):
                rospy.loginfo("Found calibration data on server")
                self.calibration = Calibration("from_param_server")
                self.calibration.pinch_calibration_min = rospy.get_param('~pinch_calibration_min')
                self.calibration.pinch_calibration_max = rospy.get_param('~pinch_calibration_max')
            else:
                rospy.logwarn_throttle(30, "No calibration data found, using defaults")

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
