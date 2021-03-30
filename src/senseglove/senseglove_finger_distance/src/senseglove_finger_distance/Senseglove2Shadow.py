#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Float64
from geometry_msgs.msg import WrenchStamped, Vector3Stamped
from shadow_ros_control.msg import FingerTipPositions
from senseglove_ros.msg import SenseGloveTipPositions
from senseglove_ros.msg import SenseGloveFeedback

from senseglove_shared_resources.srv import Calibrate
from finger_distance_calibration import Calibration


class SenseGloveShadowControl:

    def __init__(self):

        # Start node
        rospy.init_node('senseglove_shadow_control')


        """ Position mapping """

        # Start finger tip command publisher
        self.shadow_cmd_pub = rospy.Publisher('finger_tip_position_cmd', FingerTipPositions, queue_size=10)

        # Start thumb joint angles publishers
        self.ns= rospy.get_namespace()
        if self.ns!='/rh/' and self.ns!='/lh/':
            self.ns='/rh/' # default to right hand if not specified
        self.pub_thj1 = rospy.Publisher('/sh_'+self.ns[1:3]+'_thj1_position_controller/command', Float64, queue_size=10)
        self.pub_thj2 = rospy.Publisher('/sh_'+self.ns[1:3]+'_thj2_position_controller/command', Float64, queue_size=10)
        self.pub_thj4 = rospy.Publisher('/sh_'+self.ns[1:3]+'_thj4_position_controller/command', Float64, queue_size=10)
        self.pub_thj5 = rospy.Publisher('/sh_'+self.ns[1:3]+'_thj5_position_controller/command', Float64, queue_size=10)

        # Start senseglove tip position subscriber
        rospy.Subscriber('senseglove/tip_positions', SenseGloveTipPositions, callback=self.map_fingers)

        # Calibration: setup default
        self.calibration = Calibration("default")

        # If calibration already on param server, load it
        if rospy.has_param('~offset_calibration') and rospy.has_param('~length_calibration'):
            rospy.loginfo("Found calibration data on server")
            self.calibration = Calibration("from_param_server")
            self.calibration.offset_calibration = rospy.get_param('~offset_calibration')
            self.calibration.length_calibration = rospy.get_param('~length_calibration')
        else:
            rospy.logwarn("No calibration data found, using defaults")

        # Declare calibration service
        self.calib_srv = rospy.Service("~calibrate", Calibrate, self.calibrate_service)
        self.calibrating = False

        # Shadow data
        self.shadow_finger_length = 96  # mm. All fingers are same length.
        self.shadow_offset = np.array([33.0, 0.0, 95.0])  # mm. Offset between palm frame and ff base


        """ Force mapping """

        self.biotacs = [0, 0, 0, 0]
        self.optoforces = [0, 0, 0, 0]

        # Start Senseglove haptic feedback publisher
        self.sg_brake_pub = rospy.Publisher('senseglove/brake_cmd', SenseGloveFeedback, queue_size=10)

        # Start Biotac subscribers if right hand
        if self.ns == '/rh/':
            self.min_biotac = rospy.get_param('/rh/min_biotac', default=500)
            self.max_biotac = rospy.get_param('/rh/max_biotac', default=650)
            rospy.loginfo("Providing feedback based on BioTac readings between {} and {}".format(self.min_biotac, self.max_biotac))
            rospy.Subscriber('/biotac_th_force', Vector3Stamped, self.write_force_biotac, 0) # thumb
            rospy.Subscriber('/biotac_in_force', Vector3Stamped, self.write_force_biotac, 1) # index finger
            rospy.Subscriber('/biotac_mi_force', Vector3Stamped, self.write_force_biotac, 2) # middle finger
            rospy.Subscriber('/biotac_ri_force', Vector3Stamped, self.write_force_biotac, 3) # ring finger

        # Start Optoforce subscribers and load settings if left hand
        if self.ns == '/lh/':
            self.min_optoforce = rospy.get_param('/lh/min_optoforce', default=0.05)
            self.max_optoforce = rospy.get_param('/lh/max_optoforce', default=0.1)
            rospy.loginfo("Providing feedback based on Optoforce readings between {} and {}".format(self.min_optoforce, self.max_optoforce))
            rospy.Subscriber('/optoforce_0', WrenchStamped, self.write_force_optoforce, 0)  # ring finger
            rospy.Subscriber('/optoforce_1', WrenchStamped, self.write_force_optoforce, 1)  # middle finger
            rospy.Subscriber('/optoforce_2', WrenchStamped, self.write_force_optoforce, 2)  # index finger
            rospy.Subscriber('/optoforce_3', WrenchStamped, self.write_force_optoforce, 3)  # thumb

        # Make sure senseglove is relieved of forces when this node closes
        rospy.on_shutdown(self.write_force_zero)

        rospy.loginfo('SenseGlove Shadow Control ready')

    def calibrate_service(self, call):
        # Stop publishing commands & feedback
        self.calibrating = True
        self.write_force_zero()

        old_calib = self.calibration
        if call.name is None:
            call.name = "Unnamed_" + str(rospy.get_time())
        self.calibration = Calibration(call.name)
        result = self.calibration.run_interactive_calibration()

        if not result:
            # Failed; Restore previous calibration
            self.calibration = old_calib

        # Resume publishing
        self.calibrating = False
        return result

    def map_fingers(self, senseglove_tip_positions):
        if not self.calibrating:

            # Send latest feedback to glove
            self.pub_feedback()

            # Construct commands
            finger_cmds = []

            for i, data in enumerate([senseglove_tip_positions.index, senseglove_tip_positions.middle, senseglove_tip_positions.ring]):

                # Extract msg data into array and initiate commands
                finger_cmds.append(np.array([data.x, data.y, data.z]))

                # Extract calibrated offset from commands
                finger_cmds[i] -= self.calibration.offset_calibration[i]

                # Scale commands using finger lengths from calibration
                finger_cmds[i] *= (self.shadow_finger_length/self.calibration.length_calibration[i])

                # Add Shadow's palm frame offset
                finger_cmds[i] += self.shadow_offset

                # Convert to meters
                finger_cmds[i] /= 1000.0

            # Construct commands msg for index, middle and ring finger
            cmd_msg = FingerTipPositions()

            for i, cmd in enumerate([cmd_msg.index, cmd_msg.middle, cmd_msg.ring]):
                cmd.x = finger_cmds[i][0]
                cmd.y = finger_cmds[i][1]
                cmd.z = finger_cmds[i][2]

            # publish Cartesian shadow commands
            self.shadow_cmd_pub.publish(cmd_msg)

            # Linearly map joint angles of thumb to Shadow
            th_data = senseglove_tip_positions.thumb
            self.pub_thj1.publish(0.5 * th_data.x - 0.7)
            self.pub_thj2.publish(0.5 * th_data.y )
            self.pub_thj4.publish(th_data.z + 1.5)
            self.pub_thj5.publish(0.1 * (th_data.x + th_data.y))

    def write_force_optoforce(self, message, index):
        force_norm_optoforce = np.sqrt(message.wrench.force.x**2 + message.wrench.force.y**2 + message.wrench.force.z**2)

        self.optoforces[index] = force_norm_optoforce

    def write_force_biotac(self, message, index):
        force_norm_biotac = np.sqrt((message.vector.x)**2 + (message.vector.y)**2 + (message.vector.z**2))

        self.biotacs[index] = force_norm_biotac

    def write_force_zero(self):
        for i in range(len(self.optoforces)):
            self.optoforces[i] = 0
        for j in range(len(self.biotacs)):
            self.biotacs[j] = 0
        self.pub_feedback()

    def pub_feedback(self):
        brake_msg = SenseGloveFeedback()

        if self.ns == '/lh/':
            brake_msg.ring = self.force_to_brake(self.optoforces[0], self.min_optoforce, self.max_optoforce)
            brake_msg.middle = self.force_to_brake(self.optoforces[1], self.min_optoforce, self.max_optoforce)
            brake_msg.index = self.force_to_brake(self.optoforces[2], self.min_optoforce, self.max_optoforce)
            brake_msg.thumb = self.force_to_brake(self.optoforces[3], self.min_optoforce, self.max_optoforce)

        if self.ns == '/rh/':
            brake_msg.thumb = self.force_to_brake(self.biotacs[0], self.min_biotac, self.max_biotac)
            brake_msg.index = self.force_to_brake(self.biotacs[1], self.min_biotac, self.max_biotac)
            brake_msg.middle = self.force_to_brake(self.biotacs[2], self.min_biotac, self.max_biotac)
            brake_msg.ring = self.force_to_brake(self.biotacs[3], self.min_biotac, self.max_biotac)

        self.sg_brake_pub.publish(brake_msg)

    def force_to_brake(self, force, min_force, max_force):
        if force < min_force:
            return 0
        else:
            brake = (force-min_force)/(max_force-min_force)*100
            brake = max(0, min(brake, 100))
            return brake

    def run(self):
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            print('Shutting down')

if __name__ == "__main__":
    myNode = SenseGloveShadowControl()
    myNode.run()
