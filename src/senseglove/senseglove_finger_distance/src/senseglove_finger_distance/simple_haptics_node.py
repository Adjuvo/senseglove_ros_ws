#!/usr/bin/env python3

"""@package docstring
ROS Node to send simple Force/Vibration feedback on SenseGlove
"""

import rospy
from std_msgs.msg import Float64MultiArray

HAPTIC_TOPIC = "/senseglove/0/rh/senseglove_haptics"

def publisher():

        rospy.init_node("DeltaPositionInstantiator")
        
        hapticLevels = Float64MultiArray()
        hapticLevels.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        pub = rospy.Publisher(HAPTIC_TOPIC, Float64MultiArray, queue_size=1)


        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            pub.publish(hapticLevels)
            r.sleep()

        while rospy.is_shutdown():

            break

if __name__ == "__main__":
    publisher()