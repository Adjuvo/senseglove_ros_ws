#!/usr/bin/env python3
import rclpy
from senseglove_interaction.haptics.nova2_vibration_node import Nova2VibrationNode

rclpy.init()
node = Nova2VibrationNode()

# Short contact pulse on thumb tip
# node.contact_pulse(Nova2VibrationNode.MOTOR_INDEX_TIP)

# Alert on both palm motors
node.palm_alert(strength=0.8)

# Fully custom waveform
# node.vibrate(
#     motor=Nova2VibrationNode.MOTOR_INDEX_TIP,
#     amplitude=1.0,
#     frequency=180.0,
#     wave_type=Nova2VibrationNode.WAVE_SINE,
#     sustain_time=1.0,
#     attack_time=0.01,
#     decay_time=0.05,
# )

node.destroy_node()
rclpy.shutdown()
