#!/usr/bin/env python3
"""
Nova 2 custom waveform vibration node

Publishes Nova2WaveformCommand messages to the hardware interface subscriber
Commands are fire-and-forget: publish once per haptic event

Topic: /senseglove/glove{SERIAL}/{rh|lh}/vibration_waveform
"""

import rclpy
from rclpy.node import Node
from senseglove_msgs.msg import Nova2WaveformCommand


class Nova2VibrationNode(Node):
    # Motor location
    MOTOR_THUMB_TIP = Nova2WaveformCommand.MOTOR_THUMB_TIP
    MOTOR_INDEX_TIP = Nova2WaveformCommand.MOTOR_INDEX_TIP
    MOTOR_PALM_INDEX = Nova2WaveformCommand.MOTOR_PALM_INDEX
    MOTOR_PALM_PINKY = Nova2WaveformCommand.MOTOR_PALM_PINKY

    # Waveform type
    WAVE_SINE = Nova2WaveformCommand.WAVE_SINE
    WAVE_SQUARE = Nova2WaveformCommand.WAVE_SQUARE
    WAVE_SAW_UP = Nova2WaveformCommand.WAVE_SAW_UP
    WAVE_SAW_DOWN = Nova2WaveformCommand.WAVE_SAW_DOWN
    WAVE_TRIANGLE = Nova2WaveformCommand.WAVE_TRIANGLE
    WAVE_NOISE = Nova2WaveformCommand.WAVE_NOISE

    def __init__(self, node_name='nova2_vibration_node'):
        super().__init__(node_name)

        self.declare_parameter('glove_serial', '03008')
        self.declare_parameter('side', 'rh')

        serial = self.get_parameter('glove_serial').value
        side = self.get_parameter('side').value

        topic = f'/senseglove/glove{serial}/{side}/vibration_waveform'
        self._pub = self.create_publisher(Nova2WaveformCommand, topic, 10)

        self.get_logger().info(f'Nova2 vibration node publishing to: {topic}')

    def vibrate(
        self,
        motor: int,
        amplitude: float = 1.0,
        frequency: float = 180.0,
        sustain_time: float = 0.15,
        wave_type: int = Nova2WaveformCommand.WAVE_SINE,
        attack_time: float = 0.01,
        decay_time: float = 0.04,
        frequency_end: float = 0.0,
        frequency_switch_time: float = 0.0,
        frequency_switch_factor: float = 1.0,
        repeat_amount: int = 1,
        infinite: bool = False,
    ):
        """Send a single fire-and-forget waveform command to one Nova 2 motor.

        Args:
            motor:                  Target motor
            amplitude:              0.0 - 1.0. Vibration intensity
            frequency:              10 - 500 Hz. Optimal for Nova 2 LRA is ~180 Hz
            sustain_time:           0.0 - 1.0 s. Duration at full amplitude
            wave_type:              Waveform shape
            attack_time:            0.0 - 1.0 s, Ramp-up duration
            decay_time:             0.0 - 1.0 s, Ramp-down duration
            frequency_end:          End frequency for a sweep. Defaults to frequency.
            frequency_switch_time:  0.0 - 1.0, Fraction of effect at which freq jumps,
                                    Set to 0 to disable frequency switching
            frequency_switch_factor: 1.0 - 3.0, Frequency multiplier at switch point
            repeat_amount:          1 - 100, How many times to repeat the waveform
            infinite:               Continuous playback. Overrides repeat_amount
        """
        msg = Nova2WaveformCommand()
        msg.motor_location = motor
        msg.amplitude = float(amplitude)
        msg.frequency_start = float(frequency)
        msg.frequency_end = float(
            frequency_end) if frequency_end > 0.0 else float(frequency)
        msg.wave_type = wave_type
        msg.attack_time = float(attack_time)
        msg.sustain_time = float(sustain_time)
        msg.decay_time = float(decay_time)
        msg.frequency_switch_time = float(frequency_switch_time)
        msg.frequency_switch_factor = float(frequency_switch_factor)
        msg.repeat_amount = int(repeat_amount)
        msg.infinite = bool(infinite)
        self._pub.publish(msg)

    def contact_pulse(self, motor: int, strength: float = 5.0):
        """Short sharp contact pulse"""
        self.vibrate(
            motor=motor,
            amplitude=strength,
            frequency=180.0,
            wave_type=self.WAVE_SINE,
            attack_time=0.01,
            sustain_time=0.15,
            decay_time=0.04,
        )

    def palm_alert(self, strength: float = 0.8):
        """Simultaneous pulse on both palm motors"""
        self.vibrate(motor=self.MOTOR_PALM_INDEX, amplitude=strength,
                     frequency=180.0, sustain_time=0.15)
        self.vibrate(motor=self.MOTOR_PALM_PINKY, amplitude=strength,
                     frequency=180.0, sustain_time=0.15)


def main(args=None):
    rclpy.init(args=args)
    node = Nova2VibrationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
