#!/usr/bin/env python3
"""
test_ps5_velocity_mapper.py - Comprehensive Unit Tests for PS5 FK Velocity Mapper Node.
"""

import os
import sys
import unittest
from unittest.mock import MagicMock

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

import rclpy
from rclpy.duration import Duration
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray
from ps5_mapper import PS5Mapper, StickAxisLock


class TestStickAxisLock(unittest.TestCase):
    def setUp(self):
        self.lock = StickAxisLock()

    def test_deadzone_clears_lock(self):
        flt_x, flt_y = self.lock.filter(0.05, 0.05, deadzone=0.10)
        self.assertEqual(flt_x, 0.0)
        self.assertEqual(flt_y, 0.0)
        self.assertIsNone(self.lock.locked_axis)

    def test_dominant_y_locks_y(self):
        flt_x, flt_y = self.lock.filter(0.3, 0.8, deadzone=0.10)
        self.assertEqual(flt_x, 0.0)
        self.assertEqual(flt_y, 0.8)
        self.assertEqual(self.lock.locked_axis, 'Y')

    def test_dominant_x_locks_x(self):
        flt_x, flt_y = self.lock.filter(0.8, 0.3, deadzone=0.10)
        self.assertEqual(flt_x, 0.8)
        self.assertEqual(flt_y, 0.0)
        self.assertEqual(self.lock.locked_axis, 'X')


class TestPS5VelocityMapper(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = PS5Mapper()
        self.published_msgs = []
        self.node.cmd_pub.publish = MagicMock(side_effect=lambda msg: self.published_msgs.append(msg))

    def tearDown(self):
        self.node.destroy_node()

    def _create_joy_msg(self, axes=None, buttons=None):
        msg = Joy()
        msg.axes = [0.0] * 8 if axes is None else list(axes)
        msg.buttons = [0] * 13 if buttons is None else list(buttons)
        return msg

    def test_neutral_input_outputs_zeros(self):
        """When controller is in deadzone / neutral, output should be [0, 0, 0, 0, 0, 0]."""
        joy_msg = self._create_joy_msg()
        self.node.joy_callback(joy_msg)
        self.node.control_loop()

        self.assertEqual(self.node.cmd_output, [0, 0, 0, 0, 0, 0])
        self.assertEqual(len(self.published_msgs), 1)
        self.assertIsInstance(self.published_msgs[0], Int16MultiArray)
        self.assertEqual(list(self.published_msgs[0].data), [0, 0, 0, 0, 0, 0])

    def test_left_stick_y_shoulder_pitch(self):
        """Full forward deflection on Left Stick Y should command max +255 on J1."""
        joy_msg = self._create_joy_msg()
        joy_msg.axes[PS5Mapper.LJOY_Y] = 1.0  # Full forward
        self.node.joy_callback(joy_msg)
        self.node.control_loop()

        self.assertEqual(self.node.cmd_output, [0, 255, 0, 0, 0, 0])

    def test_left_stick_x_base_yaw(self):
        """Full deflection on Left Stick X commands J0 with inverted polarity (-255)."""
        joy_msg = self._create_joy_msg()
        joy_msg.axes[PS5Mapper.LJOY_X] = 1.0  # Full left
        self.node.joy_callback(joy_msg)
        self.node.control_loop()

        self.assertEqual(self.node.cmd_output, [-255, 0, 0, 0, 0, 0])

    def test_right_stick_y_default_elbow(self):
        """When RB is unpressed, Right Stick Y controls J2 (Elbow Pitch)."""
        joy_msg = self._create_joy_msg()
        joy_msg.axes[PS5Mapper.RJOY_Y] = -1.0  # Full backward
        self.node.joy_callback(joy_msg)
        self.node.control_loop()

        self.assertEqual(self.node.cmd_output, [0, 0, -255, 0, 0, 0])

    def test_right_stick_wrist_gimbal_layer(self):
        """When RB is held, Right Stick Y controls J3 (Pitch) and X controls J4 (Roll with inverted polarity)."""
        joy_msg = self._create_joy_msg()
        joy_msg.buttons[PS5Mapper.RB] = 1      # Hold RB
        joy_msg.axes[PS5Mapper.RJOY_Y] = 1.0   # J3 Pitch forward
        joy_msg.axes[PS5Mapper.RJOY_X] = -1.0  # J4 Roll right (inverted -> +255)
        self.node.joy_callback(joy_msg)
        self.node.control_loop()

        self.assertEqual(self.node.cmd_output, [0, 0, 0, 255, 255, 0])

    def test_gripper_dpad_left_open(self):
        """D-Pad Left commands constant +128 (Open) on Gripper (J5)."""
        joy_msg = self._create_joy_msg()
        joy_msg.axes[PS5Mapper.DPAD_X] = 1.0  # D-Pad Left
        self.node.joy_callback(joy_msg)
        self.node.control_loop()

        self.assertEqual(self.node.cmd_output, [0, 0, 0, 0, 0, 128])

    def test_gripper_dpad_right_close(self):
        """D-Pad Right commands constant -128 (Close) on Gripper (J5)."""
        joy_msg = self._create_joy_msg()
        joy_msg.axes[PS5Mapper.DPAD_X] = -1.0  # D-Pad Right
        self.node.joy_callback(joy_msg)
        self.node.control_loop()

        self.assertEqual(self.node.cmd_output, [0, 0, 0, 0, 0, -128])

    def test_gripper_polarity_inversion(self):
        """When invert_gripper is True, Left commands -128 and Right commands +128."""
        self.node.set_parameters([rclpy.parameter.Parameter('invert_gripper', rclpy.Parameter.Type.BOOL, True)])
        
        # D-Pad Left
        joy_left = self._create_joy_msg()
        joy_left.axes[PS5Mapper.DPAD_X] = 1.0
        self.node.joy_callback(joy_left)
        self.node.control_loop()
        self.assertEqual(self.node.cmd_output[5], -128)

        # D-Pad Right
        joy_right = self._create_joy_msg()
        joy_right.axes[PS5Mapper.DPAD_X] = -1.0
        self.node.joy_callback(joy_right)
        self.node.control_loop()
        self.assertEqual(self.node.cmd_output[5], 128)

    def test_precision_crawl_mode(self):
        """When LB is held, commands scale by precision_scale (default 0.30 -> ~76)."""
        joy_msg = self._create_joy_msg()
        joy_msg.buttons[PS5Mapper.LB] = 1      # Hold LB
        joy_msg.axes[PS5Mapper.LJOY_Y] = 1.0   # Full deflection on J1
        self.node.joy_callback(joy_msg)
        self.node.control_loop()

        expected = int(round(255 * 0.30))  # 76
        self.assertEqual(self.node.cmd_output[1], expected)

    def test_emergency_stop_latches_to_zero(self):
        """Pressing PS button engages E-Stop and clamps all outputs to 0."""
        # 1. Deflect stick while running
        joy_msg = self._create_joy_msg()
        joy_msg.axes[PS5Mapper.LJOY_Y] = 1.0
        joy_msg.axes[PS5Mapper.DPAD_X] = 1.0
        self.node.joy_callback(joy_msg)
        self.node.control_loop()
        self.assertEqual(self.node.cmd_output[1], 255)
        self.assertEqual(self.node.cmd_output[5], 128)

        # 2. Press PS button
        estop_msg = self._create_joy_msg()
        estop_msg.axes[PS5Mapper.LJOY_Y] = 1.0
        estop_msg.axes[PS5Mapper.DPAD_X] = 1.0
        estop_msg.buttons[PS5Mapper.PS_BTN] = 1
        self.node.joy_callback(estop_msg)
        self.assertTrue(self.node.e_stop_active)
        self.node.control_loop()
        self.assertEqual(self.node.cmd_output, [0, 0, 0, 0, 0, 0])

    def test_watchdog_timeout_zeros_output(self):
        """Watchdog callback should zero output if time exceeds timeout."""
        joy_msg = self._create_joy_msg()
        joy_msg.axes[PS5Mapper.LJOY_Y] = 1.0
        joy_msg.axes[PS5Mapper.DPAD_X] = 1.0
        self.node.joy_callback(joy_msg)
        self.node.control_loop()
        self.assertEqual(self.node.cmd_output[1], 255)
        self.assertEqual(self.node.cmd_output[5], 128)

        # Simulate last joy received 1.0 second ago
        past_time = self.node.get_clock().now() - Duration(seconds=1.0)
        self.node.last_joy_time = past_time

        self.node.watchdog_callback()
        self.assertTrue(self.node.signal_lost)
        self.node.control_loop()
        self.assertEqual(self.node.cmd_output, [0, 0, 0, 0, 0, 0])


if __name__ == '__main__':
    unittest.main()
