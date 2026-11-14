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
        """When controller is in deadzone / neutral, output should be [0, 0, 0, 0, 0, 0, 0]."""
        joy_msg = self._create_joy_msg()
        self.node.joy_callback(joy_msg)
        self.node.control_loop()

        self.assertEqual(self.node.cmd_output, [0, 0, 0, 0, 0, 0, 0])
        self.assertEqual(len(self.published_msgs), 1)
        self.assertIsInstance(self.published_msgs[0], Int16MultiArray)
        self.assertEqual(list(self.published_msgs[0].data), [0, 0, 0, 0, 0, 0, 0])

    def test_left_stick_y_shoulder_pitch(self):
        """Full forward deflection on Left Stick Y should command max +255 on J1."""
        joy_msg = self._create_joy_msg()
        joy_msg.axes[PS5Mapper.LJOY_Y] = 1.0  # Full forward
        self.node.joy_callback(joy_msg)
        self.node.control_loop()

        self.assertEqual(self.node.cmd_output, [0, 255, 0, 0, 0, 0, 0])

    def test_left_stick_x_base_yaw(self):
        """Full deflection on Left Stick X commands J0 with inverted polarity (-255)."""
        joy_msg = self._create_joy_msg()
        joy_msg.axes[PS5Mapper.LJOY_X] = 1.0  # Full left
        self.node.joy_callback(joy_msg)
        self.node.control_loop()

        self.assertEqual(self.node.cmd_output, [-255, 0, 0, 0, 0, 0, 0])

    def test_right_stick_y_default_elbow(self):
        """When RB is unpressed, Right Stick Y controls J2 (Elbow Pitch)."""
        joy_msg = self._create_joy_msg()
        joy_msg.axes[PS5Mapper.RJOY_Y] = -1.0  # Full backward
        self.node.joy_callback(joy_msg)
        self.node.control_loop()

        self.assertEqual(self.node.cmd_output, [0, 0, -255, 0, 0, 0, 0])

    def test_right_stick_wrist_gimbal_layer(self):
        """When RB is held, Right Stick Y controls J3 (Pitch) and X controls J4 (Roll with inverted polarity)."""
        joy_msg = self._create_joy_msg()
        joy_msg.buttons[PS5Mapper.RB] = 1      # Hold RB
        joy_msg.axes[PS5Mapper.RJOY_Y] = 1.0   # J3 Pitch forward
        joy_msg.axes[PS5Mapper.RJOY_X] = -1.0  # J4 Roll right (inverted -> +255)
        self.node.joy_callback(joy_msg)
        self.node.control_loop()

        self.assertEqual(self.node.cmd_output, [0, 0, 0, 255, 255, 0, 0])

    def test_gripper_dpad_left_open(self):
        """D-Pad Left commands constant -1 on Gripper (J5)."""
        joy_msg = self._create_joy_msg()
        joy_msg.axes[PS5Mapper.DPAD_X] = 1.0  # D-Pad Left
        self.node.joy_callback(joy_msg)
        self.node.control_loop()

        self.assertEqual(self.node.cmd_output, [0, 0, 0, 0, 0, -1, 0])

    def test_gripper_dpad_right_close(self):
        """D-Pad Right commands constant +1 on Gripper (J5)."""
        joy_msg = self._create_joy_msg()
        joy_msg.axes[PS5Mapper.DPAD_X] = -1.0  # D-Pad Right
        self.node.joy_callback(joy_msg)
        self.node.control_loop()

        self.assertEqual(self.node.cmd_output, [0, 0, 0, 0, 0, 1, 0])

    def test_gripper_polarity_inversion(self):
        """When invert_gripper is True, Left commands +1 and Right commands -1."""
        self.node.set_parameters([rclpy.parameter.Parameter('invert_gripper', rclpy.Parameter.Type.BOOL, True)])
        
        # D-Pad Left
        joy_left = self._create_joy_msg()
        joy_left.axes[PS5Mapper.DPAD_X] = 1.0
        self.node.joy_callback(joy_left)
        self.node.control_loop()
        self.assertEqual(self.node.cmd_output[5], 1)

        # D-Pad Right
        joy_right = self._create_joy_msg()
        joy_right.axes[PS5Mapper.DPAD_X] = -1.0
        self.node.joy_callback(joy_right)
        self.node.control_loop()
        self.assertEqual(self.node.cmd_output[5], -1)

    def test_actuator_dpad_up_extend(self):
        """D-Pad Up commands +255 (Extend) on Linear Actuator (J6)."""
        joy_msg = self._create_joy_msg()
        joy_msg.axes[PS5Mapper.DPAD_Y] = 1.0  # D-Pad Up
        self.node.joy_callback(joy_msg)
        self.node.control_loop()

        self.assertEqual(self.node.cmd_output, [0, 0, 0, 0, 0, 0, 255])

    def test_actuator_dpad_down_retract(self):
        """D-Pad Down commands -255 (Retract) on Linear Actuator (J6)."""
        joy_msg = self._create_joy_msg()
        joy_msg.axes[PS5Mapper.DPAD_Y] = -1.0  # D-Pad Down
        self.node.joy_callback(joy_msg)
        self.node.control_loop()

        self.assertEqual(self.node.cmd_output, [0, 0, 0, 0, 0, 0, -255])

    def test_actuator_precision_crawl_mode(self):
        """When LB is held, Linear Actuator speed scales by precision_scale (default 0.30 -> ~76)."""
        joy_msg = self._create_joy_msg()
        joy_msg.buttons[PS5Mapper.LB] = 1      # Hold LB
        joy_msg.axes[PS5Mapper.DPAD_Y] = 1.0   # D-Pad Up
        self.node.joy_callback(joy_msg)
        self.node.control_loop()

        expected = int(round(255 * 0.30))  # 76
        self.assertEqual(self.node.cmd_output[6], expected)

    def test_actuator_polarity_inversion(self):
        """When invert_actuator is True, Up commands -255 and Down commands +255."""
        self.node.set_parameters([rclpy.parameter.Parameter('invert_actuator', rclpy.Parameter.Type.BOOL, True)])

        # D-Pad Up
        joy_up = self._create_joy_msg()
        joy_up.axes[PS5Mapper.DPAD_Y] = 1.0
        self.node.joy_callback(joy_up)
        self.node.control_loop()
        self.assertEqual(self.node.cmd_output[6], -255)

        # D-Pad Down
        joy_down = self._create_joy_msg()
        joy_down.axes[PS5Mapper.DPAD_Y] = -1.0
        self.node.joy_callback(joy_down)
        self.node.control_loop()
        self.assertEqual(self.node.cmd_output[6], 255)

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
        joy_msg.axes[PS5Mapper.DPAD_Y] = 1.0
        self.node.joy_callback(joy_msg)
        self.node.control_loop()
        self.assertEqual(self.node.cmd_output[1], 255)
        self.assertEqual(self.node.cmd_output[5], -1)
        self.assertEqual(self.node.cmd_output[6], 255)

        # 2. Press PS button
        estop_msg = self._create_joy_msg()
        estop_msg.axes[PS5Mapper.LJOY_Y] = 1.0
        estop_msg.axes[PS5Mapper.DPAD_X] = 1.0
        estop_msg.axes[PS5Mapper.DPAD_Y] = 1.0
        estop_msg.buttons[PS5Mapper.PS_BTN] = 1
        self.node.joy_callback(estop_msg)
        self.assertTrue(self.node.e_stop_active)
        self.node.control_loop()
        self.assertEqual(self.node.cmd_output, [0, 0, 0, 0, 0, 0, 0])

    def test_watchdog_timeout_zeros_output(self):
        """Watchdog callback should zero output if time exceeds timeout."""
        joy_msg = self._create_joy_msg()
        joy_msg.axes[PS5Mapper.LJOY_Y] = 1.0
        joy_msg.axes[PS5Mapper.DPAD_X] = 1.0
        joy_msg.axes[PS5Mapper.DPAD_Y] = 1.0
        self.node.joy_callback(joy_msg)
        self.node.control_loop()
        self.assertEqual(self.node.cmd_output[1], 255)
        self.assertEqual(self.node.cmd_output[5], -1)
        self.assertEqual(self.node.cmd_output[6], 255)

        # Simulate last joy received 1.0 second ago
        past_time = self.node.get_clock().now() - Duration(seconds=1.0)
        self.node.last_joy_time = past_time

        self.node.watchdog_callback()
        self.assertTrue(self.node.signal_lost)
        self.node.control_loop()
        self.assertEqual(self.node.cmd_output, [0, 0, 0, 0, 0, 0, 0])

    def test_options_button_toggles_ik_mode(self):
        """Pressing OPTIONS button toggles between FK and IK modes with rising-edge latch."""
        self.assertFalse(self.node.ik_mode_active)

        # 1. First press -> toggles to IK
        toggle_msg = self._create_joy_msg()
        toggle_msg.buttons[PS5Mapper.OPTIONS] = 1
        self.node.joy_callback(toggle_msg)
        self.assertTrue(self.node.ik_mode_active)

        # 2. Holding the button in subsequent cycles -> remains in IK
        held_msg = self._create_joy_msg()
        held_msg.buttons[PS5Mapper.OPTIONS] = 1
        self.node.joy_callback(held_msg)
        self.assertTrue(self.node.ik_mode_active)

        # 3. Release button
        release_msg = self._create_joy_msg()
        release_msg.buttons[PS5Mapper.OPTIONS] = 0
        self.node.joy_callback(release_msg)
        self.assertTrue(self.node.ik_mode_active)

        # 4. Second press -> toggles back to FK
        toggle_again = self._create_joy_msg()
        toggle_again.buttons[PS5Mapper.OPTIONS] = 1
        self.node.joy_callback(toggle_again)
        self.assertFalse(self.node.ik_mode_active)

    def test_pseudo_ik_reach_forward_equal_links(self):
        """In IK mode, Left Stick Y forward produces [-0.5, 1.0, -0.5] ratio on [J1, J2, J3]."""
        # Switch to IK mode
        toggle_msg = self._create_joy_msg()
        toggle_msg.buttons[PS5Mapper.OPTIONS] = 1
        self.node.joy_callback(toggle_msg)
        self.assertTrue(self.node.ik_mode_active)

        # Deflect Left Stick Y full forward (+1.0)
        reach_msg = self._create_joy_msg()
        reach_msg.axes[PS5Mapper.LJOY_Y] = 1.0
        self.node.joy_callback(reach_msg)
        self.node.control_loop()

        # Output format: [J0, J1, J2, J3, J4, Gripper, Actuator]
        # J1 (Shoulder): -0.5 * 255 = -128 (pitches down/forward)
        # J2 (Elbow):    +1.0 * 255 = 255 (extends at 2x rate)
        # J3 (Wrist):    -0.5 * 255 = -128 (auto-levels gripper)
        self.assertEqual(self.node.cmd_output[0], 0)
        self.assertEqual(self.node.cmd_output[1], -128)
        self.assertEqual(self.node.cmd_output[2], 255)
        self.assertEqual(self.node.cmd_output[3], -128)
        self.assertEqual(self.node.cmd_output[4], 0)

    def test_pseudo_ik_reach_backward_equal_links(self):
        """In IK mode, Left Stick Y backward produces [+0.5, -1.0, +0.5] ratio on [J1, J2, J3]."""
        # Switch to IK mode
        toggle_msg = self._create_joy_msg()
        toggle_msg.buttons[PS5Mapper.OPTIONS] = 1
        self.node.joy_callback(toggle_msg)

        # Deflect Left Stick Y full backward (-1.0)
        reach_msg = self._create_joy_msg()
        reach_msg.axes[PS5Mapper.LJOY_Y] = -1.0
        self.node.joy_callback(reach_msg)
        self.node.control_loop()

        self.assertEqual(self.node.cmd_output[1], 128)
        self.assertEqual(self.node.cmd_output[2], -255)
        self.assertEqual(self.node.cmd_output[3], 128)

    def test_pseudo_ik_elevation(self):
        """In IK mode, Right Stick Y with RB unpressed drives coordinated elevation."""
        toggle_msg = self._create_joy_msg()
        toggle_msg.buttons[PS5Mapper.OPTIONS] = 1
        self.node.joy_callback(toggle_msg)

        # Right Stick Y forward (+1.0), RB not held
        elev_msg = self._create_joy_msg()
        elev_msg.axes[PS5Mapper.RJOY_Y] = 1.0
        self.node.joy_callback(elev_msg)
        self.node.control_loop()

        # J1: +255 (Shoulder up)
        # J2: -128 (Elbow tucks)
        # J3: -128 (Wrist keeps horizon level)
        self.assertEqual(self.node.cmd_output[1], 255)
        self.assertEqual(self.node.cmd_output[2], -128)
        self.assertEqual(self.node.cmd_output[3], -128)

    def test_pseudo_ik_wrist_manual_override(self):
        """In IK mode, holding RB enables direct manual wrist pitch and roll overrides."""
        toggle_msg = self._create_joy_msg()
        toggle_msg.buttons[PS5Mapper.OPTIONS] = 1
        self.node.joy_callback(toggle_msg)

        # RB held, Right Stick deflected
        wrist_msg = self._create_joy_msg()
        wrist_msg.buttons[PS5Mapper.RB] = 1
        wrist_msg.axes[PS5Mapper.RJOY_Y] = 1.0   # Wrist Pitch
        wrist_msg.axes[PS5Mapper.RJOY_X] = 1.0   # Wrist Roll (invert_j4 = True -> -255)
        self.node.joy_callback(wrist_msg)
        self.node.control_loop()

        self.assertEqual(self.node.cmd_output[1], 0)
        self.assertEqual(self.node.cmd_output[2], 0)
        self.assertEqual(self.node.cmd_output[3], 255)
        self.assertEqual(self.node.cmd_output[4], -255)

    def test_pseudo_ik_precision_crawl(self):
        """In IK mode, holding LB scales all IK outputs by precision_scale (30%)."""
        toggle_msg = self._create_joy_msg()
        toggle_msg.buttons[PS5Mapper.OPTIONS] = 1
        self.node.joy_callback(toggle_msg)

        # LB held, Left Stick Y forward
        crawl_msg = self._create_joy_msg()
        crawl_msg.buttons[PS5Mapper.LB] = 1
        crawl_msg.axes[PS5Mapper.LJOY_Y] = 1.0
        self.node.joy_callback(crawl_msg)
        self.node.control_loop()

        # Scaled by 30%:
        # J1: round(-0.5 * 255 * 0.30) = -38
        # J2: round(1.0 * 255 * 0.30)  = 76
        # J3: round(-0.5 * 255 * 0.30) = -38
        self.assertEqual(self.node.cmd_output[1], -38)
        self.assertEqual(self.node.cmd_output[2], 76)
        self.assertEqual(self.node.cmd_output[3], -38)

    def test_live_trim_shoulder_ratio(self):
        """Holding Square + RT increases shoulder magnitude; LT decreases shoulder magnitude."""
        init_val = float(self.node.get_parameter('ik_reach_shoulder_ratio').value)
        self.assertEqual(init_val, -0.5)

        # 1. Hold Square and press RT -> should increase magnitude to -0.55
        trim_msg = self._create_joy_msg()
        trim_msg.buttons[PS5Mapper.SQUARE] = 1
        trim_msg.buttons[PS5Mapper.RT_BTN] = 1
        self.node.joy_callback(trim_msg)
        self.assertAlmostEqual(float(self.node.get_parameter('ik_reach_shoulder_ratio').value), -0.55)

        # 2. Release RT
        rel_msg = self._create_joy_msg()
        rel_msg.buttons[PS5Mapper.SQUARE] = 1
        self.node.joy_callback(rel_msg)

        # 3. Hold Square and press LT -> should decrease magnitude back to -0.50
        lt_msg = self._create_joy_msg()
        lt_msg.buttons[PS5Mapper.SQUARE] = 1
        lt_msg.buttons[PS5Mapper.LT_BTN] = 1
        self.node.joy_callback(lt_msg)
        self.assertAlmostEqual(float(self.node.get_parameter('ik_reach_shoulder_ratio').value), -0.50)

    def test_live_trim_elbow_ratio(self):
        """Holding Circle + RT (axis) increases elbow magnitude; LT (axis) decreases it."""
        init_val = float(self.node.get_parameter('ik_reach_elbow_ratio').value)
        self.assertEqual(init_val, 1.0)

        # 1. Hold Circle and press RT via axis (<0.0) -> increases to 1.05
        trim_msg = self._create_joy_msg()
        trim_msg.buttons[PS5Mapper.CIRCLE] = 1
        trim_msg.axes[PS5Mapper.RT] = -1.0
        self.node.joy_callback(trim_msg)
        self.assertAlmostEqual(float(self.node.get_parameter('ik_reach_elbow_ratio').value), 1.05)

        # 2. Release RT
        rel_msg = self._create_joy_msg()
        rel_msg.buttons[PS5Mapper.CIRCLE] = 1
        rel_msg.axes[PS5Mapper.RT] = 1.0
        self.node.joy_callback(rel_msg)

        # 3. Hold Circle and press LT via axis (<0.0) -> decreases to 1.00
        lt_msg = self._create_joy_msg()
        lt_msg.buttons[PS5Mapper.CIRCLE] = 1
        lt_msg.axes[PS5Mapper.LT] = -1.0
        self.node.joy_callback(lt_msg)
        self.assertAlmostEqual(float(self.node.get_parameter('ik_reach_elbow_ratio').value), 1.00)

    def test_live_trim_wrist_pitch_ratio(self):
        """Holding Triangle + RT increases wrist magnitude; LT decreases it."""
        init_val = float(self.node.get_parameter('ik_reach_wrist_ratio').value)
        self.assertEqual(init_val, -0.5)

        # 1. Hold Triangle + RT -> increases magnitude to -0.55
        trim_msg = self._create_joy_msg()
        trim_msg.buttons[PS5Mapper.TRIANGLE] = 1
        trim_msg.buttons[PS5Mapper.RT_BTN] = 1
        self.node.joy_callback(trim_msg)
        self.assertAlmostEqual(float(self.node.get_parameter('ik_reach_wrist_ratio').value), -0.55)

        # 2. Release RT
        rel_msg = self._create_joy_msg()
        rel_msg.buttons[PS5Mapper.TRIANGLE] = 1
        self.node.joy_callback(rel_msg)

        # 3. Hold Triangle + LT -> decreases magnitude back to -0.50
        lt_msg = self._create_joy_msg()
        lt_msg.buttons[PS5Mapper.TRIANGLE] = 1
        lt_msg.buttons[PS5Mapper.LT_BTN] = 1
        self.node.joy_callback(lt_msg)
        self.assertAlmostEqual(float(self.node.get_parameter('ik_reach_wrist_ratio').value), -0.50)

    def test_live_trim_requires_modifier(self):
        """Pressing RT/LT without holding Square/Circle/Triangle does not alter ratios."""
        s_val = float(self.node.get_parameter('ik_reach_shoulder_ratio').value)
        e_val = float(self.node.get_parameter('ik_reach_elbow_ratio').value)
        w_val = float(self.node.get_parameter('ik_reach_wrist_ratio').value)

        # Press RT with no face buttons held
        msg = self._create_joy_msg()
        msg.buttons[PS5Mapper.RT_BTN] = 1
        self.node.joy_callback(msg)

        self.assertEqual(float(self.node.get_parameter('ik_reach_shoulder_ratio').value), s_val)
        self.assertEqual(float(self.node.get_parameter('ik_reach_elbow_ratio').value), e_val)
        self.assertEqual(float(self.node.get_parameter('ik_reach_wrist_ratio').value), w_val)


if __name__ == '__main__':
    unittest.main()
