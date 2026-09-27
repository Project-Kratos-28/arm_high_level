#!/usr/bin/env python3
"""
test_ps5_lights_node.py - Comprehensive Unit & Integration Tests for PS5 Lights Node.
"""

import sys
import os
import unittest
import json
import time

# Add src to path so we can import ps5_lights_node directly
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

import rclpy
from std_msgs.msg import ColorRGBA, String, Float32, Int32, Float64MultiArray
from sensor_msgs.msg import Joy, JoyFeedback

from ps5_lights_node import (
    BaseDualSenseDriver,
    MockDualSenseDriver,
    SysfsDualSenseDriver,
    LightingAnimationEngine,
    PS5LightsNode,
)


class TestLightingAnimationEngine(unittest.TestCase):
    """Tests the 25 Hz deterministic animation math and state generation."""

    def setUp(self):
        class DummyLogger:
            def info(self, msg): pass
            def warn(self, msg, throttle_duration_sec=0): pass
            def error(self, msg): pass

        self.driver = MockDualSenseDriver(DummyLogger())
        self.engine = LightingAnimationEngine(self.driver, update_rate=25.0)

    def test_solid_color(self):
        self.engine.set_color(10, 200, 150)
        self.engine.set_brightness(1.0)
        self.engine.set_effect(LightingAnimationEngine.EFFECT_SOLID)
        self.engine.tick()

        self.assertEqual(self.driver.r, 10)
        self.assertEqual(self.driver.g, 200)
        self.assertEqual(self.driver.b, 150)
        self.assertEqual(self.driver.brightness, 255)

    def test_brightness_scaling(self):
        self.engine.set_color(100, 100, 100)
        self.engine.set_brightness(0.5)
        self.engine.set_effect(LightingAnimationEngine.EFFECT_SOLID)
        self.engine.tick()

        self.assertEqual(self.driver.brightness, 128)

    def test_player_leds(self):
        self.engine.set_player_leds(0b01010)
        self.engine.set_effect(LightingAnimationEngine.EFFECT_SOLID)
        self.engine.tick()
        self.assertEqual(self.driver.player_bitmask, 0b01010)

    def test_rainbow_hsv(self):
        r, g, b = LightingAnimationEngine._hsv_to_rgb(0, 1.0, 1.0)
        self.assertEqual((r, g, b), (255, 0, 0))

        r, g, b = LightingAnimationEngine._hsv_to_rgb(120, 1.0, 1.0)
        self.assertEqual((r, g, b), (0, 255, 0))

        r, g, b = LightingAnimationEngine._hsv_to_rgb(240, 1.0, 1.0)
        self.assertEqual((r, g, b), (0, 0, 255))

    def test_accent_flash(self):
        self.engine.set_color(0, 180, 255)
        self.engine.trigger_accent(255, 255, 255, duration_sec=0.5)
        self.engine.tick()

        # Accent should override the base color with white
        self.assertEqual(self.driver.r, 255)
        self.assertEqual(self.driver.g, 255)
        self.assertEqual(self.driver.b, 255)

        # After expiry, ticks should restore the base color
        time.sleep(0.55)
        self.engine.tick()
        self.assertEqual(self.driver.r, 0)
        self.assertEqual(self.driver.g, 180)
        self.assertEqual(self.driver.b, 255)

    def test_generation_change_flushes_state(self):
        # 1. First tick sets target color
        self.engine.set_color(0, 180, 255)
        self.engine.set_brightness(1.0)
        self.engine.set_effect(LightingAnimationEngine.EFFECT_SOLID)
        self.engine.tick()
        self.assertEqual(self.driver.r, 0)
        self.assertEqual(self.driver.b, 255)

        # 2. Simulate hardware power cycle / reconnect: hardware resets to 0
        self.driver.r = 0
        self.driver.b = 0

        # Normal tick without generation change does not write redundant state
        self.engine.tick()
        self.assertEqual(self.driver.b, 0)

        # 3. Driver reports new generation (controller reconnected)
        self.driver._generation += 1
        self.engine.tick()

        # Engine should flush active state immediately to new hardware
        self.assertEqual(self.driver.r, 0)
        self.assertEqual(self.driver.g, 180)
        self.assertEqual(self.driver.b, 255)

    def test_failed_write_retries(self):
        self.engine.set_color(123, 45, 67)
        self.driver.fail_rgb = True
        self.engine.tick()

        # Write failed, driver didn't update and engine didn't cache success
        self.assertNotEqual((self.driver.r, self.driver.g, self.driver.b), (123, 45, 67))
        self.assertEqual((self.engine._last_r, self.engine._last_g, self.engine._last_b), (-1, -1, -1))

        # Once driver is healthy again, next tick writes successfully
        self.driver.fail_rgb = False
        self.engine.tick()
        self.assertEqual((self.driver.r, self.driver.g, self.driver.b), (123, 45, 67))


class TestPS5LightsNodeROS(unittest.TestCase):
    """Tests the ROS 2 node interfaces, mode tracking, and topic overrides."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = PS5LightsNode()
        # Force Mock driver for tests
        self.node.driver = MockDualSenseDriver(self.node.get_logger())
        self.node.anim.driver = self.node.driver

    def tearDown(self):
        self.node.destroy_node()

    def test_initial_mode_is_fk(self):
        # Default startup state should be FK Mode (Cyan, Player 1)
        self.assertEqual(self.node.arm_mode, 0)
        self.assertEqual(self.node.anim.target_r, 0)
        self.assertEqual(self.node.anim.target_g, 180)
        self.assertEqual(self.node.anim.target_b, 255)
        self.assertEqual(self.node.anim.target_player_leds, 0b00100)

    def test_options_button_toggles_mode(self):
        joy_msg = Joy()
        joy_msg.buttons = [0] * 14

        # Press OPTIONS (button index 9)
        joy_msg.buttons[9] = 1
        self.node.joy_callback(joy_msg)
        self.assertEqual(self.node.arm_mode, 1)  # Switched to IK
        self.assertEqual(self.node.anim.target_g, 255)  # Emerald green

        # Release OPTIONS
        joy_msg.buttons[9] = 0
        self.node.joy_callback(joy_msg)
        self.assertEqual(self.node.arm_mode, 1)

        # Press OPTIONS again -> back to FK
        joy_msg.buttons[9] = 1
        self.node.joy_callback(joy_msg)
        self.assertEqual(self.node.arm_mode, 0)
        self.assertEqual(self.node.anim.target_b, 255)

    def test_estop_button_activates_red_blink(self):
        joy_msg = Joy()
        joy_msg.buttons = [0] * 14

        # Press PS Button (index 10)
        joy_msg.buttons[10] = 1
        self.node.joy_callback(joy_msg)
        self.assertTrue(self.node.e_stop_active)
        self.assertEqual(self.node.anim.target_r, 255)
        self.assertEqual(self.node.anim.target_g, 0)
        self.assertEqual(self.node.anim.target_b, 0)
        self.assertEqual(self.node.anim.target_effect, LightingAnimationEngine.EFFECT_BLINK)

    def test_crawl_and_orientation_player_leds(self):
        joy_msg = Joy()
        joy_msg.buttons = [0] * 14
        joy_msg.axes = [0.0] * 6

        # In FK Mode, center dot is on (0b00100)
        self.assertEqual(self.node.anim.target_player_leds, 0b00100)

        # 1. Hold LB (button 4): Purple lightbar + Leftmost LED (0b00001) turns on -> 0b00101
        joy_msg.buttons[4] = 1
        self.node.joy_callback(joy_msg)
        self.assertTrue(self.node.crawl_active)
        self.assertEqual(self.node.anim.target_r, 180)
        self.assertEqual(self.node.anim.target_b, 255)
        self.assertEqual(self.node.anim.target_player_leds, 0b00101)

        # 2. Also Hold RB (button 5): Rightmost LED (0b10000) turns on -> 0b10101
        joy_msg.buttons[5] = 1
        self.node.joy_callback(joy_msg)
        self.assertTrue(self.node.orientation_layer)
        self.assertEqual(self.node.anim.target_player_leds, 0b10101)

        # 3. Release LB: returns to Cyan, Rightmost + Center LED on -> 0b10100
        joy_msg.buttons[4] = 0
        self.node.joy_callback(joy_msg)
        self.assertFalse(self.node.crawl_active)
        self.assertEqual(self.node.anim.target_r, 0)
        self.assertEqual(self.node.anim.target_b, 255)
        self.assertEqual(self.node.anim.target_player_leds, 0b10100)

        # 4. Release RB: returns to normal FK Center LED -> 0b00100
        joy_msg.buttons[5] = 0
        self.node.joy_callback(joy_msg)
        self.assertEqual(self.node.anim.target_player_leds, 0b00100)

    def test_gripper_load_triggers_accent(self):
        msg = Float64MultiArray()
        msg.data = [0.0, 0.8]  # Load = 0.8 (> 0.5)
        self.node.gripper_callback(msg)
        self.assertIsNotNone(self.node.anim.accent_color)
        self.assertEqual(self.node.anim.accent_color, (100, 230, 255))  # Soft Cyan

    def test_manual_override_set_color(self):
        color_msg = ColorRGBA()
        color_msg.r = 1.0
        color_msg.g = 0.5
        color_msg.b = 0.0
        color_msg.a = 0.8
        self.node.set_color_callback(color_msg)

        self.assertEqual(self.node.manual_override_mode, "manual")
        self.assertEqual(self.node.anim.target_r, 255)
        self.assertEqual(self.node.anim.target_g, 128)
        self.assertEqual(self.node.anim.target_b, 0)
        self.assertAlmostEqual(self.node.anim.target_brightness, 0.8, places=2)

    def test_manual_mode_restore_auto(self):
        # Override to manual
        mode_msg = String()
        mode_msg.data = "estop"
        self.node.set_mode_callback(mode_msg)
        self.assertEqual(self.node.manual_override_mode, "estop")
        self.assertEqual(self.node.anim.target_r, 255)

        # Restore to auto
        mode_msg.data = "auto"
        self.node.set_mode_callback(mode_msg)
        self.assertIsNone(self.node.manual_override_mode)
        self.assertEqual(self.node.anim.target_b, 255)  # Back to FK mode Cyan

    def test_rth_action_status_auto_resets(self):
        from action_msgs.msg import GoalStatusArray, GoalStatus
        # 1. Engage RTH via SHARE button
        joy_msg = Joy()
        joy_msg.buttons = [0] * 14
        joy_msg.axes = [0.0] * 6
        joy_msg.buttons[8] = 1
        self.node.joy_callback(joy_msg)
        self.assertTrue(self.node.rth_active)
        self.assertEqual(self.node.anim.target_r, 255)
        self.assertEqual(self.node.anim.target_g, 215)  # Golden Yellow

        # 2. Action status reports all goals finished (STATUS_SUCCEEDED = 4)
        status_msg = GoalStatusArray()
        goal = GoalStatus()
        goal.status = GoalStatus.STATUS_SUCCEEDED
        status_msg.status_list = [goal]
        self.node.rth_action_status_callback(status_msg)

        # RTH should automatically be cleared and restore FK Cyan
        self.assertFalse(self.node.rth_active)
        self.assertEqual(self.node.anim.target_r, 0)
        self.assertEqual(self.node.anim.target_b, 255)

    def test_rth_home_position_auto_resets(self):
        # 1. Engage RTH
        self.node.rth_active = True
        self.node._update_lighting_from_state()
        self.assertEqual(self.node.anim.target_r, 255)

        # 2. Arm reaches home posture
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [0.0, 0.0, 2.0072, 0.0, 0.0, 0.0]
        self.node.arm_cmd_callback(cmd_msg)

        # Should automatically detect home and reset RTH
        self.assertFalse(self.node.rth_active)
        self.assertEqual(self.node.anim.target_b, 255)

    def test_rth_stick_motion_auto_resets(self):
        # 1. Engage RTH
        self.node.rth_active = True
        self.node._update_lighting_from_state()

        # 2. Operator touches stick
        joy_msg = Joy()
        joy_msg.buttons = [0] * 14
        joy_msg.axes = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.node.joy_callback(joy_msg)

        # Should immediately clear RTH
        self.assertFalse(self.node.rth_active)


if __name__ == '__main__':
    unittest.main()
