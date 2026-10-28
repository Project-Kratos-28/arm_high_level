#!/usr/bin/env python3
"""test_ps5_mapper_rth.py - Unit Tests for PS5Mapper RTH Watchdog and Lockdown logic."""

import os
import sys
import unittest
from unittest.mock import MagicMock

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

import rclpy
from rclpy.time import Time
from ps5_mapper import PS5Mapper


class TestPS5MapperRTHLockdown(unittest.TestCase):
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
        # Mock action client and goal handle
        self.node._cancel_rth_goal = MagicMock(wraps=self.node._cancel_rth_goal)

    def tearDown(self):
        self.node.destroy_node()

    def test_watchdog_cancels_rth_on_signal_loss(self):
        """Verify watchdog_callback aborts RTH when signal drops and rth_active is True."""
        self.node.rth_active = True
        self.node.signal_lost = False
        
        # Simulate last joy received 1.0 second ago (timeout is 0.2s)
        past_time = self.node.get_clock().now() - rclpy.duration.Duration(seconds=1.0)
        self.node.last_joy_time = past_time

        # Trigger watchdog check
        self.node.watchdog_callback()

        self.assertTrue(self.node.signal_lost)
        self.node._cancel_rth_goal.assert_called_once_with(reason="comms_loss")

    def test_watchdog_does_not_cancel_rth_if_signal_intact(self):
        """Verify watchdog does not call cancel if signal is within timeout."""
        self.node.rth_active = True
        self.node.signal_lost = False

        # Simulate last joy received 0.05 seconds ago (timeout is 0.2s)
        recent_time = self.node.get_clock().now() - rclpy.duration.Duration(seconds=0.05)
        self.node.last_joy_time = recent_time

        self.node.watchdog_callback()

        self.assertFalse(self.node.signal_lost)
        self.node._cancel_rth_goal.assert_not_called()

    def test_estop_cancels_rth_with_estop_reason(self):
        """Verify E-Stop press during RTH cancels with reason='estop'."""
        self.node.rth_active = True
        self.node.e_stop_active = False
        self.node.prev_estop_btn_state = 0

        # Simulate rising edge on PS button (1)
        self.node.check_estop_btn(1)

        self.assertTrue(self.node.e_stop_active)
        self.node._cancel_rth_goal.assert_called_once_with(reason="estop")


if __name__ == '__main__':
    unittest.main()
