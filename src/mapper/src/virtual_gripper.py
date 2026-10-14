#!/usr/bin/env python3
"""
virtual_gripper.py

Autonomous virtual gripper simulation node:
- Subscribes to /arm_cmd to read momentary gripper commands (-1.0 Close, +1.0 Open, 0.0 Stop).
- Simulates physical gripper motion at a constant speed of 0.2 rad/s.
- Simulates grasp contact, load buildup, and grip status.
- Publishes /gripper_feedback [angle, load, gripping] continuously at 50 Hz.
- Runs indefinitely until interrupted (Ctrl+C).
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JoyFeedback


class VirtualGripperNode(Node):
    """ROS 2 node that simulates physical gripper kinematics and load dynamics."""

    def __init__(self):
        super().__init__('virtual_gripper')

        # Parameters
        self.declare_parameter('speed', 0.2)             # Gripper speed constant (rad/s)
        self.declare_parameter('contact_angle', 0.20)    # Grasp contact obstacle angle (rad)
        self.declare_parameter('min_angle', 0.00)        # Fully closed physical limit (rad)
        self.declare_parameter('max_angle', 1.57)        # Fully open physical limit (rad)
        self.declare_parameter('initial_angle', 0.80)    # Starting position (rad)
        self.declare_parameter('update_rate', 50.0)      # Physics loop rate (Hz)

        self.speed = float(self.get_parameter('speed').value)
        self.contact_angle = float(self.get_parameter('contact_angle').value)
        self.min_angle = float(self.get_parameter('min_angle').value)
        self.max_angle = float(self.get_parameter('max_angle').value)
        self.virtual_angle = float(self.get_parameter('initial_angle').value)

        # Simulation state
        self.virtual_load = 10.0
        self.virtual_gripping = 0.0
        self.cmd_gripper = 0.0
        self.rumble_intensity = 0.0

        # ROS 2 Interfaces
        self.feedback_pub = self.create_publisher(Float64MultiArray, '/gripper_feedback', 10)
        self.cmd_sub = self.create_subscription(Float64MultiArray, '/arm_cmd', self.cmd_callback, 10)
        self.rumble_sub = self.create_subscription(JoyFeedback, '/joy/set_feedback', self.rumble_callback, 10)

        # 50 Hz deterministic physics integration loop
        dt = 1.0 / self.get_parameter('update_rate').value
        self.timer = self.create_timer(dt, self.physics_tick)

        # Periodic status logger (1 Hz)
        self.log_timer = self.create_timer(1.0, self.status_logger)

        self.get_logger().info(
            f"Virtual Gripper Node initialized | Speed: {self.speed:.2f} rad/s | "
            f"Range: [{self.min_angle:.2f}, {self.max_angle:.2f}] rad | "
            f"Contact Obstacle: {self.contact_angle:.2f} rad"
        )

    def cmd_callback(self, msg: Float64MultiArray):
        """Records active momentary gripper command from /arm_cmd[5]."""
        if len(msg.data) >= 6:
            self.cmd_gripper = float(msg.data[5])

    def rumble_callback(self, msg: JoyFeedback):
        """Records DualSense haptic rumble intensity published by mapper."""
        if msg.type == JoyFeedback.TYPE_RUMBLE:
            self.rumble_intensity = float(msg.intensity)

    def physics_tick(self):
        """50 Hz physics integration step."""
        dt = 0.02

        # 1. Open Command (+1.0)
        if self.cmd_gripper > 0.5:
            self.virtual_angle = min(self.max_angle, self.virtual_angle + self.speed * dt)
            self.virtual_load = max(10.0, self.virtual_load - 200.0 * dt)
            self.virtual_gripping = 0.0

        # 2. Close Command (-1.0)
        elif self.cmd_gripper < -0.5:
            if self.virtual_angle > self.contact_angle:
                # Free motion closing
                self.virtual_angle = max(self.contact_angle, self.virtual_angle - self.speed * dt)
                self.virtual_load = 20.0
                self.virtual_gripping = 0.0
            else:
                # Contact with object: position holds, load builds up
                self.virtual_angle = self.contact_angle
                self.virtual_load = min(1000.0, self.virtual_load + 250.0 * dt)
                self.virtual_gripping = 1.0 if self.virtual_load > 40.0 else 0.0

        # 3. Neutral / Hold Command (0.0)
        else:
            # Load decays slowly when held
            self.virtual_load = max(10.0, self.virtual_load - 50.0 * dt)
            if self.virtual_load <= 40.0:
                self.virtual_gripping = 0.0

        # Publish /gripper_feedback [angle, load, gripping]
        feedback_msg = Float64MultiArray()
        feedback_msg.data = [
            float(self.virtual_angle),
            float(self.virtual_load),
            float(self.virtual_gripping),
        ]
        self.feedback_pub.publish(feedback_msg)

    def status_logger(self):
        """Prints live operational status once per second."""
        cmd_str = "+1 (OPEN)" if self.cmd_gripper > 0.5 else ("-1 (CLOSE)" if self.cmd_gripper < -0.5 else " 0 (HOLD)")
        grip_str = "GRIPPING" if self.virtual_gripping > 0.5 else "FREE"
        pct = int(round(max(0.0, min(1.0, (self.virtual_angle - self.min_angle) / (self.max_angle - self.min_angle))) * 100))

        self.get_logger().info(
            f"CMD: {cmd_str:10s} | Pos: {self.virtual_angle:.2f} rad ({pct:3d}%) | "
            f"Load: {self.virtual_load:5.1f} | State: {grip_str:8s} | Rumble: {self.rumble_intensity:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = VirtualGripperNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
