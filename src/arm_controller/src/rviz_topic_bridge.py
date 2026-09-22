#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class RvizTopicBridge(Node):
    """
    Bridges commanded arm positions from /arm_cmd (published by ps5_mapper or ik_solver_node)
    to /joint_states for robot_state_publisher and live RViz visualization.
    """

    JOINT_NAMES = [
        "base_yaw_joint",
        "shoulder_joint",
        "elbow_joint",
        "wrist_pitch_joint",
        "wrist_roll_joint",
        "gripper_joint",
    ]

    def __init__(self):
        super().__init__('rviz_topic_bridge')

        # Joint state publisher
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Commanded joint positions subscriber
        self.arm_cmd_sub = self.create_subscription(
            Float64MultiArray, 'arm_cmd', self.arm_cmd_callback, 10
        )

        self.get_logger().info("Rviz Topic Bridge started (/arm_cmd -> /joint_states).")

    def arm_cmd_callback(self, msg: Float64MultiArray):
        """Immediately converts and publishes /arm_cmd messages onto /joint_states."""
        if len(msg.data) < 6:
            self.get_logger().warn(
                f"Received /arm_cmd with {len(msg.data)} elements, expected 6.",
                throttle_duration_sec=2.0,
            )
            return

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.header.frame_id = 'base_link'
        js.name = list(self.JOINT_NAMES)
        js.position = [float(x) for x in msg.data[:6]]

        self.joint_state_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = RvizTopicBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
