import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String


class ServoMonitor(Node):

    def __init__(self):
        super().__init__('servo_monitor')

        # Subscribe to servo angle
        self.angle_sub = self.create_subscription(
            Float32,
            '/servo/angle',
            self.angle_callback,
            10
        )

        # Subscribe to grip status
        self.grip_sub = self.create_subscription(
            String,
            '/servo/grip',
            self.grip_callback,
            10
        )

        self.get_logger().info(
            'Servo monitor started'
        )

    def angle_callback(self, msg):
        self.get_logger().info(
            f'Angle: {msg.data:.1f}°'
        )

    def grip_callback(self, msg):
        self.get_logger().info(
            f'Grip: {msg.data}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = ServoMonitor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()