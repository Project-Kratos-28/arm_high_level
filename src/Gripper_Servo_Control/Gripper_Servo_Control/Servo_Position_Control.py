import rclpy
from rclpy.node import Node

from Gripper_Servo_Control.servo_control.ST3215 import ST3215

from std_msgs.msg import Float64MultiArray

import sys
import termios
import tty
import select


SERVO_ID = 1
PORT = "/dev/ttyACM0"

# Measured calibration
RAW_AT_0_DEG = 2000
RAW_AT_90_DEG = 787

# Load threshold for gripping
GRIP_LOAD_THRESHOLD = 35


class ServoTestNode(Node):

    def __init__(self):
        super().__init__('servo_test_node')

        # -------------------------------------------------
        # SERVO
        # -------------------------------------------------

        self.servo = ST3215(PORT)

        # Current commanded angle
        self.angle = 0.0

        self.min_angle = -20.0
        self.max_angle = 90.0

        # Keyboard states
        self.Open = 0
        self.Close = 0

        # -------------------------------------------------
        # ROS PUBLISHER
        # -------------------------------------------------

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/gripper_state',
            10
        )

        # -------------------------------------------------
        # CHECK SERVO
        # -------------------------------------------------

        if not self.servo.check_servo(SERVO_ID):
            raise RuntimeError("ST3215 not responding")

        # -------------------------------------------------
        # 50 Hz CONTROL LOOP
        # -------------------------------------------------

        self.timer = self.create_timer(
            0.02,
            self.update_angle
        )

        # -------------------------------------------------
        # TERMINAL
        # -------------------------------------------------

        self.get_logger().info(
            "Servo test started"
        )

        self.get_logger().info(
            "UP    = Open (+0.75°)"
        )

        self.get_logger().info(
            "DOWN  = Close (-0.75°)"
        )

        self.get_logger().info(
            "Grip detection: Load > 35"
        )

        self.get_logger().info(
            "Publishing /gripper_state"
        )

        self.get_logger().info(
            "Format: [angle, grip]"
        )

        self.get_logger().info(
            "grip = 1.0 Gripping"
        )

        self.get_logger().info(
            "grip = 0.0 Not Gripping"
        )

        self.get_logger().info(
            "Press Ctrl+C to exit"
        )

        # Put terminal into cbreak mode
        self.old_terminal_settings = termios.tcgetattr(
            sys.stdin
        )

        tty.setcbreak(
            sys.stdin.fileno()
        )

    # -----------------------------------------------------
    # KEYBOARD INPUT
    # -----------------------------------------------------

    def read_keyboard(self):

        self.Open = 0
        self.Close = 0

        if not select.select(
            [sys.stdin],
            [],
            [],
            0
        )[0]:
            return

        key = sys.stdin.read(1)

        # Arrow keys start with ESC
        if key != '\x1b':
            return

        # Read '['
        if not select.select(
            [sys.stdin],
            [],
            [],
            0.01
        )[0]:
            return

        key2 = sys.stdin.read(1)

        if key2 != '[':
            return

        # Read final arrow key character
        if not select.select(
            [sys.stdin],
            [],
            [],
            0.01
        )[0]:
            return

        key3 = sys.stdin.read(1)

        # UP arrow
        if key3 == 'A':
            self.Open = 1

        # DOWN arrow
        elif key3 == 'B':
            self.Close = 1

    # -----------------------------------------------------
    # SERVO UPDATE + PUBLISH
    # -----------------------------------------------------

    def update_angle(self):

        # Read keyboard
        self.read_keyboard()

        # -------------------------------------------------
        # UPDATE ANGLE
        # -------------------------------------------------

        if self.Open == 1:
            self.angle += 1

        elif self.Close == 1:
            self.angle -= 1

        # -------------------------------------------------
        # CLAMP ANGLE
        # -------------------------------------------------

        self.angle = max(
            self.min_angle,
            min(
                self.angle,
                self.max_angle
            )
        )

        # -------------------------------------------------
        # ANGLE -> RAW SERVO POSITION
        # -------------------------------------------------

        raw_position = int(
            RAW_AT_0_DEG
            + (self.angle / 90.0)
            * (
                RAW_AT_90_DEG
                - RAW_AT_0_DEG
            )
        )

        # -------------------------------------------------
        # COMMAND SERVO
        # -------------------------------------------------

        self.servo.write_angle(
            SERVO_ID,
            raw_position
        )

        # -------------------------------------------------
        # READ SERVO LOAD
        # -------------------------------------------------

        load = self.servo.ReadCurrent(
            SERVO_ID
        )

        # -------------------------------------------------
        # GRIP DETECTION
        #
        # 1.0 = Gripping
        # 0.0 = Not Gripping
        # -------------------------------------------------

        if load > GRIP_LOAD_THRESHOLD:
            grip = 1.0
        else:
            grip = 0.0

        # -------------------------------------------------
        # CREATE FLOAT64 ARRAY
        #
        # [angle, grip]
        # -------------------------------------------------

        msg = Float64MultiArray()

        msg.data = [
            float(self.angle),
            grip
        ]

        # -------------------------------------------------
        # PUBLISH
        # -------------------------------------------------

        self.publisher.publish(
            msg
        )

        # -------------------------------------------------
        # PRINT ONLY WHEN KEY IS PRESSED
        # -------------------------------------------------

        if self.Open == 1 or self.Close == 1:

            if grip == 1.0:
                grip_text = "Gripping"
            else:
                grip_text = "Not Gripping"

            self.get_logger().info(
                f"Angle: {self.angle:.1f}° | "
                f"Load: {load:.1f} | "
                f"Grip: {grip_text}"
            )

    # -----------------------------------------------------
    # CLEANUP
    # -----------------------------------------------------

    def destroy_node(self):

        # Restore terminal
        try:
            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                self.old_terminal_settings
            )
        except Exception:
            pass

        # Close servo port
        try:
            self.servo.close()
        except Exception:
            pass

        super().destroy_node()


# =========================================================
# MAIN
# =========================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = ServoTestNode()

    try:
        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()