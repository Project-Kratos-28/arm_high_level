#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JoyFeedback
from std_msgs.msg import Float64MultiArray


class StickAxisLock:
    """
    Filters 2-axis joystick input by locking onto the dominant axis once moved outside
    the deadzone. Prevents accidental diagonal cross-talk (e.g. moving X while pushing Y).
    The lock resets when the stick returns to the deadzone center.
    """

    def __init__(self):
        self.locked_axis = None  # None, 'X', or 'Y'

    def filter(self, x: float, y: float, deadzone: float) -> tuple[float, float]:
        abs_x = abs(x)
        abs_y = abs(y)

        # Reset lock when stick is centered inside the deadzone
        if abs_x <= deadzone and abs_y <= deadzone:
            self.locked_axis = None
            return 0.0, 0.0

        # Lock onto dominant axis upon first stroke outside deadzone
        if self.locked_axis is None:
            if abs_y >= abs_x:
                self.locked_axis = 'Y'
            else:
                self.locked_axis = 'X'

        # Pass only the locked axis, strictly suppressing the other
        if self.locked_axis == 'Y':
            return 0.0, (y if abs_y > deadzone else 0.0)
        else:
            return (x if abs_x > deadzone else 0.0), 0.0


class PS5Mapper(Node):
    """
    ROS 2 teleoperation node for PS5 DualSense controller mapping to /arm_cmd and /joy/set_feedback.

    Features:
      - Dominant-Axis Locking (eliminates diagonal joystick slip-up)
      - Constant Speed Gripper on D-Pad (Left = Open, Right = Close)
      - Live Joint Speed Trimming via Shape Buttons + Triggers:
          * CROSS (Hold)    + RT/LT -> Base Yaw Speed (+/-)
          * SQUARE (Hold)   + RT/LT -> Shoulder Pitch Speed (+/-)
          * CIRCLE (Hold)   + RT/LT -> Elbow Pitch Speed (+/-)
          * TRIANGLE (Hold) + RT/LT -> Wrist Speed (+/-)
          * Tap for single step (±0.02 rad/s) or hold for smooth auto-repeat ramping.
          * Motion locked during speed tuning for safety.
      - Layered Macro / Wrist Control (Right Bumper RB toggles Right Stick to Wrist Gimbal)
      - Precision Crawl Mode (Left Bumper LB scales speed to 30%)
      - Software Emergency Stop Lock (PS Button toggles latched halt)
      - Signal Loss Watchdog Timer (auto-stops on disconnect / >0.2s timeout)
    """

    MODE = 0  # 0: FK, 1: IK

    # Axis indices for PS5 controller on /joy
    LJOY_X = 0
    LJOY_Y = 1
    LT     = 2
    RJOY_X = 3
    RJOY_Y = 4
    RT     = 5
    DPAD_X = 6
    DPAD_Y = 7

    # Button indices for PS5 controller on /joy
    CROSS    = 0
    CIRCLE   = 1
    SQUARE   = 2
    TRIANGLE = 3
    LB       = 4
    RB       = 5
    LT_BTN   = 6
    RT_BTN   = 7
    SHARE    = 8
    OPTIONS  = 9
    PS_BTN   = 10
    LJOY_BTN = 11
    RJOY_BTN = 12

    DEADZONE = 0.1

    def __init__(self):
        super().__init__('ps5_mapper')

        self.prev_mode_btn_state = 0                        # IK/FK toggle button state
        self.prev_estop_btn_state = 0                       # EStop button state.
        self.e_stop_active = False                          # EStop state

        # Axis-locking filter for Left Stick
        self.left_stick_lock = StickAxisLock()              # Only for JoyY since it ontrols two unrelated joints.

        # Configurable node parameters
        self.declare_parameter('deadzone', self.DEADZONE)   # No axis value gets through unless greater than this.

        self.declare_parameter('gripper_scale', 0.1)        # Constant gripper velocity         (rad/s)
        self.declare_parameter('min_speed', 0.02)           # Minimum joint speed clamp         (rad/s)
        self.declare_parameter('max_speed', 0.50)           # Maximum joint speed clamp         (rad/s)
        self.declare_parameter('speed_step', 0.02)          # Increment step for speed trimming (rad/s)

        self.declare_parameter('precision_scale', 0.3)      # Speed multiplier when holding LB  (30%)

        self.declare_parameter('axis_lock_enabled', True)   # Enable dominant-axis locking filter

        self.declare_parameter('watchdog_timeout', 0.2)     # Duration before auto-stop on signal loss
        self.declare_parameter('watchdog_rate', 10.0)       # Watchdog check frequency (Hz)

        # Configureable through controller AND ROS2 CLI.
        self.declare_parameter('base_speed', 0.1)           # Base Yaw velocity                 (rad/s)
        self.declare_parameter('shoulder_speed', 0.1)       # Shoulder Pitch velocity           (rad/s)
        self.declare_parameter('elbow_speed', 0.1)          # Elbow Pitch velocity              (rad/s)
        self.declare_parameter('wrist_speed', 0.1)          # Wrist Pitch/Roll velocity         (rad/s)


        # Dynamic runtime joint speed values
        self.base_speed = self.get_parameter('base_speed').value
        self.shoulder_speed = self.get_parameter('shoulder_speed').value
        self.elbow_speed = self.get_parameter('elbow_speed').value
        self.wrist_speed = self.get_parameter('wrist_speed').value

        # Speed trimming timing state
        self.last_trim_time = 0.0
        self.trim_held_start_time = 0.0
        self.prev_lt_active = False
        self.prev_rt_active = False

        # Subscription to controller inputs
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        # Gripper force & angle feedback subscription
        self.grip_feedback_subscription = self.create_subscription(
            Float64MultiArray,
            'gripper_state',
            self.grip_feedback_callback,
            10
        )

        # Publisher for arm commands: [base_yaw, shoulder, elbow, wrist_pitch, wrist_roll, gripper]
        self.publisher = self.create_publisher(
            Float64MultiArray,
            'arm_cmd',
            10
        )

        # Publisher for DualSense haptic rumble feedback
        self.feedback_publisher = self.create_publisher(
            JoyFeedback,
            '/joy/set_feedback',
            10
        )

        # Watchdog safety state
        self.last_joy_time = None
        self.is_stopped = True
        watchdog_period = 1.0 / self.get_parameter('watchdog_rate').value
        self.watchdog_timer = self.create_timer(watchdog_period, self.watchdog_callback)

        self.get_logger().info("PS5 Mapper Node started.")

    def apply_deadzone(self, value: float, threshold: float = None) -> float:
        """Applies a standard deadzone filter to an axis value."""
        if threshold is None:
            threshold = self.get_parameter('deadzone').value
        return value if abs(value) > threshold else 0.0

    def get_trigger_value(self, raw_axis_val: float) -> float:
        """
        Normalizes PS5 analog trigger axis from [-1.0, 1.0] (1.0=unpressed, -1.0=fully pressed)
        to [0.0, 1.0] (0.0=unpressed, 1.0=fully pressed).
        """
        normalized = (1.0 - raw_axis_val) / 2.0
        deadzone = self.get_parameter('deadzone').value
        if normalized <= deadzone:
            return 0.0
        return min(1.0, normalized)

    def _apply_speed_delta(self, joint_name: str, delta: float, min_s: float, max_s: float):

        """Increments or decrements joint speed within safe bounds and logs the update."""

        if joint_name == "Base Yaw":
            self.base_speed = max(min_s, min(max_s, round(self.base_speed + delta, 3)))
            new_val = self.base_speed
        elif joint_name == "Shoulder Pitch":
            self.shoulder_speed = max(min_s, min(max_s, round(self.shoulder_speed + delta, 3)))
            new_val = self.shoulder_speed
        elif joint_name == "Elbow Pitch":
            self.elbow_speed = max(min_s, min(max_s, round(self.elbow_speed + delta, 3)))
            new_val = self.elbow_speed
        elif joint_name == "Wrist":
            self.wrist_speed = max(min_s, min(max_s, round(self.wrist_speed + delta, 3)))
            new_val = self.wrist_speed
        else:
            return

        self.get_logger().info(f"[SPEED TRIM] {joint_name} speed: {new_val:.2f} rad/s")

    def check_estop_btn(self, new_state: int):
        if new_state == 1 and self.prev_estop_btn_state == 0:
            self.e_stop_active = not self.e_stop_active
            if self.e_stop_active:
                self.get_logger().error("EMERGENCY STOP ENGAGED! All arm motions locked.")
            else:
                self.get_logger().info("EMERGENCY STOP CLEARED. Normal operation resumed.")
        self.prev_estop_btn_state = new_state

    def check_mode_btn(self, new_state: int):
        if new_state == 1 and self.prev_mode_btn_state == 0:
            self.MODE = 1 - self.MODE
            if self.MODE == 1:
                self.get_logger().warn("Switched to IK mode (IK Cartesian mappings pending configuration).")
            else:
                self.get_logger().info("Switched to FK mode.")
        self.prev_mode_btn_state = new_state

    def joy_callback(self, msg: Joy):
        """Processes incoming controller inputs and publishes arm velocity commands."""
        # Validate minimum expected axes and buttons
        if len(msg.axes) <= max(self.DPAD_X, self.RT, self.RJOY_Y) or len(msg.buttons) <= max(self.PS_BTN, self.OPTIONS, self.RB):
            return

        # 1. Emergency Stop / Motion Lock Latch (PS Button)
        self.check_estop_btn(msg.buttons[self.PS_BTN])

        if self.e_stop_active:
            # Command immediate zero velocities while E-Stop is latched
            stop_msg = Float64MultiArray()
            stop_msg.data = [0.0] * 6
            self.publisher.publish(stop_msg)
            self.last_joy_time = self.get_clock().now()
            self.is_stopped = True
            return

        # 2. Mode toggling via OPTIONS button (rising edge detection)
        self.check_mode_btn(msg.buttons[self.OPTIONS])

        # 3. Gripper on D-Pad (Constant Speed: Left = Open, Right = Close)
        gripper_scale = self.get_parameter('gripper_scale').value
        dpad_x = msg.axes[self.DPAD_X]
        if dpad_x > 0.5:
            gripper_cmd = gripper_scale       # Open
        elif dpad_x < -0.5:
            gripper_cmd = -gripper_scale     # Close
        else:
            gripper_cmd = 0.0

        # 4. Joint Speed Trimming (Shape Buttons + Triggers)
        cross_held = bool(msg.buttons[self.CROSS])       # Base Yaw
        square_held = bool(msg.buttons[self.SQUARE])     # Shoulder Pitch
        circle_held = bool(msg.buttons[self.CIRCLE])     # Elbow Pitch
        triangle_held = bool(msg.buttons[self.TRIANGLE]) # Wrist

        is_tuning_speed = cross_held or square_held or circle_held or triangle_held

        if is_tuning_speed:
            # Identify selected joint
            if cross_held:
                selected_joint = "Base Yaw"
            elif square_held:
                selected_joint = "Shoulder Pitch"
            elif circle_held:
                selected_joint = "Elbow Pitch"
            else:
                selected_joint = "Wrist"

            lt_val = self.get_trigger_value(msg.axes[self.LT])
            rt_val = self.get_trigger_value(msg.axes[self.RT])
            rt_active = (rt_val > 0.5)  # Increase speed
            lt_active = (lt_val > 0.5)  # Decrease speed

            step = self.get_parameter('speed_step').value
            min_s = self.get_parameter('min_speed').value
            max_s = self.get_parameter('max_speed').value
            now_sec = self.get_clock().now().nanoseconds / 1e9

            delta = 0.0
            if rt_active and not lt_active:
                delta = step
            elif lt_active and not rt_active:
                delta = -step

            if delta != 0.0:
                is_new_press = (rt_active and not self.prev_rt_active) or (lt_active and not self.prev_lt_active)
                if is_new_press:
                    self.trim_held_start_time = now_sec
                    self.last_trim_time = now_sec
                    self._apply_speed_delta(selected_joint, delta, min_s, max_s)
                else:
                    # Auto-repeat after 0.4s initial delay, then every 0.15s
                    if (now_sec - self.trim_held_start_time) > 0.4 and (now_sec - self.last_trim_time) > 0.15:
                        self.last_trim_time = now_sec
                        self._apply_speed_delta(selected_joint, delta, min_s, max_s)

            self.prev_rt_active = rt_active
            self.prev_lt_active = lt_active

            # Safety Lockout: Zero all arm joints during speed tuning; gripper remains controllable
            cmd_msg = Float64MultiArray()
            cmd_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, gripper_cmd]
            self.publisher.publish(cmd_msg)
            self.last_joy_time = self.get_clock().now()
            self.is_stopped = False
            return

        # Reset trigger edge state when shape buttons are released
        self.prev_rt_active = False
        self.prev_lt_active = False

        deadzone = self.get_parameter('deadzone').value
        lb_held = bool(msg.buttons[self.LB])
        rb_held = bool(msg.buttons[self.RB])

        # Speed scaling multiplier: LB enables precision crawl speed (30%)
        speed_mult = self.get_parameter('precision_scale').value if lb_held else 1.0

        cmd_msg = Float64MultiArray()

        if self.MODE == 0:  # FK Mode
            # Filter stick inputs
            if self.get_parameter('axis_lock_enabled').value:
                lx, ly = self.left_stick_lock.filter(msg.axes[self.LJOY_X], msg.axes[self.LJOY_Y], deadzone)
            else:
                lx = self.apply_deadzone(msg.axes[self.LJOY_X], deadzone)
                ly = self.apply_deadzone(msg.axes[self.LJOY_Y], deadzone)

            rx = self.apply_deadzone(msg.axes[self.RJOY_X], deadzone)
            ry = self.apply_deadzone(msg.axes[self.RJOY_Y], deadzone)

            # Left Stick controls Base Yaw (X) and Shoulder Pitch (Y)
            base_yaw = lx * self.base_speed * speed_mult
            shoulder = ly * self.shoulder_speed * speed_mult

            if not rb_held:
                # Default Reach Mode: Right Stick Y controls Elbow Pitch (RJoy_X is inactive)
                elbow = ry * self.elbow_speed * speed_mult
                wrist_pitch = 0.0
                wrist_roll = 0.0
            else:
                # Wrist Gimbal Mode (RB held): Right Stick controls Wrist Pitch (Y) and Wrist Roll (X)
                elbow = 0.0
                wrist_pitch = ry * self.wrist_speed * speed_mult
                wrist_roll = rx * self.wrist_speed * speed_mult

            # Layout: [base_yaw, shoulder, elbow, wrist_pitch, wrist_roll, gripper]
            cmd_msg.data = [base_yaw, shoulder, elbow, wrist_pitch, wrist_roll, gripper_cmd]

        elif self.MODE == 1:  # IK Mode (Placeholder)
            # Arm joints remain zeroed until IK controls are defined; gripper remains active
            cmd_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, gripper_cmd]

        # Publish command and update watchdog timestamp
        self.publisher.publish(cmd_msg)
        self.last_joy_time = self.get_clock().now()
        self.is_stopped = False

    def watchdog_callback(self):
        """Monitors joystick heartbeat; publishes zero velocities if joystick communication is lost."""
        if self.last_joy_time is None or self.is_stopped:
            return

        elapsed_sec = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
        timeout = self.get_parameter('watchdog_timeout').value

        if elapsed_sec > timeout:
            stop_msg = Float64MultiArray()
            stop_msg.data = [0.0] * 6
            self.publisher.publish(stop_msg)
            self.is_stopped = True
            self.get_logger().warn(
                f"Joy input timed out ({elapsed_sec:.2f}s > {timeout}s). Arm stopped."
            )

    def grip_feedback_callback(self, msg: Float64MultiArray):
        """Receives gripper feedback and publishes haptic rumble commands to DualSense."""
        self.get_logger().info(f"Gripper Feedback: {"Gripping" if msg.data[1] else "Not Gripping"}")
        
        # Left Motor (heavy low-frequency rumble)
        left_msg = JoyFeedback()
        left_msg.type = JoyFeedback.TYPE_RUMBLE
        left_msg.id = 0
        left_msg.intensity = msg.data[1]
        self.feedback_publisher.publish(left_msg)

        # Right Motor (light high-frequency buzz)
        right_msg = JoyFeedback()
        right_msg.type = JoyFeedback.TYPE_RUMBLE
        right_msg.id = 1
        right_msg.intensity = msg.data[1]
        self.feedback_publisher.publish(right_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PS5Mapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()