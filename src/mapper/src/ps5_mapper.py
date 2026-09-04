#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
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
    ROS 2 teleoperation node for PS5 DualSense controller mapping to /arm_cmd (joint commands),
    /arm_target_pose (Cartesian end-effector target for IK), and /joy/set_feedback (haptics).

    Features:
      - Dual Mode Teleoperation (toggled via OPTIONS button):
          * Mode 0 (FK): Joint-by-joint angle control on /arm_cmd.
          * Mode 1 (IK): Cylindrical Cartesian end-effector control (TCP at gripper tip) on /arm_target_pose.
      - World-Space Auto-Leveling for 4-Bevel Differential Wrist:
          * Automatically counter-rotates wrist pitch to maintain gripper parallel to ground (or chosen pitch).
          * Wrist Roll spins purely around the gripper approach vector.
      - Deterministic 50 Hz Position Integration Loop.
      - Dominant-Axis Locking (eliminates diagonal joystick slip-up).
      - Gripper Position Integration on D-Pad Left/Right (Left = Open, Right = Close) in unified /arm_cmd array.
      - Gripper Max Speed Trimming on D-Pad Up/Down (+/-).
      - Live Arm / Cartesian Speed Trimming via Shape Buttons + Triggers:
          * CROSS (Hold)    + RT/LT -> Base Yaw / Azimuth Max Speed (+/-)
          * SQUARE (Hold)   + RT/LT -> Shoulder Pitch / Reach Max Speed (+/-)
          * CIRCLE (Hold)   + RT/LT -> Elbow Pitch / Elevation Max Speed (+/-)
          * TRIANGLE (Hold) + RT/LT -> Wrist Pitch & Roll Max Speed (+/-)
      - Layered Orientation Control (Right Bumper RB toggles Right Stick to Wrist / World Pitch & Roll).
      - Precision Crawl Mode (Left Bumper LB scales speeds to 30%).
      - Software Emergency Stop Lock (PS Button toggles latched halt).
      - Signal Loss Watchdog Timer (holds target positions on disconnect / >0.2s timeout).
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

    # Button indices for PS5 controller on /joy (Linux evdev / joy_node mapping)
    CROSS    = 0  # Bottom (Base Yaw / Azimuth)
    CIRCLE   = 1  # Right  (Elbow Pitch / Elevation)
    TRIANGLE = 2  # Top    (Wrist / World Pitch & Roll)
    SQUARE   = 3  # Left   (Shoulder Pitch / Reach)
    LB       = 4  # Precision Crawl (30%)
    RB       = 5  # Layer Toggle (Wrist Gimbal / World Orientation)
    LT_BTN   = 6
    RT_BTN   = 7
    SHARE    = 8
    OPTIONS  = 9  # Mode Toggle (FK <-> IK)
    PS_BTN   = 10 # Emergency Stop Lock Latch
    LJOY_BTN = 11
    RJOY_BTN = 12

    DEADZONE = 0.1

    # Home positions (radians) — all joints start here on node startup
    HOME_POSITIONS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # Index mapping: [base_yaw, shoulder, elbow, wrist_pitch, wrist_roll, gripper]

    # Software joint limits (min, max) in radians — prevents command windup
    JOINT_LIMITS = [
        (-3.14,  3.14),   # 0: Base Yaw        (±180°)
        (-1.57,  1.57),   # 1: Shoulder Pitch  (±90°)
        (-2.09,  2.09),   # 2: Elbow Pitch     (±120°)
        (-1.57,  1.57),   # 3: Wrist Pitch     (±90°)
        (-3.14,  3.14),   # 4: Wrist Roll      (±180°)
        (-0.35,  1.57),   # 5: Gripper
    ]

    # Cartesian workspace limits (min, max) in meters / radians
    # TODO: Update workspace limits with physical robot arm dimensions
    REACH_LIMITS       = (0.05, 0.60)   # Radial reach (m) from base to TCP
    ELEV_LIMITS        = (-0.30, 0.60)  # Elevation (m) from base to TCP
    AZIMUTH_LIMITS     = (-3.14, 3.14)  # Base azimuth (rad) (±180°)
    WORLD_PITCH_LIMITS = (-1.57, 1.57)  # World pitch (rad) relative to horizon (0=horizontal)
    ROLL_LIMITS        = (-3.14, 3.14)  # Wrist axial roll (rad) (±180°)

    # Initial Cartesian home state (r, theta, z, world_pitch, roll)
    HOME_CARTESIAN = {
        'r': 0.25,
        'theta': 0.0,
        'z': 0.15,
        'world_pitch': 0.0,
        'roll': 0.0
    }

    # Control loop rate
    CONTROL_RATE = 50.0   # Hz
    DT = 1.0 / CONTROL_RATE

    @staticmethod
    def euler_to_quaternion(yaw: float, pitch: float, roll: float) -> tuple[float, float, float, float]:
        """
        Converts Euler angles (yaw=Z, pitch=Y, roll=X) to a quaternion tuple (x, y, z, w).
        Uses Z-Y-X intrinsic sequence (yaw -> pitch -> roll).
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    def __init__(self):
        super().__init__('ps5_mapper')

        self.prev_mode_btn_state = 0                        # IK/FK toggle button state
        self.prev_estop_btn_state = 0                       # EStop button state
        self.e_stop_active = False                          # EStop state

        # Axis-locking filter for Left Stick
        self.left_stick_lock = StickAxisLock()

        # ----- ROS 2 Parameters -----
        self.declare_parameter('deadzone', self.DEADZONE)

        self.declare_parameter('trim_min_bound', 0.02)      # Minimum trim speed clamp          (rad/s)
        self.declare_parameter('trim_max_bound', 0.50)      # Maximum trim speed clamp          (rad/s)
        self.declare_parameter('speed_step', 0.02)          # Increment step for speed trimming (rad/s)

        self.declare_parameter('precision_scale', 0.3)      # Speed multiplier when holding LB  (30%)

        self.declare_parameter('axis_lock_enabled', True)   # Enable dominant-axis locking filter

        self.declare_parameter('watchdog_timeout', 0.2)     # Duration before signal-lost flag
        self.declare_parameter('watchdog_rate', 10.0)       # Watchdog check frequency (Hz)

        # Configurable joint speeds (FK Mode)
        self.declare_parameter('max_base_speed', 0.1)       # Base Yaw max speed                (rad/s)
        self.declare_parameter('max_shoulder_speed', 0.1)   # Shoulder Pitch max speed          (rad/s)
        self.declare_parameter('max_elbow_speed', 0.1)      # Elbow Pitch max speed             (rad/s)
        self.declare_parameter('max_wrist_speed', 0.1)      # Wrist Pitch/Roll max speed        (rad/s)
        self.declare_parameter('max_gripper_speed', 0.1)    # Gripper max speed                 (rad/s)

        # Configurable Cartesian speeds (IK Mode)
        self.declare_parameter('max_reach_speed', 0.10)       # Radial reach max speed          (m/s)
        self.declare_parameter('max_elev_speed', 0.10)        # Vertical elevation max speed    (m/s)
        self.declare_parameter('max_azimuth_speed', 0.20)     # Base azimuth max speed          (rad/s)
        self.declare_parameter('max_world_pitch_speed', 0.20) # World pitch max speed           (rad/s)
        self.declare_parameter('max_roll_speed', 0.30)        # Wrist roll max speed            (rad/s)

        # Dynamic runtime joint max speed values
        self.max_base_speed = self.get_parameter('max_base_speed').value
        self.max_shoulder_speed = self.get_parameter('max_shoulder_speed').value
        self.max_elbow_speed = self.get_parameter('max_elbow_speed').value
        self.max_wrist_speed = self.get_parameter('max_wrist_speed').value
        self.max_gripper_speed = self.get_parameter('max_gripper_speed').value

        # Dynamic runtime Cartesian max speed values
        self.max_reach_speed = self.get_parameter('max_reach_speed').value
        self.max_elev_speed = self.get_parameter('max_elev_speed').value
        self.max_azimuth_speed = self.get_parameter('max_azimuth_speed').value
        self.max_world_pitch_speed = self.get_parameter('max_world_pitch_speed').value
        self.max_roll_speed = self.get_parameter('max_roll_speed').value

        # ----- Target Joint Position State (FK & Gripper) -----
        self.target_positions = list(self.HOME_POSITIONS)

        # ----- Target Cartesian State (IK Mode) -----
        self.target_r = self.HOME_CARTESIAN['r']
        self.target_theta = self.HOME_CARTESIAN['theta']
        self.target_z = self.HOME_CARTESIAN['z']
        self.target_world_pitch = self.HOME_CARTESIAN['world_pitch']
        self.target_roll = self.HOME_CARTESIAN['roll']

        # ----- Shared Input State (written by joy_callback, read by control_loop) -----
        self.lx = 0.0               # Left stick X (filtered)
        self.ly = 0.0               # Left stick Y (filtered)
        self.rx = 0.0               # Right stick X (filtered)
        self.ry = 0.0               # Right stick Y (filtered)
        self.dpad_x = 0.0           # D-Pad X axis (gripper open/close)
        self.lb_held = False        # Left bumper state
        self.rb_held = False        # Right bumper state
        self.is_tuning_speed = False  # True while shape button is held (arm lockout)
        self.signal_lost = False    # Set by watchdog on timeout

        # ----- Speed Trimming Timing State (Arm Joints) -----
        self.last_trim_time = 0.0
        self.trim_held_start_time = 0.0
        self.prev_lt_active = False
        self.prev_rt_active = False

        # ----- Gripper Speed Trimming Timing State (D-Pad Up/Down) -----
        self.gripper_trim_last_time = 0.0
        self.gripper_trim_held_start_time = 0.0
        self.prev_dpad_up = False
        self.prev_dpad_down = False

        # ----- Subscriptions -----
        self.subscription = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10
        )

        self.grip_feedback_subscription = self.create_subscription(
            Float64MultiArray, 'gripper_state', self.grip_feedback_callback, 10
        )

        # TODO: Subscribe to /joint_states for live arm feedback and bumpless mode handoff

        # Arm position commands: [base_yaw, shoulder, elbow, wrist_pitch, wrist_roll, gripper]
        self.publisher = self.create_publisher(
            Float64MultiArray, 'arm_cmd', 10
        )

        # IK target command array for IK Solver: [r, theta, z, world_pitch, roll, gripper]
        self.ik_target_pub = self.create_publisher(
            Float64MultiArray, 'arm_ik_cmd', 10
        )

        # IK Cartesian target pose publisher (for RViz / MoveIt visualization)
        self.target_pose_pub = self.create_publisher(
            PoseStamped, 'arm_target_pose', 10
        )

        self.feedback_publisher = self.create_publisher(
            JoyFeedback, '/joy/set_feedback', 10
        )

        # ----- Timers -----
        # Watchdog (10 Hz)
        self.last_joy_time = None
        watchdog_period = 1.0 / self.get_parameter('watchdog_rate').value
        self.watchdog_timer = self.create_timer(watchdog_period, self.watchdog_callback)

        # Control loop (50 Hz)
        self.control_timer = self.create_timer(self.DT, self.control_loop)

        self.get_logger().info("PS5 Mapper Node started (Dual Mode: FK & IK Cartesian Control).")

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

    def _apply_speed_delta(self, axis_name: str, delta: float, min_s: float, max_s: float):
        """Increments or decrements joint / Cartesian max speed within safe bounds and logs update."""
        if axis_name in ("Base Yaw", "Azimuth"):
            if self.MODE == 0:
                self.max_base_speed = max(min_s, min(max_s, round(self.max_base_speed + delta, 3)))
                val, unit = self.max_base_speed, "rad/s"
            else:
                self.max_azimuth_speed = max(min_s, min(max_s, round(self.max_azimuth_speed + delta, 3)))
                val, unit = self.max_azimuth_speed, "rad/s"
        elif axis_name in ("Shoulder Pitch", "Reach"):
            if self.MODE == 0:
                self.max_shoulder_speed = max(min_s, min(max_s, round(self.max_shoulder_speed + delta, 3)))
                val, unit = self.max_shoulder_speed, "rad/s"
            else:
                self.max_reach_speed = max(0.01, min(0.50, round(self.max_reach_speed + (delta * 0.5), 3)))
                val, unit = self.max_reach_speed, "m/s"
        elif axis_name in ("Elbow Pitch", "Elevation"):
            if self.MODE == 0:
                self.max_elbow_speed = max(min_s, min(max_s, round(self.max_elbow_speed + delta, 3)))
                val, unit = self.max_elbow_speed, "rad/s"
            else:
                self.max_elev_speed = max(0.01, min(0.50, round(self.max_elev_speed + (delta * 0.5), 3)))
                val, unit = self.max_elev_speed, "m/s"
        elif axis_name in ("Wrist", "Pitch & Roll"):
            if self.MODE == 0:
                self.max_wrist_speed = max(min_s, min(max_s, round(self.max_wrist_speed + delta, 3)))
                val, unit = self.max_wrist_speed, "rad/s"
            else:
                self.max_world_pitch_speed = max(min_s, min(max_s, round(self.max_world_pitch_speed + delta, 3)))
                self.max_roll_speed = max(min_s, min(max_s, round(self.max_roll_speed + delta, 3)))
                val, unit = self.max_world_pitch_speed, "rad/s"
        elif axis_name == "Gripper":
            self.max_gripper_speed = max(min_s, min(max_s, round(self.max_gripper_speed + delta, 3)))
            val, unit = self.max_gripper_speed, "rad/s"
        else:
            return

        self.get_logger().info(f"[SPEED TRIM] {axis_name} max speed: {val:.2f} {unit}")

    def check_estop_btn(self, new_state: int):
        if new_state == 1 and self.prev_estop_btn_state == 0:
            self.e_stop_active = not self.e_stop_active
            if self.e_stop_active:
                self.get_logger().error("EMERGENCY STOP ENGAGED! Arm target position locked.")
            else:
                self.get_logger().info("EMERGENCY STOP CLEARED. Normal operation resumed.")
        self.prev_estop_btn_state = new_state

    def check_mode_btn(self, new_state: int):
        if new_state == 1 and self.prev_mode_btn_state == 0:
            self.MODE = 1 - self.MODE
            if self.MODE == 1:
                # TODO: Implement FK -> IK bumpless transition by computing Forward Kinematics
                #       from self.target_positions to initialize Cartesian targets (r, theta, z, world_pitch, roll).
                self.get_logger().info("Switched to IK Mode (Cartesian target pose published to /arm_target_pose).")
            else:
                # TODO: Implement IK -> FK bumpless transition by syncing self.target_positions
                #       from /joint_states feedback before resuming FK joint publishing on /arm_cmd.
                self.get_logger().info("Switched to FK Mode (Joint positions published to /arm_cmd).")
        self.prev_mode_btn_state = new_state

    def joy_callback(self, msg: Joy):
        """Records controller input state. Does NOT publish arm commands (handled by control_loop)."""
        # Validate minimum expected axes and buttons
        if len(msg.axes) <= max(self.DPAD_X, self.DPAD_Y, self.RT, self.RJOY_Y) or len(msg.buttons) <= max(self.PS_BTN, self.OPTIONS, self.RB):
            return

        # Update watchdog heartbeat
        self.last_joy_time = self.get_clock().now()
        self.signal_lost = False

        # 1. Emergency Stop / Motion Lock Latch (PS Button)
        self.check_estop_btn(msg.buttons[self.PS_BTN])

        # 2. Mode toggling via OPTIONS button (rising edge detection)
        self.check_mode_btn(msg.buttons[self.OPTIONS])

        # 3. Record D-Pad X axis for gripper open/close
        self.dpad_x = msg.axes[self.DPAD_X]

        # 4. D-Pad Y axis: Gripper Speed Trimming (Up = increase, Down = decrease)
        dpad_y = msg.axes[self.DPAD_Y]
        dpad_up = (dpad_y > 0.5)
        dpad_down = (dpad_y < -0.5)

        step = self.get_parameter('speed_step').value
        min_s = self.get_parameter('trim_min_bound').value
        max_s = self.get_parameter('trim_max_bound').value
        now_sec = self.get_clock().now().nanoseconds / 1e9

        grip_delta = 0.0
        if dpad_up and not dpad_down:
            grip_delta = step
        elif dpad_down and not dpad_up:
            grip_delta = -step

        if grip_delta != 0.0:
            is_new_press = (dpad_up and not self.prev_dpad_up) or (dpad_down and not self.prev_dpad_down)
            if is_new_press:
                self.gripper_trim_held_start_time = now_sec
                self.gripper_trim_last_time = now_sec
                self._apply_speed_delta("Gripper", grip_delta, min_s, max_s)
            else:
                if (now_sec - self.gripper_trim_held_start_time) > 0.4 and (now_sec - self.gripper_trim_last_time) > 0.15:
                    self.gripper_trim_last_time = now_sec
                    self._apply_speed_delta("Gripper", grip_delta, min_s, max_s)

        self.prev_dpad_up = dpad_up
        self.prev_dpad_down = dpad_down

        # 5. Speed Trimming (Shape Buttons + Triggers)
        cross_held = bool(msg.buttons[self.CROSS])
        square_held = bool(msg.buttons[self.SQUARE])
        circle_held = bool(msg.buttons[self.CIRCLE])
        triangle_held = bool(msg.buttons[self.TRIANGLE])

        self.is_tuning_speed = cross_held or square_held or circle_held or triangle_held

        if self.is_tuning_speed:
            if self.MODE == 0:
                if cross_held:
                    selected_axis = "Base Yaw"
                elif square_held:
                    selected_axis = "Shoulder Pitch"
                elif circle_held:
                    selected_axis = "Elbow Pitch"
                else:
                    selected_axis = "Wrist"
            else:
                if cross_held:
                    selected_axis = "Azimuth"
                elif square_held:
                    selected_axis = "Reach"
                elif circle_held:
                    selected_axis = "Elevation"
                else:
                    selected_axis = "Pitch & Roll"

            lt_val = self.get_trigger_value(msg.axes[self.LT])
            rt_val = self.get_trigger_value(msg.axes[self.RT])
            rt_active = (rt_val > 0.5)
            lt_active = (lt_val > 0.5)

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
                    self._apply_speed_delta(selected_axis, delta, min_s, max_s)
                else:
                    if (now_sec - self.trim_held_start_time) > 0.4 and (now_sec - self.last_trim_time) > 0.15:
                        self.last_trim_time = now_sec
                        self._apply_speed_delta(selected_axis, delta, min_s, max_s)

            self.prev_rt_active = rt_active
            self.prev_lt_active = lt_active
            return  # Don't update stick axes while tuning

        # Reset trigger edge state when shape buttons are released
        self.prev_rt_active = False
        self.prev_lt_active = False

        # 6. Record stick axes and bumper states
        deadzone = self.get_parameter('deadzone').value
        self.lb_held = bool(msg.buttons[self.LB])
        self.rb_held = bool(msg.buttons[self.RB])

        if self.get_parameter('axis_lock_enabled').value:
            self.lx, self.ly = self.left_stick_lock.filter(msg.axes[self.LJOY_X], msg.axes[self.LJOY_Y], deadzone)
        else:
            self.lx = self.apply_deadzone(msg.axes[self.LJOY_X], deadzone)
            self.ly = self.apply_deadzone(msg.axes[self.LJOY_Y], deadzone)

        self.rx = self.apply_deadzone(msg.axes[self.RJOY_X], deadzone)
        self.ry = self.apply_deadzone(msg.axes[self.RJOY_Y], deadzone)

    def control_loop(self):
        """50 Hz control loop: integrates inputs into FK / IK target poses and publishes."""
        dt = self.DT
        speed_mult = self.get_parameter('precision_scale').value if self.lb_held else 1.0

        # Freeze position increments if E-Stop or signal lost
        if not (self.e_stop_active or self.signal_lost):

            # Freeze arm increments during speed tuning, but allow gripper
            if not self.is_tuning_speed:
                if self.MODE == 0:
                    # --- FK Mode: Joint Position Integration ---
                    if not self.rb_held:
                        # Default Reach Mode: Right Stick Y controls Elbow
                        self.target_positions[0] += self.lx * self.max_base_speed * speed_mult * dt      # Base Yaw
                        self.target_positions[1] += self.ly * self.max_shoulder_speed * speed_mult * dt  # Shoulder Pitch
                        self.target_positions[2] += self.ry * self.max_elbow_speed * speed_mult * dt     # Elbow Pitch
                    else:
                        # Wrist Gimbal Mode (RB held): Right Stick controls Wrist Pitch (Y) & Roll (X)
                        self.target_positions[0] += self.lx * self.max_base_speed * speed_mult * dt      # Base Yaw
                        self.target_positions[1] += self.ly * self.max_shoulder_speed * speed_mult * dt  # Shoulder Pitch
                        self.target_positions[3] += self.ry * self.max_wrist_speed * speed_mult * dt     # Wrist Pitch
                        self.target_positions[4] += self.rx * self.max_wrist_speed * speed_mult * dt     # Wrist Roll

                elif self.MODE == 1:
                    # --- IK Mode: Cylindrical Coordinates Integration ---
                    # Left Stick: Base Azimuth (X) and Radial Reach (Y)
                    self.target_theta += self.lx * self.max_azimuth_speed * speed_mult * dt
                    self.target_r += self.ly * self.max_reach_speed * speed_mult * dt

                    if not self.rb_held:
                        # Translation Mode: Right Stick Y controls Vertical Elevation (Z)
                        self.target_z += self.ry * self.max_elev_speed * speed_mult * dt
                    else:
                        # Orientation Layer (RB held): Right Stick controls World Pitch (Y) & Axial Roll (X)
                        self.target_world_pitch += self.ry * self.max_world_pitch_speed * speed_mult * dt
                        self.target_roll += self.rx * self.max_roll_speed * speed_mult * dt

                    # Clamp Cartesian targets to workspace boundaries
                    self.target_r = max(self.REACH_LIMITS[0], min(self.REACH_LIMITS[1], self.target_r))
                    self.target_z = max(self.ELEV_LIMITS[0], min(self.ELEV_LIMITS[1], self.target_z))
                    self.target_theta = max(self.AZIMUTH_LIMITS[0], min(self.AZIMUTH_LIMITS[1], self.target_theta))
                    self.target_world_pitch = max(
                        self.WORLD_PITCH_LIMITS[0], min(self.WORLD_PITCH_LIMITS[1], self.target_world_pitch)
                    )
                    self.target_roll = max(self.ROLL_LIMITS[0], min(self.ROLL_LIMITS[1], self.target_roll))

                    # Compute Cartesian Coordinates (TCP / Gripper tip relative to Base)
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = 'base_link'
                    pose_msg.pose.position.x = self.target_r * math.cos(self.target_theta)
                    pose_msg.pose.position.y = self.target_r * math.sin(self.target_theta)
                    pose_msg.pose.position.z = self.target_z

                    # Compute Auto-Leveling Orientation Quaternion
                    qx, qy, qz, qw = self.euler_to_quaternion(
                        self.target_theta, self.target_world_pitch, self.target_roll
                    )
                    pose_msg.pose.orientation.x = qx
                    pose_msg.pose.orientation.y = qy
                    pose_msg.pose.orientation.z = qz
                    pose_msg.pose.orientation.w = qw

                    # Publish Cartesian Pose for visualization (RViz / MoveIt)
                    self.target_pose_pub.publish(pose_msg)

                    # Publish unified IK command array [r, theta, z, world_pitch, roll, gripper] to /arm_ik_cmd
                    ik_msg = Float64MultiArray()
                    ik_msg.data = [
                        float(self.target_r),
                        float(self.target_theta),
                        float(self.target_z),
                        float(self.target_world_pitch),
                        float(self.target_roll),
                        float(self.target_positions[5])
                    ]
                    self.ik_target_pub.publish(ik_msg)

            # Gripper integration (always active, even during arm speed tuning)
            if self.dpad_x > 0.5:
                self.target_positions[5] += self.max_gripper_speed * speed_mult * dt    # Open
            elif self.dpad_x < -0.5:
                self.target_positions[5] -= self.max_gripper_speed * speed_mult * dt    # Close

        # Clamp all joints to limits to prevent command windup
        for i, (lo, hi) in enumerate(self.JOINT_LIMITS):
            self.target_positions[i] = max(lo, min(hi, self.target_positions[i]))

        # Mode-Gated Publishing:
        # FK Mode: ps5_mapper directly publishes [J0..J4, Gripper] to /arm_cmd
        # IK Mode: ps5_mapper published to /arm_ik_cmd above; the IK Solver will publish to /arm_cmd
        if self.MODE == 0:
            cmd_msg = Float64MultiArray()
            cmd_msg.data = list(self.target_positions)
            self.publisher.publish(cmd_msg)

    def watchdog_callback(self):
        """Monitors joystick heartbeat; sets signal_lost flag if communication drops."""
        if self.last_joy_time is None:
            return

        elapsed_sec = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
        timeout = self.get_parameter('watchdog_timeout').value

        if elapsed_sec > timeout and not self.signal_lost:
            self.signal_lost = True
            self.get_logger().warn(
                f"Joy input timed out ({elapsed_sec:.2f}s > {timeout}s). Position held."
            )

    def grip_feedback_callback(self, msg: Float64MultiArray):
        """Receives gripper feedback and publishes haptic rumble commands to DualSense."""
        if len(msg.data) < 2:
            return

        is_gripping = bool(msg.data[1])
        self.get_logger().info(f"Gripper Feedback: {'Gripping' if is_gripping else 'Not Gripping'}")

        # Left Motor (heavy low-frequency rumble)
        left_msg = JoyFeedback()
        left_msg.type = JoyFeedback.TYPE_RUMBLE
        left_msg.id = 0
        left_msg.intensity = float(msg.data[1])
        self.feedback_publisher.publish(left_msg)

        # Right Motor (light high-frequency buzz)
        right_msg = JoyFeedback()
        right_msg.type = JoyFeedback.TYPE_RUMBLE
        right_msg.id = 1
        right_msg.intensity = float(msg.data[1])
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
