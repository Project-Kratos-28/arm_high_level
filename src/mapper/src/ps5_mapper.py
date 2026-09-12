#!/usr/bin/env python3

import math
import numpy as np
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy, JoyFeedback
from std_msgs.msg import Float64MultiArray


class StickAxisLock:
    """
    Filters 2-axis joystick input by locking onto the dominant axis once moved outside
    the deadzone, while allowing clean dynamic switching when the user deliberately deflects
    the secondary axis. Prevents accidental diagonal cross-talk without locking the user out.
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
        else:
            # Allow clean dynamic transfer if secondary axis becomes noticeably dominant
            if self.locked_axis == 'Y' and abs_x > max(deadzone * 1.5, abs_y * 1.2):
                self.locked_axis = 'X'
            elif self.locked_axis == 'X' and abs_y > max(deadzone * 1.5, abs_x * 1.2):
                self.locked_axis = 'Y'

        # Pass only the locked axis, strictly suppressing the other
        if self.locked_axis == 'Y':
            return 0.0, (y if abs_y > deadzone else 0.0)
        else:
            return (x if abs_x > deadzone else 0.0), 0.0


def _t_mat(rot=None, trans=(0.0, 0.0, 0.0)):
    """Constructs a 4x4 homogeneous transformation matrix."""
    T = np.eye(4)
    if rot is not None:
        T[:3, :3] = rot.as_matrix()
    T[:3, 3] = trans
    return T


class PS5Mapper(Node):
    """
    ROS 2 teleoperation node for PS5 DualSense controller mapping to /arm_cmd (joint commands),
    /arm_ik_cmd (Cartesian targets for MoveIt TRAC-IK solver), /arm_target_pose (RViz visualization),
    and /joy/set_feedback (DualSense haptic rumble).

    Features:
      - Dual-Mode Teleoperation (toggled via OPTIONS button):
          * Mode 0 (FK): Joint-by-joint angle integration published directly to /arm_cmd,
            with continuous mirroring to /arm_fk_sync for solver warm-seeding.
          * Mode 1 (IK): Cylindrical Cartesian control of wrist_center (end of Link 2) published to /arm_ik_cmd.
      - Dynamic Leash Anti-Windup (25 mm limit clamp):
          * Clamps Cartesian targets within 25 mm of the actual reached wrist position when pushing against
            workspace boundaries, preventing the target frame from drifting away and ensuring instant reversal.
      - Bumpless Bidirectional State Synchronization:
          * FK -> IK: Analytical 3D Forward Kinematics to wrist_center initializes Cartesian targets
            (r, theta, z, world_pitch, roll) directly from the current physical joint configuration.
          * IK -> FK: Continuously mirrors solved joint angles from /arm_joint_sync while in IK mode,
            allowing immediate and seamless resumption of joint-level control with zero jump.
      - World-Space Auto-Leveling Orientation:
          * Maintains gripper pitch relative to ground horizon across translations in IK mode.
          * Right Bumper (RB) toggles the Right Stick to control World Pitch and Axial Wrist Roll.
      - Deterministic 50 Hz Position Integration Loop.
      - Dominant-Axis Locking (eliminates accidental diagonal stick cross-talk).
      - Gripper Position Integration on D-Pad Left/Right (Left = Open, Right = Close).
      - Gripper Max Speed Trimming on D-Pad Up/Down (+/-).
      - Live Joint & Cartesian Speed Trimming via Shape Buttons + Triggers (LT/RT):
          * CROSS (Hold)    + RT/LT -> Base Yaw (FK) / Azimuth (IK) Max Speed (+/-)
          * SQUARE (Hold)   + RT/LT -> Shoulder Pitch (FK) / Reach (IK) Max Speed (+/-)
          * CIRCLE (Hold)   + RT/LT -> Elbow Pitch (FK) / Elevation (IK) Max Speed (+/-)
          * TRIANGLE (Hold) + RT/LT -> Wrist Pitch & Roll (FK/IK) Max Speed (+/-)
      - Precision Crawl Mode (Left Bumper LB scales speeds to 30%).
      - Software Emergency Stop Lock (PS Button toggles latched motion halt).
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

    # Button indices for PS5 controller on /joy
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

    # Home positions (radians) — elbow at +2.0072 rad (115° forward from straight-up reference)
    HOME_POSITIONS = [0.0, 0.0, 2.0072, 0.0, 0.0, 0.0]
    # Index mapping: [base_yaw, shoulder, elbow, wrist_pitch, wrist_roll, gripper]

    # Software joint limits (min, max) in radians — prevents command windup
    JOINT_LIMITS = [
        (-3.14,  3.14),   # 0: Base Yaw        (±180°)
        (-1.57,  1.57),   # 1: Shoulder Pitch  (±90°)
        ( 0.00,  2.50),   # 2: Elbow Pitch     (0 to +143.2°) — prevents backward elbow bending
        (-1.57,  1.57),   # 3: Wrist Pitch     (±90°)
        (-3.14,  3.14),   # 4: Wrist Roll      (±180°)
        (-0.35,  1.57),   # 5: Gripper
    ]

    # Kinematic mounting constants for wrist_center relative to base_link
    BASE_PIVOT_X       = 0.043511       # Base yaw rotation axis X (m)
    BASE_PIVOT_Y       = -0.012448      # Base yaw rotation axis Y (m)
    BASE_PIVOT_Z       = 0.03425        # Base yaw joint Z elevation (m)
    ARM_LATERAL_OFFSET = -0.023         # Arm sagittal lateral offset in rotating frame (m)

    # Cartesian workspace limits (min, max) in meters / radians for wrist_center (end of Link 2):
    REACH_LIMITS       = (0.04, 0.98)   # Forward sagittal reach (m) from shoulder to wrist_center
    ELEV_LIMITS        = (-0.45, 1.08)  # Elevation (m) from base to wrist_center
    AZIMUTH_LIMITS     = (-3.14, 3.14)  # Base azimuth (rad) (±180°, 0 = forward, + = CCW, - = CW)
    WORLD_PITCH_LIMITS = (-1.57, 1.57)  # World pitch (rad) relative to horizon (0=horizontal)
    ROLL_LIMITS        = (-3.14, 3.14)  # Wrist axial roll (rad) (±180°)

    # Spherical workspace envelope for wrist_center
    # Model: sphere centered at (r=0, z=SHOULDER_PIVOT_Z) with radius WORKSPACE_RADIUS.
    SHOULDER_PIVOT_Z   = 0.1385         # Exact shoulder joint pivot elevation (m) in base_link
    WORKSPACE_RADIUS   = 0.98           # Spherical workspace radius (m) to wrist_center (physical max: 0.990 m)
    LEASH_MAX_M        = 0.025          # 25 mm max visual lead in 3D (dynamic leash anti-windup)

    # Initial Cartesian home state (r, theta, z, world_pitch, roll) for wrist_center
    # Home posture at q=[0,0,2.0072,0,0,0]: forward reach r=0.4775m, theta=0.0rad (forward), z=0.3315m
    HOME_CARTESIAN = {
        'r': 0.4775,
        'theta': 0.0,
        'z': 0.3315,
        'world_pitch': -0.4363,
        'roll': 0.0
    }

    # Control loop rate
    CONTROL_RATE = 50.0   # Hz
    DT = 1.0 / CONTROL_RATE

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

        # Cached read-only parameters (constant throughout node lifetime)
        self.deadzone = self.get_parameter('deadzone').value
        self.precision_scale = self.get_parameter('precision_scale').value
        self.trim_min_bound = self.get_parameter('trim_min_bound').value
        self.trim_max_bound = self.get_parameter('trim_max_bound').value
        self.speed_step = self.get_parameter('speed_step').value
        self.axis_lock_enabled = self.get_parameter('axis_lock_enabled').value
        self.watchdog_timeout = self.get_parameter('watchdog_timeout').value

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

        # Subscriber for bumpless IK -> FK joint mirroring from ik_solver_node
        self.joint_sync_sub = self.create_subscription(
            Float64MultiArray, 'arm_joint_sync', self.joint_sync_callback, 10
        )

        # Arm position commands: [base_yaw, shoulder, elbow, wrist_pitch, wrist_roll, gripper]
        self.publisher = self.create_publisher(
            Float64MultiArray, 'arm_cmd', 10
        )

        # FK state sync publisher (for continuous warm-seeding of ik_solver_node during Mode 0)
        self.fk_sync_pub = self.create_publisher(
            Float64MultiArray, 'arm_fk_sync', 10
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
            threshold = self.deadzone
        return value if abs(value) > threshold else 0.0

    def get_trigger_value(self, raw_axis_val: float) -> float:
        """
        Normalizes PS5 analog trigger axis from [-1.0, 1.0] (1.0=unpressed, -1.0=fully pressed)
        to [0.0, 1.0] (0.0=unpressed, 1.0=fully pressed).
        """
        normalized = (1.0 - raw_axis_val) / 2.0
        if normalized <= self.deadzone:
            return 0.0
        return min(1.0, normalized)

    def _clamp_to_workspace_sphere(self):
        """
        Enforces that (target_r, target_z) lies inside the spherical kinematic envelope:
            r² + (z - SHOULDER_PIVOT_Z)² ≤ WORKSPACE_RADIUS²
        Also applies the hard floor/ceiling on z and minimum-r floor (base column deadzone).

        This replaces independent rectangle clamping on r and z, preventing
        the IK solver from being commanded into corners that are kinematically
        unreachable (which causes silent stick freezes / position hold).
        """
        # 1. Hard floor/ceiling on z (absolute physical extremes)
        self.target_z = max(self.ELEV_LIMITS[0], min(self.ELEV_LIMITS[1], self.target_z))

        # 2. Spherical ceiling on r given current z
        z_rel = self.target_z - self.SHOULDER_PIVOT_Z
        r_max_at_z = math.sqrt(max(0.0, self.WORKSPACE_RADIUS**2 - z_rel**2))
        r_max_at_z = min(r_max_at_z, self.REACH_LIMITS[1])  # also honour absolute r ceiling

        # 3. Clamp r into [r_min, r_max_at_z]
        self.target_r = max(self.REACH_LIMITS[0], min(r_max_at_z, self.target_r))

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
        """
        Detects rising edge on the PS button to toggle the software Emergency Stop / Motion Lock.
        When active, position increments in control_loop are frozen, holding current arm and gripper position.
        """
        if new_state == 1 and self.prev_estop_btn_state == 0:
            self.e_stop_active = not self.e_stop_active
            if self.e_stop_active:
                self.get_logger().error("EMERGENCY STOP ENGAGED! Arm target position locked.")
            else:
                self.get_logger().info("EMERGENCY STOP CLEARED. Normal operation resumed.")
        self.prev_estop_btn_state = new_state

    def joint_sync_callback(self, msg: Float64MultiArray):
        """
        Callback for /arm_joint_sync topic published by ik_solver_node.
        Continuously mirrors the latest solved joint angles into self.target_positions
        while operating in IK mode (MODE == 1) to enable bumpless IK -> FK transitions.
        """
        if self.MODE == 1 and len(msg.data) >= 5:
            for i in range(5):
                self.target_positions[i] = msg.data[i]

    def check_mode_btn(self, new_state: int):
        """
        Detects rising edge on the OPTIONS button to toggle between Mode 0 (FK) and Mode 1 (IK).
        Executes bumpless handoffs:
          - FK -> IK: Computes 3D Forward Kinematics from current self.target_positions to initialize
                      Cartesian targets (r, theta, z, world_pitch, roll) at the exact physical gripper pose.
          - IK -> FK: Resumes joint publishing from self.target_positions (already synchronized via /arm_joint_sync).
        """
        if new_state == 1 and self.prev_mode_btn_state == 0:
            self.MODE = 1 - self.MODE
            if self.MODE == 1:
                # Bumpless FK -> IK transition: compute Cartesian wrist targets from current joint angles
                r, theta, z, world_pitch, roll = self.compute_wrist_fk(self.target_positions)
                self.target_r = r
                self.target_z = z
                self._clamp_to_workspace_sphere()
                self.target_theta = max(self.AZIMUTH_LIMITS[0], min(self.AZIMUTH_LIMITS[1], theta))
                self.target_world_pitch = max(self.WORLD_PITCH_LIMITS[0], min(self.WORLD_PITCH_LIMITS[1], world_pitch))
                self.target_roll = max(self.ROLL_LIMITS[0], min(self.ROLL_LIMITS[1], roll))
                self.get_logger().info(
                    f"Switched to IK Mode (Bumpless FK->IK sync: r={self.target_r:.3f}m, "
                    f"theta={self.target_theta:.3f}rad, z={self.target_z:.3f}m, pitch={self.target_world_pitch:.3f}rad)."
                )
            else:
                # Bumpless IK -> FK transition: self.target_positions was continuously mirrored via /arm_joint_sync
                self.get_logger().info(
                    f"Switched to FK Mode (Bumpless IK->FK sync: resumed joints "
                    f"{[round(x, 3) for x in self.target_positions[:5]]})."
                )
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

        step = self.speed_step
        min_s = self.trim_min_bound
        max_s = self.trim_max_bound
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

            # Safety Lockout: Zero stick inputs and reset axis lock filter during speed tuning
            self.lx = 0.0
            self.ly = 0.0
            self.rx = 0.0
            self.ry = 0.0
            self.left_stick_lock.locked_axis = None
            self.lb_held = bool(msg.buttons[self.LB])
            self.rb_held = bool(msg.buttons[self.RB])
            return

        # Reset trigger edge state when shape buttons are released
        self.prev_rt_active = False
        self.prev_lt_active = False

        # 6. Record stick axes and bumper states
        deadzone = self.deadzone
        self.lb_held = bool(msg.buttons[self.LB])
        self.rb_held = bool(msg.buttons[self.RB])

        if self.axis_lock_enabled:
            self.lx, self.ly = self.left_stick_lock.filter(msg.axes[self.LJOY_X], msg.axes[self.LJOY_Y], deadzone)
        else:
            self.lx = self.apply_deadzone(msg.axes[self.LJOY_X], deadzone)
            self.ly = self.apply_deadzone(msg.axes[self.LJOY_Y], deadzone)

        self.rx = self.apply_deadzone(msg.axes[self.RJOY_X], deadzone)
        self.ry = self.apply_deadzone(msg.axes[self.RJOY_Y], deadzone)

    def control_loop(self):
        """
        Main 50 Hz deterministic integration and publishing loop.

        Behavior:
          - Evaluates safety interlocks (E-Stop latch, watchdog signal loss timeout).
          - In Mode 0 (FK):
              * Integrates stick inputs into individual joint angles (self.target_positions).
              * Clamps angles to JOINT_LIMITS to prevent command windup.
              * Publishes 6-element Float64MultiArray [J0..J4, gripper] directly to /arm_cmd.
          - In Mode 1 (IK):
              * Integrates stick inputs into cylindrical coordinates (r, theta, z, world_pitch, roll).
              * Clamps Cartesian state to spherical workspace envelope (WORKSPACE_RADIUS, SHOULDER_PIVOT_Z)
                plus azimuth, world_pitch, and roll hard limits.
              * Publishes PoseStamped to /arm_target_pose for live RViz visualization.
              * Publishes 6-element Float64MultiArray [r, theta, z, pitch, roll, gripper] to /arm_ik_cmd
                (commanding ik_solver_node, which solves TRAC-IK and outputs to /arm_cmd).
          - Gripper integration runs across both modes (D-Pad Left = Open, D-Pad Right = Close).
        """
        dt = self.DT
        speed_mult = self.precision_scale if self.lb_held else 1.0

        # Freeze position increments if E-Stop or signal lost
        if not (self.e_stop_active or self.signal_lost):

            # Freeze arm increments during speed tuning, but allow gripper
            if not self.is_tuning_speed:
                if self.MODE == 0:
                    # --- FK Mode: Joint Position Integration ---
                    self.target_positions[0] += self.lx * self.max_base_speed * speed_mult * dt      # Base Yaw
                    self.target_positions[1] += self.ly * self.max_shoulder_speed * speed_mult * dt  # Shoulder Pitch

                    if not self.rb_held:
                        # Default Reach Mode: Right Stick Y controls Elbow
                        self.target_positions[2] += self.ry * self.max_elbow_speed * speed_mult * dt     # Elbow Pitch
                    else:
                        # Wrist Gimbal Mode (RB held): Right Stick controls Wrist Pitch (Y) & Roll (X)
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

                    # Dynamic Leash Anti-Windup against actual solved wrist position (in r, theta, z)
                    r_act, theta_act, z_act, _, _ = self.compute_wrist_fk(self.target_positions)
                    dr = self.target_r - r_act
                    dz = self.target_z - z_act
                    dtheta = self.target_theta - theta_act
                    while dtheta > math.pi:
                        dtheta -= 2.0 * math.pi
                    while dtheta < -math.pi:
                        dtheta += 2.0 * math.pi

                    # Tangential lead along the arc: r * dtheta
                    d_tangential = max(0.10, r_act) * dtheta
                    dist_3d = math.hypot(dr, d_tangential, dz)
                    if dist_3d > self.LEASH_MAX_M:
                        scale = self.LEASH_MAX_M / dist_3d
                        self.target_r = r_act + dr * scale
                        self.target_z = z_act + dz * scale
                        self.target_theta = theta_act + dtheta * scale

                    # Clamp Cartesian targets to spherical workspace envelope
                    self._clamp_to_workspace_sphere()
                    self.target_theta = max(self.AZIMUTH_LIMITS[0], min(self.AZIMUTH_LIMITS[1], self.target_theta))
                    self.target_world_pitch = max(
                        self.WORLD_PITCH_LIMITS[0], min(self.WORLD_PITCH_LIMITS[1], self.target_world_pitch)
                    )
                    self.target_roll = max(self.ROLL_LIMITS[0], min(self.ROLL_LIMITS[1], self.target_roll))

            # Gripper integration (always active, even during arm speed tuning)
            if self.dpad_x > 0.5:
                self.target_positions[5] += self.max_gripper_speed * speed_mult * dt    # Open
            elif self.dpad_x < -0.5:
                self.target_positions[5] -= self.max_gripper_speed * speed_mult * dt    # Close

        # Clamp all joints to limits to prevent command windup
        for i, (lo, hi) in enumerate(self.JOINT_LIMITS):
            self.target_positions[i] = max(lo, min(hi, self.target_positions[i]))

        # Mode-Gated Publishing:
        # FK Mode: ps5_mapper directly publishes [J0..J4, Gripper] to /arm_cmd and /arm_fk_sync
        # IK Mode: ps5_mapper publishes [r, theta, z, world_pitch, roll, gripper] to /arm_ik_cmd
        #          and PoseStamped to /arm_target_pose; ik_solver_node solves TRAC-IK and outputs to /arm_cmd
        if self.MODE == 0:
            cmd_msg = Float64MultiArray()
            cmd_msg.data = list(self.target_positions)
            self.publisher.publish(cmd_msg)
            self.fk_sync_pub.publish(cmd_msg)
        elif self.MODE == 1:
            # Compute Cartesian Coordinates (wrist_center relative to base_link)
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'base_link'
            pose_msg.pose.position.x = (
                self.BASE_PIVOT_X
                + self.target_r * math.sin(self.target_theta)
                + self.ARM_LATERAL_OFFSET * math.cos(self.target_theta)
            )
            pose_msg.pose.position.y = (
                self.BASE_PIVOT_Y
                - self.target_r * math.cos(self.target_theta)
                + self.ARM_LATERAL_OFFSET * math.sin(self.target_theta)
            )
            pose_msg.pose.position.z = self.target_z

            # Compute Auto-Leveling Orientation Quaternion matching physical tool0 frame:
            # R_tool0 = R_z(theta) @ R_x(-world_pitch) @ R_y(roll) @ R_z(pi)
            r_mat = (
                R.from_euler('z', self.target_theta).as_matrix()
                @ R.from_euler('x', -self.target_world_pitch).as_matrix()
                @ R.from_euler('y', self.target_roll).as_matrix()
                @ R.from_euler('z', np.pi).as_matrix()
            )
            qx, qy, qz, qw = R.from_matrix(r_mat).as_quat()
            pose_msg.pose.orientation.x = float(qx)
            pose_msg.pose.orientation.y = float(qy)
            pose_msg.pose.orientation.z = float(qz)
            pose_msg.pose.orientation.w = float(qw)

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

    def watchdog_callback(self):
        """Monitors joystick heartbeat; sets signal_lost flag if communication drops."""
        if self.last_joy_time is None:
            return

        elapsed_sec = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
        timeout = self.watchdog_timeout

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

        intensity = float(msg.data[1])
        for rumble_id in (0, 1):
            fb = JoyFeedback()
            fb.type = JoyFeedback.TYPE_RUMBLE
            fb.id = rumble_id
            fb.intensity = intensity
            self.feedback_publisher.publish(fb)

    def compute_wrist_fk(self, joints: list[float]) -> tuple[float, float, float, float, float]:
        """
        Computes 3D Forward Kinematics for wrist_center (end of Link 2 / wrist pitch axis)
        relative to base_link from joint angles.
        Derived from link origin vectors and rotations in arm_cad.urdf.xacro.

        Args:
            joints: List of joint angles in radians [base_yaw, shoulder, elbow, wrist_pitch, wrist_roll, ...].

        Returns:
            tuple (r, theta, z, world_pitch, roll):
              - r (float): Forward sagittal reach from shoulder to wrist_center (meters).
              - theta (float): Base azimuth angle relative to forward (radians, 0 = forward).
              - z (float): Vertical elevation of wrist_center relative to base_link origin (meters).
              - world_pitch (float): Gripper pitch angle relative to the ground horizon (radians).
              - roll (float): Wrist axial roll angle (radians).
        """
        q = joints

        # Transform in the arm rotating frame (q0 = 0)
        T_arm = _t_mat(trans=(0.0145, 0.0, 0.070))
        T_arm = T_arm @ _t_mat(R.from_euler('x', q[1])) @ _t_mat(trans=(-0.038, 0.0, 0.450))
        T_arm = T_arm @ _t_mat(R.from_euler('x', -2.00719)) @ _t_mat(R.from_euler('x', q[2]))
        T_arm = T_arm @ _t_mat(trans=(0.0005, -0.47751, -0.222701))
        p_arm = T_arm[:3, 3]

        # Forward sagittal reach (arm extends along -Y in rotating frame)
        r = float(-p_arm[1])
        # Elevation relative to base_link origin
        z = float(self.BASE_PIVOT_Z + p_arm[2])
        # Base azimuth angle is directly q[0]
        theta = float(q[0])
        # Gripper pitch relative to ground horizon
        world_pitch = float(-(q[1] + q[2] - 2.00719 + q[3] + 0.4363323))
        roll = float(q[4])
        return r, theta, z, world_pitch, roll

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
