#!/usr/bin/env python3
"""
ps5_mapper.py - Rudimentary FK Velocity Teleoperation Node for PS5 DualSense Controller.

Translates PlayStation 5 controller inputs into direct forward kinematics (FK) velocity / PWM commands
for an embedded micro-ROS ESP32 motor driver bridge.

Control Pipeline:
    PS5 (DualSense) -> joy_node -> ps5_mapper -> /arm_cmd (Int16MultiArray [J0..J4, Gripper, Actuator] ∈ [-255, 255])
                    -> micro-ROS Agent -> ESP32 -> Motor Drivers -> Arm Motors & Linear Actuator

Outputs:
    /arm_cmd (std_msgs/msg/Int16MultiArray):
        A 7-element array of integers [-255, 255]:
        -255 = max speed in reverse polarity / retract
           0 = stopped / neutral
        +255 = max speed in forward polarity / extend

Joint Assignments:
    - J0 (Base Yaw): Left Stick X
    - J1 (Shoulder Pitch): Left Stick Y
    - J2 (Elbow Pitch): Right Stick Y (default layer, RB unpressed)
    - J3 (Wrist Pitch): Right Stick Y (wrist layer, RB held)
    - J4 (Wrist Roll): Right Stick X (wrist layer, RB held)
    - J5 (Gripper): D-Pad Left/Right (-1 Close / +1 Open)
    - J6 (Linear Actuator): D-Pad Up/Down (+255 Extend / -255 Retract)

Features & Safety:
    - Dominant-Axis Locking on Left Stick (eliminates accidental diagonal cross-talk)
    - Mode Toggle on OPTIONS Button: Switch between FK (Joint Jogging) and Pseudo-IK (Reach/Elevate Synergy)
    - Live Ratio Trimming: Hold Square (Shoulder), Circle (Elbow), or Triangle (Wrist Pitch), then RT to increase / LT to decrease
    - Right Bumper (RB) layer toggle for Wrist Gimbal (J3 Pitch / J4 Roll) vs Elbow (J2 Pitch) / Elevation
    - Left Bumper (LB) Precision Crawl mode (scales speeds by precision_scale, default 30% / ±76)
    - Emergency Stop Latch on PS Button (latches output to [0, 0, 0, 0, 0, 0, 0])
    - Heartbeat Watchdog Timer (auto-zeros output on joy disconnect / >0.2s timeout)
    - Per-joint configurable polarity inversion (invert_j0 .. invert_j4)
    - Per-joint configurable max speed clamp (max_j0_speed .. max_j4_speed)
    - Deterministic 50 Hz control loop
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy, JoyFeedback
from std_msgs.msg import Int16MultiArray


class StickAxisLock:
    """
    Filters 2-axis joystick input by locking onto the dominant axis once moved outside
    the deadzone, while allowing clean dynamic switching when the operator deliberately
    deflects the secondary axis. Prevents accidental diagonal cross-talk.
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


class PS5Mapper(Node):
    """
    ROS 2 teleoperation node mapping PS5 DualSense inputs directly to 5-joint velocity commands.
    """

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
    TRIANGLE = 2
    SQUARE   = 3
    LB       = 4   # Precision Crawl (30%)
    RB       = 5   # Wrist Gimbal Layer Toggle
    LT_BTN   = 6
    RT_BTN   = 7
    SHARE    = 8
    OPTIONS  = 9
    PS_BTN   = 10  # Emergency Stop Latch
    LJOY_BTN = 11
    RJOY_BTN = 12

    def __init__(self):
        super().__init__('ps5_mapper')

        # ----- Declare ROS 2 Parameters -----
        self.declare_parameter('deadzone', 0.10)
        self.declare_parameter('precision_scale', 0.30)
        self.declare_parameter('axis_lock_enabled', True)
        self.declare_parameter('watchdog_timeout', 0.20)
        self.declare_parameter('watchdog_rate', 10.0)
        self.declare_parameter('control_rate', 50.0)

        # Configurable joint polarity inversions (J0 Base Yaw & J4 Wrist Roll inverted)
        self.declare_parameter('invert_j0', True)
        self.declare_parameter('invert_j1', False)
        self.declare_parameter('invert_j2', False)
        self.declare_parameter('invert_j3', False)
        self.declare_parameter('invert_j4', True)
        self.declare_parameter('invert_gripper', False)
        self.declare_parameter('invert_actuator', False)

        # Configurable max speeds (clamped to [-255, 255])
        self.declare_parameter('max_j0_speed', 255)
        self.declare_parameter('max_j1_speed', 255)
        self.declare_parameter('max_j2_speed', 255)
        self.declare_parameter('max_j3_speed', 255)
        self.declare_parameter('max_j4_speed', 255)
        self.declare_parameter('gripper_speed', 128)
        self.declare_parameter('actuator_speed', 255)

        # Configurable IK Synergy Ratios & Gains (Assuming equal link lengths L1 = L2 = L)
        self.declare_parameter('ik_reach_shoulder_ratio', -0.5)
        self.declare_parameter('ik_reach_elbow_ratio', 1.0)
        self.declare_parameter('ik_reach_wrist_ratio', -0.5)
        self.declare_parameter('ik_elevate_shoulder_gain', 1.0)
        self.declare_parameter('ik_elevate_elbow_gain', -0.5)
        self.declare_parameter('ik_elevate_wrist_gain', -0.5)
        self.declare_parameter('ik_trim_step', 0.05)

        # Cache parameter values
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.precision_scale = float(self.get_parameter('precision_scale').value)
        self.axis_lock_enabled = bool(self.get_parameter('axis_lock_enabled').value)
        self.watchdog_timeout = float(self.get_parameter('watchdog_timeout').value)

        # Left stick dominant axis filter
        self.left_stick_lock = StickAxisLock()

        # Teleop state
        self.e_stop_active = False
        self.prev_estop_btn_state = 0
        self.ik_mode_active = False
        self.prev_options_btn_state = 0
        self.signal_lost = False
        self.last_joy_time = None

        # Live trimming state (Square=Shoulder, Circle=Elbow, Triangle=Wrist Pitch + RT/LT)
        self.trim_rt_prev = False
        self.trim_lt_prev = False
        self.trim_hold_start_time = None
        self.trim_last_repeat_time = None

        # Filtered stick & button states
        self.lx = 0.0
        self.ly = 0.0
        self.rx = 0.0
        self.ry = 0.0
        self.lb_held = False
        self.rb_held = False
        self.gripper_cmd = 0
        self.dpad_y = 0.0

        # Current command array [J0, J1, J2, J3, J4, Gripper, LinearActuator]
        self.cmd_output = [0, 0, 0, 0, 0, 0, 0]

        # ----- Subscriptions & Publishers -----
        self.joy_sub = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10
        )

        self.cmd_pub = self.create_publisher(
            Int16MultiArray, 'arm_cmd', 10
        )

        self.feedback_pub = self.create_publisher(
            JoyFeedback, '/joy/set_feedback', 10
        )

        # ----- Timers -----
        control_period = 1.0 / float(self.get_parameter('control_rate').value)
        self.control_timer = self.create_timer(control_period, self.control_loop)

        watchdog_period = 1.0 / float(self.get_parameter('watchdog_rate').value)
        self.watchdog_timer = self.create_timer(watchdog_period, self.watchdog_callback)

        self.get_logger().info(
            "PS5 Mapper Node initialized: Pure FK Velocity Mode -> /arm_cmd (Int16MultiArray [J0..J4, Gripper, Actuator] ∈ [-255, 255])."
        )

    def normalize_axis(self, value: float) -> float:
        """Applies deadzone filter and normalizes deflection linearly to [-1.0, 1.0]."""
        abs_val = abs(value)
        if abs_val <= self.deadzone:
            return 0.0
        ratio = (abs_val - self.deadzone) / (1.0 - self.deadzone)
        ratio = min(1.0, max(0.0, ratio))
        return ratio if value > 0.0 else -ratio

    def check_estop(self, estop_btn_state: int):
        """Toggles Emergency Stop on rising edge of the PS Button."""
        if estop_btn_state == 1 and self.prev_estop_btn_state == 0:
            self.e_stop_active = not self.e_stop_active
            if self.e_stop_active:
                self.get_logger().error("EMERGENCY STOP ENGAGED! All arm motor velocities clamped to 0.")
                self.trigger_rumble(0.1, 0.1)
            else:
                self.get_logger().info("EMERGENCY STOP CLEARED. Normal operation resumed.")
                self.trigger_rumble(0.1, 0.1)

                self.trigger_rumble(0.1, 0.1)
        self.prev_estop_btn_state = estop_btn_state

    def check_mode_toggle(self, options_btn_state: int):
        """Toggles between FK and Pseudo-IK mode on rising edge of OPTIONS button."""
        if options_btn_state == 1 and self.prev_options_btn_state == 0:
            self.ik_mode_active = not self.ik_mode_active
            mode_name = "PSEUDO-IK (Reach Synergy)" if self.ik_mode_active else "FK (Joint Velocity)"
            self.get_logger().info(f"Control mode toggled to: {mode_name}")
            if self.ik_mode_active:
                self.trigger_rumble(0.2, 0.4)
            else:
                self.trigger_rumble(0.3, 0.1)
            # Momentarily zero outputs to avoid transient jerks across mode switches
            self.cmd_output = [0, 0, 0, 0, 0, 0, 0]
        self.prev_options_btn_state = options_btn_state

    def trigger_rumble(self, low_freq: float, high_freq: float):
        """Sends momentary haptic feedback to PS5 controller."""
        msg_left = JoyFeedback()
        msg_left.type = JoyFeedback.TYPE_RUMBLE
        msg_left.id = 0
        msg_left.intensity = float(low_freq)
        self.feedback_pub.publish(msg_left)

        msg_right = JoyFeedback()
        msg_right.type = JoyFeedback.TYPE_RUMBLE
        msg_right.id = 1
        msg_right.intensity = float(high_freq)
        self.feedback_pub.publish(msg_right)

    def apply_ratio_trim(self, param_name: str, joint_name: str, direction: int):
        """
        Increases (direction=+1) or decreases (direction=-1) the magnitude of the specified ratio.
        Clamps magnitude between 0.0 and 3.0. Updates the ROS parameter so it takes immediate effect.
        """
        current_val = float(self.get_parameter(param_name).value)
        step = float(self.get_parameter('ik_trim_step').value)

        # For ratios with a negative sign (Shoulder, Wrist), increasing magnitude means becoming more negative
        sign = -1.0 if current_val < 0 else (1.0 if current_val > 0 else (1.0 if joint_name == 'Elbow' else -1.0))
        new_val = current_val + (direction * sign * step)
        new_val = round(new_val, 3)

        # Clamp magnitude: prevent sign inversion when decreasing, and clamp max magnitude to 3.0
        if sign > 0:
            new_val = max(0.0, min(3.0, new_val))
        else:
            new_val = min(0.0, max(-3.0, new_val))

        self.set_parameters([Parameter(param_name, Parameter.Type.DOUBLE, new_val)])
        self.get_logger().info(f"[Live Trim] {joint_name} Reach Ratio: {new_val:.2f} (Magnitude: {abs(new_val):.2f})")
        self.trigger_rumble(0.05, 0.05)

    def process_live_trim(self, msg: Joy):
        """
        Handles live trimming of IK reach ratios when holding Square, Circle, or Triangle:
          - Square held: Shoulder reach ratio
          - Circle held: Elbow reach ratio
          - Triangle held: Wrist pitch reach ratio
        While one of these is held:
          - RT increases magnitude
          - LT decreases magnitude
        """
        square_held = len(msg.buttons) > self.SQUARE and bool(msg.buttons[self.SQUARE])
        circle_held = len(msg.buttons) > self.CIRCLE and bool(msg.buttons[self.CIRCLE])
        triangle_held = len(msg.buttons) > self.TRIANGLE and bool(msg.buttons[self.TRIANGLE])

        if square_held:
            target_param = 'ik_reach_shoulder_ratio'
            joint_name = 'Shoulder'
        elif circle_held:
            target_param = 'ik_reach_elbow_ratio'
            joint_name = 'Elbow'
        elif triangle_held:
            target_param = 'ik_reach_wrist_ratio'
            joint_name = 'Wrist Pitch'
        else:
            self.trim_hold_start_time = None
            self.trim_rt_prev = False
            self.trim_lt_prev = False
            return

        # Check RT (Right Trigger): button 7 or axis 5
        rt_btn = len(msg.buttons) > self.RT_BTN and bool(msg.buttons[self.RT_BTN])
        rt_axis = len(msg.axes) > self.RT and (msg.axes[self.RT] < 0.0)
        rt_active = rt_btn or rt_axis

        # Check LT (Left Trigger): button 6 or axis 2
        lt_btn = len(msg.buttons) > self.LT_BTN and bool(msg.buttons[self.LT_BTN])
        lt_axis = len(msg.axes) > self.LT and (msg.axes[self.LT] < 0.0)
        lt_active = lt_btn or lt_axis

        now_sec = self.get_clock().now().nanoseconds / 1e9

        if rt_active and not lt_active:
            if not self.trim_rt_prev:
                # Rising edge: step immediately
                self.trim_hold_start_time = now_sec
                self.trim_last_repeat_time = now_sec
                self.apply_ratio_trim(target_param, joint_name, direction=+1)
            elif self.trim_hold_start_time is not None:
                # Held: autorepeat
                if (now_sec - self.trim_hold_start_time > 0.4) and (now_sec - self.trim_last_repeat_time > 0.15):
                    self.trim_last_repeat_time = now_sec
                    self.apply_ratio_trim(target_param, joint_name, direction=+1)
        elif lt_active and not rt_active:
            if not self.trim_lt_prev:
                # Rising edge: step immediately
                self.trim_hold_start_time = now_sec
                self.trim_last_repeat_time = now_sec
                self.apply_ratio_trim(target_param, joint_name, direction=-1)
            elif self.trim_hold_start_time is not None:
                # Held: autorepeat
                if (now_sec - self.trim_hold_start_time > 0.4) and (now_sec - self.trim_last_repeat_time > 0.15):
                    self.trim_last_repeat_time = now_sec
                    self.apply_ratio_trim(target_param, joint_name, direction=-1)
        else:
            self.trim_hold_start_time = None

        self.trim_rt_prev = rt_active
        self.trim_lt_prev = lt_active

    def joy_callback(self, msg: Joy):
        """Processes incoming controller stick and button inputs."""
        self.last_joy_time = self.get_clock().now()
        if self.signal_lost:
            self.signal_lost = False
            self.get_logger().info("Joystick signal restored. Normal teleop resumed.")

        # 1. Emergency Stop Check (PS Button)
        if len(msg.buttons) > self.PS_BTN:
            self.check_estop(msg.buttons[self.PS_BTN])

        # 2. Mode Toggle Check (OPTIONS Button)
        if len(msg.buttons) > self.OPTIONS:
            self.check_mode_toggle(msg.buttons[self.OPTIONS])

        # 3. Live Trimming Check (Square/Circle/Triangle + RT/LT)
        if not self.e_stop_active:
            self.process_live_trim(msg)

        # If E-Stop is active, immediately zero stored stick values
        if self.e_stop_active:
            self.lx = 0.0
            self.ly = 0.0
            self.rx = 0.0
            self.ry = 0.0
            self.lb_held = False
            self.rb_held = False
            self.gripper_cmd = 0
            self.dpad_y = 0.0
            return

        # 2. Record Bumper Modifiers
        if len(msg.buttons) > self.LB:
            self.lb_held = bool(msg.buttons[self.LB])
        if len(msg.buttons) > self.RB:
            self.rb_held = bool(msg.buttons[self.RB])

        # 3. Filter Left Stick (J0: Base Yaw, J1: Shoulder Pitch)
        raw_lx = msg.axes[self.LJOY_X] if len(msg.axes) > self.LJOY_X else 0.0
        raw_ly = msg.axes[self.LJOY_Y] if len(msg.axes) > self.LJOY_Y else 0.0

        if self.axis_lock_enabled:
            flt_lx, flt_ly = self.left_stick_lock.filter(raw_lx, raw_ly, self.deadzone)
        else:
            flt_lx, flt_ly = raw_lx, raw_ly

        self.lx = self.normalize_axis(flt_lx)
        self.ly = self.normalize_axis(flt_ly)

        # 4. Filter Right Stick (J2: Elbow, or J3: Wrist Pitch & J4: Wrist Roll)
        raw_rx = msg.axes[self.RJOY_X] if len(msg.axes) > self.RJOY_X else 0.0
        raw_ry = msg.axes[self.RJOY_Y] if len(msg.axes) > self.RJOY_Y else 0.0

        self.rx = self.normalize_axis(raw_rx)
        self.ry = self.normalize_axis(raw_ry)

        # 5. Gripper on D-Pad Left/Right (Left = Open, Right = Close)
        raw_dpad_x = msg.axes[self.DPAD_X] if len(msg.axes) > self.DPAD_X else 0.0
        speed = int(self.get_parameter('gripper_speed').value)
        if raw_dpad_x > 0.5:
            self.gripper_cmd = -1    # D-Pad Left: Open
        elif raw_dpad_x < -0.5:
            self.gripper_cmd = 1    # D-Pad Right: Close
        else:
            self.gripper_cmd = 0

        # 6. Linear Actuator on D-Pad Up/Down (Up = Extend [+255], Down = Retract [-255])
        raw_dpad_y = msg.axes[self.DPAD_Y] if len(msg.axes) > self.DPAD_Y else 0.0
        if raw_dpad_y > 0.5:
            self.dpad_y = 1.0     # D-Pad Up: Extend
        elif raw_dpad_y < -0.5:
            self.dpad_y = -1.0    # D-Pad Down: Retract
        else:
            self.dpad_y = 0.0

    def compute_joint_pwm(self, norm_val: float, max_speed_param: str, invert_param: str) -> int:
        """Converts normalized deflection [-1.0, 1.0] to integer PWM speed [-255, 255]."""
        if abs(norm_val) < 1e-4:
            return 0

        max_speed = int(self.get_parameter(max_speed_param).value)
        max_speed = max(0, min(255, max_speed))

        speed_scale = self.precision_scale if self.lb_held else 1.0
        pwm = int(round(norm_val * max_speed * speed_scale))
        pwm = max(-max_speed, min(max_speed, pwm))

        if bool(self.get_parameter(invert_param).value):
            pwm = -pwm

        return int(max(-255, min(255, pwm)))

    def control_loop(self):
        """
        Deterministic 50 Hz control loop. Computes and streams 7-element /arm_cmd array.
        """
        if self.e_stop_active or self.signal_lost:
            self.cmd_output = [0, 0, 0, 0, 0, 0, 0]
        else:
            # J0: Base Yaw (Left Stick X)
            j0_pwm = self.compute_joint_pwm(self.lx, 'max_j0_speed', 'invert_j0')

            if not self.ik_mode_active:
                # --- FK Mode ---
                # J1: Shoulder Pitch (Left Stick Y)
                j1_pwm = self.compute_joint_pwm(self.ly, 'max_j1_speed', 'invert_j1')

                if not self.rb_held:
                    # Default Reach Layer: Right Stick Y controls J2 (Elbow Pitch)
                    j2_pwm = self.compute_joint_pwm(self.ry, 'max_j2_speed', 'invert_j2')
                    j3_pwm = 0
                    j4_pwm = 0
                else:
                    # Wrist Gimbal Layer (RB held): Right Stick controls J3 Pitch (Y) & J4 Roll (X)
                    j2_pwm = 0
                    j3_pwm = self.compute_joint_pwm(self.ry, 'max_j3_speed', 'invert_j3')
                    j4_pwm = self.compute_joint_pwm(self.rx, 'max_j4_speed', 'invert_j4')
            else:
                # --- Pseudo-IK Synergy Mode ---
                reach_s_ratio = float(self.get_parameter('ik_reach_shoulder_ratio').value)
                reach_e_ratio = float(self.get_parameter('ik_reach_elbow_ratio').value)
                reach_w_ratio = float(self.get_parameter('ik_reach_wrist_ratio').value)

                elev_s_gain = float(self.get_parameter('ik_elevate_shoulder_gain').value)
                elev_e_gain = float(self.get_parameter('ik_elevate_elbow_gain').value)
                elev_w_gain = float(self.get_parameter('ik_elevate_wrist_gain').value)

                # Reach is driven by Left Stick Y (self.ly)
                # Elevation is driven by Right Stick Y (self.ry) when RB is unpressed
                if not self.rb_held:
                    elev_input = self.ry
                    manual_wrist_pitch = 0.0
                    j4_pwm = 0
                else:
                    # When RB is held, Right Stick Y/X provides manual Wrist Pitch & Roll overrides
                    elev_input = 0.0
                    manual_wrist_pitch = self.ry
                    j4_pwm = self.compute_joint_pwm(self.rx, 'max_j4_speed', 'invert_j4')

                # Combined normalized command contributions
                s_norm = (self.ly * reach_s_ratio) + (elev_input * elev_s_gain)
                e_norm = (self.ly * reach_e_ratio) + (elev_input * elev_e_gain)
                w_norm = (self.ly * reach_w_ratio) + (elev_input * elev_w_gain) + manual_wrist_pitch

                # Clamp combined normalized demands to [-1.0, 1.0] to prevent over-saturation
                s_norm = max(-1.0, min(1.0, s_norm))
                e_norm = max(-1.0, min(1.0, e_norm))
                w_norm = max(-1.0, min(1.0, w_norm))

                j1_pwm = self.compute_joint_pwm(s_norm, 'max_j1_speed', 'invert_j1')
                j2_pwm = self.compute_joint_pwm(e_norm, 'max_j2_speed', 'invert_j2')
                j3_pwm = self.compute_joint_pwm(w_norm, 'max_j3_speed', 'invert_j3')

            # J5: Gripper Jaw (D-Pad Left/Right -> -1 / +1)
            gripper_pwm = self.gripper_cmd
            if bool(self.get_parameter('invert_gripper').value):
                gripper_pwm = -gripper_pwm

            # J6: Linear Actuator (D-Pad Up/Down -> [-255, 255])
            actuator_pwm = self.compute_joint_pwm(self.dpad_y, 'actuator_speed', 'invert_actuator')

            self.cmd_output = [j0_pwm, j1_pwm, j2_pwm, j3_pwm, j4_pwm, gripper_pwm, actuator_pwm]

        # Publish 7-element Int16MultiArray on /arm_cmd
        msg = Int16MultiArray()
        msg.data = list(self.cmd_output)
        self.cmd_pub.publish(msg)

    def watchdog_callback(self):
        """Monitors joystick heartbeat. Forces all outputs to 0 on timeout."""
        if self.last_joy_time is None:
            return

        elapsed_sec = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
        if elapsed_sec > self.watchdog_timeout and not self.signal_lost:
            self.signal_lost = True
            self.cmd_output = [0, 0, 0, 0, 0, 0, 0]
            msg = Int16MultiArray()
            msg.data = [0, 0, 0, 0, 0, 0, 0]
            self.cmd_pub.publish(msg)
            self.get_logger().warn(
                f"Joy input timed out ({elapsed_sec:.2f}s > {self.watchdog_timeout}s). All motors stopped."
            )


def main(args=None):
    rclpy.init(args=args)
    node = PS5Mapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Publish clean stop on exit
        try:
            stop_msg = Int16MultiArray()
            stop_msg.data = [0, 0, 0, 0, 0, 0, 0]
            node.cmd_pub.publish(stop_msg)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
