#!/usr/bin/env python3

import sys
import select
import termios
import tty
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64MultiArray

HELP_MSG = """
===================================================================
               ARM KEYBOARD TELEOP & JOINT BRIDGE
===================================================================
 Hold keys to move. Release to immediately stop & center axes.
-------------------------------------------------------------------
  [W] / [S] : Reach +/- (IK)      | Shoulder +/- (FK)
  [A] / [D] : Azimuth +/- (IK)    | Base Yaw +/- (FK)
  [I] / [K] : Elevation +/- (IK)  | Elbow +/- (FK)
  [J] / [L] : Wrist Roll +/- (IK) | Wrist Roll +/- (FK)
  [U] / [O] : Wrist Pitch +/-     | Wrist Pitch +/-

  [ [ ] / [ ] ] : Gripper Open / Close

  [M]       : TOGGLE MODE (FK <-> IK)
  [C]       : Toggle Precision Crawl Mode (30% Speed)
  [R]       : Toggle Orientation Layer (RB button)
  [SPACE]   : Immediate Stop / Center
  [X]       : Emergency Stop Lock Toggle
  [Q]       : Quit Teleop
===================================================================
"""

class KeyboardTeleopBridge(Node):
    """
    Dual-purpose test node:
      1. Maps keyboard keystrokes to sensor_msgs/Joy simulating a PS5 DualSense controller.
      2. Bridges /arm_cmd (from ps5_mapper or ik_solver_node) to /joint_states for live RViz animation.
    """

    JOINT_NAMES = [
        "base_yaw_joint",
        "shoulder_joint",
        "elbow_joint",
        "wrist_pitch_joint",
        "wrist_roll_joint",
        "gripper_joint"
    ]

    STICK_STEP = 1.0  # Full deflection for all motion axes

    def __init__(self):
        super().__init__('keyboard_teleop_bridge')

        # ----------------- Joy Publisher -----------------
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)

        # ----------------- Joint State Bridge -----------------
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.arm_cmd_sub = self.create_subscription(
            Float64MultiArray, 'arm_cmd', self.arm_cmd_callback, 10
        )

        # Internal joint position storage [J0..J4, gripper] — initial home posture (elbow bent 115 deg)
        self.current_joints = [0.0, 0.0, 2.0072, 0.0, 0.0, 0.0]
        self.lock = threading.Lock()

        # PS5 Controller State Simulation
        # Axes: [LJOY_X=0, LJOY_Y=1, LT=2, RJOY_X=3, RJOY_Y=4, RT=5, DPAD_X=6, DPAD_Y=7]
        self.axes = [0.0] * 8
        self.axes[2] = 1.0  # LT unpressed
        self.axes[5] = 1.0  # RT unpressed

        # Buttons: 0:X, 1:O, 2:Tri, 3:Sq, 4:LB, 5:RB, 6:LT, 7:RT, 8:Share, 9:Opt, 10:PS, 11:L3, 12:R3
        self.buttons = [0] * 13

        self.rb_active = False
        self.lb_active = False
        self.estop_active = False
        self.mode_name = "FK Mode (Mode 0)"
        self.last_key_time = 0.0
        self.is_moving = False

        # 50 Hz Publisher Timer
        self.timer = self.create_timer(0.02, self.timer_callback)

    def arm_cmd_callback(self, msg: Float64MultiArray):
        """Receives commanded joint positions from either ps5_mapper or ik_solver_node."""
        with self.lock:
            for i in range(min(len(msg.data), 6)):
                self.current_joints[i] = msg.data[i]

    def timer_callback(self):
        """Publishes /joy and /joint_states at 50 Hz."""
        now = self.get_clock().now()

        # Auto-center motion axes when key is released (>0.2s since last keystroke)
        if self.is_moving and (time.time() - self.last_key_time > 0.2):
            self.center_motion_axes()

        # 1. Publish /joint_states for RViz animation
        js = JointState()
        js.header.stamp = now.to_msg()
        js.header.frame_id = 'base_link'
        js.name = list(self.JOINT_NAMES)
        with self.lock:
            js.position = list(self.current_joints)
        self.joint_state_pub.publish(js)

        # 2. Publish /joy
        joy_msg = Joy()
        joy_msg.header.stamp = now.to_msg()
        joy_msg.header.frame_id = 'teleop_keyboard'
        joy_msg.axes = list(self.axes)
        joy_msg.buttons = list(self.buttons)
        # self.joy_pub.publish(joy_msg)

        # Clear one-shot buttons after one tick
        for btn_idx in (9, 10):
            if self.buttons[btn_idx]:
                self.buttons[btn_idx] = 0

    def print_status(self, active_key=''):
        """Prints a single-line status bar at the bottom without scrolling the terminal."""
        motion_str = f"[{active_key.upper()}]" if active_key else "IDLE"
        crawl_str = "ON (30%)" if self.lb_active else "OFF"
        rb_str = "ON" if self.rb_active else "OFF"
        estop_str = " | [E-STOP ACTIVE]" if self.estop_active else ""
        sys.stdout.write(f"\r\033[K[STATUS] Mode: {self.mode_name} | Crawl: {crawl_str} | RB: {rb_str} | Motion: {motion_str}{estop_str}")
        sys.stdout.flush()

    def center_motion_axes(self):
        """Resets all transient motion axes and temporary button overrides to center (0.0)."""
        with self.lock:
            self.axes[0] = 0.0
            self.axes[1] = 0.0
            self.axes[3] = 0.0
            self.axes[4] = 0.0
            self.axes[6] = 0.0
            if not self.rb_active:
                self.buttons[5] = 0
            self.is_moving = False
        self.print_status()

    def update_motion(self, axes_dict=None, rb_override=None, key_char=''):
        """Applies axis deflection and updates watchdog timer while key is held."""
        with self.lock:
            if axes_dict:
                for idx, val in axes_dict.items():
                    self.axes[idx] = val
            if rb_override is not None:
                self.buttons[5] = rb_override
            elif not self.rb_active:
                self.buttons[5] = 0
            self.last_key_time = time.time()
            self.is_moving = True
        self.print_status(key_char)

    def trigger_mode_toggle(self):
        """Simulates pressing the OPTIONS button (button 9)."""
        self.buttons[9] = 1
        if "FK" in self.mode_name:
            self.mode_name = "IK Mode (Mode 1)"
        else:
            self.mode_name = "FK Mode (Mode 0)"
        self.print_status()

    def trigger_estop(self):
        """Simulates pressing the PS button (button 10)."""
        self.buttons[10] = 1
        self.estop_active = not self.estop_active
        self.print_status()


def get_key(settings, timeout=0.04):
    """Reads a single keystroke non-blockingly from stdin if a TTY is attached."""
    if not sys.stdin.isatty():
        time.sleep(timeout)
        return ''
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    print(HELP_MSG)
    settings = None
    if sys.stdin.isatty():
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = KeyboardTeleopBridge()

    def run_spin():
        try:
            rclpy.spin(node)
        except Exception:
            pass

    # Run ROS spin in background daemon thread
    spin_thread = threading.Thread(target=run_spin, daemon=True)
    spin_thread.start()

    stick_step = node.STICK_STEP
    node.print_status()

    try:
        while rclpy.ok():
            key = get_key(settings, timeout=0.03).lower()

            if not key:
                continue

            if key == 'q':
                print("\n\nExiting keyboard teleop...")
                break

            # Left Stick (W/S/A/D)
            elif key == 'w':
                node.update_motion({1: stick_step}, key_char='W')
            elif key == 's':
                node.update_motion({1: -stick_step}, key_char='S')
            elif key == 'a':
                node.update_motion({0: stick_step}, key_char='A')
            elif key == 'd':
                node.update_motion({0: -stick_step}, key_char='D')

            # Right Stick (I/K/J/L)
            elif key == 'i':
                node.update_motion({4: stick_step}, key_char='I')
            elif key == 'k':
                node.update_motion({4: -stick_step}, key_char='K')
            elif key == 'j':
                node.update_motion({3: stick_step}, rb_override=1, key_char='J')
            elif key == 'l':
                node.update_motion({3: -stick_step}, rb_override=1, key_char='L')

            # Wrist Pitch direct helper (U/O)
            elif key == 'u':
                node.update_motion({4: stick_step}, rb_override=1, key_char='U')
            elif key == 'o':
                node.update_motion({4: -stick_step}, rb_override=1, key_char='O')

            # Gripper Open / Close ( [ and ] )
            elif key == '[':
                node.update_motion({6: 1.0}, key_char='[')
            elif key == ']':
                node.update_motion({6: -1.0}, key_char=']')

            # Stop / Zero Sticks manually
            elif key == ' ':
                node.center_motion_axes()

            # Mode Toggle (M)
            elif key == 'm':
                node.trigger_mode_toggle()

            # RB Layer Toggle (R)
            elif key == 'r':
                node.rb_active = not node.rb_active
                node.buttons[5] = 1 if node.rb_active else 0
                node.print_status()

            # LB Precision Crawl (C)
            elif key == 'c':
                node.lb_active = not node.lb_active
                node.buttons[4] = 1 if node.lb_active else 0
                node.print_status()

            # E-Stop Toggle (X)
            elif key == 'x':
                node.trigger_estop()

    except Exception as e:
        print(f"Keyboard loop error: {e}")

    finally:
        if settings is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            except Exception:
                pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
