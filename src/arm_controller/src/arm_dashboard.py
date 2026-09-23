#!/usr/bin/env python3
"""
arm_dashboard.py - Live Telemetry & Diagnostics TUI Dashboard for 5-DOF Robotic Arm.

Displays real-time joint-space telemetry, ground-relative angles, task-space kinematics,
dynamic leash anti-windup status, teleoperation modes, control loop frequencies, and
system diagnostic alerts using Python's 'rich' library.

Features:
- Prominent Master State Banner (E-STOP, JOY TIMEOUT, RTH, IK MODE 1, FK MODE 0).
- Independent & Context-Switching Speed Trims (FK Joint Trims in FK, IK Cartesian in IK).
- Responsive Interactive Tabbed Views (Overview, Joints, Task-Space & RTH, Speeds & Logs)
  supporting keypress switching (0-3 / TAB / Space) and auto-cycling (A).
- High refresh rate (10 Hz) with sliding-window topic rate estimation.
"""

import os
import sys
import math
import time
import argparse
import collections
import threading
import select

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from rcl_interfaces.msg import Log

from rich.console import Console
from rich.live import Live
from rich.table import Table
from rich.panel import Panel
from rich.layout import Layout
from rich.text import Text


class RateEstimator:
    """Sliding-window frequency estimator for ROS 2 topics."""

    def __init__(self, window_size: int = 25):
        self.window_size = window_size
        self.timestamps = collections.deque(maxlen=window_size)

    def tick(self):
        self.timestamps.append(time.time())

    def get_rate(self) -> float:
        if len(self.timestamps) < 2:
            return 0.0
        elapsed = self.timestamps[-1] - self.timestamps[0]
        if elapsed <= 1e-4:
            return 0.0
        return (len(self.timestamps) - 1) / elapsed

    def is_active(self, timeout_sec: float = 0.5) -> bool:
        if not self.timestamps:
            return False
        return (time.time() - self.timestamps[-1]) < timeout_sec


class ArmDashboardNode(Node):
    """ROS 2 node that subscribes to arm telemetry topics and maintains display state."""

    # Software joint limits (min, max) in radians (matches URDF)
    JOINT_LIMITS = [
        (-3.14,  3.14),   # J0: Base Yaw        (±180°)
        (-1.57,  1.57),   # J1: Shoulder Pitch  (±90°)
        ( 0.00,  2.50),   # J2: Elbow Pitch     (0 to +143.2°)
        (-1.57,  1.57),   # J3: Wrist Pitch     (±90°)
        (-3.14,  3.14),   # J4: Wrist Roll      (±180°)
        ( 0.00,  1.57),   # J5: Gripper         (0 to +90°, matches URDF)
    ]

    JOINT_NAMES = [
        "J0: Base Yaw",
        "J1: Shoulder Pitch",
        "J2: Elbow Pitch",
        "J3: Wrist Pitch",
        "J4: Wrist Roll",
        "J5: Gripper Jaw",
    ]

    HOME_JOINTS = [0.0, 0.0, 2.0072, 0.0, 0.0]

    BASE_PIVOT_X       = 0.043511
    BASE_PIVOT_Y       = -0.012448
    BASE_PIVOT_Z       = 0.03425
    ARM_LATERAL_OFFSET = -0.023
    WORKSPACE_RADIUS   = 0.98
    SHOULDER_PIVOT_Z   = 0.1385
    LEASH_MAX_M        = 0.025

    def __init__(self):
        super().__init__('arm_dashboard')

        self.lock = threading.Lock()

        # Telemetry State
        self.joint_positions = [0.0, 0.0, 2.0072, 0.0, 0.0, 0.0]
        self.prev_joint_positions = list(self.joint_positions)
        self.joint_velocities = [0.0] * 6
        self.last_joint_time = time.time()

        # IK Target State
        self.target_r = 0.4775
        self.target_theta = 0.0
        self.target_z = 0.3315
        self.target_world_pitch = -0.4363
        self.target_roll = 0.0

        # Commanded Target Pose (from /arm_target_pose if available)
        self.cmd_x = 0.0
        self.cmd_y = 0.0
        self.cmd_z = 0.0

        # Physical Forward Kinematics State
        self.act_r = 0.4775
        self.act_theta = 0.0
        self.act_z = 0.3315
        self.act_x = 0.0
        self.act_y = 0.0
        self.act_world_pitch = -0.4363
        self.act_roll = 0.0
        self.leash_dist_m = 0.0
        self.sphere_dist_m = 0.0

        # Controller & Mode State
        self.control_mode = "FK Mode (Mode 0)"
        self.crawl_active = False
        self.orientation_active = False
        self.estop_active = False
        self.rth_status = "IDLE"
        self.rth_progress = 0.0
        self.rth_initial_dist = 1.0
        self.gripper_load = 0.0
        self.is_gripping = False

        # Independent Speed Trims (Dynamic)
        # FK Joint Speeds (rad/s)
        self.fk_speed_base = 0.10
        self.fk_speed_shoulder = 0.10
        self.fk_speed_elbow = 0.10
        self.fk_speed_wrist = 0.10

        # IK Cartesian Speeds (m/s & rad/s)
        self.ik_speed_azimuth = 0.20
        self.ik_speed_reach = 0.10
        self.ik_speed_elev = 0.10
        self.ik_speed_world_pitch = 0.20
        self.ik_speed_roll = 0.30

        # Gripper (shared)
        self.speed_gripper = 0.10

        # Tab Navigation State
        # 0: Overview, 1: Joints & Limits, 2: Task-Space & RTH, 3: Speeds & Logs
        self.active_tab = 0
        self.autocycle_enabled = False
        self.last_autocycle_time = time.time()
        self.autocycle_period = 2.0

        # Rate Estimators
        self.rate_arm_cmd = RateEstimator(window_size=30)
        self.rate_joy = RateEstimator(window_size=30)
        self.rate_ik_cmd = RateEstimator(window_size=30)

        # Solver Status
        self.solver_status = "IDLE (FK MODE)"

        # Recent Diagnostic Logs (maxlen=10)
        self.log_messages = collections.deque(maxlen=10)
        self.log_messages.append((time.strftime("%H:%M:%S"), "INFO", "Arm Dashboard initialized."))

        # Subscriptions
        self.create_subscription(Float64MultiArray, 'arm_cmd', self._arm_cmd_cb, 10)
        self.create_subscription(Float64MultiArray, 'arm_ik_cmd', self._arm_ik_cmd_cb, 10)
        self.create_subscription(PoseStamped, 'arm_target_pose', self._target_pose_cb, 10)
        self.create_subscription(Joy, 'joy', self._joy_cb, 10)
        self.create_subscription(Float64MultiArray, 'gripper_state', self._gripper_cb, 10)
        self.create_subscription(Log, '/rosout', self._rosout_cb, 20)

        # Initial forward kinematics computation
        self._update_fk()

    # ── Subscribers ──────────────────────────────────────────────────────────

    def _arm_cmd_cb(self, msg: Float64MultiArray):
        now = time.time()
        with self.lock:
            self.rate_arm_cmd.tick()
            dt = max(1e-4, now - self.last_joint_time)
            data = msg.data

            for i in range(min(6, len(data))):
                val = data[i]
                raw_vel = (val - self.prev_joint_positions[i]) / dt
                self.joint_velocities[i] = 0.7 * self.joint_velocities[i] + 0.3 * raw_vel
                self.prev_joint_positions[i] = self.joint_positions[i]
                self.joint_positions[i] = val

            self.last_joint_time = now

            # Live RTH Progress computation
            if self.rth_status == "ACTIVE":
                dist = math.hypot(*[self.joint_positions[i] - self.HOME_JOINTS[i] for i in range(5)])
                self.rth_progress = max(0.0, min(1.0, 1.0 - (dist / self.rth_initial_dist)))

            # Recompute live forward kinematics of physical wrist
            self._update_fk()

    def _arm_ik_cmd_cb(self, msg: Float64MultiArray):
        with self.lock:
            self.rate_ik_cmd.tick()
            if len(msg.data) >= 5:
                self.target_r = msg.data[0]
                self.target_theta = msg.data[1]
                self.target_z = msg.data[2]
                self.target_world_pitch = msg.data[3]
                self.target_roll = msg.data[4]
            self.control_mode = "IK Mode (Mode 1)"

    def _target_pose_cb(self, msg: PoseStamped):
        with self.lock:
            self.cmd_x = msg.pose.position.x
            self.cmd_y = msg.pose.position.y
            self.cmd_z = msg.pose.position.z

    def _joy_cb(self, msg: Joy):
        with self.lock:
            self.rate_joy.tick()
            if len(msg.buttons) > 10:
                self.crawl_active = bool(msg.buttons[4])
                self.orientation_active = bool(msg.buttons[5])

    def _gripper_cb(self, msg: Float64MultiArray):
        with self.lock:
            if len(msg.data) >= 2:
                self.is_gripping = bool(msg.data[1] > 0.5)
                self.gripper_load = float(msg.data[1])

    def _rosout_cb(self, msg: Log):
        relevant_nodes = ("ps5_mapper", "ik_solver_node", "rth_node", "rviz_topic_bridge")
        if any(node in msg.name for node in relevant_nodes):
            t_str = time.strftime("%H:%M:%S", time.localtime(msg.stamp.sec))
            lvl = "INFO"
            if msg.level == 30:
                lvl = "WARN"
            elif msg.level >= 40:
                lvl = "ERROR"

            clean_text = msg.msg.strip().replace("\n", " ")
            if len(clean_text) > 85:
                clean_text = clean_text[:82] + "..."

            with self.lock:
                if "[RTH] ENGAGED" in clean_text:
                    self.rth_status = "ACTIVE"
                    q = self.joint_positions
                    self.rth_initial_dist = max(0.01, math.hypot(*[q[i] - self.HOME_JOINTS[i] for i in range(5)]))
                    self.rth_progress = 0.0
                elif "[RTH] Complete" in clean_text or "[RTH] Ended" in clean_text or "[RTH] Canceled" in clean_text:
                    self.rth_status = "IDLE"
                    self.rth_progress = 1.0
                elif "EMERGENCY STOP ENGAGED" in clean_text:
                    self.estop_active = True
                elif "EMERGENCY STOP CLEARED" in clean_text:
                    self.estop_active = False
                elif "Switched to IK Mode" in clean_text:
                    self.control_mode = "IK Mode (Mode 1)"
                elif "Switched to FK Mode" in clean_text:
                    self.control_mode = "FK Mode (Mode 0)"
                elif "[SPEED TRIM]" in clean_text:
                    self._parse_speed_trim(clean_text)
                elif "[YAW GUARD]" in clean_text:
                    self.solver_status = "YAW-GUARD ACTIVE"
                elif "IK solver could not find" in clean_text:
                    self.solver_status = "BOUNDARY DECOUPLED"
                elif "Kinematics solver successfully loaded" in clean_text:
                    self.solver_status = "SOLVED (TRAC-IK)"

                self.log_messages.append((t_str, lvl, f"[{msg.name}] {clean_text}"))

    def _parse_speed_trim(self, text: str):
        """Parses individual FK and IK speed trim updates logged by ps5_mapper."""
        try:
            parts = text.split("[SPEED TRIM]")[1].split("max speed:")
            axis = parts[0].strip()
            val = float(parts[1].strip().split()[0])
            if "Base Yaw" in axis:
                self.fk_speed_base = val
            elif "Azimuth" in axis:
                self.ik_speed_azimuth = val
            elif "Shoulder Pitch" in axis:
                self.fk_speed_shoulder = val
            elif "Reach" in axis:
                self.ik_speed_reach = val
            elif "Elbow Pitch" in axis:
                self.fk_speed_elbow = val
            elif "Elevation" in axis:
                self.ik_speed_elev = val
            elif "Wrist" in axis:
                self.fk_speed_wrist = val
            elif "Pitch & Roll" in axis or "World Pitch" in axis:
                self.ik_speed_world_pitch = val
                self.ik_speed_roll = val
            elif "Gripper" in axis:
                self.speed_gripper = val
        except Exception:
            pass

    # ── Forward Kinematics & Leash Math ──────────────────────────────────────

    def _update_fk(self):
        """Computes analytical 3D forward kinematics for wrist_center in nanoseconds."""
        q = self.joint_positions
        q1 = q[1]
        theta2 = q[1] - 2.00719 + q[2]

        # Exact closed-form kinematics matching URDF link origins:
        # Upper arm: length 0.450 along shoulder angle q1
        # Forearm: local origin (0.0005, -0.47751, -0.222701) rotated by theta2
        self.act_r = 0.450 * math.sin(q1) + 0.47751 * math.cos(theta2) - 0.222701 * math.sin(theta2)
        self.act_z = 0.10425 + 0.450 * math.cos(q1) - 0.47751 * math.sin(theta2) - 0.222701 * math.cos(theta2)
        self.act_theta = q[0]
        self.act_world_pitch = -(theta2 + q[3] + 0.4363323)
        self.act_roll = q[4]

        self.act_x = (
            self.BASE_PIVOT_X
            + self.act_r * math.sin(self.act_theta)
            + self.ARM_LATERAL_OFFSET * math.cos(self.act_theta)
        )
        self.act_y = (
            self.BASE_PIVOT_Y
            - self.act_r * math.cos(self.act_theta)
            + self.ARM_LATERAL_OFFSET * math.sin(self.act_theta)
        )

        if "FK" in self.control_mode:
            self.target_r = self.act_r
            self.target_z = self.act_z
            self.target_theta = self.act_theta
            self.target_world_pitch = self.act_world_pitch
            self.target_roll = self.act_roll
            self.leash_dist_m = 0.0
        else:
            dr = self.target_r - self.act_r
            dz = self.target_z - self.act_z
            dtheta = self.target_theta - self.act_theta
            while dtheta > math.pi:
                dtheta -= 2.0 * math.pi
            while dtheta < -math.pi:
                dtheta += 2.0 * math.pi
            d_tangential = max(0.10, self.act_r) * dtheta
            self.leash_dist_m = math.hypot(dr, d_tangential, dz)

        if "IK" in self.control_mode:
            self.solver_status = "SOLVED (TRAC-IK)"
        else:
            self.solver_status = "IDLE (FK MODE)"

        z_rel = self.act_z - self.SHOULDER_PIVOT_Z
        self.sphere_dist_m = math.hypot(self.act_r, z_rel)

    # ── Tab Navigation ───────────────────────────────────────────────────────

    def set_tab(self, tab_idx: int):
        with self.lock:
            self.active_tab = max(0, min(3, tab_idx))

    def next_tab(self):
        with self.lock:
            self.active_tab = (self.active_tab + 1) % 4

    def prev_tab(self):
        with self.lock:
            self.active_tab = (self.active_tab - 1) % 4

    def toggle_autocycle(self):
        with self.lock:
            self.autocycle_enabled = not self.autocycle_enabled
            self.last_autocycle_time = time.time()

    # ── UI Rendering & Layout Generation ─────────────────────────────────────

    def generate_layout(self) -> Layout:
        """Constructs the responsive TUI layout based on active tab and master state."""
        if self.autocycle_enabled:
            now = time.time()
            if now - self.last_autocycle_time > self.autocycle_period:
                self.last_autocycle_time = now
                self.active_tab = (self.active_tab + 1) % 4

        with self.lock:
            header_panel = self._build_header_panel()
            tab_bar = self._build_tab_bar()

            layout = Layout()

            if self.active_tab == 0:
                # Overview (All-in-One View)
                joint_panel = self._build_joint_table()
                task_panel = self._build_task_space_panel()
                teleop_panel = self._build_teleop_panel()
                footer_panel = self._build_footer_panel()

                layout.split(
                    Layout(name="header", size=4),
                    Layout(name="tab_bar", size=1),
                    Layout(name="joints", size=10),
                    Layout(name="middle", size=9),
                    Layout(name="footer", size=7),
                )
                layout["header"].update(header_panel)
                layout["tab_bar"].update(tab_bar)
                layout["joints"].update(joint_panel)
                layout["middle"].split_row(
                    Layout(task_panel, ratio=1),
                    Layout(teleop_panel, ratio=1),
                )
                layout["footer"].update(footer_panel)

            elif self.active_tab == 1:
                # Tab 1: Joints & Limits Focus (Expands to fill terminal height)
                joint_panel = self._build_joint_table(expanded=True)
                layout.split(
                    Layout(name="header", size=4),
                    Layout(name="tab_bar", size=1),
                    Layout(name="joints", ratio=1),
                )
                layout["header"].update(header_panel)
                layout["tab_bar"].update(tab_bar)
                layout["joints"].update(joint_panel)

            elif self.active_tab == 2:
                # Tab 2: Task-Space & Kinematics Focus
                task_panel = self._build_task_space_panel(expanded=True)
                layout.split(
                    Layout(name="header", size=4),
                    Layout(name="tab_bar", size=1),
                    Layout(name="task_space", ratio=1),
                )
                layout["header"].update(header_panel)
                layout["tab_bar"].update(tab_bar)
                layout["task_space"].update(task_panel)

            elif self.active_tab == 3:
                # Tab 3: Teleop Speeds & Event Logs Focus
                teleop_panel = self._build_teleop_panel(expanded=True)
                footer_panel = self._build_footer_panel(expanded=True)
                layout.split(
                    Layout(name="header", size=4),
                    Layout(name="tab_bar", size=1),
                    Layout(name="speeds", ratio=1),
                    Layout(name="footer", ratio=1),
                )
                layout["header"].update(header_panel)
                layout["tab_bar"].update(tab_bar)
                layout["speeds"].update(teleop_panel)
                layout["footer"].update(footer_panel)

            return layout

    def _build_header_panel(self) -> Panel:
        """Constructs the high-contrast Master State Banner emphasizing active system mode."""
        watchdog_ok = self.rate_joy.is_active(timeout_sec=0.25)
        rate_j = self.rate_joy.get_rate()

        now_sec = time.time()
        # 1.0 Hz flash for E-Stop (500ms ON / 500ms OFF)
        flash_estop = (int(now_sec * 2.0) % 2 == 0)
        # 0.5 Hz flash for Joystick Signal Loss (1000ms ON / 1000ms OFF)
        flash_joy = (int(now_sec * 1.0) % 2 == 0)

        if self.estop_active:
            banner_title = "🛑  EMERGENCY STOP ACTIVE  │  MOTION LOCKED"
            banner_subtitle = "PS Button pressed — all motor position increments frozen"
            if flash_estop:
                border_color = "bold red"
                badge_style = "bold white on red"
            else:
                border_color = "dim red"
                badge_style = "bold red on black"
        elif not watchdog_ok:
            banner_title = "⚠️  JOYSTICK SIGNAL LOST  │  WATCHDOG TIMEOUT"
            banner_subtitle = "Communication timeout (>0.25s) — holding last commanded pose"
            if flash_joy:
                border_color = "bold yellow"
                badge_style = "bold black on yellow"
            else:
                border_color = "dim yellow"
                badge_style = "bold yellow on black"
        elif self.rth_status == "ACTIVE":
            pct = int(self.rth_progress * 100)
            banner_title = f"🔄  RETURN-TO-HOME EXECUTING  │  {pct}% COMPLETE"
            banner_subtitle = "Interpolating smoothly to home posture (move any stick to cancel)"
            border_color = "bold yellow"
            badge_style = "bold black on yellow"
        elif "IK" in self.control_mode:
            banner_title = "🎯  IK MODE 1 (CARTESIAN TELEOP)  │  TRAC-IK"
            banner_subtitle = "Cylindrical (r, θ, z) translation with auto-horizon leveling"
            border_color = "bold cyan"
            badge_style = "bold black on cyan"
        else:
            banner_title = "🕹️  FK MODE 0 (DIRECT JOINT CONTROL)  │  MANUAL"
            banner_subtitle = "Direct independent joint velocity integration"
            border_color = "bold magenta"
            badge_style = "bold white on magenta"

        # Precision Mode (Engaged / Disengaged)
        prec_mode = "[bold yellow]Engaged[/bold yellow]" if self.crawl_active else "[dim]Disengaged[/dim]"
        prec_badge = f"Precision Mode: {prec_mode}"

        # Right Stick Controls: Elbow / Wrist
        rs_target = "[bold yellow]Wrist[/bold yellow]" if self.orientation_active else "[cyan]Elbow[/cyan]"
        rs_badge = f"Right Stick Controls: {rs_target}"

        if watchdog_ok:
            joy_badge = f"[green]JOY {rate_j:.0f}Hz[/green]"
        else:
            joy_badge = "[bold black on yellow] NO JOY [/bold black on yellow]" if flash_joy else "[bold yellow on black] NO JOY [/bold yellow on black]"

        markup = (
            f"[{badge_style}] {banner_title} [/{badge_style}]   [dim]({banner_subtitle})[/dim]\n"
            f"{prec_badge}  │  {rs_badge}  │  {joy_badge}  │  Solver: [cyan]{self.solver_status}[/cyan]"
        )
        return Panel(Text.from_markup(markup), border_style=border_color, height=4)

    def _build_tab_bar(self) -> Text:
        """Constructs the interactive tab selection navigation bar."""
        tabs = [
            (0, "0: Overview"),
            (1, "1: Joints & Limits"),
            (2, "2: Task-Space & RTH"),
            (3, "3: Speeds & Logs"),
        ]
        parts = []
        for idx, name in tabs:
            if idx == self.active_tab:
                parts.append(f"[bold black on white] {name} [/bold black on white]")
            else:
                parts.append(f"[dim]{name}[/dim]")

        cycle_str = "[bold green]ON (2s)[/bold green]" if self.autocycle_enabled else "[dim]OFF[/dim]"
        nav_hint = f"[dim]│ Keys: [bold]0-3[/bold]=Tab  [bold]TAB[/bold]/[bold]Shift+TAB[/bold]=Next/Prev  [bold]A[/bold]=Auto-Cycle ({cycle_str})  [bold]Q[/bold]=Quit[/dim]"

        return Text.from_markup("  " + "   ".join(parts) + "   " + nav_hint)

    def _build_joint_table(self, expanded: bool = False) -> Panel:
        """Builds the joint telemetry table with ground angles and limit meters."""
        table = Table(box=None, expand=True, padding=(0, 1), show_header=True, header_style="bold cyan")
        table.add_column("Joint", style="bold white", width=18)
        table.add_column("wrt Ground [deg]", justify="right", style="bold yellow", width=19)
        table.add_column("ROS Joint [rad]", justify="right", width=15)
        table.add_column("Velocity", justify="right", width=12)
        table.add_column("Limit Range", justify="center", style="dim", width=18)
        table.add_column("Position Limit Bar", justify="left", width=30 if not expanded else 45)

        q = self.joint_positions
        vels = self.joint_velocities

        gripper_pct = int(round(max(0.0, min(1.0, (q[5] - self.JOINT_LIMITS[5][0]) / (self.JOINT_LIMITS[5][1] - self.JOINT_LIMITS[5][0]))) * 100))

        ground_angles = [
            math.degrees(q[0]),                                             # J0: Base Yaw (Azimuth)
            90.0 - math.degrees(q[1]),                                      # J1: Shoulder (Elevation)
            -math.degrees(q[1] + q[2] - 2.00719),                           # J2: Elbow (Forearm)
            -math.degrees(q[1] + q[2] - 2.00719 + q[3] + 0.4363323),        # J3: Wrist (Horizon)
            math.degrees(q[4]),                                             # J4: Wrist Roll
            math.degrees(q[5]),                                             # J5: Gripper
        ]

        ground_labels = [
            f"{ground_angles[0]:+6.1f}° (Azimuth)",
            f"{ground_angles[1]:+6.1f}° (Elevation)",
            f"{ground_angles[2]:+6.1f}° (Forearm)",
            f"{ground_angles[3]:+6.1f}° (Horizon)",
            f"{ground_angles[4]:+6.1f}° (Roll)",
            f"{gripper_pct:3d}% Open",
        ]

        bar_width = 13 if not expanded else 21

        for i in range(6):
            name = self.JOINT_NAMES[i]
            q_rad = q[i]
            v_degs = math.degrees(vels[i])
            min_lim, max_lim = self.JOINT_LIMITS[i]

            min_deg = math.degrees(min_lim)
            max_deg = math.degrees(max_lim)

            if i == 5:
                range_str = "[   0%,   100%]"
                bar_markup = self._format_gripper_bar(q_rad, min_lim, max_lim, width=bar_width)
            else:
                range_str = f"[{min_deg:+5.0f}°, {max_deg:+5.0f}°]"
                bar_markup = self._format_limit_bar(q_rad, min_lim, max_lim, width=bar_width)

            vel_style = "dim"
            if abs(v_degs) > 15.0:
                vel_style = "bold yellow"
            elif abs(v_degs) > 2.0:
                vel_style = "green"

            table.add_row(
                name,
                ground_labels[i],
                f"{q_rad:+7.3f} rad",
                f"[{vel_style}]{v_degs:+6.1f}°/s[/{vel_style}]",
                range_str,
                bar_markup,
            )

        title = "[bold white]Joint-Space Telemetry[/bold white] (Primary: [yellow]wrt Ground [deg][/yellow] | Secondary: ROS [rad])"
        return Panel(table, title=title, border_style="cyan")

    def _format_limit_bar(self, val: float, min_v: float, max_v: float, width: int = 13) -> str:
        """Generates a color-coded ASCII bar meter representing limit proximity (-100% to +100%)."""
        span = max_v - min_v
        u = max(0.0, min(1.0, (val - min_v) / span)) if span > 1e-5 else 0.5
        pos = int(round(u * (width - 1)))
        mid = width // 2

        if u <= 0.05 or u >= 0.95:
            color = "bold red"
            tag = "[bold red]LIMIT[/bold red]"
        elif u <= 0.15 or u >= 0.85:
            color = "yellow"
            tag = "[yellow] WARN[/yellow]"
        else:
            color = "green"
            tag = "[green]  OK [/green]"

        chars = ["[dim]-[/dim]"] * width
        chars[mid] = "[dim]|[/dim]"

        if pos < mid:
            for i in range(pos + 1, mid):
                chars[i] = f"[{color}]=[/{color}]"
            chars[pos] = f"[{color}]◀[/{color}]"
        elif pos > mid:
            for i in range(mid + 1, pos):
                chars[i] = f"[{color}]=[/{color}]"
            chars[pos] = f"[{color}]▶[/{color}]"
        else:
            chars[mid] = "[bold white]|[/bold white]"

        bar_str = "".join(chars)

        pct = int(round((2.0 * u - 1.0) * 100))
        if pct > 0:
            pct_str = f"+{pct}%"
        elif pct < 0:
            pct_str = f"{pct}%"
        else:
            pct_str = "0%"

        return f"[{bar_str}] [{color}]{pct_str:>5}[/{color}] {tag}"

    def _format_gripper_bar(self, val: float, min_v: float, max_v: float, width: int = 13) -> str:
        """Generates a 0% to 100% progress bar meter for the gripper jaw."""
        span = max_v - min_v
        u = max(0.0, min(1.0, (val - min_v) / span)) if span > 1e-5 else 0.0
        pct = int(round(u * 100))

        fill = int(round(u * width))
        empty = width - fill

        if self.is_gripping:
            color = "bold green"
            tag = "[bold green] GRIP[/bold green]"
        elif pct <= 2:
            color = "cyan"
            tag = "[cyan] CLSD[/cyan]"
        elif pct >= 98:
            color = "bold cyan"
            tag = "[bold cyan] OPEN[/bold cyan]"
        else:
            color = "cyan"
            tag = "[green]  OK [/green]"

        bar_str = f"[{color}]{'=' * fill}[/{color}][dim]{'-' * empty}[/dim]"
        pct_str = f"{pct}%"

        return f"[{bar_str}] [{color}]{pct_str:>5}[/{color}] {tag}"

    def _build_task_space_panel(self, expanded: bool = False) -> Panel:
        """Builds the task-space kinematics, leash anti-windup, and RTH progress panel."""
        table = Table(box=None, expand=True, padding=(0, 1), show_header=False)
        table.add_column("Field", style="dim", width=22)
        table.add_column("Value", style="bold white", width=32)

        # Cylindrical
        table.add_row("Radial Reach (r):", f"[bold green]{self.act_r:.3f} m[/bold green]")
        table.add_row("Elevation (z):", f"[bold green]{self.act_z:.3f} m[/bold green]")
        table.add_row("Azimuth (theta):", f"{math.degrees(self.act_theta):+5.1f}° ({self.act_theta:+5.3f} rad)")

        # 3D Cartesian Position
        table.add_row("Wrist (X, Y, Z):", f"({self.act_x:+.3f}, {self.act_y:+.3f}, {self.act_z:+.3f}) m")
        table.add_row("Ground World Pitch:", f"{math.degrees(self.act_world_pitch):+5.1f}° (Horizon)")

        # Dynamic Leash (0 to 25 mm)
        if "FK" in self.control_mode:
            table.add_row("Dynamic Leash (3D):", "[dim]IDLE (FK MODE)[/dim]")
        else:
            leash_mm = self.leash_dist_m * 1000.0
            leash_ratio = min(1.0, leash_mm / (self.LEASH_MAX_M * 1000.0))
            leash_color = "green" if leash_mm < 12.0 else ("yellow" if leash_mm < 20.0 else "bold red")
            leash_bar = self._format_gauge(leash_ratio, width=10, color=leash_color)
            table.add_row("Dynamic Leash (3D):", f"{leash_mm:4.1f}/25.0 mm {leash_bar}")

        # Spherical Workspace Margin
        sphere_ratio = min(1.0, self.sphere_dist_m / self.WORKSPACE_RADIUS)
        sphere_color = "green" if sphere_ratio < 0.85 else ("yellow" if sphere_ratio < 0.95 else "bold red")
        sphere_bar = self._format_gauge(sphere_ratio, width=10, color=sphere_color)
        table.add_row("Workspace Sphere:", f"{self.sphere_dist_m:.3f}/0.980 m {sphere_bar}")

        # Return-to-Home Action Progress Bar
        if self.rth_status == "ACTIVE":
            pct = int(self.rth_progress * 100)
            rth_color = "green" if self.rth_progress > 0.8 else "yellow"
            rth_bar = self._format_gauge(self.rth_progress, width=12, color=rth_color)
            table.add_row("[bold yellow]RTH Action Progress:[/bold yellow]", f"[bold yellow]{pct}%[/bold yellow] {rth_bar}")

        if expanded:
            table.add_row("Tool Roll Angle:", f"{math.degrees(self.act_roll):+5.1f}° ({self.act_roll:+.3f} rad)")
            table.add_row("IK Motion Solver:", f"[cyan]{self.solver_status}[/cyan]")

        return Panel(table, title="[bold white]Task-Space Kinematics & Workspace[/bold white]", border_style="magenta")

    def _build_teleop_panel(self, expanded: bool = False) -> Panel:
        """Builds teleoperation speeds table with context-switching (FK vs IK) and gripper state."""
        table = Table(box=None, expand=True, padding=(0, 1), show_header=True, header_style="bold yellow")
        table.add_column("Parameter", style="bold white", width=22)
        table.add_column("Status / Speed", style="bold white", width=26)

        # Gripper row
        grip_pct = int(round(max(0.0, min(1.0, (self.joint_positions[5] - self.JOINT_LIMITS[5][0]) / (self.JOINT_LIMITS[5][1] - self.JOINT_LIMITS[5][0]))) * 100))
        if self.is_gripping:
            grip_str = f"[bold green]GRIPPING[/bold green] ({grip_pct}% | Load: {self.gripper_load:.1f})"
        else:
            state_desc = "CLOSED" if grip_pct <= 2 else ("OPEN" if grip_pct >= 98 else f"{grip_pct}% OPEN")
            grip_str = f"[cyan]{state_desc}[/cyan]"
        table.add_row("Gripper State:", grip_str)
        table.add_row("Gripper Max Speed:", f"{self.speed_gripper:.2f} rad/s")

        # Context-Switching Speed Trims (FK vs IK)
        if "FK" in self.control_mode:
            table.add_row("── FK Joint Trims ──", "[dim](rad/s)[/dim]")
            table.add_row("Base Yaw Speed:", f"{self.fk_speed_base:.2f} rad/s")
            table.add_row("Shoulder Pitch Speed:", f"{self.fk_speed_shoulder:.2f} rad/s")
            table.add_row("Elbow Pitch Speed:", f"{self.fk_speed_elbow:.2f} rad/s")
            table.add_row("Wrist Pitch/Roll Speed:", f"{self.fk_speed_wrist:.2f} rad/s")
            border_col = "magenta"
            title = "[bold magenta]Teleoperation Speeds (FK Mode 0)[/bold magenta]"
        else:
            table.add_row("── IK Cartesian Trims ──", "[dim](m/s & rad/s)[/dim]")
            table.add_row("Base Azimuth Speed:", f"{self.ik_speed_azimuth:.2f} rad/s")
            table.add_row("Radial Reach Speed:", f"{self.ik_speed_reach:.2f} m/s")
            table.add_row("Vertical Elev Speed:", f"{self.ik_speed_elev:.2f} m/s")
            table.add_row("World Pitch Speed:", f"{self.ik_speed_world_pitch:.2f} rad/s")
            table.add_row("Axial Roll Speed:", f"{self.ik_speed_roll:.2f} rad/s")
            border_col = "cyan"
            title = "[bold cyan]Teleoperation Speeds (IK Mode 1)[/bold cyan]"

        if self.rth_status == "ACTIVE":
            pct = int(self.rth_progress * 100)
            table.add_row("RTH Routine State:", f"[bold yellow]ACTIVE ({pct}%)[/bold yellow]")
        else:
            table.add_row("RTH Routine State:", "[dim]IDLE[/dim]")

        return Panel(table, title=title, border_style=border_col)

    def _build_footer_panel(self, expanded: bool = False) -> Panel:
        """Builds system rates and recent message log scrollbox."""
        rate_cmd = self.rate_arm_cmd.get_rate()
        rate_joy = self.rate_joy.get_rate()

        cmd_style = "bold green" if rate_cmd >= 45.0 else ("yellow" if rate_cmd >= 30.0 else "bold red")
        joy_style = "bold green" if rate_joy >= 45.0 else ("yellow" if rate_joy >= 15.0 else "dim")
        solv_style = "bold green" if "SOLVED" in self.solver_status else ("bold yellow" if "GUARD" in self.solver_status else "dim")

        diag_line = (
            f"/arm_cmd: [{cmd_style}]{rate_cmd:4.1f} Hz[/{cmd_style}]   │   "
            f"/joy: [{joy_style}]{rate_joy:4.1f} Hz[/{joy_style}]   │   "
            f"IK Solver: [{solv_style}]{self.solver_status}[/{solv_style}]"
        )

        log_count = 7 if expanded else 3
        log_lines = []
        for t_str, lvl, text in list(self.log_messages)[-log_count:]:
            lvl_color = "cyan" if lvl == "INFO" else ("yellow" if lvl == "WARN" else "bold red")
            log_lines.append(f"[dim]{t_str}[/dim] [{lvl_color}][{lvl}][/{lvl_color}] {text}")

        while len(log_lines) < (log_count if not expanded else 5):
            log_lines.append("[dim]...[/dim]")

        content = Text.from_markup(f"{diag_line}\n" + "─" * 90 + "\n" + "\n".join(log_lines))
        return Panel(content, title="[bold white]Diagnostics, Rates & Event Ticker[/bold white]", border_style="green", height=7 if not expanded else None)

    def _format_gauge(self, ratio: float, width: int = 10, color: str = "green") -> str:
        """Formats a miniature progress gauge [====------]."""
        clamped = max(0.0, min(1.0, ratio))
        fill_count = int(round(clamped * width))
        empty_count = width - fill_count
        return f"[[{color}]{'=' * fill_count}[/{color}][dim]{'-' * empty_count}[/dim]] [{color}]{int(clamped * 100):2d}%[/{color}]"


def _start_keyboard_listener(node: ArmDashboardNode):
    """Starts a non-blocking background thread reading user keypresses for tab navigation."""
    if not sys.stdin.isatty():
        return

    import termios
    import tty

    def _worker():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while rclpy.ok():
                r, _, _ = select.select([sys.stdin], [], [], 0.2)
                if r:
                    ch = sys.stdin.read(1)
                    if ch in ('0', '1', '2', '3'):
                        node.set_tab(int(ch))
                    elif ch == '\t':  # Tab -> forward
                        node.next_tab()
                    elif ch == '\x1b':  # Escape sequence, e.g. Shift-Tab (\x1b[Z)
                        r_seq, _, _ = select.select([sys.stdin], [], [], 0.05)
                        if r_seq:
                            seq = sys.stdin.read(2)
                            if seq == '[Z':  # Shift-Tab (Back-Tab)
                                node.prev_tab()
                    elif ch == ' ':
                        node.next_tab()
                    elif ch in ('a', 'A'):
                        node.toggle_autocycle()
                    elif ch in ('q', 'Q'):
                        os._exit(0)
        except Exception:
            pass
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            except Exception:
                pass

    thread = threading.Thread(target=_worker, daemon=True)
    thread.start()


def main(args=None):
    parser = argparse.ArgumentParser(description="Arm Telemetry Dashboard")
    parser.add_argument("--parent-pid", type=int, default=None, help="PID of launcher process to monitor")
    parsed_args, remaining_args = parser.parse_known_args()

    # If launched from ros2 launch, monitor parent PID to auto-close window when launch is terminated (Ctrl+C)
    if parsed_args.parent_pid:
        def _parent_watchdog(pid: int):
            while True:
                time.sleep(0.25)
                try:
                    os.kill(pid, 0)
                except (ProcessLookupError, OSError):
                    os._exit(0)

        watchdog_thread = threading.Thread(target=_parent_watchdog, args=(parsed_args.parent_pid,), daemon=True)
        watchdog_thread.start()

    rclpy.init(args=remaining_args if remaining_args else args)
    node = ArmDashboardNode()

    # Start keyboard listener for interactive tab navigation
    _start_keyboard_listener(node)

    # Spin ROS 2 subscriptions in a background daemon thread
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    console = Console()
    with Live(node.generate_layout(), console=console, screen=True, refresh_per_second=10) as live:
        try:
            while rclpy.ok():
                live.update(node.generate_layout())
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
