#!/usr/bin/env python3
"""
arm_dashboard.py - Live Telemetry & Diagnostics TUI Dashboard for Rudimentary FK Velocity Controller.

Visualizes real-time 5-joint PWM velocity commands (-255 to +255), controller states,
E-Stop interlocks, watchdog heartbeat, and communication rates using Python's 'rich' library.
"""

import os
import sys
import time
import signal
import argparse
import collections
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
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
    """ROS 2 node subscribing to /arm_cmd and /joy telemetry for live TUI rendering."""

    JOINT_NAMES = [
        "J0: Base Yaw",
        "J1: Shoulder Pitch",
        "J2: Elbow Pitch",
        "J3: Wrist Pitch",
        "J4: Wrist Roll",
        "J5: Gripper Jaw",
    ]

    def __init__(self):
        super().__init__('arm_dashboard')

        self.lock = threading.Lock()

        # Parameters
        self.declare_parameter('refresh_rate', 30.0)  # TUI refresh frequency (Hz)
        self.refresh_rate = float(self.get_parameter('refresh_rate').value)

        # Commanded PWM velocities: [J0, J1, J2, J3, J4, Gripper] in [-255, 255]
        self.cmd_pwm = [0, 0, 0, 0, 0, 0]

        # Safety & Modifier State
        self.estop_active = False
        self.joy_connected = False
        self.crawl_active = False
        self.wrist_layer_active = False

        # Topic Rates
        self.rate_joy = RateEstimator(window_size=30)
        self.rate_arm_cmd = RateEstimator(window_size=30)

        # Recent Diagnostics Logs (maxlen=8)
        self.log_messages = collections.deque(maxlen=8)
        self.log_messages.append((time.strftime("%H:%M:%S"), "INFO", "Arm Telemetry Dashboard started."))

        # Subscriptions
        self.create_subscription(Int16MultiArray, 'arm_cmd', self._arm_cmd_cb, 10)
        self.create_subscription(Joy, 'joy', self._joy_cb, 10)
        self.create_subscription(Log, '/rosout', self._rosout_cb, 20)

    def _arm_cmd_cb(self, msg: Int16MultiArray):
        with self.lock:
            self.rate_arm_cmd.tick()
            self.cmd_pwm = [int(x) for x in msg.data]

    def _joy_cb(self, msg: Joy):
        with self.lock:
            self.rate_joy.tick()
            self.joy_connected = True
            if len(msg.buttons) > 5:
                self.crawl_active = bool(msg.buttons[4])        # LB
                self.wrist_layer_active = bool(msg.buttons[5])  # RB

    def _rosout_cb(self, msg: Log):
        if "ps5_mapper" in msg.name or "joy" in msg.name:
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
                if "EMERGENCY STOP ENGAGED" in clean_text:
                    self.estop_active = True
                elif "EMERGENCY STOP CLEARED" in clean_text:
                    self.estop_active = False
                elif "timed out" in clean_text:
                    self.joy_connected = False
                elif "signal restored" in clean_text:
                    self.joy_connected = True

                self.log_messages.append((t_str, lvl, f"[{msg.name}] {clean_text}"))


def make_bipolar_bar(val: int, max_val: int = 255, half_w: int = 10) -> str:
    """Generates a centered bipolar bar meter [-max_val ... 0 ... +max_val]."""
    track = "─" * half_w
    if val == 0:
        return f"[dim]{track}│{track}[/dim]"

    ratio = max(-1.0, min(1.0, val / float(max_val)))
    if ratio > 0:
        fill_len = int(round(ratio * half_w))
        fill = "█" * fill_len
        rem = "─" * (half_w - fill_len)
        return f"[dim]{track}[/dim][green]│{fill}[/green][dim]{rem}[/dim]"
    else:
        fill_len = int(round(abs(ratio) * half_w))
        fill = "█" * fill_len
        rem = "─" * (half_w - fill_len)
        return f"[dim]{rem}[/dim][cyan]{fill}│[/cyan][dim]{track}[/dim]"


def generate_layout(node: ArmDashboardNode) -> Layout:
    layout = Layout()
    layout.split_column(
        Layout(name="banner", size=3),
        Layout(name="main", ratio=1),
        Layout(name="footer", size=3),
    )

    with node.lock:
        estop = node.estop_active
        joy_ok = node.joy_connected and node.rate_joy.is_active(timeout_sec=0.5)
        crawl = node.crawl_active
        wrist_layer = node.wrist_layer_active
        cmd_pwm = list(node.cmd_pwm)
        rate_cmd = node.rate_arm_cmd.get_rate()
        rate_joy = node.rate_joy.get_rate()
        logs = list(node.log_messages)

    # 1. State Banner
    banner_text = Text()
    if estop:
        banner_text.append(" [ EMERGENCY STOP ENGAGED ] ", style="bold white on red blink")
    elif not joy_ok:
        banner_text.append(" [ JOYSTICK TIMEOUT / DISCONNECTED ] ", style="bold white on red")
    else:
        banner_text.append(" [ SYSTEM OPERATIONAL ] ", style="bold white on green")

    banner_text.append("   Mode: ")
    banner_text.append("FK VELOCITY (STM32 PWM)", style="bold cyan")

    if wrist_layer:
        banner_text.append("  │  Layer: ", style="dim")
        banner_text.append("WRIST GIMBAL (RB)", style="bold yellow")
    else:
        banner_text.append("  │  Layer: ", style="dim")
        banner_text.append("DEFAULT REACH", style="bold white")

    if crawl:
        banner_text.append("  │  Modifier: ", style="dim")
        banner_text.append("CRAWL 30% (LB)", style="bold magenta")

    layout["banner"].update(Panel(banner_text, style="white", border_style="cyan"))

    # 2. Main Content Split: PWM Table on left, Logs & Rates on right
    layout["main"].split_row(
        Layout(name="joints", ratio=2),
        Layout(name="side", ratio=1),
    )

    # Joint PWM Table
    table = Table(title="[bold]Arm Joint Velocity Commands (/arm_cmd)[/bold]", expand=True)
    table.add_column("Joint", style="bold white", width=20, no_wrap=True)
    table.add_column("PWM (-255..+255)", justify="right", width=16, no_wrap=True)
    table.add_column("Polarity", justify="center", width=12, no_wrap=True)
    table.add_column("Velocity Meter [-255..0..+255]", justify="center", no_wrap=True)

    for i, name in enumerate(node.JOINT_NAMES):
        if i >= len(cmd_pwm):
            break
        val = cmd_pwm[i]
        pct = (val / 255.0) * 100.0

        if val > 0:
            val_str = f"[green]{val:+4d} ({pct:+5.1f}%)[/green]"
            pol_str = "[bold green]FORWARD[/bold green]" if i < 5 else "[bold green]OPEN[/bold green]"
        elif val < 0:
            val_str = f"[cyan]{val:+4d} ({pct:+5.1f}%)[/cyan]"
            pol_str = "[bold cyan]REVERSE[/bold cyan]" if i < 5 else "[bold cyan]CLOSE[/bold cyan]"
        else:
            val_str = "[dim]   0 (  0.0%)[/dim]"
            pol_str = "[dim]NEUTRAL[/dim]"

        bar = make_bipolar_bar(val, max_val=255, half_w=10)
        table.add_row(name, val_str, pol_str, bar)

    layout["joints"].update(Panel(table, border_style="blue"))

    # Side Panel: Telemetry & Logs
    side_layout = Layout()
    side_layout.split_column(
        Layout(name="rates", size=6),
        Layout(name="diagnostics", ratio=1),
    )

    rates_text = Text()
    rates_text.append(f"• /joy rate:     ", style="white")
    rates_text.append(f"{rate_joy:5.1f} Hz\n", style="green" if rate_joy > 10 else "red")
    rates_text.append(f"• /arm_cmd rate: ", style="white")
    rates_text.append(f"{rate_cmd:5.1f} Hz\n", style="green" if rate_cmd > 10 else "red")
    rates_text.append(f"• Target Node:   ", style="white")
    rates_text.append("micro-ROS Agent / STM32 Bridge", style="cyan")

    side_layout["rates"].update(Panel(rates_text, title="Topic Rates", border_style="dim"))

    log_table = Table(box=None, expand=True, padding=(0, 1))
    log_table.add_column("Time", style="dim", width=8)
    log_table.add_column("Lvl", width=5)
    log_table.add_column("Message", overflow="ellipsis")

    for t_str, lvl, msg_content in logs:
        lvl_style = "green" if lvl == "INFO" else ("yellow" if lvl == "WARN" else "bold red")
        log_table.add_row(t_str, f"[{lvl_style}]{lvl}[/{lvl_style}]", msg_content)

    side_layout["diagnostics"].update(Panel(log_table, title="Diagnostic Logs", border_style="dim"))
    layout["side"].update(side_layout)

    # 3. Footer
    footer_text = Text(
        "Control Flow: PS5 Controller → /joy → ps5_mapper → /arm_cmd [-255, 255] → micro-ROS → STM32 → Motor Drivers",
        style="dim italic",
    )
    layout["footer"].update(Panel(footer_text, border_style="dim"))

    return layout


def _start_keyboard_listener():
    """Listens for 'q', 'Q', or Esc on stdin to allow operator to close dashboard immediately."""
    if not sys.stdin.isatty():
        return

    import termios
    import tty
    import select

    def _worker():
        fd = sys.stdin.fileno()
        try:
            old_settings = termios.tcgetattr(fd)
        except Exception:
            return

        try:
            tty.setcbreak(fd)
            while True:
                r, _, _ = select.select([fd], [], [], 0.20)
                if r:
                    ch = os.read(fd, 16).decode('utf-8', errors='ignore')
                    if any(c in ('q', 'Q', '\x1b', '\x03') for c in ch):
                        os._exit(0)
        except Exception:
            pass
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            except Exception:
                pass

    t = threading.Thread(target=_worker, daemon=True)
    t.start()


def main(args=None):
    # Parse launcher process PID if spawned via launch file
    parser = argparse.ArgumentParser(description="Arm Telemetry Dashboard")
    parser.add_argument("--parent-pid", type=int, default=None, help="PID of launcher process to monitor")
    parsed_args, remaining_args = parser.parse_known_args()

    # Graceful exit signal handlers
    def _sig_handler(sig, frame):
        os._exit(0)

    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)
    try:
        signal.signal(signal.SIGHUP, _sig_handler)
    except Exception:
        pass

    # If launched from ros2 launch, monitor parent PID to auto-close window when launch process dies
    if parsed_args.parent_pid:
        def _parent_watchdog(pid: int):
            while True:
                time.sleep(0.15)
                try:
                    os.kill(pid, 0)
                except (ProcessLookupError, OSError):
                    # Parent process has exited / killed
                    os._exit(0)

        watchdog_thread = threading.Thread(target=_parent_watchdog, args=(parsed_args.parent_pid,), daemon=True)
        watchdog_thread.start()

    _start_keyboard_listener()

    rclpy.init(args=remaining_args if remaining_args else args)
    node = ArmDashboardNode()

    # Spin in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    console = Console()
    target_fps = max(5.0, min(60.0, node.refresh_rate))
    frame_period = 1.0 / target_fps

    try:
        with Live(console=console, auto_refresh=False, screen=True) as live:
            while rclpy.ok():
                t0 = time.time()
                live.update(generate_layout(node), refresh=True)
                dur = time.time() - t0
                time.sleep(max(0.001, frame_period - dur))
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        os._exit(0)


if __name__ == '__main__':
    main()
