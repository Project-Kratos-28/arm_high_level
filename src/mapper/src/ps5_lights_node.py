#!/usr/bin/env python3
"""
ps5_lights_node.py - ROS 2 PS5 DualSense Controller Lights & Mode Telemetry Node.

Provides complete manipulation of the Sony PlayStation 5 DualSense controller lighting:
  - 24-bit RGB Touchpad Lightbar (color, brightness, solid/pulse/blink/rainbow effects).
  - 5 White Player Indicator LEDs (player 1-5 or custom bitmasks).
  - Autonomous robot teleoperation tracking (Mode 0 FK, Mode 1 IK, E-Stop, RTH, Crawl, Tuning, Gripper contact, Signal loss).
  - Manual ROS 2 control topics (/ps5_lights/set_color, /ps5_lights/set_effect, /ps5_lights/set_mode, etc.).
  - Multi-backend driver architecture (Linux sysfs hid-playstation, pydualsense HID reports, Mock fallback).
"""

import os
import sys
import glob
import math
import time
import json
from typing import Optional, Tuple, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String, Float32, Int32, Float32MultiArray
from sensor_msgs.msg import Joy, JoyFeedback
from action_msgs.msg import GoalStatusArray, GoalStatus


# =============================================================================
# Hardware Driver Backends
# =============================================================================

class BaseDualSenseDriver:
    """Abstract base class for DualSense LED hardware drivers."""

    def is_available(self) -> bool:
        raise NotImplementedError

    def get_generation(self) -> int:
        """Returns connection generation counter, incremented whenever hardware connects or reconnects."""
        return 0

    def set_rgb(self, r: int, g: int, b: int) -> bool:
        raise NotImplementedError

    def set_brightness(self, brightness: int) -> bool:
        raise NotImplementedError

    def set_player_leds(self, bitmask: int) -> bool:
        """Sets the 5 white player LEDs using a 5-bit mask (bits 0-4 for LEDs 1-5)."""
        raise NotImplementedError

    def close(self):
        pass


class SysfsDualSenseDriver(BaseDualSenseDriver):
    """
    Direct Linux sysfs driver for Sony hid-playstation kernel driver.
    Path: /sys/class/leds/input*:rgb:indicator and input*:white:player-*
    Zero external C/HID dependencies. Supports seamless hotplug and replug.
    """

    def __init__(self, logger):
        self.logger = logger
        self.rgb_indicator_dir = None
        self.multi_intensity_file = None
        self.brightness_file = None
        self.player_led_files = [None] * 5  # Player 1..5 brightness file paths
        self._last_scan_time = 0.0
        self._generation = 0
        self._find_devices(force=True)

    def get_generation(self) -> int:
        return self._generation

    def _find_devices(self, force: bool = False) -> bool:
        """Dynamically finds connected DualSense sysfs paths, supporting hotplug."""
        now = time.time()
        # Rate limit re-scans when not forced (max once per 0.5s)
        if not force and (now - self._last_scan_time < 0.5):
            return self.rgb_indicator_dir is not None and os.path.exists(self.rgb_indicator_dir)
        self._last_scan_time = now

        old_dir = self.rgb_indicator_dir
        found_dir = None
        found_multi = None
        found_bright = None
        found_players = [None] * 5

        # Find RGB indicator led directories matching Sony DualSense
        dirs = glob.glob("/sys/class/leds/*:rgb:indicator")
        for d in dirs:
            try:
                target = os.path.realpath(d)
                if "054c:0ce6" in target.lower():
                    found_dir = d
                    found_multi = os.path.join(d, "multi_intensity")
                    found_bright = os.path.join(d, "brightness")
                    break
            except Exception:
                continue

        if found_dir:
            base_prefix = found_dir.replace(":rgb:indicator", "")
            for p in range(1, 6):
                p_file = f"{base_prefix}:white:player-{p}/brightness"
                if os.path.exists(p_file):
                    found_players[p - 1] = p_file

        self.rgb_indicator_dir = found_dir
        self.multi_intensity_file = found_multi
        self.brightness_file = found_bright
        self.player_led_files = found_players

        if self.rgb_indicator_dir != old_dir:
            self._generation += 1
            if self.rgb_indicator_dir:
                self.logger.info(f"[SysfsDriver] DualSense LEDs connected at: {self.rgb_indicator_dir}")
            elif old_dir:
                self.logger.warn(f"[SysfsDriver] DualSense disconnected (was at {old_dir}). Waiting for reconnect...")

        return self.rgb_indicator_dir is not None

    def is_available(self) -> bool:
        if not self.multi_intensity_file or not os.path.exists(self.multi_intensity_file):
            if not self._find_devices():
                return False
        return os.access(self.multi_intensity_file, os.W_OK)

    def set_rgb(self, r: int, g: int, b: int) -> bool:
        if not self.multi_intensity_file or not os.path.exists(self.multi_intensity_file):
            if not self._find_devices():
                return False
        try:
            with open(self.multi_intensity_file, 'w') as f:
                f.write(f"{r} {g} {b}\n")
            return True
        except (FileNotFoundError, OSError, PermissionError):
            # Controller disconnected or reconnected at new input path: re-discover and retry once
            self.rgb_indicator_dir = None
            self.multi_intensity_file = None
            if self._find_devices(force=True) and self.multi_intensity_file:
                try:
                    with open(self.multi_intensity_file, 'w') as f:
                        f.write(f"{r} {g} {b}\n")
                    return True
                except Exception:
                    pass
            return False

    def set_brightness(self, brightness: int) -> bool:
        if not self.brightness_file or not os.path.exists(self.brightness_file):
            if not self._find_devices():
                return False
        try:
            with open(self.brightness_file, 'w') as f:
                f.write(f"{brightness}\n")
            return True
        except (FileNotFoundError, OSError, PermissionError):
            self.rgb_indicator_dir = None
            self.brightness_file = None
            if self._find_devices(force=True) and self.brightness_file:
                try:
                    with open(self.brightness_file, 'w') as f:
                        f.write(f"{brightness}\n")
                    return True
                except Exception:
                    pass
            return False

    def set_player_leds(self, bitmask: int) -> bool:
        first_valid = next((p for p in self.player_led_files if p is not None), None)
        if not first_valid or not os.path.exists(first_valid):
            if not self._find_devices():
                return False

        success = True
        for i in range(5):
            p_file = self.player_led_files[i]
            if p_file and os.path.exists(p_file):
                state = 1 if (bitmask & (1 << i)) else 0
                try:
                    with open(p_file, 'w') as f:
                        f.write(f"{state}\n")
                except (FileNotFoundError, OSError, PermissionError):
                    success = False
            else:
                success = False
        return success


class PyDualSenseDriver(BaseDualSenseDriver):
    """Secondary driver using the pydualsense library for direct HID reports."""

    def __init__(self, logger):
        self.logger = logger
        self.ds = None
        self._connected = False
        self._init_device()

    def _init_device(self):
        try:
            import pydualsense
            from pydualsense.enums import PlayerID, LedOptions, Brightness
            self.ds = pydualsense.pydualsense()
            self.ds.init()
            self.ds.light.setLEDOption(LedOptions.Both)
            self._connected = True
            self.logger.info("[PyDualSenseDriver] Connected to DualSense via pydualsense.")
        except Exception as e:
            self.ds = None
            self._connected = False

    def is_available(self) -> bool:
        return self._connected and self.ds is not None

    def set_rgb(self, r: int, g: int, b: int) -> bool:
        if not self.is_available():
            return False
        try:
            self.ds.light.setColorI(r, g, b)
            return True
        except Exception as e:
            self.logger.warn(f"[PyDualSenseDriver] Failed to set RGB: {e}", throttle_duration_sec=5.0)
            return False

    def set_brightness(self, brightness: int) -> bool:
        if not self.is_available():
            return False
        try:
            from pydualsense.enums import Brightness
            if brightness > 170:
                b_enum = Brightness.high
            elif brightness > 85:
                b_enum = Brightness.medium
            else:
                b_enum = Brightness.low
            self.ds.light.setBrightness(b_enum)
            return True
        except Exception:
            return False

    def set_player_leds(self, bitmask: int) -> bool:
        if not self.is_available():
            return False
        try:
            from pydualsense.enums import PlayerID
            # Map bitmask to player ID
            if bitmask & 0b00100 or bitmask == 1:
                self.ds.light.setPlayerID(PlayerID.PLAYER_1)
            elif bitmask & 0b01010 or bitmask == 2:
                self.ds.light.setPlayerID(PlayerID.PLAYER_2)
            elif bitmask & 0b10101 or bitmask == 3:
                self.ds.light.setPlayerID(PlayerID.PLAYER_3)
            elif bitmask & 0b11011 or bitmask == 4:
                self.ds.light.setPlayerID(PlayerID.PLAYER_4)
            else:
                self.ds.light.setPlayerID(PlayerID.PLAYER_1)
            return True
        except Exception:
            return False

    def close(self):
        if self.ds:
            try:
                self.ds.close()
            except Exception:
                pass


class MockDualSenseDriver(BaseDualSenseDriver):
    """Mock/Virtual driver for simulation or when hardware permissions are pending."""

    def __init__(self, logger):
        self.logger = logger
        self.r = 0
        self.g = 0
        self.b = 0
        self.brightness = 255
        self.player_bitmask = 0
        self._generation = 0
        self.fail_rgb = False

    def is_available(self) -> bool:
        return True

    def get_generation(self) -> int:
        return self._generation

    def set_rgb(self, r: int, g: int, b: int) -> bool:
        if self.fail_rgb:
            return False
        self.r, self.g, self.b = r, g, b
        return True

    def set_brightness(self, brightness: int) -> bool:
        self.brightness = brightness
        return True

    def set_player_leds(self, bitmask: int) -> bool:
        self.player_bitmask = bitmask
        return True


# =============================================================================
# Lighting Animation Engine
# =============================================================================

class LightingAnimationEngine:
    """
    Computes real-time dynamic light frames (solid, pulse, blink, rainbow, accents)
    and drives the hardware at a deterministic update rate (25 Hz).
    """

    EFFECT_SOLID = "solid"
    EFFECT_PULSE = "pulse"
    EFFECT_BLINK = "blink"
    EFFECT_RAINBOW = "rainbow"
    EFFECT_OFF = "off"

    def __init__(self, driver: BaseDualSenseDriver, update_rate: float = 25.0):
        self.driver = driver
        self.dt = 1.0 / update_rate
        self.time_sec = 0.0

        # Current target attributes
        self.target_r = 0
        self.target_g = 180
        self.target_b = 255
        self.target_brightness = 1.0       # 0.0 to 1.0
        self.target_effect = self.EFFECT_SOLID
        self.target_effect_rate = 1.0      # Hz
        self.target_player_leds = 0b00100  # Center dot (Player 1)

        # Gripper contact flash state
        self.accent_color: Optional[Tuple[int, int, int]] = None
        self.accent_expiry = 0.0

        # Last written hardware states (avoids redundant I/O writes)
        self._last_r = -1
        self._last_g = -1
        self._last_b = -1
        self._last_brightness = -1
        self._last_player_leds = -1
        self._last_driver_generation = -1

    def set_color(self, r: int, g: int, b: int):
        self.target_r = max(0, min(255, int(r)))
        self.target_g = max(0, min(255, int(g)))
        self.target_b = max(0, min(255, int(b)))

    def set_brightness(self, brightness: float):
        self.target_brightness = max(0.0, min(1.0, float(brightness)))

    def set_effect(self, effect: str, rate: float = 1.0):
        eff = effect.lower().strip()
        if eff in (self.EFFECT_SOLID, self.EFFECT_PULSE, self.EFFECT_BLINK, self.EFFECT_RAINBOW, self.EFFECT_OFF):
            self.target_effect = eff
        self.target_effect_rate = max(0.1, float(rate))

    def set_player_leds(self, bitmask: int):
        self.target_player_leds = int(bitmask) & 0x1F

    def trigger_accent(self, r: int = 255, g: int = 255, b: int = 255, duration_sec: float = 0.25):
        """Triggers a transient color accent (e.g. white flash when gripper makes contact)."""
        self.accent_color = (r, g, b)
        self.accent_expiry = time.time() + duration_sec

    def tick(self):
        """Advances animation time and updates hardware output."""
        self.time_sec += self.dt
        now = time.time()

        # Check for driver connection changes / hotplug re-connection
        gen = self.driver.get_generation()
        if gen != self._last_driver_generation:
            self._last_driver_generation = gen
            self._last_r = -1
            self._last_g = -1
            self._last_b = -1
            self._last_brightness = -1
            self._last_player_leds = -1

        # 1. Base color calculation
        if self.target_effect == self.EFFECT_OFF:
            r, g, b = 0, 0, 0
            b_scale = 0.0
        elif self.target_effect == self.EFFECT_RAINBOW:
            # Hue cycle (0 to 360 degrees)
            hue = (self.time_sec * self.target_effect_rate * 360.0) % 360.0
            r, g, b = self._hsv_to_rgb(hue, 1.0, 1.0)
            b_scale = 1.0
        else:
            r, g, b = self.target_r, self.target_g, self.target_b
            if self.target_effect == self.EFFECT_PULSE:
                # Smooth sine breathing: oscillates between 0.15 and 1.0
                phase = math.sin(2.0 * math.pi * self.target_effect_rate * self.time_sec)
                b_scale = 0.15 + 0.85 * (0.5 * (phase + 1.0))
            elif self.target_effect == self.EFFECT_BLINK:
                # Square wave blink
                phase = math.sin(2.0 * math.pi * self.target_effect_rate * self.time_sec)
                b_scale = 1.0 if phase > 0 else 0.0
            else:  # SOLID
                b_scale = 1.0

        # Apply global brightness
        eff_brightness = int(round(self.target_brightness * b_scale * 255))
        eff_brightness = max(0, min(255, eff_brightness))

        # Check for transient accent flash
        if self.accent_color and now < self.accent_expiry:
            # Blend accent color
            r, g, b = self.accent_color
            eff_brightness = max(eff_brightness, 220)

        # 2. Player LEDs calculation
        player_mask = self.target_player_leds
        if self.target_effect == self.EFFECT_BLINK:
            # Player LEDs blink in sync with effect
            if b_scale == 0.0:
                player_mask = 0
        elif self.target_effect == self.EFFECT_PULSE and self.target_effect_rate <= 1.5:
            # Cycle inward from outside: Outer pair (1 & 5) -> Middle pair (2 & 4) -> Center dot (3)
            inward_sequence = [0b10001, 0b01010, 0b00100, 0b00000]
            step = int((self.time_sec * 2.5) % len(inward_sequence))
            player_mask = inward_sequence[step]

        # 3. Commit to hardware only when changed or dirty
        if (r, g, b) != (self._last_r, self._last_g, self._last_b):
            if self.driver.set_rgb(r, g, b):
                self._last_r, self._last_g, self._last_b = r, g, b
            else:
                self._last_r, self._last_g, self._last_b = -1, -1, -1

        if eff_brightness != self._last_brightness:
            if self.driver.set_brightness(eff_brightness):
                self._last_brightness = eff_brightness
            else:
                self._last_brightness = -1

        if player_mask != self._last_player_leds:
            if self.driver.set_player_leds(player_mask):
                self._last_player_leds = player_mask
            else:
                self._last_player_leds = -1

    @staticmethod
    def _hsv_to_rgb(h: float, s: float, v: float) -> Tuple[int, int, int]:
        c = v * s
        x = c * (1.0 - abs((h / 60.0) % 2 - 1.0))
        m = v - c
        if h < 60:
            rp, gp, bp = c, x, 0
        elif h < 120:
            rp, gp, bp = x, c, 0
        elif h < 180:
            rp, gp, bp = 0, c, x
        elif h < 240:
            rp, gp, bp = 0, x, c
        elif h < 300:
            rp, gp, bp = x, 0, c
        else:
            rp, gp, bp = c, 0, x
        return int(round((rp + m) * 255)), int(round((gp + m) * 255)), int(round((bp + m) * 255))


# =============================================================================
# Main ROS 2 PS5 Lights Node
# =============================================================================

class PS5LightsNode(Node):
    """
    ROS 2 node for controlling PS5 controller lights.
    Supports auto teleoperation mode tracking and manual ROS 2 override topics.
    """

    # Lighting Presets for System States: (R, G, B, Effect, Effect Rate, Player LED Bitmask)
    PRESET_FK_MODE      = (0, 180, 255, LightingAnimationEngine.EFFECT_SOLID, 1.0, 0b00100)  # Sky Blue / Cyan (Player 3 center dot)
    PRESET_IK_MODE      = (0, 255, 60,  LightingAnimationEngine.EFFECT_SOLID, 1.0, 0b01010)  # Emerald Green (Player 2 & 4 pair)
    PRESET_ESTOP        = (255, 0, 0,   LightingAnimationEngine.EFFECT_BLINK, 1.0, 0b11111)  # Urgent Bright Red (1 Hz Flash, All LEDs)
    PRESET_RTH          = (255, 215, 0, LightingAnimationEngine.EFFECT_PULSE, 1.0, 0b11111)  # Golden Yellow (1 Hz Pulse, Inward Cycling)
    PRESET_CRAWL        = (180, 0, 255, LightingAnimationEngine.EFFECT_SOLID, 1.0, 0b00101)  # Violet / Purple
    PRESET_SPEED_TUNING = (255, 153, 0, LightingAnimationEngine.EFFECT_SOLID, 1.0, 0b11111)  # Solid Amber (All 5 LEDs)

    def __init__(self):
        super().__init__('ps5_lights_node')

        # Declare Parameters
        self.declare_parameter('auto_mode', True)             # Track robot teleoperation state automatically
        self.declare_parameter('driver_backend', 'auto')      # 'auto', 'sysfs', 'pydualsense', 'mock'
        self.declare_parameter('update_rate', 25.0)           # Animation loop rate (Hz)
        self.declare_parameter('default_brightness', 1.0)     # Default light brightness (0.0 to 1.0)

        self.auto_mode = self.get_parameter('auto_mode').value
        self.backend_pref = self.get_parameter('driver_backend').value
        update_rate = self.get_parameter('update_rate').value
        self.brightness = self.get_parameter('default_brightness').value

        # Initialize Hardware Driver
        self.driver = self._select_driver()
        self.anim = LightingAnimationEngine(self.driver, update_rate)
        self.anim.set_brightness(self.brightness)

        # Internal Teleoperation State Tracker
        self.arm_mode = 0             # 0: FK, 1: IK
        self.e_stop_active = False
        self.rth_active = False
        self.crawl_active = False
        self.orientation_layer = False
        self.speed_tuning_active = False
        self.signal_lost = False
        self.is_gripping = False
        self.last_joy_time = None
        self.manual_override_mode = None  # None: Auto; String: 'manual', 'fk', 'ik', etc.

        # ----- Subscriptions for Teleoperation & Robot Telemetry -----
        self.status_sub = self.create_subscription(
            String, 'arm_teleop_status', self.teleop_status_callback, 10
        )
        self.joy_sub = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10
        )
        self.gripper_sub = self.create_subscription(
            Float32MultiArray, 'gripper_feedback', self.gripper_callback, 10
        )

        # ----- Subscriptions for Manual & CLI Light Manipulation -----
        self.set_color_sub = self.create_subscription(
            ColorRGBA, 'ps5_lights/set_color', self.set_color_callback, 10
        )
        self.set_effect_sub = self.create_subscription(
            String, 'ps5_lights/set_effect', self.set_effect_callback, 10
        )
        self.set_player_leds_sub = self.create_subscription(
            Int32, 'ps5_lights/set_player_leds', self.set_player_leds_callback, 10
        )
        self.set_brightness_sub = self.create_subscription(
            Float32, 'ps5_lights/set_brightness', self.set_brightness_callback, 10
        )
        self.set_mode_sub = self.create_subscription(
            String, 'ps5_lights/set_mode', self.set_mode_callback, 10
        )
        self.feedback_sub = self.create_subscription(
            JoyFeedback, '/joy/set_feedback', self.joy_feedback_callback, 10
        )

        # ----- Telemetry Publisher -----
        self.state_pub = self.create_publisher(
            String, 'ps5_lights/status', 10
        )

        # Button states for rising-edge detection
        self.prev_options_btn = 0
        self.prev_estop_btn = 0
        self.prev_rth_btn = 0

        # Subscriptions for arm command topics to double-confirm active control mode
        self.arm_cmd_sub = self.create_subscription(
            Float32MultiArray, 'arm_cmd', self.arm_cmd_callback, 10
        )
        self.arm_ik_cmd_sub = self.create_subscription(
            Float32MultiArray, 'arm_ik_cmd', self.arm_ik_cmd_callback, 10
        )

        # RTH action status subscription to automatically detect when RTH completes
        self.HOME_JOINTS = [0.0, 0.0, 2.0072, 0.0, 0.0]
        self.rth_status_sub = self.create_subscription(
            GoalStatusArray, '/return_to_home/_action/status', self.rth_action_status_callback, 10
        )

        # Timers
        timer_period = 1.0 / update_rate
        self.anim_timer = self.create_timer(timer_period, self.animation_tick)
        self.telemetry_timer = self.create_timer(1.0, self.telemetry_tick)

        # Initial Preset Application
        self._apply_preset(self.PRESET_FK_MODE)
        self.get_logger().info(
            f"PS5 Lights Node ready! Backend: {self.driver.__class__.__name__}, Auto Mode: {self.auto_mode}"
        )

    def _select_driver(self) -> BaseDualSenseDriver:
        """Selects the best available hardware driver based on configuration and permissions."""
        pref = self.backend_pref.lower()

        if pref == 'mock':
            self.get_logger().info("Using Mock driver for DualSense LEDs (requested by parameter).")
            return MockDualSenseDriver(self.get_logger())

        if pref in ('auto', 'sysfs'):
            sysfs_drv = SysfsDualSenseDriver(self.get_logger())
            if sysfs_drv.is_available() or pref == 'sysfs':
                self.get_logger().info("Using Linux Sysfs driver for DualSense LEDs (hotplug enabled).")
                return sysfs_drv
            # Check if sysfs leds exist on the system but permissions are missing
            if glob.glob("/sys/class/leds/*:rgb:indicator"):
                self.get_logger().warn(
                    "PS5 DualSense detected, but user permissions to write to LEDs are not granted.\n"
                    "To enable physical controller lighting, run the one-time udev setup script:\n"
                    "  bash /home/kratos/workspace/src/mapper/scripts/setup_dualsense_udev.sh\n"
                    "Falling back to Mock driver for safe operation without crashes."
                )
                return MockDualSenseDriver(self.get_logger())
            # Device not plugged in at startup: SysfsDualSenseDriver stays armed for hotplug
            self.get_logger().info("DualSense not currently plugged in; Linux Sysfs driver armed for hotplug.")
            return sysfs_drv

        if pref in ('auto', 'pydualsense'):
            try:
                py_drv = PyDualSenseDriver(self.get_logger())
                if py_drv.is_available():
                    self.get_logger().info("Using pydualsense driver for DualSense LEDs.")
                    return py_drv
            except Exception:
                pass

        return MockDualSenseDriver(self.get_logger())

    # =========================================================================
    # Mode Arbitration & Teleoperation Callbacks
    # =========================================================================

    def teleop_status_callback(self, msg: String):
        """Receives structured status from ps5_mapper."""
        try:
            data = json.loads(msg.data)
            self.arm_mode = data.get('mode', self.arm_mode)
            self.e_stop_active = data.get('estop', self.e_stop_active)
            self.rth_active = data.get('rth', self.rth_active)
            self.crawl_active = data.get('crawl', self.crawl_active)
            self.orientation_layer = data.get('orientation_layer', self.orientation_layer)
            self.speed_tuning_active = data.get('tuning_speed', self.speed_tuning_active)
            self.signal_lost = data.get('signal_lost', self.signal_lost)
            if self.auto_mode and self.manual_override_mode is None:
                self._update_lighting_from_state()
        except Exception as e:
            self.get_logger().warn(f"Failed to parse /arm_teleop_status: {e}", throttle_duration_sec=5.0)

    def rth_action_status_callback(self, msg: GoalStatusArray):
        """Monitors ReturnToHome action server status to immediately detect when RTH completes."""
        has_executing = any(g.status == GoalStatus.STATUS_EXECUTING for g in msg.status_list)
        if self.rth_active and not has_executing:
            # Action has completed (succeeded, canceled, or aborted)
            self.rth_active = False
            if self.auto_mode and self.manual_override_mode is None:
                self._update_lighting_from_state()
        elif not self.rth_active and has_executing:
            self.rth_active = True
            if self.auto_mode and self.manual_override_mode is None:
                self._update_lighting_from_state()

    def arm_cmd_callback(self, msg: Float32MultiArray):
        """Monitors arm joint command stream to detect arrival at home posture."""
        if self.rth_active and len(msg.data) >= 5:
            dist = math.hypot(*[msg.data[i] - self.HOME_JOINTS[i] for i in range(5)])
            if dist < 0.02:
                self.rth_active = False
                if self.auto_mode and self.manual_override_mode is None:
                    self._update_lighting_from_state()

    def arm_ik_cmd_callback(self, msg: Float32MultiArray):
        # IK command published means we are in IK Mode (Mode 1)
        if self.arm_mode != 1:
            self.arm_mode = 1
            if self.auto_mode and self.manual_override_mode is None:
                self._update_lighting_from_state()

    def joy_callback(self, msg: Joy):
        """Direct joystick monitoring for standalone operation."""
        self.last_joy_time = self.get_clock().now()
        if self.signal_lost:
            self.signal_lost = False
            if self.auto_mode and self.manual_override_mode is None:
                self._update_lighting_from_state()

        num_buttons = len(msg.buttons)
        state_changed = False

        # Any manual stick input clears RTH state once operator moves sticks
        if self.rth_active and len(msg.axes) >= 4:
            if any(abs(msg.axes[i]) > 0.15 for i in (0, 1, 3, 4)):
                self.rth_active = False
                state_changed = True

        if num_buttons > 10:
            # 1. Mode toggle (OPTIONS button, index 9)
            options_btn = msg.buttons[9]
            if options_btn == 1 and self.prev_options_btn == 0:
                self.arm_mode = 1 - self.arm_mode
                state_changed = True
            self.prev_options_btn = options_btn

            # 2. E-Stop latch (PS button, index 10)
            estop_btn = msg.buttons[10]
            if estop_btn == 1 and self.prev_estop_btn == 0:
                self.e_stop_active = not self.e_stop_active
                if self.e_stop_active and self.rth_active:
                    self.rth_active = False
                state_changed = True
            self.prev_estop_btn = estop_btn

            # 3. RTH toggle (SHARE button, index 8)
            share_btn = msg.buttons[8]
            if share_btn == 1 and self.prev_rth_btn == 0:
                self.rth_active = not self.rth_active
                state_changed = True
            self.prev_rth_btn = share_btn

            # 4. Precision Crawl Mode (LB button, index 4)
            lb_held = bool(msg.buttons[4])
            if lb_held != self.crawl_active:
                self.crawl_active = lb_held
                state_changed = True

            # 5. Orientation Layer (RB button, index 5)
            rb_held = bool(msg.buttons[5])
            if rb_held != self.orientation_layer:
                self.orientation_layer = rb_held
                state_changed = True

            # 6. Speed Tuning (Shape buttons: Cross=0, Circle=1, Triangle=2, Square=3)
            shape_held = bool(msg.buttons[0] or msg.buttons[1] or msg.buttons[2] or msg.buttons[3])
            if shape_held != self.speed_tuning_active:
                self.speed_tuning_active = shape_held
                state_changed = True

        if state_changed and self.auto_mode and self.manual_override_mode is None:
            self._update_lighting_from_state()

    def gripper_callback(self, msg: Float32MultiArray):
        """Detects load on the gripper to trigger a tactile soft cyan accent flash."""
        if len(msg.data) >= 2:
            gripping = bool(msg.data[1] > 0.5)
            if gripping and not self.is_gripping:
                # Rising edge of grip load: trigger crisp Soft Cyan (100, 230, 255) 300ms accent pulse
                self.anim.trigger_accent(100, 230, 255, duration_sec=0.3)
            self.is_gripping = gripping

    def _update_lighting_from_state(self):
        """
        Evaluates system state hierarchy and assigns the appropriate lighting preset:
        E-Stop > RTH > Speed Tuning > Crawl & Orientation > FK / IK Mode
        """
        if self.e_stop_active:
            self._apply_preset(self.PRESET_ESTOP)
        elif self.rth_active:
            self._apply_preset(self.PRESET_RTH)
        elif self.speed_tuning_active:
            self._apply_preset(self.PRESET_SPEED_TUNING)
        else:
            # Base mode colors & player LEDs
            if self.arm_mode == 1:
                # IK Mode: Emerald Green (0, 255, 60), Player 2 & 4 (0b01010)
                r, g, b = 0, 255, 60
                player_mask = 0b01010
            else:
                # FK Mode: Sky Blue / Cyan (0, 180, 255), Player 3 Center dot (0b00100)
                r, g, b = 0, 180, 255
                player_mask = 0b00100

            # Precision Crawl (LB held): shift lightbar to Violet/Purple and light leftmost LED 1 (0b00001)
            if self.crawl_active:
                r, g, b = 180, 0, 255
                player_mask |= 0b00001

            # Orientation Layer (RB held): light rightmost LED 5 (0b10000)
            if self.orientation_layer:
                player_mask |= 0b10000

            self.anim.set_color(r, g, b)
            self.anim.set_effect(LightingAnimationEngine.EFFECT_SOLID)
            self.anim.set_player_leds(player_mask)

    def _apply_preset(self, preset: Tuple[int, int, int, str, float, int]):
        r, g, b, eff, rate, player_mask = preset
        self.anim.set_color(r, g, b)
        self.anim.set_effect(eff, rate)
        self.anim.set_player_leds(player_mask)

    # =========================================================================
    # Manual ROS 2 Control Interfaces
    # =========================================================================

    def set_color_callback(self, msg: ColorRGBA):
        """Allows direct manual RGB color control from ROS topic."""
        self.manual_override_mode = "manual"
        r = int(round(msg.r * 255))
        g = int(round(msg.g * 255))
        b = int(round(msg.b * 255))
        self.anim.set_color(r, g, b)
        if msg.a > 0.0:
            self.anim.set_brightness(msg.a)
        self.get_logger().info(f"[Manual] Set RGB: ({r}, {g}, {b})")

    def set_effect_callback(self, msg: String):
        """Sets the lighting effect: 'solid', 'pulse', 'blink', 'rainbow', 'off'."""
        eff = msg.data.lower().strip()
        self.anim.set_effect(eff)
        self.get_logger().info(f"[Manual] Set effect: {eff}")

    def set_player_leds_callback(self, msg: Int32):
        """Sets player LEDs. Accepts 1-5 (player number) or bitmask (0b00000 to 0b11111)."""
        val = msg.data
        if 1 <= val <= 5:
            # Player 1..5 standard mapping
            masks = {
                1: 0b00100,  # Center dot
                2: 0b01010,  # 2 dots
                3: 0b10101,  # 3 dots
                4: 0b11011,  # 4 dots
                5: 0b11111,  # 5 dots
            }
            mask = masks[val]
        else:
            mask = val & 0x1F
        self.anim.set_player_leds(mask)
        self.get_logger().info(f"[Manual] Set player LEDs mask: {bin(mask)}")

    def set_brightness_callback(self, msg: Float32):
        """Sets brightness scale (0.0 to 1.0)."""
        self.anim.set_brightness(msg.data)
        self.get_logger().info(f"[Manual] Set brightness: {msg.data:.2f}")

    def set_mode_callback(self, msg: String):
        """
        Switches between auto tracking and manual mode:
          'auto'    -> Restores automatic tracking of arm teleoperation modes.
          'fk'      -> Forces FK Mode preset.
          'ik'      -> Forces IK Mode preset.
          'estop'   -> Forces E-Stop preset.
          'rth'     -> Forces RTH preset.
          'rainbow' -> Activates rainbow cycle.
          'off'     -> Turns lights off.
        """
        mode_str = msg.data.lower().strip()
        if mode_str == "auto":
            self.manual_override_mode = None
            self.auto_mode = True
            self._update_lighting_from_state()
            self.get_logger().info("Switched to AUTO lighting mode (tracking robot state).")
        elif mode_str == "fk":
            self.manual_override_mode = "fk"
            self._apply_preset(self.PRESET_FK_MODE)
        elif mode_str == "ik":
            self.manual_override_mode = "ik"
            self._apply_preset(self.PRESET_IK_MODE)
        elif mode_str == "estop":
            self.manual_override_mode = "estop"
            self._apply_preset(self.PRESET_ESTOP)
        elif mode_str == "rth":
            self.manual_override_mode = "rth"
            self._apply_preset(self.PRESET_RTH)
        elif mode_str == "rainbow":
            self.manual_override_mode = "rainbow"
            self.anim.set_effect(LightingAnimationEngine.EFFECT_RAINBOW, 0.5)
        elif mode_str == "off":
            self.manual_override_mode = "off"
            self.anim.set_effect(LightingAnimationEngine.EFFECT_OFF)
        else:
            self.manual_override_mode = "manual"
            self.get_logger().info(f"Custom mode set: {mode_str}")

    def joy_feedback_callback(self, msg: JoyFeedback):
        """Standard ROS 2 JoyFeedback handler for TYPE_LED."""
        if msg.type == JoyFeedback.TYPE_LED:
            # Map intensity to brightness
            self.anim.set_brightness(msg.intensity)

    # =========================================================================
    # Animation & Telemetry Loops
    # =========================================================================

    def animation_tick(self):
        """Deterministic 25 Hz animation cycle."""
        # Watchdog check for joystick timeout
        if self.last_joy_time:
            elapsed = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
            if elapsed > 0.4 and not self.signal_lost:
                self.signal_lost = True
                if self.auto_mode and self.manual_override_mode is None:
                    self._update_lighting_from_state()

        self.anim.tick()

    def telemetry_tick(self):
        """Publishes 1 Hz status telemetry for diagnostics."""
        telemetry = {
            "backend": self.driver.__class__.__name__,
            "hardware_connected": self.driver.is_available(),
            "auto_mode": self.auto_mode and self.manual_override_mode is None,
            "active_mode": self.manual_override_mode or ("IK" if self.arm_mode == 1 else "FK"),
            "rgb": [self.anim.target_r, self.anim.target_g, self.anim.target_b],
            "effect": self.anim.target_effect,
            "brightness": round(self.anim.target_brightness, 2),
            "player_leds": bin(self.anim.target_player_leds),
            "estop": self.e_stop_active,
            "rth": self.rth_active,
            "crawl": self.crawl_active,
            "signal_lost": self.signal_lost
        }
        msg = String()
        msg.data = json.dumps(telemetry)
        self.state_pub.publish(msg)

    def destroy_node(self):
        # Gracefully reset lights on exit
        try:
            self.anim.set_effect(LightingAnimationEngine.EFFECT_OFF)
            self.anim.tick()
            self.driver.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PS5LightsNode()
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
