#!/usr/bin/env python3
"""
delivery_controller.py -- Piston motor + Pixy2 delivery subsystem (TurtleBot 4)
==============================================================================

ENGR 3100 group project.  This program ONLY does:
    * VEX 393 motor control through PCA9685 -> VEX Motor Controller 29 (timed PWM)
    * Pixy2 (side-mounted) block detection, color confirmation, ejection monitoring
    * Delivery decisions (which station each block needs)
    * ROS 2 messages to/from the teammate's navigation program (request + confirmation)

It never drives the robot and never publishes to /cmd_vel.

Hardware this code is written for (and NOTHING else):
    TurtleBot 4 Raspberry Pi, PCA9685 board, VEX MC29, VEX 393 2-wire motor,
    side-facing Pixy2, external 6 V pack (unverified), slider-crank, channel, blocks.
There is NO encoder, limit switch, position sensor or break-beam sensor.  The
code therefore never knows the real crank angle or piston position.  Timed
motor runs are approximations only, and the Pixy2 is the only feedback.

------------------------------------------------------------------------------
RUN MODES
------------------------------------------------------------------------------
  python3 delivery_controller.py --offline-test
        No ROS, no hardware.  Runs every simulated scenario with an internal
        simulated navigation harness.  Works on Windows, WSL, or the Pi.
  python3 delivery_controller.py --offline-run red,red,blue
        One simulated mission with the full log printed.
  python3 delivery_controller.py --check-config
        Lists every hardware PLACEHOLDER / safety item still missing.
  python3 delivery_controller.py --mode hardware --blocks 3
        Real motor + Pixy2 on the TurtleBot 4 Pi (ROS 2).  Refuses to start until
        every PLACEHOLDER and safety verification is complete.
  python3 delivery_controller.py --mode simulation --blocks 3 --sim-order red,green,blue
        ROS 2 topics with SIMULATED motor/camera (navigation must be provided
        by the teammate's program or by hand with `ros2 topic pub`).

------------------------------------------------------------------------------
LABELS USED IN COMMENTS
------------------------------------------------------------------------------
  [VERIFIED API]  taken from official library documentation / official examples
  [VERIFIED SPEC] taken from manufacturer documentation (see README "Sources")
  [PROPOSED]      interface designed for this project (the /delivery/* topics)
  PLACEHOLDER     unknown value -- must be measured / verified.  None = not set.
  PLACEHOLDER-TUNE  starting value that is NOT safety-critical; tune in testing
"""

from __future__ import annotations

import argparse
import atexit
import json
import math
import os
import signal
import sys
import threading
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Dict, List, Optional, Tuple

# =============================================================================
# SECTION 1 -- CONFIGURATION  (edit these during lab testing)
# =============================================================================
# None = PLACEHOLDER that MUST be filled in before hardware mode will run.
# Pixy2 pixel coordinates: x 0..315, y 0..207 (Pixy2 CCC API documentation).
# =============================================================================

HARDWARE_CONFIG: Dict[str, object] = {

    # ======================= MOTOR (PCA9685 -> MC29 -> VEX 393) =============
    "motor_type": "VEX_393",
    "motor_controller": "VEX_MC29",

    # PCA9685 connection.  Find with `ls /dev/i2c*` and `sudo i2cdetect -y <bus>`.
    "i2c_bus": None,                    # PLACEHOLDER  N in /dev/i2c-N
    "pca9685_address": None,            # PLACEHOLDER  e.g. 0x40 ONLY if i2cdetect shows it
    "pca9685_channel": None,            # PLACEHOLDER  0-15, the header the MC29 is plugged into
    # MC29 is documented by pulse WIDTH (1-2 ms).  The repetition rate is not in
    # the VEX product listing -> measure/verify before use.
    "pwm_frequency_hz": None,           # PLACEHOLDER
    # The PCA9685's internal oscillator is nominally 25 MHz but varies per chip
    # [VERIFIED API: adafruit_pca9685 reference_clock_speed].  If a scope shows
    # 1500 us commanded != 1500 us measured, correct this value.
    "pca9685_reference_clock_hz": 25_000_000,   # PLACEHOLDER-TUNE

    # MC29 [VERIFIED SPEC]: 1 ms = full reverse, 2 ms = full forward, 1.5 ms = neutral.
    # Your actual neutral must still be confirmed on the bench (clock error,
    # controller tolerance).  Neutral = stop COMMANDING rotation.  It does not
    # hold position and does not prove the motor has physically stopped.
    "motor_neutral_pulse_us": None,     # PLACEHOLDER  bench-verified neutral
    # "How strongly the motor drives": move these closer to / farther from neutral.
    "motor_forward_pulse_us": None,     # PLACEHOLDER  e.g. somewhere between neutral and 2000
    "motor_reverse_pulse_us": None,     # PLACEHOLDER  e.g. somewhere between 1000 and neutral
    # Motor rotation direction: which command pushes the piston TOWARD the edge.
    # The controller only ever uses this one; it never reverses on its own.
    "motor_advance_command": None,      # PLACEHOLDER  "forward" or "reverse"
    # Hard pulse limits from the MC29 documentation; never exceeded.
    "mc29_min_pulse_us": 1000,          # [VERIFIED SPEC]
    "mc29_max_pulse_us": 2000,          # [VERIFIED SPEC]

    # ----- Timed increments (approximate movement, NOT a position) -----
    "search_run_time_s": None,          # PLACEHOLDER  motor-on time per searching increment
    "unload_run_time_s": None,          # PLACEHOLDER  motor-on time per unloading increment (smaller)
    # Slider-crank nonlinearity: the same motor-on time moves the piston a
    # different distance at different stages of the stroke.  Optional stage
    # table, selected by the ESTIMATED stage (cumulative advance run time since
    # the operator-verified start).  Measure each stage with calipers using
    # hardware_calibration.py.  None = always use the two values above.
    # Format: [{"until_cumulative_s": 0.30, "search_run_time_s": 0.08, "unload_run_time_s": 0.04}, ...]
    "run_time_stages": None,            # PLACEHOLDER (optional, but recommended)
    "min_run_time_s": None,             # PLACEHOLDER  shortest run that still reliably moves the crank
    "motor_settle_time_s": None,        # PLACEHOLDER  wait after neutral before reading the camera
    "max_single_motor_run_time_s": None,     # PLACEHOLDER  hard cap for any one run
    # Conservative cumulative advance budget from the verified start position.
    # Set BELOW the run time that reached full extension in your tests (margin!).
    # Cumulative run time is NOT a reliable crank position; it is only a limit.
    "max_cumulative_motor_run_time_s": None, # PLACEHOLDER
    "max_search_increments": None,      # PLACEHOLDER  per search (one block)
    "max_unload_increments": None,      # PLACEHOLDER  per block being unloaded
    "max_total_increments": None,       # PLACEHOLDER  whole mission

    # ----- Camera feedback on motor increments -----
    # If a visible block moves less than this after an increment, the increment
    # "made no progress" (jam, stall, dead battery, or crank past full extension).
    "min_progress_px_per_increment": None,   # PLACEHOLDER
    "no_progress_increment_limit": 3,        # PLACEHOLDER-TUNE
    # If one increment moves a visible block more than this, later increments of
    # that kind are shortened (handles the fast middle of the slider-crank stroke).
    # Must be smaller than the distance the next block can move before tipping.
    "max_advance_px_per_increment": None,    # PLACEHOLDER
    "increment_shrink_factor": 0.6,          # PLACEHOLDER-TUNE

    # ======================= PIXY2 (side-facing) =============================
    "color_signatures": {"red": None, "green": None, "blue": None},  # PLACEHOLDER (PixyMon 1-7)
    "pixy_program": "color_connected_components",   # [VERIFIED API] official demo program name
    "pixy_max_blocks": 20,
    "camera_read_period_s": 0.05,       # PLACEHOLDER-TUNE  20 Hz reads
    "camera_comm_timeout_s": 1.0,       # PLACEHOLDER-TUNE  no successful read for this long -> fault
    "pixy_health_check_period_s": 2.0,
    "max_consecutive_camera_errors": 5, # PLACEHOLDER-TUNE
    # Region of interest (x_min, x_max, y_min, y_max): the channel only.
    "roi_px": None,                     # PLACEHOLDER
    # Valid block = at least this big (side face of a block).
    "min_block_width_px": None,         # PLACEHOLDER
    "min_block_height_px": None,        # PLACEHOLDER
    "min_block_area_px": None,          # PLACEHOLDER
    # Smaller detections (block only partly in view) = "block entering view".
    "entering_min_area_px": None,       # PLACEHOLDER
    # Image axis along which blocks travel ("x" or "y") and its sign
    # (+1: coordinate increases toward the unloading edge, -1: decreases).
    "leading_axis": None,               # PLACEHOLDER  determine in PixyMon
    "leading_direction": None,          # PLACEHOLDER  determine in PixyMon
    "edge_px": None,                    # PLACEHOLDER  pixel coordinate of the unloading edge
    "exit_zone_margin_px": None,        # PLACEHOLDER  "at the edge" tolerance
    "handover_jump_px": None,           # PLACEHOLDER  leading-edge backward jump = next block leads
    "color_confirmation_frames": 5,     # PLACEHOLDER-TUNE  consistent readings to confirm color
    "empty_confirmation_frames": 6,     # PLACEHOLDER-TUNE  empty readings before advancing
    "color_max_attempt_frames": 40,     # PLACEHOLDER-TUNE  give up -> uncertain identification
    "lost_block_tolerance_frames": 8,   # PLACEHOLDER-TUNE
    "ejection_verify_frames": 6,        # PLACEHOLDER-TUNE  readings to verify an ejection
    # Side view: a falling block moves DOWN in the image before it disappears.
    # Only enable after you have seen this in PixyMon / hardware_calibration.py.
    "fall_detection_enabled": False,
    "fall_axis": None,                  # PLACEHOLDER  "x" or "y" (perpendicular to leading_axis)
    "fall_direction": None,             # PLACEHOLDER  +1 or -1 (sign of "downward" in the image)
    "fall_min_px": None,                # PLACEHOLDER
    "ejection_requires_fall_evidence": False,   # True = no fall seen -> uncertain delivery

    # ======================= MISSION =========================================
    "loaded_block_count": None,         # PLACEHOLDER  set with --blocks (1-3)
    "max_supported_blocks": 3,
    "station_ids": {"red": "station_red", "green": "station_green",
                    "blue": "station_blue"},           # PLACEHOLDER  agree with teammate
    "mission_timeout_s": None,          # optional overall limit, e.g. 900

    # ======================= ROS 2 / NAVIGATION INTERFACE [PROPOSED] =========
    "topic_destination": "/delivery/destination",
    "topic_nav_status": "/delivery/nav_status",
    "topic_stop": "/delivery/stop",
    "topic_status": "/delivery/status",
    "topic_start": "/delivery/start",
    "topic_abort": "/delivery/abort",
    "topic_operator_ejection": "/delivery/operator_ejection_confirm",
    "control_period_s": 0.01,           # motor deadlines are checked at this rate
    "status_publish_period_s": 0.5,
    "destination_republish_period_s": 0.5,
    "nav_ack_timeout_s": 3.0,           # PLACEHOLDER-TUNE
    "nav_timeout_s": None,              # PLACEHOLDER  longest trip x 2
    "nav_heartbeat_timeout_s": 1.0,     # PLACEHOLDER-TUNE  nav_status must arrive faster than this
    "arrival_settle_time_s": 1.0,       # PLACEHOLDER-TUNE  stationary dwell before unloading
    "stationary_loss_grace_s": 2.0,     # PLACEHOLDER-TUNE  motor idle: wait this long, then fault
    # Independent stationary check from the TurtleBot 4 (Create 3) odometry.
    "use_odom_stationary_check": True,
    "odom_topic": "/odom",              # PLACEHOLDER  verify: `ros2 topic list` (namespace?)
    "odom_max_linear_mps": 0.01,        # PLACEHOLDER-TUNE
    "odom_max_angular_radps": 0.02,     # PLACEHOLDER-TUNE
    "odom_timeout_s": 1.0,
}

# Operator verifications.  Hardware mode stays DISABLED until every item is True.
# Set each one only after you have personally checked it (README Part 11).
SAFETY_VERIFICATIONS: Dict[str, bool] = {
    "battery_voltage_and_ratings_verified": False,      # 6 V pack vs MC29 (8.5 V max) and PCA9685 board V+ rating
    "wiring_and_common_ground_verified": False,
    "mc29_accepts_pca9685_signal_level_verified": False,# PCA9685 output level = its VCC (often 3.3 V)
    "neutral_verified_motor_does_not_turn": False,
    "advance_direction_verified": False,
    "movement_limits_measured_with_linkage": False,     # cumulative budget + increments measured
    "manual_power_disconnect_practiced": False,         # someone can unplug the 6 V pack instantly
}

CRITICAL_HARDWARE_KEYS = [
    "i2c_bus", "pca9685_address", "pca9685_channel", "pwm_frequency_hz",
    "motor_neutral_pulse_us", "motor_forward_pulse_us", "motor_reverse_pulse_us",
    "motor_advance_command",
    "search_run_time_s", "unload_run_time_s", "min_run_time_s", "motor_settle_time_s",
    "max_single_motor_run_time_s", "max_cumulative_motor_run_time_s",
    "max_search_increments", "max_unload_increments", "max_total_increments",
    "min_progress_px_per_increment", "max_advance_px_per_increment",
    "roi_px", "min_block_width_px", "min_block_height_px", "min_block_area_px",
    "entering_min_area_px", "leading_axis", "leading_direction", "edge_px",
    "exit_zone_margin_px", "handover_jump_px",
    "loaded_block_count", "nav_timeout_s",
]

# -----------------------------------------------------------------------------
# SIMULATION-ONLY values (invented for the mock mechanism).  Never used in
# hardware mode.  Do NOT copy them into HARDWARE_CONFIG.
# -----------------------------------------------------------------------------
SIMULATION_OVERRIDES: Dict[str, object] = {
    "i2c_bus": "SIM", "pca9685_address": "SIM", "pca9685_channel": "SIM",
    "pwm_frequency_hz": 50,
    "motor_neutral_pulse_us": 1500, "motor_forward_pulse_us": 1650,
    "motor_reverse_pulse_us": 1350, "motor_advance_command": "forward",
    "search_run_time_s": 0.05, "unload_run_time_s": 0.025,
    "run_time_stages": [
        {"until_cumulative_s": 0.15, "search_run_time_s": 0.06, "unload_run_time_s": 0.03},
        {"until_cumulative_s": 9.99, "search_run_time_s": 0.05, "unload_run_time_s": 0.025},
    ],
    "min_run_time_s": 0.01, "motor_settle_time_s": 0.15,
    "max_single_motor_run_time_s": 0.12, "max_cumulative_motor_run_time_s": 0.60,
    "max_search_increments": 25, "max_unload_increments": 25, "max_total_increments": 120,
    "min_progress_px_per_increment": 4, "max_advance_px_per_increment": 45,
    "color_signatures": {"red": 1, "green": 2, "blue": 3},
    "roi_px": (0, 315, 40, 200),
    "min_block_width_px": 60, "min_block_height_px": 30, "min_block_area_px": 2500,
    "entering_min_area_px": 200,
    "leading_axis": "x", "leading_direction": 1, "edge_px": 260,
    "exit_zone_margin_px": 45, "handover_jump_px": 50,
    "fall_axis": "y", "fall_direction": 1, "fall_min_px": 15,
    "loaded_block_count": 3, "nav_timeout_s": 30.0,
    "use_odom_stationary_check": False,
}

# Simulated mechanism (SIMULATION ONLY) -- mm along the channel.
SIM_CRANK_MM, SIM_LINK_MM = 20.0, 60.0
SIM_FREE_SPEED_DEG_S = 600.0        # 100 rpm at full command (VEX 393 spec value)
SIM_EDGE_S_MM = 28.0                # unloading edge
SIM_CAMERA_MIN_S_MM = 10.0          # camera sees the channel from here to past the edge
SIM_BLOCK_MM = 8.0
SIM_PX_PER_MM = 260.0 / (SIM_EDGE_S_MM - SIM_CAMERA_MIN_S_MM)   # edge at x=260 px


def slider_crank_displacement_mm(theta_deg: float, crank_mm: float, link_mm: float) -> float:
    """Ideal slider-crank piston displacement from full retraction.
    theta = 0 deg: fully retracted; 180 deg: fully extended; beyond 180 deg the
    piston RETRACTS again (over-rotation).  Used ONLY by the simulator and to
    explain the nonlinearity -- the real code never knows theta."""
    phi = math.radians(180.0 - theta_deg)
    def x(p: float) -> float:
        return crank_mm * math.cos(p) + math.sqrt(link_mm ** 2 - (crank_mm * math.sin(p)) ** 2)
    return x(phi) - x(math.pi)


# =============================================================================
# SECTION 2 -- UTILITIES AND CONFIG VALIDATION
# =============================================================================

class Logger:
    def __init__(self, prefix: str = "", sink: Optional[Callable[[str, str], None]] = None,
                 quiet: bool = False):
        self.prefix, self.sink, self.quiet = prefix, sink, quiet

    def _emit(self, level: str, msg: str) -> None:
        text = f"{self.prefix}{msg}"
        if self.sink:
            self.sink(level, text)
        elif not self.quiet or level == "ERROR":
            print(f"[{level}] {text}", flush=True)

    def info(self, m: str) -> None: self._emit("INFO", m)
    def warn(self, m: str) -> None: self._emit("WARN", m)
    def error(self, m: str) -> None: self._emit("ERROR", m)
    def debug(self, m: str) -> None: self._emit("DEBUG", m)


class ConfigurationError(Exception):
    pass


class HardwareError(Exception):
    pass


def load_config(mode: str, loaded_blocks: Optional[int]) -> Dict[str, object]:
    cfg = dict(HARDWARE_CONFIG)
    if mode == "simulation":
        cfg.update(SIMULATION_OVERRIDES)
    if loaded_blocks is not None:
        cfg["loaded_block_count"] = loaded_blocks
    return cfg


def advance_pulse_us(cfg: Dict[str, object]) -> float:
    key = "motor_forward_pulse_us" if cfg["motor_advance_command"] == "forward" else "motor_reverse_pulse_us"
    return float(cfg[key])


def validate_config(cfg: Dict[str, object], mode: str) -> List[str]:
    """Return problems.  Hardware mode runs only when this list is empty."""
    p: List[str] = [f"PLACEHOLDER not set: {k}" for k in CRITICAL_HARDWARE_KEYS if cfg.get(k) is None]
    for color, sig in cfg["color_signatures"].items():
        if sig is None:
            p.append(f"PLACEHOLDER not set: color_signatures['{color}']")
        elif not (isinstance(sig, int) and 1 <= sig <= 7):
            p.append(f"color_signatures['{color}'] must be an integer 1-7")
    sigs = [s for s in cfg["color_signatures"].values() if s is not None]
    if len(sigs) != len(set(sigs)):
        p.append("two colors share one Pixy2 signature")
    if mode == "hardware":
        p += [f"SAFETY VERIFICATION not done: {k}" for k, v in SAFETY_VERIFICATIONS.items() if v is not True]
    if any(s.startswith("PLACEHOLDER") or s.startswith("color_sig") for s in p):
        return p

    lo, hi = float(cfg["mc29_min_pulse_us"]), float(cfg["mc29_max_pulse_us"])
    neu, fwd, rev = (float(cfg[k]) for k in ("motor_neutral_pulse_us", "motor_forward_pulse_us",
                                               "motor_reverse_pulse_us"))
    if not (lo < neu < hi):
        p.append("motor_neutral_pulse_us must be inside the MC29 1000-2000 us range")
    if not (neu < fwd <= hi):
        p.append("motor_forward_pulse_us must be above neutral and <= mc29_max_pulse_us")
    if not (lo <= rev < neu):
        p.append("motor_reverse_pulse_us must be below neutral and >= mc29_min_pulse_us")
    if cfg["motor_advance_command"] not in ("forward", "reverse"):
        p.append("motor_advance_command must be 'forward' or 'reverse'")
    single = float(cfg["max_single_motor_run_time_s"])
    run_values = [cfg["search_run_time_s"], cfg["unload_run_time_s"]]
    for stage in cfg.get("run_time_stages") or []:
        try:
            run_values += [stage["search_run_time_s"], stage["unload_run_time_s"]]
            float(stage["until_cumulative_s"])
        except (KeyError, TypeError, ValueError):
            p.append("run_time_stages entries need until_cumulative_s, search_run_time_s, unload_run_time_s")
    for value in run_values:
        if not (float(cfg["min_run_time_s"]) <= float(value) <= single):
            p.append(f"run time {value} s must be between min_run_time_s and max_single_motor_run_time_s")
    if float(cfg["max_cumulative_motor_run_time_s"]) <= 0:
        p.append("max_cumulative_motor_run_time_s must be positive")
    if cfg["leading_axis"] not in ("x", "y") or cfg["leading_direction"] not in (1, -1):
        p.append("leading_axis must be 'x'/'y' and leading_direction +1/-1")
    if cfg["fall_detection_enabled"]:
        if cfg["fall_axis"] not in ("x", "y") or cfg["fall_direction"] not in (1, -1) or cfg["fall_min_px"] is None:
            p.append("fall detection enabled but fall_axis / fall_direction / fall_min_px not set")
        elif cfg["fall_axis"] == cfg["leading_axis"]:
            p.append("fall_axis must differ from leading_axis")
    if cfg["ejection_requires_fall_evidence"] and not cfg["fall_detection_enabled"]:
        p.append("ejection_requires_fall_evidence needs fall_detection_enabled")
    n = cfg["loaded_block_count"]
    if not (isinstance(n, int) and 1 <= n <= int(cfg["max_supported_blocks"])):
        p.append("loaded_block_count must be 1-3 (use --blocks)")
    if float(cfg["max_advance_px_per_increment"]) >= float(cfg["handover_jump_px"]) * 3:
        p.append("max_advance_px_per_increment looks too large compared with handover_jump_px")
    return p


# =============================================================================
# SECTION 3 -- HARDWARE DRIVERS (real and simulated)
# =============================================================================

class MC29MotorDriver:
    """PCA9685 channel feeding a VEX MC29.  ONLY sends pulse widths.

    [VERIFIED API] Adafruit CircuitPython on Linux (Blinka):
        adafruit_extended_bus.ExtendedI2C(bus_id)
        adafruit_pca9685.PCA9685(i2c, address=..., reference_clock_speed=...)
        pca.frequency = hz ; pca.channels[n].duty_cycle = 0..0xFFFF ; pca.deinit()
    IMPORTANT: the PCA9685 keeps generating the LAST commanded pulse by itself.
    If this program crashes or the Pi hangs while a drive pulse is active, the
    motor can keep running.  Software cannot fully prevent that (README).
    """
    simulated = False

    def __init__(self, cfg: Dict[str, object]):
        self.cfg = cfg
        self._lock = threading.Lock()     # watchdog thread also writes neutral
        self._i2c = self._pca = self._channel = None

    def connect(self) -> None:
        try:
            from adafruit_extended_bus import ExtendedI2C   # [VERIFIED API]
            from adafruit_pca9685 import PCA9685            # [VERIFIED API]
        except ImportError as exc:
            raise HardwareError("adafruit-circuitpython-pca9685 / adafruit-extended-bus "
                                "not installed on THIS computer (README Part 8)") from exc
        try:
            self._i2c = ExtendedI2C(int(self.cfg["i2c_bus"]))
            self._pca = PCA9685(self._i2c, address=int(self.cfg["pca9685_address"]),
                                reference_clock_speed=int(self.cfg["pca9685_reference_clock_hz"]))
            self._pca.frequency = float(self.cfg["pwm_frequency_hz"])
            self._channel = self._pca.channels[int(self.cfg["pca9685_channel"])]
        except (OSError, ValueError) as exc:
            raise HardwareError(f"PCA9685 not reachable (bus {self.cfg['i2c_bus']}, address "
                                f"{self.cfg['pca9685_address']}): {exc}") from exc
        self.write_pulse_us(float(self.cfg["motor_neutral_pulse_us"]))   # neutral immediately

    def write_pulse_us(self, pulse_us: float) -> None:
        lo, hi = float(self.cfg["mc29_min_pulse_us"]), float(self.cfg["mc29_max_pulse_us"])
        if not (lo <= pulse_us <= hi):
            raise HardwareError(f"refusing pulse {pulse_us:.0f} us outside MC29 range {lo:.0f}-{hi:.0f}")
        period_us = 1_000_000.0 / float(self.cfg["pwm_frequency_hz"])
        duty = int(round(pulse_us / period_us * 0xFFFF))
        with self._lock:
            try:
                self._channel.duty_cycle = duty
            except (OSError, AttributeError) as exc:
                raise HardwareError(f"I2C write to PCA9685 failed: {exc}") from exc

    def neutral(self) -> None:
        self.write_pulse_us(float(self.cfg["motor_neutral_pulse_us"]))

    def close(self) -> None:
        # Leave the channel at NEUTRAL (not 0 % duty: the MC29's behavior with no
        # pulses is not documented).  Then release the Python objects.
        try:
            self.neutral()
        except Exception:  # noqa: BLE001
            pass
        for obj in (self._pca, self._i2c):
            try:
                if obj is not None:
                    obj.deinit()
            except Exception:  # noqa: BLE001
                pass


class Pixy2UsbCamera:
    """Pixy2 over USB with the official libpixyusb2 SWIG module.

    [VERIFIED API] charmedlabs/pixy2 python_demos: pixy.init(), pixy.change_prog(),
    pixy.get_frame_width(), pixy.ccc_get_blocks(max, BlockArray), pixy.BlockArray(n),
    Block fields m_signature, m_x, m_y, m_width, m_height, m_angle, m_index, m_age.
    NOT documented: whether ccc_get_blocks returns negative error codes.  So a
    count of 0 is never trusted alone: the frame width is re-checked
    periodically and "no block" needs several consecutive empty readings.
    """
    simulated = False

    def __init__(self, cfg: Dict[str, object]):
        self.cfg = cfg
        self._pixy = self._array = None
        self._last_health = -1e9

    def connect(self) -> None:
        try:
            import pixy  # built with pixy2/scripts/build_python_demos.sh  [VERIFIED API]
        except ImportError as exc:
            raise HardwareError("Pixy2 'pixy' module not found. Build it on the Pi and copy "
                                "pixy.py + _pixy*.so next to this file (README Part 8).") from exc
        if pixy.init() < 0:
            raise HardwareError("pixy.init() failed: Pixy2 not connected or no USB permission")
        if pixy.change_prog(str(self.cfg["pixy_program"])) < 0:
            raise HardwareError("pixy.change_prog() failed")
        if pixy.get_frame_width() <= 0:
            raise HardwareError("pixy.get_frame_width() failed")
        self._pixy = pixy
        self._array = pixy.BlockArray(int(self.cfg["pixy_max_blocks"]))

    def read(self, now: float):
        """(ok, error_text, [RawBlock]).  ok=False is a COMMUNICATION failure."""
        if self._pixy is None:
            return False, "camera not connected", []
        try:
            if now - self._last_health >= float(self.cfg["pixy_health_check_period_s"]):
                self._last_health = now
                if self._pixy.get_frame_width() <= 0:
                    return False, "Pixy2 health check failed (frame width)", []
            n_max = int(self.cfg["pixy_max_blocks"])
            count = self._pixy.ccc_get_blocks(n_max, self._array)
        except Exception as exc:  # noqa: BLE001
            return False, f"Pixy2 read raised {exc!r}", []
        if count is None or count < 0:
            return False, f"ccc_get_blocks returned {count}", []
        out = []
        for i in range(min(count, n_max)):
            b = self._array[i]
            out.append(RawBlock(int(b.m_signature), int(b.m_x), int(b.m_y),
                                int(b.m_width), int(b.m_height), int(b.m_index)))
        return True, "", out

    def close(self) -> None:
        self._pixy = None   # the SWIG module documents no close(); released at exit


@dataclass
class RawBlock:
    signature: int
    x: int
    y: int
    width: int
    height: int
    index: int


# --------------------------- simulated hardware ------------------------------

class SimulatedMechanism:
    """SIMULATION ONLY: crank driven by a DC motor, slider-crank piston, a row of
    blocks pushed along the channel, and a side-view fake Pixy2.
    Includes speed variation and coasting after neutral (neutral != stopped)."""

    def __init__(self, cfg, order: List[str], start_gap_mm: float = 2.0, seed: int = 0):
        import random
        self.cfg, self.rng = cfg, random.Random(seed)
        self.theta = 0.0                  # crank angle (deg) -- unknown to the controller
        self.pulse = float(cfg["motor_neutral_pulse_us"])
        self.coast_until = -1.0
        self.coast_speed = 0.0
        self.battery = self.rng.uniform(0.9, 1.1)
        self.blocks: List[dict] = []      # front-most first
        self.falling: List[dict] = []
        self.fallen: List[str] = []
        rear = start_gap_mm
        for color in reversed(order):
            self.blocks.insert(0, {"color": color, "rear": rear, "front": rear + SIM_BLOCK_MM, "index": None})
            rear += SIM_BLOCK_MM
        self.next_index = 1
        # fault injection
        self.jam_at_deg: Optional[float] = None
        self.dead_motor = False
        self.i2c_fail_after: Optional[int] = None
        self.writes = 0
        self.camera_fail_after: Optional[int] = None
        self.reads = 0
        self.hide: Optional[Tuple[float, int]] = None   # (front_mm threshold, frames)
        self.hidden_left = 0
        self.dropout = 0.0
        self.flip = 0.0

    def piston_mm(self) -> float:
        return slider_crank_displacement_mm(self.theta, SIM_CRANK_MM, SIM_LINK_MM)

    def set_pulse(self, pulse: float, now: float) -> None:
        self.writes += 1
        if self.i2c_fail_after is not None and self.writes > self.i2c_fail_after:
            raise HardwareError("SIMULATED I2C failure")
        neutral = float(self.cfg["motor_neutral_pulse_us"])
        if pulse == neutral and self.pulse != neutral:
            self.coast_until, self.coast_speed = now + 0.02, self._speed(self.pulse) * 0.5
        self.pulse = pulse

    def _speed(self, pulse: float) -> float:
        if self.dead_motor:
            return 0.0
        neutral = float(self.cfg["motor_neutral_pulse_us"])
        sign = 1.0 if (pulse > neutral) == (self.cfg["motor_advance_command"] == "forward") else -1.0
        level = abs(pulse - neutral) / 500.0
        return sign * level * SIM_FREE_SPEED_DEG_S * self.battery * self.rng.uniform(0.95, 1.05)

    def update(self, now: float, dt: float) -> None:
        neutral = float(self.cfg["motor_neutral_pulse_us"])
        speed = self._speed(self.pulse) if self.pulse != neutral else (
            self.coast_speed if now < self.coast_until else 0.0)
        new_theta = self.theta + speed * dt
        if self.jam_at_deg is not None and new_theta > self.jam_at_deg:
            new_theta = self.jam_at_deg
        self.theta = new_theta
        pusher = self.piston_mm()
        for b in reversed(self.blocks):                  # push the chain
            if b["rear"] < pusher:
                shift = pusher - b["rear"]
                b["rear"] += shift
                b["front"] += shift
            pusher = b["front"]
        staying = []
        for b in self.blocks:
            if (b["rear"] + b["front"]) / 2.0 > SIM_EDGE_S_MM:
                b["fall_frames"] = 2                     # visible falling for 2 frames
                self.falling.append(b)
                self.fallen.append(b["color"])
            else:
                staying.append(b)
        self.blocks = staying
        if self.hide and self.blocks and self.blocks[0]["front"] >= self.hide[0]:
            self.hidden_left, self.hide = self.hide[1], None

    def camera(self):
        self.reads += 1
        if self.camera_fail_after is not None and self.reads > self.camera_fail_after:
            return False, "SIMULATED camera disconnect", []
        if self.hidden_left > 0:
            self.hidden_left -= 1
            return True, "", []
        if self.rng.random() < self.dropout:
            return True, "", []
        sig = self.cfg["color_signatures"]
        out = []
        groups: List[list] = []                          # merge touching same colors
        for b in sorted(self.blocks, key=lambda b: b["rear"]):
            lo, hi = max(b["rear"], SIM_CAMERA_MIN_S_MM), min(b["front"], SIM_EDGE_S_MM + 3)
            if hi - lo <= 0.3:
                continue
            if groups and groups[-1][0] == b["color"] and lo - groups[-1][2] < 0.5:
                groups[-1][2] = hi
            else:
                groups.append([b["color"], lo, hi, b])
        for color, lo, hi, first in groups:
            if first["index"] is None:
                first["index"], self.next_index = self.next_index, self.next_index + 1
            out.append(self._raw(color, lo, hi, 110, first["index"]))
        for b in list(self.falling):                    # falling block: lower in image
            if b["fall_frames"] > 0:
                drop = 25 * (3 - b["fall_frames"])
                out.append(self._raw(b["color"], SIM_EDGE_S_MM, SIM_EDGE_S_MM + 3, 110 + drop, 999))
                b["fall_frames"] -= 1
            else:
                self.falling.remove(b)
        for r in out:
            if self.rng.random() < self.flip:
                r.signature = self.rng.choice([v for v in sig.values() if v != r.signature])
        return True, "", out

    def _raw(self, color, lo, hi, y, index) -> RawBlock:
        x0 = (lo - SIM_CAMERA_MIN_S_MM) * SIM_PX_PER_MM
        x1 = (hi - SIM_CAMERA_MIN_S_MM) * SIM_PX_PER_MM
        return RawBlock(int(self.cfg["color_signatures"][color]), int((x0 + x1) / 2), int(y),
                        max(1, int(x1 - x0)), 80, int(index))


class SimulatedMotorDriver:
    simulated = True

    def __init__(self, cfg, mech: SimulatedMechanism, clock: Callable[[], float]):
        self.cfg, self.mech, self.clock = cfg, mech, clock

    def connect(self) -> None:
        self.neutral()

    def write_pulse_us(self, pulse_us: float) -> None:
        lo, hi = float(self.cfg["mc29_min_pulse_us"]), float(self.cfg["mc29_max_pulse_us"])
        if not (lo <= pulse_us <= hi):
            raise HardwareError("refusing pulse outside MC29 range")
        self.mech.set_pulse(pulse_us, self.clock())

    def neutral(self) -> None:
        self.write_pulse_us(float(self.cfg["motor_neutral_pulse_us"]))

    def close(self) -> None:
        try:
            self.neutral()
        except HardwareError:
            pass


class SimulatedCamera:
    simulated = True

    def __init__(self, mech: SimulatedMechanism):
        self.mech = mech

    def connect(self) -> None:
        pass

    def read(self, now: float):
        return self.mech.camera()

    def close(self) -> None:
        pass


# =============================================================================
# SECTION 4 -- MOTOR INCREMENT CONTROLLER (timed, non-blocking, guarded)
# =============================================================================

class MotorIncrementController:
    """Runs ONE short timed increment at a time:  drive -> neutral -> settle.

    Non-blocking: start_increment() writes the drive pulse and records a
    deadline; service() (called every control tick) writes NEUTRAL when the
    deadline passes.  A separate watchdog thread (real-time modes) writes
    neutral if service() is not called in time (e.g. a stalled callback).
    It cannot help if the whole process or the Pi dies -- the PCA9685 would
    keep the last pulse.  That is why a manual power disconnect is required.
    """

    def __init__(self, cfg, driver, log: Logger, real_time: bool, clock: Callable[[], float]):
        self.cfg, self.driver, self.log, self.clock = cfg, driver, log, clock
        self.running = False
        self.run_started = 0.0
        self.run_deadline = 0.0
        self.settle_until = 0.0
        self.locked = True
        self.cumulative_s = 0.0          # sum of measured advance run times (NOT a position)
        self.total_increments = 0
        self.shrink = {"search": 1.0, "unload": 1.0}
        self._wd_stop = threading.Event()
        self._wd_thread = None
        if real_time:
            self._wd_thread = threading.Thread(target=self._watchdog, daemon=True)
            self._wd_thread.start()

    # -- watchdog (second line of defense inside this process) --
    def _watchdog(self) -> None:
        limit = float(self.cfg["max_single_motor_run_time_s"]) + 0.05
        while not self._wd_stop.wait(0.005):
            if self.running and time.monotonic() - self.run_started > limit:
                try:
                    self.driver.neutral()
                    self.log.error("WATCHDOG: increment overran its deadline -> NEUTRAL written")
                except Exception:  # noqa: BLE001
                    pass

    def stop_watchdog(self) -> None:
        self._wd_stop.set()

    # -- state --
    def lock(self) -> None:
        self.locked = True
        if self.running:
            self.stop_now("motion locked")

    def unlock(self) -> None:
        self.locked = False

    def busy(self, now: float) -> bool:
        return self.running or now < self.settle_until

    def run_time_for(self, kind: str) -> float:
        """Base run time for this stage of the (estimated) stroke, then camera-
        based shrinking.  Stage selection uses cumulative run time, which is only
        an ESTIMATE of stroke stage."""
        base = float(self.cfg[f"{kind}_run_time_s"])
        for stage in self.cfg.get("run_time_stages") or []:
            if self.cumulative_s < float(stage["until_cumulative_s"]):
                base = float(stage[f"{kind}_run_time_s"])
                break
        return max(float(self.cfg["min_run_time_s"]), base * self.shrink[kind])

    def remaining_budget_s(self) -> float:
        return float(self.cfg["max_cumulative_motor_run_time_s"]) - self.cumulative_s

    def start_increment(self, kind: str, now: float) -> Tuple[bool, str]:
        if self.locked:
            return False, "motor locked"
        if self.busy(now):
            return False, "previous increment not finished"
        if self.total_increments >= int(self.cfg["max_total_increments"]):
            return False, "max_total_increments reached"
        run = self.run_time_for(kind)
        if run > float(self.cfg["max_single_motor_run_time_s"]):
            return False, "run time above max_single_motor_run_time_s"
        if self.cumulative_s + run > float(self.cfg["max_cumulative_motor_run_time_s"]):
            return False, (f"cumulative run-time budget would be exceeded "
                           f"({self.cumulative_s:.2f}+{run:.2f} > {self.cfg['max_cumulative_motor_run_time_s']} s)")
        self.driver.write_pulse_us(advance_pulse_us(self.cfg))   # may raise HardwareError
        self.running, self.run_started, self.run_deadline = True, now, now + run
        self.total_increments += 1
        return True, f"{kind} increment {run:.3f} s"

    def service(self, now: float) -> None:
        if self.running and now >= self.run_deadline:
            self._finish(now)

    def _finish(self, now: float) -> None:
        self.driver.neutral()
        actual = now - self.run_started            # includes timing overshoot
        self.running = False
        self.cumulative_s += actual
        self.settle_until = now + float(self.cfg["motor_settle_time_s"])

    def stop_now(self, why: str) -> None:
        """Immediate neutral (faults, navigation, shutdown)."""
        try:
            self.driver.neutral()
        finally:
            if self.running:
                self.cumulative_s += max(0.0, self.clock() - self.run_started)
                self.running = False
                self.log.warn(f"Motor stopped early: {why}")

    def shorten(self, kind: str) -> None:
        old = self.shrink[kind]
        self.shrink[kind] = max(0.05, old * float(self.cfg["increment_shrink_factor"]))
        self.log.info(f"Block moved a lot in one {kind} increment -> shortening {kind} "
                      f"increments (factor {old:.2f} -> {self.shrink[kind]:.2f})")


# =============================================================================
# SECTION 5 -- PIXY2 INTERPRETATION (side view)
# =============================================================================

@dataclass
class Detection:
    color: Optional[str]
    signature: int
    front: float          # leading-edge "progress" toward the unloading edge (px, larger = closer)
    rear: float           # trailing edge, same units (still visible when the front leaves the view)
    fall_coord: Optional[float]
    raw: RawBlock


@dataclass
class CameraReading:
    ok: bool
    error: str = ""
    valid: List[Detection] = field(default_factory=list)     # full-size, known color
    entering: List[Detection] = field(default_factory=list)  # partial blocks in view
    unknown_signatures: int = 0

    @property
    def leading(self) -> Optional[Detection]:
        """Valid block CLOSEST TO THE EDGE (not the largest)."""
        return max(self.valid, key=lambda d: d.front) if self.valid else None

    @property
    def leading_any(self) -> Optional[Detection]:
        both = self.valid + self.entering
        return max(both, key=lambda d: d.front) if both else None


class CameraEvent(Enum):
    COMM_FAILURE = "camera communication failure"
    NO_BLOCK = "no block visible"
    BLOCK_ENTERING = "block entering field of view (partial)"
    VALID_BLOCK = "valid block identified"
    MULTIPLE_BLOCKS = "multiple blocks visible"
    APPROACHING_EDGE = "leading block approaching the unloading edge"


class BlockDetector:
    def __init__(self, cfg):
        self.cfg = cfg
        self.sig_to_color = {int(s): c for c, s in cfg["color_signatures"].items()}
        self.axis, self.dir = cfg["leading_axis"], int(cfg["leading_direction"])
        self.edge = float(cfg["edge_px"]) * self.dir

    def process(self, ok: bool, error: str, raw: List[RawBlock]) -> CameraReading:
        if not ok:
            return CameraReading(ok=False, error=error)
        x0, x1, y0, y1 = self.cfg["roi_px"]
        r = CameraReading(ok=True)
        for b in raw:
            if not (x0 <= b.x <= x1 and y0 <= b.y <= y1) or b.signature > 7:
                continue
            center = b.x if self.axis == "x" else b.y
            half = (b.width if self.axis == "x" else b.height) / 2.0
            fall = None
            if self.cfg["fall_detection_enabled"] or self.cfg.get("fall_axis"):
                fall_axis = self.cfg.get("fall_axis")
                if fall_axis in ("x", "y"):
                    fall = (b.x if fall_axis == "x" else b.y) * int(self.cfg.get("fall_direction") or 1)
            color = self.sig_to_color.get(b.signature)
            d = Detection(color, b.signature, center * self.dir + half, center * self.dir - half, fall, b)
            area = b.width * b.height
            if color is None:
                r.unknown_signatures += 1
                continue
            if b.width >= self.cfg["min_block_width_px"] and b.height >= self.cfg["min_block_height_px"] \
                    and area >= self.cfg["min_block_area_px"]:
                r.valid.append(d)
            elif area >= self.cfg["entering_min_area_px"]:
                r.entering.append(d)
        return r

    def in_exit_zone(self, front: float) -> bool:
        return front >= self.edge - float(self.cfg["exit_zone_margin_px"])

    def classify(self, r: CameraReading) -> CameraEvent:
        if not r.ok:
            return CameraEvent.COMM_FAILURE
        if r.leading is None:
            return CameraEvent.BLOCK_ENTERING if r.entering else CameraEvent.NO_BLOCK
        if self.in_exit_zone(r.leading.front):
            return CameraEvent.APPROACHING_EDGE
        return CameraEvent.MULTIPLE_BLOCKS if len(r.valid) > 1 else CameraEvent.VALID_BLOCK


class ColorConfirmer:
    """Multi-reading vote on the leading block's color.  update() returns None
    until decided, then ("COLOR", color, det) / ("EMPTY", ..) / ("UNSTABLE", ..)."""

    def __init__(self, cfg):
        self.need = int(cfg["color_confirmation_frames"])
        self.need_empty = int(cfg["empty_confirmation_frames"])
        self.max_frames = int(cfg["color_max_attempt_frames"])
        self.frames = self.streak = self.empty = 0
        self.color: Optional[str] = None

    def update(self, r: CameraReading):
        self.frames += 1
        lead = r.leading
        if lead is None:
            self.empty += 1
            self.streak, self.color = 0, None
            if self.empty >= self.need_empty:
                return ("EMPTY", None, None)
        else:
            self.empty = 0
            self.streak = self.streak + 1 if lead.color == self.color else 1
            self.color = lead.color
            if self.streak >= self.need:
                return ("COLOR", self.color, lead)
        if self.frames >= self.max_frames:
            return ("UNSTABLE", None, None)
        return None


# =============================================================================
# SECTION 6 -- DELIVERY STATE MACHINE (no ROS, no hardware inside)
# =============================================================================

class State(Enum):
    INITIALIZING = "INITIALIZING"
    WAITING_FOR_START = "WAITING_FOR_START"          # operator verifies piston start position
    SEARCHING_FOR_BLOCK = "SEARCHING_FOR_BLOCK"
    CONFIRMING_COLOR = "CONFIRMING_COLOR"
    NAVIGATING = "NAVIGATING"                        # request sent, waiting for ACCEPTED
    WAITING_FOR_ARRIVAL = "WAITING_FOR_ARRIVAL"
    VERIFYING_AT_STATION = "VERIFYING_AT_STATION"
    UNLOADING = "UNLOADING"
    VERIFYING_EJECTION = "VERIFYING_EJECTION"
    UNCERTAIN_DELIVERY = "UNCERTAIN_DELIVERY"        # stopped; operator must decide
    CHECKING_NEXT_BLOCK = "CHECKING_NEXT_BLOCK"
    COMPLETE = "COMPLETE"
    FAULT = "FAULT"


MOTION_STATES = (State.SEARCHING_FOR_BLOCK, State.UNLOADING)
NAV_ACCEPTED, NAV_NAVIGATING, NAV_ARRIVED, NAV_FAILED = "ACCEPTED", "NAVIGATING", "ARRIVED", "FAILED"
START_WAIT_S = 10.0


class DeliveryCore:
    """All delivery decisions.  Call tick(now) every control period."""

    def __init__(self, cfg, camera, driver, log: Logger, publish_destination, publish_stop,
                 simulated: bool, real_time: bool, session_id: Optional[str] = None,
                 clock: Callable[[], float] = time.monotonic):
        self.cfg, self.camera, self.log = cfg, camera, log
        self.simulated = simulated
        self.detector = BlockDetector(cfg)
        self.motor = MotorIncrementController(cfg, driver, log, real_time, clock)
        self.driver = driver
        self._pub_dest, self._pub_stop = publish_destination, publish_stop
        self.session_id = session_id or uuid.uuid4().hex[:8]

        self.state, self.state_since = State.INITIALIZING, 0.0
        self.fault_reason = ""
        self.uncertain_reason = ""
        self.start_requested_at: Optional[float] = None
        self.mission_started: Optional[float] = None
        self.operator_ejection: Optional[bool] = None

        self.loaded = int(cfg["loaded_block_count"])
        self.delivered: List[str] = []
        self.current_color: Optional[str] = None
        self.current_station: Optional[str] = None
        self.target_station: Optional[str] = None
        self.request_id = 0
        self.nav_sent_at = 0.0
        self.last_dest_pub = -1e9
        self.arrival_since: Optional[float] = None
        self.nav: Optional[dict] = None
        self.nav_time = -1e9
        self.odom_still_since: Optional[float] = None
        self.odom_time = -1e9
        self.not_stationary_since: Optional[float] = None

        self.last_camera_read = -1e9
        self.last_camera_ok = None
        self.camera_errors = 0
        self.confirmer: Optional[ColorConfirmer] = None
        self.reading_after_step: Optional[CameraReading] = None

        # per-block tracking
        self.search_increments = 0
        self.unload_increments = 0
        self.no_progress = 0
        self.contact_established = False   # camera has seen the piston move a block
        self.front_before_step: Optional[float] = None
        self.last_kind = "search"
        self.last_front: Optional[float] = None
        self.reached_exit_zone = False
        self.fall_baseline: Optional[float] = None
        self.fall_seen = False
        self.lost = 0
        self.color_mismatch = 0
        self.verify_n = self.verify_gone = 0

    # ---------------------------------------------------------------- basics
    def _set_state(self, s: State, now: float, why: str) -> None:
        if s != self.state:
            self.log.info(f"STATE {self.state.value} -> {s.value}: {why}")
        self.state, self.state_since = s, now
        if s in MOTION_STATES:
            self.motor.unlock()
        else:
            self.motor.lock()       # neutral is written if an increment was running

    def fault(self, now: float, reason: str) -> None:
        if self.state == State.FAULT:
            return
        try:
            self.motor.stop_now("fault")
        except HardwareError as exc:
            reason += f" (neutral write ALSO failed: {exc} -- disconnect motor power!)"
        self.fault_reason = reason
        self.log.error(f"FAULT: {reason}")
        self._set_state(State.FAULT, now, reason)
        self._send_stop(reason)

    def _send_stop(self, reason: str) -> None:
        try:
            self._pub_stop({"session": self.session_id, "request_id": self.request_id, "reason": reason})
        except Exception:  # noqa: BLE001
            pass

    def uncertain(self, now: float, reason: str) -> None:
        self.motor.lock()
        self.uncertain_reason = reason
        self.operator_ejection = None
        self.log.error(f"UNCERTAIN DELIVERY: {reason}  Inspect the channel, then publish "
                       f"true (block is off the platform) or false on {self.cfg['topic_operator_ejection']}.")
        self._set_state(State.UNCERTAIN_DELIVERY, now, reason)

    # ---------------------------------------------------------------- inputs
    def on_nav_status(self, msg: dict, now: float) -> None:
        self.nav, self.nav_time = msg, now

    def on_odom(self, linear: float, angular: float, now: float) -> None:
        self.odom_time = now
        still = abs(linear) <= float(self.cfg["odom_max_linear_mps"]) and \
            abs(angular) <= float(self.cfg["odom_max_angular_radps"])
        if not still:
            self.odom_still_since = None
        elif self.odom_still_since is None:
            self.odom_still_since = now

    def request_start(self, now: float) -> None:
        if self.state == State.WAITING_FOR_START:
            self.start_requested_at = now

    def request_abort(self, now: float) -> None:
        self.fault(now, "operator abort")

    def on_operator_ejection(self, ejected: bool, now: float) -> None:
        if self.state == State.UNCERTAIN_DELIVERY:
            self.operator_ejection = bool(ejected)

    # ------------------------------------------------------- stationary checks
    def nav_fresh(self, now: float) -> bool:
        return self.nav is not None and now - self.nav_time <= float(self.cfg["nav_heartbeat_timeout_s"])

    def stationary(self, now: float, dwell: float = 0.0) -> Tuple[bool, str]:
        if not self.nav_fresh(now):
            return False, "no fresh navigation status"
        if self.nav.get("is_stopped") is not True:
            return False, "navigation does not confirm the robot is stationary"
        if self.cfg["use_odom_stationary_check"]:
            if now - self.odom_time > float(self.cfg["odom_timeout_s"]):
                return False, f"no fresh odometry on {self.cfg['odom_topic']}"
            if self.odom_still_since is None or now - self.odom_still_since < dwell:
                return False, "odometry shows motion"
        return True, ""

    def at_confirmed_station(self, now: float) -> Tuple[bool, str]:
        """Fresh confirmation that the robot is stopped AT the current station."""
        ok, why = self.stationary(now)
        if not ok:
            return False, why
        n = self.nav
        if n.get("session") != self.session_id or n.get("request_id") != self.request_id:
            return False, "navigation status is for a different request"
        if n.get("state") != NAV_ARRIVED or n.get("station") != self.current_station:
            return False, "navigation no longer reports ARRIVED at this station"
        return True, ""

    # ------------------------------------------------------------- camera
    def poll_camera(self, now: float) -> Optional[CameraReading]:
        """Returns a new reading when one is due, else None.  Communication
        failures are NEVER treated as an empty platform."""
        if now - self.last_camera_read < float(self.cfg["camera_read_period_s"]):
            return None
        self.last_camera_read = now
        ok, err, raw = self.camera.read(now)
        r = self.detector.process(ok, err, raw)
        if not r.ok:
            self.camera_errors += 1
            self.log.warn(f"Camera error {self.camera_errors}: {r.error}")
            too_many = self.camera_errors >= int(self.cfg["max_consecutive_camera_errors"])
            too_long = self.last_camera_ok is not None and \
                now - self.last_camera_ok > float(self.cfg["camera_comm_timeout_s"])
            if too_many or too_long:
                self.fault(now, f"Camera failure: {r.error}")
            return None
        self.camera_errors, self.last_camera_ok = 0, now
        return r

    def _track(self, r: CameraReading) -> None:
        """Remember leading-edge position and look for downward (falling) motion."""
        lead = r.leading
        if lead is not None and self.detector.in_exit_zone(lead.front):
            self.reached_exit_zone = True
            if self.fall_baseline is None and lead.fall_coord is not None:
                self.fall_baseline = lead.fall_coord
        if self.cfg["fall_detection_enabled"] and self.fall_baseline is not None:
            for d in r.valid + r.entering:
                if d.fall_coord is not None and self.detector.in_exit_zone(d.front) and \
                        d.fall_coord - self.fall_baseline >= float(self.cfg["fall_min_px"]):
                    if not self.fall_seen:
                        self.log.info("Camera saw the edge block move DOWNWARD (falling evidence).")
                    self.fall_seen = True

    def _new_block(self, det: Detection) -> None:
        self.last_front = det.front
        self.reached_exit_zone = self.detector.in_exit_zone(det.front)
        self.fall_baseline = det.fall_coord if self.reached_exit_zone else None
        self.fall_seen = False
        self.lost = self.color_mismatch = self.unload_increments = self.no_progress = 0

    # ----------------------------------------------------- increments + progress
    def _advance(self, kind: str, now: float, reading: CameraReading) -> bool:
        lead = reading.leading_any
        self.front_before_step = (lead.front, lead.rear) if lead else None
        self.last_kind = kind
        try:
            ok, msg = self.motor.start_increment(kind, now)
        except HardwareError as exc:
            self.fault(now, f"Motor controller / I2C failure: {exc}")
            return False
        if not ok:
            self.fault(now, f"Motor increment refused: {msg}. Timed control cannot see the piston; "
                       "check the mechanism before restarting.")
            return False
        self.log.info(f"{msg} (estimated cumulative {self.motor.cumulative_s:.2f} s of "
                      f"{self.cfg['max_cumulative_motor_run_time_s']} s budget)")
        return True

    def _check_progress(self, reading: CameraReading, now: float) -> bool:
        """After an increment: did the visible block move?  Returns False on fault."""
        if self.front_before_step is None:
            return True
        lead = reading.leading_any
        if lead is None:
            return True                 # block may have left; handled by caller
        front0, rear0 = self.front_before_step
        self.front_before_step = None
        if lead.front - front0 < -float(self.cfg["handover_jump_px"]) / 2:
            return True                 # handover / new leading block, not a progress sample
        # Use whichever edge moved more: the front edge stops changing once the
        # block hangs past the camera's view, but the rear edge keeps moving.
        moved = max(lead.front - front0, lead.rear - rear0)
        if moved < float(self.cfg["min_progress_px_per_increment"]):
            if not self.contact_established:
                # Piston may still be closing the gap to the blocks.  Only the
                # cumulative budget / increment limits protect this phase.
                self.log.info("Visible block not moving yet (piston may not be touching the row).")
                return True
            self.no_progress += 1
            self.log.warn(f"Visible block moved only {moved:.0f}px after an increment "
                          f"({self.no_progress}/{self.cfg['no_progress_increment_limit']}).")
            if self.no_progress >= int(self.cfg["no_progress_increment_limit"]):
                self.fault(now, "Block is not moving although the motor was commanded: possible jam, "
                           "stalled/unpowered motor, or crank rotated past full extension.")
                return False
        else:
            self.no_progress = 0
            self.contact_established = True
            if moved > float(self.cfg["max_advance_px_per_increment"]):
                self.motor.shorten(self.last_kind)
        return True

    # ------------------------------------------------------------- main tick
    def startup(self, now: float) -> None:
        tag = "SIMULATION" if self.simulated else "HARDWARE"
        self.log.info(f"=== Delivery subsystem, {tag} mode, session {self.session_id}, "
                      f"{self.loaded} block(s) expected ===")
        try:
            self.driver.connect()          # writes NEUTRAL first
            self.camera.connect()
        except HardwareError as exc:
            self.fault(now, f"Initialization failed: {exc}")
            return
        self.log.info(f"Motor neutral commanded ({self.cfg['motor_neutral_pulse_us']} us). "
                      "Neutral is a command only; confirm visually that the motor is still.")
        self._set_state(State.WAITING_FOR_START, now,
                        f"verify the piston START POSITION by eye, then publish on {self.cfg['topic_start']}")

    def tick(self, now: float) -> None:
        try:
            self.motor.service(now)
            self._tick(now)
        except HardwareError as exc:
            self.fault(now, f"Motor controller / I2C failure: {exc}")

    def _tick(self, now: float) -> None:
        s = self.state
        if s == State.FAULT:
            if now - self.last_dest_pub > 1.0:
                self._send_stop(self.fault_reason)
                self.last_dest_pub = now
            return
        if s in (State.INITIALIZING, State.COMPLETE):
            return
        limit = self.cfg.get("mission_timeout_s")
        if limit and self.mission_started is not None and now - self.mission_started > float(limit):
            self.fault(now, "mission timeout")
            return

        # Safety: motor may run only in motion states, and in UNLOADING only while
        # the robot is confirmed stopped at the right station.
        if self.motor.running and s not in MOTION_STATES:
            self.motor.stop_now(f"state {s.value}")
        if s in (State.UNLOADING, State.VERIFYING_EJECTION):
            ok, why = self.at_confirmed_station(now)
            if not ok:
                if self.motor.running:
                    self.fault(now, f"Lost stationary/arrival confirmation while the motor was running: {why}")
                    return
                self.motor.lock()
                self.not_stationary_since = self.not_stationary_since or now
                if now - self.not_stationary_since > float(self.cfg["stationary_loss_grace_s"]):
                    self.fault(now, f"Lost stationary/arrival confirmation at the station: {why}")
                return
            self.not_stationary_since = None
            if s == State.UNLOADING:
                self.motor.unlock()
        if s in (State.SEARCHING_FOR_BLOCK, State.CONFIRMING_COLOR, State.CHECKING_NEXT_BLOCK,
                 State.VERIFYING_AT_STATION):
            ok, why = self.stationary(now)
            if not ok:
                if self.motor.running:
                    self.fault(now, f"Robot not confirmed stationary while the motor was running: {why}")
                    return
                self.motor.lock()
                self.not_stationary_since = self.not_stationary_since or now
                if now - self.not_stationary_since > float(self.cfg["stationary_loss_grace_s"]):
                    self.fault(now, f"Robot not confirmed stationary: {why}")
                return
            self.not_stationary_since = None
            if s == State.SEARCHING_FOR_BLOCK:
                self.motor.unlock()

        handler = {
            State.WAITING_FOR_START: self._t_wait_start,
            State.SEARCHING_FOR_BLOCK: self._t_search,
            State.CONFIRMING_COLOR: self._t_confirm,
            State.NAVIGATING: self._t_navigating,
            State.WAITING_FOR_ARRIVAL: self._t_arrival,
            State.VERIFYING_AT_STATION: self._t_verify_station,
            State.UNLOADING: self._t_unload,
            State.VERIFYING_EJECTION: self._t_verify_eject,
            State.UNCERTAIN_DELIVERY: self._t_uncertain,
            State.CHECKING_NEXT_BLOCK: self._t_next,
        }[s]
        handler(now)

    # ----------------------------------------------------- Stage: start
    def _t_wait_start(self, now: float) -> None:
        if self.start_requested_at is None:
            return
        ok, why = self.stationary(now)
        if not ok:
            if now - self.start_requested_at > START_WAIT_S:
                self.fault(now, f"Start refused: {why}. Is the navigation program running?")
            return
        self.mission_started = now
        self.log.info("Operator confirmed the piston start position. Cumulative run-time "
                      "budget starts at 0 from here.")
        self.confirmer = ColorConfirmer(self.cfg)
        self._set_state(State.SEARCHING_FOR_BLOCK, now, "mission started")

    # ----------------------------------------------------- Stage: search
    def _t_search(self, now: float) -> None:
        if self.motor.busy(now):
            return
        r = self.poll_camera(now)
        if r is None:
            return
        if not self._check_progress(r, now):
            return
        if r.leading is not None:
            self.log.info(f"Camera: {self.detector.classify(r).value} -> stop and confirm color.")
            self.confirmer = ColorConfirmer(self.cfg)
            self._set_state(State.CONFIRMING_COLOR, now, "candidate block detected")
            return
        verdict = self.confirmer.update(r) if self.confirmer else None
        if verdict is None:
            return                               # need several empty readings first
        self.confirmer = ColorConfirmer(self.cfg)
        if self.search_increments >= int(self.cfg["max_search_increments"]):
            self.fault(now, f"No block found after {self.search_increments} search increments "
                       f"({len(self.delivered)}/{self.loaded} delivered): wrong block count, jam, "
                       "or the piston is at the end of its usable stroke.")
            return
        kind = "unload" if r.entering else "search"   # small steps once a block is entering view
        if r.entering:
            self.log.info("Camera: block entering view -> using small increments.")
        if self._advance(kind, now, r):
            self.search_increments += 1

    # ----------------------------------------------------- Stage: color
    def _t_confirm(self, now: float) -> None:
        r = self.poll_camera(now)
        if r is None:
            return
        verdict = self.confirmer.update(r)
        if verdict is None:
            return
        kind, color, det = verdict
        if kind == "EMPTY":
            self.confirmer = ColorConfirmer(self.cfg)
            self._set_state(State.SEARCHING_FOR_BLOCK, now, "detection did not persist")
            return
        if kind == "UNSTABLE":
            self.fault(now, "Uncertain block identification (color not stable). Check signatures, "
                       "lighting, ROI.")
            return
        self._new_block(det)
        self.search_increments = 0
        self.current_color = color
        station = str(self.cfg["station_ids"][color])
        self.log.info(f"Leading block confirmed {color.upper()} -> destination '{station}'.")
        if station == self.current_station and self.at_confirmed_station(now)[0]:
            self._set_state(State.UNLOADING, now, "already stopped at the correct station")
        else:
            self._request_destination(station, color, now)

    # ----------------------------------------------------- Stage: navigation
    def _request_destination(self, station: str, color: str, now: float) -> None:
        self.request_id += 1
        self.target_station, self.current_station = station, None
        self.nav_sent_at, self.last_dest_pub, self.arrival_since = now, -1e9, None
        self.log.info(f"Requesting destination '{station}' for {color.upper()} block "
                      f"(request {self.request_id}). Motor stays at neutral while travelling.")
        self._set_state(State.NAVIGATING, now, f"destination {station}")

    def _publish_destination(self, now: float) -> None:
        if now - self.last_dest_pub >= float(self.cfg["destination_republish_period_s"]):
            self._pub_dest({"session": self.session_id, "request_id": self.request_id,
                            "color": self.current_color, "station": self.target_station})
            self.last_dest_pub = now

    def _matching(self, now: float) -> Optional[dict]:
        if not self.nav_fresh(now):
            return None
        n = self.nav
        if n.get("session") != self.session_id or n.get("request_id") != self.request_id:
            return None                              # stale or for another request
        return n

    def _t_navigating(self, now: float) -> None:
        self._publish_destination(now)
        n = self._matching(now)
        if n and n.get("state") == NAV_FAILED:
            self.fault(now, f"Navigation failure: {n.get('detail', '')}")
            return
        if n and n.get("state") in (NAV_ACCEPTED, NAV_NAVIGATING, NAV_ARRIVED):
            self._set_state(State.WAITING_FOR_ARRIVAL, now, "navigation accepted the request")
            return
        if now - self.nav_sent_at > float(self.cfg["nav_ack_timeout_s"]):
            self.fault(now, "Navigation did not acknowledge the destination request")

    def _t_arrival(self, now: float) -> None:
        self._publish_destination(now)
        if now - self.nav_sent_at > float(self.cfg["nav_timeout_s"]):
            self.fault(now, f"Navigation timeout (destination '{self.target_station}')")
            return
        if not self.nav_fresh(now) and now - self.nav_time > 2 * float(self.cfg["nav_heartbeat_timeout_s"]):
            self.fault(now, "Navigation communication lost while travelling")
            return
        n = self._matching(now)
        if n is None:
            self.arrival_since = None
            return
        if n.get("state") == NAV_FAILED:
            self.fault(now, f"Navigation failure: {n.get('detail', '')}")
            return
        if n.get("state") != NAV_ARRIVED:
            self.arrival_since = None
            return
        if n.get("station") != self.target_station:
            self.fault(now, f"Incorrect destination confirmation: navigation reports "
                       f"'{n.get('station')}', requested '{self.target_station}'")
            return
        ok, _ = self.stationary(now, float(self.cfg["arrival_settle_time_s"]))
        if not ok:
            self.arrival_since = None
            return
        self.arrival_since = self.arrival_since or now
        if now - self.arrival_since < float(self.cfg["arrival_settle_time_s"]):
            return
        self.current_station = self.target_station
        self.confirmer = ColorConfirmer(self.cfg)
        self._set_state(State.VERIFYING_AT_STATION, now, f"arrived and stationary at '{self.current_station}'")

    def _t_verify_station(self, now: float) -> None:
        r = self.poll_camera(now)
        if r is None:
            return
        verdict = self.confirmer.update(r)
        if verdict is None:
            return
        kind, color, det = verdict
        if kind == "COLOR" and color == self.current_color:
            self._new_block(det)
            self._set_state(State.UNLOADING, now, f"leading block is still {color.upper()}")
        elif kind == "COLOR":
            self.fault(now, f"Leading block is now {color}, expected {self.current_color}. "
                       "Not unloading at this station.")
        else:
            self.fault(now, f"Could not re-confirm the leading block at the station ({kind}).")

    # ----------------------------------------------------- Stage: unloading
    def _t_unload(self, now: float) -> None:
        r = self.poll_camera(now)
        if self.motor.busy(now):
            if r is not None:
                self._track(r)          # keep watching (falling motion) while moving/settling
            return
        if r is None:
            return
        if not self._check_progress(r, now):
            return
        self._track(r)
        lead = r.leading
        if lead is None:
            if self.reached_exit_zone:
                self._begin_verify(now, "leading block left the view at the unloading edge")
                return
            self.lost += 1
            if self.lost >= int(self.cfg["lost_block_tolerance_frames"]):
                self.uncertain(now, "block disappeared AWAY from the unloading edge (obscured? tipped?)")
            return                                   # never push a block we cannot see
        self.lost = 0
        if self.last_front is not None and lead.front < self.last_front - float(self.cfg["handover_jump_px"]):
            if self.reached_exit_zone:
                self._begin_verify(now, "leading edge jumped back: next block is now leading")
            else:
                self.uncertain(now, "leading edge jumped back before the block reached the edge")
            return
        if lead.color != self.current_color:
            self.color_mismatch += 1
            if self.color_mismatch >= int(self.cfg["color_confirmation_frames"]) * 2:
                self.fault(now, f"Leading block color changed to {lead.color} without a detected "
                           "ejection. Stopping to prevent a wrong delivery.")
            return
        self.color_mismatch = 0
        self.last_front = max(self.last_front if self.last_front is not None else lead.front, lead.front)
        if self.unload_increments >= int(self.cfg["max_unload_increments"]):
            self.fault(now, f"Block not ejected after {self.unload_increments} unloading increments "
                       "(jam or crank at end of stroke?)")
            return
        if self._advance("unload", now, r):
            self.unload_increments += 1

    def _begin_verify(self, now: float, why: str) -> None:
        self.verify_n = self.verify_gone = 0
        self._set_state(State.VERIFYING_EJECTION, now, why)

    def _t_verify_eject(self, now: float) -> None:
        r = self.poll_camera(now)
        if r is None:
            return
        self._track(r)
        lead = r.leading
        gone = lead is None or lead.front < self.last_front - float(self.cfg["handover_jump_px"])
        self.verify_n += 1
        self.verify_gone += int(gone)
        if self.verify_n < int(self.cfg["ejection_verify_frames"]):
            return
        if self.verify_gone < self.verify_n - 1:
            self.log.warn("Ejection not confirmed (block still seen) -> continue unloading.")
            self._set_state(State.UNLOADING, now, "block still present")
            return
        if self.cfg["ejection_requires_fall_evidence"] and not self.fall_seen:
            self.uncertain(now, "block left the view but no downward (falling) motion was seen")
            return
        self._count_delivery(now, "camera" + (" + falling motion" if self.fall_seen else ""))

    def _count_delivery(self, now: float, evidence: str) -> None:
        self.delivered.append(self.current_color)
        self.log.info(f"DELIVERED #{len(self.delivered)}: {self.current_color.upper()} at "
                      f"'{self.current_station}' (evidence: {evidence}).")
        self.confirmer = ColorConfirmer(self.cfg)
        self._set_state(State.CHECKING_NEXT_BLOCK, now, "identify the next block")

    def _t_uncertain(self, now: float) -> None:
        if self.operator_ejection is None:
            return
        if self.operator_ejection:
            self._count_delivery(now, "OPERATOR confirmation")
        else:
            self.fault(now, f"Operator reports the block was NOT delivered ({self.uncertain_reason})")

    # ----------------------------------------------------- Stage: next block
    def _t_next(self, now: float) -> None:
        r = self.poll_camera(now)
        if r is None:
            return
        verdict = self.confirmer.update(r)
        if verdict is None:
            return
        kind, color, det = verdict
        if len(self.delivered) >= self.loaded:
            if kind == "COLOR":
                self.fault(now, f"Incorrect block count: {self.loaded} delivered but another "
                           f"{color} block is visible.")
                return
            self._complete(now)
            return
        if kind == "UNSTABLE":
            self.fault(now, "Next block color uncertain")
            return
        if kind == "EMPTY":
            self.confirmer = ColorConfirmer(self.cfg)
            self.search_increments = 0
            self._set_state(State.SEARCHING_FOR_BLOCK, now,
                            f"{self.loaded - len(self.delivered)} block(s) remain; advancing them into view")
            return
        self._new_block(det)
        self.current_color = color
        station = str(self.cfg["station_ids"][color])
        if station == self.current_station:
            ok, why = self.at_confirmed_station(now)
            if ok:
                self._set_state(State.UNLOADING, now,
                                f"next block is also {color.upper()}: staying at '{station}'")
            else:
                self.fault(now, f"Same-color block but station confirmation is not fresh: {why}")
        else:
            self.log.info(f"Next block is {color.upper()}: motor stopped, requesting new destination.")
            self._request_destination(station, color, now)

    def _complete(self, now: float) -> None:
        self.motor.stop_now("mission complete")
        self._set_state(State.COMPLETE, now, "all expected blocks delivered")
        self._send_stop("mission complete")
        tag = "SIMULATED " if self.simulated else ""
        self.log.info(f"*** {tag}MISSION COMPLETE: {len(self.delivered)} delivered: "
                      f"{', '.join(c.upper() for c in self.delivered)} ***")

    # ----------------------------------------------------- shutdown + status
    def shutdown(self, reason: str) -> None:
        self.log.info(f"Shutdown: {reason}. Commanding neutral.")
        try:
            self.motor.stop_now(reason)
        except Exception as exc:  # noqa: BLE001
            self.log.error(f"Neutral write failed at shutdown ({exc}) -- DISCONNECT MOTOR POWER")
        self.motor.lock()
        self.motor.stop_watchdog()
        self._send_stop(reason)
        self.driver.close()
        self.camera.close()

    def status(self) -> dict:
        return {"mode": "SIMULATION" if self.simulated else "HARDWARE", "session": self.session_id,
                "state": self.state.value, "fault_reason": self.fault_reason,
                "uncertain_reason": self.uncertain_reason if self.state == State.UNCERTAIN_DELIVERY else "",
                "expected_blocks": self.loaded, "delivered": list(self.delivered),
                "current_color": self.current_color, "current_station": self.current_station,
                "target_station": self.target_station, "request_id": self.request_id,
                "motor_running": self.motor.running,
                "estimated_cumulative_run_s": round(self.motor.cumulative_s, 3),
                "increments": self.motor.total_increments}


# =============================================================================
# SECTION 7 -- BUILDERS
# =============================================================================

def build_core(mode, cfg, log, publish_destination, publish_stop, sim_order=None,
               clock=time.monotonic, real_time=True, session_id=None, mech=None):
    problems = validate_config(cfg, mode)
    if problems:
        raise ConfigurationError("Configuration incomplete -- motor will NOT be energized:\n  - "
                                 + "\n  - ".join(problems))
    if mode == "hardware":
        driver, camera = MC29MotorDriver(cfg), Pixy2UsbCamera(cfg)
    else:
        if mech is None:
            order = sim_order or ["red", "green", "blue"][: int(cfg["loaded_block_count"])]
            mech = SimulatedMechanism(cfg, order)
        driver, camera = SimulatedMotorDriver(cfg, mech, clock), SimulatedCamera(mech)
    core = DeliveryCore(cfg, camera, driver, log, publish_destination, publish_stop,
                        simulated=(mode != "hardware"), real_time=real_time, session_id=session_id,
                        clock=clock)
    return core, mech


# =============================================================================
# SECTION 8 -- OFFLINE SIMULATION HARNESS (no ROS, simulated navigation inside)
# =============================================================================

class SimNavigationHarness:
    """Internal stand-in that answers destination requests -- ONLY for offline
    tests.  It is NOT a navigation program."""

    def __init__(self, travel_s: float = 3.0):
        self.travel_s = travel_s
        self.session = None
        self.req = 0
        self.state = "IDLE"
        self.station = "start"
        self.t0 = 0.0
        self.visits: List[str] = []
        self.stopped = True
        # fault injection
        self.wrong_station = self.never_arrive = self.silent = False
        self.stale_first = False
        self.moving_during_unload_at: Optional[float] = None
        self.prev = None

    def on_destination(self, m: dict, now: float) -> None:
        if (m["session"], m["request_id"]) == (self.session, self.req):
            return                                      # duplicate
        if m["session"] == self.session and m["request_id"] < self.req:
            return                                      # stale
        self.prev = (self.session, self.req, self.station)
        self.session, self.req, self.target = m["session"], m["request_id"], m["station"]
        self.state, self.t0, self.stopped = "ACCEPTED", now, False

    def on_stop(self, m: dict, now: float) -> None:
        if self.state in ("ACCEPTED", "NAVIGATING"):
            self.state = "STOPPED"
        self.stopped = True

    def status(self, now: float) -> Optional[dict]:
        if self.silent:
            return None
        if self.state == "ACCEPTED" and now - self.t0 > 0.2:
            self.state = "NAVIGATING"
        if self.state == "NAVIGATING" and not self.never_arrive and now - self.t0 > self.travel_s:
            self.state, self.stopped, self.arrived_at = "ARRIVED", True, now
            self.station = "station_WRONG" if self.wrong_station else self.target
            self.visits.append(self.station)
        self._n = getattr(self, "_n", 0) + 1
        if self.stale_first and self.prev and self.prev[1] and self.state != "ARRIVED" and self._n % 2:
            # every other message: a stale ARRIVED for the PREVIOUS request while travelling
            return {"session": self.prev[0], "request_id": self.prev[1], "state": "ARRIVED",
                    "station": self.target, "is_stopped": True}
        stopped = self.stopped
        if self.moving_during_unload_at is not None and self.state == "ARRIVED" and \
                now - self.arrived_at >= self.moving_during_unload_at:
            stopped = False
        return {"session": self.session, "request_id": self.req, "state": self.state,
                "station": self.station if self.state == "ARRIVED" else None,
                "is_stopped": stopped, "detail": ""}


def run_offline(order, blocks=None, setup=None, quiet=True, cfg_changes=None,
                start_gap_mm=2.0, seed=0, max_s=240.0):
    cfg = load_config("simulation", blocks if blocks is not None else len(order))
    cfg.update(cfg_changes or {})
    clock = {"t": 0.0}
    nav = SimNavigationHarness()
    mech = SimulatedMechanism(cfg, order, start_gap_mm=start_gap_mm, seed=seed)
    log = Logger("[SIMULATION] ", quiet=quiet)
    core, _ = build_core("simulation", cfg, log,
                         lambda m: nav.on_destination(m, clock["t"]),
                         lambda m: nav.on_stop(m, clock["t"]),
                         clock=lambda: clock["t"], real_time=False, mech=mech)
    if setup:
        setup(core, mech, nav, cfg)
    core.startup(0.0)
    core.request_start(0.0)
    dt = float(cfg["control_period_s"])
    wrong, multi = [], []
    while clock["t"] < max_s and core.state not in (State.COMPLETE, State.FAULT):
        t = clock["t"]
        st = nav.status(t)
        if st is not None:
            core.on_nav_status(st, t)
        before = len(mech.fallen)
        core.tick(t)
        mech.update(t, dt)
        new = mech.fallen[before:]
        if len(new) > 1:
            multi.append(new)
        for c in new:     # SAFETY INVARIANT: falls only when stopped at that color's station
            if not (nav.state == "ARRIVED" and nav.station == cfg["station_ids"][c]
                    and core.state in (State.UNLOADING, State.VERIFYING_EJECTION)):
                wrong.append((c, nav.state, nav.station, core.state.value))
        if core.state == State.UNCERTAIN_DELIVERY and getattr(core, "_auto_operator", None) is not None:
            core.on_operator_ejection(core._auto_operator(mech), t)
        clock["t"] = t + dt
    core.shutdown("simulation end")
    return {"state": core.state.value, "delivered": core.delivered, "fallen": mech.fallen,
            "visits": nav.visits, "fault": core.fault_reason, "t": round(clock["t"], 1),
            "wrong": wrong, "multi": multi, "uncertain": core.uncertain_reason,
            "theta": round(mech.theta, 1)}


def offline_self_test(verbose: bool = False) -> bool:
    print("=" * 74)
    print("OFFLINE SELF-TEST -- SIMULATED motor, camera, mechanism and navigation.")
    print("These results say NOTHING about the real hardware.")
    print("=" * 74)
    S = {"red": "station_red", "green": "station_green", "blue": "station_blue"}
    cases = []

    def ok_case(name, order, stations, **kw):
        cases.append((name, order, kw, lambda r: r["state"] == "COMPLETE" and r["delivered"] == order
                      and r["fallen"] == order and r["visits"] == [S[c] for c in stations]))

    def fault_case(name, order, setup, text, **kw):
        kw["setup"] = setup
        cases.append((name, order, kw, lambda r: r["state"] == "FAULT" and text.lower() in r["fault"].lower()))

    ok_case("Order R,G,B (block visible at start)", ["red", "green", "blue"], ["red", "green", "blue"])
    ok_case("Order B,R,G", ["blue", "red", "green"], ["blue", "red", "green"])
    ok_case("Consecutive same color R,R,B", ["red", "red", "blue"], ["red", "blue"])
    ok_case("Three same color G,G,G", ["green"] * 3, ["green"])
    ok_case("Split same color R,B,R", ["red", "blue", "red"], ["red", "blue", "red"])
    ok_case("Two blocks B,G", ["blue", "green"], ["blue", "green"])
    ok_case("One block R", ["red"], ["red"])
    ok_case("No block visible at start (search first)", ["green", "red"], ["green", "red"], start_gap_mm=8.0)
    for seed in (1, 2, 3):
        def noisy(core, mech, nav, cfg):
            mech.dropout, mech.flip = 0.10, 0.05
        ok_case(f"Noisy camera (10% dropped, 5% wrong color) seed {seed}", ["red", "blue", "red"],
                ["red", "blue", "red"], setup=noisy, seed=seed)

    def stale(core, mech, nav, cfg):
        nav.stale_first = True
    ok_case("Stale arrival confirmations ignored", ["red", "blue"], ["red", "blue"], setup=stale)

    def cam_fail(core, mech, nav, cfg):
        mech.camera_fail_after = 30
    fault_case("Camera failure", ["red", "green"], cam_fail, "camera failure")

    def wrong(core, mech, nav, cfg):
        nav.wrong_station = True
    fault_case("Incorrect destination confirmation", ["red"], wrong, "incorrect destination")

    def never(core, mech, nav, cfg):
        nav.never_arrive = True
    fault_case("Navigation timeout", ["red"], never, "navigation timeout")

    def silent(core, mech, nav, cfg):
        nav.silent = True
    fault_case("Navigation not running", ["red"], silent, "start refused")

    def moving(core, mech, nav, cfg):
        nav.moving_during_unload_at = 2.0      # seconds after arrival
    fault_case("Loss of stationary confirmation", ["red", "blue"], moving, "stationary")

    def dead(core, mech, nav, cfg):
        mech.dead_motor = True
    fault_case("Motor failure (motor does not turn) -> stopped by limits", ["red", "green"], dead, "")

    def i2c(core, mech, nav, cfg):
        mech.i2c_fail_after = 6
    fault_case("Motor controller / I2C failure", ["red", "green"], i2c, "i2c")

    def jam(core, mech, nav, cfg):
        mech.jam_at_deg = 45.0
    fault_case("Mechanical jam", ["red", "green"], jam, "not moving")

    def budget(core, mech, nav, cfg):
        cfg["max_cumulative_motor_run_time_s"] = 0.25
    fault_case("Excessive motor movement (cumulative budget)", ["red", "green", "blue"], budget, "budget")

    def steps(core, mech, nav, cfg):
        cfg["max_unload_increments"] = 3
    fault_case("Excessive unloading increments", ["red"], steps, "unloading increments")

    cases.append(("Wrong count: --blocks 3, only 2 loaded (budget/limits stop it)", ["red", "blue"],
                  {"setup": None, "blocks": 3},
                  lambda r: r["state"] == "FAULT" and r["delivered"] == ["red", "blue"]))
    cases.append(("Wrong count: --blocks 1, 2 loaded", ["red", "blue"], {"blocks": 1},
                  lambda r: r["state"] == "FAULT" and "incorrect block count" in r["fault"].lower()))

    def hide_far(core, mech, nav, cfg):
        mech.hide = (19.0, 40)        # block vanishes while still far from the edge
        core._auto_operator = None
    cases.append(("Ambiguous ejection -> stop and report (no operator answer)", ["red", "green"],
                  {"setup": hide_far, "max_s": 40},
                  lambda r: r["state"] == "UNCERTAIN_DELIVERY" and r["delivered"] == []))

    def hide_then_op(core, mech, nav, cfg):
        mech.hide = (19.0, 40)
        core._auto_operator = lambda m: False
    fault_case("Ambiguous ejection, operator says NOT delivered", ["red"], hide_then_op, "not delivered")

    def fall_req(core, mech, nav, cfg):
        cfg["fall_detection_enabled"] = True
        cfg["ejection_requires_fall_evidence"] = True
    ok_case("Side-view falling motion used as ejection evidence", ["red", "green"], ["red", "green"],
            setup=fall_req)

    all_ok = True
    for name, order, kw, check in cases:
        r = run_offline(order, **kw)
        passed = check(r) and not r["wrong"] and not r["multi"]
        all_ok &= passed
        print(f"[{'PASS' if passed else 'FAIL'}] {name}")
        print(f"        state={r['state']} delivered={r['delivered']} fell={r['fallen']} "
              f"stations={r['visits']} sim_t={r['t']}s")
        if r["fault"] or r["uncertain"]:
            print(f"        reason: {r['fault'] or r['uncertain']}")
        if r["wrong"] or r["multi"]:
            print(f"        UNSAFE: wrong-place drops {r['wrong']}  multi-drops {r['multi']}")
    print("=" * 74)
    print("ALL SIMULATED TESTS PASSED (simulation only)" if all_ok else "SOME SIMULATED TESTS FAILED")
    return all_ok


# =============================================================================
# SECTION 9 -- ROS 2 NODE (thin wrapper; navigation-method independent)
# =============================================================================

def run_ros(mode: str, cfg, sim_order, session_id) -> int:
    import rclpy                                         # [VERIFIED API]
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import Bool, Empty, String         # [VERIFIED API]

    class DeliveryControllerNode(Node):
        def __init__(self):
            super().__init__("delivery_controller")
            L = self.get_logger()
            sinks = {"INFO": L.info, "WARN": L.warn, "ERROR": L.error, "DEBUG": L.debug}
            tag = "[SIMULATION] " if mode == "simulation" else ""
            self.log = Logger(tag, sink=lambda lv, t: sinks.get(lv, L.info)(t))
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
            self.pub_dest = self.create_publisher(String, cfg["topic_destination"], qos)
            self.pub_stop = self.create_publisher(String, cfg["topic_stop"], qos)
            self.pub_status = self.create_publisher(String, cfg["topic_status"], qos)
            self.core, self.mech = build_core(mode, cfg, self.log, self._json(self.pub_dest),
                                              self._json(self.pub_stop), sim_order=sim_order,
                                              session_id=session_id)
            self.create_subscription(String, cfg["topic_nav_status"], self._on_nav, qos)
            self.create_subscription(Empty, cfg["topic_start"],
                                     lambda _m: self.core.request_start(time.monotonic()), qos)
            self.create_subscription(Empty, cfg["topic_abort"],
                                     lambda _m: self.core.request_abort(time.monotonic()), qos)
            self.create_subscription(Bool, cfg["topic_operator_ejection"],
                                     lambda m: self.core.on_operator_ejection(m.data, time.monotonic()), qos)
            if cfg["use_odom_stationary_check"]:
                from nav_msgs.msg import Odometry        # [VERIFIED API]
                self.create_subscription(Odometry, cfg["odom_topic"], self._on_odom, 10)
            self.core.startup(time.monotonic())
            self.create_timer(float(cfg["control_period_s"]), self._tick)
            self.create_timer(float(cfg["status_publish_period_s"]), self._status)
            self._last_sim = time.monotonic()

        def _json(self, pub):
            def send(payload: dict) -> None:
                msg = String()
                msg.data = json.dumps(payload)
                pub.publish(msg)
            return send

        def _on_nav(self, msg) -> None:
            try:
                data = json.loads(msg.data)
                if not isinstance(data, dict):
                    raise ValueError("not a JSON object")
                for key in ("session", "request_id", "state", "is_stopped"):
                    if key not in data:
                        raise ValueError(f"missing field '{key}'")
            except ValueError as exc:
                self.log.warn(f"Ignoring invalid nav_status message: {exc}")
                return
            self.core.on_nav_status(data, time.monotonic())

        def _on_odom(self, msg) -> None:
            t = msg.twist.twist
            self.core.on_odom(math.hypot(t.linear.x, t.linear.y), t.angular.z, time.monotonic())

        def _tick(self) -> None:
            now = time.monotonic()
            if self.mech is not None:                    # advance simulated physics
                self.mech.update(now, now - self._last_sim)
                self._last_sim = now
            self.core.tick(now)

        def _status(self) -> None:
            self._json(self.pub_status)(self.core.status())

    rclpy.init()
    node, code = None, 0

    def emergency_neutral() -> None:     # runs at interpreter exit, even after errors
        if node is not None:
            try:
                node.core.driver.neutral()
            except Exception:  # noqa: BLE001
                pass
    atexit.register(emergency_neutral)

    def on_sigterm(_signum, _frame):     # e.g. `kill <pid>`: treat like Ctrl+C
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_sigterm)
    try:
        node = DeliveryControllerNode()
        rclpy.spin(node)
    except ConfigurationError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        code = 2
    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C / termination received.")
    except Exception as exc:  # noqa: BLE001 -- unexpected error: still go to neutral
        print(f"[ERROR] Unexpected error: {exc!r}", file=sys.stderr)
        code = 1
    finally:
        if node is not None:
            node.core.shutdown("program stopping")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


# =============================================================================
# SECTION 10 -- ENTRY POINT
# =============================================================================

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Piston motor + Pixy2 delivery subsystem")
    ap.add_argument("--mode", choices=["hardware", "simulation"])
    ap.add_argument("--blocks", type=int, help="number of loaded blocks (1-3)")
    ap.add_argument("--sim-order", help="SIMULATION ONLY: colors from the unloading end, e.g. red,red,blue")
    ap.add_argument("--session-id", help="fixed session id (testing with `ros2 topic pub`)")
    ap.add_argument("--offline-test", action="store_true")
    ap.add_argument("--offline-run", metavar="COLORS")
    ap.add_argument("--check-config", action="store_true")
    a, _ = ap.parse_known_args(argv)

    if a.offline_test:
        return 0 if offline_self_test() else 1
    if a.offline_run:
        order = [c.strip() for c in a.offline_run.split(",") if c.strip()]
        r = run_offline(order, blocks=a.blocks, quiet=False)
        print(f"\nSIMULATED RESULT: {r['state']} delivered={r['delivered']} stations={r['visits']}")
        return 0
    if a.check_config:
        problems = validate_config(load_config("hardware", a.blocks), "hardware")
        print("Hardware configuration and safety verifications complete." if not problems else
              "Hardware mode is DISABLED until these are resolved:\n  - " + "\n  - ".join(problems))
        return 0 if not problems else 2
    if a.mode is None:
        ap.error("choose --offline-test, --offline-run, --check-config, or --mode")
    order = None
    if a.sim_order:
        if a.mode != "simulation":
            ap.error("--sim-order only with --mode simulation")
        order = [c.strip() for c in a.sim_order.split(",") if c.strip()]
        a.blocks = a.blocks or len(order)
    cfg = load_config(a.mode, a.blocks)
    problems = validate_config(cfg, a.mode)
    if problems:
        print("[ERROR] Motor will NOT be energized. Resolve:\n  - " + "\n  - ".join(problems), file=sys.stderr)
        return 2
    return run_ros(a.mode, cfg, order, a.session_id)


if __name__ == "__main__":
    sys.exit(main())
