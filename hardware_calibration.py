#!/usr/bin/env python3
"""
hardware_calibration.py -- bench testing + calibration for the piston motor and Pixy2
====================================================================================

Run ON THE TURTLEBOT 4 RASPBERRY PI (over SSH).  Does not use ROS.  Uses the same
drivers and configuration as delivery_controller.py, so what you calibrate here
is exactly what the controller will command.

    python3 hardware_calibration.py motor  --bus 1 --address 0x40 --channel 0 --freq 50
    python3 hardware_calibration.py camera
    python3 hardware_calibration.py motor  --mock      # practice on a laptop (SIMULATED)
    python3 hardware_calibration.py camera --mock      # practice on a laptop (SIMULATED)

Command-line values override HARDWARE_CONFIG for this session only; copy the
values you settle on into HARDWARE_CONFIG in delivery_controller.py yourself.
Every measurement you type is appended to calibration_log.csv.

MOTOR SAFETY (read before running):
  * First tests with the crank/link DISCONNECTED from the piston.
  * Keep the 6 V pack connector within reach and unplug it if anything is wrong.
    The PCA9685 keeps outputting the last pulse if this program or the Pi crashes.
  * Ctrl+C at any time commands neutral.
"""

from __future__ import annotations

import argparse
import csv
import os
import statistics
import sys
import threading
import time
from typing import Dict, List, Optional

import delivery_controller as dc

LOG_FILE = "calibration_log.csv"
UTILITY_MAX_JOG_S = 1.0       # hard cap for any single jog in this utility


def log_row(kind: str, **values) -> None:
    new = not os.path.exists(LOG_FILE)
    with open(LOG_FILE, "a", newline="") as handle:
        writer = csv.writer(handle)
        if new:
            writer.writerow(["time", "kind", "data"])
        writer.writerow([time.strftime("%Y-%m-%d %H:%M:%S"), kind,
                         "; ".join(f"{k}={v}" for k, v in values.items())])


def ask_float(prompt: str) -> Optional[float]:
    text = input(prompt).strip()
    if text.lower() in ("", "x", "q"):
        return None
    try:
        return float(text)
    except ValueError:
        print("  not a number")
        return ask_float(prompt)


# =============================================================================
# MOTOR UTILITY
# =============================================================================

class MotorBench:
    def __init__(self, cfg: Dict[str, object], mock: bool, max_jog_s: float):
        self.cfg, self.mock = cfg, mock
        self.max_jog_s = min(max_jog_s, UTILITY_MAX_JOG_S)
        self.energize_ok = False
        self.running_since: Optional[float] = None
        self.lock = threading.Lock()
        self.cumulative_s = 0.0            # since the last 'zero' (operator-verified start)
        if mock:
            self.mech = dc.SimulatedMechanism(cfg, [])
            self.driver = dc.SimulatedMotorDriver(cfg, self.mech, time.monotonic)
        else:
            self.driver = dc.MC29MotorDriver(cfg)
        threading.Thread(target=self._watchdog, daemon=True).start()

    def _watchdog(self) -> None:
        while True:
            time.sleep(0.005)
            started = self.running_since
            if started is not None and time.monotonic() - started > self.max_jog_s + 0.1:
                try:
                    self.driver.neutral()
                    print("\n  WATCHDOG: jog overran -> NEUTRAL")
                except Exception:  # noqa: BLE001
                    pass

    def connect(self) -> None:
        self.driver.connect()           # writes neutral first
        tag = "SIMULATED " if self.mock else ""
        print(f"  {tag}PCA9685 connected, NEUTRAL ({self.cfg['motor_neutral_pulse_us']} us) commanded.")
        print("  Neutral is only a command. Look at the motor: it must be completely still.")

    def neutral(self) -> None:
        self.driver.neutral()

    def _sim_step(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.mech.update(time.monotonic(), 0.005)
            time.sleep(0.005)

    def jog(self, pulse: float, seconds: float, counts_toward_stroke: bool = False) -> Optional[float]:
        """Energize for `seconds`, then neutral.  Returns measured on-time."""
        if seconds <= 0 or seconds > self.max_jog_s:
            print(f"  refused: jog time must be 0 < t <= {self.max_jog_s} s")
            return None
        if not self.energize_ok:
            print("  FIRST ENERGIZE: crank/link disconnected (initial tests)? 6 V pack disconnect within reach?")
            if input("  Type ENERGIZE to allow motor motion this session: ").strip() != "ENERGIZE":
                print("  not energized")
                return None
            self.energize_ok = True
        if input(f"  Press Enter to run {pulse:.0f} us for {seconds:.3f} s (anything else cancels): ") != "":
            return None
        start = time.monotonic()
        try:
            self.driver.write_pulse_us(pulse)      # range-checked against MC29 1000-2000 us
            self.running_since = start
            if self.mock:
                self._sim_step(seconds)
            else:
                time.sleep(seconds)
        finally:
            self.driver.neutral()
            self.running_since = None
        on_time = time.monotonic() - start
        if counts_toward_stroke:
            self.cumulative_s += on_time
        if self.mock:
            self._sim_step(0.05)
            print(f"  SIMULATED piston displacement now {self.mech.piston_mm():.1f} mm "
                  f"(crank {self.mech.theta:.0f} deg -- real hardware gives NO such reading)")
        print(f"  ran {on_time:.3f} s, NEUTRAL commanded")
        return on_time


def motor_menu(bench: MotorBench) -> None:
    cfg = bench.cfg
    help_text = """
  scan                 list I2C addresses on the bus (PCA9685 must appear)
  neutral [us]         set/send the neutral pulse (default from config); check motor is still
  jog <us> <sec>       run one pulse for a short time, then neutral
  dir                  short forward jog, then short reverse jog -> note which ADVANCES the piston
  measure <us> <sec> <n>  repeat a jog n times, typing caliper readings (piston displacement)
  zero                 operator verified piston at START position -> cumulative time = 0
  stroke <us> <sec>    jogs from the start position, typing caliper readings, until you type x
                       (gives the run time vs displacement curve = slider-crank nonlinearity)
  summary              suggested config values from this session's measurements
  stop                 neutral now
  quit                 neutral and exit"""
    print(help_text)
    measurements: List[dict] = []
    stroke: List[dict] = []
    while True:
        try:
            parts = input("\nmotor> ").strip().split()
        except EOFError:
            parts = ["quit"]
        if not parts:
            continue
        cmd, args = parts[0].lower(), parts[1:]
        try:
            if cmd == "help":
                print(help_text)
            elif cmd == "scan":
                scan_i2c(cfg, bench.mock)
            elif cmd == "neutral":
                if args:
                    cfg["motor_neutral_pulse_us"] = float(args[0])
                bench.neutral()
                ans = input(f"  Neutral {cfg['motor_neutral_pulse_us']} us sent. Is the motor completely still? (y/n) ")
                log_row("neutral_check", pulse_us=cfg["motor_neutral_pulse_us"], still=ans.strip().lower())
                if ans.strip().lower() != "y":
                    print("  Try values a few us above/below. Find the lowest pulse where it creeps one way and")
                    print("  the highest where it creeps the other way; use the middle as neutral.")
            elif cmd == "jog" and len(args) == 2:
                t = bench.jog(float(args[0]), float(args[1]))
                if t:
                    log_row("jog", pulse_us=args[0], seconds=round(t, 3))
            elif cmd == "dir":
                fwd, rev = cfg.get("motor_forward_pulse_us"), cfg.get("motor_reverse_pulse_us")
                if fwd is None or rev is None:
                    neutral = float(cfg["motor_neutral_pulse_us"])
                    fwd = fwd or neutral + 100
                    rev = rev or neutral - 100
                    print(f"  (using test pulses {fwd:.0f} / {rev:.0f} us -- small drive near neutral)")
                print("  FORWARD jog:")
                bench.jog(float(fwd), 0.2)
                print("  REVERSE jog:")
                bench.jog(float(rev), 0.2)
                ans = input("  Which one moved the piston TOWARD the unloading edge? (forward/reverse) ").strip().lower()
                log_row("direction", advance_command=ans)
                print(f'  Set "motor_advance_command": "{ans}" in HARDWARE_CONFIG.')
            elif cmd == "measure" and len(args) == 3:
                pulse, secs, n = float(args[0]), float(args[1]), int(args[2])
                stage = input("  Stage label (e.g. 'early', 'middle', 'late' of the stroke): ").strip()
                for i in range(n):
                    before = ask_float(f"  [{i + 1}/{n}] caliper reading BEFORE (mm, x to stop): ")
                    if before is None:
                        break
                    t = bench.jog(pulse, secs)
                    if t is None:
                        break
                    after = ask_float("  caliper reading AFTER (mm): ")
                    if after is None:
                        break
                    row = {"pulse_us": pulse, "seconds": round(t, 3), "stage": stage,
                           "moved_mm": round(after - before, 2)}
                    measurements.append(row)
                    log_row("increment", **row)
                    print(f"  moved {after - before:.2f} mm in {t:.3f} s")
            elif cmd == "zero":
                bench.cumulative_s = 0.0
                stroke.clear()
                print("  Cumulative run time reset. Only valid if YOU verified the piston start position.")
            elif cmd == "stroke" and len(args) == 2:
                pulse, secs = float(args[0]), float(args[1])
                print("  Linkage must be CONNECTED for this test, blocks removed. Stop well before full extension.")
                start_mm = ask_float("  caliper reading at the START position (mm): ")
                if start_mm is None:
                    continue
                while True:
                    t = bench.jog(pulse, secs, counts_toward_stroke=True)
                    if t is None:
                        break
                    pos = ask_float(f"  cumulative {bench.cumulative_s:.3f} s -> caliper reading (mm, x to stop): ")
                    if pos is None:
                        break
                    row = {"pulse_us": pulse, "cumulative_s": round(bench.cumulative_s, 3),
                           "displacement_mm": round(pos - start_mm, 2)}
                    stroke.append(row)
                    log_row("stroke", **row)
                    if len(stroke) >= 2:
                        d = stroke[-1]["displacement_mm"] - stroke[-2]["displacement_mm"]
                        print(f"  this increment moved {d:.2f} mm")
                        if d <= 0:
                            print("  WARNING: no forward movement -- crank may be at/after full extension. STOP.")
            elif cmd == "summary":
                motor_summary(measurements, stroke)
            elif cmd == "stop":
                bench.neutral()
                print("  NEUTRAL commanded")
            elif cmd == "quit":
                bench.neutral()
                print("  NEUTRAL commanded. Bye.")
                return
            else:
                print("  unknown command (type help)")
        except dc.HardwareError as exc:
            print(f"  HARDWARE ERROR: {exc}")
            try:
                bench.neutral()
            except Exception:  # noqa: BLE001
                print("  !!! could not write neutral -- DISCONNECT MOTOR POWER !!!")


def motor_summary(measurements: List[dict], stroke: List[dict]) -> None:
    if measurements:
        print("  Increment repeatability (same pulse/time can move different amounts):")
        groups: Dict[tuple, List[float]] = {}
        for m in measurements:
            groups.setdefault((m["stage"], m["pulse_us"], m["seconds"]), []).append(m["moved_mm"])
        for (stage, pulse, secs), moved in groups.items():
            print(f"   stage '{stage}' {pulse:.0f} us x {secs:.3f} s: mean {statistics.mean(moved):.2f} mm, "
                  f"min {min(moved):.2f}, max {max(moved):.2f} (n={len(moved)})")
        print("  Choose unload_run_time_s so the MAX observed movement is well below the distance the")
        print("  next block can travel before it tips; search_run_time_s can be larger.")
    if stroke:
        print("  Stroke curve (cumulative run time -> displacement):")
        for row in stroke:
            print(f"   {row['cumulative_s']:.3f} s -> {row['displacement_mm']:.2f} mm")
        best = max(stroke, key=lambda r: r["displacement_mm"])
        print(f"  Largest displacement {best['displacement_mm']:.2f} mm at {best['cumulative_s']:.3f} s.")
        print("  Repeat the stroke test several times (battery level changes the curve). Set")
        print("  max_cumulative_motor_run_time_s clearly BELOW the smallest run time at which full")
        print("  extension was reached, and build run_time_stages from where increments were large/small.")
    if not measurements and not stroke:
        print("  no measurements yet")


def scan_i2c(cfg: Dict[str, object], mock: bool) -> None:
    if mock:
        print("  SIMULATED scan: [0x40]")
        return
    try:
        from adafruit_extended_bus import ExtendedI2C    # [VERIFIED API]
    except ImportError:
        print("  adafruit-extended-bus not installed; use `sudo i2cdetect -y <bus>` instead")
        return
    i2c = ExtendedI2C(int(cfg["i2c_bus"]))
    while not i2c.try_lock():                           # busio.I2C locking API
        pass
    try:
        print("  addresses:", [hex(a) for a in i2c.scan()])
    finally:
        i2c.unlock()
        i2c.deinit()


# =============================================================================
# CAMERA UTILITY
# =============================================================================

class CameraBench:
    def __init__(self, cfg: Dict[str, object], mock: bool):
        self.cfg, self.mock = cfg, mock
        if mock:
            sim = dict(cfg)
            sim.update(dc.SIMULATION_OVERRIDES)
            self.mech = dc.SimulatedMechanism(sim, ["red", "red", "blue"])
            self.mech.pulse = 1560.0                     # SIMULATED slow continuous push
            self.camera = dc.SimulatedCamera(self.mech)
            self.cfg = sim
        else:
            self.camera = dc.Pixy2UsbCamera(cfg)
        self.color_of = {s: c for c, s in self.cfg["color_signatures"].items() if s is not None}

    def connect(self) -> None:
        self.camera.connect()
        print("  " + ("SIMULATED camera" if self.mock else "Pixy2 connected over USB"))

    def frames(self, seconds: float, period: float = 0.1):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if self.mock:
                for _ in range(int(period / 0.005)):
                    self.mech.update(time.monotonic(), 0.005)
            ok, err, blocks = self.camera.read(time.monotonic())
            yield ok, err, blocks
            time.sleep(0 if self.mock else period)

    def describe(self, b: dc.RawBlock) -> str:
        return (f"sig {b.signature} ({self.color_of.get(b.signature, 'UNMAPPED')}) x={b.x} y={b.y} "
                f"w={b.width} h={b.height} area={b.width * b.height} index={b.index}")


def camera_menu(cam: CameraBench) -> None:
    help_text = """
  stream [sec]     print every detected block (signature, x, y, width, height, area, index)
  axis             push ONE block slowly toward the edge by hand -> suggests leading_axis/direction
  edge             hold a block with its front exactly at the unloading edge -> suggests edge_px
  roi [sec]        move blocks through the channel -> suggests roi_px
  sizes [sec]      block sizes at all positions -> suggests minimum width/height/area
  watch [sec]      run the controller's own detector with the CURRENT config: leading block,
                   approaching edge, handover, apparent fall (push blocks off by hand)
  quit"""
    print(help_text)
    while True:
        try:
            parts = input("\ncamera> ").strip().split()
        except EOFError:
            return
        if not parts:
            continue
        cmd = parts[0].lower()
        secs = float(parts[1]) if len(parts) > 1 else 10.0
        if cmd == "help":
            print(help_text)
        elif cmd == "stream":
            for ok, err, blocks in cam.frames(secs):
                if not ok:
                    print(f"  COMMUNICATION FAILURE: {err}")      # not the same as 'no blocks'
                elif not blocks:
                    print("  (no blocks)")
                else:
                    print("  " + " | ".join(cam.describe(b) for b in blocks))
        elif cmd == "axis":
            print("  Push ONE block slowly toward the unloading edge for 8 s ...")
            xs, ys = [], []
            for ok, _, blocks in cam.frames(8):
                if ok and blocks:
                    b = max(blocks, key=lambda b: b.width * b.height)
                    xs.append(b.x)
                    ys.append(b.y)
            if len(xs) < 5:
                print("  not enough detections")
                continue
            dx, dy = xs[-1] - xs[0], ys[-1] - ys[0]
            axis = "x" if abs(dx) >= abs(dy) else "y"
            sign = 1 if (dx if axis == "x" else dy) > 0 else -1
            print(f"  x changed {dx:+d}, y changed {dy:+d}  ->  leading_axis='{axis}', leading_direction={sign:+d}")
            log_row("camera_axis", dx=dx, dy=dy, axis=axis, direction=sign)
        elif cmd == "edge":
            input("  Hold a block with its FRONT exactly at the unloading edge, then press Enter ...")
            fronts = {"x+": [], "x-": [], "y+": [], "y-": []}
            for ok, _, blocks in cam.frames(3):
                if ok and blocks:
                    b = max(blocks, key=lambda b: b.width * b.height)
                    fronts["x+"].append(b.x + b.width / 2)
                    fronts["x-"].append(b.x - b.width / 2)
                    fronts["y+"].append(b.y + b.height / 2)
                    fronts["y-"].append(b.y - b.height / 2)
            if not fronts["x+"]:
                print("  no block seen")
                continue
            print("  Front-edge pixel coordinate, depending on your axis/direction:")
            for k, v in fronts.items():
                print(f"   {k}: {statistics.median(v):.0f}")
            print("  edge_px = the value for your leading_axis and direction (x+ = axis x, direction +1).")
            log_row("camera_edge", **{k: round(statistics.median(v)) for k, v in fronts.items()})
        elif cmd in ("roi", "sizes"):
            print("  Move blocks through the whole visible part of the channel ...")
            seen = []
            for ok, _, blocks in cam.frames(secs):
                if ok:
                    seen += blocks
            if not seen:
                print("  no blocks seen")
                continue
            if cmd == "roi":
                x0 = min(b.x for b in seen); x1 = max(b.x for b in seen)
                y0 = min(b.y for b in seen); y1 = max(b.y for b in seen)
                print(f"  block centers spanned x {x0}-{x1}, y {y0}-{y1}.")
                print(f"  Suggested roi_px (with margin, check for background objects!): "
                      f"({max(0, x0 - 10)}, {min(315, x1 + 10)}, {max(0, y0 - 10)}, {min(207, y1 + 10)})")
                log_row("camera_roi", x0=x0, x1=x1, y0=y0, y1=y1)
            else:
                full = sorted(seen, key=lambda b: b.width * b.height)
                upper = full[len(full) // 2:]          # ignore partial blocks
                w = min(b.width for b in upper); h = min(b.height for b in upper)
                a = min(b.width * b.height for b in upper)
                print(f"  smallest 'full' block seen: w={w} h={h} area={a}")
                print(f"  Suggested start: min_block_width_px={int(w * 0.7)}, min_block_height_px={int(h * 0.7)}, "
                      f"min_block_area_px={int(a * 0.6)}, entering_min_area_px={int(a * 0.1)}")
                log_row("camera_sizes", w=w, h=h, area=a)
        elif cmd == "watch":
            problems = [p for p in dc.validate_config(dict(cam.cfg, loaded_block_count=1), "simulation")
                        if any(k in p for k in ("roi", "block", "leading", "edge", "exit", "handover", "signature"))]
            if problems:
                print("  Set the camera config values first:\n   - " + "\n   - ".join(problems))
                continue
            det = dc.BlockDetector(cam.cfg)
            last_front = None
            base_fall = None
            for ok, err, blocks in cam.frames(secs, period=0.05):
                r = det.process(ok, err, blocks)
                event = det.classify(r)
                lead = r.leading
                extra = ""
                if lead is not None:
                    extra = f" leading={lead.color} front={lead.front:.0f}"
                    if last_front is not None and lead.front < last_front - float(cam.cfg["handover_jump_px"]):
                        extra += "  <-- HANDOVER (leading edge jumped back)"
                    if det.in_exit_zone(lead.front) and lead.fall_coord is not None:
                        base_fall = base_fall if base_fall is not None else lead.fall_coord
                    last_front = lead.front
                for d in r.valid + r.entering:
                    if base_fall is not None and d.fall_coord is not None and det.in_exit_zone(d.front):
                        drop = d.fall_coord - base_fall
                        if abs(drop) > 3:
                            extra += f"  downward offset {drop:+.0f}px"
                print(f"  {event.value}{extra}")
        elif cmd == "quit":
            return
        else:
            print("  unknown command (type help)")


# =============================================================================
# ENTRY POINT
# =============================================================================

def main() -> int:
    ap = argparse.ArgumentParser(description="Motor and Pixy2 calibration utility")
    ap.add_argument("device", choices=["motor", "camera"])
    ap.add_argument("--mock", action="store_true", help="SIMULATED hardware (laptop practice)")
    ap.add_argument("--bus", type=int)
    ap.add_argument("--address", type=lambda s: int(s, 0))
    ap.add_argument("--channel", type=int)
    ap.add_argument("--freq", type=float)
    ap.add_argument("--neutral", type=float, help="neutral pulse to start from (us)")
    ap.add_argument("--max-jog", type=float, default=0.3, help=f"max jog time (s), <= {UTILITY_MAX_JOG_S}")
    a = ap.parse_args()

    cfg = dict(dc.HARDWARE_CONFIG)
    if a.mock:
        cfg.update(dc.SIMULATION_OVERRIDES)
        print("=== SIMULATED HARDWARE: nothing here says anything about the real robot ===")
    for key, val in (("i2c_bus", a.bus), ("pca9685_address", a.address), ("pca9685_channel", a.channel),
                     ("pwm_frequency_hz", a.freq), ("motor_neutral_pulse_us", a.neutral)):
        if val is not None:
            cfg[key] = val

    if a.device == "motor":
        missing = [k for k in ("i2c_bus", "pca9685_address", "pca9685_channel", "pwm_frequency_hz",
                               "motor_neutral_pulse_us") if cfg.get(k) is None]
        if missing:
            print("Give these first (command line or HARDWARE_CONFIG): " + ", ".join(missing))
            print("Example: python3 hardware_calibration.py motor --bus 1 --address 0x40 --channel 0 "
                  "--freq <verified Hz> --neutral 1500")
            return 2
        bench = MotorBench(cfg, a.mock, a.max_jog)
        try:
            bench.connect()
            motor_menu(bench)
        except KeyboardInterrupt:
            print("\n  Ctrl+C")
        except dc.HardwareError as exc:
            print(f"HARDWARE ERROR: {exc}")
        finally:
            try:
                bench.neutral()
                print("  NEUTRAL commanded on exit. Unplug the 6 V pack when finished.")
            except Exception:  # noqa: BLE001
                print("  !!! neutral write failed -- DISCONNECT MOTOR POWER NOW !!!")
            bench.driver.close()
        return 0

    cam = CameraBench(cfg, a.mock)
    try:
        cam.connect()
        camera_menu(cam)
    except KeyboardInterrupt:
        print()
    except dc.HardwareError as exc:
        print(f"HARDWARE ERROR: {exc}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
