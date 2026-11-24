# Piston Motor + Pixy2 Delivery Subsystem (TurtleBot 4)

ENGR 3100 group project. This folder contains **only** the piston motor and Pixy2 subsystem, plus the ROS 2 messages it exchanges with the navigation program. It contains no navigation, driving, line following, or `/cmd_vel` code.

| File | Purpose |
|---|---|
| `delivery_controller.py` | The subsystem: configuration, VEX 393/MC29 motor control through the PCA9685, Pixy2 detection, delivery state machine, ROS 2 interface, offline simulation and self-test |
| `hardware_calibration.py` | Standalone bench utility for the motor and the Pixy2 (no ROS). Has a `--mock` mode for laptop practice |
| `README.md` | This document |

**Hardware assumed (and nothing else):** TurtleBot 4 Raspberry Pi, PCA9685 board, VEX Motor Controller 29, VEX 393 2-wire motor, side-mounted Pixy2, 6 V pack (unverified), slider-crank, channel, and red/green/blue blocks. There is no encoder, limit switch, position sensor, or break-beam sensor, so the code has no interface for any of them.

---

## 1. Limitations you must understand first

1. **The code never knows where the piston is.** The VEX 393 runs through an MC29 with no position feedback. A "50 ms increment" means *the motor was commanded for 50 ms*. The actual distance depends on battery voltage, load, friction, crank angle, and motor-to-motor variation. Cumulative run time is used **only as a conservative limit**, never as a position.
2. **Timed control cannot guarantee the crank won't over-rotate.** Past full extension, a slider-crank *retracts* the piston. The code reduces the risk in four ways:
   - a measured cumulative run-time budget, set below full extension with margin;
   - maximum increment counts;
   - a per-run time cap;
   - a camera check that faults if a visible block stops moving while the motor runs.

   None of these is a guarantee. Battery sag or a slipping linkage can still put the crank somewhere unexpected.
3. **The PCA9685 keeps outputting its last pulse on its own.** If Python crashes, the Pi freezes, or the I2C link dies while a drive pulse is active, the motor can keep turning. The code writes neutral after every increment, on every fault, on Ctrl+C, on `kill`, and at interpreter exit, and it runs an in-process watchdog thread. None of that helps if the whole process or the Pi stops. **The only protection then is a person unplugging the 6 V pack**, so the code requires you to confirm you have practiced this. No extra hardware is assumed. A hardware cutoff or watchdog would be the proper fix if your group gets one later.
4. **Neutral ≠ stopped.** Neutral (1.5 ms per VEX) only stops *commanding* rotation. The motor coasts briefly and does not hold its position. Watch it to confirm.
5. **Camera-only ejection evidence.** A block can vanish because it fell, because something covered it, or because it tipped but stayed on the platform. The code counts a delivery only when the block left the view *at the edge*, or the next block took the lead after the current block had reached the edge, and this persists over several readings. Optionally it also requires a downward (falling) motion seen from the side. Otherwise it stops in `UNCERTAIN_DELIVERY` and waits for you to inspect the channel.
6. **Same-color blocks touching each other** look like one colored shape to the Pixy2. The code tracks the shape's front edge: when the front block falls, the front edge jumps backward by about a block length (handover). This works only if the camera sees the region at and just behind the edge.
7. **The first increments may show no block motion** while the piston closes the gap to the row. Jam detection arms only after the camera has seen the piston move a block. Before that, only the run-time budget and increment limits protect the mechanism. The same applies to a motor that never turns at all: it is stopped by the limits, not recognized as a dead motor.

### Electrical items to verify (not assumed by the code)
- **The MC29 takes motor power from its 3-wire plug.** Its orange wire is power, and VEX rates it at 3 A maximum and an 8.5 V maximum battery voltage. Plugged into a PCA9685 header, it draws motor current from the PCA9685 board's V+ rail. Find out which board you have, and whether its V+ terminal, traces, and capacitor can carry the motor current at 6 V.
- **Stall current can exceed the MC29's rating.** VEX's 393 documentation lists stall currents of 3.6 A and 4.8 A in different places (at 7.2 V), both above the MC29's 3 A limit. A jam can therefore overload the MC29, so keep runs short.
- **Signal level.** The PCA9685's output level follows its VCC, usually 3.3 V from the Pi. VEX does not document the MC29's input threshold. Verify on the bench that the MC29 responds reliably.
- **Grounds.** The 6 V pack, PCA9685, and Pi need a common ground. Check polarity before connecting the pack.

---

## 2. How the motor control works

`MotorIncrementController` (Section 4 of `delivery_controller.py`) runs one increment at a time:

1. **Drive:** writes the advance pulse (`motor_forward_pulse_us` or `motor_reverse_pulse_us`, chosen by `motor_advance_command`) and records a deadline.
2. **Neutral:** the 10 ms control timer writes `motor_neutral_pulse_us` as soon as the deadline passes. ROS callbacks are never blocked.
3. **Settle:** no camera decision is made until `motor_settle_time_s` has passed.
4. **Read:** the camera is read again and the state machine decides whether another increment is needed.

Before every increment, the code refuses to drive if:
- the motor is locked (any state except searching or unloading);
- the robot is not freshly confirmed stationary;
- the run would exceed `max_single_motor_run_time_s`;
- the cumulative budget `max_cumulative_motor_run_time_s` would be exceeded;
- an increment-count limit has been reached.

Pulse widths outside the documented MC29 range (1000–2000 µs) are rejected. The controller never sends the reverse direction on its own, so the piston never retracts automatically. Reverse is used only in the calibration utility.

### Adjusting the increments (lab tuning)
All settings are at the top of `delivery_controller.py`, in `HARDWARE_CONFIG`:

| You want to change… | Edit |
|---|---|
| How hard the motor drives | `motor_forward_pulse_us` / `motor_reverse_pulse_us`: closer to neutral = weaker, closer to 2000/1000 = stronger |
| Which way advances the piston | `motor_advance_command` = `"forward"` or `"reverse"` |
| Increment size while searching | `search_run_time_s` |
| Increment size while unloading (near the edge) | `unload_run_time_s` (smaller) |
| Pause between moves | `motor_settle_time_s` |
| Longest single run | `max_single_motor_run_time_s` |
| Total movement allowed per mission | `max_cumulative_motor_run_time_s` |
| Maximum number of moves | `max_search_increments`, `max_unload_increments`, `max_total_increments` |
| PCA9685 connection | `i2c_bus`, `pca9685_address`, `pca9685_channel`, `pwm_frequency_hz` |

**Slider-crank nonlinearity.** The same run time moves the piston little near the ends of the stroke and a lot in the middle. Two tools handle this:
- `run_time_stages` lets you set different search and unload run times for different *estimated* stages of the stroke. The stage is selected by cumulative run time since the operator-verified start. Measure the stages with `hardware_calibration.py motor` → `stroke` and `measure`.
- The camera watches how far a visible block moved after each increment. If it moved more than `max_advance_px_per_increment`, later increments of that kind are shortened by `increment_shrink_factor`.

---

## 3. How the Pixy2 code works (side-facing camera)

- **Driver:** `Pixy2UsbCamera` uses the official libpixyusb2 SWIG module (`pixy.init`, `pixy.change_prog("color_connected_components")`, `pixy.ccc_get_blocks`, `pixy.BlockArray`). Pixy2's onboard color-signature recognition does the detection; no OpenCV is involved.
- **Communication failure ≠ empty platform.** A read that raises, returns a negative count, or fails the periodic frame-width check is a communication error. Repeated errors, or no good read for `camera_comm_timeout_s`, cause a fault. A good read with no blocks is "empty," and the code needs `empty_confirmation_frames` of them in a row before advancing. This also covers the undocumented case where a non-blocking read returns 0 because no new frame was ready.
- **Classification** (`BlockDetector.classify`): no block / block entering view (partial, below the minimum size) / valid block / multiple blocks / approaching the edge (in the exit zone). Events: the leading edge jumping back (**handover**: the next block now leads), and the block leaving the view at the edge (apparent fall).
- **Leading block** = the valid block whose front edge is **closest to the unloading edge**, never the largest. `leading_axis` and `leading_direction` define "toward the edge" in image coordinates.
- **Color confirmation:** `color_confirmation_frames` consecutive readings must agree. If no stable color is reached after `color_max_attempt_frames` readings, the result is an "uncertain identification" fault.
- **Progress:** the code compares the front and the rear edge before and after each increment. The rear edge matters because the front edge stops changing once the block hangs past the camera's view.
- **Side-view fall detection (optional):** a falling block moves *down* in the image. With `fall_detection_enabled`, the code compares the edge block's vertical coordinate (`fall_axis`, `fall_direction`) with its resting value. With `ejection_requires_fall_evidence` set, a delivery without seen downward motion becomes uncertain. Enable it only after `hardware_calibration.py camera` → `watch` shows the motion reliably. Pixy2 is read at about 20 Hz, and a fall may last only a couple of frames.

### Configuring and calibrating the side camera
1. **PixyMon v2 (Windows):** connect the Pixy2 to the laptop with USB. Teach signature 1 = red, 2 = green, 3 = blue (any 1–7 works) on the *side faces* of the real blocks, under the lighting the robot will actually see. Make sure the channel walls, the robot, and the background create no blocks. Save the signatures to the camera. They are stored on the Pixy2, so they travel with it to the Pi.
2. Put the numbers in `color_signatures`.
3. On the Pi, run `python3 hardware_calibration.py camera`:
   - `stream`: check signatures, x, y, width, height, and area live.
   - `axis`: push one block toward the edge by hand → sets `leading_axis` and `leading_direction`.
   - `edge`: hold a block's front exactly at the edge → sets `edge_px`.
   - `roi`: move blocks through the whole channel → sets `roi_px`; tighten it to exclude the background.
   - `sizes`: → `min_block_width_px`, `min_block_height_px`, `min_block_area_px`, `entering_min_area_px`.
   - Set `exit_zone_margin_px` to about ¼ of a block length in pixels, and `handover_jump_px` to about ½ of a block length in pixels.
   - `watch`: runs the controller's own detector with your settings. Push blocks through and off by hand to check approaching-edge, handover, and downward-offset readings, then set `fall_*` if you use fall detection.

---

## 4. Navigation interface (what your teammate integrates)

All `/delivery/*` names are proposed project topics, not TurtleBot 4 topics. JSON travels in `std_msgs/msg/String`.

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `/delivery/destination` | String (JSON) | delivery → nav | `{"session","request_id","color","station"}`, republished every 0.5 s while active |
| `/delivery/nav_status` | String (JSON) | nav → delivery | `{"session","request_id","state","station","is_stopped","detail"}`. **Publish at least every 0.2 s (≥5 Hz)**; status older than 1.0 s counts as lost |
| `/delivery/stop` | String (JSON) | delivery → nav | on fault, completion, or shutdown (every 1 s while faulted) |
| `/delivery/status` | String (JSON) | delivery → anyone | 2 Hz subsystem status (navigation can use it as a watchdog) |
| `/delivery/start`, `/delivery/abort` | `std_msgs/Empty` | operator → delivery | start = "I verified the piston start position" |
| `/delivery/operator_ejection_confirm` | `std_msgs/Bool` | operator → delivery | answers an `UNCERTAIN_DELIVERY` (true = block is off the platform) |
| `/odom` (verify name) | `nav_msgs/Odometry` | robot → delivery | optional independent stationary check (`use_odom_stationary_check`) |

Navigation `state` values: `IDLE`, `ACCEPTED`, `NAVIGATING`, `ARRIVED`, `FAILED`, `STOPPED`.

```json
{"session": "3f9a1c2e", "request_id": 2, "color": "blue", "station": "station_blue"}
{"session": "3f9a1c2e", "request_id": 2, "state": "ARRIVED", "station": "station_blue", "is_stopped": true, "detail": ""}
```

**What this subsystem accepts.** Unloading starts only when all of these hold:
1. The `session` and `request_id` match the current request. Anything else is stale or duplicate and is ignored.
2. `state` is `ARRIVED` and `station` equals the requested station. A different station is an "incorrect destination confirmation" fault.
3. `is_stopped` is `true`.
4. Odometry is still, if that check is enabled.
5. All of the above have held continuously for `arrival_settle_time_s`.

The same confirmation must stay fresh **during** unloading. If it is lost while the motor runs, the motor goes to neutral immediately and the subsystem faults. Other failures:
- No `ACCEPTED` within `nav_ack_timeout_s` → fault.
- No arrival within `nav_timeout_s` → fault and stop message.
- Status lost while travelling → fault.
- `FAILED` → fault.

**What the teammate's program must do:**
- Subscribe to `/delivery/destination` and treat (`session`, `request_id`) as the identity of a request: ignore repeats, and ignore lower IDs within the same session.
- Publish `/delivery/nav_status` at ≥5 Hz at all times.
- Report `ARRIVED` with the station ID only when actually at that station and stopped.
- Set `is_stopped` honestly.
- Stop on `/delivery/stop`.

The navigation method itself (line following or anything else) does not matter to this subsystem.

**Changes from the previous version:**
- `navigation_interface_example.py` has been removed; this section is now the specification.
- `/delivery/destination` gained a `color` field.
- The status freshness limit is now 1.0 s.
- `/delivery/operator_ejection_confirm` is new.

All other topic names and fields are unchanged.

---

## 5. Installing software

**Windows:** Python 3 (for `--offline-test` and `--mock`) and PixyMon v2. Nothing else.

**WSL (Ubuntu 24.04, ROS 2 Jazzy):** ROS 2 is already installed. Check with:
```bash
source /opt/ros/jazzy/setup.bash && ros2 interface show std_msgs/msg/Bool
```
WSL needs no hardware libraries.

**TurtleBot 4 Raspberry Pi (over SSH).** First check what it runs: `lsb_release -a`, then `echo $ROS_DISTRO`. Do not assume Jazzy. Installing a library on the laptop does not install it on the Pi.

```bash
# Pixy2 Python module (official build script)
sudo apt update
sudo apt install swig libusb-1.0-0-dev g++ git build-essential python3-dev
python --version || sudo apt install python-is-python3      # the build script calls `python`
cd ~ && git clone https://github.com/charmedlabs/pixy2.git
cd pixy2/scripts && ./build_python_demos.sh
cd ../build/python_demos && python3 get_blocks_python_demo.py   # verify: blocks print
cp pixy.py _pixy*.so ~/TurtleBot4_Project/                      # next to delivery_controller.py (add to .gitignore)

# I2C tools + PCA9685 library
sudo apt install i2c-tools
sudo usermod -aG i2c $USER            # log out and back in afterwards
# Ubuntu 22.04 (Humble image):
python3 -m pip install --user adafruit-circuitpython-pca9685 adafruit-extended-bus
# Ubuntu 24.04 (Jazzy image) blocks plain pip installs (PEP 668):
python3 -m pip install --user --break-system-packages adafruit-circuitpython-pca9685 adafruit-extended-bus
# verify
python3 -c "import adafruit_pca9685, adafruit_extended_bus; print('PCA9685 libraries OK')"
```
If the Pixy2 only works with `sudo`, add a udev rule rather than running as root. Create `/etc/udev/rules.d/99-pixy2.rules` containing `SUBSYSTEM=="usb", ATTR{idVendor}=="<VID from lsusb>", MODE="0666"`, then run `sudo udevadm control --reload-rules` and replug the camera.

---

## 6. Testing (in this order)

**A. Offline (any computer, no hardware):**
```bash
python3 delivery_controller.py --offline-test        # all simulated scenarios
python3 delivery_controller.py --offline-run red,red,blue   # one mission with the full log
python3 hardware_calibration.py motor --mock          # practice the motor menu
python3 hardware_calibration.py camera --mock         # practice the camera menu
```
Expected: `ALL SIMULATED TESTS PASSED (simulation only)` for 27 scenarios:
- orders R-G-B and B-R-G;
- R-R-B, G-G-G, R-B-R;
- 1, 2, and 3 blocks;
- a block visible at start, and no block visible at start;
- a noisy camera, run with three seeds;
- stale arrival messages;
- camera failure;
- incorrect destination;
- navigation timeout, and navigation not running;
- loss of stationary confirmation;
- a dead motor, an I2C failure, and a mechanical jam;
- the cumulative budget and the unloading-increment limit;
- two wrong-count cases;
- ambiguous ejection, and an operator answering "not delivered";
- fall-evidence mode.

The harness also checks that every simulated block fell only while the simulated robot was stopped at that block's station, and never two blocks in one increment. **These are simulation results only.**

**B. ROS 2 in WSL (still simulated hardware):**
```bash
python3 delivery_controller.py --mode simulation --sim-order red,blue --session-id test
```
Then, in a second terminal, play the navigation side by hand: `ros2 topic pub -r 10 /delivery/nav_status std_msgs/msg/String "{data: '{\"session\":\"test\",\"request_id\":0,\"state\":\"IDLE\",\"station\":null,\"is_stopped\":true}'}"`. Publish `/delivery/start`, watch `ros2 topic echo /delivery/destination`, and then change the published status to `request_id` 1, `ARRIVED`, `station_red`.

**C. Physical bench tests (TurtleBot 4 Pi):**
1. **Electrical check (power off):** pack voltage measured, polarity, common ground, MC29 orange/black/white wiring to the PCA9685 header, V+ rating of your board.
2. **PCA9685:** `sudo i2cdetect -y <bus>` shows the address. In `hardware_calibration.py motor --bus N --address 0x.. --channel C --freq F --neutral 1500`, run `scan`.
3. **Neutral (linkage disconnected):** run `neutral`. The motor must be completely still. Adjust by a few µs, and find where it creeps each way; use the middle.
4. **Direction and drive level (linkage disconnected):** `dir`, then `jog <us> <sec>` with short times. Raise the drive slowly, and confirm `stop` and Ctrl+C give neutral.
5. **Practice the manual cutoff:** with the motor running a jog, unplug the pack.
6. **Linkage connected, no blocks:** `zero` at the verified start, then `stroke <us> <sec>` with calipers until clearly before full extension. Repeat at least 3 times, including with a partly discharged pack → `max_cumulative_motor_run_time_s` (below the smallest time that reached full extension) and `run_time_stages`.
7. **Increment sizes with blocks:** `measure <us> <sec> <n>` near the edge → choose `unload_run_time_s` so the largest observed move is well under the distance the next block can travel before tipping. Choose `search_run_time_s` similarly.
8. **Camera:** run the Part 3 procedure, then `watch` while pushing blocks off by hand.
9. Fill in every PLACEHOLDER and set each `SAFETY_VERIFICATIONS` item to `True` only after you have done it. Then check: `python3 delivery_controller.py --check-config --blocks 3`.
10. **Integrated test, robot stationary:** run hardware mode, publish `nav_status` by hand (as in B), and deliver one block. Then do 2–3 blocks with same and different colors. Only after that, run with the real navigation program.

---

## 7. Running on the TurtleBot 4

1. Connect the laptop to the robot's network.
2. `ssh <user>@<robot-ip>`. Default username `ubuntu` on standard images; PLACEHOLDER if your lab changed it. VS Code: **Remote-SSH → Connect to Host**.
3. Get the code: `git clone <your repo> ~/TurtleBot4_Project` the first time, then `git pull`.
4. ROS environment: `echo $ROS_DISTRO`. If empty, run `source /opt/ros/<distro>/setup.bash`. `ros2 topic list` must show the robot's topics, including the odometry topic name.
5. Terminal 1:
   ```bash
   cd ~/TurtleBot4_Project && python3 delivery_controller.py --mode hardware --blocks 3
   ```
6. The teammate starts the navigation program.
7. Check the piston start position by eye, then:
   ```bash
   ros2 topic pub --once /delivery/start std_msgs/msg/Empty "{}"
   ```
8. Monitor with `ros2 topic echo /delivery/status`. If it reports `UNCERTAIN_DELIVERY`, look at the channel and answer:
   ```bash
   ros2 topic pub --once /delivery/operator_ejection_confirm std_msgs/msg/Bool "{data: true}"
   ```
   Use `false` if the block was not delivered.
9. To stop:
   - `ros2 topic pub --once /delivery/abort std_msgs/msg/Empty "{}"`, or Ctrl+C. Both command neutral.
   - **If anything looks wrong, unplug the 6 V pack.**

Use `tmux` so an SSH drop does not kill the run. If SSH drops anyway, assume nothing and check the motor.

---

## 8. Values still requiring measurement or verification

| Group | Values | How |
|---|---|---|
| Electrical | 6 V pack voltage/condition; PCA9685 board model and V+ rating; MC29 response to 3.3 V signal; common ground | multimeter, board documentation, bench test |
| PCA9685 | `i2c_bus`, `pca9685_address`, `pca9685_channel`, `pwm_frequency_hz`, `pca9685_reference_clock_hz` | `ls /dev/i2c*`, `i2cdetect`, wiring. The MC29 listing documents pulse width, not repetition rate; verify the frequency. Oscilloscope if available |
| Motor | `motor_neutral_pulse_us`, `motor_forward_pulse_us`, `motor_reverse_pulse_us`, `motor_advance_command` | calibration utility: `neutral`, `dir`, `jog` |
| Movement | `search_run_time_s`, `unload_run_time_s`, `run_time_stages`, `min_run_time_s`, `motor_settle_time_s`, `max_single_motor_run_time_s`, `max_cumulative_motor_run_time_s`, `max_*_increments` | `stroke`, `measure` with calipers, repeated at different battery levels |
| Camera feedback | `min_progress_px_per_increment`, `max_advance_px_per_increment` | `camera watch` during slow manual pushes; px per increment from `measure` |
| Pixy2 | `color_signatures`, `roi_px`, size thresholds, `entering_min_area_px`, `leading_axis`, `leading_direction`, `edge_px`, `exit_zone_margin_px`, `handover_jump_px`, `fall_*` | PixyMon + camera utility (Part 3) |
| Mission / ROS | `loaded_block_count` (`--blocks`), `station_ids` (agree with teammate), `nav_timeout_s`, `odom_topic`, robot ROS distro, username, `ROS_DOMAIN_ID` | teammate, `ros2 topic list`, `echo $ROS_DISTRO` |
| Safety | every item in `SAFETY_VERIFICATIONS` | done by you, in person |

Values marked `PLACEHOLDER-TUNE` have starting values that are not safety-critical; tune them during testing.

## 9. What has and has not been tested
- ✅ `--offline-test`: 27 simulated scenarios pass, plus extra noisy-camera runs with five more random seeds (Python 3.13, simulated mechanism with speed variation and coasting).
- ✅ The ROS 2 node ran a full R-R-B mission in real time against a **minimal stand-in for rclpy** that I wrote for this check, with simulated hardware. That is **not** real ROS 2; run test 6B in WSL.
- ✅ `hardware_calibration.py` motor and camera menus exercised in `--mock` mode.
- ❌ Nothing has run on the TurtleBot 4, the PCA9685, the MC29, the VEX 393, or the Pixy2. `MC29MotorDriver` and `Pixy2UsbCamera` follow the documented APIs but have never executed. The simulator is idealized.

## Sources
- VEX Motor Controller 29 (pulse widths, wiring): https://www.vexrobotics.com/276-2193.html
- VEX 2-Wire Motor 393: https://www.vexrobotics.com/393-motors.html
- Adafruit PCA9685 guide (V+, VCC logic level, OE): https://learn.adafruit.com/16-channel-pwm-servo-driver
- Adafruit PCA9685 API: https://docs.circuitpython.org/projects/pca9685/en/latest/api.html
- Adafruit ExtendedI2C: https://docs.circuitpython.org/projects/extended_bus/en/stable/api.html
- Pixy2 Python module build: https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:building_libpixyusb_as_a_python_module_on_linux
- Pixy2 CCC API: https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:ccc_api
- TurtleBot 4 user interface board (power limits): https://turtlebot.github.io/turtlebot4-user-manual/electrical/pcba.html
