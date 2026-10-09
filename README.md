<h1 align="center">A Unified Continuum Robot Actuation Platform - Software</h1>

<p align="center">
  Open-source software to help researchers get started with high-DOF robots.<br>
  We focus on continuum robots, but the platform is agnostic to robot type!
</p>

<p align="center">
  <a href="https://ucsdmorimotolab.github.io/unified-actuation/"><b>Project website</b></a>
  &nbsp;·&nbsp;
  <b>Looking for hardware?</b> See <a href="https://github.com/UCSDMorimotoLab/unified-actuation-hardware"><b>unified-actuation-hardware</b></a>
</p>

<p align="center">
  <img src="images/overview_combined.jpg" alt="Left: system overview, an operator at the input device teleoperates the actuation assembly on a serial manipulator over a surgical simulator, driving a TDCR endoscope and a CTR instrument. Right: degrees of freedom, outer yaw, outer pitch and insertion plus actuation-assembly DOFs for a 2-segment TDCR or a 3-tube CTR" width="100%">
</p>

---

ROS 2 workspace for low-level motor actuation of continuum robots (concentric
tube robots / CTRs and tendon-driven continuum robots / TDCRs) built on Nanotec
motor controllers over EtherCAT.

The workspace contains one first-party package, **`actuation_control_python`**,
plus a vendored copy of the **`crtk_python_client`** / `crtk_msgs` (CRTK ROS
abstraction layer) that the scripts build on.

---

## Workspace layout

```
unified-actuation-software/
├── src/
│   ├── actuation_control/
│   │   └── actuation_control_python/
│   │       ├── scripts/                 # runnable ROS 2 nodes (ros2 run ...)
│   │       ├── src/actuation_control/   # importable library (motor driver + helpers)
│   │       ├── CMakeLists.txt           # ament_cmake; GLOBs scripts/*.py into lib/
│   │       └── package.xml
│   └── crtk/                            # vendored CRTK client (crtk.ral, msg conversions)
├── build/  install/  log/              # colcon output (git-ignored)
└── README.md
```

### Build & source

```bash
cd ~/unified-actuation-software
colcon build --symlink-install
source install/setup.bash
```

Every script is installed to `lib/actuation_control_python/` and is launched with:

```bash
ros2 run actuation_control_python <script>.py <CR_NAMESPACE>
```

`<CR_NAMESPACE>` is a required positional argument (e.g. `TDCR`, `CTR`). All of a
node's topics are created **relative to that namespace** via `crtk.ral`
(`ral.create_child(CR_namespace)`).

### External dependencies

| Dependency | Used for |
|---|---|
| `nanotec_nanolib` (`Nanolib`) | Nanotec controller bus access (EtherCAT), object-dictionary reads/writes |
| [`crtk_python_client`](https://github.com/collaborative-robotics/crtk_python_client) (`crtk`) | `crtk.ral` ROS abstraction layer, argv parsing, rate control (vendored in `src/crtk/`) |
| [`crtk_msgs`](https://github.com/collaborative-robotics/crtk_msgs) | CRTK ROS message/service definitions used by `crtk_python_client` (vendored in `src/crtk/`) |
| ROS 2 `rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs` | messaging |
| `PyKDL` | SE(3) frame / quaternion conversions |
| `numpy` | math |
| `inputs` | raw keyboard event capture for the teleop nodes |

---

## Topic conventions (all relative to `<CR_NAMESPACE>`)

| Topic | Type | Direction | Meaning |
|---|---|---|---|
| `setpoint_ms` | `sensor_msgs/JointState` | command | desired **absolute** motor positions (`.position`, ordered as `motorIDs`) |
| `measured_ms` | `sensor_msgs/JointState` | feedback | measured motor positions (`.position`); `.effort` unused / empty |

## Motor ID convention

Motors are addressed by string IDs mapped to EtherCAT device IDs in
`NanotecDriver.get_device_id()`:

| Prefix | Type | Unit | Control | Device IDs |
|---|---|---|---|---|
| `S1`–`S7` | stepper (lead-screw translation) | mm | open loop | 1–7 |
| `B1`–`B5` | BLDC (tube rotation) | rad | closed loop | 8–12 |

`motorIDs` is declared as a module-level list near the top of each script — edit
it there to match the motors physically wired for your configuration.

---

## `scripts/` — runnable nodes

| Script | Role | Publishes | Subscribes | Talks to HW? |
|---|---|---|---|---|
| [`abs_motor_pos_control.py`](src/actuation_control/actuation_control_python/scripts/abs_motor_pos_control.py) | Position-only low-level controller | `measured_ms` | `setpoint_ms` | yes |
| [`keyboard_command_motor_pos.py`](src/actuation_control/actuation_control_python/scripts/keyboard_command_motor_pos.py) | Per-motor keyboard teleop → setpoints | `setpoint_ms` | — | no |
| [`keyboard_command_motor_pos_pay_in_out.py`](src/actuation_control/actuation_control_python/scripts/keyboard_command_motor_pos_pay_in_out.py) | Antagonistic pay-in/pay-out keyboard teleop → setpoints | `setpoint_ms` | — | no |
| [`drive_motors_to_home.py`](src/actuation_control/actuation_control_python/scripts/drive_motors_to_home.py) | One-shot ramp back to home config | `setpoint_ms` | — | no |
| [`drive_motors_to_start.py`](src/actuation_control/actuation_control_python/scripts/drive_motors_to_start.py) | One-shot ramp into a teleop start pose | `setpoint_ms` | — | no |
| [`move_sine_command_motor_pos.py`](src/actuation_control/actuation_control_python/scripts/move_sine_command_motor_pos.py) | Sine-wave setpoint generator | `setpoint_ms` | — | no |

All share the same skeleton: module-level `motorIDs`, a class named after
the file with `__init__(self, ral, CR_namespace)` and a nested `class CR`
holding the ROS pub/sub, a `run()` loop paced by `self.sleep_rate.sleep()`, and a
`main()` using `crtk.ral.parse_argv` / `argparse` / `ral.spin_and_execute`.

### `abs_motor_pos_control.py`

Closes the position loop on each `servo_rate` cycle (currently 100 Hz), reading
the latest `setpoint_ms`, driving the motors, then republishing
`measure_motor_abs_pos()` on `measured_ms`. No tension / load-cell feedback —
pure position control. The `BLOCKING` flag picks how the motors are driven (the
file currently ships `BLOCKING = False`):

| `BLOCKING` | Nanotec mode | Behaviour |
|---|---|---|
| `True` | Profile Position (pp) | applies `setpoint − previous_setpoint` as a **relative** move via `drive_all_motors_relative_wait_between()` and **waits** for "target reached" before the next. Good for discrete moves; a continuously changing setpoint makes the motor creep. |
| `False` | Cyclic Synchronous Position (CSP) | captures the motor pose at first setpoint (`csp_base`) and every cycle streams `csp_base + setpoint` as an **absolute** target via `drive_all_motors_absolute_no_wait()` — **no waiting**. Use this to track a trajectory. |

```bash
ros2 run actuation_control_python abs_motor_pos_control.py TDCR
```

Key parameters (edit in file): `motorIDs`, `BLOCKING`, `servo_rate` (100 Hz).

> CSP normally expects a cyclic PDO / sync signal; here the target is streamed via
> SDO writes. If the drive won't engage CSP, tune the interpolation time period
> (`0x60C2`, set in `motor_helper.setup_motor`) or switch that block to
> interpolated position mode (`0x6060` = 7).

### `keyboard_command_motor_pos.py`

Keyboard teleop that integrates key holds into an internal absolute setpoint
vector and publishes it on `setpoint_ms` (~10 Hz). Pair it with
`abs_motor_pos_control.py` on the same namespace. Does **not** touch hardware.

| Key (hold) | Action |
|---|---|
| `0`–`5` | select motor by index into `motorIDs` |
| `w` / `s` | increment / decrement the selected motor's setpoint |
| `z` | slow mode (0.1×) while held |

Step per cycle: stepper (`S*`) motors move `step_size` mm (default 0.2);
BLDC (`B*`) motors move `bldc_scaled_step` rad (a scaled-down `step_size`).

```bash
ros2 run actuation_control_python keyboard_command_motor_pos.py TDCR
# in another terminal:
ros2 run actuation_control_python abs_motor_pos_control.py TDCR
```

### `keyboard_command_motor_pos_pay_in_out.py`

Same as `keyboard_command_motor_pos.py` but a key drives an **antagonistic
(pay-in / pay-out) pair**: one motor is paid in, its partner paid out by the same
amount. Integrates key holds into an absolute setpoint vector (~10 Hz) and
publishes `setpoint_ms`. Does **not** touch hardware — the motor control is done
by `abs_motor_pos_control.py` on the same namespace (with its `BLOCKING` flag).
`motorIDs` must be even-length; consecutive entries form pairs
`(0,1), (2,3), (4,5), …`.

| Key (hold) | Action |
|---|---|
| `0`, `1`, `2`, … | select pair (`key i` → motors `2i`, `2i+1`) |
| `w` / `s` | pay motor `2i` in / out; motor `2i+1` moves the opposite way |
| `z` | slow mode (0.1×) while held |

Step per cycle: `step_size` mm for `S*` motors, `bldc_scaled_step` rad for `B*`.

```bash
ros2 run actuation_control_python keyboard_command_motor_pos_pay_in_out.py TDCR
# in another terminal:
ros2 run actuation_control_python abs_motor_pos_control.py TDCR
```

### `drive_motors_to_home.py` / `drive_motors_to_start.py`

One-shot startup routines for a CTR. Both wait for the user to press **Enter**,
then ramp an absolute setpoint vector at 5 Hz and publish it on `setpoint_ms`
(feed into `abs_motor_pos_control.py`). Once the ramp finishes they hold the
final setpoint. The translation stages are stepped in a **staggered,
collision-free order** (inner tube, then middle, then outer).

* **`drive_motors_to_home.py`** — assumes the modules/tubes are at arbitrary
  positions; ramps them **back to the front frame** (home) and rotations back to
  zero, *without* a power cycle. `cmd_values` = signed distance each axis must
  travel: `[t1, t2, t3 (mm), rot1, rot2, rot3 (deg-magnitude)]`.
* **`drive_motors_to_start.py`** — the inverse: run right after a power cycle
  from the home configuration (all zeros, tube curvatures aligned in −y) and
  extend the translation stages **outward** into a teleop start pose. Rotation is
  homed manually.

```bash
ros2 run actuation_control_python drive_motors_to_home.py  CTR
ros2 run actuation_control_python drive_motors_to_start.py CTR
```

Key parameters (edit in file): `motorIDs`, `cmd_values`, `servo_rate` (5 Hz),
`stepper_step` (0.2 mm), `bldc_step` (0.017453 rad ≈ 1°).

### `move_sine_command_motor_pos.py`

Setpoint generator that sweeps one or more motors through a sine wave and
publishes it on `setpoint_ms` (~50 Hz). Does not touch hardware — pair it with
`abs_motor_pos_control.py` on the same namespace. Waits for **Enter** before
starting.

The published waveform is **zero-mean**:

```
setpoint[i] = envelope(t) · amplitudes[i] · sin(2π·frequencies[i]·t + phases_deg[i])
```

`abs_motor_pos_control` integrates the difference between successive `setpoint_ms`
messages as a **relative** move, so the sweep runs **about whatever position the
motors are in when it starts** — it does not jump to an absolute home first. The
`RAMP_S` envelope forces the waveform to start and end at exactly 0, so the
motors return to their starting position. When the sweep finishes the node prints
`sine_completed` and keeps publishing 0.

Per-motor parameters (parallel lists, one entry per `motorIDs`, edit in file):

| Parameter | Meaning |
|---|---|
| `amplitudes` | peak amplitude — **mm** for `S*` (translation), **rad** for `B*` (rotation); `0` holds that motor |
| `frequencies` | Hz |
| `phases_deg` | phase offset; set a pair to `0` / `180` for antagonistic pay-in / pay-out |
| `DURATION_S` | total sweep time |
| `RAMP_S` | raised-cosine ease-in/ease-out so amplitude starts and ends at 0 (no jump) |

After `DURATION_S` it prints `sine_completed` and keeps publishing 0 (motors hold
at their sweep-start position).

```bash
ros2 run actuation_control_python move_sine_command_motor_pos.py TDCR
ros2 run actuation_control_python abs_motor_pos_control.py       TDCR
```

> Note: `drive_motors_to_home` iterates `range(len(motorIDs) - 3)` in its
> translation loop while `drive_motors_to_start` uses `range(len(motorIDs) - 4)`
> — carried over from the original scripts; reconcile if your `motorIDs` length
> changes.

---

## `src/actuation_control/` — library

Importable as `from actuation_control import …`. Public API re-exported in
[`__init__.py`](src/actuation_control/actuation_control_python/src/actuation_control/__init__.py):
`MotorHelper`, `NanotecDriver`, `SwitchHelper`, everything in `nanolib_helper`,
`binary_operations`, `ros2_message_conversion`, and `PyKDL_helper`.

### `nanolib_driver_code.py` — `NanotecDriver`

Top-level orchestrator used by the scripts. Construct with a list of motor IDs;
it connects the bus and instantiates one `MotorHelper` per motor.

| Method | Description |
|---|---|
| `__init__(motorIDs, blocking=True)` | runs `setup_controller()` then `setup_motor()`; `blocking` picks Profile Position (`True`) vs CSP (`False`) operating mode |
| `setup_controller()` | lists bus hardware, auto-selects the wired EtherCAT (Ethernet) adapter (5 s timeout, then interactive prompt to pick), opens it |
| `setup_motor()` | for each ID, infers type from prefix and creates a `MotorHelper` (`stepper`/`mm`/`open_loop` or `bldc`/`rad`/`closed_loop`), in `position` or `csp` operating mode per `blocking` |
| `get_motor_type(motorID)` | `'stepper'` if ID contains `S`, `'bldc'` if `B` |
| `get_device_id(motorID)` | ID→EtherCAT device number lookup (`S1..S7`→1..7, `B1..B5`→8..12) |
| `drive_all_motors_relative_wait_between(curr_setpoint_jp)` | **blocking**: relative move of every motor, waiting for each to settle before the next |
| `drive_all_motors_absolute_no_wait(abs_targets)` | **non-blocking (CSP)**: stream one absolute target per motor to `0x607A`, return immediately |
| `measure_motor_abs_pos()` | list of measured positions (mm / rad), sign-flipped for the ROS message |
| `measure_motor_abs_pos_raw()` | same without the sign flip — matches the `0x607A` target frame, used to seed CSP |
| `measure_motor_current()` | list of measured currents (mA) |
| `home_motors()` / `disconnect_controller()` | thin wrappers (currently stubs — see `MotorHelper`) |

### `motor_helper.py` — `MotorHelper`

One instance per physical motor; also keeps class-level registries
`motor_registry` (by motor ID) and `id_registry` (by device ID) so the
class-methods can act on many motors at once.

**Per-instance**

| Method | Description |
|---|---|
| `__init__(nanolib_helper, bus_hw_id, motor_id, device_id, motor_type, unit, operating_mode, control_mode)` | registers, connects, and configures the motor |
| `connect_motor()` / `disconnect_motor()` | scan bus, match `device_id`, open/close the device + object dictionary |
| `setup_motor()` | writes the full object-dictionary configuration: SI units (`0x60A8`), open/closed loop (`0x3202`), motor type, peak/rated/idle currents, feed constant (`0x6092`, 2 mm/rev) or gear ratio (`0x6091`, 190:1), pole pairs (`0x2030`), polarity (`0x607E`), velocity (`0x6081`), accel/decel ramps (`0x60C5/6`, `0x6083/4`), position window + window time (`0x6067/8`), positioning option code (`0x60F2`). Then, per `operating_mode`: `position` → mode `0x6060`=1 + relative control-word bit; `csp` → mode `0x6060`=8 + interpolation time period `0x60C2` |
| `enable_motor()` / `reset_motor()` | walk the CiA-402 state machine to *operation enabled* / *switched on disabled*; on enable, `0x607A` is seeded to 0 (pp) or to the current actual position (csp) so nothing jumps |
| `get_motor_pos()` | read `0x6064`, `uint32→int32`, scale ×0.001 → mm or rad |
| `get_motor_fault()` | `0` or the Nanotec error number from `0x1003:01` |
| `get_motor_current()` | read `0x2039:05`, `uint32→int32` → mA |
| `move_motor_relative_and_wait(target_value)` | **pp / blocking**: write target (`0x607A`), toggle new-setpoint bit of the control word (rising edge), block until *target reached* (`0x6041` bit 10), then check the fault register and raise on a non-zero error |
| `set_target_position_no_wait(target_value)` | **csp / non-blocking**: write an absolute target to `0x607A` and return (no handshake, no fault check) |

**Class methods** (operate over a list of motor IDs, in order)

| Method | Description |
|---|---|
| `move_all_motors_relative_wait_between(motor_ids, target_positions)` | blocking relative move of each motor, waiting for each to settle before the next (serial; ~0.02 s/motor) |
| `set_all_target_positions_no_wait(motor_ids, target_positions)` | non-blocking absolute position stream for CSP mode |
| `get_motors_pos(motor_ids)` | list of positions (**sign negated** to match command convention) |
| `get_motors_pos_raw(motor_ids)` | same, without the sign flip (CSP-target frame) |
| `get_motors_current(motor_ids)` | list of currents (mA) |

### `nanolib_helper.py` — `NanolibHelper` (+ `ScanBusCallback`)

Thin Python wrapper over the Nanotec `Nanolib` accessor. `setup()` must be
called first.

| Method | Description |
|---|---|
| `setup()` | create/store the `NanoLibAccessor` |
| `get_bus_hardware()` | list available (supported) bus hardware IDs |
| `create_bus_hardware_options(bus_hw_id)` | build open options (CANopen baud, Modbus RTU serial params, else none) |
| `open_bus_hardware(id, options)` / `close_bus_hardware(id)` | open / close the bus |
| `scan_bus(bus_hw_id)` | scan and return device IDs (progress printed via `ScanBusCallback`) |
| `create_device(device_id)` / `connect_device(h)` / `disconnect_device(h)` | device handle lifecycle |
| `read_number(h, od)` / `read_number_od(od_dict, od)` | read an OD entry as an int (by handle or by assigned object dictionary) |
| `write_number(h, val, od, bit_length)` / `write_number_od(od_dict, val, od)` | write an OD entry |
| `read_array(h, od)` / `read_string(h, od)` / `read_string_od(od_dict, od)` | read array / string entries |
| `get_device_object_dictionary(h)` / `get_object_entry(od_dict, idx)` / `get_object(od_dict, od)` | object-dictionary accessors |
| `set_logging_level(level)` | `Nanolib.LogLevel_*` |
| `get_profinet_dcp_interface()` | PROFINET DCP interface handle |
| `create_error_message(fn, h, od, err)` | format a descriptive error string |

### `switch_helper.py` — `SwitchHelper`

Configures a negative/positive limit switch on an already-connected motor
controller (looked up by device ID in `MotorHelper.id_registry`).

| Method | Description |
|---|---|
| `connect_switch()` | resolve the owning motor's object dictionary / nanolib handle by `device_id` |
| `setup_switch()` | set pins 39/41 to digital input (`0x3272`), enable input routing (`0x3240:08`), route pin 41 → positive and pin 39 → negative limit-switch bits of `0x60FD` (`0x3242`), set limit-switch error option to quick-stop (`0x3701`), persist to non-volatile memory (`0x1010:03`) |

> `switch_helper` still references a `get_controller_object()` accessor that is
> not currently defined on `MotorHelper` — treat it as partially stale.

### `ros2_message_conversion.py`

NumPy / PyKDL → ROS 2 message helpers (all take `ral` for the timestamp).

| Function | Returns |
|---|---|
| `convert_np_array_to_JointPosition(ral, list, joint_names)` | `JointState` with `.position` only |
| `convert_np_array_to_JointState(ral, pos_list, effort_list, joint_names)` | `JointState` with `.position` + `.effort` |
| `convert_np_pos_to_PointStamped(ral, pos)` | `PointStamped` from `[x,y,z]` |
| `convert_float_to_PointStamped(ral, num)` | `PointStamped` with `x=num` |
| `convert_np_tf_to_PoseStamped(ral, T, frame_id="world")` | `PoseStamped` from a 4×4 transform |
| `convert_PoseStamped_to_PyKDL(msg)` | `PyKDL.Frame` |
| `convert_np_tf_to_TransformStamped(ral, T, frame_name)` | `TransformStamped` |
| `convert_np_tf_to_PoseArray(ral, T_stack)` | `PoseArray` from a `(4,4,N)` stack |
| `convert_np_array_to_PointCloud2(ral, np_matrix)` | `PointCloud2` from an `m×3` array |

### `PyKDL_helper.py`

| Function | Description |
|---|---|
| `convert_np_tf_to_PyKDL_frame(T)` | 4×4 NumPy transform → `PyKDL.Frame` |
| `convert_PyKDL_frame_to_np_tf(frame)` | `PyKDL.Frame` → 4×4 NumPy transform |

### `binary_operations.py`

| Function | Description |
|---|---|
| `uint32_to_int32(x)` | reinterpret a 32-bit unsigned int as signed |
| `check_bit(num, bit)` | `1` if `bit` (0 = LSB) is set, else `0` |

### `helper_var.py`

Standalone kinematics constants and trajectory loaders for the antagonistic /
CTR / tendon-driven-wrist configurations (not re-exported by `__init__.py`).

Constants: `ins_init`, `prox_tend_length_init`, `distal_tend_length_init`,
`q_init`, `max_tendon_command_prox/dist`, `mm_to_blcd_cmd`,
`mm_to_endo_insertion_stepper_mm`.

| Function | Description |
|---|---|
| `relative_joint_target_to_motor_targets(rel_joint_cmd)` | relative joint cmd `[insertion, prox tendon, distal tendon]` → 5 antagonistic motor targets (with tendon-command clipping) |
| `abs_joint_target_to_motor_targets(abs_joint_cmd)` | same, from absolute joint values (offset by the `*_init` home values) |
| `bound_joint_target(joint_targ)` | clip a joint target to the safe tendon range |
| `load_path_csv(file_path, tip_pos)` | read `x,z` waypoints, offset by tip position |
| `load_traj_csv(file_path, tip_pos, tip_orien)` | read `x,z,T` trajectory, offset by tip pose |
| `load_traj_and_q_csv(file_path, tip_pos, tip_orien)` | read `x,z,T,q0,q1,q2` trajectory + joint values |

---

## Hardware & firmware reference

### Nanotec Plug & Drive Studio (Windows GUI)

On ML laptop #1 (Plug & Drive Studio + Npcap installed):

1. Open `Motor_Test_04242025.nprj` from the Plug & Drive `templates` folder.
2. Set bus type **EtherCAT**, pick the Ethernet adapter, scan & connect.
3. **Scan devices**, select the ID per the wiring chart, **Connect**.
4. Motion test: `Mi_S/B → Device Settings → Basic Setup → Motion Test`.

Always sanity-check the settings below in the GUI before running from ROS
(**some may need to be re-set when switching between GUI and ROS control**).

#### Lead-screw + stepper (translation)

* Units: 10⁻⁶ m — commanding `1000` ⇔ 1 mm (`OD 60A8`)
* Feed constant: `2000` linear feed / 1 shaft rev = 2 mm/rev (`OD 6092:01/02`)
* Basic: Stepper (`OD 3203`), pole-pair count 50 (`OD 2030`), **open loop**
  (`OD 3202`) — closed loop does not work here
* Current settings: TODO

#### BLDC (rotation)

* Units: 10⁻³ rad — commanding `100` ⇔ 0.1 rad ⇔ 5.7° (`OD 60A8`)
* Gear ratio: 190 motor rev / 1 shaft rev (19:1 gearbox × 10:1 worm) (`OD 6091:01/02`)
* Basic: BLDC (`OD 3203`), pole-pair count 1 (`OD 2030`), **closed loop** (`OD 3202`)
* Current settings: TODO

#### Negative limit switches

* Cap module translations: 1 on the back of the front frame + 1 per module
* Option code: Q-stop (`OD 3701`); set special-function enable to 0 (`OD 3240`)
  for a negative limit switch

See the Nanotec GUI and firmware manuals for details.

### Wiring

At full capacity the rig supports 12 motors (3 boards × 4 controllers). One board
is currently out of service, so only 8 motors are usable — see
`current_wiring.png`. Full diagram: `full_wiring.png`.

![current wiring](current_wiring.png)
![full wiring](full_wiring.png)

### Homing procedure (CTR, custom ML-lab firmware)

Input: the initial transmission and rotation of the tubes.

1. Drive the front module forward until the front-frame limit switch trips, then
   the middle module until the front-module switch trips, then the rear module
   until the middle-module switch trips.
2. Save each module's current transmission value as the translation "zero"
   (global variables updated throughout operation).
3. Rotate each tube in small increments until the curvatures visually align.
4. Drive rear, then middle, then front module by the initial transmission, then
   rotate the tubes by the initial rotation.

### Homing procedure (TDCR)

TODO.
