# Franka gamepad teleoperation

Delta, Speed and Force control for the Franka Research 3 using a generic gamepad.
The app reads the current configured TCP before starting and starts disarmed.

| Mode | Deflection commands |
| --- | --- |
| Delta | A continuously integrated TCP pose target through CartesianPose |
| Speed | Direct TCP linear/angular velocity through CartesianVelocities |
| Force | Sustained directional force/torque bias through joint torques, with damping and orthogonal pose hold |

Delta and Speed use the same configurable limits, defaulting to **20 mm/s** and
**20 degrees/s**. Force defaults to **5 N** translation and **0.5 Nm** rotation.
Combined inputs retain the respective vector magnitude limits.

**X cycles Delta → Speed → Force → Delta.** A change is accepted only while
disarmed, with zero app velocity/wrench, measured robot rest for at least 100 ms,
neutral motion controls, no gripper operation and released Y/A/B. A rejected
press is not queued. Holding X changes the mode once. The old SDK control loop
ends at rest before the new API starts. Release X and press Y separately to arm
the new mode; simultaneous X/Y cannot arm or switch modes. The console reports
the selected mode, controller readiness and rejected requests.

Measured stillness uses a 20 ms filter on signed Cartesian velocity, followed by
a 100 ms quiet window. Instantaneous joint velocity, gross TCP motion and
commanded velocity are checked separately. This avoids repeatedly resetting
readiness on stationary FR3 velocity-estimator noise. Rejected arm requests name
the blocking condition; diagnostics include raw/filtered velocity, quiet-window
duration and arming refusal bits.

The left stick and L2/R2 act in the **robot base frame**. All rotations
use the **TCP's local axes**, and D-pad up/down adds translation along the TCP's
local +Z/−Z axis. Rotations change orientation about the TCP origin without
changing its position. Local Z follows the TCP orientation as it rotates. Base
and local translation inputs combine under the same translation vector limit.
In Force mode these same inputs produce force/torque components instead of
velocity: base XYZ force, TCP-local rotational torque, and D-pad TCP-local Z force.
There is no application workspace box.

| Control | Action |
| --- | --- |
| Left stick up/down | Base +X / −X |
| Left stick right/left | Base −Y / +Y |
| L2 / R2 | Base +Z / −Z |
| D-pad up/down | TCP-local +Z / −Z translation |
| Right stick right/left | Positive / negative rotation about TCP X |
| Right stick up/down | Positive / negative rotation about TCP Y |
| R1 / L1 | Positive / negative rotation about TCP Z |
| Y | Toggle arm/disarm |
| X | Cycle mode while disarmed and still |
| A | Close until contact using the hand app's 30 N grasp parameters |
| B | Open to the calibrated Hand's maximum width |

The app starts disarmed. Release the motion controls and A/B before pressing Y
to arm. Y again decelerates to a hold. Holding a button does not repeat actions.
A/B work only while armed. The arm decelerates and waits for measured and
commanded rest before a gripper operation; it holds during that operation.
Opening takes priority if A and B are pressed together. There is no automatic
lift. Disarming stops an in-flight gripper command without automatically opening.

## Run

Place this repository at `~/workspace/franka/ws/src/franka_gamepad_teleop`.
From `~/workspace/franka`:

```bash
# Build only this app.
./franka exec bash -c 'cd /workspace/ws && colcon build --symlink-install --packages-select franka_gamepad_teleop --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON'

# Read the pad only; no robot connection.
./franka gamepad --inspect

# Integrate a synthetic starting TCP; no robot connection.
./franka gamepad --simulate

# Print the current physical TCP and exit.
./franka gamepad --read-state

# Preview target changes while reading the robot; no motion commands (default).
./franka gamepad --preview

# Real control starts disarmed. Y enables output; X changes mode at rest.
./franka gamepad --execute
```

The robot address defaults to the workspace's `config/robot.env`; override with
`--robot-ip ADDRESS`. Use `--duration 10` for a timed session, Ctrl+C to stop,
and `--dry-run` to print the command without starting it. The launcher also runs
directly: `python3 ws/src/franka_gamepad_teleop/tools/run.py --inspect`.

Delta and Speed use libfranka Cartesian pose/velocity motion generation and
Cartesian impedance control. Zero speed holds the motion generator's endpoint.
Delta's first command uses `O_T_EE_c` for continuity; returning from external
torque control captures the measured pose. The measured starting TCP printed
to the console is `O_T_EE`: the existing configured EE, including its flange-to-EE
transform. The app does not redefine the TCP or change collision thresholds,
robot safety functions, built-in impedance parameters, joint limits or tool/load settings.
Run only one arm/gripper control application at a time.

## Configuration

Edit [`config/gamepad.yaml`](config/gamepad.yaml), then restart. Select another
file inside the workspace with `--config PATH`. Validate YAML without either
device using `./franka gamepad --validate-config`.

Each motion component contains a list of `{source, scale}` entries. Axis names
are `left_x`, `left_y`, `right_x`, `right_y`, `left_trigger`, `right_trigger`.
Button names are `a`, `b`, `x`, `y`, `back`, `guide`, `start`, `left_stick`,
`right_stick`, `left_shoulder`, `right_shoulder`, and
`dpad_up`, `dpad_down`, `dpad_left`, `dpad_right`.
For example, changing `translation_y` scale from `-1.0` to `1.0` reverses Y travel.
`translation_x/y/z` refer to the base frame; `rotation_x/y/z` refer to the TCP
frame. `translation_tcp_z` sets the additional local-Z bindings, defaulting to
D-pad up (+) and down (−). Older profiles without that key disable this extra
translation input while retaining TCP-local rotation.
`mode` selects the initial `delta`, `speed` or `force` mode, and
`bindings.mode_cycle` maps the cycle button (default `x`). Action buttons must be
distinct and cannot also be motion inputs. Older profiles use X and the default
force settings when these fields are omitted.

SDL supplies standard names through its controller mapping database. The
workstation's **QIXIONG PC Gamepad** is recognized: L2 is raw axis 5, R2 axis 4,
Y raw button 4, and L1/R1 raw buttons 6/7. These indices are diagnostic only;
YAML uses standard names. `--inspect` prints the SDL mapping and mapped commands.
Use `gamepad.name_contains` to select another pad, or an empty string to match
any recognized pad. Ambiguous multiple matches or missing required axes/action
buttons are rejected.

`motion.max_translation_m_s` and `motion.max_rotation_deg_s` set the full-scale
rates. Deadzones are removed and remaining deflection is rescaled linearly.
Velocity changes are acceleration limited; libfranka's Cartesian rate limiter
and a configurable low-pass filter apply to physical commands. Releasing input
or disarming therefore takes a short deceleration to rest.

A calls `grasp(0, 0.04, 30, 0.001, 0.08)`, matching the hand app's contact-grasp
settings, and reports whether an object was confirmed. B uses the calibrated
Hand's reported `max_width`. No automatic homing runs at startup; an existing
valid calibration is required. Set `gripper.enabled: false` to run without a Hand.

## Sustained Force mode

Force mode maintains the requested directional wrench bias for as long as the
input is held. It does not regulate measured contact force with a PI loop.
Arming captures the current measured TCP pose. Deflection produces force and
moment components, ramped at the configured slew rates. The hold reference
follows measured motion along the driven force/torque direction, so a spring
back to the original arming pose cannot cancel the sustained input. Orthogonal
directions retain pose holding.

In free space the robot can continue moving while the input is held. Against an
object it continues commanding the directional wrench bias. Releasing the input
ramps that bias down, then holds near the stopping pose instead of returning to
the original arming pose. Disarming or starting a gripper operation also captures
the current pose before ramping out the bias.

`force_control` configures bias limits, slew rates, Cartesian stiffness/damping,
nullspace posture holding, FR3 joint-limit repulsion and joint torque limits/rate.
The former hardcoded force-mode validation ceilings are editable under
`force_control.ceilings`, using the same names as the operating settings. The
joint-margin range uses `joint_margin_min_rad` and `joint_margin_max_rad`.
For example, to allow a 5 Nm requested torque:

```yaml
force_control:
  max_torque_Nm: 5.0
  torque_slew_Nm_s: 5.0
  ceilings:
    max_torque_Nm: 5.0
    torque_slew_Nm_s: 10.0
```

The shipped profile lists all application ceilings. Omitted ceiling fields retain
their previous defaults for older profiles. Errors identify the operating field,
its value and the configured ceiling. Values must remain finite and positive;
the minimum joint-margin bound can be zero. These are application validation
limits; changing them does not override SDK or robot-side constraints. Restart to
load edits; no rebuild is needed after this version is installed.

Stiffness applies to the retained hold error; it does not impose a finite travel
distance along the force-driven direction. Velocity damping is at least
`max_force_N / motion.max_translation_m_s` and
`max_torque_Nm / (motion.max_rotation_deg_s * pi / 180)`.
This bounds nominal free-space speed in an ideal model. Damping reduces net
drive as velocity rises; the requested bias remains active. This is not a
certified speed limit or a guarantee of measured contact force. Actual behavior
depends on the robot, payload, contact and controller tracking.

The sum of the Cartesian bias, restoring and damping terms is vector-limited to
5 N / 0.5 Nm by default. Coriolis compensation and bounded nullspace/joint-limit
torques are additional joint-space terms; these limits are not a calibrated
contact-force guarantee. Gravity/friction compensation remains with the robot.
No application Cartesian workspace box is added. The app does not raise collision
thresholds or change robot-side safety functions.

Force uses the SDK model/Jacobian in the real-time torque callback, with fixed-size
calculations and continuous torque output. Physical Speed/Force behavior and API
handovers still need operator commissioning; offline tests do not establish
physical performance or resolve the previously observed rotational tracking error.

## Checks and failure handling

For rotation-drift diagnostics, enable optional logging:

```bash
./franka gamepad --execute --log
# Or choose a new filename inside the workspace:
./franka gamepad --execute --log logs/z-rotation.csv
```

This records a CSV and a `.csv.meta.yaml` sidecar under the workspace's `logs/`
directory. Logging does not change control settings or the integration/filtering
code. The callback copies samples into a fixed queue at up to 250 Hz; a separate
thread writes files. A full queue drops diagnostic samples without blocking
control. Normal shutdown drains the queue and records dropped-sample counts.
Existing log files are never overwritten.

Recorded values include control mode/backend/readiness, requested velocity and
wrench, actual task wrench and sent joint torques, raw/mapped inputs, target pose, measured TCP,
SDK-reported last commanded and desired poses, joints, commanded twist and
acceleration, estimated external wrench, timing and command success rate.
All poses are column-major 4×4 matrices, with base XYZ in indices 12–14 in metres.
The input and app target belong to the current callback; the reported SDK/robot
state precedes processing that new target. The sidecar preserves the YAML
settings, controller mapping, initial TCP/stiffness frame transforms and tool/load
mass/centre-of-mass data. Simulation rows have `has_robot=0`.

Gamepad loss, stale polling, invalid input, excessive callback timing gaps,
gripper faults and exit requests latch a stop and end the session after rest.
Restart after a latched stop. Robot/network/reflex errors exit without automatic
recovery or rearming. Epoch-tagged gripper requests cannot survive disarm/rearm.
SDL polling, network gripper operations and logging stay outside the RT callback.

```bash
./franka exec bash -c 'cd /workspace/ws/build/franka_gamepad_teleop && ctest --output-on-failure'
```

Tests use a fixed profile under `test/fixtures/`, independently of edits to the
live YAML. They cover binding signs, proportional input, vector caps, frames,
mode interlocks, measured stillness, velocity commands, force/torque computation,
compliant-plant behavior, continuity, gripper fencing and logging. An SDL virtual
controller plus an SDK stand-in executes the production dispatch code through
Delta → Speed → Force → Delta with no network or physical robot connection.

## Dependencies

Developed with libfranka **0.20.5**, Eigen **3.4**, SDL **2.30**, yaml-cpp **0.8**,
and the existing ROS 2 Jazzy/colcon environment. There is no dependency on the
hand teleoperation packages. The workspace supplies its Docker environment and
`franka` wrapper. For a fresh workspace, run `tools/run.py` directly or add this
wrapper case: `gamepad) exec python3 "$ROOT/ws/src/franka_gamepad_teleop/tools/run.py" "$@" ;;`.
