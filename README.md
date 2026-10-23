# Franka gamepad teleoperation

Delta pose control for the Franka Research 3 using a generic gamepad. The app
reads the current configured TCP before starting. Deflection continuously moves
a target pose at up to **20 mm/s** and **20 degrees/s**. Combined translation
and rotation inputs retain their respective vector magnitude limits.

All translations and rotations use the **robot base frame**. Rotations change
orientation about the current TCP origin, without rotating its position around
the base. There is no application workspace box.

| Control | Action |
| --- | --- |
| Left stick up/down | Base +X / −X |
| Left stick right/left | Base −Y / +Y |
| L2 / R2 | Base +Z / −Z |
| Right stick right/left | Positive / negative rotation about base X |
| Right stick up/down | Positive / negative rotation about base Y |
| R1 / L1 | Positive / negative rotation about base Z |
| Y | Toggle arm/disarm |
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

# Real control starts disarmed. Y enables motion.
./franka gamepad --execute
```

The robot address defaults to the workspace's `config/robot.env`; override with
`--robot-ip ADDRESS`. Use `--duration 10` for a timed session, Ctrl+C to stop,
and `--dry-run` to print the command without starting it. The launcher also runs
directly: `python3 ws/src/franka_gamepad_teleop/tools/run.py --inspect`.

Real mode uses libfranka Cartesian pose motion generation and Cartesian impedance
control. It holds the current pose while disarmed. The first command uses
`O_T_EE_c` to preserve FCI startup continuity. The measured starting TCP printed
to the console is `O_T_EE`: the existing configured EE, including its flange-to-EE
transform. The app does not redefine the TCP or change collision thresholds,
robot safety functions, impedance parameters, joint limits or tool/load settings.
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

## Checks and failure handling

Gamepad loss, stale polling, invalid input, excessive callback timing gaps,
gripper faults and exit requests latch a stop and end the session after rest.
Restart after a latched stop. Robot/network/reflex errors exit without automatic
recovery or rearming. Epoch-tagged gripper requests cannot survive disarm/rearm.
SDL polling, network gripper operations and logging stay outside the RT callback.

```bash
./franka exec bash -c 'cd /workspace/ws/build/franka_gamepad_teleop && ctest --output-on-failure'
```

Tests cover binding signs, proportional input, vector rate caps, base rotation
composition, arming/disarming, deceleration, stale/disconnected input, gripper
request fencing, YAML validation and an SDL virtual controller. Physical motion
and gripping require an operator-run commissioning session.

Only delta mode is implemented. Speed and force modes will be added separately.

## Dependencies

Developed with libfranka **0.20.5**, Eigen **3.4**, SDL **2.30**, yaml-cpp **0.8**,
and the existing ROS 2 Jazzy/colcon environment. There is no dependency on the
hand teleoperation packages. The workspace supplies its Docker environment and
`franka` wrapper. For a fresh workspace, run `tools/run.py` directly or add this
wrapper case: `gamepad) exec python3 "$ROOT/ws/src/franka_gamepad_teleop/tools/run.py" "$@" ;;`.
