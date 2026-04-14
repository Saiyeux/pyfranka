# pyfranka

Python bindings for the Franka Emika Panda robot arm, built with pybind11 on top of [libfranka](https://github.com/frankaemika/libfranka).

Exposes the full `franka::Robot` control interface to Python, including real-time motion callbacks, model computation, and gripper control.

## Requirements

- RT Linux (real-time kernel) on the control PC
- libfranka (tested with the version matching your robot's firmware)
- Eigen3
- Python 3.8
- CMake ≥ 3.10

## Build

```bash
git clone <this repo>
cd pyfranka
mkdir build && cd build
cmake ..
make -j$(nproc)
cd ..
pip install -e .
```

CMake will look for libfranka via `find_package(Franka)`. If libfranka is installed to a non-standard path, set `CMAKE_PREFIX_PATH` before building:

```bash
cmake .. -DCMAKE_PREFIX_PATH=/path/to/libfranka/install
```

The compiled extension (`franka_pybind.cpython-38-x86_64-linux-gnu.so`) is placed in `pyfranka/` automatically.

## Quick Start

```python
from pyfranka.franka_pybind import (
    FrankaApi, RobotMode, MotionGenerator,
    JointPositions, JointPositionsMotionFinished,
    CartesianVelocities, CartesianVelocitiesFinished,
)

robot = FrankaApi()
robot.init_config("192.168.1.1", log_size=5000)
robot.set_default_behavior()
robot.set_collision_behavior(
    [25,25,22,20,19,17,14], [35,35,32,30,29,27,24],
    [25,25,22,20,19,17,14], [35,35,32,30,29,27,24],
    [30,30,30,25,25,25],    [40,40,40,35,35,35],
    [30,30,30,25,25,25],    [40,40,40,35,35,35],
)

# Check robot is ready
state = robot.readOnce()
if state.robot_mode == RobotMode.kReflex:
    robot.automatic_error_recovery()

# Move to home position
import math
q_home = [0, -math.pi/4, 0, -3*math.pi/4, 0, math.pi/2, math.pi/4]
robot.robot_control(joint_positions_handle=MotionGenerator(0.5, q_home).operator)
```

## API

### `FrankaApi`

| Method | Description |
|--------|-------------|
| `init_config(ip, realtime_config, log_size)` | Connect to robot |
| `set_default_behavior()` | Apply safe default collision / impedance settings |
| `set_collision_behavior(...)` | Set torque and force thresholds (4- or 8-array form) |
| `robot_control(handle=callback)` | Run real-time control loop (9 overloads, see below) |
| `read_robot_state(callback)` | Read-only state loop |
| `readOnce()` | Read current `RobotState` once |
| `loadModel()` | Load `Model` for kinematics / dynamics computation |
| `automatic_error_recovery()` | Clear reflex state |
| `set_joint_impedance(K_theta)` | Set joint impedance stiffness |
| `set_cartesian_impedance(K_x)` | Set Cartesian impedance stiffness |
| `set_guiding_mode(axes, elbow)` | Enable guiding mode on specific axes |
| `setEE(NE_T_EE)` | Set EE frame transform |
| `setK(EE_T_K)` | Set stiffness frame |
| `setLoad(mass, CoM, inertia)` | Set end-effector load |
| `stop()` | Stop current motion |

### `robot_control` overloads

All callbacks receive `(RobotState, Duration)` and must return the matching control type.
**All parameter names are required** (pybind11 limitation with default arguments).

```
robot_control(torques_handle=cb)
robot_control(joint_positions_handle=cb)
robot_control(joint_velocities_handle=cb)
robot_control(cartesian_pose_handle=cb)
robot_control(cartesian_velocities_handle=cb)
robot_control(torques_handle=cb, joint_positions_handle=cb)
robot_control(torques_handle=cb, joint_velocities_handle=cb)
robot_control(torques_handle=cb, cartesian_pose_handle=cb)
robot_control(torques_handle=cb, cartesian_velocities_handle=cb)
```

To finish motion inside a callback, wrap the return value:

```python
return JointPositionsMotionFinished(joint_positions_obj)
return CartesianVelocitiesFinished(CartesianVelocities([0]*6))
```

### `MotionGenerator`

Generates a smooth joint-space trajectory to a goal configuration.

```python
mg = MotionGenerator(speed_factor, q_goal)   # speed_factor: 0.0–1.0
robot.robot_control(joint_positions_handle=mg.operator)
```

### `Model`

Loaded via `model = robot.loadModel()`. Provides:

```python
model.pose(Frame.kEndEffector, robot_state)          # 4×4 EE pose (list[16])
model.zeroJacobian(Frame.kEndEffector, robot_state)  # 6×7 Jacobian (list[42])
model.bodyJacobian(Frame.kEndEffector, robot_state)
model.mass(robot_state)                              # 7×7 mass matrix (list[49])
model.coriolis(robot_state)                          # list[7]
model.gravity(robot_state, gravity_vector)           # list[7]
```

### `RobotState` fields (read-only)

```
O_T_EE        — EE pose in base frame (list[16], column-major)
q, q_d        — joint positions: actual / desired (list[7])
dq, dq_d      — joint velocities (list[7])
tau_J         — joint torques (list[7])
robot_mode    — RobotMode enum
current_errors, last_motion_errors  — Errors object
time          — Duration
```

### `Gripper`

```python
from pyfranka.franka_pybind import Gripper

gripper = Gripper("192.168.1.1")
gripper.homing()
gripper.grasp(width=0.05, speed=0.1, force=10)
gripper.move(width=0.08, speed=0.1)
state = gripper.readOnce()   # GripperState: width, max_width, is_grasped
gripper.stop()
```

## Examples

See `example/` for runnable scripts:

- `generate_joint_position_motion.py` — joint-space oscillation via position callback
- `generate_joint_velocity_motion.py` — joint velocity control
- `generate_cartesian_pose_motion.py` — Cartesian pose control
- `generate_cartesian_velocity_motion.py` — Cartesian velocity control
