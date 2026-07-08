
# hex_ros_robot_arm
[中文](README_CN.md) | **English** c

## Table of Contents

- [1. About](#1-about)
- [2. Package Structure](#2-package-structure)
- [3. Topics](#3-topics)
- [4. Control Modes](#4-control-modes)
- [5. Parameters](#5-parameters)
- [6. Dependencies](#6-dependencies)
- [7. Quick Start](#7-quick-start)

---

## 1. About

This is a **unified ROS driver package** for the **Archer Y6**, **Firefly Y6**, and **Hello Y6** manipulators.

It contains three independent real-robot nodes in a single ROS package, sharing a common `utility/` interface layer. The three nodes are independent — they do not dispatch by `robot_type` — each has its own `entry_point`:

- **Archer Y6** — 6-DoF arm + gripper (MIT/position/end-effector control)
- **Firefly Y6** — 6-DoF arm + gripper, structurally symmetric to Archer (`robot_type=27`)
- **Hello Y6** — 6-DoF read-only arm (state-only), with joystick handle and RGB LED

Each node connects to the real controller over WebSocket, subscribes to control commands, and publishes real-time arm state. Supports both **ROS 1** and **ROS 2**.

---

git clone https://github.com/hexfellow/hex_ros_common.git
## 2. Package Structure

```
hex_ros_robot_arm/
├── config/                        # Configuration files (one per node)
│   ├── ros1/
│   │   ├── archer_params.yaml     #   Archer Y6 ROS 1 params
│   │   ├── firefly_params.yaml    #   Firefly Y6 ROS 1 params
│   │   └── hello_params.yaml      #   Hello Y6 ROS 1 params
│   └── ros2/
│       ├── archer_params.yaml     #   Archer Y6 ROS 2 params
│       ├── firefly_params.yaml    #   Firefly Y6 ROS 2 params
│       └── hello_params.yaml      #   Hello Y6 ROS 2 params
├── launch/                        # Launch files (one per node)
│   ├── ros1/
│   │   ├── archer.launch          #   Archer Y6 ROS 1 launch
│   │   ├── firefly.launch         #   Firefly Y6 ROS 1 launch
│   │   └── hello.launch           #   Hello Y6 ROS 1 launch
│   └── ros2/
│       ├── archer.launch.py       #   Archer Y6 ROS 2 launch
│       ├── firefly.launch.py      #   Firefly Y6 ROS 2 launch
│       └── hello.launch.py        #   Hello Y6 ROS 2 launch
├── hex_ros_robot_arm/             # Core source
│   ├── robot_archer_y6.py         #   Archer Y6 main node (control loop + ROS interface)
│   ├── robot_firefly_y6.py        #   Firefly Y6 main node (control loop + ROS interface)
│   ├── robot_hello_y6.py          #   Hello Y6 main node (read-only control loop + ROS interface)
│   ├── test_ctrl.py               #   Interactive test node (debug tool)
│   ├── utility/                   #   Common DataInterface (Archer / Firefly)
│   ├── hello_utils/               #   Hello Y6 DataInterface (read-only + joystick + RGB LED)
│   └── test_utils/                #   Test DataInterface
├── resource/                      # ament resource marker
├── test/                          # ROS standard tests
├── setup.py                       # Python packaging (4 entry_points)
├── package.xml                    # ROS package manifest (dual-system conditional deps)
└── README.md                      # English documentation
```

### Three Interface Layers

| Module | Nodes | Publishes | Subscribes | Description |
|--------|-------|-----------|------------|-------------|
| `utility/` | archer, firefly | manip_state, joint_states (7-DoF), /clock | manip_ctrl | Standard arm interface (arm + gripper) |
| `hello_utils/` | hello | manip_state, joint_states (6-DoF), joy_state, /clock | color_cmd | Read-only mode (no gripper, adds joystick + RGB LED) |
| `test_utils/` | test_ctrl | — | — | Debug: manually constructs control commands |

---

## 3. Topics

### Archer Y6 / Firefly Y6 (arm + gripper)

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| sub | `manip_ctrl` | `hex_ros_msgs/(msg/)HexRosRoboManipCtrlStamped` | Arm + gripper control command |
| pub | `manip_state` | `hex_ros_msgs/(msg/)HexRosRoboManipStateStamped` | Arm + gripper state feedback |
| pub | `joint_states` | `sensor_msgs/(msg/)JointState` | 7 joints (6 arm + 1 gripper) |
| pub | `/clock` | `rosgraph_msgs/(msg/)Clock` | Clock for sim_time compatibility |

### Hello Y6 (read-only arm + joystick + RGB LED)

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| pub | `manip_state` | `hex_ros_msgs/(msg/)HexRosRoboManipStateStamped` | Arm state feedback (gripper empty) |
| pub | `joint_states` | `sensor_msgs/(msg/)JointState` | 6 arm joints (no gripper) |
| pub | `joy_state` | `hex_ros_msgs/(msg/)HexRosTeleopHandleStateStamped` | Joystick handle state (axis_x/y, trigger, buttons W/X/Y/Z) |
| pub | `/clock` | `rosgraph_msgs/(msg/)Clock` | Clock for sim_time compatibility |
| sub | `color_cmd` | `std_msgs/(msg/)ColorRGBA` | RGB LED color command (float 0-1 → int 0-255) |

### MIT Mode Warning

Incorrect kp/kd values may cause violent motion or equipment damage.

> Operate in a safe area with emergency stop accessible.

---

## 4. Control Modes

### Archer Y6 / Firefly Y6

**Arm control modes:** MIT (impedance), JNT (position), EE (end-effector pose), NONE

**Gripper control modes:** MIT (impedance), JNT (position), TAU (force/torque), NONE

### Hello Y6

**No control modes.** Hello Y6 is a read-only (state-only) device — it does not accept control commands.

---

## 5. Parameters

### Archer Y6 / Firefly Y6

| Param | Default | Description |
|-------|---------|-------------|
| `ctrl_rate` | 1000.0 | Main control loop rate [Hz] |
| `rate_state` | 500.0 | State publish rate (decimated from ctrl_rate) [Hz] |
| `robot_host` | 192.168.1.100 | Robot controller IP address |
| `robot_port` | 8439 | WebSocket port |
| `robot_frame_id` | `base_link` | Frame ID in state message header |
| `robot_grip_type` | `gp80` | Gripper type: `gp80` (1-DoF) or `empty` (no gripper) |
| `state_buffer_size` | 200 | Driver state buffer size |
| `sens_ts` | `false` | Use sensor hardware timestamps |
| `use_ros_time` | `false` | Time source: `false` → PTP, `true` → ROS clock |

### Hello Y6

| Param | Default | Description |
|-------|---------|-------------|
| `ctrl_rate` | 500.0 | Main control loop rate [Hz] |
| `rate_state` | 100.0 | State publish rate (decimated from ctrl_rate) [Hz] |
| `robot_host` | 192.168.1.100 | Robot controller IP address |
| `robot_port` | 8439 | WebSocket port |
| `robot_frame_id` | `base_link` | Frame ID in state message header |
| `state_buffer_size` | 200 | Driver state buffer size |
| `sens_ts` | `false` | Use sensor hardware timestamps |
| `use_ros_time` | `false` | Time source: `false` → PTP, `true` → ROS clock |

> Hello Y6 has no `robot_grip_type` parameter (no motorized gripper), and its default `ctrl_rate` / `rate_state` are lower.

---

## 6. Dependencies

### Python Packages

```shell
pip3 install 'hex-util-msg>=0.1.0a0'
pip3 install 'hex-util-ros>=0.0.1a0'
pip3 install 'hex-util-runtime>=0.0.0,<0.1.0'
pip3 install 'hex-driver-robot>=0.1.0'
```

### ROS Packages

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_common.git
git clone https://github.com/hexfellow/hex_ros_robot_arm.git
```

---

## 7. Quick Start

### 1. Create Workspace

```shell
mkdir -p hex_ws/src
cd hex_ws/src
```

### 2. Clone Repositories

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_common.git
git clone https://github.com/hexfellow/hex_ros_robot_arm.git
```

### 3. Build

**ROS 1:**

```shell
source /opt/ros/noetic/setup.bash
cd hex_ws
catkin_make
source devel/setup.bash --extend
```

**ROS 2:**

```shell
source /opt/ros/humble/setup.bash
cd hex_ws
colcon build
source install/setup.bash --extend
```

### 4. Use

```shell
# ROS 2 — Archer Y6
ros2 launch hex_ros_robot_arm archer.launch.py \
    robot_host:=192.168.1.100 robot_port:=8439 robot_grip_type:=empty

# ROS 2 — Firefly Y6
ros2 launch hex_ros_robot_arm firefly.launch.py \
    robot_host:=192.168.1.100 robot_port:=8439 robot_grip_type:=empty

# ROS 2 — Hello Y6 (read-only)
ros2 launch hex_ros_robot_arm hello.launch.py \
    robot_host:=192.168.1.100 robot_port:=8439

# ROS 2 — Archer Y6 (with test node)
ros2 launch hex_ros_robot_arm archer.launch.py \
    robot_host:=192.168.1.100 robot_port:=8439 robot_grip_type:=empty test:=true
```

> Replace `robot_host` and `robot_port` with your robot controller's actual IP and port.
