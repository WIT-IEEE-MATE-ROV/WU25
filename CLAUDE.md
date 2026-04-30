# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

WU25 is the 2025 MATE ROV Competition codebase for a submarine ROV. It is a ROS 2 (Jazzy) package (`wu25`) written in C++23, targeting an **Orange Pi 5** (RK3588S SoC) as the onboard computer.

## Build Commands

This is a ROS 2 package. It must be built from the workspace root (not from inside the package):

```bash
cd ~/ros2_ws
source install/local_setup.sh
colcon build
```

Build with dummy hardware (no wiringPi/GPIO required, for development on non-Orange-Pi machines):

```bash
colcon build --cmake-args -DDUMMY=True
```

Run individual nodes after building:

```bash
ros2 run wu25 thrusters
ros2 run wu25 bno_node
```

Run both nodes with the packaged launch file (preserves environment and runs with sudo):

```bash
source install/local_setup.bash
ros2 launch wu25 start_nodes.launch.py
```

Note: the `start_nodes.launch.py` file uses a `sudo -E env ... bash -c` prefix so nodes that require hardware access (GPIO/SPI/I2C) can run with elevated privileges while preserving the environment variables needed by ROS.

Systemd service for control server
---------------------------------
To run the control server on boot on the Orange Pi, create a systemd unit file `/etc/systemd/system/rov-control.service` with contents similar to:

```ini
[Unit]
Description=ROV Control Server
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/ros2_ws/src/WU25
Environment=PYTHONUNBUFFERED=1
ExecStart=/home/pi/ros2_ws/src/WU25/.venv/bin/python3 tools/rov_control_server.py
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

Then enable and start with:
```bash
sudo systemctl daemon-reload
sudo systemctl enable rov-control.service
sudo systemctl start rov-control.service
sudo journalctl -u rov-control.service -f
```


There are no automated tests beyond `ament_lint`. To run lint checks:

```bash
colcon test --packages-select wu25
```

## Architecture

### Two ROS 2 Nodes

**`bno_node`** (`src/bno_node.cpp`) — IMU node
- Reads orientation from the BNO08x IMU via SPI (`deps/sh2` library + `src/drivers/BNO08x_ada.cpp`)
- Uses `libgpiod` for GPIO interrupt handling and `wiringPi` for SPI on the Orange Pi 5
- Publishes `geometry_msgs/QuaternionStamped` on topic `bno/quat` at 100 Hz
- In DUMMY mode, wiringPi is replaced by `src/dummy/wiringPi.c`

**`thrusters`** (`src/thruster_node.cpp`) — Thruster/motion node
- Subscribes to `sensor_msgs/Joy` (Xbox controller via ROS joy node)
- Outputs 8-channel PWM commands to a PCA9685 I2C PWM driver (`src/drivers/pca9685.cpp`)
- Runs a 50 Hz control loop

### Thruster Math (`src/thrusters/`, `include/thrusters/`)

The core of the system. `Thrusters` decomposes a 6-DOF `ThrustVector` (3 linear + 3 angular, in kgf) into 8 individual thruster force outputs using Eigen's `CompleteOrthogonalDecomposition`:

- **Horizontal thrusters** (indices 0–3): handle X, Y translation and yaw. Config matrix uses physical mounting angles (40° from center).
- **Vertical thrusters** (indices 4–7): handle Z translation, pitch, and roll. Config matrix uses physical mounting positions.

`ThrusterData` (`src/thrusters/thruster_data.cpp`) maps thrust (kgf) ↔ PWM using polynomial regression fit to `data/T200-Public-Performance-Data.csv` (Blue Robotics T200 thruster performance data).

Physical thruster channel mapping in `thruster_node.cpp`:
- FLH=6, FRH=1, BLH=7, BRH=2 (horizontal)
- FLV=4, FRV=3, BLV=5, BRV=0 (vertical)

Some horizontal thrusters are negated (`negatePeriod()`) due to motor orientation.

### Control Modes

`Thrusters` supports several control modes toggled at runtime:

| Method | Effect |
|---|---|
| `SetHoldIdleRotation(true)` | `QuatPIDController` holds last orientation when no angular input |
| `SetHoldIdleDepth(true)` | `PIDController` holds depth when no vertical input |
| `SetDepthLock(true)` | Rotates horizontal thrust to global XY plane (gravity-compensated translation) |
| `SetAngVelControl(true)` | Treats angular thrust vector as target angular velocity (rad/s) via PID |

### Controller Input (`include/util/controller_map.hpp`)

Xbox controller axis/button enum mappings. Left stick = XY translation, right stick = yaw/pitch, triggers = depth (differential). Bumpers control an auxiliary servo (channel 8 on PCA9685).

### Dependencies (all in `deps/` as git submodules)

- **Eigen** — matrix math for thruster decomposition
- **sh2** — Hillcrest BNO08x IMU protocol (SensorHub 2)
- **csv** (header-only) — CSV parsing for thruster performance data
- **wiringOP** — Orange Pi GPIO/SPI (replaces wiringPi for Orange Pi 5)
- **libgpiod** — GPIO interrupt handling for IMU
- **matplotplusplus** — used for offline PWM↔thrust curve visualization only

## Hardware Notes

- **SPI** must be enabled on the Orange Pi 5 via device tree modification (see README for dtc commands to enable `spi@fecb0000`). The IMU appears at `/dev/spidev4.1`.
- **PCA9685** is on I2C address `0x40`, configured at 100 Hz PWM frequency.
- PWM neutral for T200 thrusters is **1500 µs**. The deadband workaround in `ThrusterNode::Loop()` oscillates between 1495/1505 to prevent ESC from entering sleep mode at idle.
