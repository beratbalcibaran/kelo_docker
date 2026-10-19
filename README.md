#  Navigation

**English** | [Türkçe](README.tr.md)

ROS 2 Jazzy navigation stack for the KELO Robile omnidirectional platform, packaged as a Docker image
for the NVIDIA Jetson Orin Nano.

| Component | Technology |
|---|---|
| Mapping | slam_toolbox |
| Localization | AMCL, Marvelmind indoor positioning |
| Navigation | Nav2 (MPPI; paths are followed forward, the platform itself is omnidirectional) |
| Sensors | Hokuyo URG-04LX (2D LiDAR), Intel RealSense D435 (depth), Marvelmind hedgehog |
| Safety | Two collision monitors (LiDAR + depth), velocity gate, watchdog, position divergence monitor |

The stack does not modify the robot's low-level driver. The interface to the robot is limited to
`/cmd_vel`, `/odom` and the `odom → base_link` transform.

> **Safety notice.** The safety functions in this project are implemented in software and are not
> certified. The URG-04LX and the D435 are not safety-rated sensors. Operation in areas shared with
> people requires a certified safety laser scanner. The hardware emergency stop operates independently
> of this software and must be within reach during every test.

## Contents

1. [Getting started](#getting-started)
2. [Project layout](#project-layout)
3. [Architecture](#architecture)
4. [Velocity command path](#velocity-command-path)
5. [Quick start](#quick-start)
6. [Configuration](#configuration)
7. [Installation on the Jetson](#installation-on-the-jetson)
8. [Commissioning procedure](#commissioning-procedure)
9. [Mapping with a sensor cart](#mapping-with-a-sensor-cart)
10. [Recording and replay](#recording-and-replay)
11. [Safety system](#safety-system)
12. [Simulator](#simulator)
13. [Testing](#testing)
14. [Operations and maintenance](#operations-and-maintenance)
15. [Open items](#open-items)
16. [Known limitations](#known-limitations)

## Getting started

The simulator runs on any 64-bit Linux computer (x86-64 or ARM64, e.g. the Jetson). No robot or
sensor is required.

1. Install Docker Engine and the Docker Compose plugin
   ([instructions](https://docs.docker.com/engine/install/)), then allow your user to run Docker:
   ```bash
   sudo usermod -aG docker $USER      # log out and back in afterwards
   ```
2. Download the project:
   ```bash
   git clone https://github.com/beratbalcibaran/kelo_docker.git
   cd kelo_docker
   ```
3. Build the image and start the control panel (the first build takes 15–30 min and about 5 GB):
   ```bash
   docker compose --profile gui up --build
   ```
4. When the log shows `KELO interface ready`, open
   `http://localhost:6080/vnc.html?autoconnect=true&resize=scale` and enter the VNC password printed
   in the log.
5. In the panel, leave the source on `sim` and follow these steps:

   **Build a map**
   1. **Start Mapping**: the simulator and RViz start, the map begins to grow.
   2. Click the panel window once, then drive with WASD or the arrow keys around the whole room
      until the walls in RViz are closed.
   3. **Save Map**: give a name (e.g. `room`). The log shows `Saved: ...` and
      `current.yaml -> ...`.
   4. **Stop Mapping**.

   **Navigate on the map**
   1. **Start Navigation**: the robot starts again at the point where mapping began. That point is
      `x=0, y=0` on the map.
   2. **Set Initial Pose**: enter X `0`, Y `0`, Yaw `0`. The robot may turn in place; wait until
      the log shows `LOCALIZED`.
   3. **Send Goal**: enter a free point on the map, e.g. X `0`, Y `2`, Yaw `0` (2 m to the left of the
      start). The log shows `Goal reached.` when the robot arrives. A goal can also be set in RViz
      with **2D Goal Pose**.
   4. **Stop Navigation** when done.

   **Localize** (without a position) needs a Marvelmind alignment file, which a fresh copy does not
   have; it then searches the whole map and may pick the wrong spot. Use **Set Initial Pose** in the
   simulator.
6. Stop with `Ctrl+C`, or `docker compose --profile gui down`.

The repository ships without maps: create one with **Start Mapping** / **Save Map** first. Maps
(`data/maps`), recordings (`data/bags`) and Marvelmind samples stay local and are not part of the repository.

## Project layout

```
kelo_docker/
├── README.md, README.tr.md     this documentation (English, Turkish)
├── Dockerfile                  builds the image: ROS 2 Jazzy, Nav2, drivers, this package
├── docker-compose.yml          start profiles (gui, sim, sim-nav, replay, production, mapping, ...)
├── docker/                     scripts that run inside the container
├── scripts/                    tests and Jetson installation, run on the host computer
├── systemd/kelo.service        starts the robot software automatically at boot (Jetson)
├── data/                       maps and recordings, mounted into the container as /data
└── src/kelo_bringup/           the ROS 2 package (all of the robot software)
```

| Path | Purpose |
|---|---|
| `docker/entrypoint.sh` | Prepares the ROS environment when a container starts |
| `docker/start-gui.sh` | Starts the browser control panel (noVNC + panel + RViz) |
| `docker/run-stack.sh` | Starts the software without the panel (used by the headless profiles and the service) |
| `docker/cyclonedds.xml` | Network settings for ROS 2 communication |
| `scripts/test_all.sh` | Builds the image and runs all tests |
| `scripts/sim_e2e_test.sh` | End-to-end tests in the simulator |
| `scripts/replay_regression.sh` | Rebuilds a map from a recording and compares it with a reference |
| `scripts/install_jetson.sh` | Installs the project on the Jetson (`/opt/kelo`, udev, systemd) |
| `data/maps/` | Saved maps (local, not in the repository); `current.yaml` points to the map used for navigation |
| `data/mm_samples_*.csv` | Marvelmind alignment samples recorded during mapping (local) |
| `data/bags/` | Sensor recordings made during mapping (not in the repository) |
| `data/hardware.yaml` | Optional local override of `config/hardware.yaml` (not in the repository) |

Inside `src/kelo_bringup/`:

| Path | Purpose |
|---|---|
| `config/hardware.yaml` | **Start here.** Which sensors are installed, serial ports, sensor positions, robot size, odometry correction |
| `config/safety.yaml` | Speed limits, stop and slowdown zones, depth camera filter |
| `config/nav2_params.yaml` | Localization (AMCL), path planning and following (Nav2), costmaps |
| `config/slam_toolbox.yaml` | Map building settings |
| `config/sim_world.yaml` | Simulator room, obstacles and odometry error |
| `launch/bringup.launch.py` | Single entry point that starts everything |
| `gui.py` | The control panel (buttons, WASD driving) |
| `urdf/kelo.urdf.xacro` | Robot model: body and sensor positions |
| `rviz/kelo.rviz` | RViz view settings |
| `udev/99-kelo-serial.rules` | Fixed device names for the USB sensors |
| `test/` | Unit tests (pytest) and reference data |

Programs in `src/kelo_bringup/kelo_bringup/`:

| File | Purpose |
|---|---|
| `stack.py`, `common.py` | Decide which programs start, depending on source, mode and installed sensors |
| `cmd_vel_gate.py` | The only program that sends speed commands to the robot; limits speed, stops on timeout |
| `cloud_filter.py` | Turns the depth image into an obstacle point cloud |
| `odom_filter.py` | Corrects the robot's wheel odometry |
| `status_node.py` | Reports the overall state and holds the localization state |
| `localize.py` | Finds the robot's position on the map (GUI: Localize) |
| `send_goal.py` | Sends a navigation goal (GUI: Send Goal) |
| `waypoint_tool.py` | Stores named positions and drives between them |
| `save_map.py` | Saves the current map (GUI: Save Map) |
| `marvelmind_bridge.py`, `mm_monitor.py`, `mm_recorder.py`, `mm_calibrate.py` | Marvelmind: position conversion, plausibility check, sample recording, map alignment |
| `odom_test.py` | Measures odometry error on the real robot |
| `map_compare.py` | Compares two maps |
| `wait_for_tf.py` | Waits until required coordinate frames are available at start-up |
| `logic.py` | Shared calculations (tested by the unit tests) |
| `sim_robot.py`, `sim/world.py` | The simulator: robot, sensors and collisions |
| `e2e.py` | End-to-end test scenarios |

### Key terms

| Term | Meaning |
|---|---|
| ROS 2 | The robot software framework; programs (nodes) exchange messages on named topics such as `/scan` |
| Docker image / container | A packaged environment with all dependencies; nothing else needs to be installed on the computer |
| Profile | A start configuration in `docker-compose.yml`, selected with `--profile` |
| Odometry | Position estimate from the wheels (or from the LiDAR) |
| SLAM / mapping | Building a map while driving |
| Localization (AMCL) | Finding the robot's position on an existing map |
| Costmap | Map of areas to avoid, used for path planning |
| Collision monitor | Stops or slows the robot when an obstacle enters a zone around it |
| TF / frame | Coordinate systems (`map`, `odom`, `base_link`, sensors) and the transforms between them |

## Architecture

The real robot, the simulator and bag replay share a single launch file
(`launch/bringup.launch.py` → `kelo_bringup/stack.py`) with identical nodes and parameters. Only the
source layer differs. The simulator publishes every sensor on the same topics, frames and message types
as the hardware.

```
                 ┌─ hardware: urg_node, realsense (depth only), marvelmind_ros2
 source layer  ──┼─ sim     : sim_robot (robot and all sensors, faulty odometry model)
                 └─ replay  : ros2 bag play --clock (use_sim_time)
                        │  /scan  /odom  depth image + camera_info  /hedgehog_pos_ang
                        ▼
 odometry       odom_source:=robot → odom_filter (speed-dependent correction) | odom_source:=lidar → rf2o
 common         robot_state_publisher, cloud_filter (depth → obstacle cloud), marvelmind_bridge,
                cmd_vel_gate, collision_monitor_teleop, status_node
 mode           mapping: slam_toolbox (+ bag recording) | navigation: AMCL + Nav2 + mm_monitor | sensors

 Nav2 → velocity_smoother → collision_monitor ──→ /cmd_vel_safe ─────┐
 teleop/GUI → /cmd_vel_teleop → collision_monitor_teleop ────────────┴→ cmd_vel_gate → /cmd_vel → robot

 TF: map → odom → base_link → {laser, camera_link → camera_depth_optical_frame, hedgehog_link}
```

### Odometry ownership

The `odom → base_link` transform always has exactly one publisher.

| Configuration | TF publisher | Odometry topic used by Nav2 |
|---|---|---|
| `odom_source:=robot`, `odom_mode: corrected` (default) | `odom_filter` (driver TF must be disabled) | `/odom_corrected` |
| `odom_source:=robot`, `odom_mode: driver` | robot driver (reconstructed from `/odom` in replay) | `/odom` |
| `odom_source:=lidar` | rf2o | `/odom_rf2o` |

All Nav2 nodes that consume odometry, including the MPPI controller, subscribe to the topic of the
active TF publisher.

### Sensor configuration

Installed sensors are declared in `hardware.yaml → sensors`. Disabling a sensor removes its driver,
costmap layer and collision-monitor source for every source type.

## Velocity command path

The stack publishes body velocities; the robot's driver converts them into wheel commands.

- **Topic:** `/cmd_vel`, type `Twist` or `TwistStamped` (`hardware.yaml → robot.cmd_vel_type`).
- **Fields:** `linear.x` forward (m/s), `linear.y` lateral (m/s), `angular.z` yaw rate (rad/s).
- **Single publisher:** `cmd_vel_gate` is the only node that publishes `/cmd_vel`. Nav2, keyboard
  teleoperation and the localization spin all pass through a collision monitor and then the gate.
- **Gate behaviour:** limits speed to 0.30 m/s and 0.80 rad/s; publishes zero velocity when no command
  is received for 0.5 s, when a safety stop is active, and on shutdown (SIGINT and SIGTERM).
- **Exception:** `odom_test calibrate-rotate` writes directly to the driver for calibration only and
  refuses to start while the gate is running.

## Quick start

### Graphical interface

The control panel runs in the browser through noVNC.

```bash
docker compose --profile gui up --build            # workstation, default source: sim
docker compose --profile gui-hardware up --build   # Jetson with hardware, default source: hardware
```

When the log shows `KELO interface ready` together with a VNC password, open
`http://localhost:6080/vnc.html?autoconnect=true&resize=scale`. Set `KELO_VNC_PASSWORD` to use a fixed
password; otherwise a random password is generated at each start.

The panel selects the source (sim, hardware, replay) and the odometry source (robot, lidar) and provides
Start Mapping, Save Map, Stop Mapping, Start Navigation, Set Initial Pose, Localize, Send Goal and Stop
Navigation. WASD and the arrow keys drive the robot through `collision_monitor_teleop` and
`cmd_vel_gate`.

RViz runs as a separate, supervised process. If it terminates, the panel restarts it without affecting
the robot, sensor or safety nodes.

### Headless profiles

| Profile | Function |
|---|---|
| `sim` | Simulator with mapping and bag recording |
| `sim-nav` | Simulator with AMCL and Nav2 on the saved map |
| `replay` | Mapping from a recorded bag (`KELO_BAG`) |
| `production` | Robot navigation (used by systemd); `KELO_MODE=sensors` keeps the robot stationary |
| `mapping` | Robot mapping; `KELO_ODOM_SOURCE=lidar` for sensor-cart mapping |

Environment variables read by `docker/run-stack.sh`:

| Variable | Values |
|---|---|
| `KELO_SOURCE` | `hardware`, `sim`, `replay` |
| `KELO_MODE` | `navigation`, `mapping`, `sensors` |
| `KELO_ODOM_SOURCE` | `robot`, `lidar` |
| `KELO_MAP` | map YAML path |
| `KELO_BAG` | bag directory, e.g. `/data/bags/<bag>` |
| `KELO_INITIAL_POSE` | `"x y yaw_deg"` (default `0 0 0`) |
| `KELO_EXTRA_ARGS` | additional launch arguments |

## Configuration

| File | Contents |
|---|---|
| `config/hardware.yaml` | Sensors, topics, frames, mounting positions, odometry correction tables |
| `config/safety.yaml` | Velocity gate, teleop collision monitor, depth filter, Marvelmind monitor, status |
| `config/nav2_params.yaml` | AMCL, Nav2, costmaps, Nav2 collision monitor |
| `config/slam_toolbox.yaml` | SLAM parameters |
| `config/sim_world.yaml` | Simulator world and odometry error model |

Paths are relative to `src/kelo_bringup/`.

## Installation on the Jetson

```bash
sudo ./scripts/install_jetson.sh --enable-service
```

The script installs the project to `/opt/kelo`, builds the image, installs the udev rules and registers
the `kelo` systemd service.

Requirements:

- JetPack with Docker and the Docker Compose plugin.
- Wired network connection to the robot computer.
- `ROS_DOMAIN_ID` in `/etc/kelo/kelo.env` matching the robot computer.

DDS middleware is Cyclone DDS (`docker/cyclonedds.xml`). Serial device paths are configured in
`hardware.yaml → serial` (see `ls /dev/serial/by-id/`).

## Commissioning procedure

### 1. Hardware check (robot stationary)

```bash
KELO_MODE=sensors docker compose --profile production up
```

Verify `ros2 topic hz /scan`, the TF tree in RViz and the periodic status line
(`docker logs kelo-production | grep STATUS`).

### 2. Odometry characterisation

The robot moves during this step. Use an open area, low speed and keep the emergency stop at hand.

```bash
docker compose run --rm production ros2 run kelo_bringup odom_test straight --speed 0.1 --distance 2.0
docker compose run --rm production ros2 run kelo_bringup odom_test rotate --rate 0.6 --turns 1
docker compose run --rm production ros2 run kelo_bringup odom_test summary
```

Measured KELO odometry under-reports yaw significantly and in a speed-dependent way: one physical
revolution is reported as approximately 68° at 0.6 rad/s (k ≈ 5.3) and 42° at 1.5 rad/s (k ≈ 8.6).
Linear error ranges from −0.6 % at 0.06 m/s to −1.8 % at 0.15 m/s.

Enter the results in `hardware.yaml → odom_correction` (`yaw_k: wz/k`, `linear_error: v/e`). Values are
interpolated linearly between points and clamped to the end values outside the table. `odom_filter`
applies the inverse correction at every step.

**Calibration above the gate limit.** To measure k above 0.80 rad/s, stop the stack and run:

```bash
docker compose down
docker compose run --rm production ros2 run kelo_bringup odom_test calibrate-rotate --rate 1.5 --turns 3
```

This mode bypasses `cmd_vel_gate`, the speed limit and the collision monitors. It is restricted to
in-place rotation (≤ 1.6 rad/s, ≤ 3 turns), requires typing `CALIBRATE` to confirm, publishes zero
velocity on every exit and refuses to run while the gate is active. Use at least three turns; the
acceleration ramp causes a slight underestimate of k. Not for production use.

### 3. Mapping

```bash
docker compose --profile mapping up
docker compose exec mapping ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_teleop
docker compose exec mapping ros2 run kelo_bringup save_map warehouse
```

Drive slowly and close the loop. `save_map` writes `/data/maps/warehouse_<date>.*` (PGM, YAML and the
slam_toolbox pose graph) and updates `current.yaml`. Edit the PGM to remove temporary objects and to
mark keep-out areas.

During mapping, sensor data is recorded to `/data/bags/map_<source>_<date>` (scan, odometry, depth
image and camera info, Marvelmind, TF; MCAP with zstd, about 1 MB/s). The point cloud is regenerated
from the depth image during replay and is not recorded. Disable recording with
`KELO_EXTRA_ARGS=record:=false`. When Marvelmind is enabled, alignment samples are written to
`/data/mm_samples_<date>.csv`.

### 4. Marvelmind alignment

```bash
docker compose exec mapping ros2 run kelo_bringup mm_calibrate /data/mm_samples_<date>.csv
```

The tool reports RMS, mean, maximum and 95th-percentile errors. The alignment is rejected when RMS
exceeds 0.15 m. On success, `/data/marvelmind_alignment.yaml` is written and published as the static
transform `map → marvelmind_map` at start-up.

### 5. Navigation

```bash
docker compose --profile production up -d     # or: systemctl start kelo
```

AMCL localizes on `current.yaml`. Nav2 starts once `map → base_link` is available.

**Initial pose.** A single Marvelmind hedgehog provides position but no heading.

| Method | Behaviour |
|---|---|
| `localize` (GUI: Localize) | Position from Marvelmind, unknown heading; the robot rotates 360° in place through the safety chain until AMCL converges, then publishes `LOCALIZED` |
| `localize --global` | AMCL global localization with rotation; selected automatically when no Marvelmind position is available |
| `localize --yaw 90` (GUI: Set Initial Pose) | Position from Marvelmind, heading supplied |
| `localize --x 1 --y 2 --yaw 90` | Full pose supplied |
| `KELO_INITIAL_POSE="x y yaw_deg"` | Fixed pose at start-up (e.g. charging station) |

When the robot is already `LOCALIZED` with low uncertainty, `localize` leaves AMCL unchanged and exits
with code 3; the GUI asks for confirmation before running `localize --force`. The state is held by
`status_node` on `/kelo/localization_state`. `send_goal` and `waypoint_tool` do not send goals while the
state is `LOCALIZING` or `FAILED`; goals sent from RViz are not subject to this check.

A second hedgehog (Marvelmind paired beacons, ≥ 0.3 m apart) would provide the heading directly and
remove the need for the localization spin.

### 6. Waypoints

Waypoints are stored in map coordinates in `/data/waypoints.yaml`.

```bash
docker compose exec production ros2 run kelo_bringup waypoint_tool add dock
docker compose exec production ros2 run kelo_bringup waypoint_tool add-click shelf_a
docker compose exec production ros2 run kelo_bringup waypoint_tool list
docker compose exec production ros2 run kelo_bringup waypoint_tool go dock shelf_a
docker compose exec production ros2 run kelo_bringup waypoint_tool loop dock shelf_a --count 10
```

`add` stores the current robot pose; `add-click` waits for a point or goal selected in RViz.

## Mapping with a sensor cart

A map can be created without the robot by mounting the LiDAR, the depth camera and the Marvelmind
hedgehog on a cart. Odometry is then provided by the LiDAR (rf2o).

1. Mount the sensors with the LiDAR level, ideally at the same height as on the robot (about 20 cm).
   Enter the measured positions in `hardware.yaml → mounts`.
2. Connect the sensors to the Jetson over USB.
3. Start mapping:
   - GUI: `docker compose --profile gui-hardware up --build`, source `hardware`, odometry `lidar`.
   - Headless: `KELO_ODOM_SOURCE=lidar docker compose --profile mapping up`.
4. Move at ≤ 0.3 m/s, turn slowly and return to the start point to close the loop.
5. Save the map and run `mm_calibrate`.
6. Before using the map on the robot, update `mounts` with the robot's measurements.

In simulation, cart mapping produced a map practically identical to robot-odometry mapping (F1 0.999).
This mode has not yet been validated with physical sensors.

## Recording and replay

Mapping bags can be replayed through the same stack. The recorded `/tf` is not replayed; SLAM or AMCL
regenerates `map → odom` and the odometry layer regenerates `odom → base_link`.

```bash
KELO_BAG=/data/bags/map_hardware_<date> docker compose --profile replay up
./scripts/replay_regression.sh data/bags/map_hardware_<date> data/maps/<reference>.yaml
./scripts/replay_regression.sh <bag> <reference.yaml> --odom lidar --min-f1 0.85
```

`replay_regression.sh` builds a map from the bag and compares it with the reference using
`map_compare` (occupied-cell precision, recall and F1 with 0.10 m tolerance).

## Safety system

| Condition | Response |
|---|---|
| No command for 0.5 s | `cmd_vel_gate` publishes zero velocity |
| Obstacle in the stop zone | Collision monitor stops the robot (Nav2 and teleop) |
| Obstacle in the slowdown zone (1.025 m ahead of the centre) | Speed scaled to 40 % |
| Predicted collision within 1.2 s (Nav2) / 1.0 s (teleop) | Speed reduced (`FootprintApproach`) |
| Obstacle outside the LiDAR plane | Detected by the depth camera (0.30–2.5 m, height 0.06–1.20 m) |
| LiDAR or camera data older than 1.0 s | Robot stopped until data resumes |
| Obstacle removed | Cleared from the costmap by the clearing cloud; navigation resumes |
| Marvelmind and AMCL differ by > 1.0 m for 2 s | `/kelo/safety_stop` asserted, zero velocity |
| No Marvelmind data | Reported as `MM_LOST`; no stop |
| Container or service shutdown | Zero velocity published; automatic restart on failure |

Speed limits are enforced in order `safety.yaml` (0.30 m/s, 0.80 rad/s) ≥ velocity smoother ≥ MPPI; a
unit test verifies this ordering.

**Footprint and stop zone.** The footprint is 0.85 × 0.53 m. The stop zone is direction-dependent
(`velocity_polygon`): forward 0.76 m from the centre (0.335 m beyond the front edge), lateral 0.465 m,
backward 0.625 m, and an octagon of about 0.59 m radius for in-place rotation. A fixed front zone would
prevent the robot from turning or reversing away from a wall; the direction-dependent zone blocks only
motion towards the obstacle.

**Sensor coverage.** The LiDAR is mounted at the front centre (scan plane 0.20 m above the floor) and
uses ±92° (`hardware.yaml → lidar`); wider rays would hit the robot's own front face. The depth camera
faces forward. The sides and the rear are therefore not covered by any sensor.

**Camera blind zone.** The D435 does not measure reliably below 0.30 m and has an 87° horizontal field
of view. The entire front edge of the stop zone must lie beyond `min_range` and within the field of view,
so that an obstacle detected only by the camera cannot disappear before entering the stop zone
(`test_config.py::test_camera_blind_zone_rule`).

**Measured response in simulation** (approx. 0.20 m/s, obstacle appearing 0.50 m beyond the front edge):

| Scenario | Result |
|---|---|
| Nav2 driving | Stop within 0.5–0.8 s, 0.39–0.44 m clearance |
| Teleop with forward key held | Stop within 0.7–0.9 s, 0.34–0.38 m clearance; no further motion while the key is held |
| Person crossing | No contact |
| Camera or LiDAR disconnected | Stop within 1.25–1.35 s |

**Monitoring topics:** `/diagnostics`, `/kelo/gate_state`, `/kelo/mm_state`, `/kelo/localization_state`,
`/collision_monitor_state`. A summary line is logged every 10 s
(`docker logs kelo-production | grep STATUS`).

## Simulator

`sim_robot` (configured in `config/sim_world.yaml`) is a lightweight simulator for the software chain,
not a physics engine. It provides:

- A 3D world with boxes, tables and partitions.
- LiDAR by ray casting at the sensor height.
- D435-equivalent depth images (16UC1, optical frame, floor included).
- Marvelmind positions with noise, outliers and dropouts.
- An omnidirectional robot with acceleration limits and a 0.5 s command timeout.
- Collision detection.
- The measured KELO odometry error model (default) or ideal odometry (`sim_odom_model:=ideal`).

Test interfaces: `/sim/ground_truth`, `/sim/contacts`, `/sim/obstacles` and
`ros2 param set /sim_robot camera_enabled false`. These topics are used by tests only.

## Testing

```bash
./scripts/test_all.sh                 # image build, unit and end-to-end tests (45–60 min)
./scripts/test_all.sh --unit-only     # unit tests only (~5 s)
./scripts/sim_e2e_test.sh A B         # selected end-to-end phases
```

**Unit tests (pytest)** cover configuration consistency (footprint, speed limits, zones, blind zone,
sources), the odometry model and its inverse, simulator ray casting and collision, the single-launch-path
rules (identical common nodes, single TF owner, consistent odometry topics) and localization logic.
Tests that require every sensor build the stack with all sensors enabled, so the results do not depend
on the current `hardware.yaml → sensors` settings.

**End-to-end phases** (`sim_e2e_test.sh`):

| Phase | Scope |
|---|---|
| A | Mapping, gate, watchdog, limits, safety stop, TF, sensors, map, alignment, bag |
| B | Navigation, localization from unknown heading, obstacle and sensor dropout scenarios, wall approach, driving around obstacles, goal motion, global localization, repeated localization |
| C | Replay regression against `test/data/sim_reference` |
| D | Odometry model and correction, calibration mode |

Inside the container, individual scenarios run with `kelo_e2e <scenario>`: `odom`, `tour`,
`localization`, `obstacle`, `dropout`, `teleop_obstacle`, `teleop_dropout`, `near_wall`, `goal_motion`, `detour`,
`relocalize`.

## Operations and maintenance

**First connection checklist**

1. Resolve the [open items](#open-items) and enter measurements in `hardware.yaml`.
2. Start in sensor mode (`KELO_MODE=sensors`) and verify `/scan`, depth, Marvelmind, `/odom` and the TF tree.
3. Confirm with `ros2 topic info -v /cmd_vel` that `cmd_vel_gate` is the only publisher.
4. With the robot raised or in an open area, verify motion directions with short key presses.
5. Release the key and confirm the robot stops within 0.5 s.
6. Verify the hardware emergency stop independently of the software.
7. Approach an obstacle slowly and measure the stopping clearance.
8. Disconnect the camera or LiDAR while moving slowly and confirm the robot stops.
9. Perform odometry characterisation and update `odom_correction`.
10. Create the map, align Marvelmind, localize and test navigation at low speed.

**Logs and service**

```bash
docker logs -f kelo-production          # rotated, 5 × 20 MB
systemctl status|restart|stop kelo
journalctl -u kelo -f
```

**Diagnostic recording**

```bash
docker compose exec production ros2 bag record -s mcap --storage-preset-profile zstd_fast \
  -o /data/bags/run --max-bag-duration 600 \
  /scan /odom /odom_corrected /tf /tf_static /cmd_vel /cmd_vel_safe /cmd_vel_nav /amcl_pose \
  /particle_cloud /marvelmind/pose /hedgehog_pos_ang /camera/camera/depth/image_rect_raw \
  /camera/camera/depth/camera_info /kelo/mm_state /kelo/mm_divergence_m /kelo/gate_state \
  /collision_monitor_state /diagnostics /local_costmap/costmap /plan
```

**Re-mapping.** Run the `mapping` profile and `save_map`. Previous maps are retained with a date
suffix and `current.yaml` points to the latest. To revert:
`ln -sfn <previous>.yaml data/maps/current.yaml`. Run `mm_calibrate` again after re-mapping.

## Open items

To be confirmed with the robot team:

- `/cmd_vel` topic name and message type (`Twist` or `TwistStamped`).
- `/odom` frame and rate, driver-side command timeout, `ROS_DOMAIN_ID`.
- Whether the driver's `odom → base_link` transform can be disabled (required by
  `odom_mode: corrected`; otherwise use `odom_mode: driver` without yaw correction).

To be verified on the robot:

- Sensor mounting positions (`hardware.yaml → mounts`, metres from `base_link`). Current values: camera
  x 0.42 z 0.68 tilted 10° down, LiDAR x 0.45 z 0.20 (estimated), hedgehog x 0 z 0.70.
- Footprint 0.85 × 0.53 m, kept identical in `nav2_params.yaml` (3 places) and `safety.yaml` (1 place);
  the stop and slowdown zones must contain the footprint (verified by `test_config.py`). The LiDAR
  protrudes about 5 cm beyond the front edge.
- Marvelmind is disabled in `hardware.yaml → sensors`; enable it once the hedgehog and beacons are
  installed. The end-to-end test follows the same setting (`$E2E_DIR/data/hardware.yaml` overrides it):
  with Marvelmind disabled its checks are reported as `SKIP` and phase B localizes from an operator
  position (`localize --x --y`).

Useful commands: `ros2 topic info -v /cmd_vel`, `ros2 topic info -v /odom`,
`ros2 run tf2_ros tf2_monitor odom base_link`.

## Known limitations

- Controller tuning, AMCL noise parameters and stopping distances have not been validated on the
  physical robot.
- Sensor mounting positions are approximate; the LiDAR position is estimated.
- With the camera tilted 10° down, obstacles 6–20 cm tall are lost by both sensors within about
  0.6–0.8 m of the front edge (below the LiDAR plane, outside the camera view). A steeper camera tilt
  (25–30°) reduces this zone.
- The sides and the rear are not covered by any sensor (front LiDAR, forward camera). For this reason
  Nav2 uses the `DiffDrive` motion model (forward motion and rotation only, no sideways motion); set
  `motion_model: "Omni"` in `nav2_params.yaml` once omnidirectional motion has been verified on the robot.
- Without Marvelmind, global localization can settle on a similar-looking place in a symmetric
  environment (observed in the simulator: the opposite corner, rotated 180°). Check the pose in RViz
  after **Localize**, or give the pose with **Set Initial Pose**.
- Driving around an obstacle is slow: in the simulator about 40–60 s for a 2.5 m detour, and about
  75–85 s when the obstacle appears suddenly within the stop zone (the robot must turn in place before
  it can continue). In roughly one of three sudden cases the robot stays stopped in front of the
  obstacle for more than 2.5 min (no collision).
- Odometry correction tables cover 0.6–1.5 rad/s and 0.06–0.15 m/s; outside this range the end values
  are used.
- The depth camera does not cover the areas beside the front corners (87° horizontal FOV); the LiDAR
  covers only its own plane.
- LiDAR odometry (rf2o) drifts in long or featureless corridors; loop closure in slam_toolbox corrects it.
- A single hedgehog provides no heading; the initial heading is estimated by rotation or supplied
  manually.
- RViz with software rendering (Mesa llvmpipe) logs a GLSL sampler validation warning at start-up; the
  map renders correctly.
- Acceptance tests (repeatability, stopping distance, kidnapped-robot recovery, one-hour endurance) are
  pending on the physical robot.
