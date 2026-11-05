# Connecting to the robot: step by step

**English** | [Türkçe](robot_startup.tr.md)

This page lists, in order, what to do to connect the KELO stack on the Jetson to the real robot and
get from there to mapping and navigation. The installation itself (Docker, the driver on the robot
computer, network settings) is in [README → Installation from scratch](../README.md#installation-from-scratch-jetson-and-robot).

Nothing is installed on the robot computer and nothing runs there by itself. The driver is started
from the Jetson with `scripts/robot.sh` as before. Everything else happens on the Jetson side: when the
cable is plugged in the robot is found, its settings are applied, navigation starts if there is a map
and the robot localizes itself.

## 0. Update the code and build the image (once per update)

```bash
cd ~/Desktop/kelo_docker
sudo systemctl stop kelo                      # if the production service is running
git fetch
git checkout gelistirme-v1
git pull
docker compose --profile gui-hardware build   # 15-30 min on the Jetson
```

If the front of the robot (`base_link` +x on the sensors' side) changed, a map made before the change
is no longer correct. Move it aside, otherwise navigation starts with it when **Automatic** is on:

```bash
mv data/maps/current.yaml data/maps/old_current.yaml
```

## 1. Robot computer

```bash
./scripts/robot.sh status      # container and driver
./scripts/robot.sh deploy      # only if robot/ changed
./scripts/robot.sh build       # only after deploy
./scripts/robot.sh start       # start the driver
./scripts/robot.sh time        # set the robot clock from the Jetson (asks for the sudo password)
```

Setting the clock is not required but it is the clean way. Without it the stack moves the odometry
time stamps to the Jetson clock (`odom_mode: corrected` and `relay`); in `driver` mode it cannot, and
the panel says "robot clock is … off: run scripts/robot.sh time".

## 2. Check the connection without moving (recommended)

With the stack off and only the driver running:

```bash
docker compose run --rm production ros2 run kelo_bringup robot_check
```

Look at the `/odom` rate (~20 Hz), the clock offset, who publishes `odom → base_link` and the
`/cmd_vel` type. Fix any `FAIL` first.

## 3. Open the panel and plug in the cable

```bash
docker compose --profile gui-hardware up
```

1. **Automatic** is ticked in the panel (default).
2. Plug in the Ethernet cable. The panel looks for the robot, reads the driver's settings and writes
   the matching `odom_mode` (`driver`, `relay` or `corrected`) to `/data/hardware.yaml` without asking.
   If the driver is not running yet it tries again every 10 s.
3. Check the `odom_mode` line in the log.
4. Without a map the log says "Automatic: no map … build one with Start Mapping -> Save Map". That is
   expected: go to step 4. With a map, navigation starts by itself: go to step 5.

## 4. Make a map

1. **Start Mapping**.
2. Drive the robot slowly through the whole room with the keys (WASD / arrows) or `robot.sh joy safe`.
   Let it see every wall and corner at least once and, if possible, come back to where it started.
3. **Save Map**, then **Stop Mapping**.

## 5. Navigation

1. **Start Navigation** (with a map, **Automatic** does this itself).
2. Leave the robot standing for a few seconds on a flat floor with 2 m free in front. The depth camera
   measures its own tilt from the floor meanwhile; the log shows `camera tilt from the floor: roll …
   pitch …`.
3. If nobody presses **Localize** within 15 s, the robot localizes itself. It may turn in place:
   **make sure the area around it is clear.**
4. The **Robot:** line at the top must say **READY** (green). If not, the line gives the reason (no
   odometry, clock offset, two TF publishers, no LiDAR, not localized, a navigation node restarting …).
5. Send a short goal of 1-2 m first (**Send Goal** or *2D Goal Pose* in RViz), then a long one. If the
   robot waits, the line says why (`WAITING: …`).

## 6. What to look at

| Where | What it tells |
|---|---|
| **Robot:** line | READY / NOT READY: reason / WAITING: reason / DRIVING |
| **Health:** line | Nodes down or restarted, CPU, GPU, RAM, temperature, the 3 busiest processes |
| **Nodes** button | Every node, its state and how often it restarted |
| Log: `camera tilt from the floor` | Measured camera tilt (above 1-2°: check the mount or the value in `hardware.yaml`) |
| Log: `robot clock` | Clock offset between the robot and the Jetson |
| Log: `incident … saved` | Recording of the problem under `/data/incidents/` |

When something goes wrong, keep `data/incidents/` and the log:

```bash
docker logs kelo-gui-hardware 2>&1 | grep -E "STATUS|LOST|robot clock|odom_conflict|camera tilt|health_monitor" > incident.txt
```

The recordings can be played back in the simulator with `ros2 bag play`.

## 7. Without the panel (once everything is verified)

```bash
sudo systemctl enable --now kelo
journalctl -u kelo -f
```

At start the production service looks for the driver every 10 s, writes the settings to
`/data/hardware.yaml` and `/data/robot.env`, then starts the stack. `KELO_AUTO=0` in
`/etc/kelo/kelo.env` turns this off.

## Troubleshooting

| Symptom | Where to look |
|---|---|
| `NOT READY: no odometry from the robot` | Cable, `robot.sh status`, `ROS_DOMAIN_ID` 42 |
| `NOT READY: robot clock is … off` | `./scripts/robot.sh time` |
| `NOT READY: two publishers of odom -> base_link` | **Find robot → Apply settings** (`odom_mode: driver`) |
| `NOT READY: not localized` | **Localize**; otherwise **Set Initial Pose** |
| `WAITING: obstacle on the path … at (x, y)` | Costmap at that point in RViz; if nothing is there, check the camera tilt |
| `WAITING: collision monitor stop` | Something right in front of the robot (or old camera/LiDAR data) |
| `goal given up after 60 s without progress` | The reason on the line; the recording in `data/incidents/` |
| **Health:** red | The node that is down in **Nodes**; it should come back within seconds |
| Robot drives backwards instead of forwards | Front direction in the driver settings; make a new map |
