# Next steps: improving mapping and Marvelmind

Your stack is well structured and we would like to build on it. This file lists four improvements we would like you to implement next.

**Constraint: use only your current sensors**, the Hokuyo URG-04LX, the RealSense D435 and the Marvelmind. No new hardware.

You can develop and debug each task with your sensor cart and with recorded bags. The final tests will be on the KELO.

---

## Task 1: two-layer map (Hokuyo + RealSense)

**Priority: high.** This is the main limitation of the current mapping.

### Problem

The Hokuyo sees a single plane about 20 cm above the floor, and that plane is the only thing saved in the map. As a result:

- a table appears only as its legs, so the space under it looks free;
- shelves, overhangs and anything above 20 cm are missing;
- objects lower than the scan plane are missing too.

The D435 does see these obstacles, and `camera_layer` already uses it in both costmaps, but only live. Nothing the camera sees while mapping is saved. The global planner therefore plans paths through tables, and the robot only discovers them when it gets close.

### What to implement

1. **Keep the pose from the lidar.** slam_toolbox with the Hokuyo stays as it is. Do not feed camera points into scan matching: the D435 is too narrow (87°) and too noisy at range.
2. **Accumulate camera obstacles in a second layer.** `cloud_filter` already produces height-filtered points in `base_link`. During mapping, transform them into `map` with the slam_toolbox pose and accumulate them in a second occupancy grid with the same resolution and origin as the lidar map. Handle free space as well, using the clearing cloud, so that people walking by do not leave permanent obstacles.
3. **Save and load it with the map.** `save_map` should also write the camera layer (for example `<name>_camera.yaml/.pgm`). In navigation, load it into the global costmap as a second static layer, next to the lidar map.
4. **Keep it optional.** With `camera: false`, or with no camera layer file, everything must work as it does today.

You may write the accumulation node yourself (preferred, since it reuses your code), or compare with an existing tool such as RTAB-Map or OctoMap. If you use one of them, check that it is available for Jazzy.

### How to evaluate

- Choose a room with tables, shelves and some low objects.
- Map it twice from the same bag: lidar only, and lidar + camera layer.
- Compare both against a reference map using `map_compare`. Report precision, recall and F1, and show the maps side by side.
- Run navigation goals placed behind or between tables, about 10 per configuration. Count successes, replans, and stops caused by obstacles discovered late.

### Deliverables

Code, a short README section explaining how to enable it, the bags used, and a table with the results.

---

## Task 2: fix Marvelmind timing and calibration

**Priority: medium.** These are small changes with a direct effect on accuracy.

### 2a. Timestamp of the measurement, not of its arrival

`marvelmind_bridge` stamps each position with `now()` when the message arrives. Marvelmind positions arrive a few tenths of a second after they are measured. At walking speed (about 1 m/s), every sample used by `mm_recorder` and `mm_calibrate` is therefore paired with the wrong map pose, by 10 to 30 cm. That is the same order as the 0.15 m acceptance threshold.

What to do:

1. Measure the delay. For example, move the hedgehog back and forth and cross-correlate the Marvelmind track with the map pose from slam_toolbox or AMCL.
2. Stamp each message with the measurement time. Either use the device `timestamp_ms` with an estimated offset to ROS time, or subtract the measured delay.
3. Make the delay a parameter, and report it in the README.

### 2b. Calibrate after mapping, not during it

`mm_recorder` pairs Marvelmind positions with map poses while the map is still being built, before loop closures correct it. If the map changes afterwards, the alignment keeps the old, wrong poses.

What to do: record the calibration samples **after** mapping. Load the finished map, localize with AMCL, drive or walk around the area, and run `mm_calibrate` on those samples. Update the README procedure accordingly.

### How to evaluate

Calibrate twice with the same data, before and after the fix. Report the RMS, the 95th percentile and the inlier ratio from `mm_calibrate`.

---

## Task 3: use Marvelmind to measure map quality

**Priority: medium.** This gives an objective number for the thesis without any extra sensor.

### Idea

Where the beacons are installed, Marvelmind gives absolute positions that do not depend on the lidar. Once the map is aligned to Marvelmind (Task 2), you can compare where the map says the robot is with where Marvelmind says it is. A map that is bent, stretched or shortened shows up as an error that grows along the path.

### What to implement

A tool, for example `ros2 run kelo_bringup map_check <bag>`, that:

1. replays a bag recorded while localizing on a finished map;
2. pairs AMCL poses with time-corrected Marvelmind positions (Task 2a);
3. reports mean, RMS, 95th percentile and maximum error;
4. plots the error over the map, so that you can see *where* the map is distorted.

### How to evaluate

Use it on maps built with different settings, for example the lidar only, the two-layer map, and different slam_toolbox parameters, and compare the numbers.

---

## Task 4 (later, optional): Marvelmind as a continuous correction

**Priority: low.** Start this only after Tasks 1–3.

Today Marvelmind is used only for the initial pose and as a safety alarm, which stops the robot when AMCL diverges by more than 1 m. The next step is to use it as a continuous correction when available. Two possible approaches:

- fuse the AMCL pose and the Marvelmind position in a filter, for example `robot_localization`, and publish the corrected `map → odom`;
- or re-seed AMCL with Marvelmind when the AMCL uncertainty grows.

**Requirement:** the system must work in the same way with Marvelmind absent or without signal: **better when present, never worse when missing**. Show this with the same route run with and without Marvelmind, using the Task 3 tool to measure the error.

---

## General notes

- Keep every new feature switchable from `hardware.yaml`, as you already do for the sensors.
- Record the bags you use for every result and keep them: they let us repeat the comparison later.
- On the KELO, the odometry comes from the KELO PC and the scans from your Jetson, so the two clocks must be synchronized (chrony) before any mapping test.
- Use `odom_mode: driver` on the KELO. The correction table in `hardware.yaml` was not measured on the robot.
- Start each task with a short plan, a few lines on what you will do and how you will test it, and share it with us before writing the code.
