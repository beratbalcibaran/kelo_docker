# Paper idea: I2MTC 2027

We would like you to try writing a paper for **IEEE I2MTC 2027** (International Instrumentation and
Measurement Technology Conference). **Full paper deadline: 4 December 2026.**

I2MTC is a measurement conference. Reviewers there look for a measurement contribution:
characterisation, calibration, uncertainty, and comparison against a reference. A description of
the navigation stack alone would not fit. The plan below uses the Marvelmind work from
`NEXT_STEPS.md` (Tasks 2 and 3), with real data and a clear measurement story.

---

## The story in one sentence

*We characterise and compensate the latency of a low-cost ultrasonic indoor positioning system, then
use it as an independent reference to measure the geometric accuracy of 2D LiDAR maps.*

Experiment 1 is needed for Experiment 2: without the latency compensation the reference has an error
of 10–30 cm when moving, which is larger than the map errors we want to measure.

Working title: *Latency Characterisation of an Ultrasonic Indoor Positioning System and its Use as a
Reference for Assessing the Accuracy of Low-Cost LiDAR Maps*.

---

## Rules for this paper

- **Real data only** in the results. Simulator results can appear only as a short remark, not as a
  contribution.
- **Do not use the odometry values from the README** (one turn read as 68°, k 5.3 / 8.6). They were
  never measured on the robot.
- All data can be recorded with the **sensor cart**. The KELO is not needed.
- Every number needs an uncertainty and a stated number of repetitions.
- Keep every bag used for a result, and write in a small table which bag produced which figure.
- Analysis runs **offline on bags**, with scripts that anyone can run again.

## Prerequisites (first days)

1. **Beacons installed and working** in the test area (at least 4, good line of sight, area large
   enough for a loop of several metres). Enable `sensors: marvelmind` in `hardware.yaml`.
2. **Hedgehog lever arm.** `hardware.yaml → mounts.hedgehog` is still `x: 0.0, y: 0.0`. Measure
   the real position of the hedgehog with respect to `base_link` on the cart (tape measure, ±1 cm)
   and enter it. This offset enters the uncertainty budget.
3. Note the Marvelmind configuration used (firmware, update rate, filtering settings, serial baud
   rate: currently 9600). Keep it fixed during all recordings, or change it on purpose and report it.
4. A finished map of the test area (needed for Experiment 1 with AMCL, and for Experiment 2).

---

## Experiment 0: characterise the reference

Without this, a reviewer will not accept the Marvelmind as a reference.

**Question.** How repeatable and how accurate is the Marvelmind position in our area?

**Procedure.**

1. Mark 8–10 points on the floor across the area. Measure the distances between them with a laser
   distance meter.
2. Place the hedgehog on each point for 60 s, without moving.
3. Repeat the whole sequence at least twice (different times or days).

**Analysis.**

- **Repeatability:** standard deviation of the position at each point. Check whether it changes
  across the area (beacon geometry).
- **Accuracy:** distances between points from the Marvelmind vs from the laser meter. Report mean
  error and RMS.
- Dropouts and outliers: rate of `FLAG_NO_DATA` and of jumps.

**Rule of thumb.** The reference uncertainty should be at least 3–4 times smaller than the errors we
want to measure. If Marvelmind is at 2–3 cm and the map errors are at 5–20 cm, the method works. If
they are of the same order, we say so in the paper as a limitation.

---

## Experiment 1: Marvelmind latency

**Question.** How long is it between the moment the Marvelmind measures a position and the moment it
arrives in ROS? How much does this delay affect the alignment with the map, and how much does the
compensation improve it?

**Why it matters.** `marvelmind_bridge.py` stamps each position with `now()` when it arrives
(`_cb`, `out.header.stamp = self.get_clock().now()`). The delay τ includes the ultrasonic cycle, the
filtering inside the hedgehog, the radio link to the modem and the serial link. At speed v, every
sample is paired with a map pose that is wrong by about **v·τ**: with τ = 0.3 s this is 30 cm at
walking speed, above the 0.15 m acceptance threshold of `mm_calibrate`.

**Setup.** Sensor cart with Hokuyo and hedgehog, everything on the Jetson (one clock, no
synchronisation problem). Reference: the LiDAR pose on the finished map (AMCL), or the slam_toolbox
pose. Record a bag with `/scan`, `/hedgehog_pos_ang` (it also contains the device `timestamp_ms`),
TF and the AMCL pose.

**Procedure.**

- Back-and-forth movements along a straight line (3–4 m), plus a few closed loops.
- Three speeds, for example 0.2, 0.5 and 1.0 m/s. About **5 repetitions per speed**.
- Optional: two Marvelmind update rates, to see if τ depends on the configuration.

**Analysis** (offline, two independent methods that check each other):

1. **τ sweep.** For each τ from 0 to 1 s in steps of 5 ms: pair each Marvelmind sample with the map
   pose interpolated at t − τ, fit the rigid alignment with `kabsch_2d` (`logic.py`), compute the
   RMS. The estimate τ\* is the minimum of RMS(τ).
2. **Cross-correlation of velocities.** Project both velocity signals on the main motion axis and
   cross-correlate them. This method does not need the alignment.

Then:

- Check whether τ\* is constant across speeds. Constant: pure transport delay. Growing with speed:
  there is a filter in the device.
- Use the device `timestamp_ms`: a linear fit of arrival time vs device time gives offset and drift
  (use the lower envelope of the arrival times). Compare the jitter of arrival-time stamps with the
  jitter of device-time stamps.

**Uncertainty.** Spread of τ\* across repetitions (type A), sweep step, and the latency of the
reference itself: the URG-04LX scan timestamp has its own offset, so we measure the latency
**relative to the LiDAR pose**. Say this clearly in the paper.

**Implementation.**

- An offline analysis script (bag → τ sweep, cross-correlation, plots).
- In `marvelmind_bridge`: a `delay_s` parameter that is subtracted from the arrival time, or a stamp
  from `timestamp_ms` plus the estimated offset. Keep it switchable, as with every other feature.
- `mm_recorder` then looks up the TF at the corrected stamp without any other change.

**Results for the paper.**

- Plot of RMS(τ) with its minimum, for each speed.
- **Main figure:** alignment error vs speed, before and after the compensation. Before: it grows
  linearly with slope ≈ τ. After: it is flat.
- Table: RMS, 95th percentile and inlier ratio from `mm_calibrate`, with and without the
  compensation, for each speed.

---

## Experiment 2: map accuracy with Marvelmind as the reference

**Question.** How geometrically distorted is a map built with a low-cost 2D LiDAR (bent, stretched,
shortened), and where is the error?

**Important.** The comparison "LiDAR map vs two-layer map" does **not** work here: the camera layer is
added on top of the map but does not change its geometry (poses come from the LiDAR, AMCL localizes on
the LiDAR map). The error against the Marvelmind would be identical. Compare configurations that do
change the geometry (see below).

**Procedure.**

1. **One mapping bag** with a long path and at least one loop.
2. Build several maps **offline from the same bag**, so that the raw data are identical and only the
   algorithm changes. Suggested configurations:
   - loop closure on vs off;
   - slam_toolbox default parameters vs tuned;
   - optional: the same area mapped slowly and quickly (two bags).
3. For each map: a **localization run with AMCL** along a path that covers the whole area, with the
   time-corrected Marvelmind (Experiment 1).

**Metrics** (a `map_check` tool, as described in `NEXT_STEPS.md` Task 3):

- Rigid alignment Marvelmind → map on all samples. The residual is the **non-rigid distortion** of
  the map. This is the same idea as the absolute trajectory error (ATE) of the SLAM benchmarks (TUM
  RGB-D benchmark, Sturm et al. 2012): cite it.
- Mean, RMS, 95th percentile and maximum of the residual.
- A second alignment **with a scale factor** (similarity transform). A scale different from 1 means
  that the map is stretched or shortened: report it in %.
- **Error map** over the area (heatmap): shows where the map is distorted.
- The same analysis on the slam_toolbox trajectory **during mapping**, from the same bag.

**Uncertainty budget.** Combine:

- Marvelmind repeatability and accuracy (Experiment 0);
- residual latency: uncertainty of τ (Experiment 1) × speed;
- hedgehog lever arm (±1 cm);
- AMCL noise (spread of repeated localization runs on the same map).

**Optional: geometric check of the camera layer.** Use the hedgehog as a **probe**: place it on the
corners of tables and low objects, and compare the measured position with the position of the same
obstacle in the camera layer. This brings the two-layer map into the paper with a real measurement
instead of a cell count.

---

## Timeline

| Week | Work |
|---|---|
| 10–16 Nov | Prerequisites, Experiment 0, latency analysis script, Experiment 1 bags |
| 17–23 Nov | Latency analysis and compensation in the bridge; mapping and localization bags; **start writing** (introduction, setup, method) |
| 24–30 Nov | `map_check` and Experiment 2 analysis, figures, results section |
| 1–4 Dec | Review with Andrea, final corrections, submission |

Share a short status every few days (what is recorded, what is analysed, first numbers).

**Fallback.** If Experiment 2 has no reliable data by 23 November, the paper becomes Experiments 0
and 1 only: *metrological characterisation of a low-cost ultrasonic IPS for mobile robotics*. That is
still a valid I2MTC paper.

---

## Paper outline (check the page limit in the I2MTC 2027 author instructions)

1. **Introduction.** Low-cost mapping, no ground truth in real indoor environments, latency as a
   hidden error source of IPS references. Contributions as a short list.
2. **Related work.** Indoor positioning systems as ground truth, latency and time-stamping in sensor
   fusion, map and trajectory accuracy metrics (ATE).
3. **System.** Sensor cart, sensors, software, data flow and time stamps (one block diagram).
4. **Method.** Experiment 0, latency estimation (both methods), map accuracy metric, uncertainty
   budget.
5. **Experimental results.** One subsection per experiment, with the figures and tables listed
   above.
6. **Discussion.** Limits: relative latency, beacon geometry, 2D only, single environment.
7. **Conclusion.**

Write in IEEE conference format (LaTeX template from the conference site). Every figure must be
readable in black and white and at column width.
