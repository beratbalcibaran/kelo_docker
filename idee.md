# Paper idea: I2MTC 2027

We would like you to write a paper for **IEEE I2MTC 2027** (International Instrumentation and
Measurement Technology Conference). **Full paper deadline: 4 December 2026. Maximum 6 pages.**

I2MTC is a measurement conference. Reviewers look for a measurement contribution: characterisation,
calibration, uncertainty, and comparison against a reference.

This plan replaces the previous one (Marvelmind latency + LiDAR map accuracy, commit `18f4e6e`).
Now that OptiTrack is available, OptiTrack is the **reference** and Marvelmind is the **instrument
under test**.

---

## The story in one sentence

*We characterise a low-cost ultrasonic indoor positioning system (Marvelmind) on a mobile robot
against a motion-capture reference (OptiTrack), calibrate it with the reference where the reference
is available, and measure how well the calibration holds where it is not.*

Working title: *Metrological Characterisation and Motion-Capture-Based Calibration of a Low-Cost
Ultrasonic Indoor Positioning System on a Mobile Robot*.

Contributions:

1. Characterisation of Marvelmind on the KELO against OptiTrack: error at rest and while moving,
   error vs speed, outliers, repeatability over 12 runs of the same path.
2. A calibration procedure based on OptiTrack, in levels of increasing complexity, validated with
   leave-one-run-out cross-validation on 24 runs.
3. Use of the calibration outside the OptiTrack area: error on surveyed floor points and at
   re-entry.

---

## Rules for this paper

- **Real data only** in the results.
- Every number needs an uncertainty and a stated number of repetitions.
- Keep every recording used for a result, and write in a small table which recording produced which
  figure.
- Analysis runs **offline**, with scripts that anyone can run again.
- **Never fit and test on the same data.** Registration and correction are estimated on calibration
  runs only and evaluated on the other runs.
- Every figure must be readable in black and white and at column width.

## Setup (before each session)

1. **Fixed mounts.** Hedgehog and OptiTrack marker sets fixed with screws, not velcro, so that their
   relative position does not change between runs. If this is not possible, measure again at every
   session.
2. **Lever arm.** Measure the (x, y) position of the hedgehog with respect to the pivot of the low
   3-marker rigid body, in the robot frame. Write down the pivot used in Motive. The lever arm is
   applied with the OptiTrack heading to bring the reference to the hedgehog position.
3. **Temperature** of the room at every session (speed of sound: about 0.17 % per °C), and the
   temperature setting in the Marvelmind Dashboard.
4. **Marvelmind configuration** (firmware, update rate, filters, IMU fusion on/off): keep it fixed,
   or change it on purpose and report it.
5. **OptiTrack calibration residual** from Motive, saved with the data.
6. **Floor checkpoints outside the OptiTrack area**: 2–3 points, measured once with a laser distance
   meter with respect to the OptiTrack frame.
7. **Synchronisation** between the Motive CSV and the Marvelmind data: already in place. Describe the
   method and its residual in the paper.

---

## Experiments

| | Test | Runs | Purpose |
|---|---|---|---|
| **E1** | Triangle driven with the joystick, stops of at least 10 s at the vertices | 12 | Error at rest (vertices), error while moving, repeatability |
| **E2** | Free paths inside the OptiTrack area, with some stops | 12 | Test data for the calibration, generality |
| **E3** | OptiTrack area → outside (stops on the floor checkpoints) → back inside | about 5 | Calibration used where the reference is not available |

- The triangles are not identical (joystick): this is fine, Marvelmind is compared with OptiTrack
  sample by sample, not with a nominal path.
- Vary the speed on purpose during E1 and E2. The error vs speed is then obtained by binning on the
  speed measured by OptiTrack, without extra runs.
- Stops are detected from the OptiTrack speed (below a threshold for at least 10 s). State the
  thresholds in the paper.

---

## Analysis

1. **Registration**: 2D rigid transform (x, y, yaw) plus a z offset between the OptiTrack and
   Marvelmind frames, with the lever arm applied. Estimated on calibration runs only. This is needed
   to compare the two systems at all; it is **not** the correction.
2. **Baseline error** (after registration):
   - at rest: bias and standard deviation at each stop point;
   - while moving: RMSE, 95th percentile, CDF, error vs speed;
   - rate of outliers and dropouts.
3. **Correction in levels**:
   - (a) registration only;
   - (b) + scale or affine transform;
   - (c) + a position-dependent error map (valid only inside the OptiTrack area).

   Leave-one-run-out on the 24 runs of E1 + E2: fit on 23, test on the one left out, repeat for all
   24. Report how much each level improves the error on the test runs.
4. **Outside the OptiTrack area (E3)**: only the global corrections (a, b). Error on the floor
   checkpoints and at re-entry.
5. **OptiTrack uncertainty**: all marker sets are on the same rigid robot, so the distance between
   two sets must be constant. Its spread is an experimental estimate of the OptiTrack uncertainty on
   this setup.

**Uncertainty budget.** Combine: OptiTrack uncertainty (point 5), lever arm, synchronisation residual
× speed, temperature effect on the speed of sound.

Related metric to cite: absolute trajectory error (ATE), TUM RGB-D benchmark, Sturm et al. 2012.

---

## Paper outline (6 pages)

| Section | Content | Pages |
|---|---|---|
| I. Introduction | Low-cost indoor positioning, reference available only in part of the area, contributions as a short list | 0.75 |
| II. Related work | Evaluations of Marvelmind and similar systems, motion capture as reference, calibration of indoor positioning systems, ATE | 0.5 |
| III. Experimental setup | Room and beacon geometry, OptiTrack (cameras, calibration residual), KELO, marker sets, hedgehog and lever arm, synchronisation, temperature. **Fig. 1:** photo and floor plan with the OptiTrack area | 1 |
| IV. Method | Frames and registration, stop detection, metrics, correction levels a/b/c, leave-one-run-out, uncertainty budget | 1 |
| V. Results | A. at rest; B. while moving and vs speed; C. correction (CDF for levels a/b/c); D. outside the OptiTrack area. **Figures:** triangles overlaid, CDF, error vs speed, path with the area outside OptiTrack. **One summary table** | 2 |
| VI. Discussion and conclusion | Limits: one room, 2D only, joystick paths, fixed beacon geometry | 0.5 |
| References | about 15 entries | 0.25 |

Write in IEEE conference format (LaTeX template from the conference site).

---

## Timeline

| Dates | Work |
|---|---|
| by 20 Nov | All recordings: E1 (12), E2 (12), E3 (about 5) |
| by 27 Nov | Analysis, figures, complete first draft |
| 1–4 Dec | Review with Andrea, corrections, submission |

Share a short status every few days (what is recorded, what is analysed, first numbers).
