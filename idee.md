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
2. A processing chain for the Marvelmind data (registration, outlier rejection, temporal filter,
   optionally refined beacon positions), tuned with OptiTrack and validated with leave-one-run-out
   cross-validation. The filter is evaluated as a noise vs lag trade-off as a function of speed.
3. Use of the processing chain outside the OptiTrack area: error on surveyed floor points and at
   re-entry.

**Marvelmind data only.** The paper does not use the KELO odometry.

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
2. **Rigid body defined once.** On 12 November the Motive rigid body was re-created several times,
   also in the middle of recordings (pivot height 0.59, 0.69, 0.71, 0.81 m), so most recordings had
   to be aligned piece by piece. Define it once, save it, and never re-create it. The current
   definition has 11 markers, while 13 are mounted: include all of them.
3. **Lever arm.** Hedgehog position with respect to the rigid body pivot (now the centroid of the
   markers), in the robot frame. It is applied with the OptiTrack heading to bring the reference to
   the hedgehog position. Estimated from the data: 15.3 cm (run 1) and 18.5 cm (last 40 points).
   Tape measure on 12 November: hedgehog 40 cm from the front markers; the upper markers are 20 cm
   further forward and 23–24 cm higher. This is consistent with the estimate. For an exact value,
   measure the hedgehog against two markers that can be identified in Motive and use their
   coordinates in `optitrack_natnet_data_descriptions.txt`.
4. **Marvelmind Dashboard.** Beacon heights are entered as 240 (m) instead of 2.40, so the reported z
   is 238 m: measure the real heights and fix them. The Marvelmind map is mirrored with respect to
   OptiTrack: fix it in the Dashboard or state it in the paper.
5. **Temperature** of the room at every session (speed of sound: about 0.17 % per °C). The
   Dashboard setting is 23 °C.
6. **Marvelmind configuration** (firmware, update rate, filters, IMU fusion on/off): keep it fixed,
   or change it on purpose and report it.
7. **OptiTrack calibration residual** from Motive, saved with the data.
8. **Floor checkpoints outside the OptiTrack area**: 2–3 points, measured once with a laser distance
   meter with respect to the OptiTrack frame.
9. **Synchronisation**: both systems are logged on the same PC by `record.py`; the Marvelmind device
   clock drifts by 0.92 % and is corrected. Describe the method and its residual in the paper.

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
- The OptiTrack V120:Trio sees up to about 2.9 m, so the covered area is about 2.7 × 2.7 m. The
  triangle must stay inside it. This area is in a corner of the beacon quadrilateral (about 3 m from
  beacons 1 and 4, 6 m from beacon 2, 8 m from beacon 5): say so in the paper, and keep it in mind
  for E3, where the beacon geometry is different.

---

## Status on 12 November

Reports in `docs/data_of_all_tests/`. Usable recordings: **run 1** (`tur1_v3`, 12 points) and the
**last 40 points** (repeat test). The others changed rigid body during the recording or covered a
very small area. These two can be used in the paper with separate alignments, stated as such.

- **E1 is in practice already recorded.** In the last 40 points the stops fall in three areas, the
  vertices of the triangle: about (3.1, 7.3) m with 13 visits, (4.5, 5.9) m with 11, (5.4, 7.5) m
  with 12, plus 4 single stops. Within a vertex the stops spread over 30–40 cm (joystick).
- **Run 1 is E2 material**: 5 visits to the start point and 7 different points.
- **Error at rest**: RMS 4.3 cm (run 1) and 9.8 cm (last 40 points); per-stop jitter median 2–3 cm.
- **Error while moving** (samples between the stops, OptiTrack speed > 3 cm/s): RMS 7.5 cm, p95
  10.8 cm (run 1, 66 s); RMS 13.2 cm, p95 27.2 cm (last 40 points, 324 s). Flat up to 0.4 m/s,
  higher above (19–22 cm, few samples).
- **Scale** 0.98–1.006 in every reliable recording: no scale error.
- **The error at the same vertex does not repeat between visits.** Mean offset per vertex 0–5 cm,
  spread between visits 4–7 cm. The robot reached the vertices with different headings, but the
  lever arm was fitted on the same data, so the geometric effect is already absorbed: the spread is
  attributed to Marvelmind. The systematic, position-dependent part is small; the random part
  dominates. A position-dependent error map would not help, so the processing chain below targets
  the random part.
- Outliers (> 20 cm from the stop median): 1.8 % of the samples in the last 40 points, most of them
  at one point.

These numbers were computed from the data embedded in the reports, with the alignment fitted on the
same stops: they are indicative and must be recomputed with the final processing.

---

## Analysis

1. **Registration**: 2D rigid transform (x, y, yaw) plus a z offset between the OptiTrack and
   Marvelmind frames, with the lever arm applied. Estimated on calibration runs only. This is needed
   to compare the two systems at all; it is **not** the correction.
2. **Baseline error** (after registration):
   - at rest: bias and standard deviation at each stop point;
   - while moving: RMSE, 95th percentile, CDF, error vs speed;
   - rate of outliers and dropouts.
3. **Processing chain** (Marvelmind data only, each step evaluated on its own):
   - (1) registration only (baseline);
   - (2) + outlier rejection (jumps), with a threshold tuned on OptiTrack;
   - (3) + temporal filter: moving average at rest, or a constant-velocity Kalman filter. It reduces
     the random part but adds lag while moving. Use OptiTrack to measure this noise vs lag trade-off
     as a function of speed, and to choose the parameters;
   - (4) optional: beacon positions refined with OptiTrack. In the Marvelmind map the distance
     B2–B4 differs by 4.7 cm from the measured one, the other pairs by 0.1 cm. This is a physical
     correction, so it also holds outside the OptiTrack area.

   Leave-one-run-out on the runs of E1 + E2: tune on all runs but one, test on the one left out,
   repeat for every run. Report how much each step improves the error on the test runs.
4. **Outside the OptiTrack area (E3)**: steps (1)–(3) and, if done, (4). Error on the floor
   checkpoints and at re-entry.
5. **OptiTrack uncertainty**: the rigid body is one 11-marker set, so use the mean marker error from
   Motive and the static noise at the stops (median 0.3–0.5 mm on 12 November). Discard frames with a
   high marker error: in the last 40 points its p95 is 14.6 mm.

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
| IV. Method | Frames and registration, stop detection, metrics, processing chain (1)–(4), leave-one-run-out, uncertainty budget | 1 |
| V. Results | A. at rest (systematic vs random part at the vertices); B. while moving and vs speed; C. processing chain (CDF for each step, noise vs lag vs speed); D. outside the OptiTrack area. **Figures:** triangles overlaid, CDF, error and lag vs speed, path with the area outside OptiTrack. **One summary table** | 2 |
| VI. Discussion and conclusion | Limits: one room, 2D only, joystick paths, fixed beacon geometry | 0.5 |
| References | about 15 entries | 0.25 |

Write in IEEE conference format (LaTeX template from the conference site).

---

## Timeline

| Dates | Work |
|---|---|
| by 20 Nov | With the rigid body fixed: more E2 runs, E3 (about 5), and E1 again if the 12 November data are not enough |
| by 27 Nov | Analysis, figures, complete first draft |
| 1–4 Dec | Review with Andrea, corrections, submission |

Share a short status every few days (what is recorded, what is analysed, first numbers).
