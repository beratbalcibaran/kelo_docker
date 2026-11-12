# Paper idea: I2MTC 2027

We would like you to write a paper for **IEEE I2MTC 2027** (International Instrumentation and
Measurement Technology Conference). **Full paper deadline: 4 December 2026. Maximum 6 pages.**

I2MTC is a measurement conference. Reviewers look for a measurement contribution: characterisation,
calibration, uncertainty, and comparison against a reference.

OptiTrack is the **reference**, Marvelmind is the **instrument under test**. The paper uses only
Marvelmind data (no KELO odometry). **No more recordings are planned**: the paper is written with the
data of 12 November (afternoon and evening).

Earlier versions of this plan are in the git history (latency + LiDAR maps: `18f4e6e`; first
OptiTrack plan and the analysis of the morning data: `92a3a74`).

---

## The story in one sentence

*We characterise a low-cost ultrasonic indoor positioning system (Marvelmind) on a mobile robot
against a motion-capture reference (OptiTrack), show how a single wrong configuration parameter (the
height of one beacon) turns centimetre errors into errors of tens of centimetres, and correct the
recorded data afterwards without repeating the measurements.*

Working title: *Metrological Characterisation of a Low-Cost Ultrasonic Indoor Positioning System on
a Mobile Robot and Correction of a Beacon Configuration Error*.

Contributions:

1. **Characterisation** of Marvelmind on the KELO against OptiTrack: at rest (precision vs accuracy,
   Allan deviation, offset at repeated visits, effect of the heading), while moving (error vs speed),
   and when leaving and re-entering the OptiTrack area.
2. **Effect of a configuration error**: with beacon 1 at 0 m instead of 2.6 m in the map, wrong 3D
   solutions in 4–49 % of the samples, median errors up to 36 cm and p95 up to 150 cm.
3. **Correction afterwards**: the recorded positions are recomputed with the corrected map, using the
   known hedgehog height as a check and **without the reference**; validated against OptiTrack.

---

## Rules for this paper

- **Real data only** in the results.
- Every number needs an uncertainty and a stated number of repetitions.
- Keep every recording used for a result, and write in a small table which recording produced which
  figure (see `paper/FIGURES.md`).
- Analysis runs **offline**, with scripts that anyone can run again (`paper/scripts/`).
- **Never fit and test on the same data.** The alignment is fitted on the evening trials only (and the
  early lever arm on the afternoon rotations); triangles, in/out and long rests are test data.
- Every figure must be readable in black and white and at column width.

---

## Data (12 November 2026)

All archives are in `experiments/archives/2026-11-12/`, described in `experiments/README.md`.

| Recording | Session | What |
|---|---|---|
| `test1`, `test2` | afternoon | rotation in place, 4 headings, twice |
| `test2_triangle`, `test4_triangle` | afternoon | triangle, fast and slow |
| `test5_in_out1` | afternoon | out of the OptiTrack area and back, 10 laps (5 slow, 5 fast) |
| `test6_wait` | afternoon | 18 min at rest |
| `deneme_2025`, `deneme_2028`, `deneme_triangle_2031` | evening, map corrected | rotation in place twice, triangle |
| `denemenew_wait_2040` | evening, map corrected | 10 min at rest |

What happened, and has to be described in the paper:

- **Beacon 1 height.** In the afternoon the map used beacon 1 at 0 m instead of 2.6 m (the value shown
  in the Dashboard had not been applied). Corrected in the evening (map of 20:24).
- **Hedgehog moved** between `test4_triangle` (18:29) and `test5_in_out1` (18:51), by 52 cm according
  to the data (about 45 cm with the tape). The two mountings have different lever arms.
- **Morning data (Experiment 0) are not used**: rigid body re-created several times, other hedgehog
  position, different map.

## Main results (details and figures in `paper/FIGURES.md`)

- **Correction of the afternoon data** (`paper/scripts/b1_correction.py`): median error from 13–36 cm
  to 3–10 cm, p95 from 50–150 cm to 10–20 cm; 81–100 % of the samples remain valid. One stop out of 23
  failed (167 cm) and is reported.
- **At rest: precision ≪ accuracy.** Jitter about 4 mm, Allan deviation 1.6 mm at 1 s and 0.2 mm at
  60 s, but offset to the reference 2.6–7.4 cm. Averaging does not improve the accuracy.
- **Repeated visits**: mean offset per vertex 3.7–10.5 cm, spread between visits 3–9 cm, jitter within a
  stop about 1.4 mm. The offset also depends on the heading in a repeatable way (rotation in place:
  0.4–14 cm).
- **While moving**: RMS 9–16 cm, p95 18–22 cm; no clear dependence on the speed up to 0.6 m/s after
  the correction. Delay 0.09 s.
- **In/out**: error before leaving 5.3 cm, after re-entry 4.3 cm (median, 10 re-entries); spread at the
  outside points 6–10 cm (upper bound, includes the stopping position of the robot).

---

## Method

1. **Correction of the map error** (afternoon only): rebuild the distances to each triple of beacons
   from the recorded position and the wrong map, recompute with the corrected map, keep the triple
   that gives the known hedgehog height with a continuous track (dynamic programming). The reference
   is not used. Validation: a Dashboard sample of the measured distances fits the corrected map within
   4 mm; the corrected data are compared with OptiTrack.
2. **Alignment** OptiTrack → Marvelmind: 2D rotation, translation and hedgehog lever arm, fitted on
   the stops of the evening trials only; lever arm of the first mounting fitted on the afternoon
   rotations with rotation and translation fixed. Robust least squares (soft L1).
3. **Delay**: 0.09 s, fitted on the evening triangle.
4. **Data selection**: OptiTrack frames with mean marker error > 3 mm are discarded; stops = OptiTrack
   speed < 2 cm/s and rotation < 3 °/s for at least 10 s, 2 s trimmed at both ends.
5. **Metrics**: 2D error to the OptiTrack hedgehog position; median, RMS, p95, CDF; per-stop offset and
   jitter; Allan deviation; error vs speed; error before leaving / after re-entry.
6. **Uncertainty budget**: OptiTrack (static noise below 0.1 mm, marker error about 1 mm), lever arm
   (from the fit and the tape), delay residual × speed, temperature (speed of sound about 0.17 %/°C;
   the Dashboard uses 23 °C, the room temperature was not measured).

The temporal filters tried on the morning data (Kalman, RTS smoother) gained at most 1 cm because the
error is a slow bias; mention it in one sentence, no figure needed.

---

## Paper outline (6 pages)

| Section | Content | Pages |
|---|---|---|
| I. Introduction | Low-cost indoor positioning on robots, why configuration matters, contributions | 0.75 |
| II. Related work | Evaluations of Marvelmind and similar systems, motion capture as reference, trajectory metrics (ATE) | 0.5 |
| III. Experimental setup | Room and beacons, OptiTrack, KELO, rigid body and hedgehog, logging and synchronisation. **Fig. 1** | 1 |
| IV. Method | Correction of the map error, alignment, delay, data selection, metrics, uncertainty | 1 |
| V. Results | A. effect of the configuration error and its correction (**Table I**, Fig. 3, opt. Fig. 2); B. at rest (Fig. 4, 6); C. while moving (Fig. 5 or one sentence); D. outside the OptiTrack area (Fig. 7) | 2 |
| VI. Discussion and conclusion | Lessons for users (check the hedgehog height in z, enable raw distances), limits: one room, 2D, joystick paths, small OptiTrack area | 0.5 |
| References | about 15 entries | 0.25 |

The LaTeX file is `paper/main.tex`; the figures are already included.

---

## Timeline

| Dates | Work |
|---|---|
| by 20 Nov | Introduction, related work (find the Marvelmind evaluation papers), setup, method; photos of the robot and the room for Fig. 1 |
| by 27 Nov | Results and discussion, complete first draft |
| 1–4 Dec | Review with Andrea, corrections, submission |

## For future sessions (lessons from 12 November)

- Check in the Dashboard that the **hedgehog z reads its real height** before recording: it reveals a
  wrong beacon height at once.
- Turn on the **raw distances** (`COMM_Raw_distances`) and log them, so that any configuration error
  can be corrected exactly afterwards.
- Charge the beacons: in the evening beacon 1 was at 2.58 V and sleeping.
- Define the Motive rigid body once; measure the hedgehog lever arm with a tape and do not move it
  during a session (or write down when it is moved).
- Write down the room temperature.
