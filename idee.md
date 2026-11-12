# Paper idea: I2MTC 2027

We would like you to write a paper for **IEEE I2MTC 2027** (International Instrumentation and
Measurement Technology Conference). **Full paper deadline: 4 December 2026. Maximum 6 pages.**

I2MTC is a measurement conference. Reviewers look for a measurement contribution: characterisation,
calibration, uncertainty, and comparison against a reference.

OptiTrack is the **reference**, Marvelmind is the **instrument under test**. The paper uses only
Marvelmind data (no KELO odometry). **No more recordings are planned**: the paper uses the two
measurement campaigns of 12 November.

Earlier versions of this plan are in the git history (latency + LiDAR maps: `18f4e6e`; first
OptiTrack plan and the analysis of the Experiment 0 data: `92a3a74`).

---

## The story in one sentence

*We characterise a low-cost ultrasonic indoor positioning system (Marvelmind) on a mobile robot
against a motion-capture reference (OptiTrack) in two campaigns at the same robot poses, show that a
single wrong configuration parameter (the height of one beacon) multiplies the errors by 3–10, and
recompute the recorded positions afterwards, without the reference, so that they match the campaign
recorded with the correct configuration.*

Working title: *Metrological Characterisation of a Low-Cost Ultrasonic Indoor Positioning System on
a Mobile Robot and Correction of a Beacon Configuration Error*.

Contributions:

1. **Characterisation** of Marvelmind on the KELO against OptiTrack: at rest (precision vs accuracy,
   Allan deviation, offset at repeated visits, effect of the heading), while moving, and when leaving
   and re-entering the OptiTrack area.
2. **Effect of a configuration error**: with beacon 1 at 0 m instead of 2.6 m in the map, median
   errors up to 36 cm and p95 up to 1.4 m.
3. **Correction afterwards and its validation**: the recorded positions are recomputed with the
   correct map, using the known hedgehog height as a check and **without the reference**. Validation by
   design: a second campaign at the same robot poses with the correct configuration. The corrected
   first campaign and the second campaign are statistically indistinguishable.

---

## Rules for this paper

- **Real data only** in the results.
- Every number needs an uncertainty and a stated number of repetitions.
- Keep every recording used for a result, and write in a small table which recording produced which
  figure (see `paper/FIGURES.md`).
- Analysis runs **offline**, with scripts that anyone can run again (`paper/scripts/`).
- **Never fit and test on the same data.** The alignment is fitted on campaign 2 only (and the lever
  arm of mounting 1 on the campaign 1 rotations); triangles, out-and-back and rests are test data.
- Every figure must be readable in black and white and at column width.
- In the paper, call the two campaigns **Campaign 1 (main test programme)** and **Campaign 2
  (reference configuration)**.

---

## Experimental design (12 November 2026)

All archives are in `experiments/archives/2026-11-12/`, described in `experiments/README.md`.

| Test | Campaign 1 (main programme) | Campaign 2 (reference configuration) |
|---|---|---|
| Rotation in place, 4 headings | 2 runs | 2 runs |
| Triangle, same 3 vertices and headings | fast + slow | 1 run |
| Rest | 18 min | 10 min |
| Out-and-back (leaves the OptiTrack area) | 10 laps, 5 slow + 5 fast | – |
| Marvelmind map | beacon 1 height not applied (0 m instead of 2.6 m), corrected afterwards | correct |
| Role | characterisation, effect of the configuration error | alignment fit, validation of the correction |

- The **robot poses are the same** in the two campaigns (vertices and headings), so the campaigns can
  be compared stop by stop.
- **Hedgehog mounting**: the hedgehog was remounted during campaign 1 (mounting 1: rotations and
  triangles of C1; mounting 2: out-and-back, rest of C1 and all of C2). The lever arm is estimated for
  each mounting (52 cm apart; tape about 45 cm). Describe it as part of the setup.
- The Experiment 0 data (`docs/data_of_all_tests/`) are not used.

## Main results (details, figures and numbers in `paper/FIGURES.md`)

- **Configuration error and correction (Table I, Fig. 3).** Stop offset median as recorded / corrected
  / C2: rotation 16.6 / 4.0 / 7.4 cm, triangle 20.7 / 7.8 / 4.9 cm; triangle moving RMS 65.4 / 11.1 /
  10.8 cm, p95 142 / 21.2 / 21.6 cm. Differences C1 corrected − C2 of the median offsets: −3.4 cm
  [−10.1, 2.2] and 2.8 cm [−5.1, 5.4] (95 % bootstrap): no significant difference. 81–100 % of the
  samples remain valid; one stop out of 23 failed (167 cm) and is reported.
- **At rest (Fig. 4, 6).** Same noise in both campaigns: jitter 4 mm, Allan deviation 1.6 mm at 1 s and
  0.2 mm at 60 s; offset to the reference 2.6 and 7.4 cm. Offset per vertex 1–10 cm, spread between
  visits 3–9 cm; repeatable heading effect in both campaigns (−44°: 0.4–4.7 cm, −134°: 10–14 cm).
- **While moving (Fig. 5).** RMS 9–16 cm, p95 18–22 cm in both campaigns, no clear dependence on the
  speed up to 0.6 m/s. Delay 0.09 s.
- **Out-and-back (Fig. 7).** Error before leaving 5.3 cm, after re-entry 4.3 cm (median, 10
  re-entries); spread at the outside points 6–10 cm (upper bound).

---

## Method

1. **Correction of the map error** (campaign 1): rebuild the distances to each triple of beacons from
   the recorded position and the wrong map, recompute with the correct map, keep the triple that gives
   the known hedgehog height with a continuous track (dynamic programming). The reference is not used.
   Check: the four distances of a sample shown in the Dashboard fit the correct map within 4 mm.
2. **Alignment** OptiTrack → Marvelmind: 2D rotation, translation and lever arm, robust least squares,
   fitted on the campaign 2 stops only; lever arm of mounting 1 on the campaign 1 rotations with
   rotation and translation fixed.
3. **Delay**: 0.09 s, fitted on the campaign 2 triangle.
4. **Data selection**: OptiTrack frames with mean marker error > 3 mm discarded; stops = speed
   < 2 cm/s and rotation < 3 °/s for at least 10 s, 2 s trimmed at both ends.
5. **Metrics**: 2D error to the OptiTrack hedgehog position; median, RMS, p95, CDF; per-stop offset and
   jitter; Allan deviation; error vs speed; error before leaving / after re-entry; differences between
   campaigns with bootstrap intervals.
6. **Uncertainty budget**: OptiTrack (static noise below 0.1 mm, marker error about 1 mm), lever arm,
   delay residual × speed, temperature (speed of sound about 0.17 %/°C; Dashboard setting 23 °C, room
   temperature not measured).

Temporal filters (Kalman, RTS smoother) were tried on the Experiment 0 data and gained at most 1 cm
because the error is a slow bias: one sentence in the paper, no figure.

---

## Paper outline (6 pages)

| Section | Content | Pages |
|---|---|---|
| I. Introduction | Low-cost indoor positioning on robots, why configuration matters, contributions | 0.75 |
| II. Related work | Evaluations of Marvelmind and similar systems, motion capture as reference, trajectory metrics (ATE) | 0.5 |
| III. Experimental setup | Room and beacons, OptiTrack, KELO, rigid body and hedgehog mountings, logging, **the two campaigns**. **Fig. 1** | 1 |
| IV. Method | Correction of the map error, alignment, delay, data selection, metrics, uncertainty | 1 |
| V. Results | A. configuration error and correction: C1 as recorded vs corrected vs C2 (**Table I**, Fig. 3); B. at rest (Fig. 4, 6); C. while moving (Fig. 5 or one sentence); D. outside the OptiTrack area (Fig. 7) | 2 |
| VI. Discussion and conclusion | Lessons for users (check the hedgehog height in z, log raw distances), limits: one room, 2D, joystick paths, small OptiTrack area | 0.5 |
| References | about 15 entries | 0.25 |

The LaTeX file is `paper/main.tex`; the figures and Table I are already included.

---

## Timeline

| Dates | Work |
|---|---|
| by 20 Nov | Introduction, related work (find the Marvelmind evaluation papers), setup, method; photos of the robot and the room for Fig. 1 |
| by 27 Nov | Results and discussion, complete first draft |
| 1–4 Dec | Review with Andrea, corrections, submission |

## For future campaigns

- Check in the Dashboard that the **hedgehog z reads its real height** before recording: it reveals a
  wrong beacon height at once.
- Turn on the **raw distances** (`COMM_Raw_distances`) and log them, so that any configuration error
  can be corrected exactly afterwards.
- Charge the beacons (beacon 1 was at 2.58 V and sleeping after campaign 2).
- Define the Motive rigid body once; measure the hedgehog lever arm with a tape and keep the mounting
  fixed for the whole campaign (or log when it changes).
- Write down the room temperature.
