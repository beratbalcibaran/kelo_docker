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
- **Never fit and test on the same data.** Cross-campaign registration: each campaign is evaluated
  with the alignment fitted on the other one; no recording is evaluated with parameters fitted on itself.
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
| Role | characterisation, effect of the configuration error | validation of the correction (the two campaigns register each other) |

- The **robot poses are the same** in the two campaigns (vertices and headings), so the campaigns can
  be compared stop by stop.
- **Hedgehog mounting**: the hedgehog was remounted during campaign 1 (mounting 1: rotations and
  triangles of C1; mounting 2: out-and-back, rest of C1 and all of C2). The lever arm is estimated for
  each mounting (52 cm apart; tape about 45 cm). Describe it as part of the setup.
- The Experiment 0 data (`docs/data_of_all_tests/`) are not used.

## Main results (details, figures, tables and numbers in `paper/FIGURES.md`)

- **Configuration error and correction (Table I, Fig. 3).** Median stop offset as recorded / corrected
  / C2: rotation 16.6 / 4.3 / 8.4 cm, triangle 20.7 / 7.8 / 5.8 cm; triangle moving RMS 65.6 / 11.0 /
  11.1 cm. C1 corrected − C2: −4.0 cm [−11.8, 3.1] and 2.0 cm [−4.3, 7.1] (95 % bootstrap): the two
  campaigns agree within the resolution of the comparison. One stop out of 23 failed (167 cm).
- **The simple fix is not enough.** Discarding samples with a wrong height keeps 55–88 % of the data
  and leaves the bias (rest 35.8 cm, triangle p95 50 cm); the correction keeps 81–100 % with errors at
  the C2 level.
- **Uncertainty (Table II).** OptiTrack 0.1 cm; registration (rotation, translation, lever arm)
  3.1–5.4 cm, the dominant term; delay 1.1 cm at 0.3 m/s; combined 5.4 cm (U = 11 cm, k = 2).
  Temperature effect on Marvelmind 1.8 cm.
- **At rest (Fig. 4, 6).** Same noise in both campaigns: jitter 4 mm, Allan deviation 1.6 mm at 1 s and
  0.2 mm at 60 s; offsets 2.6 and 5.5 cm, within the registration uncertainty. Repeatable heading effect
  in both campaigns (−44°: 2.6–5.5 cm, −134°: 12.9–16.5 cm).
- **Why the heading matters.** The beacon left out of the position solution depends strongly on the
  heading and is preferentially on one side/rear of the robot: consistent with shading by the robot
  body and mast.
- **Sensitivity.** Median error stable within 1 cm for hedgehog height ±5 cm, tolerance 7–15 cm, with
  or without the continuity term.
- **While moving (Fig. 5).** About 11 cm RMS, p95 18–23 cm in both campaigns; no clear dependence on
  the speed up to 0.6 m/s. Delay 0.105 ± 0.036 s.
- **Out-and-back (Fig. 7).** Error before leaving 5.4 cm, after re-entry 4.4 cm (median, 10
  re-entries); spread at the outside points 6–10 cm (upper bound).

---

## Method

1. **Correction of the map error** (campaign 1): rebuild the distances to each triple of beacons from
   the recorded position and the wrong map, recompute with the correct map, keep the triple that gives
   the known hedgehog height with a continuous track (dynamic programming). The reference is not used.
   Check: the four distances of a sample shown in the Dashboard fit the correct map within 4 mm.
2. **Cross-campaign registration** OptiTrack → Marvelmind (2D rotation, translation, lever arm, robust
   least squares on stop medians): C1 is evaluated with the registration fitted on C2; C2 with the one
   fitted on the C1 stops with the same mounting; the lever arm of mounting 1 is fitted on the other C1
   rotation run. No recording is evaluated with parameters fitted on itself.
3. **Delay**: median of the per-recording estimates, 0.105 ± 0.036 s.
4. **Data selection**: OptiTrack frames with mean marker error > 3 mm discarded; stops = speed
   < 2 cm/s and rotation < 3 °/s for at least 10 s, 2 s trimmed at both ends.
5. **Metrics**: 2D error to the OptiTrack hedgehog position; median, RMS, p95, CDF; per-stop offset and
   jitter; Allan deviation; error vs speed; error before leaving / after re-entry; differences between
   campaigns with bootstrap intervals.
6. **Uncertainty budget** (Table II): OptiTrack marker error; registration by bootstrap over the stops;
   delay spread × speed; synchronisation jitter; temperature simulated on the beacon geometry.
7. **Supporting analyses**: height filter alone vs correction; beacon left out vs heading; sensitivity
   of the correction to its settings.

Temporal filters (Kalman, RTS smoother) were tried on the Experiment 0 data and gained at most 1 cm
because the error is a slow bias: one sentence in the paper, no figure.

---

## Paper outline (6 pages)

| Section | Content | Pages |
|---|---|---|
| I. Introduction | Low-cost indoor positioning on robots, why configuration matters, contributions | 0.75 |
| II. Related work | Evaluations of Marvelmind and similar systems, motion capture as reference, trajectory metrics (ATE) | 0.5 |
| III. Experimental setup | Room and beacons, OptiTrack, KELO, rigid body and hedgehog mountings, logging, **the two campaigns**. **Fig. 1** | 1 |
| IV. Method | Correction of the map error, cross-campaign registration, delay, data selection, metrics, uncertainty budget (**Table II**) | 1 |
| V. Results | A. configuration error and correction: C1 as recorded / height filter / corrected vs C2 (**Table I**, Fig. 3), sensitivity; B. at rest (Fig. 4, 6) and the heading effect (beacon left out); C. while moving (Fig. 5 or one sentence); D. outside the OptiTrack area (Fig. 7) | 2 |
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
