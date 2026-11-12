# Figures for the I2MTC paper: what they show and how to use them

Preliminary figures made on 12 November 2026 from the data that are **embedded in the HTML reports**
(time, x, y already aligned; no Marvelmind z, no robot heading per sample, no Motive marker error).
They are good enough to decide the story of the paper and to start writing. Every number below must
be **recomputed from the raw recordings** before submission; the scripts are in `scripts/`, the
numbers in `figures/figures_numbers.json`. All figures are sized for one IEEE column (3.45 in) and
readable in black and white.

## Two setups: always say which one

| | Setup A (morning) | Setup B (afternoon) |
|---|---|---|
| Recordings | run 1 (`tur1_v3`, free path, 12 stops), last 40 points (triangle, about 12 laps) | P1 rotations (`test1`, `test2`), triangle fast (`test2_triangle`), triangle slow (`test4_triangle`), in/out (`test5_in_out1`) |
| Hedgehog | about 40 cm from the front markers | about 5 cm from the front markers, same height; moved by 45 cm |
| Marvelmind map | beacon heights entered as 240 m, map mirrored | heights fixed |
| Motive rigid body | re-created several times (only run 1 and last 40 points usable) | fixed, markers screwed |
| Wrong 3D solutions (z out of band) | rare (1.8 % jumps) | 4–49 % of the samples |

Setup A and setup B are aligned separately. Do not mix their numbers in one statistic.

---

## Fig. 1: triangle paths (setup A)

`figures/fig1_triangle_paths.pdf`

**What it shows.** OptiTrack path (black line) and Marvelmind positions (grey dots) for the last 40
points, about 12 laps of the triangle marked on the floor. A, B, C are the vertices.

**How it was made.** Marvelmind frame; OptiTrack aligned by Berat's analysis on the stops. Parts
where the OptiTrack markers fell (red in the replay, error > 25 cm) are removed.

**Use in the paper.** Results, at rest / repeatability, or Setup, as an example of the test path.
Optional if space is short: Fig. 2 carries the result.

**Suggested caption.** *Path of the robot over about 12 laps of the triangle: OptiTrack reference
(line) and Marvelmind positions (dots). A, B and C are the vertices where the robot stopped.*

---

## Fig. 2: offset at each visit of the vertices (setup A)

`figures/fig2_vertex_offsets.pdf`

**What it shows.** Each small marker is one stop: the Marvelmind median minus the OptiTrack position.
Big markers are the mean per vertex.

| Vertex | Visits | Mean offset | Spread between visits (std x / y) |
|---|---|---|---|
| A | 13 | 2.2 cm | 4.0 / 6.7 cm |
| B | 11 | 2.1 cm | 5.4 / 6.0 cm |
| C | 11 | 3.1 cm | 5.6 / 6.5 cm |

**Message.** The systematic, position-dependent part of the error is small (2–3 cm). The error
changes from one visit to the next (4–7 cm), but it is constant during a stop (jitter 2–5 cm). So:
a position-dependent correction map would not help, and averaging longer at a stop does not help
either. This is the main result at rest.

**Caveat.** The alignment was fitted on the same stops, and the robot reached the vertices with
different headings. Fig. 5 (setup B) shows that the heading matters, so part of this spread may be
the heading. Recompute with the raw data and the heading of each stop.

**Use in the paper.** Results, A (at rest). Keep.

**Suggested caption.** *Marvelmind offset at each stop on the three vertices (small markers) and
mean per vertex (large markers). The mean is 2–3 cm, the spread between visits 4–7 cm.*

---

## Fig. 3: error vs speed (setup A)

`figures/fig3_error_vs_speed.pdf`

**What it shows.** RMS of the Marvelmind error in speed bins (speed from OptiTrack); the first point
is at rest (< 3 cm/s). Error bars: 95 % bootstrap interval.

| Speed (m/s) | rest | 0.03–0.1 | 0.1–0.2 | 0.2–0.3 | 0.3–0.4 | 0.4–0.55 |
|---|---|---|---|---|---|---|
| run 1 (cm) | 4.5 | 5.8 | 5.8 | 6.2 | 7.2 | – |
| last 40 points (cm) | 8.7 | 8.9 | 10.6 | 11.1 | 11.5 | 14.1 (33 samples) |

**Message.** The error grows slowly with speed, by about 3–5 cm up to 0.4 m/s. The fast and slow
triangles of setup B were recorded for this figure: add them when the raw data are available.

**Caveat.** Consecutive samples are correlated, so the bootstrap intervals are too narrow. Use blocks
(for example one block per stop-to-stop segment) in the final version.

**Use in the paper.** Results, B (while moving). Keep.

**Suggested caption.** *RMS error of Marvelmind as a function of the speed measured by OptiTrack.
The first point of each curve is at rest.*

---

## Fig. 4: processing chain, temporal filters (setup A)

`figures/fig4_cdf_processing.pdf`

**What it shows.** CDF of the error on the last 40 points, at rest and while moving, for the raw
data, a causal constant-velocity Kalman filter and the offline Kalman + RTS smoother. The filter
parameters were tuned on run 1 and tested here (never tuned and tested on the same recording).

| | Rest RMS / p95 | Moving RMS / p95 |
|---|---|---|
| raw | 8.7 / 15.9 cm | 10.5 / 18.3 cm |
| Kalman, causal | 9.0 / 15.9 cm | 10.4 / 18.0 cm |
| Kalman + RTS, offline | 8.7 / 15.9 cm | 9.8 / 16.5 cm |

**Message.** Filtering in time gains at most about 1 cm, because the error is slow (Fig. 2). Delay
compensation was also tried: same result.

**Use in the paper.** Results, C. The figure can be replaced by one sentence or by rows of the
summary table if space is short.

**Suggested caption.** *Error CDF on the test recording for the raw Marvelmind data and two
temporal filters tuned on another recording.*

---

## Fig. 5: offset vs robot heading (setup B)

`figures/fig5_heading.pdf`

**What it shows.** Offset at each stop against the robot heading from OptiTrack, for all stops of
the afternoon, with one common alignment (Berat's table `common_alignment_durmalar.csv`, 39 stops,
lever arm estimated 0.15 m). Circles: P1, rotation in place at one point, two repetitions. Squares
and triangles: the two vertices used in the triangle and in/out tests.

**Message.** At the same place the error depends strongly on the heading, and it repeats:

| P1 heading | −134° | −44° | +45/47° | +137° |
|---|---|---|---|---|
| repetition 1 | 10.0 cm | 7.3 cm | 55.0 cm | 29.4 cm |
| repetition 2 | 10.0 cm | 4.9 cm | 59.0 cm | 45.7 cm |

At the vertex near (0.5, 0.4) m: heading +76° gives 5–6 cm, heading −85…−94° gives 19–41 cm. The most
likely cause is the robot body hiding the hedgehog from one or more beacons at some headings. This is
probably the most interesting finding of the day.

**Caveats (important).**
- The lever arm is estimated from the data, not measured. A wrong lever arm also produces an error
  that depends on the heading. Measure it and recompute before claiming the effect.
- At the vertex near (−1.2, −1.2) m, the same heading (about −85°) gives 3–11 cm in the slow triangle
  and 36–49 cm in the fast triangle and in the in/out test. Either something changed between those
  recordings, or the effect also depends on the speed or on the path. Check with the raw data.

**Use in the paper.** Results, A or a separate subsection ("effect of the robot heading"). Keep if
confirmed with the measured lever arm.

**Suggested caption.** *Marvelmind offset at the stops as a function of the robot heading. Circles:
rotation in place at one point, two repetitions.*

---

## Fig. 6: z consistency check (setup B)

`figures/fig6_zcheck.pdf`

**What it shows.** Moving error (RMS and 95th percentile) with all samples and after removing the
samples whose Marvelmind z is outside ±0.4 m of the known hedgehog height. Numbers from Berat's table
`common_alignment_ozet.csv` (common alignment).

| Recording | Removed | RMS all → z check | p95 all → z check |
|---|---|---|---|
| triangle, fast | 34 % | 36.3 → 30.0 cm | 54.6 → 50.4 cm |
| triangle, slow | 49 % | 82.6 → 31.4 cm | 156.7 → 56.4 cm |
| in/out | 44 % | 50.6 → 34.9 cm | 138.6 → 63.6 cm |

**Message.** The hedgehog height on the robot is fixed, so z is a free integrity check: it removes
the wrong 3D solutions and cuts the large errors (p95) by about a factor of 2–3. The cost is
availability: up to half of the samples are removed. Present it as an accuracy vs availability
trade-off. This is the most useful step of the processing chain.

**Caveat.** The remaining error (30–35 cm RMS) is still dominated by the heading effect and by the
uncertain alignment of setup B.

**Use in the paper.** Results, C (processing chain). Keep.

**Suggested caption.** *Moving error with all Marvelmind samples and after the z consistency check;
the labels give the share of samples removed.*

---

## Fig. 7: in/out test (setup B)

`figures/fig7_inout.pdf`

**What it shows.** Top: Marvelmind error over time (only where OptiTrack is available); grey bands:
the robot is outside the OptiTrack area. Bottom: OptiTrack path (black) and Marvelmind positions after
the z check (grey); the robot leaves the OptiTrack area towards the right.

**Status.** Only a first look. Missing for the paper, both need the raw data:
- the **error at re-entry** compared with the error before leaving (from the embedded data only 3 of
  10 re-entries can be evaluated);
- the **spread of Marvelmind on the outside points C and D** over the 10 laps (the z check removed
  too many samples there to detect the stops reliably).

The positions of C and D are only approximate, so the paper reports the re-entry error and the spread
outside, not an absolute accuracy outside the OptiTrack area.

**Use in the paper.** Results, D. Redo with the raw data.

---

## What is needed to make the final figures

1. Raw recordings (`kayit_20261112_*.csv`) of all afternoon tests, filtered and unfiltered, and P2
   (10 min at rest, for the Allan deviation).
2. The Motive rigid body definition of the afternoon (NatNet data descriptions).
3. The hedgehog position measured with a tape with respect to two markers that can be identified in
   Motive.
4. The log of the tests (file → test type, speed, notes).

## Which figures to keep in 6 pages

Recommended: Fig. 2, 3, 5, 6, 7 plus the setup figure (photo and floor plan). Fig. 1 and Fig. 4 only
if space allows (Fig. 4 can become two rows of the summary table).
