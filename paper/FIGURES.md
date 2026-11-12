# Figures for the I2MTC paper: what they show and how to use them

Final figures from the recordings of **12 November 2026, afternoon and evening**. They are made by
`scripts/analysis.py` (run `python paper/scripts/analysis.py .` from the repository root after
unzipping `experiments/archives/2026-11-12/*.zip` into `experiments/extracted/`). All numbers are
in `figures/numbers.json`. Figures are sized for one IEEE column (Fig. 1: two columns) and readable
in black and white.

The morning data (Experiment 0, `docs/data_of_all_tests/`) are **not** used: the Motive rigid body
was re-created several times, the hedgehog was in another place and the Marvelmind map was different.

## Data and how they were processed

| Recording | Session | What | Hedgehog mount |
|---|---|---|---|
| `test1`, `test2` | afternoon | rotation in place, 4 headings, twice | before the move |
| `test2_triangle` | afternoon | triangle, fast | before the move |
| `test4_triangle` | afternoon | triangle, slow | before the move |
| `test5_in_out1` | afternoon | out of the OptiTrack area and back, 10 laps (5 slow, 5 fast) | after the move |
| `test6_wait` | afternoon | 18 min at rest | after the move |
| `deneme_2025`, `deneme_2028` | evening | rotation in place, twice | after the move |
| `deneme_triangle_2031` | evening | triangle | after the move |
| `denemenew_wait_2040` | evening | 10 min at rest | after the move |

1. **Beacon 1 height.** In the afternoon the Marvelmind map used beacon 1 at 0 m instead of 2.6 m
   (the value shown in the Dashboard had not been applied). The afternoon positions were recomputed
   with the corrected map by `scripts/b1_correction.py`, **without OptiTrack** (see
   `experiments/README.md`). The measured distances were fine: a sample shown in the Dashboard fits
   the corrected map within 4 mm. 81–100 % of the samples remain valid; the others are marked
   invalid because no triple of beacons gives a plausible hedgehog height.
2. **Hedgehog moved.** Between `test4_triangle` (18:29) and `test5_in_out1` (18:51) the hedgehog was
   moved. Lever arm with respect to the rigid body pivot: (−0.144, −0.141) m before, (0.214, 0.241) m
   after, i.e. 52 cm (tape: about 45 cm).
3. **Alignment** OptiTrack → Marvelmind map (rotation, translation, lever arm): fitted **only** on the
   stops of the three evening trials. The lever arm before the move is fitted on the afternoon
   rotations (`test1`, `test2`) with rotation and translation fixed. Triangles, in/out and the long
   rests are never used for fitting, so their errors are a fair test.
4. **Delay** of Marvelmind with respect to OptiTrack: 0.09 s, estimated on the evening triangle and
   applied to all recordings.
5. OptiTrack frames with a mean marker error above 3 mm are discarded. Stops: OptiTrack speed below
   2 cm/s and rotation below 3 °/s for at least 10 s, 2 s trimmed at both ends.
6. **One stop failed** the correction (`test4_triangle` at 318 s, 39 samples, 167 cm). It is reported
   and excluded from the per-vertex and per-speed statistics; it stays in the CDF (Fig. 3).

## Summary table (use it as Table I)

Error = 2D distance between Marvelmind and the OptiTrack hedgehog position.

| Recording | As recorded, median / p95 | Corrected, median / p95 | Moving, RMS / p95 | Valid after correction |
|---|---|---|---|---|
| rotation 1 (`test1`) | 6.6 / 17.8 cm | 4.9 / 13.6 cm | – | 82 % |
| rotation 2 (`test2`) | 19.9 / 129.5 cm | 3.5 / 11.8 cm | – | 95 % |
| triangle, fast | 23.5 / 50.9 cm | 7.0 / 9.9 cm | 9.4 / 18.1 cm | 86 % |
| triangle, slow | 17.1 / 140.6 cm | 10.3 / 20.6 cm | 15.6 / 22.4 cm | 84 % |
| in/out (inside the OptiTrack area) | 13.4 / 150.9 cm | 6.4 / 20.2 cm | 11.1 / 17.7 cm | 81 % |
| rest, 18 min | 35.8 / 36.3 cm | 2.6 / 3.1 cm | – | 100 % |
| evening triangle (correct map) | – | 8.2 / 13.8 cm at rest | 10.8 / 21.6 cm | – |
| evening rest, 10 min | – | 7.4 / 7.9 cm | – | – |

"As recorded" is computed with the best rigid alignment fitted on the as-recorded data themselves,
so it is the most favourable view of the wrong map.

---

## Fig. 1: setup (two columns)

`figures/fig1_setup.pdf`

(a) Floor plan in the Marvelmind map frame: the four stationary beacons with their heights, the
area covered by OptiTrack (blue), the triangle (orange) and the Marvelmind path of the in/out test
(grey), which leaves the OptiTrack area to the right. (b) Robot seen from above: the 13 markers of the
rigid body, its pivot, and the hedgehog before and after it was moved.

**Use.** Section III. Add the photo of the robot as a third panel when it is available.

**Suggested caption.** *Experimental setup. (a) Floor plan with the stationary beacons (height
above the floor), the area covered by OptiTrack, the triangle path and the in/out path. (b) Top view
of the OptiTrack rigid body on the robot and position of the Marvelmind hedgehog in the two mountings.*

---

## Fig. 2: as recorded vs corrected (slow triangle)

`figures/fig2_correction.pdf`

The slow triangle with the positions as recorded (wrong beacon 1 height) and after the correction,
both against the OptiTrack path.

**Message.** The wrong map gives scattered, partly wrong positions (wrong 3D solutions); after the
correction the path follows the reference. Optional if space is short: Fig. 3 carries the numbers.

**Suggested caption.** *Slow triangle: Marvelmind positions (dots) against the OptiTrack reference
(line), (a) as recorded with the wrong height of beacon 1, (b) after the correction.*

---

## Fig. 3: error CDF before and after the correction

`figures/fig3_cdf.pdf`

**Message.** The correction removes the large errors: the p95 drops from 50–150 cm to 10–20 cm, the
median from 13–36 cm to 3–10 cm (Table I). It works for all kinds of test, at rest and while moving.

**Use.** Results, "Effect of a configuration error and its correction". Keep.

**Suggested caption.** *Cumulative distribution of the Marvelmind error as recorded (dotted) and after
the correction of the beacon 1 height (solid).*

---

## Fig. 4: at rest (vertices and rotation in place)

`figures/fig4_rest.pdf`

(a) Offset at each stop on the three triangle vertices (both triangles); large markers: mean per
vertex. (b) Offset at the stops of the rotation in place against the robot heading, four repetitions
(two afternoon, two evening).

| Vertex | Stops | Mean offset | Spread between visits (std x / y) | Jitter within a stop |
|---|---|---|---|---|
| 1 | 7 | 3.7 cm | 8.9 / 3.5 cm | 1.3 mm |
| 2 | 8 | 5.8 cm | 3.0 / 3.9 cm | 1.4 mm |
| 3 | 6 | 10.5 cm | 4.9 / 4.4 cm | 1.4 mm |

Rotation in place: the offset depends on the heading and the pattern repeats: −134° gives
10–14 cm in three of four runs, −44° gives 0.4–4.7 cm.

**Message.** At rest Marvelmind is very repeatable within a stop (about 1 mm), but the offset changes
by several cm from one visit to the next and with the heading. The error is a slowly varying bias, not
noise.

**Use.** Results, at rest. Keep.

**Suggested caption.** *Offset at rest. (a) Offset at each stop on the three triangle vertices (small
markers) and mean per vertex (large markers). (b) Offset against the robot heading for the rotation
in place, four repetitions.*

---

## Fig. 5: error vs speed

`figures/fig5_speed.pdf`

RMS error in speed bins (speed from OptiTrack, delay 0.09 s compensated); bins with fewer than 30
samples are not shown.

**Message.** After the correction there is no clear dependence on the speed up to 0.6 m/s: the
RMS stays at about 7–14 cm. (The large speed effect seen during the tests was caused by the wrong
map.) Note: the "fast" triangle has few samples above 0.2 m/s; check the speeds of the two triangles.

**Use.** Results, while moving. Can be replaced by one sentence if space is short.

**Suggested caption.** *RMS error of Marvelmind as a function of the speed measured by OptiTrack.*

---

## Fig. 6: Allan deviation at rest

`figures/fig6_allan.pdf`

Allan deviation (2D) of the Marvelmind position during the two long rests (18 min afternoon, corrected;
10 min evening), with OptiTrack for comparison. Dotted lines: offset of each rest from the reference.

| | 18 min (afternoon) | 10 min (evening) |
|---|---|---|
| Jitter (std, 2D) | 4.0 mm | 4.1 mm |
| Allan deviation at 1 / 10 / 60 s | 1.6 / 0.5 / 0.23 mm | 1.6 / 0.5 / 0.21 mm |
| Offset to the reference | 2.6 cm | 7.4 cm |

**Message.** Precision and accuracy differ by more than an order of magnitude: averaging reduces the
noise to a fraction of a millimetre, but the offset (3–7 cm) stays. Averaging longer does not improve
the accuracy. Also: the two rests agree, so the correction of the afternoon data does not change the
noise behaviour.

**Use.** Results, at rest. Keep.

**Suggested caption.** *Allan deviation of the Marvelmind position at rest (two recordings) and of
OptiTrack. The dotted lines give the offset of each recording from the reference.*

---

## Fig. 7: in/out test

`figures/fig7_inout.pdf`

(a) Error over time where OptiTrack is available; grey bands: the robot is outside the OptiTrack area.
(b) OptiTrack (blue) and Marvelmind (grey) paths; crosses: Marvelmind position at the stops on the
outside points C and D.

| | Value |
|---|---|
| Re-entries evaluated | 10 |
| Error in the 8 s before leaving / after re-entry (median) | 5.3 / 4.3 cm |
| Spread at the outside points (std x / y, 4 stops each) | C: 5.8 / 6.9 cm, D: 7.6 / 9.6 cm |

**Message.** Leaving the OptiTrack area does not degrade Marvelmind: the error after re-entry is the
same as before. Outside, the repeatability over the visits is 6–10 cm; this is an upper bound, because
it also includes how precisely the robot was stopped on the floor marks. There is no absolute accuracy
outside the OptiTrack area (the floor marks were not surveyed).

**Use.** Results, outside the OptiTrack area. Keep.

**Suggested caption.** *Out-and-back test. (a) Marvelmind error over time; grey: outside the OptiTrack
area. (b) Paths and Marvelmind positions at the outside stops.*

---

## Which figures in 6 pages

Recommended: Fig. 1 (two columns, with the photo), 3, 4, 6, 7 and Table I. Fig. 2 and Fig. 5 only if
space allows (Fig. 5 can become one sentence).
