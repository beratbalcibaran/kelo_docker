# Figures, tables and analyses for the I2MTC paper

Everything here is produced by `scripts/analysis.py` (run `python paper/scripts/analysis.py .` from the
repository root after unzipping `experiments/archives/2026-11-12/*.zip` into `experiments/extracted/`).
All numbers are in `figures/numbers.json`. Figures are sized for one IEEE column (Fig. 1: two columns)
and readable in black and white.

## Index

| Item | File | Paper section | Status |
|---|---|---|---|
| Table: test campaigns | in `main.tex` | III | keep |
| Fig. 1: annotated photo of the room + zoom on the robot (3D markers), one column, first page | `figures/fig1_teaser.pdf` (two-column variant: `fig1_photo.pdf`) | before I | keep |
| Floor plan with beacons, triangle A–B–C, stops D, E | `figures/fig_floorplan.pdf` | III | keep |
| Table II: uncertainty budget | in `main.tex` | IV | keep |
| Table I: C1 as recorded / z filter / corrected / C2, Monte Carlo intervals | in `main.tex` | V-A | keep |
| Fig. 2: slow triangle as recorded vs corrected | `figures/fig2_correction.pdf` | V-A | optional |
| Fig. 3: CDF per test type | `figures/fig3_cdf.pdf` | V-A | keep |
| Fig. 8: Monte Carlo of beacon height errors | `figures/fig8_montecarlo.pdf` | V-B | keep |
| Fig. 4: at rest (vertices, heading) | `figures/fig4_rest.pdf` | V-C | keep |
| Fig. 6: Allan deviation | `figures/fig6_allan.pdf` | V-C | keep |
| Fig. 5: error vs speed | `figures/fig5_speed.pdf` | V-D | optional |
| Fig. 7: out-and-back | `figures/fig7_inout.pdf` | V-E | keep |

In `main.tex` the results sections are: A. configuration error and correction; B. tolerance to beacon
height errors (simulation); C. at rest; D. while moving; E. outside the OptiTrack area. LaTeX numbers the
figures in order of appearance, so the final numbering will differ from the file names.

## Experimental design: two campaigns

| | Campaign 1 (C1): main test programme | Campaign 2 (C2): reference configuration |
|---|---|---|
| Purpose | characterisation over all test types | same robot poses with the correct Marvelmind map; validation of the correction of C1 |
| Marvelmind map | beacon 1 at 0 m instead of 2.6 m (value not applied); positions recomputed afterwards | correct (beacon 1 at 2.6 m) |
| Rotation in place, 4 headings | 2 runs (`test1`, `test2`) | 2 runs (`deneme_2025`, `deneme_2028`) |
| Triangle (same 3 vertices and headings) | fast (`test2_triangle`) and slow (`test4_triangle`) | 1 run (`deneme_triangle_2031`) |
| Rest | 18 min (`test6_wait`) | 10 min (`denemenew_wait_2040`) |
| Out-and-back (leaves the OptiTrack area) | 10 laps (`test5_in_out1`) | – |

**Hedgehog mounting.** Mounting 1 for the C1 rotations and triangles, mounting 2 for the C1
out-and-back and rest and all of C2. Lever arms (rigid body pivot → hedgehog): (−0.144, −0.141) m and
(0.214, 0.241) m.

## Processing

1. **Correction of C1** (`scripts/b1_correction.py`), **without OptiTrack**: distances to each triple
   of beacons rebuilt from the recorded position and the wrong map, position recomputed with the
   correct map, triple chosen by the known hedgehog height (0.70 m) and continuity. One stop out of 23
   (C1 slow triangle, 39 samples, 167 cm) is corrected wrongly: reported, excluded from the per-stop
   statistics. Including it, the triangle stops of C1 corrected give median 8.2 cm and p95 19.8 cm
   (instead of 7.8 / 14.2): write it in the paper, so that the exclusion is transparent.
2. **Cross-campaign registration** (OptiTrack → Marvelmind map: rotation, translation, lever arm,
   robust least squares on stop medians). No recording is evaluated with parameters fitted on itself:
   - C1 recordings: registration fitted on the **C2 stops**; lever arm of mounting 1 fitted on the C1
     rotation run that is not being evaluated (test1 ↔ test2);
   - C2 recordings: registration fitted on the **C1 stops with the same mounting** (out-and-back and
     rest, corrected data).
3. **Delay**: best delay per recording with motion 0.04–0.21 s; applied value = median, 0.105 s.
4. OptiTrack frames with mean marker error > 3 mm discarded. Stops: speed < 2 cm/s and rotation
   < 3 °/s for at least 10 s, 2 s trimmed at both ends.
5. **Uncertainty**: budget (Table II) and Monte Carlo propagation (JCGM 101) of the comparison; the
   intervals of Table I come from the Monte Carlo.
6. **Simulation**: Monte Carlo of beacon height errors with the true geometry and trajectories (Fig. 8).
7. **Supporting analyses**: height filter alone vs correction (Table I), beacon left out vs heading,
   sensitivity of the correction to its settings.

---

## Table I: C1 as recorded, z filter only, corrected, and C2

Median / p95 (cm). Stops: per-stop offset. Moving: RMS / p95. Last column: difference C1 corrected −
C2 with the 95 % Monte Carlo interval (registration, delay, OptiTrack noise, stop resampling).

| Test | C1 as recorded | C1 z filter only | C1 corrected | C2 reference | C1 corr. − C2 |
|---|---|---|---|---|---|
| Rotation, stops (8 / 8) | 16.6 / 85.8 | – | 4.3 / 14.0 | 8.4 / 16.1 | −4.0 [−12.8, 2.4] |
| Triangle, stops (22 / 9) | 20.7 / 126.7 | – | 7.8 / 14.2 | 5.8 / 14.2 | 2.0 [−3.9, 10.6] |
| Triangle, moving | 65.6 / 141.5 | 20.4 RMS | 11.0 / 21.1 | 11.1 / 23.4 | −0.1 [−2.4, 3.7] (RMS) |
| Triangle, all samples | – | 12.4 / 50.4 | 7.7 / 19.9 | – | – |
| Rest (1 / 1) | 35.9 | 35.8 | 2.6 | 5.5 | −2.9 [−7.4, 7.3] |
| Kept samples | 100 % | 55–100 % | 81–100 % | 100 % | |

All samples with reference (median / p95, and share of samples kept):

| Test | C1 z filter only | C1 corrected |
|---|---|---|
| Rotation | 14.0 / 20.6, 88 % kept | 5.5 / 14.9, 89 % kept |
| Triangle | 12.4 / 50.4, 59 % kept | 7.7 / 19.9, 85 % kept |
| Out-and-back (inside) | 9.9 / 44.3, 55 % kept | 6.5 / 20.3, 81 % kept |
| Rest | 35.8 / 36.3, 100 % kept | 2.6 / 3.1, 100 % kept |

**Messages.**
- The configuration error multiplies the errors by 3–10 (p95 up to 1.5 m, out-and-back; 1.4 m in the triangle).
- The simple fix, discarding samples with a wrong height, removes the worst outliers but keeps the
  bias (rest: 35.8 cm) and throws away up to 45 % of the data. The correction keeps 81–100 % of the
  samples and brings the errors down to the C2 level.
- C1 corrected and C2 agree: all four differences have Monte Carlo intervals that include zero, and the
  moving RMS differs by 0.1 cm. The intervals at the stops are 10–15 cm wide (few stops, registration
  uncertainty): write "agree within the resolution of the comparison", not "equivalent".

## Table II: uncertainty budget of the comparison

Standard uncertainties (k = 1) of the reference hedgehog position, 2D.

| Contribution | Value | How |
|---|---|---|
| OptiTrack position (mean marker error) | 0.11 cm | Motive, all recordings; static noise 0.05 mm |
| Registration (rotation, translation, lever arm) | 3.1–5.4 cm | bootstrap over the stops used for the fit (300 resamples); C2: 3.1 cm, C1 mounting 1: 4.9 cm, mounting 2: 5.4 cm |
| of which lever arm alone | 1.8–2.9 cm per axis | same bootstrap; check with the tape: 42.3 vs 40 cm (mounting 1), move 52 vs 45 cm |
| Delay, 0.105 ± 0.036 s, at 0.3 m/s | 1.1 cm | spread of the four per-recording estimates |
| Synchronisation (transport jitter 5 ms) at 0.3 m/s | 0.15 cm | device time stamp vs PC arrival |
| **Combined, at rest / moving** | **5.4 / 5.5 cm** (U, k = 2: 10.8 / 11.0 cm) | root sum of squares |
| Temperature (affects Marvelmind, not the reference) | 1.8 cm | ±3 °C around the 23 °C setting, 0.17 %/°C, simulated on the beacon geometry |

**Message.** The OptiTrack instrument is not the limit; the **registration** is. It is a common-mode
term (it shifts all samples of a recording together) and comes from the scatter of the Marvelmind
offsets used to fit it, mostly through the lever arm. Consequences for the text: differences between
campaigns and the gross effect of the configuration error (tens of cm) are well resolved; absolute
offsets at rest of 3–8 cm are of the same order as the registration uncertainty and should be stated
with it. Precision (Allan deviation) does not depend on the registration. The same inputs are
propagated by Monte Carlo below.

## Monte Carlo propagation of the uncertainty (JCGM 101, GUM Supplement 1)

300 draws; in each draw the registration parameters come from the stop bootstrap, the delay from
N(0.105 s, 0.036 s), the OptiTrack position gets 1.1 mm noise, and the stops are resampled; all
metrics of Table I are recomputed. 95 % coverage intervals (cm):

| Metric | Point value | MC 95 % interval |
|---|---|---|
| Rotation, median stop offset C1 corrected / C2 | 4.3 / 8.4 | [1.6, 11.2] / [5.5, 16.7] |
| Rotation, C1 corr. − C2 | −4.0 | [−12.8, 2.4] |
| Triangle, median stop offset C1 corrected / C2 | 7.8 / 5.8 | [5.1, 14.9] / [2.1, 12.3] |
| Triangle, C1 corr. − C2 | 2.0 | [−3.9, 10.6] |
| Rest offset C1 / C2 | 2.6 / 5.5 | [0.6, 12.0] / [2.9, 10.0] |
| Rest, C1 − C2 | −2.9 | [−7.4, 7.3] |
| Triangle moving RMS C1 / C2 | 11.0 / 11.1 | [10.4, 15.2] / [10.2, 13.6] |
| Triangle moving RMS, C1 − C2 | −0.1 | [−2.4, 3.7] |

**Message.** All differences between the campaigns contain zero; the moving RMS agrees within ±3 cm.
These intervals replace the stop-only bootstrap intervals in Table I: they include the registration
and delay uncertainty.

## Monte Carlo of beacon height errors (Fig. 8)

Simulation with the true beacon geometry (correct map) and the true hedgehog trajectories of the C1
slow triangle and out-and-back (3009 samples, z = 0.70 m). For each beacon and each height error
(±0.05 … −2.6 m) the map that the Marvelmind software would build is recomputed from the true
beacon-to-beacon distances with the wrong height (as happened in C1: the other beacons move in plan).
Ranges get 3 mm Gaussian noise; the solver uses a random triple of beacons (uniform: a pessimistic
model, the real solver avoided the wrong beacon more often). Errors after the best rigid registration,
as for the real data. Then the correction of `b1_correction.py` is applied.

| Beacon height error | p95 position error | median \|z error\| | samples with \|z error\| > 10 cm |
|---|---|---|---|
| ±0.05 m | 1.8–2.8 cm | 0.9–2.0 cm | 0 % |
| ±0.10 m | 2.7–5.1 cm | 2.0–3.9 cm | 0–13 % |
| ±0.25 m | 5.7–12.7 cm | 5.3–10.2 cm | 24–51 % |
| −0.5 m | 11–19 cm | 11–20 cm | 55–67 % |
| −1.0 m | 22–46 cm | 22–38 cm | 66–81 % |
| −2.6 m | 60–191 cm | 56–81 cm | 75–100 % |
| no error | 1.3 cm | 0.4 cm | 0 % |

- **Check against the real case**: B1 −2.6 m gives p95 181 cm and 58 % wrong 3D solutions; C1 as
  recorded had p95 51–151 cm and 4–49 % wrong solutions. Same order; the uniform-triple model is
  pessimistic.
- **Correction**: in every simulated case the correction brings the error back to the no-error level
  (p95 1.2–1.5 cm) and keeps 99.6–100 % of the samples. In the real data it keeps 81–100 % and one stop
  out of 23 fails: the real solver does not always follow the three-beacon model.
- **Practical tolerance**: height errors up to ±0.1 m add at most about 5 cm (p95) and are hardly
  visible in z; between 0.1 and 0.5 m the error is already 5–30 cm but only part of the samples show
  a z error above 10 cm ("silent" errors); above 0.5 m the z check reveals the error almost always.
  **Advice: measure beacon heights within ±5 cm and check the hedgehog z before recording.**

**Caption.** *Monte Carlo of a wrong beacon height, for each beacon. (a) p95 position error as a function
of the height error (filled: beacon set too low in the map, open: too high; dotted: after the
correction; star: campaign 1 measured). (b) Median error of the height reported for the mobile
beacon; dashed line: 10 cm.*

## Beacon triple vs robot heading (text, maybe a small table)

For every corrected C1 sample the correction tells which beacon was **not** used (inferred from the
height, not measured). Share of samples per heading sector:

| Heading | Samples | B1 excluded | B2 | B4 | B5 |
|---|---|---|---|---|---|
| −180…−90° | 4701 | 20.5 % | 28.5 % | 4.5 % | 46.4 % |
| −90…0° | 2614 | 0.5 % | 48.5 % | 15.0 % | 36.0 % |
| 0…90° | 2160 | 0.6 % | 56.0 % | 10.0 % | 33.4 % |
| 90…180° | 10007 | 62.5 % | 8.1 % | 6.9 % | 22.6 % |

Relative bearing of the excluded beacon (direction of the beacon seen from the hedgehog, with respect
to the robot heading): 37 % of the cases fall in the 90…135° sector, where only 16 % of all beacon
directions lie; 12.5 % in −180…−135° (baseline 5.7 %).

**Message.** Which beacon drops out depends strongly on the robot heading, and it is preferentially
a beacon on one side/rear of the robot: consistent with the robot body and mast shading the hedgehog.
Say "consistent with", because the triple is inferred.

## Heading: registration or IPS? (text, `numbers.json` → `heading_decomposition`)

A lever-arm error δℓ gives an offset R(θ)R(ψ)δℓ that **rotates with the heading**; translation and frame
rotation give a constant offset at a fixed position; the delay vanishes at rest. So a heading pattern in
Fig. 4b is not automatically a property of the IPS. The stop offsets of the rotation in place, as
complex numbers, are fitted with `o_k = c_run + a·e^{iψ_k} + b·e^{−iψ_k}` (c_run: constant per run).
A lever-arm error enters only `a`; `b` rotates **against** the heading and no registration error can
produce it.

| Campaign | Same heading, run 1 vs run 2 | \|a\| (lever-arm-like) | \|b\| (not from registration) | u (per axis) | Residual per axis |
|---|---|---|---|---|---|
| C1 (corrected) | 4.6–14.6 cm | 1.8 cm | 2.4 cm | 2.4 cm | 6.7 cm |
| C2 (reference) | 0.5–6.2 cm | 7.6 cm | 6.9 cm | 1.3 cm | 3.6 cm |

**Message.** In C2 the heading pattern is repeatable and has a 6.9 ± 1.3 cm component that the
registration cannot explain: a real heading-dependent bias of the IPS. The 7.6 cm co-rotating part is
compatible with a lever-arm error of the size of the 7 cm tape discrepancy. In C1 the two runs do not
repeat (up to 14.6 cm at the same heading) and neither term is significant, so do **not** write "the
same pattern in both campaigns".

## Sensitivity of the correction (text)

C1 triangles and out-and-back, all samples with reference:

| Setting | Kept | Median / p95 | Errors > 50 cm |
|---|---|---|---|
| nominal (height 0.70 m, tolerance 0.10 m, continuity 2) | 83.9 % | 7.3 / 20.1 cm | 0.49 % |
| height 0.65 m | 83.4 % | 8.0 / 25.9 cm | 3.8 % |
| height 0.75 m | 78.4 % | 7.5 / 20.3 cm | 0.11 % |
| tolerance 0.07 m | 74.8 % | 7.4 / 20.0 cm | 0.09 % |
| tolerance 0.15 m | 91.2 % | 7.5 / 20.7 cm | 1.6 % |
| no continuity term | 84.5 % | 7.3 / 20.2 cm | 0.75 % |
| continuity weight 5 | 83.0 % | 7.3 / 19.9 cm | 0.03 % |

**Message.** The median is stable within 1 cm; the tolerance trades kept samples against rare gross
errors; the continuity term mainly removes the gross errors. An assumed height 5 cm too low is the
most harmful setting.

---

## Fig. 1: annotated photo of the room (two columns)

`figures/fig1_photo.pdf`, made by `scripts/fig_setup_photo.py` from `photos/room_IMG_4706.jpg`
(run from `paper/`: `python scripts/fig_setup_photo.py <optitrack_rigid_body_markers.csv> vertical nofade` for the
one-column teaser `fig1_teaser.pdf` used in `main.tex`, `horizontal` for the two-column variant `fig1_photo.pdf`).
It is placed before the Introduction, so that it appears at the top of the second column of the first page,
with the zoom on the robot below the photo.
Photo with callouts: stationary beacons B1 2.60 m, B2 2.50 m, B4 2.40 m, B5 2.55 m; OptiTrack
V120:Trio on its tripod (120 Hz, 1.93 m); one label with four arrows for the beacons, a tag with name and height next to each and its working area (schematic); KELO robot; triangle A–B–C
on the tape marks; out-and-back stops D and E. Lateral zoom on the robot: 3D view of the rigid
body and of the mobile beacon in the two mountings.

**Names used everywhere** (Marvelmind map frame, approximate): A (1.15, −3.30), B (1.20, −1.40),
C (2.70, −1.85), D (4.40, −3.30), E (5.10, −1.90) m. Beacons in the photo: B1 ceiling near-left, B4
above the window near the tripod, B2 left wall at the back, B5 on the mural wall near the window corner
(deduced from the right-handed map frame; to be confirmed by Andrea).

**Caption.** *Test setup: Marvelmind stationary beacons (height above the floor), OptiTrack bar and working area, triangle A–B–C and out-and-back stops D, E. The zoom on the KELO robot shows the OptiTrack markers, the rigid body pivot and the mobile beacon in the mountings M1 and M2.*

## Floor plan (single column)

`figures/fig_floorplan.pdf` — Marvelmind map frame: beacons with heights, OptiTrack coverage, triangle
A–B–C, stops D and E, out-and-back path.

**Caption.** *Floor plan in the Marvelmind map frame: stationary beacons with their height, area covered
by OptiTrack, triangle A–B–C and out-and-back stops D and E (names as in Fig. 1).*

## Previous Fig. 1 (floor plan + 3D, two columns, no longer used in main.tex)

`figures/fig1_setup.pdf` — (a) floor plan in the Marvelmind map frame: beacons with heights, OptiTrack
coverage, triangle, out-and-back path. (b) 3D view of the rigid body in the robot body frame: two
groups of 5 markers on the mast, 3 lower markers 18.5 cm below at the height of the hedgehog, pivot,
hedgehog in the two mountings (M1, M2). Check: hedgehog 42 cm from the lower markers in mounting 1
(tape 40 cm), 10 cm from their centre in mounting 2 (tape about 5 cm); hedgehog 0.69 m above the floor
vs 0.70 m reported by Marvelmind with the correct map.

**Caption.** *Experimental setup. (a) Floor plan with the stationary beacons (height above the floor),
the area covered by OptiTrack, the triangle and the out-and-back path. (b) OptiTrack markers on the
robot, rigid body pivot and position of the Marvelmind hedgehog in the two mountings (robot body
frame).* Add the photo of the robot when available.

## Fig. 2: C1 slow triangle, as recorded and corrected (optional)

**Caption.** *Campaign 1, slow triangle: Marvelmind positions (dots) against the OptiTrack reference
(line), (a) as recorded with the wrong height of beacon 1, (b) after the correction.*

## Fig. 3: CDF per test type

Four curves per panel (rotation, triangle, rest): C1 as recorded (dotted), C1 z filter only
(dash-dot), C1 corrected (solid), C2 reference (dashed). The corrected curve lies on the C2 curve; the
z filter alone stays between.

**Caption.** *Error distribution per test type: campaign 1 as recorded, with the height filter only,
and after the correction, and campaign 2 with the correct configuration.*

## Fig. 4: at rest, both campaigns

(a) Offset at each stop on the vertices A, B, C (colour and marker = vertex; open: C1 corrected, filled: C2).
(b) Offset against heading, rotation in place, two runs per campaign.

| Vertex | C1: stops, mean offset, spread (std x / y) | C2: stops, mean offset, spread |
|---|---|---|
| A | 7, 3.7 cm, 8.9 / 3.5 cm | 3, 2.0 cm, 7.8 / 8.4 cm |
| B | 9, 5.6 cm, 2.8 / 3.8 cm | 3, 4.2 cm, 7.7 / 4.4 cm |
| C | 6, 10.5 cm, 4.9 / 4.4 cm | 3, 2.7 cm, 1.2 / 2.6 cm |

Rotation in place: in C2 the two runs repeat (−44°: 2.9–4.5 cm, −134°: 15.4–16.5 cm); in C1 they do
not (up to 14.6 cm apart at the same heading). See "Heading: registration or IPS?" before writing. Jitter within a stop about 1.4 mm.

**Caption.** *Offset at rest. (a) Offset at each stop on the three triangle vertices, campaign 1
corrected (open) and campaign 2 (filled). (b) Offset against the robot heading for the rotation in
place, two runs per campaign.*

## Fig. 5: error vs speed (optional)

No clear dependence on the speed up to 0.6 m/s: 7–16 cm RMS in both campaigns. Can become one
sentence. The "fast" C1 triangle has few samples above 0.2 m/s: check the speeds with Berat.

## Fig. 6: Allan deviation at rest, both campaigns

| | C1 rest, 18 min (corrected) | C2 rest, 10 min |
|---|---|---|
| Jitter (std, 2D) | 4.0 mm | 4.1 mm |
| Allan deviation at 1 / 10 / 60 s | 1.6 / 0.5 / 0.23 mm | 1.6 / 0.5 / 0.21 mm |
| Offset to the reference | 2.6 cm | 5.5 cm |

Same noise in both campaigns; precision and accuracy differ by more than an order of magnitude.
Note that the offsets are within the registration uncertainty (Table II).

## Fig. 7: out-and-back test (C1)

Error before leaving / after re-entry 5.4 / 4.4 cm (median, 10 re-entries); spread at the outside
points D 5.8 / 6.9 cm, E 7.6 / 9.6 cm (std x / y, 4 stops each, upper bound). No absolute accuracy
outside the OptiTrack area.

**Caption.** *Out-and-back test. (a) Marvelmind error over time; grey: outside the OptiTrack area.
(b) Paths and Marvelmind positions at the stops D and E outside the area.*

## Fig. 8: Monte Carlo of beacon height errors

`figures/fig8_montecarlo.pdf` — description, numbers and caption in the section "Monte Carlo of
beacon height errors" above. (a) p95 position error vs |height error| for each beacon (filled: set too
low, open: too high; dotted: after the correction; star: C1 measured). (b) Median error of the height
reported for the mobile beacon, with the 10 cm line of the height check.

---

## Which figures and tables in 6 pages

Recommended: Fig. 1 (photo), floor plan, 3, 4, 6, 7, 8, Table I (with the z-filter column and the Monte
Carlo intervals) and Table II. Triple vs heading and sensitivity in the text. Fig. 2 and Fig. 5 only if
space allows.
