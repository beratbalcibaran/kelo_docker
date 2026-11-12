# Figures, tables and analyses for the I2MTC paper

Everything here is produced by `scripts/analysis.py` (run `python paper/scripts/analysis.py .` from the
repository root after unzipping `experiments/archives/2026-11-12/*.zip` into `experiments/extracted/`).
All numbers are in `figures/numbers.json`. Figures are sized for one IEEE column (Fig. 1: two columns)
and readable in black and white.

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
   statistics.
2. **Cross-campaign registration** (OptiTrack → Marvelmind map: rotation, translation, lever arm,
   robust least squares on stop medians). No recording is evaluated with parameters fitted on itself:
   - C1 recordings: registration fitted on the **C2 stops**; lever arm of mounting 1 fitted on the C1
     rotation run that is not being evaluated (test1 ↔ test2);
   - C2 recordings: registration fitted on the **C1 stops with the same mounting** (out-and-back and
     rest, corrected data).
3. **Delay**: best delay per recording with motion 0.04–0.21 s; applied value = median, 0.105 s.
4. OptiTrack frames with mean marker error > 3 mm discarded. Stops: speed < 2 cm/s and rotation
   < 3 °/s for at least 10 s, 2 s trimmed at both ends.

---

## Table I: C1 as recorded, z filter only, corrected, and C2

Median / p95 (cm). Stops: per-stop offset. Moving: RMS / p95. Last column: difference of the median
stop offsets C1 corrected − C2, 95 % bootstrap interval.

| Test | C1 as recorded | C1 z filter only | C1 corrected | C2 reference | C1 corr. − C2 |
|---|---|---|---|---|---|
| Rotation, stops (8 / 8) | 16.6 / 85.8 | – | 4.3 / 14.0 | 8.4 / 16.1 | −4.0 [−11.8, 3.1] |
| Triangle, stops (22 / 9) | 20.7 / 126.7 | – | 7.8 / 14.2 | 5.8 / 14.2 | 2.0 [−4.3, 7.1] |
| Triangle, moving | 65.6 / 141.5 | 20.4 RMS | 11.0 / 21.1 | 11.1 / 23.4 | – |
| Rest (1 / 1) | 35.9 | 35.8 | 2.6 | 5.5 | – |

All samples with reference (median / p95, and share of samples kept):

| Test | C1 z filter only | C1 corrected |
|---|---|---|
| Rotation | 14.0 / 20.6, 88 % kept | 5.5 / 14.9, 89 % kept |
| Triangle | 12.4 / 50.4, 59 % kept | 7.7 / 19.9, 85 % kept |
| Out-and-back (inside) | 9.9 / 44.3, 55 % kept | 6.5 / 20.3, 81 % kept |
| Rest | 35.8 / 36.3, 100 % kept | 2.6 / 3.1, 100 % kept |

**Messages.**
- The configuration error multiplies the errors by 3–10 (p95 up to 1.4 m).
- The simple fix, discarding samples with a wrong height, removes the worst outliers but keeps the
  bias (rest: 35.8 cm) and throws away up to 45 % of the data. The correction keeps 81–100 % of the
  samples and brings the errors down to the C2 level.
- C1 corrected and C2 agree: the median differences are within ±4 cm with intervals that include
  zero, the moving errors differ by 0.1 cm RMS. With 8–22 stops per campaign the intervals are about
  ±7 cm wide: write "agree within the resolution of the comparison", not "equivalent".

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
with it. Precision (Allan deviation) does not depend on the registration.

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
This explains the repeatable heading effect of Fig. 4b. Say "consistent with", because the triple is
inferred.

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

## Fig. 1: setup (two columns)

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

(a) Offset at each stop on the three vertices (colour = vertex; open: C1 corrected, filled: C2).
(b) Offset against heading, rotation in place, two runs per campaign.

| Vertex | C1: stops, mean offset, spread (std x / y) | C2: stops, mean offset, spread |
|---|---|---|
| 1 | 6, 10.5 cm, 4.9 / 4.4 cm | 3, 2.7 cm, 1.2 / 2.6 cm |
| 2 | 7, 3.7 cm, 8.9 / 3.5 cm | 3, 2.0 cm, 7.8 / 8.4 cm |
| 3 | 9, 5.6 cm, 2.8 / 3.8 cm | 3, 4.2 cm, 7.7 / 4.4 cm |

Rotation in place, the same pattern in both campaigns: −44° gives 2.6–5.5 cm in all four runs,
−134° gives 12.9–16.5 cm in the three runs that have it. Jitter within a stop about 1.4 mm.

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
points C 5.8 / 6.9 cm, D 7.6 / 9.6 cm (std x / y, 4 stops each, upper bound). No absolute accuracy
outside the OptiTrack area.

---

## Which figures and tables in 6 pages

Recommended: Fig. 1 (with the photo), 3, 4, 6, 7, Table I (with the z-filter column) and Table II.
Triple vs heading and sensitivity in the text. Fig. 2 and Fig. 5 only if space allows.
