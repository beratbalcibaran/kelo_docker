# Figures for the I2MTC paper: what they show and how to use them

Figures from the two measurement campaigns of **12 November 2026**, made by `scripts/analysis.py`
(run `python paper/scripts/analysis.py .` from the repository root after unzipping
`experiments/archives/2026-11-12/*.zip` into `experiments/extracted/`). All numbers are in
`figures/numbers.json`. Figures are sized for one IEEE column (Fig. 1: two columns) and readable in
black and white.

## Experimental design: two campaigns

| | Campaign 1 (C1): main test programme | Campaign 2 (C2): reference configuration |
|---|---|---|
| Purpose | characterisation over all test types | same robot poses with the correct Marvelmind map; used to fit the alignment and to validate the correction of C1 |
| Marvelmind map | beacon 1 at 0 m instead of 2.6 m (value not applied in the map); positions recomputed afterwards with the corrected map | correct (beacon 1 at 2.6 m) |
| Rotation in place, 4 headings | 2 runs (`test1`, `test2`) | 2 runs (`deneme_2025`, `deneme_2028`) |
| Triangle (3 vertices, same poses in C1 and C2) | fast (`test2_triangle`) and slow (`test4_triangle`) | 1 run (`deneme_triangle_2031`) |
| Rest | 18 min (`test6_wait`) | 10 min (`denemenew_wait_2040`) |
| Out-and-back (leaves the OptiTrack area) | 10 laps (`test5_in_out1`) | – |

The robot poses are the same in the two campaigns: the three triangle vertices with the same headings
(about 46° and ±178°) and the four headings of the rotation in place (±44°, ±134°).

**Hedgehog mounting.** The hedgehog was remounted during C1: mounting 1 for the C1 rotations and
triangles, mounting 2 for the out-and-back, the C1 rest and all of C2. Lever arm with respect to the
rigid body pivot: (−0.144, −0.141) m and (0.214, 0.241) m (52 cm apart; tape about 45 cm).

**Processing.**
1. C1 positions recomputed by `scripts/b1_correction.py` **without OptiTrack**: distances to each
   triple of beacons rebuilt from the recorded position and the wrong map, position recomputed with
   the correct map, triple chosen by the known hedgehog height and continuity. Validation: the four
   distances of one sample shown in the Dashboard fit the correct map within 4 mm. 81–100 % of the
   samples remain valid; one stop out of 23 (C1 slow triangle, 39 samples) is corrected wrongly
   (167 cm), reported and excluded from the per-stop statistics.
2. Alignment OptiTrack → Marvelmind map (rotation, translation, lever arm) fitted **only on the C2
   stops** (rotations and triangle). The lever arm of mounting 1 is fitted on the C1 rotations with
   rotation and translation fixed. Delay 0.09 s, fitted on the C2 triangle.
3. OptiTrack frames with mean marker error above 3 mm discarded. Stops: speed < 2 cm/s and rotation
   < 3 °/s for at least 10 s, 2 s trimmed at both ends.

## Table I: C1 as recorded, C1 corrected, C2 (use it in the paper)

Offset at the stops (median / p95, cm), moving error for the triangles (RMS / p95, cm), and the
difference of the median offsets C1 corrected − C2 with its 95 % bootstrap interval.

| Test | Stops C1 / C2 | C1 as recorded | C1 corrected | C2 reference | C1 corr. − C2 |
|---|---|---|---|---|---|
| Rotation in place, at rest | 8 / 8 | 16.6 / 85.8 | 4.0 / 12.1 | 7.4 / 13.5 | −3.4 [−10.1, 2.2] |
| Triangle, at rest | 22 / 9 | 20.7 / 126.7 | 7.8 / 14.2 | 4.9 / 13.3 | 2.8 [−5.1, 5.4] |
| Triangle, moving (RMS / p95) | – | 65.4 / 142.2 | 11.1 / 21.2 | 10.8 / 21.6 | – |
| Rest (18 min / 10 min) | 1 / 1 | 35.9 | 2.6 | 7.4 | – |

**Message.** The configuration error multiplies the errors by 3–10 (p95 up to 1.4 m). After the
correction, C1 is statistically indistinguishable from C2: the intervals of the differences include
zero and the moving errors are the same within 0.4 cm. The correction recovers the performance of a
correctly configured system.

Per recording (median / p95 of all samples with reference, cm):

| Recording | As recorded | Corrected / correct map | Moving RMS / p95 | Valid after correction |
|---|---|---|---|---|
| C1 rotation 1 | 6.6 / 17.8 | 4.9 / 13.6 | – | 82 % |
| C1 rotation 2 | 19.9 / 129.5 | 3.5 / 11.8 | – | 95 % |
| C1 triangle, fast | 23.5 / 50.9 | 7.0 / 9.9 | 9.4 / 18.1 | 86 % |
| C1 triangle, slow | 17.1 / 140.6 | 10.3 / 20.6 | 15.6 / 22.4 | 84 % |
| C1 out-and-back (inside) | 13.4 / 150.9 | 6.4 / 20.2 | 11.1 / 17.7 | 81 % |
| C1 rest, 18 min | 35.8 / 36.3 | 2.6 / 3.1 | – | 100 % |
| C2 rotation 1 / 2 | – | 10.9 / 14.4, 7.2 / 13.2 | – | – |
| C2 triangle | – | 8.2 / 13.8 | 10.8 / 21.6 | – |
| C2 rest, 10 min | – | 7.4 / 7.9 | – | – |

---

## Fig. 1: setup (two columns)

`figures/fig1_setup.pdf`

(a) Floor plan in the Marvelmind map frame: stationary beacons with their heights, area covered by
OptiTrack, triangle, and the out-and-back path that leaves the OptiTrack area. (b) 3D view of the
rigid body in the robot body frame: two groups of 5 markers on the mast, 3 lower markers 18.5 cm
below them at the height of the hedgehog, the pivot, and the hedgehog in the two mountings (M1, M2).

Check of the geometry (Motive exports the markers with y as the vertical axis): the hedgehog is 42 cm
from the lower markers in mounting 1 (tape: 40 cm) and 10 cm from their centre in mounting 2 (tape:
about 5 cm from the markers); its height, 0.69 m from the floor, matches the 0.70 m reported by
Marvelmind with the correct map.

**Use.** Section III. Add the photo of the robot as a third panel.

**Caption.** *Experimental setup. (a) Floor plan with the stationary beacons (height above the floor),
the area covered by OptiTrack, the triangle and the out-and-back path. (b) OptiTrack markers on the
robot, rigid body pivot and position of the Marvelmind hedgehog in the two mountings (robot body
frame).*

---

## Fig. 2: C1 slow triangle, as recorded and corrected (optional)

`figures/fig2_correction.pdf`

**Caption.** *Campaign 1, slow triangle: Marvelmind positions (dots) against the OptiTrack reference
(line), (a) as recorded with the wrong height of beacon 1, (b) after the correction.*

---

## Fig. 3: CDF per test type, C1 as recorded / C1 corrected / C2

`figures/fig3_cdf.pdf`

Three panels (rotation in place, triangle, rest), all samples with reference (at rest and moving).

**Message.** This is the central figure of the coupled design: the corrected C1 curve (solid) lies on
the C2 reference curve (dashed), far from the as-recorded curve (dotted).

**Caption.** *Error distribution per test type: campaign 1 as recorded (dotted), campaign 1 after the
correction (solid) and campaign 2 with the correct configuration (dashed).*

---

## Fig. 4: at rest, both campaigns

`figures/fig4_rest.pdf`

(a) Offset at each stop on the three triangle vertices (colour = vertex; open: C1 corrected, filled:
C2). (b) Offset against the robot heading in the rotation in place, two runs per campaign.

| Vertex | C1: stops, mean offset, spread (std x / y) | C2: stops, mean offset, spread |
|---|---|---|
| 1 | 6, 10.5 cm, 4.9 / 4.4 cm | 3, 2.5 cm, 1.3 / 2.5 cm |
| 2 | 7, 3.7 cm, 8.9 / 3.5 cm | 3, 1.3 cm, 8.0 / 8.6 cm |
| 3 | 9, 5.6 cm, 2.8 / 3.8 cm | 3, 7.0 cm, 7.7 / 4.4 cm |

Rotation in place: the same pattern in both campaigns, although the hedgehog mounting and the map
were different: −44° gives 0.4–4.7 cm in all four runs, −134° gives 10–14 cm in three of four.

**Message.** The offset at rest is a pose-dependent bias of a few cm that changes between visits and
with the heading, much larger than the jitter within a stop (about 1.4 mm). Its repetition in both
campaigns shows that it belongs to the system, not to the correction.

**Caption.** *Offset at rest. (a) Offset at each stop on the three triangle vertices, campaign 1
corrected (open) and campaign 2 (filled). (b) Offset against the robot heading for the rotation in
place, two runs per campaign.*

---

## Fig. 5: error vs speed (optional)

`figures/fig5_speed.pdf`

RMS error in speed bins (bins with fewer than 30 samples not shown), C1 triangles, C2 triangle and C1
out-and-back.

**Message.** No clear dependence on the speed up to 0.6 m/s once the map is correct: 7–16 cm RMS in
both campaigns. Can be replaced by one sentence. Note: the "fast" C1 triangle has few samples above
0.2 m/s; check the speeds of the two triangles with Berat.

**Caption.** *RMS error of Marvelmind as a function of the speed measured by OptiTrack.*

---

## Fig. 6: Allan deviation at rest, both campaigns

`figures/fig6_allan.pdf`

| | C1 rest, 18 min (corrected) | C2 rest, 10 min |
|---|---|---|
| Jitter (std, 2D) | 4.0 mm | 4.1 mm |
| Allan deviation at 1 / 10 / 60 s | 1.6 / 0.5 / 0.23 mm | 1.6 / 0.5 / 0.21 mm |
| Offset to the reference | 2.6 cm | 7.4 cm |

**Message.** The two campaigns have the same noise (curves on top of each other), and precision and
accuracy differ by more than an order of magnitude: averaging lowers the noise to a fraction of a
millimetre, the offset of a few cm stays.

**Caption.** *Allan deviation of the Marvelmind position at rest in the two campaigns, and of
OptiTrack. The dotted lines give the offset of each recording from the reference.*

---

## Fig. 7: out-and-back test (C1)

`figures/fig7_inout.pdf`

| | Value |
|---|---|
| Re-entries evaluated | 10 |
| Error in the 8 s before leaving / after re-entry (median) | 5.3 / 4.3 cm |
| Spread at the outside points (std x / y, 4 stops each) | C: 5.8 / 6.9 cm, D: 7.6 / 9.6 cm |

**Message.** Leaving the OptiTrack area does not degrade Marvelmind; outside, the repeatability over
the visits is 6–10 cm (upper bound: it includes how precisely the robot was stopped on the floor
marks). No absolute accuracy outside the OptiTrack area (floor marks not surveyed).

**Caption.** *Out-and-back test. (a) Marvelmind error over time; grey: outside the OptiTrack area.
(b) Paths and Marvelmind positions at the outside stops.*

---

## Which figures in 6 pages

Recommended: Fig. 1 (with the photo), 3, 4, 6, 7 and Table I. Fig. 2 and Fig. 5 only if space allows.
