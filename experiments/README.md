# Experiments (I2MTC paper)

Raw recordings and reports, kept as zip archives. `experiments/` is excluded from the Docker build
(`.dockerignore`). To work on them, unzip into `experiments/extracted/` (ignored by git):

    for z in experiments/archives/*/*.zip; do unzip -q -o "$z" -d experiments/extracted/; done

The morning data of 12 November (Experiment 0) stay in `docs/data_of_all_tests/`.

## 12 November 2026: two measurement campaigns

**Campaign 1 (C1), main test programme**: rotation in place (x2), triangle fast and slow,
out-and-back, 18 min at rest. Recorded with an error in the Marvelmind map (beacon 1 height, 2.6 m,
not applied: the map used 0 m); positions recomputed afterwards (see below).
**Campaign 2 (C2), reference configuration**: rotation in place (x2), triangle and 10 min at rest at
the same robot poses, with the correct map. Used to fit the alignment and to validate the correction.

| Archive | Campaign | Content |
|---|---|---|
| `KELO_Testleri_12Kasim.zip` | C1 | Berat's reports, figures and tables of 5 C1 recordings (no raw data) |
| `KELO_6Tests_Unfiltered.zip` | C1 | raw CSV of 6 recordings: `test1`, `test2` (rotation in place), `test2_triangle` (fast), `test4_triangle` (slow), `test5_in_out1` (out-and-back), `test6_wait` (18 min at rest); settings (Marvelmind map, Motive rigid body, calibration) |
| `KELO_6Tests_Filtered.zip` | C1 | same, with Marvelmind samples outside z −0.4…+0.4 m marked invalid |
| `KELO_B1fix_Trials_Unfiltered.zip` | C2 | `deneme_2025`, `deneme_2028` (rotation in place), `deneme_triangle_2031` (triangle); correct map `marvelmind_map_2026-11-12_2024.ini` with the measured beacon-to-beacon distances, and the C1 map |
| `KELO_B1fix_Trials_Filtered.zip` | C2 | same, z filtered |
| `KELO_Wait_AfterB1Fix_Unfiltered.zip` | C2 | `denemenew_wait_2040`: 10 min at rest |
| `KELO_Wait_AfterB1Fix_Filtered.zip` | C2 | same, z filtered |

Screenshots of the Marvelmind Dashboard (C1 map with beacon 1 at z = 0, and the correct map) and of
the Motive rigid body are in `2026-11-12/screenshots/`.

**Hedgehog mounting.** The hedgehog was remounted during C1: mounting 1 for the C1 rotations and
triangles, mounting 2 for the out-and-back, the C1 rest and all of C2. Lever arm with respect to the
rigid body pivot, estimated from the data: (−0.144, −0.141) m and (0.214, 0.241) m, 52 cm apart (tape:
about 45 cm).

## Correction of campaign 1

`paper/scripts/b1_correction.py` recomputes the C1 Marvelmind positions with the correct map, without
using OptiTrack. The measured distances were fine (a Dashboard sample of C1 fits the correct map
within 4 mm); only the map was wrong. For each sample the script rebuilds the distances to each triple
of beacons, recomputes the position with the correct map, and keeps the triple that gives the known
hedgehog height (0.70 m in the correct map) with a continuous track. Results in
`2026-11-12/campaign1_corrected/<recording>_corrected.csv` (`valid` = 0 when no triple gives a
height within 10 cm of the expected one: 0–19 % of the samples).

Validation against OptiTrack and against C2 (cross-campaign registration: each campaign evaluated
with the alignment fitted on the other one): see `paper/FIGURES.md`, Table I. In short, the median
stop offset goes from 17–21 cm (as recorded) to 4–8 cm (corrected), against 6–8 cm in C2; the
differences C1 corrected − C2 are within the resolution of the comparison (registration uncertainty
about 5 cm, Table II). One stop of
`test4_triangle` (318 s, 39 samples) is corrected wrongly (167 cm).
