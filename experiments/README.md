# Experiments (I2MTC paper)

Raw recordings and reports, kept as zip archives. `experiments/` is excluded from the Docker build
(`.dockerignore`). To work on them, unzip into `experiments/extracted/` (ignored by git):

    for z in experiments/archives/*/*.zip; do unzip -q -o "$z" -d experiments/extracted/; done

The morning data of 12 November (Experiment 0) stay in `docs/data_of_all_tests/`.

## 12 November 2026

| Archive | Session | Content |
|---|---|---|
| `KELO_Testleri_12Kasim.zip` | afternoon, 18:07–19:27 | Berat's reports, figures and tables of the 5 afternoon recordings (no raw data) |
| `KELO_6Tests_Unfiltered.zip` | afternoon | raw CSV of 6 recordings: `test1`, `test2` (rotation in place), `test2_triangle` (fast), `test4_triangle` (slow), `test5_in_out1` (out of the OptiTrack area and back), `test6_wait` (18 min at rest); settings (Marvelmind map, Motive rigid body, calibration) |
| `KELO_6Tests_Filtered.zip` | afternoon | same, with Marvelmind samples outside z −0.4…+0.4 m marked invalid |
| `KELO_B1fix_Trials_Unfiltered.zip` | evening, 20:25–20:39 | after setting the beacon 1 height: `deneme_2025`, `deneme_2028` (rotation in place), `deneme_triangle_2031` (triangle); new map `marvelmind_map_2026-11-12_2024.ini` with the measured beacon-to-beacon distances, and the map before the fix |
| `KELO_B1fix_Trials_Filtered.zip` | evening | same, z filtered |
| `KELO_Wait_AfterB1Fix_Unfiltered.zip` | evening, 20:40 | `denemenew_wait_2040`: 10 min at rest with the corrected map |
| `KELO_Wait_AfterB1Fix_Filtered.zip` | evening | same, z filtered |

Screenshots of 18:07 and after the fix (Marvelmind Dashboard with beacon 1 at z = 0, Motive rigid body) are in
`2026-11-12/screenshots/`.

**Afternoon configuration error.** Beacon 1 is mounted at 2.6 m, but the Marvelmind map used 0 m
(the height shown in the Dashboard had not been applied). The afternoon positions are affected:
wrong 3D solutions in 4–49 % of the samples and errors of 30–60 cm. In the 20:24 map beacon 1 has
the right height but is **sleeping** (battery 2.58 V), so the evening trials probably used only
beacons 2, 4 and 5.

## Correction of the afternoon recordings

`paper/scripts/b1_correction.py` recomputes the afternoon Marvelmind positions with the corrected
map (20:24), without using OptiTrack. The measured distances were fine (a Dashboard sample at 18:07
fits the corrected map within 4 mm); only the map was wrong. For each sample the script rebuilds the
distances to each triple of beacons, recomputes the position with the corrected map, and keeps the
triple that gives the known hedgehog height (0.70 m in the corrected map) with a continuous track.
Results in `2026-11-12/afternoon_corrected/<recording>_corrected.csv` (`valid` = 0 when no triple
gives a plausible height: 0-10 % of the samples).

Check against OptiTrack (alignment estimated only on the evening recordings):

| Recording | Before (median) | After correction (median / p95) |
|---|---|---|
| `test6_wait` (18 min at rest) | 35 cm | 3.5 / 3.9 cm |
| `test1`, `test2` (rotation) | 26-32 cm | 4-9 / 12-17 cm |
| `test2_triangle`, `test4_triangle` | 42 cm | 6-11 / 24-56 cm |
| `test5_in_out1` | 7 cm (lever) / 28 cm (common fit) | 7 / 21 cm |

**The hedgehog was moved between `test4_triangle` (18:29) and `test5_in_out1` (18:51).** Lever arm
with respect to the rigid body pivot, estimated from the data: (−0.114, −0.155) m for `test1`–`test4`,
(0.210, 0.246) m from `test5` on (evening). The difference is 51 cm; the tape gave about 45 cm.
