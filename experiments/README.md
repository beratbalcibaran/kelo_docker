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

Screenshots of 18:07 (Marvelmind Dashboard with beacon 1 at z = 0, Motive rigid body) are in
`2026-11-12/screenshots/`.

**Afternoon configuration error.** Beacon 1 is mounted at 2.6 m, but the Marvelmind map used 0 m
(the height shown in the Dashboard had not been applied). The afternoon positions are affected:
wrong 3D solutions in 4–49 % of the samples and errors of 30–60 cm. In the 20:24 map beacon 1 has
the right height but is **sleeping** (battery 2.58 V), so the evening trials probably used only
beacons 2, 4 and 5.
