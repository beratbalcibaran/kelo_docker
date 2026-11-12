# Scripts for the preliminary figures

These scripts read the data **embedded in the HTML reports** (time, x, y already aligned), not the
raw recordings. They are a first look; the final figures must be made from the raw files.

1. `extract.py <reports dir> <out dir>`: writes `run1.json` and `last40.json` from
   `docs/data_of_all_tests/reports/` (morning of 12 November).
2. The afternoon data come from the `RECS` block of `KELO_Tests_Report_EN.html` in
   `KELO_Testleri_12Kasim.zip` (sent by e-mail), saved as `pm_recs.json`, plus the tables in its
   `tablolar/` folder.
3. `figures.py <figures dir> <unzipped KELO_Testleri_12Kasim dir>`, run in the folder with the JSON
   files, writes the PDF/PNG figures and `figures_numbers.json`. Needs numpy and matplotlib.
`analyse.py` holds the shared functions (loading, jump rejection, Kalman filter, RTS smoother).
