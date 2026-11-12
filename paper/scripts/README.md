# Scripts for the paper

Unzip the archives first (from the repository root):

    for z in experiments/archives/*/*.zip; do unzip -q -o "$z" -d experiments/extracted/; done

1. `b1_correction.py <raw csv> <output csv>`: recomputes the afternoon Marvelmind positions of
   12 November with the corrected beacon 1 height (no OptiTrack). Outputs are already in
   `experiments/2026-11-12/afternoon_corrected/`.
2. `analysis.py <repository root>`: alignment, metrics and all figures of the paper; writes
   `paper/figures/*.pdf|png` and `paper/figures/numbers.json`. Needs numpy, scipy, matplotlib.

What each figure shows and the numbers: `paper/FIGURES.md`.
