import argparse
import csv
import math
import sys

import numpy as np
import yaml

from kelo_bringup.common import alignment_file
from kelo_bringup.logic import apply_se2, kabsch_2d, path_length

MIN_INLIER_RATIO = 0.90


def calibrate(rows, max_rms, min_samples=30, min_path=3.0):
    arr = np.asarray(rows, dtype=float)
    if len(arr) < min_samples:
        raise ValueError(f'Not enough samples: {len(arr)} < {min_samples}')
    plen = path_length(arr[:, :2])
    if plen < min_path:
        raise ValueError(f'Path too short: {plen:.2f} m < {min_path} m (cover more of the area)')
    yaw, tx, ty, err = kabsch_2d(arr[:, :2], arr[:, 2:4])
    keep = err <= err.mean() + 3.0 * err.std()
    if min_samples <= keep.sum() < len(arr):
        yaw, tx, ty, _ = kabsch_2d(arr[keep, :2], arr[keep, 2:4])
    px, py = apply_se2(yaw, tx, ty, arr[:, 0], arr[:, 1])
    all_err = np.hypot(px - arr[:, 2], py - arr[:, 3])
    inlier_err = all_err[keep] if keep.any() else all_err
    rms_in = float(np.sqrt(np.mean(inlier_err ** 2)))
    rms_all = float(np.sqrt(np.mean(all_err ** 2)))
    inlier_ratio = float(keep.mean())
    return {
        'x': float(tx), 'y': float(ty), 'yaw': float(yaw),
        'stats': {
            'samples': int(len(arr)),
            'inliers': int(keep.sum()),
            'outliers': int((~keep).sum()),
            'path_m': round(plen, 2),
            'rms_m': round(rms_in, 4),
            'rms_all_m': round(rms_all, 4),
            'mean_m': round(float(inlier_err.mean()), 4),
            'max_m': round(float(all_err.max()), 4),
            'p95_m': round(float(np.percentile(all_err, 95)), 4),
            'yaw_deg': round(math.degrees(yaw), 2),
            'ok': bool(rms_in <= max_rms and inlier_ratio >= MIN_INLIER_RATIO),
        },
    }


def main():
    ap = argparse.ArgumentParser(description='Marvelmind -> map rijit hizalama')
    ap.add_argument('csv', help='mm_recorder CSV file')
    ap.add_argument('--max-rms', type=float, default=0.15, help='acceptance threshold (m)')
    ap.add_argument('--no-write', action='store_true', help='do not write the file, only report')
    args = ap.parse_args()
    with open(args.csv, encoding='utf-8') as f:
        rows = [(float(r['mm_x']), float(r['mm_y']), float(r['map_x']), float(r['map_y']))
                for r in csv.DictReader(f)]
    try:
        res = calibrate(rows, args.max_rms)
    except ValueError as exc:
        print(f'ERROR: {exc}')
        sys.exit(2)
    print(yaml.safe_dump(res, sort_keys=False, allow_unicode=True))
    if not res['stats']['ok']:
        print(f'WARNING: RMS > {args.max_rms} m or inlier ratio < {MIN_INLIER_RATIO * 100:.0f}%: '
              'alignment NOT written (possible reflections, beacon geometry or line-of-sight issue).')
        sys.exit(1)
    if args.no_write:
        return
    out = alignment_file()
    with open(out, 'w', encoding='utf-8') as f:
        yaml.safe_dump(res, f, sort_keys=False)
    print(f'Written: {out}  (published as a static TF once marvelmind_bridge restarts)')


if __name__ == '__main__':
    main()
