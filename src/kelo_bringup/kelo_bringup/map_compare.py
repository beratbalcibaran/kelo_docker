DESCRIPTION = """Compare two occupancy maps (precision, recall, F1).

usage:
  ros2 run kelo_bringup map_compare <candidate.yaml> <reference.yaml> [--tol 0.10] [--min-f1 0.85]

exit code 0 when F1 >= min-f1
"""
import argparse
import os
import sys

import numpy as np
import yaml


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    fields, i = [], 0
    while len(fields) < 4:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while data[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        fields.append(data[i:j])
        i = j
    if fields[0] != b'P5':
        raise ValueError(f'only binary PGM (P5) is supported: {path}')
    w, h = int(fields[1]), int(fields[2])
    return np.frombuffer(data[i + 1:i + 1 + w * h], dtype=np.uint8).reshape(h, w)


def load_map(yaml_path):
    with open(yaml_path, encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    img_path = meta['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(os.path.realpath(yaml_path)), img_path)
    img = read_pgm(img_path).astype(float)
    occ = (255.0 - img) / 255.0 if not meta.get('negate', 0) else img / 255.0
    rows, cols = np.nonzero(occ > float(meta.get('occupied_thresh', 0.65)))
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    h = img.shape[0]
    xs = ox + (cols + 0.5) * res
    ys = oy + (h - 1 - rows + 0.5) * res
    return np.stack([xs, ys], axis=1), res


def match_ratio(a, b, tol):
    if len(a) == 0:
        return 0.0
    if len(b) == 0:
        return 0.0
    cell = tol
    keys = {}
    for idx, (x, y) in enumerate(np.floor(b / cell).astype(int)):
        keys.setdefault((x, y), []).append(idx)
    hit = 0
    for p, (cx, cy) in zip(a, np.floor(a / cell).astype(int)):
        found = False
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for j in keys.get((cx + dx, cy + dy), ()):
                    if np.hypot(*(b[j] - p)) <= tol:
                        found = True
                        break
                if found:
                    break
            if found:
                break
        hit += found
    return hit / len(a)


def compare(cand_yaml, ref_yaml, tol=0.10):
    a, _ = load_map(cand_yaml)
    b, _ = load_map(ref_yaml)
    precision, recall = match_ratio(a, b, tol), match_ratio(b, a, tol)
    f1 = 2 * precision * recall / (precision + recall) if precision + recall > 0 else 0.0
    return {'precision': precision, 'recall': recall, 'f1': f1, 'cand_cells': len(a), 'ref_cells': len(b)}


def main():
    ap = argparse.ArgumentParser(description=DESCRIPTION, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('candidate')
    ap.add_argument('reference')
    ap.add_argument('--tol', type=float, default=0.10)
    ap.add_argument('--min-f1', type=float, default=0.85)
    args = ap.parse_args()
    r = compare(args.candidate, args.reference, args.tol)
    ok = r['f1'] >= args.min_f1
    print(f"map_compare: precision={r['precision']:.3f} recall={r['recall']:.3f} F1={r['f1']:.3f} "
          f"(cells {r['cand_cells']} / {r['ref_cells']}, tol {args.tol} m) -> "
          f"{'PASS' if ok else 'FAIL'} (threshold {args.min_f1})")
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
