"""Correct the Marvelmind positions of campaign 1 (12 November; beacon 1 height not applied).

During campaign 1 the Marvelmind map used beacon 1 at z = 0 m instead of 2.6 m, so the
map (and the positions) were wrong. The distances measured by the hedgehog were fine: with the
corrected map (campaign 2) the four measured distances of a sample shown in the Dashboard agree within
4 mm.

Marvelmind computes each position from three beacons. For each recorded sample this script
1. rebuilds the distances to each possible triple of beacons from the position and the wrong map,
2. recomputes the position with the corrected map for every triple,
3. picks the triple whose height matches the known hedgehog height and keeps the track continuous
   (dynamic programming over time). OptiTrack is NOT used.

Usage: python b1_correction.py <raw csv> <output csv>
Output: one row per Marvelmind sample: t, x, y, z (corrected map), triple, z_dev, valid.
"""
import csv, sys, itertools
import numpy as np

WRONG = {1: (0.506, -0.004, 0.0), 2: (6.457, -0.013, 2.5), 4: (0.055, -5.052, 2.4), 5: (9.65, -3.83, 2.55)}  # map 17:41
RIGHT = {1: (0.204, 0.124, 2.6), 2: (6.672, 0.169, 2.5), 4: (0.116, -5.484, 2.4), 5: (9.65, -3.83, 2.55)}    # map 20:24
HEDGE_Z = 0.70      # hedgehog height in the corrected map (Dashboard sample and campaign 2 recordings: 0.65-0.72 m)
Z_TOL = 0.10        # samples whose best triple is further than this from HEDGE_Z are marked invalid
JUMP_W = 2.0        # weight of the jump between consecutive samples in the continuity cost
TRIPLES = list(itertools.combinations([1, 2, 4, 5], 3))
W = {k: np.array(v) for k, v in WRONG.items()}; Rt = {k: np.array(v) for k, v in RIGHT.items()}

def trilaterate(B, d):
    p1, p2, p3 = B
    ex = (p2 - p1) / np.linalg.norm(p2 - p1); i = ex @ (p3 - p1)
    ey = p3 - p1 - i * ex; ey /= np.linalg.norm(ey); ez = np.cross(ex, ey)
    D = np.linalg.norm(p2 - p1); j = ey @ (p3 - p1)
    x = (d[0] ** 2 - d[1] ** 2 + D ** 2) / (2 * D)
    y = (d[0] ** 2 - d[2] ** 2 + i ** 2 + j ** 2) / (2 * j) - i * x / j
    z = np.sqrt(max(d[0] ** 2 - x ** 2 - y ** 2, 0))
    a, b = p1 + x * ex + y * ey + z * ez, p1 + x * ex + y * ey - z * ez
    return a if a[2] < b[2] else b          # beacons on top: the hedgehog is below them

def candidates(p):
    return np.array([trilaterate([Rt[k] for k in tri], [np.linalg.norm(p - W[k]) for k in tri]) for tri in TRIPLES])

def correct(P):
    C = np.array([candidates(p) for p in P])                     # n x 4 x 3
    zdev = np.abs(C[:, :, 2] - HEDGE_Z)
    n, k = zdev.shape; cost = zdev[0].copy(); back = np.zeros((n, k), int)
    for j in range(1, n):
        jump = np.linalg.norm(C[j][None, :, :2] - C[j - 1][:, None, :2], axis=2)
        tot = cost[:, None] + JUMP_W * jump
        back[j] = np.argmin(tot, axis=0); cost = tot[back[j], range(k)] + zdev[j]
    s = np.zeros(n, int); s[-1] = np.argmin(cost)
    for j in range(n - 1, 0, -1): s[j - 1] = back[j, s[j]]
    return C[range(n), s], s, zdev[range(n), s]

if __name__ == '__main__':
    rows = [r for r in csv.DictReader(open(sys.argv[1])) if r['src'] == 'mm' and r['x'] != '' and r['valid'] == '1']
    P = np.array([[float(r['x']), float(r['y']), float(r['z'])] for r in rows])
    X, s, zd = correct(P)
    with open(sys.argv[2], 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['t', 'x', 'y', 'z', 'triple', 'z_dev', 'valid', 'device_ms'])
        for r, x, si, z in zip(rows, X, s, zd):
            w.writerow([r['t'], f'{x[0]:.4f}', f'{x[1]:.4f}', f'{x[2]:.4f}', '-'.join(map(str, TRIPLES[si])), f'{z:.3f}', int(z < Z_TOL), r['aux1']])
    print(f'{sys.argv[1]}: {len(rows)} samples, valid after correction {np.mean(zd < Z_TOL) * 100:.1f} %')
