import math

import numpy as np

OCCUPIED, FREE, UNKNOWN = 100, 0, -1


class CameraGrid:

    def __init__(self, z_min=0.06, z_max=1.20, dz=0.10, l_hit=0.85, l_miss=-0.40,
                 l_min=-2.0, l_max=3.5, occ_l=1.5, free_l=-0.4, graze_margin=0.0):
        self.z_min, self.dz = float(z_min), float(dz)
        self.nz = max(1, int(math.ceil((float(z_max) - self.z_min) / self.dz - 1e-9)))
        self.l_hit, self.l_miss = float(l_hit), float(l_miss)
        self.l_min, self.l_max = float(l_min), float(l_max)
        self.occ_l, self.free_l = float(occ_l), float(free_l)
        self.graze = float(graze_margin)
        self.res = None
        self.origin = (0.0, 0.0)
        self.width = self.height = 0
        self.log = np.zeros((0, 0, self.nz), dtype=np.float32)
        self.top = np.full((0, 0, self.nz), -np.inf, dtype=np.float32)

    @property
    def ready(self):
        return self.res is not None and self.width > 0 and self.height > 0

    def align(self, res, ox, oy, width, height):
        res, width, height = float(res), int(width), int(height)
        new = np.zeros((height, width, self.nz), dtype=np.float32)
        top = np.full((height, width, self.nz), -np.inf, dtype=np.float32)
        if self.res is not None and abs(res - self.res) < 1e-9:
            dc = int(round((self.origin[0] - ox) / res))
            dr = int(round((self.origin[1] - oy) / res))
            r0, c0 = max(0, dr), max(0, dc)
            r1, c1 = min(height, dr + self.height), min(width, dc + self.width)
            if r1 > r0 and c1 > c0:
                new[r0:r1, c0:c1] = self.log[r0 - dr:r1 - dr, c0 - dc:c1 - dc]
                top[r0:r1, c0:c1] = self.top[r0 - dr:r1 - dr, c0 - dc:c1 - dc]
        self.res, self.origin = res, (float(ox), float(oy))
        self.width, self.height = width, height
        self.log, self.top = new, top

    def _voxels(self, pts):
        pts = np.asarray(pts, dtype=float).reshape(-1, 3)
        col = np.floor((pts[:, 0] - self.origin[0]) / self.res).astype(np.int64)
        row = np.floor((pts[:, 1] - self.origin[1]) / self.res).astype(np.int64)
        k = np.floor((pts[:, 2] - self.z_min) / self.dz).astype(np.int64)
        ok = ((col >= 0) & (col < self.width) & (row >= 0) & (row < self.height)
              & (k >= 0) & (k < self.nz) & (pts[:, 2] >= self.z_min))
        return (row[ok] * self.width + col[ok]) * self.nz + k[ok], pts[ok, 2]

    def _ray_samples(self, origin, ends):
        o = np.asarray(origin, dtype=float).reshape(1, 3)
        e = np.asarray(ends, dtype=float).reshape(-1, 3)
        if len(e) == 0:
            return np.zeros((0, 3))
        vec = e - o
        length = np.linalg.norm(vec, axis=1)
        step = 0.5 * min(self.res, self.dz)
        usable = np.maximum(length - self.res, 0.0)
        n = int(np.ceil(usable.max() / step)) if len(usable) else 0
        if n == 0:
            return np.zeros((0, 3))
        t = np.arange(n) * step
        keep = t[None, :] < usable[:, None]
        unit = vec / np.maximum(length, 1e-9)[:, None]
        pts = o[:, None, :] + unit[:, None, :] * t[None, :, None]
        return pts[keep]

    def integrate(self, origin, hits, clear_ends):
        if not self.ready:
            return 0, 0
        flat, top = self.log.reshape(-1), self.top.reshape(-1)
        hit_idx, hit_z = self._voxels(hits)
        hit = np.unique(hit_idx)
        first = hit[np.isneginf(top[hit])]
        flat[first] = np.maximum(flat[first], 0.0)
        np.maximum.at(top, hit_idx, hit_z.astype(np.float32))
        ray_idx, ray_z = self._voxels(self._ray_samples(origin, clear_ends))
        crossed, inv = np.unique(ray_idx, return_inverse=True)
        lowest = np.full(len(crossed), np.inf)
        np.minimum.at(lowest, inv, ray_z)
        seen_top = top[crossed]
        through = np.isneginf(seen_top) | (lowest <= seen_top + self.graze)
        miss = np.setdiff1d(crossed[through], hit, assume_unique=True)
        flat[miss] = np.maximum(flat[miss] + self.l_miss, self.l_min)
        flat[hit] = np.minimum(flat[hit] + self.l_hit, self.l_max)
        gone = miss[flat[miss] <= 0.0]
        top[gone] = -np.inf
        return len(hit), len(miss)

    def occupancy(self):
        out = np.full((self.height, self.width), UNKNOWN, dtype=np.int8)
        if not self.ready:
            return out
        occ = (self.log >= self.occ_l).any(axis=2)
        free = (self.log <= self.free_l).any(axis=2) & ~occ
        out[free] = FREE
        out[occ] = OCCUPIED
        return out


def fit_to(grid, src_origin, res, dst_origin, dst_w, dst_h):
    out = np.full((int(dst_h), int(dst_w)), UNKNOWN, dtype=grid.dtype)
    dc = int(round((src_origin[0] - dst_origin[0]) / res))
    dr = int(round((src_origin[1] - dst_origin[1]) / res))
    h, w = grid.shape
    r0, c0 = max(0, dr), max(0, dc)
    r1, c1 = min(dst_h, dr + h), min(dst_w, dc + w)
    if r1 > r0 and c1 > c0:
        out[r0:r1, c0:c1] = grid[r0 - dr:r1 - dr, c0 - dc:c1 - dc]
    return out


def write_map(stem, grid, res, origin):
    pix = np.full(grid.shape, 205, dtype=np.uint8)
    pix[grid == FREE] = 254
    pix[grid == OCCUPIED] = 0
    pix = pix[::-1]
    with open(stem + '.pgm', 'wb') as f:
        f.write(f'P5\n# CREATOR: kelo camera_mapper {res:.3f} m/pix\n{pix.shape[1]} {pix.shape[0]}\n255\n'
                .encode())
        f.write(pix.tobytes())
    with open(stem + '.yaml', 'w', encoding='utf-8') as f:
        f.write(f'image: {stem.rsplit("/", 1)[-1]}.pgm\nmode: trinary\nresolution: {float(res)!r}\n'
                f'origin: [{float(origin[0])!r}, {float(origin[1])!r}, 0]\nnegate: 0\n'
                f'occupied_thresh: 0.65\nfree_thresh: 0.196\n')
