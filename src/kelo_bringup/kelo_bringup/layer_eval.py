DESCRIPTION = """Task 1 evaluation helpers for a sim world (reference map and per-object coverage).

usage:
  python3 -m kelo_bringup.layer_eval reference <world.yaml> <out_stem> [--band 0.06 1.20]
  python3 -m kelo_bringup.layer_eval coverage <world.yaml> <map.yaml> [--camera] [--tol 0.10]
  python3 -m kelo_bringup.layer_eval false <world.yaml> <map.yaml> [--camera] [--tol 0.10]
  python3 -m kelo_bringup.layer_eval figure <out.png> <reference.yaml> <title>=<map.yaml>[:camera] ...

reference: outline of everything in the height band (walls, table tops, shelves, low objects),
           in the map frame (map origin = robot start pose of the world).
coverage:  share of each object's outline found in a map (lidar map, plus its camera layer with --camera).
false:     occupied cells farther than tol from every object (phantom obstacles). The reference holds
           outlines only, so map_compare counts the filled inside of an object against precision;
           this count does not.
figure:    maps side by side on the extent of the reference: free white, unknown grey, LiDAR map
           obstacles black, cells added by the camera layer orange (':camera' after a map).
"""
import argparse
import math

import numpy as np
import yaml

from kelo_bringup.camera_grid import FREE, OCCUPIED, write_map
from kelo_bringup.map_compare import camera_yaml_of, load_map, match_ratio, read_pgm
from kelo_bringup.sim import world as W

RES = 0.05


def _load(path):
    with open(path, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    st = cfg.get('start', {})
    return cfg, (float(st.get('x', 0.0)), float(st.get('y', 0.0)), float(st.get('yaw', 0.0)))


def _objects(cfg, band):
    """{name: (boxes, cylinders)} of the parts that reach into the height band."""
    room = cfg.get('room', {})
    out = {'walls': ([], [])}
    boxes, cyls = W.load_world({'room': room, 'obstacles': []})
    out['walls'] = (boxes, cyls)
    for ob in cfg.get('obstacles', []) or []:
        b, c = W.obstacle_from_dict(ob)
        b = [x for x in b if x.hi[2] > band[0] and x.lo[2] < band[1]]
        c = [x for x in c if x.z2 > band[0] and x.z1 < band[1]]
        out[ob.get('name', ob.get('type', 'obstacle'))] = (b, c)
    return out


def _raster(boxes, cyls, xs, ys):
    gx, gy = np.meshgrid(xs, ys)
    occ = np.zeros(gx.shape, dtype=bool)
    for b in boxes:
        occ |= (gx >= b.lo[0]) & (gx <= b.hi[0]) & (gy >= b.lo[1]) & (gy <= b.hi[1])
    for c in cyls:
        occ |= np.hypot(gx - c.x, gy - c.y) <= max(c.r, 0.75 * RES)
    return occ


def _outline_points(cfg, parts, interior_only=True):
    room = cfg.get('room', {})
    w, h = float(room.get('width', 8.0)), float(room.get('height', 6.0))
    xs = np.arange(-0.3, w + 0.3, RES) + RES / 2
    ys = np.arange(-0.3, h + 0.3, RES) + RES / 2
    occ = _raster(*parts, xs, ys)
    gx, gy = np.meshgrid(xs, ys)
    inside = (gx > 0) & (gx < w) & (gy > 0) & (gy < h)
    free = inside & ~occ
    near_free = np.zeros_like(occ)
    near_free[1:, :] |= free[:-1, :]
    near_free[:-1, :] |= free[1:, :]
    near_free[:, 1:] |= free[:, :-1]
    near_free[:, :-1] |= free[:, 1:]
    edge = occ & near_free
    return np.stack([gx[edge], gy[edge]], axis=1)


def _to_map(pts, start):
    sx, sy, syaw = start
    c, s = math.cos(-syaw), math.sin(-syaw)
    dx, dy = pts[:, 0] - sx, pts[:, 1] - sy
    return np.stack([c * dx - s * dy, s * dx + c * dy], axis=1)


def object_outlines(cfg, start, band):
    return {name: _to_map(_outline_points(cfg, parts), start)
            for name, parts in _objects(cfg, band).items()}


def reference(world, out_stem, band=(0.06, 1.20)):
    cfg, start = _load(world)
    pts = np.concatenate(list(object_outlines(cfg, start, band).values()))
    lo = np.floor(pts.min(axis=0) / RES) * RES - 5 * RES
    hi = pts.max(axis=0) + 5 * RES
    wd, ht = int(math.ceil((hi[0] - lo[0]) / RES)), int(math.ceil((hi[1] - lo[1]) / RES))
    grid = np.full((ht, wd), FREE, dtype=np.int8)
    idx = np.floor((pts - lo) / RES).astype(int)
    grid[idx[:, 1], idx[:, 0]] = OCCUPIED
    write_map(out_stem, grid, RES, (float(lo[0]), float(lo[1])))
    return len(pts)


def coverage(world, map_yaml, camera=False, tol=0.10, band=(0.06, 1.20)):
    cfg, start = _load(world)
    cand, _ = load_map(map_yaml)
    if camera:
        cam, _ = load_map(camera_yaml_of(map_yaml))
        cand = np.concatenate([cand, cam])
    return {name: match_ratio(pts, cand, tol) for name, pts in object_outlines(cfg, start, band).items()}


def _from_map(pts, start):
    sx, sy, syaw = start
    c, s = math.cos(syaw), math.sin(syaw)
    return np.stack([sx + c * pts[:, 0] - s * pts[:, 1], sy + s * pts[:, 0] + c * pts[:, 1]], axis=1)


def false_obstacles(world, map_yaml, camera=False, tol=0.10, band=(0.06, 1.20)):
    cfg, start = _load(world)
    cand, _ = load_map(map_yaml)
    if camera:
        cand = np.concatenate([cand, load_map(camera_yaml_of(map_yaml))[0]])
    cand = np.unique(np.round(cand, 4), axis=0)
    wx, wy = _from_map(cand, start).T
    near = np.zeros(len(cand), dtype=bool)
    for boxes, cyls in _objects(cfg, band).values():
        for b in boxes:
            dx = np.maximum(0.0, np.maximum(b.lo[0] - wx, wx - b.hi[0]))
            dy = np.maximum(0.0, np.maximum(b.lo[1] - wy, wy - b.hi[1]))
            near |= np.hypot(dx, dy) <= tol
        for c in cyls:
            near |= np.hypot(wx - c.x, wy - c.y) - c.r <= tol
    return {'occupied': int(len(cand)), 'false': int((~near).sum())}


UNKNOWN_C, FREE_C, OCC_C, CAMERA_C = (205, 205, 205), (255, 255, 255), (0, 0, 0), (235, 120, 0)


def _classes(map_yaml, xs, ys):
    """0 unknown, 1 free, 2 occupied at map-frame points (xs, ys) of a map yaml."""
    import os
    with open(map_yaml, encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    img_path = meta['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(os.path.realpath(map_yaml)), img_path)
    img = read_pgm(img_path).astype(float)
    p = (255.0 - img) / 255.0 if not meta.get('negate', 0) else img / 255.0
    res, (ox, oy) = float(meta['resolution']), meta['origin'][:2]
    col = np.floor((xs - ox) / res).astype(int)
    row = img.shape[0] - 1 - np.floor((ys - oy) / res).astype(int)
    out = np.zeros(xs.shape, dtype=np.uint8)
    ok = (col >= 0) & (col < img.shape[1]) & (row >= 0) & (row < img.shape[0])
    v = p[row[ok], col[ok]]
    out[ok] = np.where(v > float(meta.get('occupied_thresh', 0.65)), 2,
                       np.where(v < float(meta.get('free_thresh', 0.25)), 1, 0))
    return out


def figure(out_png, reference_yaml, panels, scale=4):
    """panels: [(title, map_yaml, with_camera)]; the reference outline is the first panel."""
    from PIL import Image, ImageDraw, ImageFont
    ref, res = load_map(reference_yaml)
    lo, hi = ref.min(axis=0) - 0.2, ref.max(axis=0) + 0.2
    xs = np.arange(lo[0], hi[0], res) + res / 2
    ys = np.arange(hi[1], lo[1], -res) - res / 2
    gx, gy = np.meshgrid(xs, ys)
    head = 26
    tiles = []
    for title, map_yaml, cam in [('reference (outline of every object, 0.06-1.20 m)', reference_yaml, False)] + panels:
        rgb = np.empty(gx.shape + (3,), dtype=np.uint8)
        base = _classes(map_yaml, gx, gy)
        rgb[:] = UNKNOWN_C
        rgb[base == 1] = FREE_C
        if cam:
            layer = _classes(camera_yaml_of(map_yaml), gx, gy)
            rgb[(layer == 1) & (base == 0)] = FREE_C
            rgb[(layer == 2) & (base != 2)] = CAMERA_C
        rgb[base == 2] = OCC_C
        img = Image.fromarray(rgb).resize((rgb.shape[1] * scale, rgb.shape[0] * scale), Image.NEAREST)
        tile = Image.new('RGB', (img.width, img.height + head), (255, 255, 255))
        tile.paste(img, (0, head))
        ImageDraw.Draw(tile).text((6, 5), title, fill=(0, 0, 0), font=ImageFont.load_default(size=16))
        tiles.append(tile)
    cols = 2
    w, h = tiles[0].size
    rows = (len(tiles) + cols - 1) // cols
    sheet = Image.new('RGB', (cols * w + (cols - 1) * 10, rows * h + (rows - 1) * 10), (255, 255, 255))
    for i, tile in enumerate(tiles):
        sheet.paste(tile, ((i % cols) * (w + 10), (i // cols) * (h + 10)))
    sheet.save(out_png)
    return sheet.size


def main():
    ap = argparse.ArgumentParser(description=DESCRIPTION, formatter_class=argparse.RawTextHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('reference')
    r.add_argument('world')
    r.add_argument('out_stem')
    r.add_argument('--band', type=float, nargs=2, default=(0.06, 1.20))
    c = sub.add_parser('coverage')
    c.add_argument('world')
    c.add_argument('map')
    c.add_argument('--camera', action='store_true')
    c.add_argument('--tol', type=float, default=0.10)
    c.add_argument('--band', type=float, nargs=2, default=(0.06, 1.20))
    f = sub.add_parser('false')
    f.add_argument('world')
    f.add_argument('map')
    f.add_argument('--camera', action='store_true')
    f.add_argument('--tol', type=float, default=0.10)
    f.add_argument('--band', type=float, nargs=2, default=(0.06, 1.20))
    g = sub.add_parser('figure')
    g.add_argument('out')
    g.add_argument('reference')
    g.add_argument('panels', nargs='+', help='<title>=<map.yaml>[:camera]')
    args = ap.parse_args()
    if args.cmd == 'figure':
        panels = []
        for spec in args.panels:
            title, _, path = spec.rpartition('=')
            cam = path.endswith(':camera')
            panels.append((title, path[:-len(':camera')] if cam else path, cam))
        print(f'figure: {args.out} {figure(args.out, args.reference, panels)}')
    elif args.cmd == 'reference':
        n = reference(args.world, args.out_stem, tuple(args.band))
        print(f'reference: {args.out_stem}.yaml ({n} outline cells)')
    elif args.cmd == 'false':
        print(yaml.safe_dump(false_obstacles(args.world, args.map, args.camera, args.tol, tuple(args.band)),
                             sort_keys=False), end='')
    else:
        cov = coverage(args.world, args.map, args.camera, args.tol, tuple(args.band))
        print(yaml.safe_dump({k: round(float(v), 3) for k, v in cov.items()}, sort_keys=False), end='')


if __name__ == '__main__':
    main()
