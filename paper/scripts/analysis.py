"""Figures and numbers for the I2MTC paper, from the two measurement campaigns of 12 November 2026.

Campaign 1 (main test programme): rotation in place x2, triangle fast and slow, out-and-back, 18 min
  at rest. Recorded with beacon 1 at 0 m instead of 2.6 m in the Marvelmind map; positions recomputed
  with b1_correction.py (experiments/2026-11-12/campaign1_corrected/). OptiTrack and the as-recorded
  Marvelmind data from KELO_6Tests_Unfiltered/data/<rec>_raw.csv.
Campaign 2 (reference configuration, corrected map): rotation in place x2, triangle, 10 min at rest
  at the same robot poses (KELO_B1fix_Trials_Unfiltered, KELO_Wait_AfterB1Fix_Unfiltered).
Unzip experiments/archives/2026-11-12/*.zip into experiments/extracted/ first.

Alignment (OptiTrack -> Marvelmind map): rotation, translation and hedgehog lever arm fitted ONLY on
the stops of campaign 2 (rotations and triangle). The hedgehog was remounted during campaign 1
(mounting 1: rotations and triangles; mounting 2: out-and-back, rests and campaign 2); the lever arm
of mounting 1 is fitted on the campaign 1 rotations with rotation and translation fixed. Triangles,
out-and-back and rests are never used for fitting.

Usage: python analysis.py <repo root>    (needs numpy, scipy, matplotlib)
Writes paper/figures/*.pdf|png and paper/figures/numbers.json.
"""
import csv, json, math, os, sys
import numpy as np
from scipy.optimize import least_squares
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = sys.argv[1] if len(sys.argv) > 1 else '.'
EX = os.path.join(ROOT, 'experiments', 'extracted')
COR = os.path.join(ROOT, 'experiments', '2026-11-12', 'campaign1_corrected')
OUT = os.path.join(ROOT, 'paper', 'figures'); os.makedirs(OUT, exist_ok=True)
NUM = {}

# Campaign 1: main test programme, recorded with an error in the Marvelmind map (beacon 1 height),
# corrected afterwards. Campaign 2: the same robot poses with the corrected map (reference configuration).
CAMPAIGN1 = {'test1': 'C1 rotation 1', 'test2': 'C1 rotation 2', 'test2_triangle': 'C1 triangle, fast',
             'test4_triangle': 'C1 triangle, slow', 'test5_in_out1': 'C1 out-and-back', 'test6_wait': 'C1 rest, 18 min'}
EARLY = ['test1', 'test2', 'test2_triangle', 'test4_triangle']          # hedgehog mounting 1 (then remounted)
CAMPAIGN2 = {'deneme_2025': ('KELO_B1fix_Trials_Unfiltered', 'C2 rotation 1'),
             'deneme_2028': ('KELO_B1fix_Trials_Unfiltered', 'C2 rotation 2'),
             'deneme_triangle_2031': ('KELO_B1fix_Trials_Unfiltered', 'C2 triangle'),
             'denemenew_wait_2040': ('KELO_Wait_AfterB1Fix_Unfiltered', 'C2 rest, 10 min')}
TYPES = {'rotation in place': (['test1', 'test2'], ['deneme_2025', 'deneme_2028']),
         'triangle': (['test2_triangle', 'test4_triangle'], ['deneme_triangle_2031']),
         'rest': (['test6_wait'], ['denemenew_wait_2040'])}
BEACONS = {1: (0.204, 0.124, 2.6), 2: (6.672, 0.169, 2.5), 4: (0.116, -5.484, 2.4), 5: (9.65, -3.83, 2.55)}
# names used in the paper and in the room photo (approximate positions in the Marvelmind map frame, m)
VERTEX_NAMES = {'A': (1.15, -3.30), 'B': (1.20, -1.40), 'C': (2.70, -1.85)}
OUTSIDE_NAMES = {'D': (4.40, -3.30), 'E': (5.10, -1.90)}
def nearest_name(xy, names):
    return min(names, key=lambda k: np.hypot(xy[0] - names[k][0], xy[1] - names[k][1]))
MAX_MARKER_ERR = 0.003   # m: OptiTrack frames with a larger mean marker error are discarded
FAILED_CM = 100          # corrected samples further than this from the reference count as a failed correction (reported, excluded from RMS per bin and per vertex)

plt.rcParams.update({'font.family': 'serif', 'font.size': 8, 'axes.linewidth': 0.6, 'lines.linewidth': 1.0,
                     'savefig.bbox': 'tight', 'savefig.pad_inches': 0.02, 'axes.grid': True, 'grid.linewidth': 0.3,
                     'grid.alpha': 0.5, 'legend.fontsize': 7, 'legend.frameon': False})
W1 = 3.45
def save(fig, name):
    fig.savefig(os.path.join(OUT, name + '.pdf')); fig.savefig(os.path.join(OUT, name + '.png'), dpi=200); plt.close(fig)

# ------------------------------------------------------------------ loading
def read_raw(path):
    O = {k: [] for k in ['t', 'x', 'y', 'qx', 'qy', 'qz', 'qw', 'err']}; M = {k: [] for k in ['t', 'x', 'y', 'z']}
    with open(path) as f:
        for r in csv.DictReader(f):
            if r['src'] == 'opt':
                if r['valid'] != '1' or r['x'] == '': continue
                e = float(r['aux2'] or 'nan')
                if not e <= MAX_MARKER_ERR: continue
                O['t'].append(float(r['t'])); O['x'].append(float(r['x'])); O['y'].append(float(r['y']))
                for k in ['qx', 'qy', 'qz', 'qw']: O[k].append(float(r[k]))
                O['err'].append(e)
            elif r['src'] == 'mm' and r['x'] != '' and r['valid'] == '1':
                M['t'].append(float(r['t'])); M['x'].append(float(r['x'])); M['y'].append(float(r['y'])); M['z'].append(float(r['z']))
    O = {k: np.array(v) for k, v in O.items()}; M = {k: np.array(v) for k, v in M.items()}
    O['yaw'] = np.unwrap(np.arctan2(2 * (O['qw'] * O['qz'] + O['qx'] * O['qy']), 1 - 2 * (O['qy'] ** 2 + O['qz'] ** 2)))
    return O, M

def read_corrected(path):
    M = {k: [] for k in ['t', 'x', 'y', 'z']}
    for r in csv.DictReader(open(path)):
        if r['valid'] != '1': continue
        for k in M: M[k].append(float(r[k]))
    return {k: np.array(v) for k, v in M.items()}

REC = {}
for n, lab in CAMPAIGN1.items():
    O, Mraw = read_raw(os.path.join(EX, 'KELO_6Tests_Unfiltered', 'data', n + '_raw.csv'))
    REC[n] = dict(label=lab, O=O, M=read_corrected(os.path.join(COR, n + '_corrected.csv')), Mraw=Mraw,
                  lever='early' if n in EARLY else 'late', session='campaign 1')
for n, (folder, lab) in CAMPAIGN2.items():
    O, M = read_raw(os.path.join(EX, folder, 'data', n + '_raw.csv'))
    REC[n] = dict(label=lab, O=O, M=M, Mraw=None, lever='late', session='campaign 2')

# ------------------------------------------------------------------ OptiTrack helpers
def ot_at(O, t, max_gap=0.05):
    """Pivot x, y, yaw interpolated at t, and a mask of samples with an OptiTrack frame within max_gap."""
    j = np.clip(np.searchsorted(O['t'], t), 1, len(O['t']) - 1)
    ok = np.minimum(np.abs(O['t'][j] - t), np.abs(O['t'][j - 1] - t)) <= max_gap
    return np.interp(t, O['t'], O['x']), np.interp(t, O['t'], O['y']), np.interp(t, O['t'], O['yaw']), ok

def speed_at(O, t):
    g = np.arange(O['t'][0], O['t'][-1], 0.1)
    x = np.interp(g, O['t'], O['x']); y = np.interp(g, O['t'], O['y'])
    v = np.hypot(np.gradient(x, 0.1), np.gradient(y, 0.1))
    v = np.convolve(v, np.ones(3) / 3, mode='same')
    return np.interp(t, g, v)

def stops(O, min_len=10.0, trim=2.0):
    g = np.arange(O['t'][0], O['t'][-1], 0.2)
    x = np.interp(g, O['t'], O['x']); y = np.interp(g, O['t'], O['y']); yaw = np.interp(g, O['t'], O['yaw'])
    j = np.clip(np.searchsorted(O['t'], g), 1, len(O['t']) - 1)
    gap = np.minimum(np.abs(O['t'][j] - g), np.abs(O['t'][j - 1] - g)) > 0.1
    v = np.hypot(np.gradient(x, 0.2), np.gradient(y, 0.2)); w = np.degrees(np.abs(np.gradient(yaw, 0.2)))
    still = (v < 0.02) & (w < 3) & ~gap
    out = []; i = 0
    while i < len(g):
        if still[i]:
            k = i
            while k + 1 < len(g) and still[k + 1]: k += 1
            a, b = g[i] + trim, g[k] - trim
            if b - a >= min_len: out.append((a, b))
            i = k + 1
        else: i += 1
    return out

# ------------------------------------------------------------------ alignment model
def rot(th): return np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
def hedge(px, py, yaw, lev, th, t):
    h = np.c_[px + np.cos(yaw) * lev[0] - np.sin(yaw) * lev[1], py + np.sin(yaw) * lev[0] + np.cos(yaw) * lev[1]]
    return h @ rot(th).T + t

def stop_table(n, use_raw=False):
    r = REC[n]; O = r['O']; M = r['Mraw'] if use_raw else r['M']; rows = []
    for a, b in stops(O):
        so = (O['t'] >= a) & (O['t'] <= b); sm = (M['t'] >= a) & (M['t'] <= b)
        if so.sum() < 50 or sm.sum() < 20: continue
        yaw = math.atan2(np.sin(O['yaw'][so]).mean(), np.cos(O['yaw'][so]).mean())
        rows.append(dict(rec=n, a=a, b=b, px=O['x'][so].mean(), py=O['y'][so].mean(), yaw=yaw,
                         mx=np.median(M['x'][sm]), my=np.median(M['y'][sm]), n=int(sm.sum()),
                         jit=float(np.hypot(M['x'][sm] - np.median(M['x'][sm]), M['y'][sm] - np.median(M['y'][sm])).std() * 100)))
    return rows

def fit(rows, th=None, t=None):
    P = np.array([[r['px'], r['py'], r['yaw']] for r in rows]); Mm = np.array([[r['mx'], r['my']] for r in rows])
    if th is None:
        best = None
        for th0 in np.radians(np.arange(-180, 180, 30)):
            t0 = (Mm - P[:, :2] @ rot(th0).T).mean(0)
            res = least_squares(lambda q: (hedge(P[:, 0], P[:, 1], P[:, 2], q[3:], q[0], q[1:3]) - Mm).ravel(),
                                np.r_[th0, t0, 0, 0], loss='soft_l1', f_scale=0.05)
            if best is None or res.cost < best.cost: best = res
        return best.x[0], best.x[1:3], best.x[3:]
    res = least_squares(lambda l: (hedge(P[:, 0], P[:, 1], P[:, 2], l, th, t) - Mm).ravel(), [0, 0], loss='soft_l1', f_scale=0.05)
    return th, t, res.x

ST = {n: stop_table(n) for n in REC}
TH, TT, LEV_LATE = fit(ST['deneme_2025'] + ST['deneme_2028'] + ST['deneme_triangle_2031'])
_, _, LEV_EARLY = fit(ST['test1'] + ST['test2'], TH, TT)
LEV = {'early': LEV_EARLY, 'late': LEV_LATE}

# Cross-campaign evaluation: every recording is evaluated with an alignment that did not use it.
# Campaign 1 recordings: alignment fitted on the campaign 2 stops (C1 rotations: lever arm of mounting 1
# fitted on the other rotation run). Campaign 2 recordings: alignment fitted on the campaign 1 stops
# with the same hedgehog mounting (out-and-back and rest, corrected data).
C2FIT = ['deneme_2025', 'deneme_2028', 'deneme_triangle_2031']
C1M2FIT = ['test5_in_out1', 'test6_wait']
TH_B, TT_B, LEV_B = fit(sum([ST[k] for k in C1M2FIT], []))
PARAMS = {}
for n in REC:
    if n in CAMPAIGN2:
        PARAMS[n] = (TH_B, TT_B, LEV_B)
    elif n in ('test1', 'test2'):
        PARAMS[n] = fit(ST['test2' if n == 'test1' else 'test1'], TH, TT)
    else:
        PARAMS[n] = (TH, TT, LEV[REC[n]['lever']])
NUM['out_of_fit_parameters'] = {n: {'rotation_deg': round(math.degrees(q[0]), 2), 'translation_m': [round(v, 3) for v in q[1]],
                                    'lever_arm_m': [round(v, 3) for v in q[2]]} for n, q in PARAMS.items() if n in list(CAMPAIGN2) + ['test1', 'test2']}
NUM['alignment'] = {'fitted_on': 'for campaign 1: campaign 2 stops (rotation x2, triangle), lever arm of mounting 1 on the C1 rotations (other run); for campaign 2: C1 out-and-back and rest stops',
                    'rotation_deg': round(math.degrees(TH), 2), 'translation_m': [round(v, 3) for v in TT],
                    'lever_arm_after_move_m': [round(v, 3) for v in LEV_LATE], 'lever_arm_before_move_m': [round(v, 3) for v in LEV_EARLY],
                    'hedgehog_moved_by_m': round(float(np.hypot(*(LEV_LATE - LEV_EARLY))), 3)}

# delay of Marvelmind with respect to OptiTrack, estimated on the campaign 2 triangle only
def errors(n, tau=0.0, use_raw=False, th=None, t=None, lev=None):
    r = REC[n]; M = r['Mraw'] if use_raw else r['M']; O = r['O']
    px, py, yaw, ok = ot_at(O, M['t'] - tau)
    th_, t_, l_ = PARAMS[n]
    H = hedge(px, py, yaw, l_ if lev is None else lev, th_ if th is None else th, t_ if t is None else t)
    e = np.hypot(M['x'] - H[:, 0], M['y'] - H[:, 1]) * 100
    return e, ok, speed_at(O, M['t'] - tau), H

taus = np.arange(0, 0.41, 0.01)
mov = lambda n, tau: (lambda e, ok, v, H: e[ok & (v > 0.05)])(*errors(n, tau))
rms = lambda a: float(np.sqrt(np.mean(np.square(a))))
def best_tau(n):
    return float(taus[int(np.argmin([rms((lambda e, ok, v, H: e[ok & (v > 0.05) & (e < FAILED_CM)])(*errors(n, tau))) for tau in taus]))])
tau_each = {n: best_tau(n) for n in ['deneme_triangle_2031', 'test2_triangle', 'test4_triangle', 'test5_in_out1']}
TAU = float(np.median(list(tau_each.values())))
NUM['delay_s'] = {'estimated_on': 'median of the best delay of the four recordings with motion', 'per_recording': tau_each, 'tau': TAU}

# ------------------------------------------------------------------ per-recording summary
summ = {}
for n, r in REC.items():
    e, ok, v, H = errors(n, TAU)
    rest = e[ok & (v < 0.02)]; moving = e[ok & (v > 0.05)]
    d = {'label': r['label'], 'session': r['session'], 'samples_with_reference': int(ok.sum()), 'failed_samples': int((ok & (e >= FAILED_CM)).sum())}
    if len(rest): d.update(rest_median=round(float(np.median(rest)), 1), rest_p95=round(float(np.percentile(rest, 95)), 1))
    if len(moving) > 20: d.update(moving_median=round(float(np.median(moving)), 1), moving_rms=round(rms(moving), 1), moving_p95=round(float(np.percentile(moving, 95)), 1))
    if r['Mraw'] is not None:
        # as recorded: give the wrong map its best case, a rigid alignment fitted on all campaign 1 stops of the same mounting
        grp = [k for k in CAMPAIGN1 if REC[k]['lever'] == r['lever']]
        th0, t0, l0 = fit(sum([stop_table(k, use_raw=True) for k in grp], []))
        e0, ok0, v0, _ = errors(n, TAU, use_raw=True, th=th0, t=t0, lev=l0)
        d.update(as_recorded_median=round(float(np.median(e0[ok0])), 1), as_recorded_p95=round(float(np.percentile(e0[ok0], 95)), 1))
        d['corrected_median'] = round(float(np.median(e[ok])), 1); d['corrected_p95'] = round(float(np.percentile(e[ok], 95)), 1)
        raw_all = len(r['Mraw']['t']); d['valid_after_correction_pct'] = round(100 * len(r['M']['t']) / raw_all, 1)
        r['fit_raw'] = (th0, t0, l0)
    summ[n] = d
NUM['per_recording'] = summ

# ------------------------------------------------------------------ coupled comparison by test type
def stop_offsets(n, use_raw=False):
    r = REC[n]; out = []
    if use_raw:
        th0, t0, l0 = r['fit_raw']
    for s in (stop_table(n, use_raw=True) if use_raw else ST[n]):
        H = hedge(np.array([s['px']]), np.array([s['py']]), np.array([s['yaw']]),
                  l0 if use_raw else PARAMS[n][2], th0 if use_raw else PARAMS[n][0], t0 if use_raw else PARAMS[n][1])[0]
        out.append(float(np.hypot(s['mx'] - H[0], s['my'] - H[1]) * 100))
    return np.array(out)
def pooled(names, use_raw=False, moving=False):
    out = []
    for n in names:
        r = REC[n]
        if use_raw:
            th0, t0, l0 = r['fit_raw']; e, ok, v, _ = errors(n, TAU, use_raw=True, th=th0, t=t0, lev=l0)
        else:
            e, ok, v, _ = errors(n, TAU)
        out.append(e[ok & ((v > 0.05) if moving else (v < 0.02))])
    return np.concatenate(out)
rng = np.random.default_rng(1)
def med_diff_ci(a, b, nb=4000):
    d = [np.median(rng.choice(a, len(a))) - np.median(rng.choice(b, len(b))) for _ in range(nb)]
    return float(np.median(a) - np.median(b)), [float(np.percentile(d, 2.5)), float(np.percentile(d, 97.5))]
coup = {}
for typ, (c1, c2) in TYPES.items():
    so1r = np.concatenate([stop_offsets(n, True) for n in c1]); so1 = np.concatenate([stop_offsets(n) for n in c1]); so2 = np.concatenate([stop_offsets(n) for n in c2])
    so1 = so1[so1 < 50]   # failed correction (reported separately)
    d = {'stops_c1': int(len(so1)), 'stops_c2': int(len(so2)),
         'stop_offset_median_cm': {'c1_as_recorded': round(float(np.median(so1r)), 1), 'c1_corrected': round(float(np.median(so1)), 1), 'c2': round(float(np.median(so2)), 1)},
         'stop_offset_p95_cm': {'c1_as_recorded': round(float(np.percentile(so1r, 95)), 1), 'c1_corrected': round(float(np.percentile(so1, 95)), 1), 'c2': round(float(np.percentile(so2, 95)), 1)}}
    if len(so1) > 1 and len(so2) > 1:
        md, ci = med_diff_ci(so1, so2); d['c1_corrected_minus_c2_median_cm'] = round(md, 1); d['ci95_cm'] = [round(x, 1) for x in ci]
    if typ == 'triangle':
        m1r, m1, m2 = pooled(c1, True, True), pooled(c1, moving=True), pooled(c2, moving=True)
        m1 = m1[m1 < FAILED_CM]
        d['moving_rms_cm'] = {'c1_as_recorded': round(rms(m1r), 1), 'c1_corrected': round(rms(m1), 1), 'c2': round(rms(m2), 1)}
        d['moving_p95_cm'] = {'c1_as_recorded': round(float(np.percentile(m1r, 95)), 1), 'c1_corrected': round(float(np.percentile(m1, 95)), 1), 'c2': round(float(np.percentile(m2, 95)), 1)}
    coup[typ] = d
NUM['coupled'] = coup

# ------------------------------------------------------------------ Fig. 1: setup (floor plan + rigid body)
fig = plt.figure(figsize=(W1 * 2.05, 3.0))
ax = fig.add_axes([0.06, 0.2, 0.41, 0.72])                       # (a) and (b): same size
for k, (x, y, z) in BEACONS.items():
    right = x > 5
    ax.plot(x, y, 's', ms=7, color='k'); ax.annotate(f'B{k}\n{z:.2f} m', (x, y), xytext=(-7 if right else 7, -3), textcoords='offset points', fontsize=7, va='top', ha='right' if right else 'left')
O = REC['test5_in_out1']['O']; Mi = REC['test5_in_out1']['M']
Hh = hedge(O['x'], O['y'], O['yaw'], LEV_LATE, TH, TT)
ax.plot(Mi['x'], Mi['y'], '.', ms=0.6, color='0.6', rasterized=True, label='Marvelmind, out-and-back test')
ax.plot(Hh[::12, 0], Hh[::12, 1], '.', ms=0.6, color='C0', rasterized=True, label='OptiTrack coverage')
H4 = hedge(REC['test4_triangle']['O']['x'], REC['test4_triangle']['O']['y'], REC['test4_triangle']['O']['yaw'], LEV_EARLY, TH, TT)
ax.plot(H4[::12, 0], H4[::12, 1], '.', ms=0.6, color='C1', rasterized=True, label='triangle')
ax.set_xlim(-1.0, 10.8); ax.set_ylim(-6.6, 1.0); ax.set_aspect('equal', adjustable='datalim')
ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
ax.legend(loc='lower right', ncol=1, markerscale=12, fontsize=6.5, framealpha=0.9, frameon=True, edgecolor='none')
ax.set_title('(a) floor plan, Marvelmind map frame', fontsize=8)
ax2 = fig.add_axes([0.55, 0.2, 0.41, 0.72], projection='3d')
# Motive exports the rigid body markers in its Y-up body frame: plan = (x, -z), height = y
mk = list(csv.DictReader(open(os.path.join(EX, 'KELO_6Tests_Unfiltered', 'settings', 'optitrack_rigid_body_markers.csv'))))
MP = np.array([[float(r['x']), -float(r['z']), float(r['y'])] for r in mk]) * 100
low = MP[:, 2] < -5                                   # the three markers at the height of the hedgehog
hz = float(MP[low, 2].mean())                         # hedgehog at the height of the low markers
zf = hz - 12
for P_, m_, lab_ in [(MP[~low], 'o', 'upper markers (2 x 5)'), (MP[low], 's', 'lower markers (3)')]:
    ax2.scatter(P_[:, 0], P_[:, 1], P_[:, 2], marker=m_, s=12, facecolors='none', edgecolors='k', linewidths=0.7, label=lab_, depthshade=False)
    for q in P_: ax2.plot([q[0], q[0]], [q[1], q[1]], [zf, q[2]], color='0.75', lw=0.4)
ax2.scatter([0], [0], [0], marker='+', s=40, color='k', label='rigid body pivot', depthshade=False)
for L_, m_, c_, lab_ in [(LEV_EARLY, 'v', 'C1', 'hedgehog, mounting 1'), (LEV_LATE, '^', 'C0', 'hedgehog, mounting 2')]:
    ax2.scatter([L_[0] * 100], [L_[1] * 100], [hz], marker=m_, s=30, color=c_, label=lab_, depthshade=False)
    ax2.plot([L_[0] * 100] * 2, [L_[1] * 100] * 2, [zf, hz], color=c_, lw=0.6, ls='--')
    ax2.text(L_[0] * 100, L_[1] * 100, hz + 3, 'M1' if m_ == 'v' else 'M2', color=c_, fontsize=7)
ax2.set_xlabel('x (cm)', labelpad=-6); ax2.set_ylabel('y (cm)', labelpad=-6); ax2.set_zlabel('z (cm)', labelpad=-7)
ax2.tick_params(labelsize=6, pad=-2)
ax2.set_zlim(zf, 10); ax2.set_box_aspect((1, 1, 0.55), zoom=1.25); ax2.view_init(elev=20, azim=60)
ax2.legend(loc='upper center', bbox_to_anchor=(0.5, -0.02), ncol=2, fontsize=6.5, handletextpad=0.3, columnspacing=0.8)
ax2.set_title('(b) markers and hedgehog, robot body frame', fontsize=8, y=1.0)
NUM['rigid_body'] = {'low_markers_height_cm': round(hz, 1), 'upper_markers_height_cm': round(float(MP[~low, 2].mean()), 1),
                     'mounting1_to_low_markers_cm': round(float(np.hypot(*(LEV_EARLY * 100 - MP[low, :2].mean(0)))), 1),
                     'mounting2_to_low_markers_cm': round(float(np.hypot(*(LEV_LATE * 100 - MP[low, :2].mean(0)))), 1)}
save(fig, 'fig1_setup')

# ------------------------------------------------------------------ floor plan (single column), names as in the photo
fig, ax = plt.subplots(figsize=(W1, 2.4))
for k, (x, y, z) in BEACONS.items():
    right = x > 5
    ax.plot(x, y, 's', ms=6, color='#c0392b')
    ax.annotate(f'B{k}  {z:.2f} m', (x, y), xytext=(-6 if right else 6, -2), textcoords='offset points', fontsize=6.5, va='top', ha='right' if right else 'left')
ax.plot(Mi['x'], Mi['y'], '.', ms=0.5, color='0.65', rasterized=True, label='Marvelmind, out-and-back')
Hall = np.vstack([hedge(REC[n]['O']['x'], REC[n]['O']['y'], REC[n]['O']['yaw'], PARAMS[n][2], TH, TT)[::20] for n in CAMPAIGN1])
ax.plot(Hall[:, 0], Hall[:, 1], '.', ms=0.5, color='#1f77b4', alpha=0.5, rasterized=True, label='OptiTrack coverage')
tri = np.array([VERTEX_NAMES[k] for k in 'ABCA'])
ax.plot(tri[:, 0], tri[:, 1], '-', color='#e67e22', lw=1.2, label='triangle')
for k, (x, y) in {**VERTEX_NAMES, **OUTSIDE_NAMES}.items():
    col = '#e67e22' if k in VERTEX_NAMES else '#8e44ad'
    ax.plot(x, y, 'o' if k in VERTEX_NAMES else 's', ms=4, color=col, mec='white', mew=0.5)
    off = {'A': (-11, -8), 'B': (-10, 3), 'C': (4, 4), 'D': (-11, 2), 'E': (5, 3)}[k]
    ax.annotate(k, (x, y), xytext=off, textcoords='offset points', fontsize=7.5, weight='bold', color=col)
ax.set_xlim(-1.0, 10.8); ax.set_ylim(-6.3, 1.0); ax.set_aspect('equal', adjustable='datalim')
ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
fig.subplots_adjust(bottom=0.28); h_, l_ = ax.get_legend_handles_labels()
fig.legend(h_, l_, loc='lower center', bbox_to_anchor=(0.5, 0.0), ncol=3, markerscale=10, fontsize=6.5)
save(fig, 'fig_floorplan')

# ------------------------------------------------------------------ Fig. 2: as recorded vs corrected (slow triangle)
n = 'test4_triangle'; r = REC[n]
fig, axs = plt.subplots(1, 2, figsize=(W1, 2.2), sharex=True, sharey=True)
_, _, _, H = errors(n, TAU)
th0, t0, l0 = r['fit_raw']
for ax, (M, title, th_, t_, l_) in zip(axs, [(r['Mraw'], '(a) as recorded', th0, t0, l0), (r['M'], '(b) corrected', TH, TT, LEV_EARLY)]):
    px, py, yaw, ok = ot_at(r['O'], M['t'] - TAU)
    Hx = hedge(px, py, yaw, l_, th_, t_)
    # express everything in the corrected map frame for the plot: map the raw frame to it with the two alignments
    if title.startswith('(a)'):
        Mxy = (np.c_[M['x'], M['y']] - t_) @ rot(th_) @ rot(TH).T + TT
    else:
        Mxy = np.c_[M['x'], M['y']]
    Hc = hedge(px, py, yaw, LEV_EARLY, TH, TT)
    ax.plot(Hc[ok, 0], Hc[ok, 1], '-', lw=0.4, color='C0', label='OptiTrack')
    ax.plot(Mxy[:, 0], Mxy[:, 1], '.', ms=0.8, color='k', rasterized=True, label='Marvelmind')
    ax.set_title(title, fontsize=8); ax.set_aspect('equal'); ax.set_xlabel('x (m)')
axs[0].set_ylabel('y (m)'); axs[0].set_xlim(0.5, 3.5); axs[0].set_ylim(-4.2, -1.0)
fig.subplots_adjust(bottom=0.27); h_, l_ = axs[1].get_legend_handles_labels(); fig.legend(h_, l_, loc='lower center', bbox_to_anchor=(0.5, 0.0), ncol=2, markerscale=6)
save(fig, 'fig2_correction')

# ------------------------------------------------------------------ Fig. 3: CDF per test type (C1 as recorded, C1 corrected, C2)
def zf_err(n):
    r = REC[n]; M = r['Mraw']; th0, t0, l0 = r['fit_raw']; keep = (M['z'] > -0.4) & (M['z'] < 0.4)
    px, py, yaw, ok = ot_at(r['O'], M['t'] - TAU); H = hedge(px, py, yaw, l0, th0, t0)
    return (np.hypot(M['x'] - H[:, 0], M['y'] - H[:, 1]) * 100)[ok & keep]
fig, axs = plt.subplots(1, 3, figsize=(W1, 2.1), sharey=True)
for ax, (typ, (c1, c2)) in zip(axs, TYPES.items()):
    for data, ls, col, lab in [(np.concatenate([pooled(c1, True), pooled(c1, True, True)]), ':', '0.45', 'C1 as recorded'),
                               (np.concatenate([pooled(c1), pooled(c1, moving=True)]), '-', 'k', 'C1 corrected'),
                               (np.concatenate([pooled(c2), pooled(c2, moving=True)]), '--', 'C3', 'C2 reference'),
                               (np.concatenate([zf_err(n) for n in c1]), '-.', 'C0', 'C1 z filter only')]:
        e = np.sort(data); ax.plot(e, np.arange(1, len(e) + 1) / len(e), ls, color=col, label=lab)
    ax.set_title(typ, fontsize=7.5); ax.set_xlim(0, 60 if typ != 'rest' else 40); ax.set_xlabel('error (cm)')
axs[0].set_ylabel('CDF'); fig.subplots_adjust(bottom=0.36); h_, l_ = axs[1].get_legend_handles_labels(); fig.legend(h_, l_, loc='lower center', bbox_to_anchor=(0.5, 0.0), ncol=2, fontsize=6.5)
save(fig, 'fig3_cdf')

# ------------------------------------------------------------------ Fig. 4: at rest (vertices and heading)
fig, axs = plt.subplots(1, 2, figsize=(W1, 2.0), gridspec_kw={'wspace': 0.45})
rows = []
for n in ['test2_triangle', 'test4_triangle', 'deneme_triangle_2031']:
    for s_ in ST[n]:
        Hs = hedge(np.array([s_['px']]), np.array([s_['py']]), np.array([s_['yaw']]), PARAMS[n][2], PARAMS[n][0], PARAMS[n][1])[0]
        rows.append((n, np.array([s_['px'], s_['py']]), (s_['mx'] - Hs[0]) * 100, (s_['my'] - Hs[1]) * 100, s_['jit'], math.degrees(s_['yaw'])))
NUM['failed_stops'] = [{'rec': r_[0], 'offset_cm': round(float(np.hypot(r_[2], r_[3])), 1)} for r_ in rows if np.hypot(r_[2], r_[3]) > 50]
rows = [r_ for r_ in rows if np.hypot(r_[2], r_[3]) <= 50]
cent = []
for row in rows:                      # vertices = same robot pose (pivot), common to both campaigns
    for c in cent:
        if np.hypot(*(row[1] - c[0][1])) < 0.45: c.append(row); break
    else: cent.append([row])
vert = []
VCOL = {'A': 'C0', 'B': 'C1', 'C': 'C2'}; VMK = {'A': 'o', 'B': 's', 'C': 'd'}
for k, c in enumerate(sorted([c for c in cent if len(c) >= 3], key=lambda c: c[0][1][0])):
    name = nearest_name(np.mean([x[1] for x in c], 0) @ rot(TH).T + TT, VERTEX_NAMES)
    for camp, filled in [('C1', False), ('C2', True)]:
        d = np.array([[x[2], x[3]] for x in c if (x[0] == 'deneme_triangle_2031') == filled])
        if not len(d): continue
        axs[0].plot(d[:, 0], d[:, 1], VMK[name], ms=3.2, mfc=(VCOL[name] if filled else 'none'), mew=0.7, color=VCOL[name])
        vert.append({'vertex': name, 'campaign': camp, 'stops': len(d), 'mean_offset_cm': round(float(np.hypot(*d.mean(0))), 1),
                     'mean_cm': [round(float(v), 1) for v in d.mean(0)],
                     'std_cm': [round(float(v), 1) for v in d.std(0, ddof=1)] if len(d) > 1 else None})
NUM['vertices'] = vert
for name in 'ABC': axs[0].plot([], [], VMK[name], color=VCOL[name], ms=3.2, label=name)
axs[0].plot([], [], 'o', color='0.3', mfc='none', ms=3.2, label='C1 corr.'); axs[0].plot([], [], 'o', color='0.3', ms=3.2, label='C2')
axs[0].legend(loc='upper center', bbox_to_anchor=(0.5, -0.32), ncol=3, fontsize=6.5, handletextpad=0.2, columnspacing=0.6)
axs[0].axhline(0, color='k', lw=0.4); axs[0].axvline(0, color='k', lw=0.4); axs[0].set_aspect('equal', adjustable='datalim')
axs[0].set_xlabel(r'$\Delta x$ (cm)'); axs[0].set_ylabel(r'$\Delta y$ (cm)'); axs[0].set_title('(a) triangle vertices', fontsize=8)
hd = []
for n, m_, c in [('test1', 'o', 'C0'), ('test2', 's', 'C1'), ('deneme_2025', '^', 'C2'), ('deneme_2028', 'v', 'C3')]:
    for s in ST[n]:
        Hs = hedge(np.array([s['px']]), np.array([s['py']]), np.array([s['yaw']]), PARAMS[n][2], PARAMS[n][0], PARAMS[n][1])[0]
        off = np.hypot(s['mx'] - Hs[0], s['my'] - Hs[1]) * 100
        axs[1].plot(math.degrees(s['yaw']), off, m_, color=c, ms=4, mfc='none', mew=0.8)
        hd.append({'rec': n, 'heading_deg': round(math.degrees(s['yaw']), 1), 'offset_cm': round(float(off), 1)})
for lab, m_, c in [('C1 run 1', 'o', 'C0'), ('C1 run 2', 's', 'C1'), ('C2 run 1', '^', 'C2'), ('C2 run 2', 'v', 'C3')]:
    axs[1].plot([], [], m_, color=c, mfc='none', label=lab)
axs[1].set_xlim(-180, 180); axs[1].set_xticks([-180, -90, 0, 90, 180]); axs[1].set_ylim(bottom=0)
axs[1].set_xlabel('heading (deg)'); axs[1].set_ylabel('offset (cm)'); axs[1].set_title('(b) rotation in place', fontsize=8)
axs[1].legend(loc='upper center', bbox_to_anchor=(0.5, -0.32), ncol=2, fontsize=6.5)
NUM['rotation_in_place'] = hd
save(fig, 'fig4_rest')

# ------------------------------------------------------------------ Fig. 5: error vs speed
edges = np.array([0, 0.02, 0.1, 0.2, 0.3, 0.4, 0.5, 0.7])
fig, ax = plt.subplots(figsize=(W1, 1.9)); NUM['error_vs_speed'] = {'edges_m_s': edges.tolist()}
for i, n in enumerate(['test4_triangle', 'test2_triangle', 'deneme_triangle_2031', 'test5_in_out1']):
    e, ok, v, _ = errors(n, TAU); xs, ys, ns = [], [], []
    for k in range(len(edges) - 1):
        sel = ok & (v >= edges[k]) & (v < edges[k + 1]) & (e < FAILED_CM)
        if sel.sum() < 30: continue
        xs.append((edges[k] + edges[k + 1]) / 2); ys.append(rms(e[sel])); ns.append(int(sel.sum()))
    ax.plot(xs, ys, 'o-s^'[i * 1 if i else 0] if False else ['o-', 's--', 'D-.', '^:'][i], ms=3, color=f'C{i}', label=REC[n]['label'])
    NUM['error_vs_speed'][n] = {'rms_cm': [round(v_, 1) for v_ in ys], 'n': ns, 'centres': xs}
ax.set_xlabel('speed from OptiTrack (m/s)'); ax.set_ylabel('RMS error (cm)'); ax.set_ylim(0, 18); ax.legend(loc='lower right', ncol=2, fontsize=6.5)
save(fig, 'fig5_speed')

# ------------------------------------------------------------------ Fig. 6: Allan deviation at rest
def allan2d(t, x, y, taus):
    dt = np.median(np.diff(t)); g = np.arange(t[0], t[-1], dt)
    X = np.interp(g, t, x); Y = np.interp(g, t, y); out = []
    for tau in taus:
        m = int(round(tau / dt))
        if m < 1 or 3 * m > len(g): out.append(np.nan); continue
        k = len(g) // m
        ax_ = X[:k * m].reshape(k, m).mean(1); ay_ = Y[:k * m].reshape(k, m).mean(1)
        out.append(math.sqrt(0.5 * np.mean(np.diff(ax_) ** 2 + np.diff(ay_) ** 2)))
    return np.array(out)
tl = np.logspace(np.log10(0.2), np.log10(300), 25)
fig, ax = plt.subplots(figsize=(W1, 2.3)); NUM['rest'] = {}
for i, n in enumerate(['test6_wait', 'denemenew_wait_2040']):
    r = REC[n]; M = r['M']; (a, b) = max(stops(r['O']), key=lambda s: s[1] - s[0])
    sm = (M['t'] >= a) & (M['t'] <= b)
    ad = allan2d(M['t'][sm], M['x'][sm], M['y'][sm], tl) * 1000
    ax.loglog(tl, ad, ['o-', 's--'][i], ms=2.5, color=f'C{i}', label=f"Marvelmind, {r['label']}")
    e, ok, v, _ = errors(n, TAU); off = float(np.median(e[ok & (M['t'] >= a) & (M['t'] <= b)]))
    ax.axhline(off * 10, color=f'C{i}', lw=0.7, ls=':')
    O = r['O']; so = (O['t'] >= a) & (O['t'] <= b)
    ado = allan2d(O['t'][so][::12], O['x'][so][::12], O['y'][so][::12], tl) * 1000
    if i == 0: ax.loglog(tl, ado, '-', color='0.5', lw=0.8, label='OptiTrack')
    NUM['rest'][n] = {'duration_s': round(b - a, 1), 'jitter_std_mm': round(float(np.hypot(M['x'][sm].std(), M['y'][sm].std()) * 1000), 2),
                      'allan_mm_at_1s_10s_60s': [round(float(np.interp(np.log(tt), np.log(tl), ad)), 2) for tt in (1, 10, 60)],
                      'offset_to_reference_cm': round(off, 1)}
ax.text(0.25, NUM['rest']['test6_wait']['offset_to_reference_cm'] * 10 * 1.15, 'offset to OptiTrack (dotted)', fontsize=6.5)
ax.set_xlabel(r'averaging time $\tau$ (s)'); ax.set_ylabel('Allan deviation, 2D (mm)')
fig.subplots_adjust(bottom=0.3); h_, l_ = ax.get_legend_handles_labels(); fig.legend(h_, l_, loc='lower center', bbox_to_anchor=(0.5, 0.0), ncol=2, fontsize=6.5)
save(fig, 'fig6_allan')

# ------------------------------------------------------------------ Fig. 7: in/out
n = 'test5_in_out1'; r = REC[n]; M = r['M']; O = r['O']
e, ok, v, H = errors(n, TAU)
gaps = []; dt = np.diff(O['t']); idx = np.where(dt > 5)[0]
for k in idx: gaps.append((O['t'][k], O['t'][k + 1]))
# stationary Marvelmind periods outside the OptiTrack area
outs = []
for a, b in gaps:
    sel = np.where((M['t'] > a + 1) & (M['t'] < b - 1))[0]; i = 0
    while i < len(sel):
        k = i
        while k + 1 < len(sel) and np.hypot(M['x'][sel[k + 1]] - np.median(M['x'][sel[i:k + 2]]), M['y'][sel[k + 1]] - np.median(M['y'][sel[i:k + 2]])) < 0.03:
            k += 1
        if M['t'][sel[k]] - M['t'][sel[i]] >= 8: outs.append((np.median(M['x'][sel[i:k + 1]]), np.median(M['y'][sel[i:k + 1]]), M['t'][sel[i]]))
        i = k + 1
clusters = []
for o in outs:
    for c in clusters:
        if np.hypot(o[0] - c[0][0], o[1] - c[0][1]) < 0.4: c.append(o); break
    else: clusters.append([o])
clusters = sorted([c for c in clusters if len(c) >= 3], key=len, reverse=True)
reent = []
for a, b in gaps:
    pre = e[ok & (M['t'] > a - 8) & (M['t'] < a)]; post = e[ok & (M['t'] > b) & (M['t'] < b + 8)]
    if len(pre) > 5 and len(post) > 5: reent.append((float(np.median(pre)), float(np.median(post))))
fig, axs = plt.subplots(2, 1, figsize=(W1, 3.4), gridspec_kw={'hspace': 0.6})
for a, b in gaps: axs[0].axvspan(a, b, color='0.88', lw=0)
axs[0].plot(M['t'][ok], e[ok], '.', ms=0.8, color='k', rasterized=True)
axs[0].set_xlim(0, M['t'][-1]); axs[0].set_ylim(0, 40); axs[0].set_xlabel('time (s)'); axs[0].set_ylabel('error (cm)')
axs[0].set_title('(a) error; grey: outside the OptiTrack area', fontsize=8)
axs[1].plot(M['x'], M['y'], '.', ms=0.6, color='0.65', rasterized=True, label='Marvelmind')
axs[1].plot(H[ok, 0], H[ok, 1], '.', ms=0.6, color='C0', rasterized=True, label='OptiTrack')
NUM['in_out'] = {'reentries': len(reent), 'error_before_exit_median_cm': round(float(np.median([p for p, q in reent])), 1) if reent else None,
                 'error_after_reentry_median_cm': round(float(np.median([q for p, q in reent])), 1) if reent else None, 'outside_points': []}
for k, c in enumerate(clusters[:2]):
    P = np.array([[o[0], o[1]] for o in c]); sd = P.std(0, ddof=1) * 100
    name = nearest_name(P.mean(0), OUTSIDE_NAMES)
    axs[1].plot(P[:, 0], P[:, 1], 'x', ms=3, color=f'C{k + 3}', label=f"outside point {name} ({len(P)} stops)")
    NUM['in_out']['outside_points'].append({'point': name, 'stops': len(P), 'std_cm': [round(float(s_), 1) for s_ in sd],
                                            'max_from_mean_cm': round(float(np.hypot(*(P - P.mean(0)).T).max() * 100), 1)})
axs[1].set_aspect('equal', adjustable='datalim'); axs[1].set_xlabel('x (m)'); axs[1].set_ylabel('y (m)')
axs[1].legend(loc='upper center', bbox_to_anchor=(0.5, -0.3), ncol=2, markerscale=3, fontsize=6.5)
axs[1].set_title('(b) path', fontsize=8)
save(fig, 'fig7_inout')

# ================================================================== additional analyses
C1_ALL = list(CAMPAIGN1)

# ---------- (A) uncertainty budget of the reference position
rngb = np.random.default_rng(7)
def boot_params(nb=300):
    """Bootstrap over stops of the two registrations. A (for campaign 1): C2 stops -> rotation,
    translation, lever arm 2; C1 rotation stops -> lever arm 1. B (for campaign 2): C1 mounting-2 stops."""
    c2 = sum([ST[k] for k in C2FIT], []); c1r = ST['test1'] + ST['test2']; c1m2 = sum([ST[k] for k in C1M2FIT], []); A, Bb = [], []
    for _ in range(nb):
        th_, t_, l2 = fit([c2[i] for i in rngb.integers(0, len(c2), len(c2))])
        _, _, l1 = fit([c1r[i] for i in rngb.integers(0, len(c1r), len(c1r))], th_, t_)
        A.append((th_, t_, l1, l2))
        Bb.append(fit([c1m2[i] for i in rngb.integers(0, len(c1m2), len(c1m2))]))
    return A, Bb
BPA, BPB = boot_params()
def ref_spread(rows, BP, lever_idx):
    """2D standard uncertainty of the reference hedgehog position at the given stops, over the bootstrap."""
    P = np.array([[r['px'], r['py'], r['yaw']] for r in rows])
    H = np.array([hedge(P[:, 0], P[:, 1], P[:, 2], b[lever_idx], b[0], b[1]) for b in BP])   # nb x n x 2
    return float(np.sqrt(np.mean(np.sum(H.var(0), axis=1)))) * 100
u_align_m1 = ref_spread(ST['test2_triangle'] + ST['test4_triangle'], BPA, 2)
u_align_m2 = ref_spread(ST['test5_in_out1'] + ST['test6_wait'], BPA, 3)
u_align_c2 = ref_spread(sum([ST[k] for k in CAMPAIGN2], []), BPB, 2)
lev1 = np.array([b[2] for b in BPA]) * 100; lev2 = np.array([b[3] for b in BPA]) * 100
# OptiTrack: mean marker error and static noise during the long rests
ot_err = float(np.mean([np.mean(REC[n]['O']['err']) for n in REC])) * 1000
ot_static = []
for n in ['test6_wait', 'denemenew_wait_2040']:
    O = REC[n]['O']; (a, b) = max(stops(O), key=lambda s_: s_[1] - s_[0]); so = (O['t'] >= a) & (O['t'] <= b)
    ot_static.append(float(np.hypot(O['x'][so].std(), O['y'][so].std())) * 1000)
# delay: standard uncertainty of the median of the four per-recording estimates
u_tau = float(np.std(list(tau_each.values()), ddof=1) / math.sqrt(len(tau_each)))
V_REF = 0.3   # m/s, typical speed
u_delay = u_tau * V_REF * 100
# synchronisation: Marvelmind transport jitter (device time stamp vs PC arrival) from the recording summaries
u_sync = 0.005 * V_REF * 100
# temperature (affects Marvelmind, not the reference): +-3 C rectangular around the 23 C setting
import itertools as _it
Bc = {k: np.array(v) for k, v in BEACONS.items()}
def trilat3(B, d):
    p1, p2, p3 = B; ex = (p2 - p1) / np.linalg.norm(p2 - p1); i_ = ex @ (p3 - p1)
    ey = p3 - p1 - i_ * ex; ey /= np.linalg.norm(ey); ez = np.cross(ex, ey); D = np.linalg.norm(p2 - p1); j_ = ey @ (p3 - p1)
    x = (d[0] ** 2 - d[1] ** 2 + D ** 2) / (2 * D); y = (d[0] ** 2 - d[2] ** 2 + i_ ** 2 + j_ ** 2) / (2 * j_) - i_ * x / j_
    z = np.sqrt(max(d[0] ** 2 - x ** 2 - y ** 2, 0)); a_, b_ = p1 + x * ex + y * ey + z * ez, p1 + x * ex + y * ey - z * ez
    return a_ if a_[2] < b_[2] else b_
uT = 3 / math.sqrt(3); eps = 0.0017 * uT
shifts = []
for px_ in np.linspace(0.8, 3.0, 5):
    for py_ in np.linspace(-3.8, -1.2, 5):
        p0 = np.array([px_, py_, 0.70])
        for tri in _it.combinations([1, 2, 4, 5], 3):
            d = [np.linalg.norm(p0 - Bc[k]) * (1 + eps) for k in tri]
            shifts.append(np.hypot(*(trilat3([Bc[k] for k in tri], d)[:2] - p0[:2])) * 100)
u_temp = float(np.sqrt(np.mean(np.square(shifts))))
u_ref_rest = math.sqrt((ot_err / 10) ** 2 + max(u_align_m1, u_align_m2, u_align_c2) ** 2)
u_ref_moving = math.sqrt(u_ref_rest ** 2 + u_delay ** 2 + u_sync ** 2)
NUM['uncertainty'] = {
    'optitrack_mean_marker_error_mm': round(ot_err, 2), 'optitrack_static_std_mm': [round(v, 3) for v in ot_static],
    'alignment_and_lever_arm_cm': {'C1_mounting_1': round(u_align_m1, 2), 'C1_mounting_2': round(u_align_m2, 2), 'C2': round(u_align_c2, 2)},
    'lever_arm_std_cm': {'mounting_1': [round(float(v), 2) for v in lev1.std(0)], 'mounting_2': [round(float(v), 2) for v in lev2.std(0)]},
    'delay_s': TAU, 'delay_per_recording_s': tau_each, 'u_delay_s': round(u_tau, 3), 'u_delay_at_0.3_m_s_cm': round(u_delay, 2),
    'u_sync_at_0.3_m_s_cm': round(u_sync, 2), 'temperature_effect_on_marvelmind_cm': round(u_temp, 2),
    'u_reference_rest_cm': round(u_ref_rest, 2), 'U_reference_rest_k2_cm': round(2 * u_ref_rest, 2),
    'u_reference_moving_cm': round(u_ref_moving, 2), 'U_reference_moving_k2_cm': round(2 * u_ref_moving, 2),
    'lever_arm_tape_check_cm': {'mounting_1_to_low_markers_data_vs_tape': [NUM['rigid_body']['mounting1_to_low_markers_cm'], 40],
                                'move_data_vs_tape': [round(float(np.hypot(*(LEV_LATE - LEV_EARLY))) * 100, 1), 45]}}

# ---------- (B) z filter only vs correction (campaign 1)
def zfilter_errors(n, moving=None):
    r = REC[n]; M = r['Mraw']; th0, t0, l0 = r['fit_raw']
    keep = (M['z'] > -0.4) & (M['z'] < 0.4)
    px, py, yaw, ok = ot_at(r['O'], M['t'] - TAU); H = hedge(px, py, yaw, l0, th0, t0)
    e = np.hypot(M['x'] - H[:, 0], M['y'] - H[:, 1]) * 100; v = speed_at(r['O'], M['t'] - TAU)
    sel = ok & keep if moving is None else ok & keep & ((v > 0.05) if moving else (v < 0.02))
    return e[sel], float(keep.mean() * 100)
zc = {}
for typ, (c1, _) in TYPES.items():
    ez = np.concatenate([zfilter_errors(n)[0] for n in c1]); kz = float(np.mean([zfilter_errors(n)[1] for n in c1]))
    ec = np.concatenate([errors(n, TAU)[0][errors(n, TAU)[1]] for n in c1])
    kc = float(np.mean([100 * len(REC[n]['M']['t']) / len(REC[n]['Mraw']['t']) for n in c1]))
    zc[typ] = {'z_filter': {'kept_pct': round(kz, 1), 'median_cm': round(float(np.median(ez)), 1), 'p95_cm': round(float(np.percentile(ez, 95)), 1)},
               'correction': {'kept_pct': round(kc, 1), 'median_cm': round(float(np.median(ec)), 1), 'p95_cm': round(float(np.percentile(ec, 95)), 1)}}
    if typ == 'triangle':
        mz = np.concatenate([zfilter_errors(n, True)[0] for n in c1]); zc[typ]['z_filter']['moving_rms_cm'] = round(rms(mz), 1)
        zc[typ]['correction']['moving_rms_cm'] = NUM['coupled']['triangle']['moving_rms_cm']['c1_corrected']
ez = zfilter_errors('test5_in_out1')
zc['out-and-back'] = {'z_filter': {'kept_pct': round(ez[1], 1), 'median_cm': round(float(np.median(ez[0])), 1), 'p95_cm': round(float(np.percentile(ez[0], 95)), 1)},
                      'correction': {'kept_pct': NUM['per_recording']['test5_in_out1']['valid_after_correction_pct'],
                                     'median_cm': NUM['per_recording']['test5_in_out1']['corrected_median'], 'p95_cm': NUM['per_recording']['test5_in_out1']['corrected_p95']}}
NUM['zfilter_vs_correction'] = zc

# ---------- (C) beacon triple used by the correction vs robot heading (campaign 1)
inv = lambda xy: (np.asarray(xy) - TT) @ rot(TH)            # Marvelmind map -> OptiTrack frame
B_ot = {k: inv(np.array(v[:2])) for k, v in BEACONS.items()}
trip_rows = []
for n in C1_ALL:
    rows = [r_ for r_ in csv.DictReader(open(os.path.join(COR, n + '_corrected.csv'))) if r_['valid'] == '1']
    t = np.array([float(r_['t']) for r_ in rows]); px, py, yaw, ok = ot_at(REC[n]['O'], t)
    for r_, a_, b_, y_, o_ in zip(rows, px, py, yaw, ok):
        if not o_: continue
        used = set(int(k) for k in r_['triple'].split('-')); excl = ({1, 2, 4, 5} - used).pop()
        hb = np.array([a_, b_]); bear = {k: (math.degrees(math.atan2(*(B_ot[k] - hb)[::-1]) - y_) + 180) % 360 - 180 for k in B_ot}
        trip_rows.append((n, math.degrees(y_), excl, bear[excl], bear))
hd = np.array([((r_[1] + 180) % 360) - 180 for r_ in trip_rows]); ex_ = np.array([r_[2] for r_ in trip_rows])
bins = [(-180, -90), (-90, 0), (0, 90), (90, 180)]
tab = {}
for lo, hi in bins:
    sel = (hd >= lo) & (hd < hi)
    if sel.sum() < 50: continue
    tab[f'{lo}..{hi} deg'] = {'samples': int(sel.sum()), 'excluded_beacon_share_pct': {f'B{k}': round(float(np.mean(ex_[sel] == k) * 100), 1) for k in [1, 2, 4, 5]}}
rel_ex = np.array([r_[3] for r_ in trip_rows]); rel_all = np.array([v_ for r_ in trip_rows for v_ in r_[4].values()])
hist_ex = np.histogram(rel_ex, bins=8, range=(-180, 180))[0]; hist_all = np.histogram(rel_all, bins=8, range=(-180, 180))[0]
NUM['triple_vs_heading'] = {'note': 'triple inferred by the correction (closest hedgehog height), campaign 1, samples with reference',
                            'by_heading': tab,
                            'excluded_beacon_relative_bearing_hist': {'bin_edges_deg': list(range(-180, 181, 45)),
                                                                      'excluded_pct': [round(float(v) * 100 / hist_ex.sum(), 1) for v in hist_ex],
                                                                      'all_beacons_pct': [round(float(v) * 100 / hist_all.sum(), 1) for v in hist_all]}}

# ---------- (D) sensitivity of the correction to its settings
import importlib.util as _ilu
_spec = _ilu.spec_from_file_location('b1c', os.path.join(ROOT, 'paper', 'scripts', 'b1_correction.py')); B1 = _ilu.module_from_spec(_spec); _spec.loader.exec_module(B1)
def run_correction(n, hz, ztol, jw):
    B1.HEDGE_Z, B1.JUMP_W = hz, jw
    M = REC[n]['Mraw']; P = np.c_[M['x'], M['y'], M['z']]
    X, s_, zd = B1.correct(P); keep = zd < ztol
    px, py, yaw, ok = ot_at(REC[n]['O'], M['t'] - TAU); th_, t_, l_ = PARAMS[n]; H = hedge(px, py, yaw, l_, th_, t_)
    e = np.hypot(X[:, 0] - H[:, 0], X[:, 1] - H[:, 1]) * 100
    return float(keep.mean() * 100), e[ok & keep]
SENS_RECS = ['test2_triangle', 'test4_triangle', 'test5_in_out1']
grid = [('nominal', 0.70, 0.10, 2.0), ('hedgehog height 0.65 m', 0.65, 0.10, 2.0), ('hedgehog height 0.75 m', 0.75, 0.10, 2.0),
        ('tolerance 0.07 m', 0.70, 0.07, 2.0), ('tolerance 0.15 m', 0.70, 0.15, 2.0), ('no continuity term', 0.70, 0.10, 0.0),
        ('continuity weight 5', 0.70, 0.10, 5.0)]
sens = {}
for lab_, hz_, zt_, jw_ in grid:
    res = [run_correction(n, hz_, zt_, jw_) for n in SENS_RECS]
    e_all = np.concatenate([r_[1] for r_ in res])
    sens[lab_] = {'kept_pct': round(float(np.mean([r_[0] for r_ in res])), 1), 'median_cm': round(float(np.median(e_all)), 1),
                  'p95_cm': round(float(np.percentile(e_all, 95)), 1), 'share_above_50cm_pct': round(float(np.mean(e_all > 50) * 100), 2)}
B1.HEDGE_Z, B1.JUMP_W = 0.70, 2.0
NUM['correction_sensitivity'] = {'recordings': SENS_RECS, 'results': sens}

# ================================================================== Monte Carlo
# ---------- (E) GUM Supplement 1 style propagation (JCGM 101): registration, delay and OptiTrack noise
# are drawn together with a resampling of the stops; every draw recomputes the metrics of Table I.
def metrics_with(parA, parB, tau, rs):
    def P_(n):
        if n in CAMPAIGN2: return parB[0], parB[1], parB[2]
        if n in ('test1', 'test2'): return parA[0], parA[1], parA[2]
        return parA[0], parA[1], parA[2] if REC[n]['lever'] == 'early' else parA[3]
    def stop_off(n):
        th_, t_, l_ = P_(n); rows = ST[n]
        if not rows: return np.array([])
        P = np.array([[r['px'], r['py'], r['yaw']] for r in rows]) + rs.normal(0, 0.0011, (len(rows), 3)) * [1, 1, 0]
        H = hedge(P[:, 0], P[:, 1], P[:, 2], l_, th_, t_)
        return np.hypot(np.array([r['mx'] for r in rows]) - H[:, 0], np.array([r['my'] for r in rows]) - H[:, 1]) * 100
    def mov_err(n):
        th_, t_, l_ = P_(n); r = REC[n]; M = r['M']
        px, py, yaw, ok = ot_at(r['O'], M['t'] - tau); v = speed_at(r['O'], M['t'] - tau)
        H = hedge(px, py, yaw, l_, th_, t_); e = np.hypot(M['x'] - H[:, 0], M['y'] - H[:, 1]) * 100
        return e[ok & (v > 0.05) & (e < FAILED_CM)]
    out = {}
    for typ, (c1, c2) in TYPES.items():
        a = np.concatenate([stop_off(n) for n in c1]); a = a[a < 50]; b = np.concatenate([stop_off(n) for n in c2])
        a = rs.choice(a, len(a)); b = rs.choice(b, len(b))          # resampling of the stops
        out[typ + ' c1'] = np.median(a); out[typ + ' c2'] = np.median(b); out[typ + ' diff'] = np.median(a) - np.median(b)
    out['triangle moving rms c1'] = rms(np.concatenate([mov_err(n) for n in TYPES['triangle'][0]]))
    out['triangle moving rms c2'] = rms(np.concatenate([mov_err(n) for n in TYPES['triangle'][1]]))
    out['triangle moving rms diff'] = out['triangle moving rms c1'] - out['triangle moving rms c2']
    return out
rsm = np.random.default_rng(11)
draws = []
for i in range(len(BPA)):
    tau_i = max(0.0, rsm.normal(TAU, u_tau))
    draws.append(metrics_with(BPA[i], BPB[i], tau_i, rsm))
keys = draws[0].keys()
NUM['montecarlo_uncertainty'] = {'draws': len(draws), 'method': 'JCGM 101 style: registration from the stop bootstrap, delay N(0.105, 0.036) s, OptiTrack noise 1.1 mm, stops resampled',
                                 'metrics_cm': {k: {'mean': round(float(np.mean([d[k] for d in draws])), 2),
                                                    'ci95': [round(float(np.percentile([d[k] for d in draws], 2.5)), 2), round(float(np.percentile([d[k] for d in draws], 97.5)), 2)]} for k in keys}}

# ---------- (F) Monte Carlo of beacon height errors: effect, detectability and correction
TRUE = {k: np.array(v) for k, v in B1.RIGHT.items()}
IDS = [1, 2, 4, 5]
D3 = {(a, b): float(np.linalg.norm(TRUE[a] - TRUE[b])) for a, b in _it.combinations(IDS, 2)}
def wrong_map(k, dh):
    """Map that the Marvelmind software would build from the true beacon distances with beacon k at
    height h_k + dh (horizontal positions re-solved, as happened in campaign 1)."""
    H = {i: TRUE[i][2] + (dh if i == k else 0.0) for i in IDS}
    hor = {ab: math.sqrt(max(d ** 2 - (H[ab[0]] - H[ab[1]]) ** 2, 0.0)) for ab, d in D3.items()}
    x0 = np.concatenate([TRUE[i][:2] for i in IDS])
    res = least_squares(lambda x: [np.hypot(*(x[2 * IDS.index(a):2 * IDS.index(a) + 2] - x[2 * IDS.index(b):2 * IDS.index(b) + 2])) - hv
                                   for (a, b), hv in hor.items()] + list(0.01 * (x - x0)), x0)
    return {i: np.r_[res.x[2 * j:2 * j + 2], H[i]] for j, i in enumerate(IDS)}
def tri_solve(Bmap, tri, d):
    return trilat3([Bmap[k] for k in tri], d)
def rigid_err(E, T):
    """2D error after the best rigid registration of the estimate on the truth (as done with real data)."""
    mE, mT = E.mean(0), T.mean(0); U, S_, Vt = np.linalg.svd((E - mE).T @ (T - mT)); Rm = (U @ Vt).T
    if np.linalg.det(Rm) < 0: Vt[-1] *= -1; Rm = (U @ Vt).T
    return np.hypot(*((E - mE) @ Rm.T + mT - T).T) * 100
# true hedgehog trajectories from the real recordings (slow triangle and out-and-back), z = 0.70 m
TRUTH = []
for n in ['test4_triangle', 'test5_in_out1']:
    e_, ok_, v_, H_ = errors(n, TAU); TRUTH.append(np.c_[H_[ok_], np.full(ok_.sum(), 0.70)])
TRUTH = np.vstack(TRUTH)[::3]
SIG_D = 0.003                  # range noise (m), consistent with the 3-4 mm jitter at rest
rsf = np.random.default_rng(5)
TRI = list(_it.combinations(IDS, 3))
DH = [-2.6, -2.0, -1.5, -1.0, -0.5, -0.25, -0.1, -0.05, 0.05, 0.1, 0.25, 0.5]
def simulate(k, dh):
    Wm = wrong_map(k, dh)
    D = np.linalg.norm(TRUTH[:, None, :] - np.array([TRUE[i] for i in IDS])[None], axis=2) + rsf.normal(0, SIG_D, (len(TRUTH), 4))
    pick = rsf.integers(0, 4, len(TRUTH))                 # triple used by the solver: uniform model
    est = np.array([tri_solve(Wm, TRI[pick[j]], [D[j, IDS.index(i)] for i in TRI[pick[j]]]) for j in range(len(TRUTH))])
    e_raw = rigid_err(est[:, :2], TRUTH[:, :2]); dz = np.abs(est[:, 2] - TRUTH[:, 2]) * 100
    B1.W, B1.Rt, B1.HEDGE_Z, B1.JUMP_W = Wm, TRUE, 0.70, 2.0
    X, s_, zd = B1.correct(est); keep = zd < 0.10
    e_cor = rigid_err(X[keep, :2], TRUTH[keep, :2])
    return {'raw_median': float(np.median(e_raw)), 'raw_p95': float(np.percentile(e_raw, 95)), 'wrong_z_pct': float(np.mean(dz > 40) * 100),
            'median_abs_dz_cm': float(np.median(dz)), 'detected_pct': float(np.mean(dz > 10) * 100),
            'corrected_median': float(np.median(e_cor)), 'corrected_p95': float(np.percentile(e_cor, 95)), 'kept_pct': float(keep.mean() * 100)}
SIM = {}
for k in IDS:
    for dh in DH:
        SIM[f'B{k} {dh:+.2f}'] = {kk: round(v, 2) for kk, v in simulate(k, dh).items()}
base = simulate(1, 0.0)
B1.W = {k: np.array(v) for k, v in B1.WRONG.items()}; B1.Rt = {k: np.array(v) for k, v in B1.RIGHT.items()}
NUM['montecarlo_configuration'] = {'samples_per_case': len(TRUTH), 'range_noise_m': SIG_D, 'triple_model': 'uniform random triple',
                                   'no_error': {kk: round(v, 2) for kk, v in base.items()}, 'cases': SIM,
                                   'measured_campaign1_for_comparison': {'as_recorded_median_cm': [NUM['per_recording'][n]['as_recorded_median'] for n in ['test2_triangle', 'test4_triangle', 'test5_in_out1']],
                                                                         'as_recorded_p95_cm': [NUM['per_recording'][n]['as_recorded_p95'] for n in ['test2_triangle', 'test4_triangle', 'test5_in_out1']]}}

# Fig. 8: Monte Carlo of the beacon height errors (x: |height error|; filled: beacon set too low, open: too high)
fig, axs = plt.subplots(1, 2, figsize=(W1, 2.3), gridspec_kw={'wspace': 0.45})
mks = {1: 'o', 2: 's', 4: '^', 5: 'D'}
neg = [dh for dh in DH if dh < 0]; pos = [dh for dh in DH if dh > 0]
for i_, k in enumerate(IDS):
    for grp, filled in [(neg, True), (pos, False)]:
        xs = np.abs(grp); r_ = [SIM[f'B{k} {dh:+.2f}'] for dh in grp]
        kw = dict(ms=2.8, lw=0.8, color=f'C{i_}', mfc=(f'C{i_}' if filled else 'none'))
        axs[0].plot(xs, [q['raw_p95'] for q in r_], mks[k] + ('-' if filled else '--'), label=(f'B{k}' if filled else None), **kw)
        axs[1].plot(xs, [q['median_abs_dz_cm'] for q in r_], mks[k] + ('-' if filled else '--'), **kw)
    r_ = [SIM[f'B{k} {dh:+.2f}'] for dh in neg]
    axs[0].plot(np.abs(neg), [q['corrected_p95'] for q in r_], ':', lw=0.8, color=f'C{i_}')
axs[0].plot([], [], 'k:', lw=0.8, label='corrected')
axs[0].plot([2.6], [max(NUM['montecarlo_configuration']['measured_campaign1_for_comparison']['as_recorded_p95_cm'])], '*', ms=7, color='k', label='C1 measured')
axs[1].axhline(10, color='k', lw=0.6, ls='--')
for ax in axs:
    ax.set_xscale('log'); ax.set_xticks([0.05, 0.25, 1, 2.6]); ax.set_xticklabels(['0.05', '0.25', '1', '2.6']); ax.minorticks_off()
    ax.set_xlabel('|height error| (m)'); ax.set_yscale('log')
axs[0].set_ylabel('p95 error (cm)'); axs[0].set_title('(a) position error', fontsize=8)
axs[1].set_ylabel('median |z error| (cm)'); axs[1].set_title('(b) height of the mobile beacon', fontsize=8)
fig.subplots_adjust(bottom=0.33); h_, l_ = axs[0].get_legend_handles_labels(); fig.legend(h_, l_, loc='lower center', bbox_to_anchor=(0.5, 0.0), ncol=3, fontsize=6.5, handletextpad=0.3, columnspacing=1.0)
save(fig, 'fig8_montecarlo')

json.dump(NUM, open(os.path.join(OUT, 'numbers.json'), 'w'), indent=1, default=float)
print(json.dumps(NUM, indent=1, default=float))
