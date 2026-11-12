"""Preliminary figures for the I2MTC paper from the data embedded in Berat's reports (12 Nov 2026)."""
import json, math, csv, sys, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from analyse import load, gate, kf, score

OUT = sys.argv[1]; PM = sys.argv[2]
os.makedirs(OUT, exist_ok=True)
plt.rcParams.update({'font.family': 'serif', 'font.size': 8, 'axes.linewidth': 0.6, 'lines.linewidth': 1.0,
                     'savefig.bbox': 'tight', 'savefig.pad_inches': 0.02, 'axes.grid': True, 'grid.linewidth': 0.3,
                     'grid.alpha': 0.5, 'legend.fontsize': 7, 'legend.frameon': False})
W1 = 3.45  # IEEE column width, inches
NUM = {}
def save(fig, name):
    fig.savefig(os.path.join(OUT, name + '.pdf')); fig.savefig(os.path.join(OUT, name + '.png'), dpi=200); plt.close(fig)

data = {n: load(n) for n in ['run1', 'last40']}
raw_json = {n: json.load(open(n + '.json')) for n in ['run1', 'last40']}

# ---------- Fig 1: triangle paths (morning, last 40 points) ----------
D = data['last40']; R = raw_json['last40']
ot = R['opt']
ox = np.array([np.nan if v is None else v for v in ot['x']]); oy = np.array([np.nan if v is None else v for v in ot['y']])
ok = np.array([s[0] for s in D['S']])
mx = np.array(D['x']); my = np.array(D['y'])
VERT = [(3.1, 7.3), (4.5, 5.9), (5.4, 7.5)]
red0 = []
for t_, e_ in zip(R['mm']['t'], R['mm']['e']):
    if e_ is not None and e_ > 0.25:
        if red0 and t_ - red0[-1][1] <= 2: red0[-1][1] = t_
        else: red0.append([t_, t_])
otv0 = np.array(ot['t'])
for a_, b_ in red0:
    msk = (otv0 >= a_ - 0.5) & (otv0 <= b_ + 0.5); ox[msk] = np.nan; oy[msk] = np.nan
fig, ax = plt.subplots(figsize=(W1, 2.6))
ax.plot(mx[ok], my[ok], '.', ms=1.2, color='0.55', label='Marvelmind', rasterized=True)
ax.plot(ox, oy, '-', lw=0.5, color='k', label='OptiTrack (reference)')
for i, (vx, vy) in enumerate(VERT):
    ax.annotate('ABC'[i], (vx, vy), xytext=(10, 10), textcoords='offset points', fontsize=9, weight='bold', bbox=dict(boxstyle='round,pad=0.15', fc='w', ec='none', alpha=0.85))
ax.set_aspect('equal'); ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.18), ncol=2, markerscale=6)
save(fig, 'fig1_triangle_paths')

# ---------- Fig 2: offsets per visit at the vertices ----------
red = []
for t, e in zip(R['mm']['t'], R['mm']['e']):
    if e is not None and e > 0.25:
        if red and t - red[-1][1] <= 2: red[-1][1] = t
        else: red.append([t, t])
longbad = [x for x in red if x[1] - x[0] >= 3]
otv = np.array(ot['t'])
fig, ax = plt.subplots(figsize=(W1, 2.4))
mk = ['o', 's', '^']; vstats = []
for i, (vx, vy) in enumerate(VERT):
    offs = []
    for s in R['stops']:
        if any(a <= s['b'] and s['a'] <= b for a, b in longbad): continue
        so = (otv >= s['a']) & (otv <= s['b']) & np.isfinite(ox)
        sm = (np.array(D['t']) >= s['a']) & (np.array(D['t']) <= s['b'])
        if so.sum() < 10 or sm.sum() < 10: continue
        px, py = ox[so].mean(), oy[so].mean()
        if math.hypot(px - vx, py - vy) > 0.45: continue
        offs.append(((np.median(mx[sm]) - px) * 100, (np.median(my[sm]) - py) * 100))
    offs = np.array(offs); mu = offs.mean(0); sd = offs.std(0, ddof=1)
    vstats.append({'vertex': 'ABC'[i], 'visits': len(offs), 'mean_cm': [round(v, 1) for v in mu],
                   'mean_norm_cm': round(float(np.hypot(*mu)), 1), 'std_cm': [round(v, 1) for v in sd]})
    ax.plot(offs[:, 0], offs[:, 1], mk[i], ms=3.5, mfc='none', mew=0.7, color=f'C{i}', label=f'{"ABC"[i]} ({len(offs)} visits)')
    ax.plot(*mu, mk[i], ms=7, color=f'C{i}', mec='k', mew=0.6)
ax.axhline(0, color='k', lw=0.5); ax.axvline(0, color='k', lw=0.5)
ax.set_aspect('equal'); ax.set_xlabel(r'offset $\Delta x$ (cm)'); ax.set_ylabel(r'offset $\Delta y$ (cm)')
ax.legend(loc='upper left', bbox_to_anchor=(1.0, 1.0))
save(fig, 'fig2_vertex_offsets'); NUM['fig2_vertex_offsets'] = vstats

# ---------- Fig 3: error vs speed (morning) ----------
def by_speed(D, edges):
    out = [[] for _ in edges[:-1]]
    for i, (ok_, p, v) in enumerate(D['S']):
        if not ok_: continue
        e = math.hypot(D['x'][i] - p[0], D['y'][i] - p[1]) * 100
        k = np.searchsorted(edges, v, side='right') - 1
        if 0 <= k < len(out): out[k].append(e)
    return out
edges = np.array([0, 0.03, 0.1, 0.2, 0.3, 0.4, 0.55])
fig, ax = plt.subplots(figsize=(W1, 2.0)); NUM['fig3_error_vs_speed'] = {}
for j, (n, lab) in enumerate([('run1', 'free path (run 1)'), ('last40', 'triangle (last 40 points)')]):
    bins = by_speed(data[n], edges)
    xs, ys, lo, hi, ns = [], [], [], [], []
    rng = np.random.default_rng(0)
    for k, b in enumerate(bins):
        if len(b) < 20: continue
        b = np.array(b); r = math.sqrt((b ** 2).mean())
        bs = [math.sqrt((rng.choice(b, len(b)) ** 2).mean()) for _ in range(300)]
        xs.append((edges[k] + edges[k + 1]) / 2 + (j - 0.5) * 0.012); ys.append(r)
        lo.append(r - np.percentile(bs, 2.5)); hi.append(np.percentile(bs, 97.5) - r); ns.append(len(b))
    ax.errorbar(xs, ys, yerr=[lo, hi], fmt='o-' if j == 0 else 's--', ms=3, capsize=2, color=f'C{j}', label=lab)
    NUM['fig3_error_vs_speed'][n] = {'bin_edges_m_s': edges.tolist(), 'rms_cm': [round(v, 1) for v in ys], 'n': ns}
ax.set_xlabel('speed from OptiTrack (m/s)'); ax.set_ylabel('error RMS (cm)'); ax.set_ylim(bottom=0); ax.legend(loc='upper left')
save(fig, 'fig3_error_vs_speed')

# ---------- Fig 4: CDF, processing chain (tuned on the other recording) ----------
def errs(D, fx, fy, moving):
    e = []
    for i, (ok_, p, v) in enumerate(D['S']):
        if not ok_ or (v >= 0.03) != moving: continue
        e.append(math.hypot(fx[i] - p[0], fy[i] - p[1]) * 100)
    return np.sort(e)
fig, axs = plt.subplots(1, 2, figsize=(W1, 1.9), sharey=True)
D = data['last40']; t, x, y = D['t'], D['x'], D['y']; acc = gate(t, x, y)
variants = [('raw', (x, y), '-', 'k'), ('Kalman, causal', kf(t, x, y, acc, 0.3), '--', 'C0'),
            ('Kalman + RTS, offline', kf(t, x, y, acc, 0.003, smooth=True), ':', 'C3')]
NUM['fig4_cdf_processing'] = {}
for k, moving in enumerate([False, True]):
    for lab, (fx, fy), ls, c in variants:
        e = errs(D, fx, fy, moving)
        axs[k].plot(e, np.arange(1, len(e) + 1) / len(e), ls, color=c, label=lab)
        NUM['fig4_cdf_processing'][f'{"moving" if moving else "rest"} / {lab}'] = {
            'rms_cm': round(float(np.sqrt((e ** 2).mean())), 1), 'p95_cm': round(float(np.percentile(e, 95)), 1), 'n': len(e)}
    axs[k].set_title('moving' if moving else 'at rest'); axs[k].set_xlabel('error (cm)'); axs[k].set_xlim(0, 35)
axs[0].set_ylabel('CDF'); axs[0].legend(loc='upper center', bbox_to_anchor=(1.1, -0.28), ncol=3)
save(fig, 'fig4_cdf_processing')

# ---------- Fig 5: heading dependence (afternoon, common alignment) ----------
rows = list(csv.DictReader(open(os.path.join(PM, 'tablolar', 'common_alignment_durmalar.csv'))))
def place(r):
    px, py = float(r['px']), float(r['py'])
    if r['kayit'].endswith(('test1', 'test2')) and 'triangle' not in r['kayit']: return 'P1 (rotation in place)'
    return 'vertex near (0.5, 0.4) m' if px > -0.3 else 'vertex near (-1.2, -1.2) m'
fig, ax = plt.subplots(figsize=(W1, 2.1)); NUM['fig5_heading'] = []
styles = {'P1 (rotation in place)': ('o', 'C0'), 'vertex near (0.5, 0.4) m': ('s', 'C1'), 'vertex near (-1.2, -1.2) m': ('^', 'C2')}
for pl, (m_, c) in styles.items():
    sel = [r for r in rows if place(r) == pl]
    ax.plot([float(r['yaw_deg']) for r in sel], [float(r['sapma_cm']) for r in sel], m_, color=c, ms=4, mfc='none', mew=0.8, label=pl)
for r in rows:
    NUM['fig5_heading'].append({'recording': r['kayit'], 'stop': int(r['nokta']), 'place': place(r),
                                'heading_deg': float(r['yaw_deg']), 'offset_cm': float(r['sapma_cm'])})
ax.set_xlabel('robot heading from OptiTrack (deg)'); ax.set_ylabel('offset at the stop (cm)')
ax.set_xticks(range(-180, 181, 90)); ax.set_xlim(-180, 180); ax.set_ylim(bottom=0); ax.legend(loc='upper left')
save(fig, 'fig5_heading')

# ---------- Fig 6: z consistency check (afternoon, common alignment) ----------
summ = {r['kayit']: r for r in csv.DictReader(open(os.path.join(PM, 'tablolar', 'common_alignment_ozet.csv')))}
recs = [('182024_test2_triangle', 'triangle\n(fast)'), ('182940_test4_triangle', 'triangle\n(slow)'), ('185141_test5_in_out1', 'in/out')]
fig, ax = plt.subplots(figsize=(W1, 1.9)); xs = np.arange(len(recs)); w = 0.2
vals = {k: [float(summ[r][k]) for r, _ in recs] for k in ['hareket_rms_ham_cm', 'hareket_rms_zfilt_cm', 'hareket_p95_ham_cm', 'hareket_p95_zfilt_cm', 'z_disi_pct']}
ax.bar(xs - 1.5 * w, vals['hareket_rms_ham_cm'], w, color='0.3', label='RMS, all samples')
ax.bar(xs - 0.5 * w, vals['hareket_rms_zfilt_cm'], w, color='0.75', label='RMS, z check')
ax.bar(xs + 0.5 * w, vals['hareket_p95_ham_cm'], w, color='0.3', hatch='///', label='p95, all samples')
ax.bar(xs + 1.5 * w, vals['hareket_p95_zfilt_cm'], w, color='0.75', hatch='///', label='p95, z check')
for i, p in enumerate(vals['z_disi_pct']):
    ax.text(xs[i] - 0.5 * w, max(vals['hareket_p95_ham_cm'][i], vals['hareket_p95_zfilt_cm'][i]) + 6, f'{p:.0f} % removed', ha='center', fontsize=6.5)
ax.set_xticks(xs); ax.set_xticklabels([l for _, l in recs]); ax.set_ylabel('moving error (cm)'); ax.set_ylim(0, 185)
ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.32), ncol=2)
save(fig, 'fig6_zcheck'); NUM['fig6_zcheck'] = {r: {k: vals[k][i] for k in vals} for i, (r, _) in enumerate(recs)}

# ---------- Fig 7: in/out test (afternoon) ----------
pm = json.load(open('pm_recs.json'))[4]
mt = np.array(pm['mm']['t']); pmx = np.array(pm['mm']['x'], float); pmy = np.array(pm['mm']['y'], float)
pe = np.array([np.nan if v is None else v * 100 for v in pm['mm']['e']])
pot = np.array(pm['opt']['t']); pok = np.array([v is not None for v in pm['opt']['x']])
gaps = []; st = None
for ti, v in zip(pot, pok):
    if not v and st is None: st = ti
    if v and st is not None:
        if ti - st > 10: gaps.append((st, ti))
        st = None
# stationary Marvelmind periods outside the OptiTrack area (>= 8 s within 6 cm of their median)
outs = []
for a, b in gaps:
    idx = np.where((mt > a) & (mt < b))[0]; i = 0
    while i < len(idx):
        j = i
        while j + 1 < len(idx) and mt[idx[j + 1]] - mt[idx[j]] < 1.0:
            seg = idx[i:j + 2]
            if np.hypot(pmx[seg] - np.median(pmx[seg]), pmy[seg] - np.median(pmy[seg])).max() > 0.06: break
            j += 1
        seg = idx[i:j + 1]
        if len(seg) > 1 and mt[seg[-1]] - mt[seg[0]] >= 8:
            outs.append((mt[seg[0]], mt[seg[-1]], float(np.median(pmx[seg])), float(np.median(pmy[seg])), len(seg)))
        i = j + 1
cl = []
for o in outs:
    for c in cl:
        if math.hypot(o[2] - c[0][2], o[3] - c[0][3]) < 0.5: c.append(o); break
    else: cl.append([o])
# re-entry: error 10 s before exit and 10 s after re-entry
reent = []
for a, b in gaps:
    pre = pe[(mt > a - 10) & (mt < a) & np.isfinite(pe)]; post = pe[(mt > b) & (mt < b + 10) & np.isfinite(pe)]
    if len(pre) > 5 and len(post) > 5: reent.append((float(np.median(pre)), float(np.median(post))))
fig, axs = plt.subplots(2, 1, figsize=(W1, 3.4), gridspec_kw={'height_ratios': [1, 1.3], 'hspace': 0.45})
ax = axs[0]
for a, b in gaps: ax.axvspan(a, b, color='0.85', lw=0)
ax.plot(mt, pe, '.', ms=1, color='k', rasterized=True)
ax.set_xlabel('time (s)'); ax.set_ylabel('error (cm)'); ax.set_ylim(0, 80); ax.set_xlim(0, mt[-1])
ax.text(0.99, 0.95, 'grey: outside the OptiTrack area', transform=ax.transAxes, ha='right', va='top', fontsize=6.5)
ax = axs[1]
pox = np.array([np.nan if v is None else v for v in pm['opt']['x']]); poy = np.array([np.nan if v is None else v for v in pm['opt']['y']])
ax.plot(pmx, pmy, '.', ms=0.8, color='0.7', rasterized=True, label='Marvelmind (z check)')
ax.plot(pox, poy, '-', lw=0.4, color='k', label='OptiTrack')
NUM['fig7_inout'] = {'outside_points': [], 'reentry_median_error_cm_before_after': [[round(p, 1), round(q, 1)] for p, q in reent]}
for k, c in enumerate(sorted(cl, key=len, reverse=True)):
    P = np.array([[o[2], o[3]] for o in c])
    if len(P) < 2: continue
    lab = 'CD'[k] if k < 2 else f'P{k}'
    sd = P.std(0, ddof=1) * 100
    NUM['fig7_inout']['outside_points'].append({'point': lab, 'stops': len(P), 'centre_m': [round(v, 3) for v in P.mean(0)],
                                                'std_cm': [round(v, 1) for v in sd], 'max_dist_from_centre_cm': round(float(np.hypot(*(P - P.mean(0)).T).max() * 100), 1)})
ax.set_aspect('equal'); ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.legend(loc='upper left', markerscale=3)
save(fig, 'fig7_inout')
json.dump(NUM, open(os.path.join(OUT, 'figures_numbers.json'), 'w'), indent=1)
print(json.dumps(NUM, indent=1)[:6000])
