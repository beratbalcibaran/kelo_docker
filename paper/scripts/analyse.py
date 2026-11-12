"""Filters on the Marvelmind data of 12 Nov, scored against OptiTrack (data embedded in Berat's reports)."""
import json, math, bisect, statistics as st

def load(name):
    d = json.load(open(name + '.json'))
    o, m = d['opt'], d['mm']
    ot = o['t']
    def ref(t):
        j = bisect.bisect_left(ot, t)
        if j <= 0 or j >= len(ot): return None
        a, b = j - 1, j
        if o['x'][a] is None or o['x'][b] is None or ot[b] - ot[a] > 0.25: return None
        w = (t - ot[a]) / (ot[b] - ot[a])
        return (o['x'][a] + w * (o['x'][b] - o['x'][a]), o['y'][a] + w * (o['y'][b] - o['y'][a]))
    # red parts = fallen markers (Andrea): group samples with error > 25 cm, pad 0.5 s
    red = []
    for t, e in zip(m['t'], m['e']):
        if e is not None and e > 0.25:
            if red and t - red[-1][1] <= 2: red[-1][1] = t
            else: red.append([t, t])
    isred = lambda t: any(a - 0.5 <= t <= b + 0.5 for a, b in red)
    S = []
    for t in m['t']:
        p = ref(t); p1 = ref(t - 0.15); p2 = ref(t + 0.15)
        ok = p is not None and p1 is not None and p2 is not None and not isred(t)
        v = math.hypot(p2[0] - p1[0], p2[1] - p1[1]) / 0.3 if ok else None
        S.append((ok, p, v))
    return {'t': m['t'], 'x': m['x'], 'y': m['y'], 'S': S, 'ref': ref}

def gate(t, x, y, G=0.20, vmax=0.6, reset=5):
    """Reject jumps: a sample further than G + vmax*dt from the last accepted one. Returns accepted mask."""
    acc = []; last = None; nrej = 0
    for i in range(len(t)):
        if last is None: acc.append(True); last = i; continue
        d = math.hypot(x[i] - x[last], y[i] - y[last]); dt = t[i] - t[last]
        if d <= G + vmax * dt or nrej >= reset:
            acc.append(True); last = i; nrej = 0
        else:
            acc.append(False); nrej += 1
    return acc

def hold(t, x, y, acc):
    """Output at every sample time: last accepted position."""
    ox, oy = [], []; lx = ly = None
    for i in range(len(t)):
        if acc[i]: lx, ly = x[i], y[i]
        ox.append(lx); oy.append(ly)
    return ox, oy

def movavg(t, x, y, acc, N):
    ox, oy = [], []; bx, by = [], []
    for i in range(len(t)):
        if acc[i]:
            bx.append(x[i]); by.append(y[i]); bx = bx[-N:]; by = by[-N:]
        ox.append(sum(bx) / len(bx)); oy.append(sum(by) / len(by))
    return ox, oy

def kf1d(t, z, acc, q, r, smooth=False):
    """Constant-velocity Kalman filter on one axis; RTS smoother if smooth."""
    X = [z[0], 0.0]; P = [[r * r, 0], [0, 1.0]]
    xs, Ps, xp, Pp, Fs = [], [], [], [], []
    tp = t[0]
    for i in range(len(t)):
        dt = max(t[i] - tp, 1e-3); tp = t[i]
        F = [[1, dt], [0, 1]]
        Xp = [X[0] + dt * X[1], X[1]]
        Q = [[q * dt ** 3 / 3, q * dt ** 2 / 2], [q * dt ** 2 / 2, q * dt]]
        PP = [[P[0][0] + dt * (P[1][0] + P[0][1]) + dt * dt * P[1][1] + Q[0][0], P[0][1] + dt * P[1][1] + Q[0][1]],
              [P[1][0] + dt * P[1][1] + Q[1][0], P[1][1] + Q[1][1]]]
        if acc[i]:
            Sv = PP[0][0] + r * r
            K = [PP[0][0] / Sv, PP[1][0] / Sv]
            inn = z[i] - Xp[0]
            X = [Xp[0] + K[0] * inn, Xp[1] + K[1] * inn]
            P = [[(1 - K[0]) * PP[0][0], (1 - K[0]) * PP[0][1]], [PP[1][0] - K[1] * PP[0][0], PP[1][1] - K[1] * PP[0][1]]]
        else:
            X, P = Xp, PP
        xs.append(X[:]); Ps.append([row[:] for row in P]); xp.append(Xp); Pp.append(PP); Fs.append(F)
    if not smooth: return [s[0] for s in xs]
    n = len(t); Xs = [None] * n; Xs[-1] = xs[-1]; PsS = Ps[-1]
    for k in range(n - 2, -1, -1):
        F = Fs[k + 1]; PPn = Pp[k + 1]; Pk = Ps[k]
        det = PPn[0][0] * PPn[1][1] - PPn[0][1] * PPn[1][0]
        inv = [[PPn[1][1] / det, -PPn[0][1] / det], [-PPn[1][0] / det, PPn[0][0] / det]]
        PF = [[Pk[0][0] * F[0][0] + Pk[0][1] * F[0][1], Pk[0][0] * F[1][0] + Pk[0][1] * F[1][1]],
              [Pk[1][0] * F[0][0] + Pk[1][1] * F[0][1], Pk[1][0] * F[1][0] + Pk[1][1] * F[1][1]]]
        C = [[PF[0][0] * inv[0][0] + PF[0][1] * inv[1][0], PF[0][0] * inv[0][1] + PF[0][1] * inv[1][1]],
             [PF[1][0] * inv[0][0] + PF[1][1] * inv[1][0], PF[1][0] * inv[0][1] + PF[1][1] * inv[1][1]]]
        d = [Xs[k + 1][0] - xp[k + 1][0], Xs[k + 1][1] - xp[k + 1][1]]
        Xs[k] = [xs[k][0] + C[0][0] * d[0] + C[0][1] * d[1], xs[k][1] + C[1][0] * d[0] + C[1][1] * d[1]]
    return [s[0] for s in Xs]

def kf(t, x, y, acc, q, r=0.03, smooth=False):
    return kf1d(t, x, acc, q, r, smooth), kf1d(t, y, acc, q, r, smooth)

def pct(v, p):
    v = sorted(v); k = (len(v) - 1) * p; f = int(k); c = min(f + 1, len(v) - 1); return v[f] + (v[c] - v[f]) * (k - f)
rms = lambda v: math.sqrt(sum(e * e for e in v) / len(v)) if v else float('nan')

def score(D, fx, fy):
    rest, mov, bins = [], [], {}
    for i, (ok, p, v) in enumerate(D['S']):
        if not ok: continue
        e = math.hypot(fx[i] - p[0], fy[i] - p[1]) * 100
        if v < 0.03: rest.append(e)
        else:
            mov.append(e); bins.setdefault(min(int(v / 0.1), 4), []).append(e)
    return {'rest': rms(rest), 'rest95': pct(rest, .95), 'mov': rms(mov), 'mov95': pct(mov, .95),
            'bins': {k: rms(b) for k, b in sorted(bins.items())}, 'J': (rms(rest) + rms(mov)) / 2}

def delay(D, fx, fy):
    """Shift tau (s) that best matches the moving samples to OptiTrack at t - tau."""
    best = None
    for k in range(0, 61):
        tau = k * 0.02; es = []
        for i, (ok, p, v) in enumerate(D['S']):
            if not ok or v < 0.1: continue
            q = D['ref'](D['t'][i] - tau)
            if q is None: continue
            es.append(math.hypot(fx[i] - q[0], fy[i] - q[1]))
        r = rms(es)
        if best is None or r < best[1]: best = (tau, r)
    return best[0]

def run(D, kind, par):
    t, x, y = D['t'], D['x'], D['y']
    if kind == 'raw': return x, y
    acc = gate(t, x, y)
    if kind == 'gate': return hold(t, x, y, acc)
    if kind == 'ma': return movavg(t, x, y, acc, par)
    if kind == 'kf': return kf(t, x, y, acc, par)
    if kind == 'rts': return kf(t, x, y, acc, par, smooth=True)

if __name__ == '__main__':
    GRID = {'raw': [None], 'gate': [None], 'ma': [2, 3, 5, 8, 12], 'kf': [0.003, 0.01, 0.03, 0.1, 0.3, 1.0], 'rts': [0.003, 0.01, 0.03, 0.1, 0.3, 1.0]}
    LABEL = {'raw': 'raw', 'gate': 'jump rejection', 'ma': 'moving avg (causal)', 'kf': 'Kalman CV (causal)', 'rts': 'Kalman + RTS (offline)'}
    data = {n: load(n) for n in ['run1', 'last40']}
    print('valid reference samples:', {n: sum(s[0] for s in D['S']) for n, D in data.items()})
    for kind, grid in GRID.items():
        print(f"\n### {LABEL[kind]}")
        for tune, test in [('last40', 'run1'), ('run1', 'last40')]:
            best = min(grid, key=lambda p: score(data[tune], *run(data[tune], kind, p))['J'])
            fx, fy = run(data[test], kind, best)
            s = score(data[test], fx, fy)
            dl = delay(data[test], fx, fy)
            b = ' '.join(f"{k/10:.1f}:{v:.1f}" for k, v in s['bins'].items())
            print(f"  test {test:6s} (param {best} tuned on {tune}): rest RMS {s['rest']:.1f} p95 {s['rest95']:.1f} | moving RMS {s['mov']:.1f} p95 {s['mov95']:.1f} | delay {dl:.2f} s | by speed {b}")
