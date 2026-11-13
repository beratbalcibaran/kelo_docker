"""Fig. 1 of Andrea's version: annotated photo of the test room with a lateral zoom on the robot
(3D view of the OptiTrack rigid body and of the Marvelmind mobile beacon).

Points were placed from Andrea's hand-annotated photo (photos/room_IMG_4706_annotated_by_Andrea.jpg),
on the clean photo photos/room_IMG_4706.jpg. Beacon IDs follow the Marvelmind map: right-handed frame,
x from the camera side towards the mural wall, y to the left (B1 near-left, B4 near-right above the
windows, B2 far-left, B5 on the mural wall near the window corner).

Usage (from the draft folder): python scripts/fig_setup_photo.py <rigid body markers csv>
"""
import csv, json, os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle, ConnectionPatch, FancyBboxPatch
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
MARKERS = sys.argv[1]
plt.rcParams.update({'font.family': 'serif', 'font.size': 7, 'savefig.bbox': 'tight', 'savefig.pad_inches': 0.02})

img = Image.open(os.path.join(ROOT, 'photos', 'room_IMG_4706.jpg'))
img = img.resize((2400, 1800), Image.LANCZOS); W, H = img.size
S = W / 1600.0                                         # points below are in the 1600 x 1200 frame
P = lambda x, y: (x * S, y * S)

BEAC = {'B1': ((185, 62), 2.60), 'B2': ((155, 495), 2.50), 'B5': ((521, 540), 2.55), 'B4': ((1293, 513), 2.40)}
VERT = {'A': (1100, 860), 'B': (640, 1045), 'C': (585, 875)}
OUT = {'D': (680, 778), 'E': (418, 803)}
OPTI = (1330, 565)
ROBOT_BOX = (895, 735, 1080, 915)                      # x0, y0, x1, y1
num = json.load(open(os.path.join(ROOT, 'figures', 'numbers.json')))
LEV1 = np.array(num['alignment']['lever_arm_before_move_m']); LEV2 = np.array(num['alignment']['lever_arm_after_move_m'])

ACC = {'beacon': '#c0392b', 'opti': '#1f77b4', 'tri': '#e67e22', 'out': '#8e44ad', 'robot': '#2c3e50'}
fig = plt.figure(figsize=(7.16, 2.75))
ax = fig.add_axes([0.0, 0.0, 0.53, 1.0]); ax.imshow(img); ax.set_axis_off()
ax.set_xlim(0, W); ax.set_ylim(H, 0)
# rounded frame around the photo
clip = FancyBboxPatch((0, 0), W, H, boxstyle='round,pad=0,rounding_size=60', transform=ax.transData, fc='none', ec='0.3', lw=0.8)
ax.add_patch(clip); ax.images[0].set_clip_path(clip)

def callout(xy, text, xytext, color, ha='center'):
    ax.annotate(text, xy=P(*xy), xytext=P(*xytext), ha=ha, va='center', fontsize=6.3, color='k',
                bbox=dict(boxstyle='round,pad=0.25,rounding_size=0.4', fc='white', ec=color, lw=0.8, alpha=0.95),
                arrowprops=dict(arrowstyle='-|>', color=color, lw=0.8, mutation_scale=6, shrinkA=0, shrinkB=2))

# OptiTrack working area (schematic: hull of the triangle, enlarged)
V = np.array([VERT[k] for k in 'ABC'], float); c = V.mean(0)
area = c + (V - c) * 1.35
ax.add_patch(Polygon([P(*p) for p in area], closed=True, fc=ACC['opti'], alpha=0.13, ec=ACC['opti'], lw=0.8, ls='--'))
# triangle and vertices
ax.add_patch(Polygon([P(*VERT[k]) for k in 'ABC'], closed=True, fc='none', ec=ACC['tri'], lw=1.4))
for k, (x, y) in VERT.items():
    ax.plot(*P(x, y), 'o', ms=4, color=ACC['tri'], mec='white', mew=0.6)
    dx, dy = {'A': (28, 22), 'B': (-34, 18), 'C': (-34, -10)}[k]
    ax.text(*P(x + dx, y + dy), k, fontsize=8, weight='bold', color=ACC['tri'], ha='center', va='center',
            bbox=dict(boxstyle='circle,pad=0.12', fc='white', ec=ACC['tri'], lw=0.6))
# out-and-back path (A-B-D-E-B-A) and outside stops
path = [VERT['B'], OUT['D'], OUT['E'], VERT['B']]
ax.plot(*zip(*[P(*p) for p in path]), ls=(0, (3, 2)), color=ACC['out'], lw=1.0)
for k, (x, y) in OUT.items():
    ax.plot(*P(x, y), 's', ms=4, color=ACC['out'], mec='white', mew=0.6)
    dx, dy = {'D': (30, -18), 'E': (-30, -16)}[k]
    ax.text(*P(x + dx, y + dy), k, fontsize=8, weight='bold', color=ACC['out'], ha='center', va='center',
            bbox=dict(boxstyle='circle,pad=0.12', fc='white', ec=ACC['out'], lw=0.6))
# beacons
for k, ((x, y), h) in BEAC.items():
    ax.plot(*P(x, y), 'o', ms=7, mfc='none', mec=ACC['beacon'], mew=1.0)
callout(BEAC['B1'][0], 'B1  (h = 2.60 m)', (420, 120), ACC['beacon'])
callout(BEAC['B2'][0], 'B2  (h = 2.50 m)', (215, 360), ACC['beacon'])
callout(BEAC['B5'][0], 'B5  (h = 2.55 m)', (600, 455), ACC['beacon'])
callout(BEAC['B4'][0], 'B4  (h = 2.40 m)', (1080, 400), ACC['beacon'])
ax.text(*P(800, 255), 'Marvelmind stationary beacons', fontsize=6.3, color=ACC['beacon'], ha='center', style='italic')
callout(OPTI, 'OptiTrack V120:Trio\n120 Hz, h = 1.93 m', (1440, 260), ACC['opti'])
callout((1000, 880), 'KELO robot\nmobile beacon + rigid body', (1300, 1075), ACC['robot'])
callout(tuple(area[1] * 0.5 + area[0] * 0.5), 'OptiTrack working area\n(about 2.5 m x 2.5 m)', (830, 1150), ACC['opti'])
callout((520, 960), 'triangle A-B-C\n(inside the area)', (330, 1110), ACC['tri'])
callout(((OUT['D'][0] + OUT['E'][0]) / 2, (OUT['D'][1] + OUT['E'][1]) / 2), 'out-and-back stops D, E\n(outside the area)', (300, 690), ACC['out'])
# zoom box on the robot
x0, y0, x1, y1 = ROBOT_BOX
ax.add_patch(Rectangle(P(x0, y0), (x1 - x0) * S, (y1 - y0) * S, fc='none', ec='0.2', lw=0.8))
ax.text(*P(30, 1170), '(a)', fontsize=8, weight='bold', ha='left', va='bottom', bbox=dict(boxstyle='round,pad=0.2', fc='white', ec='none', alpha=0.9))

# 3D view of the rigid body (robot body frame); Motive exports markers Y-up: plan = (x, -z), height = y
ax3 = fig.add_axes([0.6, 0.12, 0.4, 0.86], projection='3d')
mk = list(csv.DictReader(open(MARKERS)))
MP = np.array([[float(r['x']), -float(r['z']), float(r['y'])] for r in mk]) * 100
low = MP[:, 2] < -5; hz = float(MP[low, 2].mean()); zf = hz - 12
for P_, m_, lab in [(MP[~low], 'o', 'mast markers (2 x 5)'), (MP[low], 's', 'lower markers (3)')]:
    ax3.scatter(P_[:, 0], P_[:, 1], P_[:, 2], marker=m_, s=12, facecolors='none', edgecolors='k', linewidths=0.7, label=lab, depthshade=False)
    for q in P_: ax3.plot([q[0]] * 2, [q[1]] * 2, [zf, q[2]], color='0.78', lw=0.4)
ax3.scatter([0], [0], [0], marker='+', s=40, color='k', label='rigid body pivot', depthshade=False)
for L, m_, col, lab in [(LEV1, 'v', '#e67e22', 'mobile beacon, M1'), (LEV2, '^', '#1f77b4', 'mobile beacon, M2')]:
    ax3.scatter([L[0] * 100], [L[1] * 100], [hz], marker=m_, s=34, color=col, label=lab, depthshade=False)
    ax3.plot([L[0] * 100] * 2, [L[1] * 100] * 2, [zf, hz], color=col, lw=0.6, ls='--')
ax3.set_xlabel('x (cm)', labelpad=-7, fontsize=6.5); ax3.set_ylabel('y (cm)', labelpad=-7, fontsize=6.5); ax3.set_zlabel('z (cm)', labelpad=-8, fontsize=6.5)
ax3.tick_params(labelsize=5.5, pad=-3)
ax3.set_zlim(zf, 10); ax3.set_box_aspect((1, 1, 0.6), zoom=1.15); ax3.view_init(elev=20, azim=60)
ax3.legend(loc='upper center', bbox_to_anchor=(0.5, 0.02), ncol=3, fontsize=5.8, handletextpad=0.2, columnspacing=0.6, frameon=False)
fig.text(0.6, 0.97, '(b) robot: OptiTrack markers and mobile beacon', fontsize=7, weight='bold', ha='left', va='top')

# zoom connectors from the robot box to panel (b)
for (px, py), (qx, qy) in [((x1, y0), (0.04, 0.78)), ((x1, y1), (0.04, 0.22))]:
    fig.add_artist(ConnectionPatch(xyA=P(px, py), coordsA=ax.transData, xyB=(qx, qy), coordsB=ax3.transAxes,
                                   color='0.35', lw=0.6, ls='-'))

out = os.path.join(ROOT, 'figures', 'fig1_photo.pdf')
fig.savefig(out, dpi=300); fig.savefig(out.replace('.pdf', '.png'), dpi=200)
print('written', out)
