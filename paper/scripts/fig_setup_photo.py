"""Fig. 1: annotated photo of the test room with a lateral zoom on the robot (3D view of the OptiTrack
rigid body and of the Marvelmind mobile beacon).

Style: one label style (cream rounded boxes, thin border, dark sans-serif text), one arrow colour (muted
red, curved arrows), labels along the edges of the photo, dashed grey zoom lines, light blue backdrop.

Points were placed from Andrea's hand-annotated photo (photos/room_IMG_4706_annotated_by_Andrea.jpg, kept
only in Andrea's draft folder) on the clean photo photos/room_IMG_4706.jpg. Beacon IDs follow the
Marvelmind map (right-handed frame): B1 near-left on the ceiling, B4 near-right above the window, B2
far-left, B5 on the mural wall near the window corner.

Usage (from the paper folder): python scripts/fig_setup_photo.py <optitrack_rigid_body_markers.csv> [vertical|horizontal] [nofade]
vertical -> figures/fig1_teaser.pdf (one column, first page); horizontal -> figures/fig1_photo.pdf (two columns).
"""
import csv, json, os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle, ConnectionPatch, FancyBboxPatch
from PIL import Image, ImageDraw, ImageFilter

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
MARKERS = sys.argv[1]
LAYOUT = sys.argv[2] if len(sys.argv) > 2 else 'vertical'     # 'vertical' (teaser, one column) or 'horizontal' (two columns)
VERTICAL = LAYOUT == 'vertical'
FADE = 'nofade' not in sys.argv[3:]                          # 'nofade': original photo, no focus mask
plt.rcParams.update({'font.family': 'sans-serif', 'font.sans-serif': ['Helvetica', 'Arial', 'DejaVu Sans'],
                     'font.size': 6.5, 'savefig.bbox': 'tight', 'savefig.pad_inches': 0.02})

# palette (as in the reference figure)
LABEL_FC, LABEL_EC, TEXT = '#f6efdc', '#dccba1', '#3a3a3a'
ARROW = '#b5525c'                      # muted red for all arrows and floor marks
AREA = '#9cc3dd'                       # light blue for the OptiTrack area
BACK = '#e3eef6'                       # backdrop
LBOX = dict(boxstyle='round,pad=0.35,rounding_size=0.5', fc=LABEL_FC, ec=LABEL_EC, lw=0.6)

CROP_X = 1460                                          # crop the photo right of the OptiTrack tripod (1600 x 1200 frame)
img = Image.open(os.path.join(ROOT, 'photos', 'room_IMG_4706.jpg')).convert('RGB')
img = img.crop((0, 0, round(img.width * CROP_X / 1600), img.height))
S = 1.5                                                # points below are in the 1600 x 1200 frame
P = lambda x, y: (x * S, y * S)
img = img.resize((round(CROP_X * S), round(1200 * S)), Image.LANCZOS); W, H = img.size

# Focus mask: keep the test area sharp, fade the rest of the room, fade furniture more.
FADE_ROOM, FADE_CLUTTER = 0.30, 0.62                  # fraction blended towards white
KEEP = {'floor': [(300, 770), (720, 742), (1130, 790), (1185, 900), (900, 1200), (330, 1200), (225, 950)],
        'robot': [(880, 720), (1095, 720), (1095, 930), (880, 930)],
        'tripod': [(1225, 540), (1415, 540), (1415, 965), (1225, 965)]}
KEEP_CIRCLES = [(185, 62), (155, 495), (521, 540), (1293, 513)]     # stationary beacons
CLUTTER = {'chairs_left': [(0, 640), (175, 640), (290, 720), (290, 900), (175, 1200), (0, 1200)],
           'tables_back': [(150, 640), (1110, 640), (1110, 790), (720, 742), (300, 770), (150, 790)],
           'desk_right': [(1100, 735), (1460, 735), (1460, 1200), (900, 1200), (1185, 900), (1130, 790)]}
def poly_mask(polys, circles=(), r=38):
    m = Image.new('L', (W, H), 0); d = ImageDraw.Draw(m)
    for pts in polys: d.polygon([P(*q) for q in pts], fill=255)
    for (cx, cy) in circles: d.ellipse([*P(cx - r, cy - r), *P(cx + r, cy + r)], fill=255)
    return np.asarray(m.filter(ImageFilter.GaussianBlur(radius=18 * S)), float) / 255.0
if FADE:
    keep = poly_mask(KEEP.values(), KEEP_CIRCLES); clutter = poly_mask(CLUTTER.values())
    fade = (FADE_ROOM + (FADE_CLUTTER - FADE_ROOM) * clutter) * (1.0 - keep)
    arr = np.asarray(img, float)
    arr = arr * (1 - fade[..., None]) + 255.0 * fade[..., None]
    img = Image.fromarray(arr.clip(0, 255).astype(np.uint8))

BEAC = {'B1': ((185, 62), 2.60), 'B2': ((155, 495), 2.50), 'B5': ((521, 540), 2.55), 'B4': ((1293, 513), 2.40)}
VERT = {'A': (1100, 860), 'B': (640, 1045), 'C': (585, 875)}
OUT = {'D': (680, 778), 'E': (418, 803)}
OPTI = (1330, 565)
ROBOT_BOX = (895, 735, 1080, 915)                      # x0, y0, x1, y1
num = json.load(open(os.path.join(ROOT, 'figures', 'numbers.json')))
LEV1 = np.array(num['alignment']['lever_arm_before_move_m']); LEV2 = np.array(num['alignment']['lever_arm_after_move_m'])

if VERTICAL:   # photo on top, zoom on the robot below (one IEEE column)
    fig = plt.figure(figsize=(3.5, 4.5))
    PHOTO_RECT, AX3_RECT = [0.015, 0.378, 0.97, 0.615], [0.02, 0.09, 0.96, 0.3]
else:          # photo on the left, zoom on the right (two columns)
    fig = plt.figure(figsize=(7.16, 2.8))
    PHOTO_RECT, AX3_RECT = [0.012, 0.03, 0.48, 0.94], [0.54, 0.13, 0.45, 0.8]
bg = fig.add_axes([0, 0, 1, 1]); bg.set_axis_off(); bg.set_xlim(0, 1); bg.set_ylim(0, 1)
bg.add_patch(FancyBboxPatch((0.005, 0.005), 0.99, 0.99, boxstyle='round,pad=0,rounding_size=0.03', fc=BACK, ec='none'))

ax = fig.add_axes(PHOTO_RECT); ax.imshow(img); ax.set_axis_off()
ax.set_xlim(0, W); ax.set_ylim(H, 0)
frame = FancyBboxPatch((0, 0), W, H, boxstyle='round,pad=0,rounding_size=70', transform=ax.transData, fc='none', ec='white', lw=1.2)
ax.add_patch(frame); ax.images[0].set_clip_path(frame)

def label(text, xy_text, targets, ha='center'):
    """One cream label with one or more curved arrows to the targets."""
    t = ax.text(*P(*xy_text), text, ha=ha, va='center', fontsize=6.5, color=TEXT, bbox=LBOX, zorder=6)
    for xy, r in targets:
        ax.annotate('', xy=P(*xy), xytext=P(*xy_text), zorder=5,
                    arrowprops=dict(arrowstyle='-|>', color=ARROW, lw=0.9, mutation_scale=7, shrinkA=0, shrinkB=3,
                                    connectionstyle=f'arc3,rad={r}', patchA=t.get_bbox_patch()))
    return t

def tag(xy, text, side):
    """Name/height tag next to a beacon circle, never touching it (gap of 34 px in the 1600 frame)."""
    x, y = xy; g = 34
    pos = {'right': ((x + g, y), 'left', 'center'), 'left': ((x - g, y), 'right', 'center'),
           'above': ((x, y - g), 'center', 'bottom'), 'below': ((x, y + g), 'center', 'top')}[side]
    (tx, ty), ha, va = pos
    ax.text(*P(tx, ty), text, ha=ha, va=va, fontsize=5.6, color=TEXT,
            bbox=dict(boxstyle='round,pad=0.2', fc='white', ec=LABEL_EC, lw=0.4, alpha=0.95), zorder=8)

# OptiTrack working area (schematic: hull of the triangle, enlarged)
V = np.array([VERT[k] for k in 'ABC'], float); c = V.mean(0); area = c + (V - c) * 1.35
ax.add_patch(Polygon([P(*p) for p in area], closed=True, fc=AREA, alpha=0.35, ec='none', zorder=2))
# triangle and out-and-back path (from C to D, E and back)
ax.add_patch(Polygon([P(*VERT[k]) for k in 'ABC'], closed=True, fc='none', ec=ARROW, lw=1.3, zorder=3))
ax.plot(*zip(*[P(*p) for p in [VERT['C'], OUT['D'], OUT['E'], VERT['C']]]), ls=(0, (3, 2)), color=ARROW, lw=0.9, zorder=3)
for k, (x, y) in {**VERT, **OUT}.items():
    ax.plot(*P(x, y), 'o' if k in VERT else 's', ms=3.5, color=ARROW, mec='white', mew=0.6, zorder=4)
    dx, dy = {'A': (30, 20), 'B': (-34, 16), 'C': (-34, -8), 'D': (32, -14), 'E': (-32, -14)}[k]
    ax.text(*P(x + dx, y + dy), k, fontsize=7, weight='bold', color=ARROW, ha='center', va='center', zorder=5,
            bbox=dict(boxstyle='circle,pad=0.15', fc=LABEL_FC, ec=LABEL_EC, lw=0.5))
# beacons: small circles and name/height tags
for k, ((x, y), h) in BEAC.items():
    ax.plot(*P(x, y), 'o', ms=6.5, mfc='none', mec=ARROW, mew=0.9, zorder=4)
tag(BEAC['B1'][0], 'B1 · 2.60 m', 'below')
tag(BEAC['B2'][0], 'B2 · 2.50 m', 'below')
tag(BEAC['B5'][0], 'B5 · 2.55 m', 'left')
tag(BEAC['B4'][0], 'B4 · 2.40 m', 'left')

# labels along the edges, aligned in rows
# beacons: one label, one curved arrow per beacon (arrows fan out without crossing)
label('Marvelmind stationary beacons', (760, 95),
      [((BEAC['B1'][0][0] + 14, BEAC['B1'][0][1] + 4), 0.05), ((BEAC['B2'][0][0] + 10, BEAC['B2'][0][1] - 12), 0.2),
       ((BEAC['B5'][0][0] + 4, BEAC['B5'][0][1] - 16), 0.1), ((BEAC['B4'][0][0] - 6, BEAC['B4'][0][1] - 15), -0.25)])
label('OptiTrack V120:Trio\n120 Hz, 1.93 m', (1265, 95), [((OPTI[0] + 30, OPTI[1] - 8), -0.3)])
label('Out-and-back stops D, E\noutside the OptiTrack area', (215, 640),
      [((OUT['E'][0] * 0.3 + OUT['D'][0] * 0.7, OUT['E'][1] * 0.3 + OUT['D'][1] * 0.7 - 6), -0.3)])   # lands between E and D, bowing away from E
label('Triangle A–B–C', (215, 1120), [((520, 960), -0.25)])
label('OptiTrack area ≈ 2.5 m × 2.5 m', (745, 1140), [(tuple(area[0] * 0.55 + area[1] * 0.45), 0.25)])
label('KELO robot: mobile beacon\nand OptiTrack rigid body', (905, 625), [((975, 775), 0.15)])

# zoom box on the robot
x0, y0, x1, y1 = ROBOT_BOX
ax.add_patch(Rectangle(P(x0, y0), (x1 - x0) * S, (y1 - y0) * S, fc='none', ec='0.25', lw=0.8, ls='--', zorder=4))

# 3D view of the rigid body (robot body frame); Motive exports markers Y-up: plan = (x, -z), height = y
ax3 = fig.add_axes(AX3_RECT, projection='3d')
ax3.set_facecolor('none')
for pane in (ax3.xaxis.pane, ax3.yaxis.pane, ax3.zaxis.pane):
    pane.set_facecolor((1, 1, 1, 0.6)); pane.set_edgecolor('0.8')
mk = list(csv.DictReader(open(MARKERS)))
MP = np.array([[float(r['x']), -float(r['z']), float(r['y'])] for r in mk]) * 100
low = MP[:, 2] < -5; hz = float(MP[low, 2].mean()); zf = hz - 12
for P_, m_, lab in [(MP[~low], 'o', 'mast markers (2 × 5)'), (MP[low], 's', 'lower markers (3)')]:
    ax3.scatter(P_[:, 0], P_[:, 1], P_[:, 2], marker=m_, s=12, facecolors='white', edgecolors=TEXT, linewidths=0.7, label=lab, depthshade=False)
    for q in P_: ax3.plot([q[0]] * 2, [q[1]] * 2, [zf, q[2]], color='0.75', lw=0.4)
ax3.scatter([0], [0], [0], marker='+', s=40, color=TEXT, label='rigid body pivot', depthshade=False)
for L, m_, col, lab in [(LEV1, 'v', ARROW, 'mobile beacon, mounting M1'), (LEV2, '^', '#3d7fb0', 'mobile beacon, mounting M2')]:
    ax3.scatter([L[0] * 100], [L[1] * 100], [hz], marker=m_, s=34, color=col, label=lab, depthshade=False)
    ax3.plot([L[0] * 100] * 2, [L[1] * 100] * 2, [zf, hz], color=col, lw=0.6, ls='--')
ax3.set_xlabel('x (cm)', labelpad=-7, fontsize=6); ax3.set_ylabel('y (cm)', labelpad=-7, fontsize=6); ax3.set_zlabel('z (cm)', labelpad=-8, fontsize=6)
ax3.tick_params(labelsize=5.3, pad=-3, colors=TEXT)
ax3.set_zlim(zf, 10); ax3.set_xlim(-25, 25); ax3.set_ylim(-22, 28); ax3.set_box_aspect((1.25, 1, 0.65) if VERTICAL else (1, 1, 0.6), zoom=1.2 if VERTICAL else 1.1); ax3.view_init(elev=20, azim=60)
if VERTICAL:
    ax3.legend(loc='upper center', bbox_to_anchor=(0.5, -0.06), ncol=3, fontsize=5.3, handletextpad=0.15, columnspacing=0.6,
               frameon=True, facecolor=LABEL_FC, edgecolor=LABEL_EC, borderpad=0.3)
    zoom = [((x0, y1), (0.17, 0.85)), ((x1, y1), (0.83, 0.85))]            # dashed lines down to the zoom
else:
    ax3.legend(loc='upper center', bbox_to_anchor=(0.5, 0.04), ncol=3, fontsize=5.6, handletextpad=0.2, columnspacing=0.7,
               frameon=True, facecolor=LABEL_FC, edgecolor=LABEL_EC)
    fig.text(0.545, 0.955, 'Robot: OptiTrack markers and mobile beacon', fontsize=7, weight='bold', ha='left', va='top', color=TEXT)
    zoom = [((x1, y0), (0.03, 0.8)), ((x1, y1), (0.03, 0.2))]
for (px, py), (qx, qy) in zoom:
    fig.add_artist(ConnectionPatch(xyA=P(px, py), coordsA=ax.transData, xyB=(qx, qy), coordsB=ax3.transAxes,
                                   color='0.45', lw=0.6, ls=(0, (3, 2))))

out = os.path.join(ROOT, 'figures', 'fig1_teaser.pdf' if VERTICAL else 'fig1_photo.pdf')
fig.savefig(out, dpi=300); fig.savefig(out.replace('.pdf', '.png'), dpi=200)
print('written', out)
