"""Routes window of the KELO control panel: pick stations on the map and run them in order."""
import base64
import json
import math
import os
import time
import tkinter as tk
from tkinter import messagebox, simpledialog, ttk

from kelo_bringup import route as rt

CANVAS_W, CANVAS_H = 760, 600
DRAG_PX = 12
PICK_PX = 14
STATUS_COLORS = {'RUNNING': '#2e7d32', 'DWELL': '#2e7d32', 'DONE': '#1565c0', 'DONE_WITH_SKIPS': '#b26a00',
                 'PAUSED': '#b26a00', 'WAITING': '#b26a00', 'FAILED': '#c62828', 'IDLE': '#555'}


class RouteWindow:
    def __init__(self, app, map_yaml):
        self.app, self.map_yaml = app, map_yaml
        self.win = tk.Toplevel(app.root)
        self.win.title('KELO Routes - stations visited one after the other')
        self.win.protocol('WM_DELETE_WINDOW', self.close)
        self.routes = rt.load_routes()
        self.name = tk.StringVar(value=next(iter(self.routes), 'route1'))
        self.stations = [dict(s) for s in self.routes.get(self.name.get(), {'stations': []})['stations']]
        self.dirty = False
        self.state, self.robot = {}, None
        self.press, self.preview = None, None
        self.map = None
        self.node = None
        self._build()
        self._load_map()
        self._connect()
        self._redraw()
        self._tick()

    # ----------------------------------------------------------------- ui
    def _build(self):
        left = ttk.Frame(self.win, padding=6)
        left.pack(side='left', fill='both', expand=True)
        self.canvas = tk.Canvas(left, width=CANVAS_W, height=CANVAS_H, bg='#808080', highlightthickness=0)
        self.canvas.pack(fill='both', expand=True)
        self.canvas.bind('<ButtonPress-1>', self._on_press)
        self.canvas.bind('<B1-Motion>', self._on_drag)
        self.canvas.bind('<ButtonRelease-1>', self._on_release)
        self.canvas.bind('<Button-3>', self._on_right)
        self.canvas.bind('<Configure>', lambda e: self._load_map())
        ttk.Label(left, foreground='#444', justify='left', text=(
            'Click on the map: add a station (after the selected one).  Click and drag: station with a heading.\n'
            'Ctrl+click: move the selected station there.  Right click: select a station.  Delete key: remove it.'
        )).pack(anchor='w', pady=(4, 0))

        right = ttk.Frame(self.win, padding=6)
        right.pack(side='right', fill='y')
        rbox = ttk.LabelFrame(right, text='Route', padding=6)
        rbox.pack(fill='x')
        self.combo = ttk.Combobox(rbox, textvariable=self.name, values=sorted(self.routes), width=22)
        self.combo.grid(row=0, column=0, columnspan=3, sticky='we')
        self.combo.bind('<<ComboboxSelected>>', lambda e: self._select_route())
        ttk.Button(rbox, text='New', command=self._new).grid(row=1, column=0, sticky='we', pady=2)
        ttk.Button(rbox, text='Save', command=self._save).grid(row=1, column=1, sticky='we', pady=2)
        ttk.Button(rbox, text='Delete', command=self._delete_route).grid(row=1, column=2, sticky='we', pady=2)

        sbox = ttk.LabelFrame(right, text='Stations (in this order)', padding=6)
        sbox.pack(fill='both', expand=True, pady=6)
        cols = ('n', 'name', 'xy', 'heading', 'mode')
        self.tree = ttk.Treeview(sbox, columns=cols, show='headings', height=12, selectmode='browse')
        for c, text, w in zip(cols, ('#', 'Name', 'x, y (m)', 'Heading', 'At the station'), (28, 80, 100, 64, 110)):
            self.tree.heading(c, text=text)
            self.tree.column(c, width=w, anchor='w')
        self.tree.pack(fill='both', expand=True)
        self.tree.bind('<<TreeviewSelect>>', lambda e: self._redraw(fill=False))
        self.tree.bind('<Double-1>', lambda e: self._edit())
        self.win.bind('<Delete>', lambda e: self._remove())
        row = ttk.Frame(sbox)
        row.pack(fill='x', pady=(4, 0))
        for text, cmd in (('Up', lambda: self._shift(-1)), ('Down', lambda: self._shift(1)), ('Edit', self._edit),
                          ('Remove', self._remove), ('Clear', self._clear)):
            ttk.Button(row, text=text, command=cmd, width=7).pack(side='left', padx=1)
        ttk.Button(sbox, text='Add the robot position', command=self._add_robot).pack(fill='x', pady=(4, 0))

        run = ttk.LabelFrame(right, text='Run', padding=6)
        run.pack(fill='x')
        lrow = ttk.Frame(run)
        lrow.pack(fill='x')
        ttk.Label(lrow, text='Laps (0 = until stopped):').pack(side='left')
        self.laps = tk.IntVar(value=1)
        ttk.Spinbox(lrow, from_=0, to=999, textvariable=self.laps, width=5).pack(side='left', padx=4)
        brow = ttk.Frame(run)
        brow.pack(fill='x', pady=4)
        for text, cmd in (('Start', self._start), ('Pause', lambda: self._cmd('pause')),
                          ('Resume', lambda: self._cmd('resume')), ('Skip', lambda: self._cmd('skip')),
                          ('Stop', lambda: self._cmd('stop'))):
            ttk.Button(brow, text=text, command=cmd, width=7).pack(side='left', padx=1)
        self.status = tk.Label(run, text='Route: -', anchor='w', justify='left', wraplength=380,
                               font=('', 10, 'bold'), fg='#555')
        self.status.pack(fill='x')
        self.hint = tk.Label(right, text='', anchor='w', justify='left', wraplength=380, fg='#b26a00')
        self.hint.pack(fill='x', pady=(6, 0))

    # ---------------------------------------------------------------- map
    def _load_map(self):
        import cv2
        try:
            gray, grid, info = rt.load_map_image(self.map_yaml)
        except (OSError, KeyError, ValueError) as exc:
            self.map = None
            self.hint.config(text=f'No map: {exc}. Build a map first (Start Mapping -> Save Map).')
            return
        h, w = gray.shape
        cw, ch = max(self.canvas.winfo_width(), 50), max(self.canvas.winfo_height(), 50)
        if cw < 60:
            cw, ch = CANVAS_W, CANVAS_H
        scale = min(cw / w, ch / h)
        img = cv2.resize(gray, (max(1, int(w * scale)), max(1, int(h * scale))), interpolation=cv2.INTER_NEAREST)
        ok, png = cv2.imencode('.png', img)
        self.photo = tk.PhotoImage(data=base64.b64encode(png.tobytes()).decode('ascii'))
        self.map = {'grid': grid, 'info': info, 'h': h, 'scale': scale}
        self._redraw()

    def _to_px(self, x, y):
        m = self.map
        return rt.world_to_pixel(x, y, m['info'], m['h'], m['scale'])

    def _to_world(self, u, v):
        m = self.map
        return rt.pixel_to_world(u, v, m['info'], m['h'], m['scale'])

    def _check(self, s):
        if self.map is None:
            return 'ok'
        return rt.map_check(self.map['grid'], self.map['info'], s['x'], s['y'], 0.30)

    # ------------------------------------------------------------ drawing
    def _selected(self):
        sel = self.tree.selection()
        return int(sel[0]) if sel else None

    def _redraw(self, fill=True):
        c = self.canvas
        c.delete('all')
        if self.map is not None:
            c.create_image(0, 0, image=self.photo, anchor='nw')
        if fill:
            self._fill_tree()
        if self.map is None:
            return
        pts = [self._to_px(s['x'], s['y']) for s in self.stations]
        if len(pts) > 1:
            c.create_line(*[v for p in pts for v in p], fill='#1e88e5', width=2, arrow='last', arrowshape=(10, 12, 4))
            if self.laps.get() != 1:
                c.create_line(*pts[-1], *pts[0], fill='#1e88e5', width=2, dash=(4, 3), arrow='last')
        sel = self._selected()
        target = self.state.get('index') if self.state.get('status') in ('RUNNING', 'DWELL', 'WAITING', 'PAUSED') \
            and self.state.get('route') == self.name.get() else None
        bad = {s['name'] for s in self.stations if self._check(s) != 'ok'}
        for k, (s, (u, v)) in enumerate(zip(self.stations, pts)):
            r = 11 if s['stop'] or s['wait_s'] else 7
            fill = '#c62828' if s['name'] in bad else ('#43a047' if k == target else ('#ffb300' if k == sel else '#1e88e5'))
            if s['yaw'] is not None:
                c.create_line(u, v, u + 26 * math.cos(s['yaw']), v - 26 * math.sin(s['yaw']), fill='#0d47a1',
                              width=3, arrow='last')
            c.create_oval(u - r, v - r, u + r, v + r, fill=fill, outline='white', width=2)
            c.create_text(u, v, text=str(k + 1), fill='white', font=('', 9, 'bold'))
            c.create_text(u + r + 4, v - r, text=s['name'], anchor='sw', fill='#111', font=('', 9))
        if self.robot is not None:
            x, y, yaw = self.robot
            u, v = self._to_px(x, y)
            pts = []
            for a, d in ((0.0, 16), (2.5, 11), (-2.5, 11)):
                pts += [u + d * math.cos(yaw + a), v - d * math.sin(yaw + a)]
            c.create_polygon(*pts, fill='#e53935', outline='white')
        if self.preview is not None:
            c.create_line(*self.preview, fill='#0d47a1', width=3, arrow='last')
        self.hint.config(text=('Red stations are on an obstacle, too close to one (< 0.3 m) or outside the mapped area: '
                               'the robot will skip them. ' if bad else '')
                         + ('Unsaved changes (Start saves them).' if self.dirty else ''))

    def _fill_tree(self):
        sel = self._selected()
        self.tree.delete(*self.tree.get_children())
        for k, s in enumerate(self.stations):
            heading = 'auto' if s['yaw'] is None else f'{math.degrees(s["yaw"]):.0f} deg'
            mode = 'pass through' if not s['stop'] and not s['wait_s'] else (
                f'stop, wait {s["wait_s"]:.0f} s' if s['wait_s'] else 'stop')
            self.tree.insert('', 'end', iid=str(k), values=(k + 1, s['name'], f'{s["x"]:.2f}, {s["y"]:.2f}', heading, mode))
        if sel is not None and sel < len(self.stations):
            self.tree.selection_set(str(sel))

    # ------------------------------------------------------- interaction
    def _nearest(self, u, v):
        best, bd = None, PICK_PX
        for k, s in enumerate(self.stations):
            su, sv = self._to_px(s['x'], s['y'])
            d = math.hypot(su - u, sv - v)
            if d < bd:
                best, bd = k, d
        return best

    def _on_press(self, e):
        if self.map is None:
            return
        self.press = (e.x, e.y, bool(e.state & 0x4))

    def _on_drag(self, e):
        if self.press is None:
            return
        u, v, _ = self.press
        self.preview = (u, v, e.x, e.y) if math.hypot(e.x - u, e.y - v) >= DRAG_PX else None
        self._redraw(fill=False)

    def _on_release(self, e):
        if self.press is None:
            return
        u, v, ctrl = self.press
        self.press, self.preview = None, None
        x, y = self._to_world(u, v)
        yaw = None
        if math.hypot(e.x - u, e.y - v) >= DRAG_PX:
            yaw = math.atan2(-(e.y - v), e.x - u)
        sel = self._selected()
        if ctrl and sel is not None:
            s = self.stations[sel]
            s.update(rt.station(s['name'], x, y, yaw if yaw is not None else s['yaw'], s['wait_s'], s['stop']))
        else:
            at = len(self.stations) if sel is None else sel + 1
            self.stations.insert(at, rt.station(self._free_name(), x, y, yaw))
            self._fill_tree()
            self.tree.selection_set(str(at))
        self.dirty = True
        self._redraw()

    def _on_right(self, e):
        k = self._nearest(e.x, e.y)
        if k is not None:
            self.tree.selection_set(str(k))
        else:
            self.tree.selection_remove(*self.tree.selection())
        self._redraw(fill=False)

    def _free_name(self):
        used = {s['name'] for s in self.stations}
        k = len(self.stations) + 1
        while f'S{k}' in used:
            k += 1
        return f'S{k}'

    def _shift(self, d):
        k = self._selected()
        if k is None or not 0 <= k + d < len(self.stations):
            return
        self.stations[k], self.stations[k + d] = self.stations[k + d], self.stations[k]
        self._fill_tree()
        self.tree.selection_set(str(k + d))
        self.dirty = True
        self._redraw()

    def _remove(self):
        k = self._selected()
        if k is None:
            return
        del self.stations[k]
        self._fill_tree()
        if self.stations:
            self.tree.selection_set(str(min(k, len(self.stations) - 1)))
        self.dirty = True
        self._redraw()

    def _clear(self):
        if self.stations and messagebox.askyesno('Clear', 'Remove all stations of this route?', parent=self.win):
            self.stations = []
            self.dirty = True
            self._redraw()

    def _add_robot(self):
        if self.robot is None:
            messagebox.showwarning('No position', 'The robot position is not known (navigation running and localized?)',
                                   parent=self.win)
            return
        x, y, yaw = self.robot
        k = self._selected()
        at = len(self.stations) if k is None else k + 1
        self.stations.insert(at, rt.station(self._free_name(), x, y, yaw))
        self._fill_tree()
        self.tree.selection_set(str(at))
        self.dirty = True
        self._redraw()

    def _edit(self):
        k = self._selected()
        if k is None:
            return
        s = self.stations[k]
        dlg = tk.Toplevel(self.win)
        dlg.title(f'Station {k + 1}')
        dlg.transient(self.win)
        name = tk.StringVar(value=s['name'])
        mode = tk.StringVar(value='stop' if s['stop'] or s['wait_s'] else 'pass')
        wait = tk.DoubleVar(value=s['wait_s'])
        auto = tk.BooleanVar(value=s['yaw'] is None)
        yaw = tk.DoubleVar(value=0.0 if s['yaw'] is None else round(math.degrees(s['yaw']), 1))
        grid = ttk.Frame(dlg, padding=10)
        grid.pack()
        ttk.Label(grid, text='Name').grid(row=0, column=0, sticky='w')
        ttk.Entry(grid, textvariable=name, width=18).grid(row=0, column=1, sticky='we')
        ttk.Radiobutton(grid, text='Stop here', variable=mode, value='stop').grid(row=1, column=0, sticky='w')
        ttk.Radiobutton(grid, text='Pass through without stopping', variable=mode, value='pass').grid(
            row=1, column=1, sticky='w')
        ttk.Label(grid, text='Wait at the station (s)').grid(row=2, column=0, sticky='w')
        ttk.Spinbox(grid, from_=0, to=3600, increment=1, textvariable=wait, width=8).grid(row=2, column=1, sticky='w')
        ttk.Checkbutton(grid, text='Heading: automatic (arrival direction)', variable=auto).grid(
            row=3, column=0, columnspan=2, sticky='w')
        ttk.Label(grid, text='Heading (deg, 0 = +x of the map)').grid(row=4, column=0, sticky='w')
        ttk.Entry(grid, textvariable=yaw, width=8).grid(row=4, column=1, sticky='w')

        def ok():
            try:
                w = max(0.0, float(wait.get()))
                y = None if auto.get() else math.radians(float(yaw.get()))
            except (tk.TclError, ValueError):
                messagebox.showerror('Station', 'Wait and heading must be numbers.', parent=dlg)
                return
            nm = name.get().strip() or s['name']
            if any(o['name'] == nm for i, o in enumerate(self.stations) if i != k):
                messagebox.showerror('Station', f'Another station is already called {nm}.', parent=dlg)
                return
            stop = mode.get() == 'stop' or w > 0.0
            self.stations[k] = rt.station(nm, s['x'], s['y'], y, w, stop)
            self.dirty = True
            dlg.destroy()
            self._redraw()
        row = ttk.Frame(grid)
        row.grid(row=5, column=0, columnspan=2, pady=(8, 0))
        ttk.Button(row, text='OK', command=ok).pack(side='left', padx=4)
        ttk.Button(row, text='Cancel', command=dlg.destroy).pack(side='left', padx=4)
        dlg.grab_set()

    # ------------------------------------------------------------ routes
    def _ask_discard(self):
        return not self.dirty or messagebox.askyesno('Unsaved changes', 'Discard the changes of this route?',
                                                      parent=self.win)

    def _select_route(self):
        name = self.name.get()
        if not self._ask_discard():
            return
        self.stations = [dict(s) for s in self.routes.get(name, {'stations': []})['stations']]
        self.dirty = False
        self._redraw()

    def _new(self):
        if not self._ask_discard():
            return
        name = simpledialog.askstring('New route', 'Route name (e.g. room1):', parent=self.win)
        if not name:
            return
        self.name.set(name.strip())
        self.stations, self.dirty = [], True
        self._redraw()

    def _save(self):
        name = self.name.get().strip()
        if not name:
            messagebox.showerror('Save', 'Give the route a name.', parent=self.win)
            return False
        names = [s['name'] for s in self.stations]
        if len(set(names)) != len(names):
            messagebox.showerror('Save', 'Station names must be different.', parent=self.win)
            return False
        self.routes = rt.load_routes()
        self.routes[name] = {'stations': [dict(s) for s in self.stations]}
        try:
            rt.save_routes(self.routes)
        except OSError as exc:
            messagebox.showerror('Save', f'Could not save {rt.routes_file()}: {exc}', parent=self.win)
            return False
        self.combo.config(values=sorted(self.routes))
        self.dirty = False
        self._redraw()
        return True

    def _delete_route(self):
        name = self.name.get()
        if name not in self.routes or not messagebox.askyesno('Delete', f'Delete route {name}?', parent=self.win):
            return
        self.routes.pop(name)
        rt.save_routes(self.routes)
        self.combo.config(values=sorted(self.routes))
        self.name.set(next(iter(self.routes), 'route1'))
        self.stations = [dict(s) for s in self.routes.get(self.name.get(), {'stations': []})['stations']]
        self.dirty = False
        self._redraw()

    # --------------------------------------------------------------- ros
    def _connect(self):
        if not self.app._ensure_drive_pub():
            self.status.config(text='Route: ROS not available')
            return
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import String
        self.node = self.app.ros_node
        self._String = String
        self.cmd_pub = self.node.create_publisher(String, '/kelo/route/command', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subs = [
            self.node.create_subscription(String, '/kelo/route/state', self._on_state, latched),
            self.node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_pose, latched),
        ]

    def _on_state(self, msg):
        try:
            self.state = json.loads(msg.data)
            self.state_t = time.time()
        except ValueError:
            pass

    def _on_pose(self, msg):
        p = msg.pose.pose
        q = p.orientation
        self.robot = (p.position.x, p.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def _cmd(self, cmd, **extra):
        if self.node is None:
            messagebox.showwarning('Route', 'ROS is not connected.', parent=self.win)
            return False
        if self.cmd_pub.get_subscription_count() == 0:
            messagebox.showwarning('Route', 'The route runner is not running: start navigation first.', parent=self.win)
            return False
        self.cmd_pub.publish(self._String(data=json.dumps(dict(cmd=cmd, **extra))))
        return True

    def _start(self):
        if not self.stations:
            messagebox.showwarning('Route', 'Add stations first (click on the map).', parent=self.win)
            return
        if self.dirty and not self._save():
            return
        bad = [s['name'] for s in self.stations if self._check(s) != 'ok']
        if bad and not messagebox.askyesno('Route', f'Stations {", ".join(bad)} are on an obstacle or outside the map '
                                           'and will be skipped. Start anyway?', parent=self.win):
            return
        try:
            laps = max(0, int(self.laps.get()))
        except (tk.TclError, ValueError):
            laps = 1
        self._cmd('start', route=self.name.get().strip(), laps=laps)

    def _tick(self):
        if not self.win.winfo_exists():
            return
        if self.node is not None:
            for _ in range(30):
                try:
                    self.app._rclpy.spin_once(self.node, timeout_sec=0.0)
                except Exception:
                    break
        st = self.state
        if st:
            laps = st.get('laps')
            where = ''
            if st.get('route'):
                where = (f'\n{st["route"]}: station {st.get("index", 0) + 1}/{st.get("stations")} '
                         f'"{st.get("station", "")}", lap {st.get("lap", 1)}/{laps if laps else "inf"}')
                if st.get('remaining_m') is not None and st.get('status') == 'RUNNING':
                    where += f', {st["remaining_m"]:.1f} m left'
            if st.get('skipped'):
                where += f'\nskipped: {", ".join(st["skipped"])}'
            self.status.config(text=f'Route: {st.get("status")}  {st.get("message", "")}{where}',
                               fg=STATUS_COLORS.get(st.get('status'), '#555'))
        self._redraw(fill=False)
        self.win.after(300, self._tick)

    def close(self):
        if self.dirty and not messagebox.askyesno('Unsaved changes', 'Close without saving the route?', parent=self.win):
            return
        if self.node is not None:
            for s in getattr(self, 'subs', []):
                self.node.destroy_subscription(s)
            self.node.destroy_publisher(self.cmd_pub)
        self.app.route_win = None
        self.win.destroy()
