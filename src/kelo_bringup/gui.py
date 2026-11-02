import os
import queue
import signal
import subprocess
import threading
import time
import tkinter as tk
from tkinter import messagebox, simpledialog, scrolledtext, ttk

import yaml

try:
    from kelo_bringup.common import MAX_SPEED_RANGE
except ImportError:  # workspace not sourced
    MAX_SPEED_RANGE = (0.05, 0.30)

PACKAGE = "kelo_bringup"
DATA_DIR = os.environ.get("KELO_DATA_DIR", "/data")
MAP_YAML = os.path.join(DATA_DIR, "maps", "current.yaml")
# settings changed in the panel are written here and merged over the packaged hardware.yaml
OVERRIDE_YAML = os.path.join(DATA_DIR, "hardware.yaml")
# (hardware.yaml key path, label): read by the stack at start, so a change applies at the next start
FEATURES = (
    (("perception", "detector"), "Object names (YOLOX)"),
    (("camera", "color"), "Colour camera"),
    (("navigation", "people_aware"), "People-aware navigation"),
    (("mapping", "camera_layer"), "Camera map layer"),
    (("views", "enabled"), "Driver views"),
    (("web", "enabled"), "Web view (:8080)"),
)

SOURCES = ("sim", "hardware", "replay")
DRIVE_LINEAR = 0.30
DRIVE_ANGULAR = 0.80
VY_MAX = 0.30  # velocity_smoother max_velocity[1] in nav2_params.yaml
LOG_MAX_LINES = 5000


class KeloGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("KELO Control Panel")
        self.root.geometry("860x760-0+0")

        self.procs = {}
        self.ros_node = None
        self.cmd_pub = None
        self.pressed_keys = set()
        self.release_ticks = 0
        self.recording = False

        self.window_procs = {}
        # log lines and UI updates from the worker threads; only the Tk thread touches the widgets
        self.ui_queue = queue.Queue()
        self._build_widgets()
        self._drain_ui_queue()
        self._refresh_status()
        self._drive_tick()
        self._window_watchdog()

    def _build_widgets(self):
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill="x")

        ttk.Label(top, text="KELO", font=("", 16, "bold")).pack(anchor="w")
        self.status_var = tk.StringVar(value="State: Stopped")
        ttk.Label(top, textvariable=self.status_var, font=("", 11)).pack(anchor="w", pady=(2, 8))

        opts = ttk.LabelFrame(top, text="Source", padding=8)
        opts.pack(fill="x", pady=4)
        self.source = tk.StringVar(value=os.environ.get("KELO_GUI_SOURCE", "sim"))
        self.odom_source = tk.StringVar(value="robot")
        self.bag_path = tk.StringVar(value="")
        self.use_rviz = tk.BooleanVar(value=True)
        self.use_camera_windows = tk.BooleanVar(value=True)
        self.sim_person = tk.BooleanVar(value=True)
        ttk.Label(opts, text="sensor source").pack(side="left")
        ttk.Combobox(opts, textvariable=self.source, values=SOURCES, width=9,
                     state="readonly").pack(side="left", padx=4)
        ttk.Label(opts, text="odometry").pack(side="left", padx=(8, 0))
        ttk.Combobox(opts, textvariable=self.odom_source, values=("robot", "lidar"), width=6,
                     state="readonly").pack(side="left", padx=4)
        ttk.Label(opts, text="bag (replay)").pack(side="left", padx=(8, 0))
        ttk.Entry(opts, textvariable=self.bag_path, width=26).pack(side="left", padx=4)

        feats = ttk.LabelFrame(top, text="Features (saved in /data/hardware.yaml, applied at the next start)",
                               padding=8)
        feats.pack(fill="x", pady=4)
        hw = self._load_hardware()
        self.features = {}
        for i, (path, label) in enumerate(FEATURES):
            var = tk.BooleanVar(value=bool(self._get(hw, path, True)))
            ttk.Checkbutton(feats, text=label, variable=var,
                            command=lambda p=path, v=var: self._feature_changed(p, v)).grid(
                row=i // 3, column=i % 3, sticky="w", padx=6, pady=1)
            self.features[path] = var
        row = (len(FEATURES) + 2) // 3
        ttk.Label(feats, text="Windows:").grid(row=row, column=0, sticky="w", padx=6, pady=(6, 0))
        wins = ttk.Frame(feats)
        wins.grid(row=row, column=1, columnspan=2, sticky="w", pady=(6, 0))
        ttk.Checkbutton(wins, text="RViz", variable=self.use_rviz).pack(side="left", padx=6)
        ttk.Checkbutton(wins, text="Camera / Surround windows", variable=self.use_camera_windows).pack(
            side="left", padx=6)
        ttk.Button(wins, text="Restart stack (apply)", command=self.restart_stack).pack(side="left", padx=10)

        slam_btns = ttk.LabelFrame(top, text="Mapping (slam_toolbox)", padding=8)
        slam_btns.pack(fill="x", pady=6)
        ttk.Button(slam_btns, text="Start Mapping", command=self.start_mapping).pack(side="left", padx=4)
        ttk.Button(slam_btns, text="Save Map", command=self.save_map).pack(side="left", padx=4)
        ttk.Button(slam_btns, text="Stop Mapping", command=lambda: self.stop("mapping")).pack(side="left", padx=4)

        nav_btns = ttk.LabelFrame(top, text="Navigation (AMCL + Nav2)", padding=8)
        nav_btns.pack(fill="x", pady=6)
        ttk.Button(nav_btns, text="Start Navigation", command=self.start_navigation).pack(side="left", padx=4)
        ttk.Button(nav_btns, text="Set Initial Pose", command=self.initial_pose_dialog).pack(side="left", padx=4)
        ttk.Button(nav_btns, text="Localize", command=self.localize).pack(side="left", padx=4)
        ttk.Button(nav_btns, text="Send Goal", command=self.send_goal_dialog).pack(side="left", padx=4)
        ttk.Button(nav_btns, text="Stop Navigation", command=lambda: self.stop("navigation")).pack(side="left", padx=4)
        ttk.Checkbutton(nav_btns, text="Person on path (sim)", variable=self.sim_person).pack(side="left", padx=6)

        speed = ttk.LabelFrame(top, text="Top speed (navigation; slower near walls and people)", padding=8)
        speed.pack(fill="x", pady=6)
        lo, hi = (round(v * 100) for v in MAX_SPEED_RANGE)
        self.max_speed = tk.IntVar(value=round(self._get(hw, ("navigation", "max_speed"), 0.25) * 100))
        self.max_speed.set(min(max(self.max_speed.get(), lo), hi))
        self.speed_label = ttk.Label(speed, width=8)
        scale = ttk.Scale(speed, from_=lo, to=hi, orient="horizontal", length=320,
                          command=lambda v: self._speed_moved(float(v)))
        scale.set(self.max_speed.get())
        scale.pack(side="left", padx=4)
        scale.bind("<ButtonRelease-1>", lambda _e: self._speed_changed())
        self.speed_label.pack(side="left", padx=4)
        self._speed_moved(self.max_speed.get())
        ttk.Label(speed, text=f"{lo}-{hi} cm/s, applied at once and saved", foreground="#555").pack(
            side="left", padx=10)

        video = ttk.LabelFrame(top, text="Video (driver views -> /data/videos)", padding=8)
        video.pack(fill="x", pady=6)
        self.rec_btn = ttk.Button(video, text="Record", command=self.toggle_record)
        self.rec_btn.pack(side="left", padx=4)
        ttk.Button(video, text="Save Clip (dashcam)", command=lambda: self._recorder("save_clip")).pack(side="left", padx=4)
        ttk.Label(video, text="Phone/tablet: http://<robot>:8080 (user kelo, KELO_WEB_PASSWORD)",
                  foreground="#555").pack(side="left", padx=10)

        self.drive_info = ttk.Label(
            top, text="Driving: WASD or arrow keys while this window has focus (works in every mode, speed limited).",
            foreground="#555")
        self.drive_info.pack(anchor="w", pady=(4, 4))

        log_frame = ttk.LabelFrame(self.root, text="Live Log", padding=6)
        log_frame.pack(fill="both", expand=True, padx=10, pady=(0, 10))
        self.log_box = scrolledtext.ScrolledText(log_frame, state="disabled", wrap="word", height=22)
        self.log_box.pack(fill="both", expand=True)

        self.root.bind("<KeyPress>", self._on_key_press)
        self.root.bind("<KeyRelease>", self._on_key_release)
        self.root.focus_set()

    def _log(self, text):
        """Safe from any thread: the text is shown by the Tk thread (_drain_ui_queue)."""
        self.ui_queue.put(text)

    def _drain_ui_queue(self):
        texts = []
        try:
            while True:
                item = self.ui_queue.get_nowait()
                if callable(item):
                    item()
                else:
                    texts.append(item)
        except queue.Empty:
            pass
        if texts:
            self.log_box.configure(state="normal")
            self.log_box.insert("end", "".join(texts))
            lines = int(self.log_box.index("end-1c").split(".")[0])
            if lines > LOG_MAX_LINES:  # the stacks log a lot: keep the panel fast over long runs
                self.log_box.delete("1.0", f"{lines - LOG_MAX_LINES + 1}.0")
            self.log_box.see("end")
            self.log_box.configure(state="disabled")
        self.root.after(100, self._drain_ui_queue)

    @staticmethod
    def _load_hardware():
        try:
            from kelo_bringup.common import load_hardware
            return load_hardware()
        except Exception:
            return {}

    @staticmethod
    def _get(cfg, path, default):
        for key in path:
            if not isinstance(cfg, dict) or key not in cfg:
                return default
            cfg = cfg[key]
        return cfg

    @staticmethod
    def _save_override(path, value):
        over = {}
        if os.path.isfile(OVERRIDE_YAML):
            with open(OVERRIDE_YAML, encoding="utf-8") as f:
                over = yaml.safe_load(f) or {}
        node = over
        for key in path[:-1]:
            if not isinstance(node.get(key), dict):
                node[key] = {}
            node = node[key]
        node[path[-1]] = value
        tmp = OVERRIDE_YAML + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            f.write("# local settings of this robot, merged over the packaged hardware.yaml (KELO panel)\n")
            yaml.safe_dump(over, f, default_flow_style=False, sort_keys=False)
        os.replace(tmp, OVERRIDE_YAML)

    def _feature_changed(self, path, var):
        try:
            self._save_override(path, bool(var.get()))
        except Exception as e:
            self._log(f"[GUI] Setting could not be saved: {e}\n")
            var.set(not var.get())
            return
        state = "on" if var.get() else "off"
        if self._running("mapping") or self._running("navigation"):
            self._log(f"[GUI] {'.'.join(path)} {state}: saved; press 'Restart stack (apply)' to use it now.\n")
        else:
            self._log(f"[GUI] {'.'.join(path)} {state}: saved; used at the next start.\n")

    def _speed_moved(self, value):
        self.max_speed.set(int(round(value)))
        self.speed_label.config(text=f"{self.max_speed.get()} cm/s")

    def _speed_changed(self):
        v = self.max_speed.get() / 100.0
        try:
            self._save_override(("navigation", "max_speed"), v)
        except Exception as e:
            self._log(f"[GUI] Speed could not be saved: {e}\n")
            return
        if not (self._running("mapping") or self._running("navigation")):
            self._log(f"[GUI] Top speed {v:.2f} m/s: saved; used at the next start.\n")
            return
        self._log(f"[GUI] Top speed {v:.2f} m/s: saved, applying to the running stack...\n")
        threading.Thread(target=self._apply_speed, args=(v,), daemon=True).start()

    def _apply_speed(self, v):
        # same values as common.nav2_params writes at start: the controller plans at v, the smoother caps at v
        cmds = [["ros2", "param", "set", "/controller_server", "FollowPath.vx_max", str(v)],
                ["ros2", "param", "set", "/velocity_smoother", "max_velocity", f"[{v}, {VY_MAX}, {DRIVE_ANGULAR}]"],
                ["ros2", "param", "set", "/velocity_smoother", "min_velocity", f"[{-v}, {-VY_MAX}, {-DRIVE_ANGULAR}]"]]
        for cmd in cmds:
            try:
                out = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
                text = (out.stdout + out.stderr).strip()
            except Exception as e:
                text = f"ERROR: {e}"
            self._log(f"[GUI] {cmd[3]} {cmd[4]}: {text}\n")

    def restart_stack(self):
        mode = "navigation" if self._running("navigation") else "mapping" if self._running("mapping") else None
        if mode is None:
            self._log("[GUI] No stack running; the settings are used at the next start.\n")
            return
        if mode == "mapping" and not messagebox.askyesno(
                "Restart mapping", "Restarting mapping starts a new map: what was mapped and not saved\n"
                "with 'Save Map' is lost.\n\nRestart anyway?", default="no"):
            return
        self._log(f"[GUI] Restarting {mode} with the current settings...\n")
        self.stop(mode)
        if mode == "navigation":
            self.start_navigation()
        else:
            self.start_mapping()

    def _refresh_status(self):
        running = [n for n, p in self.procs.items() if p.poll() is None]
        self.status_var.set("State: Running -> " + ", ".join(running) if running else "State: Stopped")
        self.root.after(1000, self._refresh_status)

    def _launch_args(self, mode):
        src = self.source.get()
        args = [f"source:={src}", f"mode:={mode}", f"odom_source:={self.odom_source.get()}",
                "use_rviz:=false"]
        if src == "replay":
            bag = self.bag_path.get().strip()
            if not bag:
                messagebox.showwarning("No bag", "enter a bag directory for replay (e.g. /data/bags/map_...)")
                return None
            args.append(f"bag:={bag}")
        return args

    def start_mapping(self):
        if self._running("navigation"):
            self._log("[GUI] Stopping navigation (before mapping)...\n")
            self.stop("navigation")
        args = self._launch_args("mapping")
        if args is not None:
            self._start("mapping", ["ros2", "launch", PACKAGE, "bringup.launch.py"] + args)

    def save_map(self):
        name = simpledialog.askstring("Save Map", "Map name (e.g. warehouse):",
                                      parent=self.root, initialvalue="map")
        if not name:
            return
        cmd = ["ros2", "run", PACKAGE, "save_map", name]
        self._log(f"\n[GUI] Saving map: {' '.join(cmd)}\n")
        try:
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            self._log(out.stdout + out.stderr)
        except Exception as e:
            self._log(f"[GUI] ERROR: {e}\n")

    def start_navigation(self):
        if not os.path.isfile(MAP_YAML):
            messagebox.showwarning(
                "No map", f"Build a map first (Start Mapping -> Save Map).\nExpected: {MAP_YAML}")
            return
        if self._running("mapping"):
            self._log("[GUI] Stopping mapping (before navigation)...\n")
            self.stop("mapping")
        args = self._launch_args("navigation")
        if args is None:
            return
        if not self._start("navigation", ["ros2", "launch", PACKAGE, "bringup.launch.py",
                                          f"map:={MAP_YAML}"] + args):
            return
        if self.source.get() == "sim" and self.sim_person.get():
            # test helper: for every goal a person steps onto the path in front of the robot
            self._start("person", ["ros2", "run", PACKAGE, "sim_person"])
        self._log("[GUI] A start pose is needed: 'Set Initial Pose' (x, y, yaw) or 'Localize' "
                  "(Marvelmind position or global search + turn in place).\n")

    LOCALIZE_ALREADY = 3

    def localize(self, force=False):
        if self._running("localize"):
            self._log("[GUI] Localization already running.\n")
            return
        cmd = ["ros2", "run", PACKAGE, "localize"] + (["--force"] if force else [])
        self._log(f"\n[GUI] Localization: {' '.join(cmd)}\n")
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        self.procs["localize"] = proc
        threading.Thread(target=self._stream_output, args=("localize", proc), daemon=True).start()
        if not force:
            self._watch_localize(proc)

    def _watch_localize(self, proc):
        code = proc.poll()
        if code is None:
            self.root.after(500, lambda: self._watch_localize(proc))
        elif code == self.LOCALIZE_ALREADY:
            if messagebox.askyesno(
                    "Already localized",
                    "The robot is already localized (low AMCL uncertainty); the pose estimate was NOT changed.\n\n"
                    "Re-localize from scratch anyway?\n"
                    "(The current estimate is discarded; without Marvelmind: global search over the whole map + turn in place.)",
                    default="no"):
                self.localize(force=True)

    def initial_pose_dialog(self):
        x = simpledialog.askstring("Start X", "Robot X on the map (metres)\n"
                                   "(empty = Marvelmind position):", parent=self.root, initialvalue="")
        if x is None:
            return
        y = ""
        if x.strip():
            y = simpledialog.askstring("Start Y", "Y (metres):", parent=self.root, initialvalue="0")
            if y is None:
                return
        yaw = simpledialog.askfloat("Start Yaw", "Direction the robot faces, yaw (degrees, map +x = 0,\n"
                                    "counter-clockwise positive):", parent=self.root, initialvalue=0.0)
        if yaw is None:
            return
        cmd = ["ros2", "run", PACKAGE, "localize", "--force", "--yaw", str(yaw)]
        if x.strip():
            cmd += ["--x", x.strip(), "--y", (y or "0").strip()]
        self._log(f"\n[GUI] Start pose: {' '.join(cmd)}\n")
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        threading.Thread(target=self._stream_output, args=("localize", proc), daemon=True).start()

    def send_goal_dialog(self):
        x = simpledialog.askfloat("Goal X", "Goal X (metres, map frame):", parent=self.root)
        if x is None:
            return
        y = simpledialog.askfloat("Goal Y", "Goal Y (metres, map frame):", parent=self.root)
        if y is None:
            return
        yaw = simpledialog.askfloat("Goal Yaw", "Yaw (degrees, empty = 0):",
                                    parent=self.root, initialvalue=0.0) or 0.0
        cmd = ["ros2", "run", PACKAGE, "send_goal", str(x), str(y), str(yaw)]
        self._log(f"\n[GUI] Sending goal: {' '.join(cmd)}\n")
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        threading.Thread(target=self._stream_output, args=("goal", proc), daemon=True).start()

    def toggle_record(self):
        self._recorder("stop" if self.recording else "start")

    def _recorder(self, action):
        def run():
            cmd = ["ros2", "service", "call", f"/video_recorder/{action}", "std_srvs/srv/Trigger"]
            try:
                out = subprocess.run(cmd, capture_output=True, text=True, timeout=15).stdout
            except Exception as e:
                out = str(e)
            msg = out.split("message=")[-1].rstrip(")\n '\"") if "message=" in out else "video recorder not running"
            ok = "success=True" in out
            if action in ("start", "stop") and ok:
                self.recording = action == "start"
                self.ui_queue.put(lambda: self.rec_btn.configure(
                    text="Stop Recording" if self.recording else "Record"))
            self._log(f"[GUI] video {action}: {msg}\n")
        threading.Thread(target=run, daemon=True).start()

    def _window_cmds(self, key):
        if key == "rviz":
            from ament_index_python.packages import get_package_share_directory
            cfg = os.path.join(get_package_share_directory(PACKAGE), "rviz", "kelo.rviz")
            # RViz and its helper (camera layer beside the map)
            return [["ros2", "run", "rviz2", "rviz2", "-d", cfg], ["ros2", "run", PACKAGE, "map_side_view"]]
        # KELO Camera and KELO Surround windows; the views themselves run in the stack
        return [["ros2", "run", PACKAGE, "camera_view"]]

    def _window_watchdog(self):
        """RViz and the camera windows follow their check boxes while a stack runs; a window that was
        closed or crashed is reopened (the stack is not affected)."""
        stack_up = self._running("mapping") or self._running("navigation")
        for key, var in (("rviz", self.use_rviz), ("camera", self.use_camera_windows)):
            want = var.get() and stack_up
            procs = self.window_procs.get(key, [])
            if procs and any(p.poll() is not None for p in procs):
                for p in procs:
                    if p.poll() is None:
                        self._kill_group(p)
                procs = []
                if want:
                    self._log(f"[GUI] {key} window closed; the stack was NOT affected. Reopening.\n")
            if want and not procs:
                try:
                    procs = [subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                              preexec_fn=os.setsid) for cmd in self._window_cmds(key)]
                except Exception as e:
                    self._log(f"[GUI] {key} could not be started: {e}\n")
                    var.set(False)
            elif not want and procs:
                for p in procs:
                    self._kill_group(p)
                procs = []
            self.window_procs[key] = procs
        self.root.after(2000, self._window_watchdog)

    @staticmethod
    def _kill_group(proc):
        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGINT)
            for _ in range(30):
                if proc.poll() is not None:
                    return
                time.sleep(0.1)
            os.killpg(pgid, signal.SIGTERM)
        except ProcessLookupError:
            pass

    def _foreign_stack(self):
        if self._running("mapping") or self._running("navigation"):
            return False
        # without the ros2 daemon (its cache still lists a stack stopped a moment ago), and only when the
        # gate is still there after a few seconds: nodes of a stopped stack can linger briefly in discovery
        for attempt in range(3):
            if attempt:
                time.sleep(1.0)
            try:
                out = subprocess.run(["ros2", "node", "list", "--no-daemon", "--spin-time", "1"],
                                     capture_output=True, text=True, timeout=10).stdout
            except Exception:
                return False
            if "/cmd_vel_gate" not in out.split():
                return False
        return True

    def _running(self, name):
        return name in self.procs and self.procs[name].poll() is None

    def _start(self, name, cmd):
        """Start a process group; False when it was not started (already running, other stack, error)."""
        if name in self.procs and self.procs[name].poll() is None:
            self._log(f"[GUI] '{name}' is already running.\n")
            return False
        if name in ("mapping", "navigation") and self._foreign_stack():
            messagebox.showwarning(
                "Another stack is running",
                "A KELO stack that this panel did not start is running (cmd_vel_gate exists).\n"
                "Two stacks cannot run at once (two gates, two sim robots). Stop it first\n"
                "(restart the container or run 'docker compose restart').")
            return False
        self._log(f"\n[GUI] Starting ({name}): {' '.join(cmd)}\n")
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, bufsize=1, preexec_fn=os.setsid)
        except FileNotFoundError:
            self._log("[GUI] ERROR: 'ros2' not found.\n")
            return False
        self.procs[name] = proc
        threading.Thread(target=self._stream_output, args=(name, proc), daemon=True).start()
        return True

    def _stream_output(self, name, proc):
        for line in proc.stdout:
            self._log(f"[{name}] {line}")
        self._log(f"[GUI] '{name}' ended.\n")

    def stop(self, group):
        names = [group] + (["person"] if group == "navigation" else [])
        if any(self._running(n) for n in names):
            self._log(f"[GUI] Stopping '{group}': sending SIGINT to the nodes "
                      "(this is the 'exit code -2' in the logs).\n")
        for name in names:
            proc = self.procs.get(name)
            if proc is None or proc.poll() is not None:
                continue
            try:
                pgid = os.getpgid(proc.pid)
                os.killpg(pgid, signal.SIGINT)
                for _ in range(50):
                    if proc.poll() is not None:
                        break
                    time.sleep(0.1)
                if proc.poll() is None:
                    os.killpg(pgid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            self._log(f"[GUI] '{name}' stopped.\n")

    def _ensure_drive_pub(self):
        if self.cmd_pub is not None:
            return True
        try:
            import rclpy
            from geometry_msgs.msg import Twist
            if not rclpy.ok():
                rclpy.init()
            self._rclpy = rclpy
            self._Twist = Twist
            self.ros_node = rclpy.create_node("kelo_gui_teleop")
            self.cmd_pub = self.ros_node.create_publisher(Twist, "/cmd_vel_teleop", 10)
            self._log("[GUI] Drive publisher (/cmd_vel_teleop) ready.\n")
            return True
        except Exception as e:
            self._log(f"[GUI] Drive publisher could not be started: {e}\n")
            return False

    DRIVE_KEYS = {"w", "a", "s", "d", "up", "down", "left", "right"}

    def _on_key_press(self, event):
        key = event.keysym.lower()
        if key in self.DRIVE_KEYS:
            self.pressed_keys.add(key)

    def _on_key_release(self, event):
        self.pressed_keys.discard(event.keysym.lower())

    def _drive_tick(self):
        if self.pressed_keys and self._ensure_drive_pub():
            lin = DRIVE_LINEAR if ("w" in self.pressed_keys or "up" in self.pressed_keys) else 0.0
            lin -= DRIVE_LINEAR if ("s" in self.pressed_keys or "down" in self.pressed_keys) else 0.0
            ang = DRIVE_ANGULAR if ("a" in self.pressed_keys or "left" in self.pressed_keys) else 0.0
            ang -= DRIVE_ANGULAR if ("d" in self.pressed_keys or "right" in self.pressed_keys) else 0.0
            msg = self._Twist()
            msg.linear.x, msg.angular.z = lin, ang
            self.cmd_pub.publish(msg)
            self.release_ticks = 5
        elif self.cmd_pub is not None and self.release_ticks > 0:
            # zero for 0.5 s after the keys are released, then silent: another driver (the web
            # view joystick) can use the same topic
            self.cmd_pub.publish(self._Twist())
            self.release_ticks -= 1
        self.root.after(100, self._drive_tick)

    def on_close(self):
        self.use_rviz.set(False)
        self.use_camera_windows.set(False)
        for procs in self.window_procs.values():
            for p in procs:
                if p.poll() is None:
                    self._kill_group(p)
        for name in list(self.procs.keys()):
            self.stop(name)
        if self.ros_node is not None:
            try:
                self.ros_node.destroy_node()
                self._rclpy.shutdown()
            except Exception:
                pass
        self.root.destroy()


def main():
    root = tk.Tk()
    app = KeloGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
