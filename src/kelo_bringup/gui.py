import json
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
    (("views", "depth"), "Depth image view"),
    (("navigation", "auto_localize"), "Localize by itself at start"),
    (("navigation", "auto_relocalize"), "Re-localize by itself when LOST"),
)
# features that are off when hardware.yaml does not name them
FEATURE_OFF_BY_DEFAULT = {("views", "depth")}

SOURCES = ("sim", "hardware", "replay")
# robot Ethernet: when it comes up the panel pins DDS to it and looks for the robot driver
ROBOT_IFACE = os.environ.get("KELO_DDS_IFACE", "enP8p1s0")
DDS_PIN = '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="{}"/></Interfaces></General></Domain></CycloneDDS>'

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
        self.stop_off = False
        self.stop_off_pub = None
        self.nav_status = None          # (time, text) from nav_watch (/kelo/nav_status)
        self.nav_sub = None
        self.health = None              # (time, dict) from health_monitor (/kelo/health)
        self.health_sub = None
        self.nodes_win = None
        self.auto_retry = None

        self.window_procs = {}
        self.iface_up = None
        self.discovering = False
        self.proposal = []
        # log lines and UI updates from the worker threads; only the Tk thread touches the widgets
        self.ui_queue = queue.Queue()
        self._build_widgets()
        # tall enough for every section and the log, within the screen
        self.root.update_idletasks()
        height = min(self.root.winfo_reqheight(), self.root.winfo_screenheight() - 60)
        self.root.geometry(f"860x{height}-0+0")
        self._drain_ui_queue()
        self._refresh_status()
        self._drive_tick()
        self._stop_off_tick()
        self._window_watchdog()
        self._watch_iface()

    def _build_widgets(self):
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill="x")

        ttk.Label(top, text="KELO", font=("", 16, "bold")).pack(anchor="w")
        self.status_var = tk.StringVar(value="State: Stopped")
        ttk.Label(top, textvariable=self.status_var, font=("", 11)).pack(anchor="w", pady=(2, 0))
        # nav_watch: READY / NOT READY: why / WAITING: why / DRIVING
        self.robot_state = tk.Label(top, text="Robot: -", font=("", 12, "bold"), anchor="w", fg="#555")
        self.robot_state.pack(anchor="w", fill="x", pady=(2, 0))
        # health_monitor: crashed / restarted nodes and the Jetson load
        hrow = ttk.Frame(top)
        hrow.pack(anchor="w", fill="x", pady=(0, 8))
        self.health_state = tk.Label(hrow, text="Health: -", font=("", 10), anchor="w", fg="#555",
                                     justify="left", wraplength=620)
        self.health_state.pack(side="left", fill="x", expand=True)
        ttk.Button(hrow, text="Nodes", command=self.show_nodes).pack(side="right")

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

        robot = ttk.LabelFrame(top, text=f"Robot (driver discovery; starts by itself when {ROBOT_IFACE} comes up)",
                               padding=8)
        robot.pack(fill="x", pady=4)
        self.find_btn = ttk.Button(robot, text="Find robot", command=self.find_robot)
        self.find_btn.pack(side="left", padx=4)
        self.apply_btn = ttk.Button(robot, text="Apply settings", command=self.apply_proposal, state="disabled")
        self.apply_btn.pack(side="left", padx=4)
        # automatic: cable up -> find the robot -> apply its settings -> start navigation (map needed);
        # nav_watch then localizes by itself and reports READY
        self.auto = tk.BooleanVar(value=os.environ.get("KELO_GUI_AUTO", "1") == "1")
        ttk.Checkbutton(robot, text="Automatic", variable=self.auto).pack(side="left", padx=4)
        self.robot_var = tk.StringVar(value="not searched yet")
        ttk.Label(robot, textvariable=self.robot_var, foreground="#555").pack(side="left", padx=10)

        feats = ttk.LabelFrame(top, text="Features (saved in /data/hardware.yaml, applied at the next start)",
                               padding=8)
        feats.pack(fill="x", pady=4)
        hw = self._load_hardware()
        self.features = {}
        for i, (path, label) in enumerate(FEATURES):
            var = tk.BooleanVar(value=bool(self._get(hw, path, path not in FEATURE_OFF_BY_DEFAULT)))
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

        manual = ttk.LabelFrame(top, text="Manual driving (keys, joystick safe)", padding=8)
        manual.pack(fill="x", pady=6)
        self.stop_btn = tk.Button(manual, text="Obstacle stop: ON", width=20, bg="#2e7d32", fg="white",
                                  activebackground="#2e7d32", activeforeground="white",
                                  command=self.toggle_obstacle_stop)
        self.stop_btn.pack(side="left", padx=4)
        self.stop_info = ttk.Label(manual, text="Stuck between obstacles? Turn it off to drive out slowly.",
                                   foreground="#555")
        self.stop_info.pack(side="left", padx=10)

        self.drive_info = ttk.Label(
            top, text="Driving: WASD or arrow keys while this window has focus (works in every mode, speed limited).",
            foreground="#555")
        self.drive_info.pack(anchor="w", pady=(4, 4))

        log_frame = ttk.LabelFrame(self.root, text="Live Log", padding=6)
        log_frame.pack(fill="both", expand=True, padx=10, pady=(0, 10))
        self.log_box = scrolledtext.ScrolledText(log_frame, state="disabled", wrap="word", height=14)
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

    # ---- robot discovery -------------------------------------------------------------------------
    @staticmethod
    def _iface_is_up():
        try:
            with open(f"/sys/class/net/{ROBOT_IFACE}/operstate", encoding="ascii") as f:
                return f.read().strip() == "up"
        except OSError:
            return False

    def _watch_iface(self):
        up = self._iface_is_up()
        if up != self.iface_up:
            first = self.iface_up is None
            self.iface_up = up
            if up:
                self._on_iface_up(first)
            elif not first:
                self._log(f"[GUI] Robot Ethernet {ROBOT_IFACE} went down (cable unplugged?).\n")
                self.robot_var.set(f"{ROBOT_IFACE} down")
        self.root.after(2000, self._watch_iface)

    def _on_iface_up(self, at_start):
        uri = os.environ.get("CYCLONEDDS_URI", "")
        if "NetworkInterface" not in uri:
            # the panel was started without the cable: DDS of the processes started from now on uses it
            os.environ["CYCLONEDDS_URI"] = (uri + "," if uri else "") + DDS_PIN.format(ROBOT_IFACE)
            self._log(f"[GUI] DDS pinned to {ROBOT_IFACE} for the stacks started from now on.\n")
            if self.ros_node is not None:
                self._log("[GUI] Restart the panel so its own driving keys use it too.\n")
        if self._running("mapping") or self._running("navigation"):
            return
        if self.source.get() == "sim":
            self.source.set("hardware")
            self._log("[GUI] Sensor source set to 'hardware'.\n")
        self._log(f"[GUI] Robot Ethernet {ROBOT_IFACE} is up: looking for the robot driver.\n")
        self.root.after(3000 if not at_start else 500, self.find_robot)

    def find_robot(self):
        if self.discovering:
            return
        if self._running("mapping") or self._running("navigation"):
            messagebox.showinfo("Find robot", "Stop mapping / navigation first: discovery needs the robot\n"
                                "driver alone (the KELO stack's own topics would be in the way).")
            return
        self.discovering = True
        self.find_btn.config(state="disabled")
        self.apply_btn.config(state="disabled")
        self.robot_var.set("searching (about 10 s)...")
        threading.Thread(target=self._discover_worker, daemon=True).start()

    def _discover_worker(self):
        result = None
        try:
            proc = subprocess.run(["ros2", "run", PACKAGE, "robot_discover", "--json"],
                                  capture_output=True, text=True, timeout=90)
            for line in proc.stdout.splitlines():
                if line.startswith("JSON "):
                    result = json.loads(line[5:])
                elif line.startswith("[robot_discover]"):
                    self._log(line + "\n")
            if result is None:
                self._log(f"[GUI] Discovery gave no result:\n{proc.stderr[-2000:]}\n")
        except Exception as e:
            self._log(f"[GUI] Discovery failed: {e}\n")
        self.ui_queue.put(lambda: self._discover_done(result))

    def _discover_done(self, result):
        self.discovering = False
        self.find_btn.config(state="normal")
        if result is None:
            self.robot_var.set("discovery failed (see log)")
            return
        fails = [t for lv, t in result["notes"] if lv == "FAIL"]
        warns = [t for lv, t in result["notes"] if lv == "WARN"]
        self.proposal = [(tuple(p), v) for p, v in result["settings"]]
        if not result["found"]:
            self.robot_var.set("robot NOT found: " + (fails[0] if fails else "see log"))
            self._auto_retry_later()
            return
        state = f"robot found, {len(warns)} warning(s)"
        if self.proposal:
            self.apply_btn.config(state="normal")
            self.robot_var.set(state + f"; {len(self.proposal)} setting(s) to apply")
            self.apply_proposal(ask=not self.auto.get())
        else:
            self.robot_var.set(state + "; settings already match")
        self._auto_start()

    def _auto_retry_later(self):
        """Automatic: the cable is in but the driver is not running yet: look again every 10 s."""
        if not self.auto.get() or not self.iface_up or self.auto_retry is not None:
            return
        def again():
            self.auto_retry = None
            if self.auto.get() and self.iface_up and not (self._running("mapping") or self._running("navigation")):
                self.find_robot()
        self.auto_retry = self.root.after(10000, again)
        self._log("[GUI] Automatic: robot driver not found yet, looking again in 10 s.\n")

    def _auto_start(self):
        """Automatic: start navigation on the saved map (nav_watch localizes by itself and says READY)."""
        if not self.auto.get() or self._running("mapping") or self._running("navigation"):
            return
        if not os.path.isfile(MAP_YAML):
            self._log(f"[GUI] Automatic: no map ({MAP_YAML}): build one with Start Mapping -> Save Map.\n")
            return
        if self.source.get() != "hardware":
            return
        self._log("[GUI] Automatic: starting navigation (the robot localizes by itself).\n")
        self.start_navigation()

    def apply_proposal(self, ask=True):
        if not self.proposal:
            return
        text = "\n".join(f"  {'.'.join(p)}: {v}" for p, v in self.proposal)
        if not ask:
            self._log(f"[GUI] Automatic: applying the robot's settings:\n{text}\n")
        elif not messagebox.askyesno("Robot found", "Discovery proposes these settings for this robot:\n\n"
                                     f"{text}\n\nWrite them now? (hardware.yaml; nothing moves the robot)"):
            return
        for path, value in self.proposal:
            try:
                if path[0] == "env":
                    self._set_env(path[1], str(value))
                else:
                    self._save_override(path, value)
                self._log(f"[GUI] {'.'.join(path)} = {value} applied.\n")
            except Exception as e:
                self._log(f"[GUI] {'.'.join(path)} could not be applied: {e}\n")
        self.proposal = []
        self.apply_btn.config(state="disabled")
        self.robot_var.set("robot found; settings applied (used at the next start)")

    def _set_env(self, key, value):
        os.environ[key] = value
        env_file = os.environ.get("KELO_ENV_FILE")
        if env_file:
            lines = []
            if os.path.isfile(env_file):
                with open(env_file, encoding="utf-8") as f:
                    lines = [ln for ln in f.read().splitlines() if not ln.startswith(key + "=")]
            lines.append(f"{key}={value}")
            with open(env_file, "w", encoding="utf-8") as f:
                f.write("\n".join(lines) + "\n")
            self._log(f"[GUI] {key}={value} saved in {env_file}.\n")
        else:
            self._log(f"[GUI] {key}={value} used for this panel session; add it to .env to keep it.\n")
        if self.ros_node is not None:
            self._log("[GUI] Restart the panel so its own driving keys use it too.\n")

    def _refresh_status(self):
        running = [n for n, p in self.procs.items() if p.poll() is None]
        self.status_var.set("State: Running -> " + ", ".join(running) if running else "State: Stopped")
        self._show_nav_status(bool(running))
        if self.stop_off and not running:
            self._set_stop_off(False)   # a new stack always starts with the obstacle stop on
        self.root.after(1000, self._refresh_status)

    def _show_nav_status(self, running):
        """Robot line from nav_watch: green READY/DRIVING, amber WAITING, red NOT READY."""
        if running and self.nav_sub is None and self._ensure_drive_pub():
            from rclpy.qos import DurabilityPolicy, QoSProfile
            from std_msgs.msg import String
            self.nav_sub = self.ros_node.create_subscription(
                String, "/kelo/nav_status", lambda m: setattr(self, "nav_status", (time.time(), m.data)),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.health_sub = self.ros_node.create_subscription(
                String, "/kelo/health", self._on_health,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        if self.ros_node is not None:
            try:
                self._rclpy.spin_once(self.ros_node, timeout_sec=0.0)
            except Exception:
                pass
        self._show_health(running)
        if not running:
            self.robot_state.config(text="Robot: -", fg="#555")
            return
        if self.nav_status is None or time.time() - self.nav_status[0] > 12.0:
            self.robot_state.config(text="Robot: starting...", fg="#555")
            return
        text = self.nav_status[1]
        color = ("#2e7d32" if text.startswith(("READY", "DRIVING")) else
                 "#c62828" if text.startswith(("NOT READY", "goal given up", "goal refused", "could not")) else
                 "#b26a00")
        self.robot_state.config(text="Robot: " + text, fg=color)

    def _on_health(self, msg):
        try:
            self.health = (time.time(), json.loads(msg.data))
        except ValueError:
            pass

    def _show_health(self, running):
        """Health line: green when every started node is up, red when one is down, amber when one was
        restarted or the Jetson is near its limits; the load (CPU, GPU, RAM, temperature) after it."""
        if not running or self.health is None or time.time() - self.health[0] > 5.0:
            self.health_state.config(text="Health: -" if not running else "Health: starting...", fg="#555")
        else:
            h = self.health[1]
            states = [s for s, _ in h.get("nodes", {}).values()]
            restarted = any(c for _, c in h.get("nodes", {}).values())
            color = ("#c62828" if "down" in states or "inactive" in states else
                     "#b26a00" if restarted or h.get("warn") else "#2e7d32")
            text = "Health: " + h.get("summary", "")
            if h.get("load_text"):
                text += "\n" + h["load_text"]
            self.health_state.config(text=text, fg=color)
        if self.nodes_win is not None:
            self._fill_nodes()

    def show_nodes(self):
        """Window with every node of the stack: state and restarts."""
        if self.nodes_win is not None and self.nodes_win.winfo_exists():
            self.nodes_win.lift()
            return
        win = tk.Toplevel(self.root)
        win.title("KELO nodes")
        tree = ttk.Treeview(win, columns=("state", "restarts"), height=24)
        tree.heading("#0", text="node")
        tree.heading("state", text="state")
        tree.heading("restarts", text="restarts")
        tree.column("#0", width=230)
        tree.column("state", width=90)
        tree.column("restarts", width=70, anchor="e")
        for tag, color in (("ok", "#2e7d32"), ("down", "#c62828"), ("inactive", "#c62828"),
                           ("waiting", "#777")):
            tree.tag_configure(tag, foreground=color)
        tree.pack(fill="both", expand=True, padx=6, pady=6)
        self.nodes_win, self.nodes_tree = win, tree

        def closed():
            self.nodes_win = None
            win.destroy()
        win.protocol("WM_DELETE_WINDOW", closed)
        self._fill_nodes()

    def _fill_nodes(self):
        tree = self.nodes_tree
        tree.delete(*tree.get_children())
        nodes = (self.health[1].get("nodes", {}) if self.health is not None else {})
        if not nodes:
            tree.insert("", "end", text="(stack not running)", values=("", ""), tags=("waiting",))
        order = {"down": 0, "inactive": 1, "ok": 2, "waiting": 3}
        for name, (state, restarts) in sorted(nodes.items(), key=lambda kv: (order.get(kv[1][0], 9), kv[0])):
            tree.insert("", "end", text=name, values=(state, restarts or ""), tags=(state,))

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

    def toggle_obstacle_stop(self):
        if not self.stop_off:
            if not any(p.poll() is None for p in self.procs.values()):
                self._log("[GUI] Start the stack first: the obstacle stop only exists with the stack on.\n")
                return
            if not messagebox.askyesno(
                    "Turn the obstacle stop off?",
                    "Manual driving will NOT stop for obstacles, at 0.15 m/s and 0.5 rad/s at most.\n"
                    "Navigation keeps its own stop.\n\nKeep a hand on the emergency stop. Continue?"):
                return
        self._set_stop_off(not self.stop_off)

    def _set_stop_off(self, off):
        self.stop_off = off
        if off:
            self.stop_btn.config(text="Obstacle stop: OFF", bg="#c62828", activebackground="#c62828")
            self.stop_info.config(text="NO STOP FOR OBSTACLES in manual driving. Turn it back on when out.",
                                  foreground="#c62828")
            self._log("[GUI] Obstacle stop OFF for manual driving.\n")
        else:
            self.stop_btn.config(text="Obstacle stop: ON", bg="#2e7d32", activebackground="#2e7d32")
            self.stop_info.config(text="Stuck between obstacles? Turn it off to drive out slowly.",
                                  foreground="#555")
            self._log("[GUI] Obstacle stop on.\n")
        self._publish_stop_off()

    def _publish_stop_off(self):
        if self.stop_off_pub is None:
            if not self._ensure_drive_pub():
                return
            from std_msgs.msg import Bool
            self._Bool = Bool
            self.stop_off_pub = self.ros_node.create_publisher(Bool, "/kelo/obstacle_stop_off", 10)
        self.stop_off_pub.publish(self._Bool(data=self.stop_off))

    def _stop_off_tick(self):
        # repeated: cmd_vel_gate turns the stop back on by itself if this goes silent
        if self.stop_off:
            self._publish_stop_off()
        self.root.after(1000, self._stop_off_tick)

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
