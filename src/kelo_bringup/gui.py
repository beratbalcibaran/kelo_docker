import os
import signal
import subprocess
import threading
import time
import tkinter as tk
from tkinter import messagebox, simpledialog, scrolledtext, ttk

PACKAGE = "kelo_bringup"
DATA_DIR = os.environ.get("KELO_DATA_DIR", "/data")
MAP_YAML = os.path.join(DATA_DIR, "maps", "current.yaml")

SOURCES = ("sim", "hardware", "replay")
DRIVE_LINEAR = 0.30
DRIVE_ANGULAR = 0.80


class KeloGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("KELO Control Panel")
        self.root.geometry("760x620-0+0")

        self.procs = {}
        self.ros_node = None
        self.cmd_pub = None
        self.pressed_keys = set()
        self.release_ticks = 0
        self.recording = False

        self.rviz_proc = None
        self.rviz_restarts = 0
        self.view_procs = []
        self._build_widgets()
        self._refresh_status()
        self._drive_tick()
        self._rviz_watchdog()

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
        self.sim_person = tk.BooleanVar(value=True)
        ttk.Label(opts, text="sensor source").pack(side="left")
        ttk.Combobox(opts, textvariable=self.source, values=SOURCES, width=9,
                     state="readonly").pack(side="left", padx=4)
        ttk.Label(opts, text="odometry").pack(side="left", padx=(8, 0))
        ttk.Combobox(opts, textvariable=self.odom_source, values=("robot", "lidar"), width=6,
                     state="readonly").pack(side="left", padx=4)
        ttk.Label(opts, text="bag (replay)").pack(side="left", padx=(8, 0))
        ttk.Entry(opts, textvariable=self.bag_path, width=26).pack(side="left", padx=4)
        ttk.Checkbutton(opts, text="RViz", variable=self.use_rviz).pack(side="left", padx=6)

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

        video = ttk.LabelFrame(top, text="Video (driver views -> /data/videos)", padding=8)
        video.pack(fill="x", pady=6)
        self.rec_btn = ttk.Button(video, text="Record", command=self.toggle_record)
        self.rec_btn.pack(side="left", padx=4)
        ttk.Button(video, text="Save Clip (dashcam)", command=lambda: self._recorder("save_clip")).pack(side="left", padx=4)
        ttk.Label(video, text="Phone/tablet: http://<robot>:8080", foreground="#555").pack(side="left", padx=10)

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
        self.log_box.configure(state="normal")
        self.log_box.insert("end", text)
        self.log_box.see("end")
        self.log_box.configure(state="disabled")

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
        self._start("navigation", ["ros2", "launch", PACKAGE, "bringup.launch.py",
                                   f"map:={MAP_YAML}"] + args)
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
                self.root.after(0, lambda: self.rec_btn.configure(text="Stop Recording" if self.recording else "Record"))
            self._log(f"[GUI] video {action}: {msg}\n")
        threading.Thread(target=run, daemon=True).start()

    def _rviz_watchdog(self):
        stack_up = self._running("mapping") or self._running("navigation")
        want = self.use_rviz.get() and stack_up
        alive = self.rviz_proc is not None and self.rviz_proc.poll() is None
        if self.rviz_proc is not None and not alive:
            code = self.rviz_proc.returncode
            self.rviz_proc = None
            if want:
                self.rviz_restarts += 1
                self._log(f"[GUI] RViz closed (exit code {code}); the stack was NOT affected. "
                          f"Reopening ({self.rviz_restarts}).\n")
        if want and self.rviz_proc is None:
            try:
                from ament_index_python.packages import get_package_share_directory
                cfg = os.path.join(get_package_share_directory(PACKAGE), "rviz", "kelo.rviz")
                self.rviz_proc = subprocess.Popen(["ros2", "run", "rviz2", "rviz2", "-d", cfg],
                                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                                  preexec_fn=os.setsid)
                if not any(p.poll() is None for p in self.view_procs):
                    # RViz helper (camera layer beside the map) and the camera windows; the objects and
                    # the driver views themselves run in the stack
                    self.view_procs = [subprocess.Popen(["ros2", "run", PACKAGE, exe],
                                                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                                        preexec_fn=os.setsid)
                                       for exe in ("map_side_view", "camera_view")]
            except Exception as e:
                self._log(f"[GUI] RViz could not be started: {e}\n")
        elif not want and alive:
            self._kill_group(self.rviz_proc)
            self.rviz_proc = None
        if not want and self.view_procs:
            for p in self.view_procs:
                self._kill_group(p)
            self.view_procs = []
        self.root.after(2000, self._rviz_watchdog)

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
        try:
            out = subprocess.run(["ros2", "node", "list"], capture_output=True, text=True, timeout=10).stdout
        except Exception:
            return False
        return "/cmd_vel_gate" in out.split()

    def _running(self, name):
        return name in self.procs and self.procs[name].poll() is None

    def _start(self, name, cmd):
        if name in self.procs and self.procs[name].poll() is None:
            return
        if name in ("mapping", "navigation") and self._foreign_stack():
            messagebox.showwarning(
                "Another stack is running",
                "A KELO stack that this panel did not start is running (cmd_vel_gate exists).\n"
                "Two stacks cannot run at once (two gates, two sim robots). Stop it first\n"
                "(restart the container or run 'docker compose restart').")
            return
        self._log(f"\n[GUI] Starting ({name}): {' '.join(cmd)}\n")
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, bufsize=1, preexec_fn=os.setsid)
        except FileNotFoundError:
            self._log("[GUI] ERROR: 'ros2' not found.\n")
            return
        self.procs[name] = proc
        threading.Thread(target=self._stream_output, args=(name, proc), daemon=True).start()

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
        if self.rviz_proc is not None and self.rviz_proc.poll() is None:
            self._kill_group(self.rviz_proc)
        for p in self.view_procs:
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
