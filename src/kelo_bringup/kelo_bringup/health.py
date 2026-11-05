"""Decisions of health_monitor without ROS: which nodes of the stack are up, which came back after a
crash, which lifecycle nodes must be activated again, and the Jetson load (CPU, GPU, RAM, temperature).

Every long-running node is started with respawn=True: a crashed process starts again by itself. A
respawned Nav2 node comes up unconfigured; lifecycle_manager_navigation runs without bonds
(bond_timeout 0) and does not notice, so health_monitor configures and activates it again.
"""
import glob
import os

OK, DOWN, WAITING, INACTIVE = 'ok', 'down', 'waiting', 'inactive'


class NodeTracker:
    """Per expected node: never seen -> waiting (Nav2 starts only once localized), seen -> ok, gone
    after being seen -> down, back again -> ok with one more restart. A lifecycle node that was active
    and is now unconfigured/inactive was restarted (or deactivated) -> inactive."""

    def __init__(self, expected):
        self.expected = list(dict.fromkeys(n for n in expected if n))
        self.seen = {n: False for n in self.expected}
        self.up = {n: False for n in self.expected}
        self.restarts = {n: 0 for n in self.expected}
        self.was_active = {n: False for n in self.expected}
        self.inactive_since = {n: None for n in self.expected}
        self.pid = {n: None for n in self.expected}

    def step(self, now, present, states=None, pids=None):
        """present: running node names; states: name -> lifecycle state label ('active', ...) for the
        lifecycle nodes whose state is known; pids: name -> process id (a new pid is a restart, also
        when it was too quick to be seen as down). Returns {name: status}, the changes
        [(name, old, new)] and the names restarted in this step."""
        states, pids = states or {}, pids or {}
        out, changes, restarted = {}, [], []
        for n in self.expected:
            old = self.status(n)
            here = n in present
            pid = pids.get(n)
            new_pid = here and pid is not None and self.pid[n] is not None and pid != self.pid[n]
            if here and self.seen[n] and (not self.up[n] or new_pid):
                self.restarts[n] += 1
                restarted.append(n)
                states.pop(n, None)          # the state read before the restart is stale
                if self.was_active[n]:
                    self.inactive_since[n] = now  # not active until it says so
            if here:
                self.seen[n] = True
                if pid is not None:
                    self.pid[n] = pid
            self.up[n] = here
            st = states.get(n)
            if here and st == 'active':
                self.was_active[n] = True
                self.inactive_since[n] = None
            elif here and st in ('unconfigured', 'inactive') and self.was_active[n]:
                if self.inactive_since[n] is None:
                    self.inactive_since[n] = now
            elif not here:
                self.inactive_since[n] = None
            new = self.status(n)
            out[n] = new
            if new != old:
                changes.append((n, old, new))
        return out, changes, restarted

    def status(self, n):
        if not self.seen[n]:
            return WAITING
        if not self.up[n]:
            return DOWN
        if self.inactive_since[n] is not None:
            return INACTIVE
        return OK

    def to_activate(self, now, after_s, allowed):
        """Lifecycle nodes in `allowed` that were active, are back but not active for `after_s` s."""
        return [n for n in self.expected if n in allowed and self.up[n]
                and self.inactive_since[n] is not None and now - self.inactive_since[n] >= after_s]

    def activated(self, n, now):
        """An activation attempt: wait `after_s` again before the next one."""
        self.inactive_since[n] = now


def summary(statuses, restarts):
    """Short text for logs and the panel: 'all 23 nodes up' or 'down: amcl; restarted: cloud_filter 1x'."""
    down = [n for n, s in statuses.items() if s == DOWN]
    inactive = [n for n, s in statuses.items() if s == INACTIVE]
    waiting = [n for n, s in statuses.items() if s == WAITING]
    parts = []
    if down:
        parts.append('down: ' + ', '.join(down))
    if inactive:
        parts.append('not active: ' + ', '.join(inactive))
    rs = [f'{n} {c}x' for n, c in restarts.items() if c]
    if rs:
        parts.append('restarted: ' + ', '.join(rs))
    if not parts:
        up = len(statuses) - len(waiting)
        text = f'all {up} nodes up'
        return text + (f' ({len(waiting)} waiting to start)' if waiting else '')
    return '; '.join(parts)


# --- Jetson load ---------------------------------------------------------------------------------

GPU_LOAD_GLOBS = ('/sys/devices/platform/gpu.0/load', '/sys/devices/gpu.0/load',
                  '/sys/devices/platform/*.gpu/load', '/sys/devices/platform/*/*.gpu/load')


def gpu_load_file(globs=GPU_LOAD_GLOBS):
    """Jetson GPU load file (0..1000), None elsewhere."""
    for g in globs:
        hits = sorted(glob.glob(g))
        if hits:
            return hits[0]
    return None


def read_gpu(path):
    if not path:
        return None
    try:
        with open(path) as f:
            return int(f.read().strip()) / 10.0
    except (OSError, ValueError):
        return None


def max_temperature(temps):
    """Highest current temperature from psutil.sensors_temperatures() ({chip: [entries]}), or None."""
    vals = [e.current for entries in (temps or {}).values() for e in entries
            if getattr(e, 'current', None) is not None and 0.0 < e.current < 150.0]
    return max(vals) if vals else None


def process_label(name, cmdline):
    """Node name of a ROS process (__node:=x), else the script / executable name."""
    for a in cmdline or []:
        if a.startswith('__node:='):
            return a.split(':=', 1)[1]
    for a in cmdline or []:
        if a.startswith('-r') and '__node:=' in a:
            return a.split('__node:=', 1)[1]
    if cmdline and os.path.basename(cmdline[0]).startswith('python') and len(cmdline) > 1:
        return os.path.basename(cmdline[1])
    return name or (os.path.basename(cmdline[0]) if cmdline else '?')


def top_processes(samples, n=3):
    """[(label, cpu %)] of the busiest processes, highest first."""
    s = sorted(((lbl, c) for lbl, c in samples if c is not None), key=lambda t: -t[1])
    return [(lbl, round(c)) for lbl, c in s[:n]]


def load_text(load):
    """'CPU 45% GPU 30% RAM 3.1/7.4 GB 58°C' from a load dict (missing parts left out)."""
    parts = []
    if load.get('cpu') is not None:
        parts.append(f"CPU {load['cpu']:.0f}%")
    if load.get('gpu') is not None:
        parts.append(f"GPU {load['gpu']:.0f}%")
    if load.get('ram_used') is not None:
        parts.append(f"RAM {load['ram_used']:.1f}/{load['ram_total']:.1f} GB")
    if load.get('temp') is not None:
        parts.append(f"{load['temp']:.0f}°C")
    top = load.get('top') or []
    if top:
        parts.append('top: ' + ', '.join(f'{lbl} {c}%' for lbl, c in top))
    return '  '.join(parts)


def load_level(load, cpu_warn=90.0, ram_warn=0.9, temp_warn=85.0):
    """'' or a warning when the Jetson is close to its limits."""
    w = []
    if load.get('cpu') is not None and load['cpu'] >= cpu_warn:
        w.append(f"CPU {load['cpu']:.0f}%")
    if load.get('ram_used') is not None and load.get('ram_total') and load['ram_used'] / load['ram_total'] >= ram_warn:
        w.append('RAM almost full')
    if load.get('temp') is not None and load['temp'] >= temp_warn:
        w.append(f"hot {load['temp']:.0f}°C")
    return ', '.join(w)
