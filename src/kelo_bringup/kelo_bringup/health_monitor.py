import json
import sys
import time

import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from kelo_bringup import health as hl

try:
    import psutil
except ImportError:
    psutil = None


class HealthMonitor(Node):
    def __init__(self):
        super().__init__('health_monitor')
        d = self.declare_parameter
        d('expected_nodes', [''])
        d('reactivate_nodes', [''])
        d('reactivate_after_s', 3.0)
        d('period_s', 1.0)
        d('state_period_s', 2.0)
        g = lambda n: self.get_parameter(n).value
        self.tracker = hl.NodeTracker([n for n in g('expected_nodes') if n and n != self.get_name()])
        self.reactivate = {n for n in g('reactivate_nodes') if n}
        self.after = float(g('reactivate_after_s'))
        self.state_period = float(g('state_period_s'))
        self.lifecycle = {}
        self.non_lifecycle = set()
        self.states = {}
        self.busy = {}
        self.first_seen = {}
        self.last_state_t = 0.0
        self.last_warn = ''
        self.gpu_file = hl.gpu_load_file()
        self.procs = {}
        if psutil is not None:
            psutil.cpu_percent(None)
        self.pub = self.create_publisher(String, '/kelo/health',
                                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(float(g('period_s')), self._safe_tick)
        self.get_logger().info(f'watching {len(self.tracker.expected)} nodes; Nav2 nodes activated again '
                               f'after a restart: {", ".join(sorted(self.reactivate)) or "none"}')


    def _graph(self):
        return {name: ns for name, ns in self.get_node_names_and_namespaces()}

    def _learn_lifecycle(self, name, ns, now):
        if name in self.lifecycle or name in self.non_lifecycle:
            return
        self.first_seen.setdefault(name, now)
        try:
            services = {s for s, _ in self.get_service_names_and_types_by_node(name, ns)}
        except Exception:
            return
        full = (ns.rstrip('/') + '/' + name)
        if full + '/get_state' in services:
            self.lifecycle[name] = (self.create_client(GetState, full + '/get_state'),
                                    self.create_client(ChangeState, full + '/change_state'))
        elif services and now - self.first_seen[name] > 5.0:
            self.non_lifecycle.add(name)

    def _ask_states(self, graph):
        for name, (get, _) in self.lifecycle.items():
            if name not in graph:
                self.states.pop(name, None)
                continue
            if name in self.busy or not get.service_is_ready():
                continue
            self.busy[name] = time.monotonic()
            fut = get.call_async(GetState.Request())
            fut.add_done_callback(lambda f, n=name: self._got_state(n, f))

    def _got_state(self, name, fut):
        self.busy.pop(name, None)
        try:
            self.states[name] = fut.result().current_state.label
        except Exception:
            pass

    def _activate(self, name):
        _, change = self.lifecycle[name]
        if not change.service_is_ready():
            return
        state = self.states.get(name)
        self.get_logger().warn(f'{name} came back {state} after a restart: activating it again')
        self.busy[name] = time.monotonic()

        def step(transition, then=None):
            fut = change.call_async(ChangeState.Request(transition=Transition(id=transition)))
            fut.add_done_callback(lambda f: done(f, transition, then))

        def done(fut, transition, then):
            try:
                ok = fut.result().success
            except Exception:
                ok = False
            if ok and then is not None:
                step(then)
                return
            self.busy.pop(name, None)
            if ok:
                self.get_logger().info(f'{name} is active again')
            else:
                self.get_logger().error(f'could not activate {name} again (transition {transition}): '
                                        'restart navigation')

        if state == 'unconfigured':
            step(Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE)
        else:
            step(Transition.TRANSITION_ACTIVATE)


    def _load(self):
        load, pids = {'gpu': hl.read_gpu(self.gpu_file)}, {}
        if psutil is None:
            return load, pids
        load['cpu'] = psutil.cpu_percent(None)
        vm = psutil.virtual_memory()
        load['ram_used'] = (vm.total - vm.available) / 1e9
        load['ram_total'] = vm.total / 1e9
        try:
            load['temp'] = hl.max_temperature(psutil.sensors_temperatures())
        except Exception:
            load['temp'] = None
        samples, alive = [], set()
        for p in psutil.process_iter(['pid']):
            pid = p.info['pid']
            alive.add(pid)
            proc = self.procs.get(pid)
            first = proc is None
            if first:
                proc = self.procs[pid] = p
            try:
                cmd = proc.cmdline()
                label = hl.process_label(proc.name(), cmd)
                if any('__node:=' in a for a in cmd):
                    pids[label] = pid
                cpu = proc.cpu_percent(None)
                if not first:
                    samples.append((label, cpu))
            except psutil.Error:
                pass
        for pid in set(self.procs) - alive:
            del self.procs[pid]
        load['top'] = hl.top_processes(samples)
        return load, pids


    def _safe_tick(self):
        try:
            self._tick()
        except Exception as exc:
            self.get_logger().error(f'health tick failed: {exc!r}')

    def _tick(self):
        now = time.monotonic()
        for name, t in list(self.busy.items()):
            if now - t > 3.0:
                del self.busy[name]
        graph = self._graph()
        load, pids = self._load()
        for name in self.tracker.expected:
            if name in graph:
                self._learn_lifecycle(name, graph[name], now)
        if now - self.last_state_t >= self.state_period:
            self.last_state_t = now
            self._ask_states(graph)
        present = {n for n in self.tracker.expected
                   if n in pids or (n in graph and self.tracker.pid[n] is None)}
        statuses, changes, restarted = self.tracker.step(now, present, self.states, pids)
        for name in restarted:
            self.get_logger().warn(f'node {name} stopped and started again by itself '
                                   f'(restart {self.tracker.restarts[name]})')
        for name, old, new in changes:
            if new == hl.DOWN:
                self.get_logger().warn(f'node {name} stopped: it is restarted by itself')
            elif new == hl.INACTIVE:
                self.get_logger().warn(f'node {name} is not active')
        for name in self.tracker.to_activate(now, self.after, self.reactivate):
            if name in self.lifecycle and name not in self.busy and name in self.states:
                self.tracker.activated(name, now)
                self._activate(name)
        warn = hl.load_level(load)
        if warn != self.last_warn:
            if warn:
                self.get_logger().warn(f'Jetson load: {warn}')
            self.last_warn = warn
        text = hl.summary(statuses, self.tracker.restarts)
        self.pub.publish(String(data=json.dumps({
            'summary': text,
            'nodes': {n: [s, self.tracker.restarts[n]] for n, s in statuses.items()},
            'load': load, 'load_text': hl.load_text(load), 'warn': warn})))


def main(args=None):
    rclpy.init(args=args)
    node = HealthMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
