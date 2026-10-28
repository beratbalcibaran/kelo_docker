"""Driver views in a phone or tablet browser, without VNC: http://<robot>:8080

- live surround view, colour and depth parking camera and mapping coverage (MJPEG, a view is only
  rendered while someone watches it);
- status: speed, goal distance, people-aware navigation, recording;
- double tap on the floor in a view: drive there (Nav2 goal, as in camera_view); Cancel goal;
- Record / Save clip (video_recorder);
- a hold-to-drive joystick (`allow_drive`): commands go to /cmd_vel_teleop, so the teleop collision
  monitor and the gate limit them like the keyboard; when the finger leaves the pad, or no command
  arrives for 0.3 s (lost connection), the robot gets zero.

Password protected (HTTP basic auth, any user name): KELO_WEB_PASSWORD, or a random one printed in the
log at start. Plain HTTP: use it on the robot's own network only. Python standard library only.
"""
import base64
import hmac
import json
import os
import secrets
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PointStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from kelo_bringup.common import load_hardware
from kelo_bringup.drive_inputs import DriveInputs

VIEWS = {'surround': '/surround_view/image', 'color': '/parking_view/color_image',
         'depth': '/parking_view/image', 'coverage': '/coverage_view/image'}
DRIVE_TIMEOUT_S = 0.3


class Frames:
    """Latest JPEG of every view and how many clients stream it."""

    def __init__(self):
        self.cond = threading.Condition()
        self.jpeg, self.seq, self.clients = {}, {}, {}

    def watching(self, view, delta):
        with self.cond:
            self.clients[view] = max(0, self.clients.get(view, 0) + delta)

    def put(self, view, data):
        with self.cond:
            self.jpeg[view] = data
            self.seq[view] = self.seq.get(view, 0) + 1
            self.cond.notify_all()

    def wait(self, view, last, timeout=2.0):
        with self.cond:
            self.cond.wait_for(lambda: self.seq.get(view, 0) != last, timeout)
            return self.seq.get(view, 0), self.jpeg.get(view)


class WebView(Node):
    def __init__(self, password):
        super().__init__('web_view')
        hw = load_hardware()
        web = hw.get('web', {}) or {}
        d = self.declare_parameter
        d('port', int(web.get('port', 8080)))
        d('allow_drive', bool(web.get('allow_drive', True)))
        d('max_fps', 8.0)
        d('max_vx', 0.30)
        d('max_wz', 0.80)
        g = lambda n: self.get_parameter(n).value
        self.port, self.allow_drive = int(g('port')), bool(g('allow_drive'))
        self.min_period = 1.0 / max(float(g('max_fps')), 0.5)
        self.max_vx, self.max_wz = float(g('max_vx')), float(g('max_wz'))
        self.password = password
        self.frames = Frames()
        self.subs, self.last_put = {}, {}
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.inputs = DriveInputs(self, hw, self.buf)
        self.recorder_state = ''
        self.create_subscription(String, '/video_recorder/state',
                                 lambda m: setattr(self, 'recorder_state', m.data), 10)
        self.click_pub = self.create_publisher(PointStamped, '/kelo/view_click', 10)
        self.drive_pub = self.create_publisher(Twist, hw['topics']['cmd_vel_teleop'], 10)
        self.cancel_cli = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.rec = {k: self.create_client(Trigger, f'/video_recorder/{k}') for k in ('start', 'stop', 'save_clip')}
        self.drive, self.drive_t, self.driving = (0.0, 0.0), -1e9, False
        self.lock = threading.Lock()
        self.create_timer(0.5, self._sync_subs)
        self.create_timer(0.1, self._drive_tick)

    # ---- ROS side (executor thread) ----
    def _sync_subs(self):
        """Subscribe to a view only while a browser streams it: the view is rendered on demand."""
        for view, topic in VIEWS.items():
            want = self.frames.clients.get(view, 0) > 0
            if want and view not in self.subs:
                self.subs[view] = self.create_subscription(Image, topic, lambda m, v=view: self._image(v, m), 2)
            elif not want and view in self.subs:
                self.destroy_subscription(self.subs.pop(view))

    def _image(self, view, msg):
        now = time.time()
        if msg.encoding != 'rgb8' or now - self.last_put.get(view, 0.0) < self.min_period:
            return
        self.last_put[view] = now
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        ok, buf = cv2.imencode('.jpg', cv2.cvtColor(rgb.reshape(msg.height, msg.width, 3), cv2.COLOR_RGB2BGR),
                               [cv2.IMWRITE_JPEG_QUALITY, 75])
        if ok:
            self.frames.put(view, buf.tobytes())

    def _drive_tick(self):
        with self.lock:
            vx, wz = self.drive
            fresh = time.time() - self.drive_t < DRIVE_TIMEOUT_S
        if fresh:
            msg = Twist()
            msg.linear.x, msg.angular.z = vx, wz
            self.drive_pub.publish(msg)
            self.driving = True
        elif self.driving:
            self.drive_pub.publish(Twist())  # finger lifted or connection lost: stop once
            self.driving = False

    # ---- HTTP side (server threads) ----
    def set_drive(self, vx, wz):
        if not self.allow_drive:
            return False
        with self.lock:
            self.drive = (max(-self.max_vx, min(self.max_vx, vx)), max(-self.max_wz, min(self.max_wz, wz)))
            self.drive_t = time.time()
        return True

    def click(self, view, u, v, goal):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = view
        msg.point.x, msg.point.y, msg.point.z = float(u), float(v), 1.0 if goal else 0.0
        self.click_pub.publish(msg)

    def cancel(self):
        """Cancel all navigation goals (empty cancel request)."""
        if not self.cancel_cli.service_is_ready():
            return False
        self.cancel_cli.call_async(CancelGoal.Request())
        return True

    def record(self, action):
        cli = self.rec.get(action)
        if cli is None or not cli.service_is_ready():
            return False, 'video recorder not running'
        fut = cli.call_async(Trigger.Request())
        end = time.time() + 3.0
        while not fut.done() and time.time() < end:
            time.sleep(0.02)
        if not fut.done():
            return False, 'no answer'
        r = fut.result()
        return r.success, r.message

    def status(self):
        vx, vy, wz = self.inputs.twist()
        _, left = self.inputs.plan_in_base()
        blocking, limit = self.inputs.social_state()
        return {'speed': round((vx * vx + vy * vy) ** 0.5, 2), 'turn': round(wz, 2),
                'goal_m': round(left, 1) if left else None, 'person_blocking': bool(blocking),
                'speed_limit': limit, 'note': self.inputs.note(), 'recorder': self.recorder_state,
                'allow_drive': self.allow_drive}


def make_handler(node):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *_):
            pass

        def _authorized(self):
            auth = self.headers.get('Authorization', '')
            if auth.startswith('Basic '):
                try:
                    _, pw = base64.b64decode(auth[6:]).decode().split(':', 1)
                except Exception:
                    pw = None
                if pw is not None and hmac.compare_digest(pw.encode(), node.password.encode()):
                    return True
            self.send_response(401)
            self.send_header('WWW-Authenticate', 'Basic realm="KELO"')
            self.send_header('Content-Length', '0')
            self.end_headers()
            return False

        def _send(self, code, body, ctype='application/json'):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if not self._authorized():
                return
            path = self.path.split('?')[0]
            if path in ('/', '/index.html'):
                return self._send(200, PAGE.encode(), 'text/html; charset=utf-8')
            if path == '/api/status':
                return self._send(200, node.status())
            if path.startswith('/stream/') and path.endswith('.mjpg'):
                return self._stream(path[len('/stream/'):-len('.mjpg')])
            self._send(404, {'error': 'not found'})

        def do_POST(self):
            if not self._authorized():
                return
            try:
                n = int(self.headers.get('Content-Length', '0'))
                req = json.loads(self.rfile.read(n) or b'{}') if n <= 4096 else {}
            except ValueError:
                return self._send(400, {'error': 'bad json'})
            path = self.path.split('?')[0]
            if path == '/api/drive':
                ok = node.set_drive(float(req.get('vx', 0.0)), float(req.get('wz', 0.0)))
                return self._send(200 if ok else 403, {'ok': ok})
            if path == '/api/click' and req.get('view') in VIEWS:
                node.click(req['view'], float(req.get('u', 0)), float(req.get('v', 0)), bool(req.get('goal', True)))
                return self._send(200, {'ok': True})
            if path == '/api/cancel':
                return self._send(200, {'ok': node.cancel()})
            if path == '/api/record' and req.get('action') in ('start', 'stop', 'save_clip'):
                ok, msg = node.record(req['action'])
                return self._send(200, {'ok': ok, 'message': msg})
            self._send(404, {'error': 'not found'})

        def _stream(self, view):
            if view not in VIEWS:
                return self._send(404, {'error': 'no such view'})
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'close')
            self.end_headers()
            node.frames.watching(view, +1)
            seq = -1
            try:
                while True:
                    seq, jpg = node.frames.wait(view, seq)
                    if jpg is None:
                        continue
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                     + str(len(jpg)).encode() + b'\r\n\r\n' + jpg + b'\r\n')
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                node.frames.watching(view, -1)
                self.close_connection = True

    return Handler


PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>KELO</title>
<style>
:root{--bg:#15171c;--panel:#1f2229;--text:#e8eaef;--dim:#9aa0ab;--blue:#2a8cff;--red:#e5484d;--amber:#f5a524}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0;background:var(--bg);color:var(--text);font:15px system-ui,-apple-system,sans-serif}
header{display:flex;align-items:center;gap:10px;padding:10px 12px;background:var(--panel)}
header b{font-size:18px;letter-spacing:.06em}
#status{flex:1;color:var(--dim);font-size:13px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
nav{display:flex;gap:6px;padding:8px 12px;overflow-x:auto}
nav button,.btn{border:0;border-radius:8px;padding:9px 14px;background:#2b2f38;color:var(--text);font-size:14px}
nav button.on{background:var(--blue)}
#wrap{position:relative;padding:0 12px}
#view{display:block;width:100%;border-radius:10px;background:#000;min-height:180px;touch-action:manipulation}
#toast{position:absolute;left:50%;top:14px;transform:translateX(-50%);background:#000c;padding:6px 12px;border-radius:16px;
  font-size:13px;display:none}
#person{display:none;margin:8px 12px 0;padding:8px 12px;border-radius:8px;background:#203a5c;color:#bcd9ff}
.row{display:flex;gap:8px;padding:10px 12px;flex-wrap:wrap}
.btn.red{background:var(--red)} .btn.amber{background:#5a4512}
#padbox{display:flex;justify-content:center;padding:6px 0 18px}
#pad{width:200px;height:200px;border-radius:50%;background:radial-gradient(#2b2f38,#1f2229);position:relative;
  touch-action:none;border:2px solid #333a46}
#knob{position:absolute;width:70px;height:70px;border-radius:50%;background:var(--blue);left:65px;top:65px;opacity:.85}
.hint{color:var(--dim);font-size:12px;text-align:center;padding:0 12px 6px}
</style></head><body>
<header><b>KELO</b><span id="status">connecting...</span></header>
<nav id="tabs"></nav>
<div id="person">Waiting for a person on the path</div>
<div id="wrap"><img id="view" alt=""><div id="toast"></div></div>
<p class="hint">Double tap on the floor: drive there</p>
<div class="row">
  <button class="btn red" id="cancel">Cancel goal</button>
  <button class="btn" id="rec">Record</button>
  <button class="btn amber" id="clip">Save clip</button>
</div>
<div id="drive"><p class="hint">Hold and drag to drive (release = stop)</p>
<div id="padbox"><div id="pad"><div id="knob"></div></div></div></div>
<script>
const VIEWS=[['surround','Surround'],['color','Camera'],['depth','Depth'],['coverage','Coverage']];
let view='surround', recording=false;
try{view=localStorage.getItem('kelo_view')||view}catch(e){}
const img=document.getElementById('view'), tabs=document.getElementById('tabs');
function post(url,body){return fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body||{})})
  .then(r=>r.json()).catch(()=>({ok:false}))}
function toast(t){const e=document.getElementById('toast');e.textContent=t;e.style.display='block';
  clearTimeout(e._t);e._t=setTimeout(()=>e.style.display='none',2500)}
function show(v){view=v;try{localStorage.setItem('kelo_view',v)}catch(e){}
  img.src='/stream/'+v+'.mjpg?'+Date.now();
  [...tabs.children].forEach(b=>b.classList.toggle('on',b.dataset.v===v))}
VIEWS.forEach(([v,t])=>{const b=document.createElement('button');b.textContent=t;b.dataset.v=v;b.onclick=()=>show(v);tabs.appendChild(b)});
show(view);
// double tap / double click: two clicks within 400 ms and 30 px (phones do not all send dblclick)
let lastTap=null;
img.addEventListener('click',e=>{
  const now=Date.now(), near=lastTap&&now-lastTap.t<400&&Math.hypot(e.clientX-lastTap.x,e.clientY-lastTap.y)<30;
  lastTap=near?null:{t:now,x:e.clientX,y:e.clientY};
  if(!near)return;
  if(view==='coverage'){toast('goals: use Surround or Camera');return}
  const r=img.getBoundingClientRect(), u=(e.clientX-r.left)*img.naturalWidth/r.width, v=(e.clientY-r.top)*img.naturalHeight/r.height;
  post('/api/click',{view,u,v,goal:true});toast('sending goal...')});
document.getElementById('cancel').onclick=()=>post('/api/cancel').then(()=>toast('goal cancelled'));
document.getElementById('rec').onclick=()=>post('/api/record',{action:recording?'stop':'start'}).then(r=>toast(r.message||'?'));
document.getElementById('clip').onclick=()=>post('/api/record',{action:'save_clip'}).then(r=>toast(r.message||'?'));
async function poll(){
  try{const s=await (await fetch('/api/status')).json();
    let t=s.speed.toFixed(2)+' m/s';
    if(s.goal_m!==null)t+='  |  goal '+s.goal_m+' m';
    if(s.speed_limit)t+='  |  person near: '+Math.round(s.speed_limit)+'%';
    if(s.recorder)t+='  |  '+s.recorder;
    document.getElementById('status').textContent=t;
    document.getElementById('person').style.display=s.person_blocking?'block':'none';
    recording=(s.recorder||'').startsWith('recording')||(s.recorder||'').startsWith('starting');
    document.getElementById('rec').textContent=recording?'Stop recording':'Record';
    document.getElementById('drive').style.display=s.allow_drive?'block':'none';
    if(s.note)toast(s.note);
  }catch(e){document.getElementById('status').textContent='no connection'}
  setTimeout(poll,1000)}
poll();
// hold-to-drive joystick: up/down = forward/back, left/right = turn; sent every 100 ms while held
const pad=document.getElementById('pad'), knob=document.getElementById('knob');
let cmd=null, timer=null;
function setKnob(dx,dy){knob.style.left=(65+dx)+'px';knob.style.top=(65+dy)+'px'}
function move(e){const r=pad.getBoundingClientRect(), p=e.touches?e.touches[0]:e;
  let dx=p.clientX-r.left-100, dy=p.clientY-r.top-100; const m=Math.hypot(dx,dy);
  if(m>65){dx*=65/m;dy*=65/m} setKnob(dx,dy); cmd={vx:-dy/65*0.3, wz:-dx/65*0.8}; e.preventDefault()}
function start(e){move(e);if(!timer)timer=setInterval(()=>cmd&&post('/api/drive',cmd),100)}
function end(){clearInterval(timer);timer=null;cmd=null;setKnob(0,0);post('/api/drive',{vx:0,wz:0})}
pad.addEventListener('touchstart',start,{passive:false});pad.addEventListener('touchmove',move,{passive:false});
pad.addEventListener('touchend',end);pad.addEventListener('touchcancel',end);
pad.addEventListener('mousedown',e=>{start(e);const mm=ev=>move(ev);
  window.addEventListener('mousemove',mm);window.addEventListener('mouseup',()=>{window.removeEventListener('mousemove',mm);end()},{once:true})});
</script></body></html>
"""


def main():
    rclpy.init()
    password = os.environ.get('KELO_WEB_PASSWORD', '').strip()
    node = WebView(password or secrets.token_urlsafe(9))
    if not password:
        node.get_logger().warn(f'KELO_WEB_PASSWORD not set, generated password for the web view: {node.password}')
    server = ThreadingHTTPServer(('0.0.0.0', node.port), make_handler(node))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(f'web view: http://<robot>:{node.port} (user kelo, password from KELO_WEB_PASSWORD), '
                           f'driving {"on" if node.allow_drive else "off"}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    server.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
