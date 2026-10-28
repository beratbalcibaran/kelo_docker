#!/bin/bash
set -u
RES="${SCREEN_RES:-2240x1260x24}"
DISP="${DISPLAY:-:99}"
N="${DISP#:}"
PASS="${KELO_VNC_PASSWORD:-}"
if [ -z "$PASS" ]; then
  PASS="$(tr -dc 'A-Za-z0-9' </dev/urandom | head -c 12)"
  echo "=============================================================="
  echo " KELO_VNC_PASSWORD not set, generated a random password:"
  echo "   $PASS"
  echo " For a fixed password: KELO_VNC_PASSWORD=... docker compose --profile gui up"
  echo "=============================================================="
fi

rm -f "/tmp/.X${N}-lock" "/tmp/.X11-unix/X${N}"
Xvfb "$DISP" -screen 0 "$RES" -ac +extension GLX +render -noreset &
for i in $(seq 1 50); do xdpyinfo -display "$DISP" >/dev/null 2>&1 && break; sleep 0.1; done

mkdir -p /root/.vnc
x11vnc -storepasswd "$PASS" /root/.vnc/passwd >/dev/null 2>&1
openbox >/dev/null 2>&1 &
x11vnc -display "$DISP" -rfbauth /root/.vnc/passwd -forever -shared -rfbport 5900 -quiet >/dev/null 2>&1 &
websockify --web /usr/share/novnc 6080 localhost:5900 >/dev/null 2>&1 &

echo "=============================================================="
echo " KELO interface ready. Open in the browser:"
echo "   http://localhost:6080/vnc.html?autoconnect=true&resize=scale"
echo " You will be asked for the VNC password (above) when connecting."
echo "=============================================================="

trap 'exit 0' TERM INT
while true; do
  python3 /ros2_ws/src/kelo_bringup/gui.py
  code=$?
  echo "[KELO] GUI closed (exit code: $code). Reopening in 2 s..."
  sleep 2
done
