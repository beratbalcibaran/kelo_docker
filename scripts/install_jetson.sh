#!/usr/bin/env bash
set -euo pipefail

[ "$(id -u)" -eq 0 ] || { echo "root required: sudo $0"; exit 1; }
SRC="$(cd "$(dirname "$0")/.." && pwd)"
DEST=/opt/kelo
ENABLE=0
[ "${1:-}" = "--enable-service" ] && ENABLE=1

echo "== Platform =="
uname -m
[ -r /etc/nv_tegra_release ] && head -n1 /etc/nv_tegra_release || echo "warning: Jetson L4T not found"
. /etc/os-release && echo "Ubuntu $VERSION_ID"
if ! command -v docker >/dev/null 2>&1; then
  echo "ERROR: docker not installed (comes with JetPack, or: apt install docker.io docker-compose-v2)"; exit 1
fi
docker compose version >/dev/null || { echo "ERROR: docker compose plugin missing"; exit 1; }

echo "== Files -> $DEST =="
mkdir -p "$DEST" /etc/kelo
rsync -a --delete --exclude data --exclude .git "$SRC"/ "$DEST"/
mkdir -p "$DEST/data"
[ -f /etc/kelo/kelo.env ] || cat > /etc/kelo/kelo.env <<'EOF'
ROS_DOMAIN_ID=0
KELO_DATA=/opt/kelo/data
EOF

echo "== udev rules =="
cp "$DEST/src/kelo_bringup/udev/99-kelo-serial.rules" /etc/udev/rules.d/
udevadm control --reload-rules && udevadm trigger

echo "== Image build =="
( cd "$DEST" && docker compose --profile production build )

echo "== systemd =="
cp "$DEST/systemd/kelo.service" /etc/systemd/system/kelo.service
systemctl daemon-reload
if [ "$ENABLE" -eq 1 ]; then
  systemctl enable kelo.service
  echo "Service enabled: starts automatically at boot. To start now: systemctl start kelo"
else
  echo "Service installed but NOT enabled. To enable: systemctl enable --now kelo"
fi
echo "Done. Next step: $DEST/README.md"
