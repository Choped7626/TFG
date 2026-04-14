#! /bin/bash
# Uso: ./build.sh [-s | -r]
#   -s  Simulación (setup_local.sh)
#   -r  Robot real (setup.sh)

usage() {
  echo "Uso: $0 [-s | -r]"
  echo "  -s  Compilar para simulación"
  echo "  -r  Compilar para robot real"
  exit 0
}

MODE=""
while getopts "srh" opt; do
  case $opt in
  s) MODE="sim" ;;
  r) MODE="real" ;;
  h) usage ;;
  esac
done

if [ -z "$MODE" ]; then
  MODE="sim"
fi

if [ "$MODE" = "sim" ]; then
  echo "[BUILD] Modo: Simulación"
  source /root/unitree_ros2/setup_local.sh
else
  echo "[BUILD] Modo: Robot real"
  source /root/unitree_ros2/setup.sh
fi

colcon build
