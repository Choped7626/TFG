#! /bin/bash
# Uso: ./run.sh [-s | -r] [-p PACKAGE]
#   -s  Simulación (setup_local.sh, ROS_DOMAIN_ID=1)
#   -r  Robot real (setup.sh,       ROS_DOMAIN_ID=0)
#   -p  Nombre del paquete (por defecto: stand_go2)

usage() {
  echo "Uso: $0 [-s | -r] [-p PACKAGE]"
  echo "  -s  Ejecutar en modo simulación"
  echo "  -r  Ejecutar en modo robot real"
  echo "  -p  Nombre del paquete (por defecto: stand_go2)"
  exit 0
}

MODE=""
PACKAGE="stand_go2"

while getopts "srp:h" opt; do
  case $opt in
  s) MODE="sim" ;;
  r) MODE="real" ;;
  p) PACKAGE="$OPTARG" ;;
  h) usage ;;
  esac
done

if [ -z "$MODE" ]; then
  MODE="sim"
fi

if [ "$MODE" = "sim" ]; then
  echo "[RUN] Modo: Simulación | Paquete: $PACKAGE"
  source /root/unitree_ros2/setup_local.sh
  export ROS_DOMAIN_ID=1
else
  echo "[RUN] Modo: Robot real | Paquete: $PACKAGE"
  source /root/unitree_ros2/setup.sh
  export ROS_DOMAIN_ID=0
fi

./install/${PACKAGE}/bin/${PACKAGE}
