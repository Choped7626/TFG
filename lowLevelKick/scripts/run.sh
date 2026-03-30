#! /bin/bash
# Uso: ./run.sh [-s | -r] [-i INTERFACE]
#   -s  Simulación (setup_local.sh, ROS_DOMAIN_ID=1)
#   -r  Robot real (setup.sh,       ROS_DOMAIN_ID=0)
#   -i  Interfaz de red (solo modo real)

usage() {
  echo "Uso: $0 [-s | -r] [-i INTERFACE]"
  echo "  -s  Ejecutar en modo simulación"
  echo "  -r  Ejecutar en modo robot real"
  echo "  -i  Interfaz de red para CycloneDDS"
  exit 0
}

# Detectar nombre del paquete desde el directorio actual
PACKAGE=$(basename "$PWD")

MODE=""
INTERFACE=""

while getopts "sri:h" opt; do
  case $opt in
  s) MODE="sim" ;;
  r) MODE="real" ;;
  i) INTERFACE="$OPTARG" ;;
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
  if [ -z "$INTERFACE" ]; then
    echo -n "[RUN] Introduce la interfaz de red (enter para mantener la actual): "
    read INTERFACE
  fi

  if [ -n "$INTERFACE" ]; then
    SETUP_FILE="/root/unitree_ros2/setup.sh"
    CURRENT=$(grep -oP 'name="\K[^"]+(?=" priority)' "$SETUP_FILE")
    if [ "$CURRENT" = "$INTERFACE" ]; then
      echo "[RUN] Interfaz ya es '$INTERFACE', no se modifica setup.sh"
    else
      sed -i "s/name=\"${CURRENT}\"/name=\"${INTERFACE}\"/" "$SETUP_FILE"
      echo "[RUN] Interfaz cambiada: '$CURRENT' → '$INTERFACE' en setup.sh"
    fi
  fi

  echo "[RUN] Modo: Robot real | Paquete: $PACKAGE"
  source /root/unitree_ros2/setup.sh
  export ROS_DOMAIN_ID=0
fi

./install/${PACKAGE}/bin/${PACKAGE}
