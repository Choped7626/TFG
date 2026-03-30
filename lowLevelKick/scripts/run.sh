#! /bin/bash
# Uso: ./run.sh [-s | -r] [-p PACKAGE] [-i INTERFACE]
#   -s  Simulación (setup_local.sh, ROS_DOMAIN_ID=1)
#   -r  Robot real (setup.sh,       ROS_DOMAIN_ID=0)
#   -p  Nombre del paquete (por defecto: stand_go2)
#   -i  Interfaz de red (solo modo real, por defecto: enp3s0)

usage() {
  echo "Uso: $0 [-s | -r] [-p PACKAGE] [-i INTERFACE]"
  echo "  -s  Ejecutar en modo simulación"
  echo "  -r  Ejecutar en modo robot real"
  echo "  -p  Nombre del paquete (por defecto: stand_go2)"
  echo "  -i  Interfaz de red para CycloneDDS (por defecto: enp3s0)"
  exit 0
}

MODE=""
PACKAGE="stand_go2"
INTERFACE=""

while getopts "srp:i:h" opt; do
  case $opt in
  s) MODE="sim" ;;
  r) MODE="real" ;;
  p) PACKAGE="$OPTARG" ;;
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
  # Pedir interfaz si no se pasó con -i
  if [ -z "$INTERFACE" ]; then
    echo -n "[RUN] Introduce la interfaz de red (enter para mantener la actual): "
    read INTERFACE
  fi

  # Modificar setup.sh solo si se especificó interfaz
  if [ -n "$INTERFACE" ]; then
    SETUP_FILE="/root/unitree_ros2/setup.sh"
    # Extraer la interfaz actual
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
