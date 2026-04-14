#! /bin/bash
# Uso: ./simulate.sh
#
# Bug conocido con env de ros2:
# https://github.com/unitreerobotics/unitree_mujoco/issues/103

echo "[SIMULATE] Lanzando simulador MuJoCo..."
env -u LD_LIBRARY_PATH /ros2_ws/unitree_mujoco/simulate/build/unitree_mujoco
