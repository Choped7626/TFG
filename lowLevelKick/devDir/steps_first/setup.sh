#! /bin/bash
# meter esto en el docker y ponerlo pa q ejecute todo guay solo, como el run, o tocar el run directamente

export LD_LIBRARY_PATH=/ros2_ws/devDir/unitree_sdk2/thirdparty/lib/x86_64:$LD_LIBRARY_PATH
source install/setup.bash

# sim
source /root/unitree_ros2/setup_local.sh 
ros2 run steps_first low_level_controller --sim
