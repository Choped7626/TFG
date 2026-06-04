#! /bin/bash
# meter esto en el docker y ponerlo pa q ejecute todo guay solo, como el run, o tocar el run directamente

# export ROS_DOMAIN_ID=0
# export LD_LIBRARY_PATH=/ros2_ws/devDir/unitree_sdk2/thirdparty/lib/x86_64:$LD_LIBRARY_PATH
source install/setup.bash
# source /root/unitree_ros2/setup.sh 

# sim

export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="lo" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'

export ROS_DOMAIN_ID=1
source install/local_setup.bash
source /root/unitree_ros2/setup_local.sh 
