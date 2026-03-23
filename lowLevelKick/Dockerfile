FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV YOLO_CONFIG_DIR=/tmp
#ENV ROBOT_IP="10.56.43.98"
#ENV CONN_TYPE="webrtc"
#ENV CYCLONEDDS_URI=/ros2_ws/cyclonedds.xml
#ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 1. Actualizar repositorios e instalar herramientas básicas
RUN apt update && apt install -y \
    git \
    python3-pip \
    clang \
    portaudio19-dev \
    python3-colcon-common-extensions \
    libcanberra-gtk-module \
    libgl1-mesa-glx \
    libx11-dev \
    net-tools \
    iputils-ping \
    iproute2 \
    curl \
    gnupg2 \
    bc \
    lsb-release \
    # Dependencias del simulador Unitree/MuJoCo
    libyaml-cpp-dev \
    libspdlog-dev \
    libboost-all-dev \
    libglfw3-dev \
    && rm -rf /var/lib/apt/lists/*

# 2. Instalar paquetes ROS específicos
RUN apt update && apt install -y \
    ros-humble-tf2-sensor-msgs \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-xacro \
    ros-humble-robot-localization \
    ros-humble-ros2-controllers \
    ros-humble-ros2-control \
    ros-humble-velodyne \
    ros-humble-velodyne-gazebo-plugins \
    ros-humble-velodyne-description \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-generator-dds-idl \
    ros-humble-navigation2 \
    ros-humble-twist-mux \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-pointcloud-to-laserscan \
    #ros-humble-foxglove-bridge \
    ros-humble-usb-cam \
    ros-humble-librealsense2* \
    ros-humble-realsense2-* \
    ros-humble-image-view \
    && rm -rf /var/lib/apt/lists/*

#RUN apt update && apt upgrade -y

RUN pip3 install --no-cache-dir mujoco pygame

# 3. Instalar unitree_sdk2
RUN git clone https://github.com/unitreerobotics/unitree_sdk2.git /tmp/unitree_sdk2 && \
    cd /tmp/unitree_sdk2 && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics && \
    make install && \
    rm -rf /tmp/unitree_sdk2

# 4. Descargar e instalar MuJoCo
ARG MUJOCO_VERSION=3.3.6
RUN mkdir -p /root/.mujoco && \
    curl -L https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz \
    -o /tmp/mujoco.tar.gz && \
    tar -xzf /tmp/mujoco.tar.gz -C /root/.mujoco && \
    rm /tmp/mujoco.tar.gz

# 5. Compilar unitree_mujoco (simulate)
ARG MUJOCO_VERSION=3.3.6
RUN git clone https://github.com/unitreerobotics/unitree_mujoco.git /ros2_ws/unitree_mujoco && \
    cd /ros2_ws/unitree_mujoco/simulate && \
    ln -s /root/.mujoco/mujoco-${MUJOCO_VERSION} mujoco && \
    mkdir build && cd build && \
    cmake .. && make -j4

# 6. Instalar paquetes Python globales
#RUN pip3 install --no-cache-dir \
#    ultralytics \
#    numpy==1.24.4

# 7. Instalar requerimientos Python del proyecto
#COPY requirements.txt /tmp/requirements.txt
#RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# 8. Workspace de trabajo
WORKDIR /ros2_ws

# 9. Comando por defecto
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && cd /ros2_ws && exec bash"]
