FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV YOLO_CONFIG_DIR=/tmp
#ENV ROBOT_IP="10.56.43.98"
#ENV CONN_TYPE="webrtc"
#ENV CYCLONEDDS_URI=/ros2_ws/cyclonedds.xml
#ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

SHELL ["/bin/bash", "-c"]

# 1. Actualizar repositorios e instalar herramientas básicas
RUN apt update && apt install -y \
    git \
    python3-pip \
    tree \
    clang \
    clangd \
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
    ros-humble-foxglove-bridge \
    ros-humble-usb-cam \
    ros-humble-librealsense2* \
    ros-humble-realsense2-* \
    ros-humble-image-view \
    && rm -rf /var/lib/apt/lists/*

RUN apt update && apt upgrade -y

# 6. Instalar paquetes Python globales
RUN pip3 install --no-cache-dir \
    ultralytics \
    numpy==1.24.4 \
    mujoco \
    pygame 

# 7. Instalar requerimientos Python del proyecto
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# NO VA, meterlo en root y no borrar , pip -e? --- ademais hai un clone q non funciona nin pa tras (buscar cal é e metelo desde local porq paso)
RUN git clone https://github.com/unitreerobotics/unitree_sdk2_python.git /tmp/unitree_sdk2_python && \  
    cd /tmp/unitree_sdk2_python && \
    git checkout ab0d8ae2fee8e3ec337ba941d5e7fd49de5fd30a && \
    pip3 install . && \
    cd / && rm -rf /tmp/unitree_sdk2_python

# 3. Instalar unitree_sdk2
RUN git clone https://github.com/unitreerobotics/unitree_sdk2.git /tmp/unitree_sdk2 && \
    cd /tmp/unitree_sdk2 && \
    git checkout 74e915b630758fe763f089528c3ff42d4396982e && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics && \
    make install && \
    rm -rf /tmp/unitree_sdk2

RUN source /opt/ros/humble/setup.bash && \
    git clone https://github.com/unitreerobotics/unitree_ros2.git /root/unitree_ros2 && \
    cd /root/unitree_ros2 && \
    git checkout 66ae09858245ac3d2231c0cc209e36a88f8d7d03 && \
    git submodule update --init --recursive && \
    cd cyclonedds_ws && colcon build

# Reemplazar foxy por humble en los scripts de setup de unitree_ros2
RUN sed -i 's/opt\/ros\/foxy/opt\/ros\/humble/g' /root/unitree_ros2/setup.sh \
    /root/unitree_ros2/setup_local.sh \
    /root/unitree_ros2/setup_default.sh

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
    cd /ros2_ws/unitree_mujoco && \
    git checkout 1a37b051a10be723405b7ed6dc839361af036d88 && \
    cd simulate && \
    ln -s /root/.mujoco/mujoco-${MUJOCO_VERSION} mujoco && \
    mkdir build && cd build && \
    cmake .. && make -j4

# soluciona problema de script?
ENV LD_LIBRARY_PATH=/root/.mujoco/lib:$LD_LIBRARY_PATH

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'source /root/unitree_ros2/cyclonedds_ws/install/setup.bash' >> /root/.bashrc

COPY scripts/ /usr/local/bin/
RUN chmod +x /usr/local/bin/build.sh \
              /usr/local/bin/run.sh \
              /usr/local/bin/sim.sh

RUN curl -LO https://github.com/neovim/neovim/releases/download/v0.11.7/nvim-linux-x86_64.tar.gz && \
    tar -xzf nvim-linux-x86_64.tar.gz -C /opt/ && \
    ln -sf /opt/nvim-linux-x86_64/bin/nvim /usr/local/bin/nvim && \
    rm nvim-linux-x86_64.tar.gz

RUN git clone https://github.com/erickkramer/nvim-ros2.git /ros2_ws/nvim-ros2

COPY config/nvim/ /root/.config/nvim/

RUN nvim --headless "+Lazy! sync" +qa || true

# 8. Workspace de trabajo
WORKDIR /ros2_ws

# 9. Comando por defecto
CMD ["bash"]
