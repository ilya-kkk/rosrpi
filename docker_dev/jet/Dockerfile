# Базовый образ с поддержкой JetPack 5 для Jetson
FROM ultralytics/ultralytics:latest-jetson-jetpack5

# Установка базовых утилит и sudo
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    lsb-release \
    apt-utils \
    net-tools \
    mesa-utils \
    wget \
    unzip \
    curl \
    git \
    mc \
    vim \
    tree \
    tzdata \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Установка инструментов сборки
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    libprotobuf-dev \
    protobuf-compiler \
    libgoogle-glog-dev \
    libgflags-dev \
    liblmdb-dev \
    libsnappy-dev \
    autoconf \
    automake \
    libtool \
    libeigen3-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Установка GStreamer и V4L2
RUN apt-get update && apt-get install -y --no-install-recommends \
    v4l-utils \
    fswebcam \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-tools \
    gstreamer1.0-libav \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Если требуется дополнительная настройка, например установка других утилит,
# можно добавить их здесь
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*


# Copy the entrypoint and bashrc scripts so we have 
# our container's environment set up correctly

# COPY entrypoint.sh /entrypoint.sh
# COPY bashrc /home/${USERNAME}/.bashrc


USER root

# install required packages

RUN apt-get update

# Базовые утилиты
RUN apt-get install -y apt-utils
RUN apt-get install -y lsb-release
RUN apt-get install -y mesa-utils
RUN apt-get install -y net-tools
RUN apt-get install -y build-essential
RUN apt-get install -y wget
RUN apt-get install -y unzip
RUN apt-get install -y curl
RUN apt-get install -y git
RUN apt-get install -y mc
RUN apt-get install -y vim
RUN apt-get install -y tree
RUN apt-get install -y tzdata

# Зависимости для протобуфов и логирования
RUN apt-get install -y libprotobuf-dev
RUN apt-get install -y protobuf-compiler
RUN apt-get install -y libgoogle-glog-dev
RUN apt-get install -y libgflags-dev
RUN apt-get install -y liblmdb-dev
RUN apt-get install -y libsnappy-dev

# Инструменты сборки
RUN apt-get install -y autoconf
RUN apt-get install -y automake
RUN apt-get install -y libtool

# ROS зависимости
RUN apt-get install -y python3-catkin-tools
RUN apt-get install -y ros-noetic-ecl-geometry
RUN apt-get install -y ros-noetic-tf2-sensor-msgs
RUN apt-get install -y ros-noetic-move-base-msgs
RUN apt-get install -y ros-noetic-image-proc
RUN apt-get install -y ros-noetic-image-transport
RUN apt-get install -y ros-noetic-camera-info-manager
RUN apt-get install -y ros-noetic-cv-bridge
RUN apt-get install -y ros-noetic-compressed-image-transport
RUN apt-get install -y ros-noetic-theora-image-transport
RUN apt-get install -y ros-noetic-rqt-image-view

# Зависимости для работы с камерой
RUN apt-get install -y v4l-utils
RUN apt-get install -y udev
RUN apt-get install -y ros-noetic-usb-cam
RUN apt-get install -y fswebcam

# Зависимости для SDL и Eigen
RUN apt-get install -y libpthread-stubs0-dev
RUN apt-get install -y libsdl-dev
RUN apt-get install -y libsdl-image1.2-dev
RUN apt-get install -y libeigen3-dev

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s 'https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc' | apt-key add - && \
    apt-get update && DEBIAN_FRONTEND=noninteractive \
                      apt-get install -y ros-noetic-desktop-full \
                                         ros-noetic-ros-control \
                                         ros-noetic-teleop-twist-keyboard        

# Install extra libraries
RUN apt-get install -y python3-pip

# RUN pip install pathlib statistics scipy

RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
                      apt-get install -y ros-noetic-ecl-geometry \
                                            ros-noetic-tf2-sensor-msgs \
                                            ros-noetic-move-base-msgs \
                                            ros-noetic-image-proc
        
RUN pip install numpy scikit-image pillow

RUN echo "source /opt/ros/noetic/setup.bash"  >> ~/.bashrc
RUN echo "source /workspace/devel/setup.bash"  >> ~/.bashrc

RUN usermod -aG video $(whoami)
RUN echo 'SUBSYSTEM=="vchiq",MODE="0666"' | tee /etc/udev/rules.d/99-camera.rules

RUN pip install pyserial

######################################

RUN apt-get update && apt-get upgrade -y 

RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# Устанавливаем необходимые Python-библиотеки
RUN pip3 install --upgrade pip && \
    pip3 install matplotlib numpy scikit-learn

RUN apt update && apt install -y ros-noetic-realsense2-camera
# Создание пользователя без root-прав
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID


# Указываем рабочую директорию
WORKDIR /workspace

# По умолчанию запуск bash
CMD ["bash"]
