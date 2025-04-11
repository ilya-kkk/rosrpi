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

<<<<<<< HEAD
# Установка ROS и связанных пакетов
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-catkin-tools \
    ros-noetic-ecl-geometry \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-move-base-msgs \
    ros-noetic-image-proc \
    ros-noetic-image-transport \
    ros-noetic-camera-info-manager \
    ros-noetic-cv-bridge \
    ros-noetic-compressed-image-transport \
    ros-noetic-theora-image-transport \
    ros-noetic-rqt-image-view \
    ros-noetic-usb-cam \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Установка зависимостей OpenCV и графики
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-opencv \
    libopencv-dev \
    libx11-dev \
    libxrender-dev \
    liblapack-dev \
    libv4l-dev \
    libzbar-dev \
    libpthread-stubs0-dev \
    libsdl-dev \
    libsdl-image1.2-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Установка Python-библиотек
=======
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

RUN apt-get update && apt-get install -y apt-utils \
                                         lsb-release \
                                         mesa-utils \
                                         net-tools \
                                         build-essential \
                                         wget \
                                         unzip \
                                         curl \
                                         git \
                                         mc \
                                         vim \
                                         tree \
                                         tzdata \
                                         libprotobuf-dev \
                                         protobuf-compiler \
                                         libgoogle-glog-dev \
                                         libgflags-dev \
                                         liblmdb-dev \
                                         libsnappy-dev \
                                         autoconf \
                                         automake \
                                         libtool \
                                         python3-catkin-tools \
                                         ros-noetic-ecl-geometry \
                                         ros-noetic-tf2-sensor-msgs \
                                         ros-noetic-move-base-msgs \
                                         ros-noetic-image-proc \
                                         ros-noetic-image-transport \
                                         ros-noetic-camera-info-manager \
                                         ros-noetic-cv-bridge \
                                         ros-noetic-compressed-image-transport \
                                         ros-noetic-theora-image-transport \
                                         ros-noetic-rqt-image-view \
                                         v4l-utils \
                                         udev \
                                         ros-noetic-usb-cam \
                                         fswebcam \
                                         libpthread-stubs0-dev libsdl-dev libsdl-image1.2-dev \
                                         libeigen3-dev 
                                        

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s 'https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc' | apt-key add - && \
    apt-get update && DEBIAN_FRONTEND=noninteractive \
                      apt-get install -y ros-noetic-desktop-full \
                                         ros-noetic-ros-control \
                                         ros-noetic-teleop-twist-keyboard        

# Install extra libraries
RUN apt-get install -y python3-pip

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
>>>>>>> 5d7f4b5132db0fc96fd2d19eeb60ee26d4868689
RUN pip3 install --upgrade pip && \
    pip3 install matplotlib numpy scikit-learn

# Создание пользователя без root-прав
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

<<<<<<< HEAD
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    mkdir -p /home/$USERNAME/.config && \
    chown -R $USERNAME:$USERNAME /home/$USERNAME/.config && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Рабочая директория
=======
# Указываем рабочую директорию
>>>>>>> 5d7f4b5132db0fc96fd2d19eeb60ee26d4868689
WORKDIR /workspace

# По умолчанию запуск bash
CMD ["bash"]
