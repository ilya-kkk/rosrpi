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

# Настройка часового пояса
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Добавление репозитория ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s 'https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc' | apt-key add -

# Установка ROS и зависимостей
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    ros-noetic-ros-control \
    ros-noetic-teleop-twist-keyboard \
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
    ros-noetic-realsense2-camera \
    python3-catkin-tools \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Установка Python-зависимостей
RUN pip3 install --upgrade pip && \
    pip3 install numpy scikit-image pillow matplotlib scikit-learn pyserial

# Создание пользователя без root-прав
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Настройка прав доступа к камере
RUN usermod -aG video $USERNAME && \
    echo 'SUBSYSTEM=="vchiq",MODE="0666"' | tee /etc/udev/rules.d/99-camera.rules

# Настройка ROS
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /workspace/devel/setup.bash" >> /home/$USERNAME/.bashrc

# Указываем рабочую директорию
WORKDIR /workspace

# Переключаемся на пользователя ros
USER $USERNAME

# По умолчанию запуск bash
CMD ["bash"]
