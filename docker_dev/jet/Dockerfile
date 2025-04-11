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
RUN pip3 install --upgrade pip && \
    pip3 install matplotlib numpy scikit-learn

# Создание пользователя без root-прав
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    mkdir -p /home/$USERNAME/.config && \
    chown -R $USERNAME:$USERNAME /home/$USERNAME/.config && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Рабочая директория
WORKDIR /workspace

# По умолчанию запуск bash
CMD ["bash"]
