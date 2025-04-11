# Базовый образ с поддержкой JetPack 5 для Jetson
FROM ultralytics/ultralytics:latest-jetson-jetpack5

# Устанавливаем дополнительные пакеты, если они отсутствуют
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    && rm -rf /var/lib/apt/lists/*

# Обновляем pip и устанавливаем необходимые Python-пакеты
RUN python3 -m pip install --upgrade pip

# Устанавливаем зависимости для YOLOv8
RUN pip install --no-cache-dir \
    opencv-python-headless \
    ultralytics

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

# Устанавливаем дополнительные пакеты, отсутствующие в базовом образе
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
                                         libopencv-dev libx11-dev liblapack-dev libv4l-dev libzbar-dev \
                                         libpthread-stubs0-dev libsdl-dev libsdl-image1.2-dev \
                                         libeigen3-dev \
                                         python3-opencv gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
                                         gstreamer1.0-tools gstreamer1.0-libav \
                                         libc6-dev:arm64 \
                                         libraspberrypi-dev \
                                         && rm -rf /var/lib/apt/lists/*

# Устанавливаем необходимые Python-библиотеки
RUN pip3 install --upgrade pip && \
    pip3 install \
    matplotlib \
    numpy \
    scikit-learn \
    ultralytics

# Указываем рабочую директорию
WORKDIR /workspace

# По умолчанию запускаем bash
CMD ["bash"]
