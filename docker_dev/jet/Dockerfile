# Базовый образ с поддержкой JetPack 5 для Jetson
FROM ultralytics/ultralytics:latest-jetson-jetpack5

# Обновляем систему и устанавливаем системные зависимости
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
                                         vim



# Install ROS desktop and Gazebo

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s 'https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc' | apt-key add - && \
    apt-get update && DEBIAN_FRONTEND=noninteractive \
                      apt-get install -y ros-noetic-desktop-full \
                                         ros-noetic-ros-control \
                                         ros-noetic-teleop-twist-keyboard        
                                        #  ros-noetic-gazebo-ros-pkgs \
                                        #  ros-noetic-gazebo-ros-control \
    #                                      python3-rosdep\
    #                                      python3-rosinstall \
    #                                      python3-rosinstall-generator \
    #                                      python3-wstool \
    #                                      build-essential \
    # rosdep init && rosdep update && \
    # echo "source /opt/ros/noetic/setup.bash"  >> ~/.bashrc && \
    # echo "source /workspace/devel/setup.bash"  >> ~/.bashrc

# Install extra libraries
RUN apt-get install -y python3-pip
# RUN pip install pathlib statistics scipy

RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
                      apt-get install -y ros-noetic-ecl-geometry \
                                            ros-noetic-tf2-sensor-msgs \
                                            ros-noetic-move-base-msgs \
                                            ros-noetic-image-proc
        
RUN pip install numpy scikit-image pillow
                                        
#RUN sudo sh \
    #-c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
    #    > /etc/apt/sources.list.d/ros-latest.list'
#RUN $ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN apt-get update && apt-get install -y python3-catkin-tools 
RUN echo "source /opt/ros/noetic/setup.bash"  >> ~/.bashrc
RUN echo "source /workspace/devel/setup.bash"  >> ~/.bashrc
RUN apt-get update && apt-get install tree

# Обновляем список пакетов
RUN apt-get update
# Устанавливаем необходимые библиотеки для ARM64
RUN apt-get install -y libc6-dev:arm64
# Устанавливаем библиотеку для работы с Raspberry Pi (включает MMAL)
RUN apt-get install -y libraspberrypi-dev
# Устанавливаем библиотеки Boost для C++
# RUN apt-get install -y libboost-all-dev
# Устанавливаем библиотеку Asio для работы с асинхронными операциями ввода/вывода
# RUN apt-get install -y libasio-dev

RUN apt-get update && apt-get install -y \
    ros-noetic-image-transport \
    ros-noetic-camera-info-manager \
    ros-noetic-cv-bridge \
    ros-noetic-compressed-image-transport \
    ros-noetic-theora-image-transport \
    ros-noetic-rqt-image-view \
    v4l-utils \
    mesa-utils \
    udev \
    ros-noetic-usb-cam \
    fswebcam

RUN apt-get update && apt-get install -y \
    libopencv-dev libx11-dev liblapack-dev libv4l-dev libzbar-dev \
    libpthread-stubs0-dev libsdl-dev libsdl-image1.2-dev \
    libeigen3-dev \
    python3-opencv gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly

RUN apt update && apt install -y gstreamer1.0-tools \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav

RUN usermod -aG video $(whoami)
RUN echo 'SUBSYSTEM=="vchiq",MODE="0666"' | tee /etc/udev/rules.d/99-camera.rules

RUN pip install pyserial

######################################

RUN apt-get update && apt-get upgrade -y 

RUN apt-get install -y tzdata && \
    ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata

# Устанавливаем дополнительные пакеты, отсутствующие в rpi_Dockerfile
RUN apt-get install -y \
    libprotobuf-dev \
    protobuf-compiler \
    libgoogle-glog-dev \
    libgflags-dev \
    liblmdb-dev \
    libsnappy-dev

RUN apt-get install -y \
    autoconf \
    automake \
    libtool

RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# Устанавливаем необходимые Python-библиотеки
RUN pip3 install --upgrade pip && \
    pip3 install \
    matplotlib \
    numpy \
    scikit-learn \
    ultralytics

RUN pip install --upgrade numpy

# Указываем рабочую директорию
WORKDIR /workspace

# Копируем исходный код (если он есть) в образ (при необходимости)
# COPY . /workspace

# По умолчанию запускаем bash
CMD ["bash"]
