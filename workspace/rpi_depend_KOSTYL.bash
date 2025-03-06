#!/bin/bash

echo "Обновление списка пакетов..."
sudo apt update && sudo apt upgrade -y

echo "Установка необходимых пакетов..."
sudo apt install -y \
    ros-noetic-image-transport \
    ros-noetic-camera-info-manager \
    ros-noetic-cv-bridge \
    ros-noetic-compressed-image-transport \
    ros-noetic-theora-image-transport \
    ros-noetic-rqt-image-view \
    v4l-utils \
    libcamera-dev \
    libcamera-tools \
    libcamera-apps \
    mesa-utils \
    udev \
    ros-noetic-usb-cam

echo "Добавление пользователя в группу видео..."
sudo usermod -aG video $(whoami)

# echo "Проверка и загрузка драйвера камеры..."
# sudo modprobe bcm2835-v4l2
# echo "bcm2835-v4l2" | sudo tee -a /etc/modules

echo "Проверка доступности камеры..."
if [ ! -e /dev/video0 ]; then
    echo "Ошибка: /dev/video0 не найден! Проверьте подключение камеры."
    exit 1
fi

if [ ! -e /dev/vchiq ]; then
    echo "Ошибка: /dev/vchiq не найден! Проверьте настройки Raspberry Pi."
    exit 1
fi

echo "Создание правил udev для доступа к камере..."
echo 'SUBSYSTEM=="vchiq",MODE="0666"' | sudo tee /etc/udev/rules.d/99-camera.rules

# sudo udevadm control --reload-rules && sudo udevadm trigger
###################### Установка из исходников #####################
# sudo apt install -y libraspberrypi-bin libraspberrypi-dev

# apt update && apt install -y \
#     cmake ninja-build pkg-config python3-pip \
#     python3-yaml python3-ply python3-jinja2 \
#     git g++ libclang-dev clang \
#     libboost-dev libgnutls28-dev openssl \
#     libtiff5-dev libevent-dev

# apt update && apt install -y \
#     meson ninja-build python3-pip
# pip3 install --upgrade meson

# git clone https://git.libcamera.org/libcamera/libcamera.git  
# cd libcamera 

# meson setup build
# ninja -C build
# ninja -C build install
# ldconfig
###################################################################
# echo "Проверка состояния камеры..."
# vcgencmd get_camera


echo "Готово!"
exit 0
