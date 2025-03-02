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
    udev

echo "Добавление пользователя в группу видео..."
sudo usermod -aG video $(whoami)

echo "Проверка и загрузка драйвера камеры..."
sudo modprobe bcm2835-v4l2
echo "bcm2835-v4l2" | sudo tee -a /etc/modules

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
sudo udevadm control --reload-rules && sudo udevadm trigger

echo "Проверка состояния камеры..."
vcgencmd get_camera

echo "Запуск ROS и проверка ноды камеры..."
source /opt/ros/noetic/setup.bash
roscore & sleep 5
rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map camera_link 10 &

echo "Готово! Теперь можно запускать ноду камеры."
exit 0
