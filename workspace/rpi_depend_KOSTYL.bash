#!/bin/bash

YELLOW='\033[1;33m'
GREEN='\033[0;32m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${YELLOW}Обновление списка пакетов...${NC}"
sudo apt update 

echo -e "${YELLOW}Установка необходимых пакетов...${NC}"
sudo apt install -y \
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

echo -e "${YELLOW}Добавление пользователя в группу видео...${NC}"
sudo usermod -aG video $(whoami)

echo -e "${YELLOW}Проверка доступности камеры...${NC}"
if [ ! -e /dev/video0 ]; then
    echo -e "${RED}Ошибка: /dev/video0 не найден! Проверьте подключение камеры.${NC}"
    exit 1
fi

if [ ! -e /dev/vchiq ]; then
    echo -e "${RED}Ошибка: /dev/vchiq не найден! Проверьте настройки Raspberry Pi.${NC}"
    exit 1
fi
echo -e "${GREEN}Проверка успешно пройдена${NC}"

echo -e "${YELLOW}Создание правил udev для доступа к камере...${NC}"
echo 'SUBSYSTEM=="vchiq",MODE="0666"' | sudo tee /etc/udev/rules.d/99-camera.rules

echo -e "${GREEN}Готово!${NC}"
exit 0
