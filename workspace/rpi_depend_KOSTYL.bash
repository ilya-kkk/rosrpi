#!/bin/bash

# Обновляем список пакетов
sudo apt-get update

# Устанавливаем необходимые библиотеки для ARM64
sudo apt-get install -y libc6-dev:arm64

# Устанавливаем библиотеку для работы с Raspberry Pi (включает MMAL)
sudo apt-get install -y libraspberrypi-dev

# Устанавливаем библиотеки Boost для C++
sudo apt-get install -y libboost-all-dev

# Устанавливаем библиотеку Asio для работы с асинхронными операциями ввода/вывода
sudo apt-get install -y libasio-dev

apt update && apt install -y v4l-utils  
apt update

######## СМЕРТЕЛЬНЫЙ НОМЕР, СОБИРАЕМ ПАКЕТ ИЗ ИСХОДНИКОВ ################
apt update && apt install -y \
    cmake ninja-build pkg-config python3-pip \
    python3-yaml python3-ply python3-jinja2 python3-pyparsing \
    python3-dev python3-pil python3-scipy \
    libboost-dev libgnutls28-dev openssl \
    libtiff-dev libjpeg-dev libpng-dev \
    libevent-dev libcurl4-openssl-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

git clone --depth=1 https://git.libcamera.org/libcamera/libcamera.git
cd libcamera

mkdir build && cd build
cmake -GNinja ..
ninja
ninja install
ldconfig
########################################################################

sudo apt install python3-rosdep
sudo rosdep init
rosdep update
cd workspace && rosdep install --from-paths src --ignore-src -r -y

echo "Скрипт выполнен успешно!"