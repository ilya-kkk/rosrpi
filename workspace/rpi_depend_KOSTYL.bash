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
