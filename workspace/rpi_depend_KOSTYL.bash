#!/bin/bash

# Обновляем список пакетов
echo "Обновляем список пакетов"
sudo apt-get update > /dev/null 2>&1

# Устанавливаем необходимые библиотеки для ARM64
echo "Устанавливаем необходимые библиотеки для ARM64"
sudo apt-get install -y libc6-dev:arm64 > /dev/null 2>&1

# Устанавливаем библиотеку для работы с Raspberry Pi (включает MMAL)
echo "Устанавливаем библиотеку для работы с Raspberry Pi (включает MMAL)"
sudo apt-get install -y libraspberrypi-dev > /dev/null 2>&1

# Устанавливаем библиотеку RT
echo "Устанавливаем библиотеку RT"
sudo apt-get install -y lib32rt1 > /dev/null 2>&1

# Устанавливаем библиотеки Boost для C++
echo "Устанавливаем библиотеки Boost для C++"
sudo apt-get install -y libboost-all-dev > /dev/null 2>&1

# Устанавливаем библиотеку Asio для работы с асинхронными операциями ввода/вывода
echo "Устанавливаем библиотеку Asio для работы с асинхронными операциями ввода/вывода"
sudo apt-get install -y libasio-dev > /dev/null 2>&1

sudo apt install -y libraspberrypi-bin > /dev/null 2>&1

echo "Устанавливаем python3-rosdep"
sudo apt install python3-rosdep > /dev/null 2>&1
echo "Инициализация rosdep"
sudo rosdep init > /dev/null 2>&1
echo "Обновление rosdep"
rosdep update > /dev/null 2>&1
echo "Установка зависимостей rosdep"
rosdep install --from-paths workspace/src --ignore-src -r -y > /dev/null 2>&1


