#!/bin/bash

set -e  # Останавливает выполнение скрипта при ошибке

echo "CONTAINER START SCRIPT RUNNING..."

# Открываем интерактивную оболочку Bash и выполняем команды внутри неё
/bin/bash -i << EOF

echo "Переходим в директорию workspace..."
cd workspace

echo "Строим catkin workspace..."
catkin build

echo "Настройка окружения с помощью setup.bash..."
source devel/setup.bash

EOF
