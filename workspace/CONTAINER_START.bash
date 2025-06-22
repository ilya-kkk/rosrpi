#!/bin/bash

# Этот скрипт выполняет сборку проекта Catkin.
# Запускайте его из корневой директории /workspace внутри контейнера.

YELLOW='\033[1;33m'
GREEN='\033[0;32m'
NC='\033[0m'

# Останавливаем выполнение скрипта при любой ошибке
set -e

echo -e "${YELLOW}CONTAINER START SCRIPT RUNNING...${NC}"

# ШАГ 1: Активируем базовое окружение ROS. Это КЛЮЧЕВОЙ шаг.
echo -e "${YELLOW}Sourcing ROS Noetic environment...${NC}"
source /opt/ros/noetic/setup.bash

echo -e "${YELLOW}Переходим в директорию /workspace...${NC}"
cd /workspace

echo -e "${YELLOW}Очистка и сборка catkin workspace...${NC}"
catkin clean -y
catkin build

echo "${YELLOW}Настройка окружения с помощью setup.bash...${NC}"
source devel/setup.bash

echo -e "\n${GREEN}Сборка успешно завершена!${NC}"

# ШАГ 2: Настраиваем .bashrc для будущих сессий, чтобы они включали и наш workspace
echo -e "${YELLOW}Настройка .bashrc для автоматической загрузки окружения...${NC}"

# Удаляем старые записи из .bashrc, чтобы избежать дублирования
sed -i '/ROS_MASTER_URI/d' ~/.bashrc
sed -i '/ROS_HOSTNAME/d' ~/.bashrc
# Убедимся, что базовый setup.bash есть и не дублируется
sed -i '/\/opt\/ros\/noetic\/setup.bash/d' ~/.bashrc
sed -i '/\/workspace\/devel\/setup.bash/d' ~/.bashrc

# Добавляем актуальные настройки в конец .bashrc
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
echo "export ROS_HOSTNAME=localhost" >> ~/.bashrc
echo "source /workspace/devel/setup.bash" >> ~/.bashrc

echo -e "\n${GREEN}Окружение настроено! Запускаем интерактивную оболочку...${NC}\n"
echo -e "\n${YELLOW}You can launch next commands \n1.roslaunch core start.launch \n2.roslaunch nav bring_up.launch" 
# Оставляем терминал открытым после выполнения скрипта
exec /bin/bash
