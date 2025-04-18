#!/bin/bash

YELLOW='\033[1;33m'
GREEN='\033[0;32m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

set -e  # Останавливает выполнение скрипта при ошибке

echo "${YELLOW}CONTAINER START SCRIPT RUNNING...${NC}"

# Открываем интерактивную оболочку Bash и выполняем команды внутри неё
/bin/bash -i << EOF

pip3 install pyserial
echo "${YELLOW}Переходим в директорию workspace...${NC}"
cd workspace

catkin clean -y
echo "${YELLOW}Строим catkin workspace...${NC}"
catkin build
export ROS_MASTER_URI=http://192.168.1.145:11311
export ROS_HOSTNAME=192.168.1.145
echo "${YELLOW}Настройка окружения с помощью setup.bash...${NC}"
source devel/setup.bash

# Публикуем команду движения для того чтобы ардуино начал отсылать данные
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"

roslaunch core start.launch
EOF

# Оставляем терминал открытым после выполнения скрипта
exec /bin/bash
