#!/bin/bash

YELLOW='\033[1;33m'
GREEN='\033[0;32m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

set -e  # Останавливает выполнение скрипта при ошибке

echo "CONTAINER START JETSON RUNNING..."

# Открываем интерактивную оболочку Bash и выполняем команды внутри неё
/bin/bash -i << EOF

echo "${YELLOW}Переходим в директорию workspace...${NC}"
cd workspace

# catkin clean -y
echo "${YELLOW}Строим catkin workspace...${NC}"
# catkin build

echo "${YELLOW}Настройка окружения с помощью setup.bash...${NC}"
source devel/setup.bash

roslaunch core start_nn.launch
EOF

# Оставляем терминал открытым после выполнения скрипта
exec /bin/bash
