#!/usr/bin/env bash

# Публикуем команду движения для того чтобы ардуино начал отсылать данные,
# выполняя команду rostopic pub внутри контейнера.
sudo docker exec -it ros_pc bash -c 'source /opt/ros/noetic/setup.bash && source /workspace/devel/setup.bash && rostopic pub -1 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"; exec bash'
exec bash
