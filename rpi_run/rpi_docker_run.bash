#!/bin/bash

xhost +local:docker || true

ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

docker run -ti --rm \
    --device /dev/video0 \
    --device /dev/vchiq:/dev/vchiq \
    --env LD_LIBRARY_PATH=/opt/vc/lib \
    --device-cgroup-rule='c 81:* rmw' \
    -v /opt/vc:/opt/vc \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -e XAUTHORITY \
    -v /dev:/dev \
    -v "$ROOT_DIR/workspace:/workspace" \
    --net=host \
    --ipc=host \
    --privileged \
    --name ros_rpi ilya9kkk/ros_arm:latest \
    bash -c "sleep 5 && /workspace/rpi_depend_KOSTYL.bash && exec bash"
