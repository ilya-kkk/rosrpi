#!/usr/bin/env bash

ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
EXEC_PATH=$PWD
echo "ROOT_DIR: $ROOT_DIR"
echo "EXEC_PATH: $EXEC_PATH"

cd $ROOT_DIR
docker build --platform linux/arm64 -t yolo_jet -f $ROOT_DIR/jet/Dockerfile_yolo $ROOT_DIR \
                                  --network=host \
                                  
cd $EXEC_PATH
