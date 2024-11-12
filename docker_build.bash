#!/usr/bin/env bash

ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
EXEC_PATH=$PWD
echo "ROOT_DIR: $ROOT_DIR"
echo "EXEC_PATH: $EXEC_PATH"

cd $ROOT_DIR
docker build -t ros_arm -f $ROOT_DIR/docker/Dockerfile $ROOT_DIR \
                                  --network=host \
                                  
cd $EXEC_PATH
