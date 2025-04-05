#!/usr/bin/env bash

ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
EXEC_PATH=$PWD
echo "ROOT_DIR: $ROOT_DIR"
echo "EXEC_PATH: $EXEC_PATH"

cd $ROOT_DIR
docker build --platform linux/arm64 -t jet -f $ROOT_DIR/jet/Dockerfile $ROOT_DIR \
                                  --network=host \
                                  
cd $EXEC_PATH
