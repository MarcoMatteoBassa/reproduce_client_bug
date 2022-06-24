#!/bin/bash

docker run -it \
            --network=host \
            --privileged \
            --restart unless-stopped \
            -v `pwd`:/home/dev/ros2_ws/src/ros \
            -v ros2_ws_x86:/home/dev/ros2_ws \
            -e "DISPLAY=${DISPLAY}" \
            --ipc=host \
            -w /home/dev/ros2_ws/ \
            --name ros2_x86_humble \
            ros2_x86_humble:latest \
            /bin/bash
