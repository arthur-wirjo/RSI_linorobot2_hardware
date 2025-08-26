#!/bin/bash
# Script to run Micro-ROS Agent Docker container

docker run -it --rm \
    -v /dev:/dev \
    -v /dev/shm:/dev/shm \
    --privileged \
    --net=host \
    microros/micro-ros-agent:rolling \
    serial --dev /dev/ttyUSB0 -b 921600 -v4  

