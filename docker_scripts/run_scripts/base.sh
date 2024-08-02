#!/bin/bash

# http://wiki.ros.org/docker/Tutorials/GUI


./stop.sh

xhost +local:root

docker run -t -d --privileged --net=host \
--name simple_amr_project \
-v $PWD/ddsconfig.xml:/ddsconfig.xml \
--env CYCLONEDDS_URI=/ddsconfig.xml \
--env ROS_DOMAIN_ID=11 \
--env="QT_X11_NO_MITSHM=1"  \
--env="DISPLAY"  \
simple_amr_project:latest
