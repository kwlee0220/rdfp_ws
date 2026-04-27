#!/usr/bin/env bash

ros2 run rdfp camera_node --ros-args -p camera_id:=4
# ros2 run rdfp camera_node --ros-args -p camera_id:=4 -p fps:=30 -p resolution:=640x480
