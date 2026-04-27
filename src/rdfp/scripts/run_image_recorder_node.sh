#!/usr/bin/env bash

ros2 run rdfp image_recorder_node --ros-args -r image:=/camera_node/image_raw -p fps:=30 \
    -p auto_start:=true -p output_dir:="/home/kwlee/tmp/recordings"