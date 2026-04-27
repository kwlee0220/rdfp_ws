#!/usr/bin/env bash

# ros2 run rdfp rdfp_image_viewer_node
ros2 run rdfp rdfp_camera_node --ros-args -p camera_id:=4
