#!/usr/bin/env bash
# 컨테이너 진입점: apt ROS + rdfp 오버레이를 소싱한 뒤 인자로 받은 명령을 실행한다.
# (ws_moveit2 같은 소스 오버레이는 소싱하지 않는다 — apt-only 보장.)
set -e

source /opt/ros/humble/setup.bash
source /ws/install/setup.bash

exec "$@"
