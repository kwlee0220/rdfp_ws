#!/usr/bin/env bash
# 소스 트리의 YAML 을 그대로 써서 JGPC 판 풀 앱 스택을 띄운다 (재빌드 불필요).
#
# 설정이 둘로 나뉘어 있으므로 **양쪽 다** 지정해야 한다. 하나만 주면 나머지는
# 설치본(install/.../share/rdfp/config/)을 읽어, 소스를 고쳐도 반영되지 않는다.
# 두 YAML 모두 rdfp_panda_mock.launch.py 와 공유한다.

ros2 launch rdfp rdfp_panda_jgpc_mock.launch.py \
  config_file:=$PWD/src/rdfp/config/panda_robot.yaml \
  image_pipeline_config_file:=$PWD/src/rdfp/config/image_pipeline.yaml
