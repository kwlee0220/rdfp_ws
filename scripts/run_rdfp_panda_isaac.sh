#!/usr/bin/env bash
# Isaac Sim 백엔드 + **수집 계층**까지 함께 띄운다.
#
# 제어 계층(run_panda_isaac.sh 가 띄우는 것) 전부에 더해:
#   session_control / target_joint_cmds_publisher / rdfp_image_viewer /
#   image_recorder
#
# **Isaac 이 먼저 떠서 Play 중이어야 하고 도메인을 맞춰야 한다** — 전제는
# run_panda_isaac.sh 의 주석을 본다 (`export ROS_DOMAIN_ID=31`).
#
# ⚠️ **카메라 노드를 띄우지 않는다.** Isaac 이 `/isaac/camera/image_raw` 를 직접 내고
# 레코더·뷰어가 그것을 구독한다. 해상도·FPS 기본값은 `config/isaac_scene.json` 의
# `camera` 블록에서 읽으므로, 바꾸려면 그 JSON 을 고치고 **`colcon build` 와
# `setup_graph.py` 재실행**을 해야 한다 — 인자로 덮어써도 Isaac 쪽은 안 바뀐다.
#
# ⚠️ **녹화 토픽 목록이 mock 과 다르다.** `config/recording_topics.list` 는 카메라가
# `/camera/image_raw` 라 Isaac 에 그대로 쓰면 **이미지가 한 장도 안 담긴다** —
# rosbag2 는 없는 토픽을 조용히 건너뛴다. mp4 는 launch 가 remap 해 주므로 증상이
# "mp4 는 있는데 rosbag 에 이미지가 없다"로만 보인다.
#
#   RECORDING_TOPICS_FILE=config/recording_topics_isaac.list ./scripts/record_rosbag.sh
#
# `dataset_config.yaml` 의 `topics_file` 도 같은 파일을 가리켜야 한다.
#
# ⚠️ **에피소드 라벨은 시작 *전에* 정한다.** `set_task_label` 은 `IN_EPISODE` 에서
# 거부되고, 빈 라벨로 시작한 에피소드는 **재적재로도 못 고친다** — bag 안의
# `/session` 메시지가 이미 비어 있기 때문이다.
#
#   ros2 service call /session_control/set_task_label rdfp_msgs/srv/SetString \
#     "{task_label: 'isaac_pick_block_a'}"
#
# 사용 예 — 전체 인자는 `--show-args` 로 본다.
#
#   ./scripts/run_rdfp_panda_isaac.sh
#   ./scripts/run_rdfp_panda_isaac.sh image_recorder_auto_start:=true
#   ./scripts/run_rdfp_panda_isaac.sh image_recorder_output_dir:=/data/recordings
#   ./scripts/run_rdfp_panda_isaac.sh enable_image_viewer_node:=false   # 화면 없는 곳
#   ./scripts/run_rdfp_panda_isaac.sh --show-args

ros2 launch rdfp rdfp_panda_isaac.launch.py "$@"
