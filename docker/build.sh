#!/usr/bin/env bash
# rdfp Docker 이미지 빌드: base → 앱 이미지들.
# 빌드 컨텍스트는 워크스페이스 루트(스크립트 상위 디렉터리)다.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WS_ROOT}"

echo "[build] context: ${WS_ROOT}"
echo "[build] 1/6 rdfp-base"
docker build -f docker/Dockerfile.base            -t rdfp-base:latest .
echo "[build] 2/6 rdfp-panda-mock"
docker build -f docker/Dockerfile.panda_mock      -t rdfp-panda-mock:latest .
echo "[build] 3/6 rdfp-panda-jgpc-mock"
docker build -f docker/Dockerfile.panda_jgpc_mock -t rdfp-panda-jgpc-mock:latest .
echo "[build] 4/6 rdfp-gazebo"
docker build -f docker/Dockerfile.gazebo          -t rdfp-gazebo:latest .
echo "[build] 5/6 rdfp-replay-mock"
docker build -f docker/Dockerfile.replay_mock     -t rdfp-replay-mock:latest .
echo "[build] 6/6 rdfp-replay-gui"
docker build -f docker/Dockerfile.replay_gui      -t rdfp-replay-gui:latest .
echo "[build] done: rdfp-base / rdfp-panda-mock / rdfp-panda-jgpc-mock / rdfp-gazebo / rdfp-replay-mock / rdfp-replay-gui"
