#!/usr/bin/env bash
# usage: kill_topic_procs.sh <topic-name>
# 예) ./kill_topic_procs.sh /session
set -euo pipefail

topic="${1:?usage: $0 <topic>}"

# 1) 토픽 엔드포인트(pub + sub)의 ROS 노드 이름 수집
nodes=$(ros2 topic info "$topic" -v 2>/dev/null \
          | awk '/^Node name:/{print $3}' | sort -u)

if [[ -z "$nodes" ]]; then
    echo "no endpoints on '$topic'"; exit 0
fi

# 2) launch 로 기동된 노드는 cmdline 에 `__node:=<name>` 가 포함된다.
#    ros2 run 으로 띄운 경우에도 대부분 executable 이 --ros-args 블록에서
#    동일 이름을 사용하므로 같은 정규식으로 매칭된다.
pids=$(for n in $nodes; do
    pgrep -f "__node:=${n}(\s|$)" || true
done | sort -u)

if [[ -z "$pids" ]]; then
    echo "no OS processes matched for nodes: $nodes"; exit 0
fi

echo "Nodes: $nodes"
echo "SIGTERM -> $pids"
kill $pids || true
sleep 1

# 3) 남아있으면 SIGKILL
for pid in $pids; do
    if kill -0 "$pid" 2>/dev/null; then
        echo "SIGKILL -> $pid"
        kill -9 "$pid" || true
    fi
done
