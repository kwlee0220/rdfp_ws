#!/usr/bin/env bash
# DB 스키마를 비우고 **`FrameRouter` 가 만든 mp4**(`videos/`)를 지운다.
#
# ⚠️ **이것만으로는 판이 안 비워진다 — 세션 레코더의 mp4 가 남는다.**
#
#   이 스크립트가 지우는 `videos/` 는 **rosbag 의 이미지 토픽에서 mp4 를 뽑는**
#   `FrameRouter` 의 출력 자리다. 수집 경로는 그쪽이 아니라 **세션 레코더**이고
#   (`recordings/`), 녹화 목록에서 카메라 토픽을 뺐으므로 `videos/` 는 보통
#   **비어 있다.** 즉 이름과 달리 실질적으로 **DB 만 지운다.**
#
#   수집한 것을 전부 지우려면 덩어리가 **다섯**이다:
#     DB · 적재 대기 bag · 보관 bag · 레코더 mp4(`recordings/`) · 학습 샘플(`datasets/`)
#   절차는 docs/collect/dataset_postproc_user_guide.md **§8.6** 에 있다.
#
# 먼저 스택을 내린다 — 수집 노드가 살아 있으면 레코더가 지우는 족족 다시 쓴다.
set -euo pipefail

ros2 run rdfp init-db --drop --yes

RDFP_VIDEOS_DIR="${RDFP_VIDEOS_DIR:-$HOME/rdfp_data/videos}"
echo "Clearing video files in ${RDFP_VIDEOS_DIR}..."
rm -rf "${RDFP_VIDEOS_DIR:?}"/*

# 남은 것을 알려 준다 — "다 지웠다"고 오해하는 것이 이 스크립트의 함정이다.
data_root="${RDFP_DATA_ROOT:-$HOME/rdfp_data}"
for leftover in recordings datasets rosbag rosbag_imported; do
    dir="$data_root/$leftover"
    [[ -d "$dir" ]] || continue
    n="$(find "$dir" -type f 2>/dev/null | wc -l)"
    [[ "$n" -gt 0 ]] \
        && echo "[clear] 남아 있음: $dir ($n 파일, $(du -sh "$dir" | cut -f1))" >&2
done
echo "[clear] 전부 지우려면: docs/collect/dataset_postproc_user_guide.md §8.6" >&2
