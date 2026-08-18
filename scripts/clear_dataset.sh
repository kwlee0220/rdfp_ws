#!/usr/bin/env bash

ros2 run rdfp init-db --drop --yes

RDFP_VIDEOS_DIR="${RDFP_VIDEOS_DIR:-$HOME/rdfp_data/videos}"
echo "Clearing video files in ${RDFP_VIDEOS_DIR}..."
rm -rf "${RDFP_VIDEOS_DIR:?}"/*