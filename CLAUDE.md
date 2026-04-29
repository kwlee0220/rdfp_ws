# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

ROS 2 (Humble) workspace for Franka Emika Panda robot development with MoveIt2. The single package `rdfp` (under `src/rdfp/`) covers: MoveIt2 launch stack, Cartesian planning, camera capture, MP4 image recording, session/episode lifecycle, rosbag → PostgreSQL/MP4 dataset ingestion, and dataset replay (with a Tk control GUI).

External MoveIt resources are pulled from `moveit_resources_panda` / `moveit_resources_panda_moveit_config`. Service/message types live in a sibling package `rdfp_msgs` (separate repo, must be built alongside).

## Build & Run

```bash
cd ~/development/ros/rdfp_ws

# Build (always run from workspace root, not from src/)
colcon build --packages-select rdfp_msgs rdfp
source install/setup.bash

# Launches — Panda + MoveIt2 stacks
ros2 launch rdfp panda_mock.launch.py            # MoveIt2 only
ros2 launch rdfp rdfp_panda_mock.launch.py       # + camera/ee_pose/recorder via YAML
ros2 launch rdfp replay_panda_mock.launch.py     # replay-mode stack (no joint_state_broadcaster)

# Launches — session/camera apps (no MoveIt)
ros2 launch rdfp rdfp.launch.py
ros2 launch rdfp rdfp_advanced.launch.py
```

`*_panda_mock.launch.py` files take a `config_file:=<path>` argument; default resolves to `<rdfp share>/config/rdfp_panda_mock.yaml` via `get_package_share_directory("rdfp")`. Source lives at `src/rdfp/config/`; setup.py glob's `config/*` into `share/rdfp/config/` at install time.

## Console Scripts

`setup.py` registers many entry points. The dataset CLI was historically a single `dataset` script with subcommands but has been split into independent top-level commands; **don't add new `dataset <sub>` subcommands** — add a sibling `*_cmd.py` module instead.

Dataset / rosbag CLIs (run from workspace root after sourcing):

```bash
ros2 run rdfp init-db --dsn-env RDFP_DB_DSN [--drop --yes]
ros2 run rdfp import   --config dataset_config.yaml
ros2 run rdfp stats    --config dataset_config.yaml
ros2 run rdfp list     --config dataset_config.yaml
ros2 run rdfp replay   42 --config dataset_config.yaml   # ROS-dep
ros2 run rdfp rosbag   list-episodes --config rosbag_config.yaml
```

Other notable scripts: `image_recorder_node`, `rdfp_image_recorder`, `camera_node`, `session_control_node`, `replay_gui` (Tk replay control GUI), `teleop_keyboard`, `session_teleop`, `target_joint_states_publisher`, `target_joint_states_executor`, `gripper_control_node`, `ee_pose_node`.

## Tests

```bash
# Full package
colcon test --packages-select rdfp
colcon test-result --verbose

# Single test directory / file (faster, doesn't require full ROS env for many tests)
cd src/rdfp
PYTHONPATH=. python3 -m pytest rdfp/dataset/tests/test_dataset_import_cmd.py -v
PYTHONPATH=. python3 -m pytest rdfp/dataset/tests/ -v
```

Test layout:
- `rdfp/dataset/tests/`, `rdfp/recorder/tests/`, `rdfp/camera/tests/`, `rdfp/rosbag/tests/` — pytest unit tests for each subpackage.
- `test/` (under `src/rdfp/`) — ament linter tests only (`test_copyright.py`, `test_flake8.py`, `test_pep257.py`).

ROS-dependent tests (anything that imports `rclpy`, `sensor_msgs`, `std_msgs`, etc.) require `source install/setup.bash` first or `colcon test`. Many `dataset/tests/*` deliberately avoid ROS imports so they run in pure-Python envs; tests that do need ROS use `pytest.importorskip` to self-skip.

## Coding Conventions

- 코드 주석(inline comments, docstring)은 한국어로 작성한다. 문장은 "~한다"(평서문) 형식을 사용한다.
- 로깅 메시지(`logger.info`, `logger.warning`, `logger.error` 등)는 영어로 작성한다.
- 예외 메시지(`ValueError`, `RuntimeError` 등)는 영어로 작성한다.
- Python 파일의 import 순서:
  1. `from __future__ import annotations` (항상 맨 처음)
  2. `from typing import ...` (다른 모든 import보다 우선)
  3. 표준 라이브러리 import
  4. 서드파티 라이브러리 import
  5. 로컬/프로젝트 import
- Type hint 는 built-in 우선: `tuple`/`list`/`dict` (대문자 generic 비권장),
  `Optional[X]` 권장 (`X | None` 비권장).
- 라인 길이 100자 제한 (신규 코드). 함수 호출/선언 인자는 한 줄로 이어 쓰고
  120자 초과 시에만 줄바꿈. **trailing comma 가급적 사용 금지** (Black 의
  magic trailing comma 동작에 의존하지 않음).

## Architecture

### Package layout — what each subpackage does

Top-level `rdfp/` (Python source root):

| Subpackage | Role |
|---|---|
| `moveit/` | `MoveGroupClient` (cartesian planning + execution), `ServoClient`, `ee_pose_publisher`, `gripper_*`, `target_joint_states_*` — direct MoveIt2 service/action wrappers. |
| `camera/` | `camera_node` (OpenCV → ROS), `image_viewer_node`, capture/reconnect helpers. |
| `recorder/` | `FFMpegMp4Recorder` (ffmpeg subprocess MP4 sink) + `image_recorder_node` (ROS adapter wrapping the recorder). |
| `session/` | `session_control_node` — state machine (IDLE → IN_SESSION → IN_EPISODE) on a transient-local topic so late subscribers see current state. |
| `teleop/` | `teleop_keyboard`, `session_teleop`. |
| `rosbag/` | rosbag2 MCAP catalog/discovery (`catalog.discover_splits`, `merged_stream`, `mcap_reader`) + `rosbag` CLI for inspecting splits/episodes without DB. |
| `dataset/` | DB schema + ingestion pipeline + replay. See "Dataset pipeline" below. Includes `replay_gui_cmd.py` — Tk GUI (`replay_gui` console_script) that orchestrates `Mp4ImageReplayer` + `TopicMessageReplayer` against a running MoveIt stack. |
| `samples/` | Manual sample/demo scripts (not entry_points). |

### Launch architecture (`launch/`)

Two families:
- **Panda + MoveIt2**: `panda_mock.launch.py` (controller-spawn chain), `rdfp_panda_mock.launch.py` (full app YAML-driven), `replay_panda_mock.launch.py` (replay variant — no joint_state_broadcaster).
- **Session/camera app**: `rdfp.launch.py`, `rdfp_advanced.launch.py`.

`*_launch_helper.py` modules carry shared argument declarations + `Node` factories so launch files compose them rather than duplicating. The Panda startup chain is intentionally sequential via `RegisterEventHandler(OnProcessExit)`: `ros2_control_node` → `joint_state_broadcaster` → `panda_arm_controller` → `panda_hand_controller` → (`move_group` + `servo` + `rviz` + camera + ee_pose). Three planning pipelines are loaded: OMPL, PILZ, CHOMP. See [src/rdfp/launch/README.md](src/rdfp/launch/README.md) for the full helper inventory.

### Dataset pipeline (`rdfp/dataset/`)

The dataset subsystem reads rosbag2 MCAP splits, segments them into episodes via `/session` state transitions (`IN_EPISODE → IN_SESSION`), and persists each episode atomically to PostgreSQL + MP4. **Episodes are identified by `(start_sec, start_nanosec)` (UNIQUE constraint); the surrogate `id` is `BIGSERIAL`.** Re-import of the same episode honors `on_existing_episode` policy (`skip`/`replace`/`error`, default `skip`); `replace` deletes via FK CASCADE and INSERTs a new id.

Module map:

| Path | Role |
|---|---|
| `import_cmd.py` | `import` console_script. Inlines the full pipeline (no `run_import` wrapper). Lazy-imports heavy ROS deps so the module loads without sourcing. |
| `replay_cmd.py` | `replay` console_script. Module top-level requires ROS sourced. |
| `stats_cmd.py` / `list_cmd.py` / `init_db_cmd.py` | thin standalone CLIs. |
| `cli_common.py` | shared argparse helpers (`add_common_args`, `add_config_arg`, `resolve_config_path`, `DEFAULT_CONFIG_FILENAME`, `load_dataset_or_fail`). |
| `config.py` | Pydantic v2 `DatasetConfig` (loaded from YAML). |
| `ingest/pipeline.py` | private pipeline helpers (`_run_parallel`, `_run_ingestion`, `_detect_all_episodes`, `_apply_episode_filters`, `_determine_topic_classification`, `_empty_summary`, `_delete_used_splits`). Exports only `PostProcError`. |
| `ingest/episode_worker.py` | per-episode transactional worker (`_open_episode` returns episode id). |
| `ingest/media/frame_router.py` + `ffmpeg_sink.py` | (episode, camera-topic) → MP4 sink, holds `image_streams`/`image_frames` rows. |
| `db/writers/` | per-table INSERTers (`session_command`, `image_stream`, `image_frame`, …). `INSERT … RETURNING id` is the source of episode_id. |
| `db/topic_message_replayer.py` | non-image stream replayer (k-way heap merge, single worker). **Rejects image topics** in `__init__` (use `Mp4ImageReplayer` for those). Has thread-safety hardening: `error` property, `join()` re-raises captured worker exception, `close()` order is `stop → stream.close → join → destroy_publisher`, skips publisher destroy if worker is still alive. |
| `db/mp4_image_replayer.py` | per-image-topic replayer (decoder thread + publisher thread + bounded queue with backpressure). Same defensive `close()` pattern. |

DSN comes from env var (default `RDFP_DB_DSN`) referenced by `db.dsn_env` in YAML — never store DSNs as plain text in config.

### Replay GUI (`rdfp/dataset/replay_gui_cmd.py`)

Tk-based control surface. Uses `Mp4ImageReplayer` for image topics and a single `TopicMessageReplayer` for non-image topics, started in lock-step with `start_time` / `first_history_time` anchors so cadence is preserved across replayers. The "Topics to replay" listbox starts with **all entries unchecked** (explicit selection required). The "위치 초기화" button calls `MoveGroupClient.move_to_named_target_async("ready")`.

## Important non-obvious behaviors

- **stale build artifacts inside `src/rdfp/`** (e.g., `src/rdfp/install/` or `src/rdfp/build/`) silently shadow the real workspace install via PYTHONPATH. If GUI/code changes don't take effect after rebuild, check `find src/rdfp -maxdepth 2 -name install -o -name build` and remove. Always run `colcon build` from the workspace root.
- **package share installs `config/*` only**, not the workspace-root YAML. If a launch can't find `rdfp_panda_mock.yaml`, copy/place it under `src/rdfp/config/` (so setup.py glob picks it up) or pass `config_file:=<absolute-path>`.
- `import_cmd.cmd_import` does the full ingestion inline — there is no `run_import` function. Tests mock `import_cmd.discover_splits` (top-level, light) rather than the heavier pipeline helpers (lazy-imported inside `cmd_import`).
- `replay_cmd.py` top-level imports `rclpy.node.Publisher` etc. → cannot import without ROS sourced. Tests use `pytest.importorskip` to self-skip in ROS-free envs.
- **Replayer lifecycle is one-shot**: calling `start()` twice on the same `TopicMessageReplayer` or `Mp4ImageReplayer` raises `RuntimeError('… already started')` even after the worker has exited cleanly (iterators are exhausted and message stamps mutated in-place; reuse would silently produce 0 publishes).
- **`/session` topic uses `TRANSIENT_LOCAL` durability** so late-joining recorders see the current state. Subscribers and `ros2 topic echo` must match this QoS.

## External docs

- `src/rdfp/README.md` — package usage walkthrough, MoveGroupClient API, image_recorder_node spec, session_control_node state machine, dataset post-processor pointers.
- `src/rdfp/launch/README.md` — launch-file/helper inventory.
- `src/rdfp/rdfp/recorder/README.md` and `docs/` — recorder details.
- `docs/rosbag2/` (workspace root) — dataset post-processor user/design/runbook docs and sample YAML configs (referenced from package README).
