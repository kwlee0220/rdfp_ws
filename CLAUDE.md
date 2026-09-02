# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

ROS 2 (Humble) workspace for Franka Emika Panda robot development with MoveIt2, and the "real/virtual robot environment" subsystem of an imitation-learning data platform (see [docs/rdfp_framework_design.md](docs/rdfp_framework_design.md)).

**Four code packages, split along a layer boundary** (plus `panda_ftsensor_robotiq`, an asset-only description package). The lower layer is usable standalone as a plain ROS 2 robot-control stack; the upper layer adds learning-data collection on top.

| package | layer | contents | depends on |
|---|---|---|---|
| `rdfp_msgs` | — | msg/srv IDL (separate repo, build first) | — |
| `panda_ftsensor_robotiq` | — | target-robot description (vendor URDF + Robotiq meshes); assets only, no code | — |
| `robot_control` | **control (lower)** | `moveit/` `camera/` `scene/`, shared `ros2_utils`/`types`/`logging_bridge`, `launch_helpers/`, non-`rdfp_`-prefixed launches, robot description | `rdfp_msgs` |
| `robot_twin` | control | REST gateway (`robot_twin`) exposing the control layer as variables/operations | `robot_control` |
| `rdfp` | **collection (upper)** | `session/` `recorder/` `camera/` `dataset/` `rosbag/` `teleop/`, `rdfp_*` launches, replay | `robot_control`, `rdfp_msgs` |

**The dependency direction is one-way and enforced by tests.** `robot_control` / `robot_twin` must never import `rdfp` — `robot_control/tests/test_layer_boundary.py` and `robot_twin/tests/test_layer_boundary.py` fail the build if they do. When the lower layer needs something from the upper layer, use the entry-point seam (see `robot_twin/backend_registry.py`), not an import.

External MoveIt resources are pulled from `moveit_resources_panda` / `moveit_resources_panda_moveit_config`.

## Build & Run

```bash
cd ~/development/ros/rdfp_ws

# Build (always run from workspace root, not from src/)
colcon build --packages-select rdfp_msgs robot_control robot_twin rdfp
source install/setup.bash

# Launches — Panda + MoveIt2 stacks
ros2 launch robot_control panda_mock.launch.py    # MoveIt2 only (control layer)
ros2 launch rdfp rdfp_panda_mock.launch.py       # + camera/ee_pose/recorder via YAML
ros2 launch rdfp rdfp_panda_isaac.launch.py      # Isaac Sim backend + collection layer
ros2 launch rdfp replay_panda_mock.launch.py     # replay-mode stack (see replay_arm_path)

# Launches — session/camera apps (no MoveIt)
ros2 launch rdfp rdfp.launch.py
ros2 launch rdfp rdfp_advanced.launch.py

# Robot twin (REST gateway; control layer only — session ops need `rdfp` installed)
ros2 run robot_twin robot_twin --config <path>
```

YAML-driven launches take a `config_file:=<path>` argument. **The default resolves against the package that owns the file, which is not always the package that owns the launch** — `rdfp_panda_jgpc_mock` (in `rdfp`) reads its controllers YAML from `robot_control`'s share. Each package's setup.py globs `config/*` into `share/<pkg>/config/`.

| config | owning package | read by | argument |
|---|---|---|---|
| `config/image_pipeline.yaml` | `robot_control` | `rdfp_panda_mock`, `rdfp_panda_jgpc_mock` | `image_pipeline_config_file` |
| | | `rdfp`, `rdfp_advanced` | `config_file` |
| | | `panda_mock`, `panda_jgpc_mock` | (defaults only, not swappable) |
| `config/panda_jgpc_ros2_controllers.yaml` | `robot_control` | `panda_jgpc_mock`, `rdfp_panda_jgpc_mock` | (fixed) |
| `config/panda.rviz`, `description/*` | `robot_control` | all Panda launches | (fixed) |
| `config/panda_robot.yaml` | `rdfp` | `rdfp_panda_mock`, `rdfp_panda_jgpc_mock` | `config_file` |
| `config/replay_panda_mock.yaml` | `rdfp` | `replay_panda_mock` | `config_file` |
| `config/teleop_mirror.yaml` | `rdfp` | `teleop_mirror` | `config_file` |
| `config/isaac_scene.json` | `robot_control` | `rdfp_panda_isaac` (카메라 기본값), Isaac 쪽 스크립트, `isaac_scene_state_node` | (fixed) |
| `config/robot_twin_panda01.yaml` | `robot_twin` | `robot_twin` (mock 스택) | `--config` |
| `config/robot_twin_panda_isaac.yaml` | `robot_twin` | `robot_twin` (Isaac 스택) | `--config` |

`image_pipeline.yaml` is the **single source for camera/viewer/recorder settings** — it exists because the app launches used to hardcode `640x480`/`id: 4` in `camera_launch_helper` while the panda launches read `1280x720`/an mp4 path from YAML. `robot_control.launch_helpers.image_pipeline` owns the loading; `launch_helpers.camera.declare_camera_arguments()` is a thin wrapper over it.

Each YAML holds **only keys with a live consumer** — a missing block means that launch doesn't start the corresponding node. Passing the wrong YAML fails fast (`KeyError`) rather than silently defaulting. Key ↔ argument mapping tables: [src/rdfp/launch/README.md](src/rdfp/launch/README.md) §3 (수집 계열) / [src/robot_control/launch/README.md](src/robot_control/launch/README.md) §2 (제어 계열).

## Console Scripts

Entry points are split across the three Python packages — **`ros2 run <pkg> <script>` needs the right package**:

| package | scripts |
|---|---|
| `robot_control` | `camera_node`, `image_capture_node`, `image_viewer_node`, `ee_pose_node`, `ee_twist_node`, `servo_auto_start_node`, `gripper_action_node`, `mock_scene_state_node`, `isaac_gripper_bridge`, `isaac_scene_state_node`, `isaac_servo_bridge`, `target_joint_cmds_publisher`, `target_joint_cmds_executor`, `target_joint_states_publisher`, `target_joint_states_executor` |
| `robot_twin` | `robot_twin` |
| `rdfp` | `rdfp_camera_node`, `rdfp_image_viewer_node`, `session_control_node`, `image_recorder_node`, `rdfp_image_recorder`, `teleop_keyboard`, `session_teleop`, `teleop_retarget`, `clutch_pedal` (USB 풋페달 → 클러치; `python3-evdev` 필요), `import`, `replay`, `stats`, `list`, `init-db`, `rosbag`, `replay_gui` |

The dataset CLI was historically a single `dataset` script with subcommands but has been split into independent top-level commands; **don't add new `dataset <sub>` subcommands** — add a sibling `*_cmd.py` module instead.

Dataset / rosbag CLIs (run from workspace root after sourcing):

```bash
ros2 run rdfp init-db --dsn-env RDFP_DB_DSN [--drop --yes]
ros2 run rdfp import   --config dataset_config.yaml
ros2 run rdfp stats    --config dataset_config.yaml
ros2 run rdfp list     --config dataset_config.yaml
ros2 run rdfp replay   42 --config dataset_config.yaml   # ROS-dep
ros2 run rdfp rosbag   list-topics --rosbag-dir /data/rosbag   # 서브커맨드는 list-topics / clear 뿐
```

## Tests

```bash
# Full package
colcon test --packages-select robot_control robot_twin rdfp
colcon test-result --verbose

# Single test directory / file (faster than colcon test)
cd src/rdfp
PYTHONPATH=.:$PYTHONPATH python3 -m pytest rdfp/dataset/tests/test_dataset_import_cmd.py -v
PYTHONPATH=.:$PYTHONPATH python3 -m pytest rdfp/dataset/tests/ -v

# Control-layer / twin suites live in their own packages
cd src/robot_control && PYTHONPATH=.:$PYTHONPATH python3 -m pytest robot_control/scene/tests/ -v
cd src/robot_twin   && PYTHONPATH=.:$PYTHONPATH python3 -m pytest robot_twin/tests/ -v
```

**Append `:$PYTHONPATH` — do not write `PYTHONPATH=.` alone.** A bare `PYTHONPATH=.`
*replaces* the ROS-provided entry, so `sensor_msgs` / `rclpy` become unimportable even in a
shell where `install/setup.bash` was sourced, and ROS-dependent modules fail at collection
with `ModuleNotFoundError`. Appending keeps both.

Test layout — colocated suites across three packages, **869 tests total** (`robot_control` 240 / `robot_twin` 227 / `rdfp` 402):

| package | suites |
|---|---|
| `robot_control` | `robot_control/moveit/tests/`, `robot_control/scene/tests/`, `robot_control/isaac/tests/`, `robot_control/functionbay/tests/`, `robot_control/tests/` (layer boundary + Isaac 쪽 스크립트 경로 계약) |
| `robot_twin` | `robot_twin/tests/` (incl. layer boundary) |
| `rdfp` | `rdfp/camera/tests/`, `rdfp/dataset/tests/`, `rdfp/recorder/tests/`, `rdfp/rosbag/tests/`, `rdfp/teleop/tests/` |

**`test_layer_boundary.py` in `robot_control` and `robot_twin` is what keeps the split real.** It AST-parses every source file in the package and fails if any imports an upper-layer package, **or imports `SessionCommand` from `rdfp_msgs`** — `rdfp_msgs` is an IDL package so the import-root check alone can't see a session-aware node sitting in the control layer (that is exactly how `rdfp_camera_node` / `rdfp_image_viewer_node` ended up there before moving to `rdfp/camera/`). `package.xml` dependencies only order the build; they do not stop an import in the wrong direction. If you need something from the upper layer, add an entry point to the `robot_twin.backends` group (see `robot_twin/backend_registry.py` and `rdfp/session/twin_backend.py`) — do not import.

**There is no `test/` directory and the ament linters never run.** `package.xml` still declares `ament_copyright` / `ament_flake8` / `ament_pep257` as `test_depend` and `setup.py` still does `find_packages(exclude=['test'])`, but the three linter test files are absent — so `colcon test` exercises only the suites above. Consequence: style violations accumulate unchecked (there are pre-existing `F401` / `E702` / `E125` hits under `dataset/tests/`). Run flake8 by hand on files you touch until the linters are restored.

ROS-dependent tests (anything that imports `rclpy`, `sensor_msgs`, `std_msgs`, etc.) require `source install/setup.bash` first or `colcon test`. Many `dataset/tests/*` deliberately avoid ROS imports so they run in pure-Python envs.

Every ROS-dependent module now self-skips, so `rdfp/dataset/tests/` is green in both environments — **101 passed / 8 skipped** without ROS, **154 passed** with it. Keep it that way when adding tests: guard with `pytest.importorskip('<ros pkg>')` **above** the `rdfp.*` imports (marking the shadowed imports `# noqa: E402`), or `pytest.mark.skipif` a single test when the rest of the module is ROS-free. Without a guard the module raises at collection and **aborts the entire run**, not just itself.

The dependency is often *transitive* — e.g. `ingest/test_filters.py` tests pure-Python filter logic but `pipeline` pulls in `sensor_msgs`, and `test_topic_message_replayer.py` stubs every ROS object yet the module under test imports `builtin_interfaces` at top level.

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

Grouped by owning package. The `robot_control` block is the standalone robot-control stack; the `rdfp` block is what makes it a learning-data platform.

**`robot_control`** (`src/robot_control/robot_control/`) — plus shared `ros2_utils.py` / `types.py` / `logging_bridge.py` used by both layers, and `launch_helpers/` (installed module, imported by launches in *both* packages).

| Subpackage | Role |
|---|---|
| `moveit/` | `MoveGroupClient` — **abstract** base (cartesian/named-target *planning* + SRDF queries) with two execution implementations: `MoveGroupJtcClient` (MoveGroup/ExecuteTrajectory actions) and `MoveGroupJgpcClient` (Float64MultiArray command streaming). Build one via `create_move_group_client(node)`. Also `TrajectoryStreamer`, `ServoClient`, `ee_pose_publisher`, `gripper_*`, `target_joint_cmds_*` (arm 명령 ↔ `sensor_msgs/JointState` 어댑터; 구형 `target_joint_states_*` 는 구현만 남고 launch 에서는 제외됨) — direct MoveIt2 service/action wrappers. |
| `camera/` | `camera_node` (OpenCV → ROS), `image_capture_node` (JPEG `CompressedImage` 전용, `ReconnectingCamera` 로 재연결), `image_viewer_node`, `OpenCvCamera` / capture helpers. **세션을 아는 노드는 여기 없다** — `rdfp_camera_node` / `rdfp_image_viewer_node` 는 `rdfp/camera/` 소속이다. |
| `scene/` | scene 물체 상태를 `/scene/objects` (`rdfp_msgs/SceneObjects`) 로 발행한다. **조작 대상만 싣는다.** 백엔드마다 노드가 하나씩이며 현재는 `mock_scene_state_node` (MoveIt planning scene 폴링)뿐이다. 좌표계·단위·쿼터니언 순서를 맞추는 책임이 전부 여기 있다 — `pose_math.py` 는 그 합성을 ROS 없이 테스트할 수 있게 분리한 순수 함수다. |

**`robot_twin`** (`src/robot_twin/robot_twin/`) — REST gateway. `config.py` (declarative variables/operations YAML; `moveit.arm_command_*` 로 백엔드별 명령 채널을 고른다 — Isaac 은 `/isaac/arm_command` + `joint_state`), `runtime.py` (ROS wiring), `backends.py` (operation handlers), `api.py` (FastAPI), `backend_registry.py` (entry-point seam for upper-layer backends).

**`rdfp`** (`src/rdfp/rdfp/`) — collection layer.

| Subpackage | Role |
|---|---|
| `camera/` | 세션 인지 카메라 어댑터. `rdfp_camera_node` (`/session` 이 `IN_EPISODE` 일 때만 이미지 발행, 끊기면 `reconnect_interval_sec` 주기로 재연결) 와 `rdfp_image_viewer_node` (프레임에 세션 상태 오버레이). 하드웨어 추상화는 `robot_control.camera.opencv_camera` 를 그대로 쓴다. |
| `session/` | `session_control_node` — state machine (IDLE → IN_SESSION → IN_EPISODE) on a transient-local topic so late subscribers see current state. Also `twin_backend.py`, registered into `robot_twin.backends` so the twin can offer session/episode operations without depending on this package. |
| `recorder/` | `FFMpegMp4Recorder` (ffmpeg subprocess MP4 sink, ROS-agnostic core). Two ROS adapters: `image_recorder_node` (service-driven start/stop) and `rdfp_image_recorder` (auto-recording driven by `/session` state, timestamp-based segmentation with a `pending_image_queue`; also writes `.jsonl` sidecar + `metadata.json`). |
| `teleop/` | `teleop_keyboard`, `session_teleop`, `teleop_retarget` (클러치 앵커 기반 leader→follower 상대 매핑), `clutch_pedal` (USB HID 풋페달 → 클러치, evdev), `ClutchClient` (클러치 서비스·상태 래퍼). |
| `rosbag/` | rosbag2 MCAP catalog/discovery (`catalog.discover_splits`, `merged_stream`, `mcap_reader`) + `rosbag` CLI for inspecting splits/episodes without DB. |
| `dataset/` | DB schema + ingestion pipeline + replay. See "Dataset pipeline" below. Includes `replay_gui_cmd.py` — Tk GUI (`replay_gui` console_script) that orchestrates `Mp4ImageReplayer` + `TopicMessageReplayer` against a running MoveIt stack. |
| `samples/` | Manual sample/demo scripts (not entry_points). |

### Launch architecture

Launch files are split by layer, helpers are not:

| package | launches |
|---|---|
| `robot_control/launch/` | `panda_mock`, `panda_jgpc_mock`, `panda_gazebo`, `panda_functionbay`, `panda_isaac` — robot control only, no session/recording |
| `rdfp/launch/` | `rdfp_panda_mock`, `rdfp_panda_jgpc_mock`, `rdfp_panda_gazebo`, `rdfp_panda_isaac`, `rdfp_panda_functionbay`, `rdfp`, `rdfp_advanced`, `rdfp_collect`, `replay_panda_mock`, `teleop_mirror` |

**Helpers live in `robot_control/robot_control/launch_helpers/` as an installed Python module** and are imported normally by launches in both packages:

```python
from robot_control.launch_helpers.camera import create_camera_node
```

They used to be sibling files in `launch/` pulled in via `sys.path.insert(0, os.path.dirname(__file__))`. That trick only works inside one directory, so it could not survive the package split — the `rdfp_*` launches must import helpers that live in another package. **Do not reintroduce the `sys.path` pattern.**

**`rdfp_collect.launch.py` starts the four collection nodes alone**, to be layered on an already-running control stack. `panda_mock` + `rdfp_collect` produces a node graph identical to `rdfp_panda_mock` (verified by diffing node lists, all collection-node parameters, and topic endpoints on an isolated domain); `panda_jgpc_mock` + `rdfp_collect arm_cmd_source:=float64_multi_array` likewise. The **Gazebo pair is not equivalent by default** — `rdfp_panda_gazebo` hardcodes backend-tied values (`camera_resolution: 640x480` from the gz sensor in `panda.gazebo.xacro`, viewer/recorder off because `simulate_camera` defaults false) instead of reading `image_pipeline.yaml`, so four arguments must be passed explicitly. Recipes: [src/rdfp/launch/README.md](src/rdfp/launch/README.md) §6.1.

Splitting is possible because **none of the four collection nodes has a hard startup dependency** — all are subscribers / state machines / service servers. The one client (`target_joint_cmds_publisher` querying the arm controller's `joints` param) runs on a timer with `joint_names_timeout` (10 s) and degrades to an empty `JointState.name`, and the split ordering (control stack first) is if anything safer than the bundled one. **The bundled launches are still the default**: they declare shared arguments (`camera_image_topic`, `camera_resolution`, `camera_fps`) in one place, so the two halves cannot silently disagree — with the split that coherence becomes the user's responsibility.

`rdfp_panda_*` launches remain supersets of their `panda_*` counterparts: same helpers, same startup chain, plus the collection nodes (`session_control`, `image_recorder`, `rdfp_image_viewer`, `target_joint_cmds_publisher`) and YAML-driven argument defaults. The Panda startup chain is intentionally sequential via `RegisterEventHandler(OnProcessExit)`: `ros2_control_node` → `joint_state_broadcaster` → `panda_arm_controller` → `panda_hand_controller` → (`move_group` + `servo` + `rviz` + camera + ee_pose + scene). Three planning pipelines are loaded: OMPL, PILZ, CHOMP. Full helper inventory: [src/robot_control/launch/README.md](src/robot_control/launch/README.md) §5. Collection-layer launches: [src/rdfp/launch/README.md](src/rdfp/launch/README.md).

The **backend scene node starts by default** in all four mock launches (`panda_mock`, `panda_jgpc_mock`, `rdfp_panda_mock`, `rdfp_panda_jgpc_mock`) via `launch_helpers.scene`; `enable_scene:=false` turns it off. Default-on is deliberate: the node costs a 2 Hz timer and mutates nothing until a `/scene/reset` request arrives, whereas leaving it off makes the twin's `reset_scene` fail on a missing service with nothing in the log pointing at the absent server. `replay_panda_mock` deliberately does **not** get one — replayed object state must come from the dataset, and a live scene node would publish a second, conflicting source.

### Dataset pipeline (`rdfp/dataset/`)

The dataset subsystem reads rosbag2 MCAP splits, segments them into episodes via `/session` state transitions (`IN_EPISODE → IN_SESSION`), and persists each episode atomically to PostgreSQL + MP4. **Episodes are identified by `(start_sec, start_nanosec)` (UNIQUE constraint); the surrogate `id` is `BIGSERIAL`.** Re-import of the same episode honors `on_existing_episode` policy (`skip`/`replace`/`error`, default `skip`); `replace` deletes via FK CASCADE and INSERTs a new id.

Module map:

| Path | Role |
|---|---|
| `import_cmd.py` | `import` console_script. Inlines the full pipeline (no `run_import` wrapper). Lazy-imports heavy ROS deps so the module loads without sourcing. |
| `replay_cmd.py` | `replay` console_script. Module top-level requires ROS sourced. |
| `stats_cmd.py` / `list_cmd.py` / `init_db_cmd.py` | thin standalone CLIs. |
| `cli_common.py` | shared argparse helpers (`add_common_args`, `add_config_arg`, `resolve_config_path`, `DEFAULT_CONFIG_FILE_PATH`, `load_dataset_or_fail`). |
| `config.py` | Pydantic v2 `DatasetConfig` (loaded from YAML). |
| `ingest/pipeline.py` | private pipeline helpers (`_run_parallel`, `_run_ingestion`, `_detect_all_episodes`, `_apply_episode_filters`, `_determine_topic_classification`, `_empty_summary`, `_delete_used_splits`). Exports only `PostProcError`. |
| `ingest/episode_worker.py` | per-episode transactional worker (`_open_episode` returns episode id). |
| `ingest/media/frame_router.py` + `ffmpeg_sink.py` | (episode, camera-topic) → MP4 sink. Inserts one `image_streams` row per produced mp4 + one `image_frames` row per frame. Exposes `consume_inserted_count() -> {'image_streams': N, 'image_frames': M}` (mirrors `WriterBase`), which the pipeline aggregates into the `summary['inserted']` table counts — without it, image rows would be silently missing from the summary even though DB has them. |
| `db/writers/` | per-table INSERTers (`session_command`, `image_stream`, `image_frame`, …). `INSERT … RETURNING id` is the source of episode_id. |
| `db/topic_message_replayer.py` | non-image stream replayer (k-way heap merge, single worker). **Rejects image topics** in `__init__` (use `Mp4ImageReplayer` for those). Has thread-safety hardening: `error` property, `join()` re-raises captured worker exception, `close()` order is `stop → stream.close → join → destroy_publisher`, skips publisher destroy if worker is still alive. |
| `db/mp4_image_replayer.py` | per-image-topic replayer (decoder thread + publisher thread + bounded queue with backpressure). Same defensive `close()` pattern. |

DSN comes from env var (default `RDFP_DB_DSN`) referenced by `db.dsn_env` in YAML — never store DSNs as plain text in config.

### Replay GUI (`rdfp/dataset/replay_gui_cmd.py`)

Tk-based control surface. Uses `Mp4ImageReplayer` for image topics and a single `TopicMessageReplayer` for non-image topics, started in lock-step with `start_time` / `first_history_time` anchors so cadence is preserved across replayers. The "Topics to replay" listbox starts with **all entries unchecked** (explicit selection required). The "위치 초기화" button calls `move_to_named_target_async("ready")` on the client built by `create_move_group_client()`; the JTC/JGPC choice happens once at construction, not per call.

## Important non-obvious behaviors

- **`MoveGroupClient` is abstract — `MoveGroupClient(node)` raises `TypeError`.** Build clients with `create_move_group_client(node, mode='auto'|'jtc'|'jgpc')`, which picks `MoveGroupJtcClient` or `MoveGroupJgpcClient` by probing `/panda_arm_controller/commands`. Consequence of `mode='auto'`: detection is a topic-graph lookup, so calling it before DDS discovery settles can mis-detect a JGPC stack as JTC — pass an explicit `mode` when you already know. `execute_trajectory()` exists only on the JTC client; `stream_trajectory()` / `*_streamed()` / `stop_streaming()` only on the JGPC one.
- **Cartesian trajectories from MoveIt are always resampled to ~10 Hz** (TOTG `resample_dt = 0.1`), and `GetCartesianPath` exposes no field to change it — shrinking `max_step` does not increase point density. JTC hides this because the controller interpolates between points; the streaming path does not, so JGPC command output is stepped at 10 Hz unless you pass `publish_rate=` (e.g. `200.0`) to `stream_trajectory()` / `follow_trajectory()` / `TrajectoryStreamer.stream()`, which linearly interpolates positions onto a uniform grid. Default `publish_rate=None` preserves the original point-time behavior.
- **stale build artifacts inside `src/<pkg>/`** (e.g., `src/rdfp/install/` or `src/rdfp/build/`) silently shadow the real workspace install via PYTHONPATH. If GUI/code changes don't take effect after rebuild, check `find src -maxdepth 2 \( -name install -o -name build \)` and remove. Always run `colcon build` from the workspace root.
- **`ros2 run rdfp <script>` fails for control-layer scripts** — they moved to `robot_control` (see the Console Scripts table). Same for `ros2 launch rdfp panda_mock.launch.py` → `ros2 launch robot_control panda_mock.launch.py`.
- **`ros2 run robot_control rdfp_camera_node` / `rdfp_image_viewer_node` also fails** — these two went the *other* way, from `robot_control/camera/` to `rdfp/camera/`, because they are driven by `/session`. Use `ros2 run rdfp <script>`; launches must spawn them with `package="rdfp"`.
- **A new `robot_control` module that imports `rdfp` breaks the build**, not at runtime but in `colcon test` via `test_layer_boundary.py`. That is intentional — the split exists to keep the control stack usable without the collection layer. Use the `robot_twin.backends` entry-point group instead.
- **package share installs `config/*` only**, not the workspace-root YAML. If a launch can't find `panda_robot.yaml`, copy/place it under `src/rdfp/config/` (so setup.py glob picks it up) or pass `config_file:=<absolute-path>`.
- `import_cmd.cmd_import` does the full ingestion inline — there is no `run_import` function. Tests mock `import_cmd.discover_splits` (top-level, light) rather than the heavier pipeline helpers (lazy-imported inside `cmd_import`).
- `replay_cmd.py` top-level imports `rclpy.node.Publisher` etc. → cannot import without ROS sourced. Tests use `pytest.importorskip` to self-skip in ROS-free envs.
- **Replayer lifecycle is one-shot**: calling `start()` twice on the same `TopicMessageReplayer` or `Mp4ImageReplayer` raises `RuntimeError('… already started')` even after the worker has exited cleanly (iterators are exhausted and message stamps mutated in-place; reuse would silently produce 0 publishes).
- **`move_to_joints` fills unspecified joints with their current value.** Only the joints you pass get a `JointConstraint`, so a partial goal would otherwise define a *set* of poses and the planner picks one arbitrarily — measured: passing `panda_joint1` alone swung the other six joints by up to 3.5 rad and put the EE behind/above the robot. `MoveGroupClient._complete_joint_values()` now reads the group's joints from the SRDF `<group_state>` (already cached for `move_to_named_target`) and the current values from `/joint_states`, then constrains every group joint. Joints outside the group (`panda_finger_*`) are deliberately not filled — mixing them in makes planning fail. A group with no `<group_state>` raises rather than silently falling back to the old behavior.
- **`moveit_servo` silently ignores all input until `start_servo` is called** — no error, no warning, just no motion. `replay_panda_mock.launch.py replay_arm_path:=ee_twist` spawns `servo_auto_start_node` to cover this; any other path that feeds `/servo_node/delta_twist_cmds` must call the `std_srvs/Trigger` service itself. `ServoClient.auto_start()`'s built-in service wait is only 3 s, which is why `servo_auto_start_node` prepends its own (`service_timeout`, default 30 s) — spawning it alongside `servo_node` in a launch would otherwise race.
- **`replay_arm_path` picks exactly one arm adapter** in `replay_panda_mock.launch.py`: `ee_twist` (**default**, `ee_twist_publisher` + `servo_auto_start`, velocity **open loop** — integration drift, start-pose dependent, 6-DOF only), `target_joint_cmds` (`target_joint_cmds_executor`, position closed-loop, redundancy preserved — highest fidelity), or `none`. They are mutually exclusive because both would write `/panda_arm_controller/joint_trajectory`. The `replay` CLI's `--topic` default is `/servo_node/delta_twist_cmds`, which matches *neither* path — pass `/ee_pose` (default path) or `/target_joint_cmds` explicitly. Rationale: [docs/replay/replay_mock_stack_guide.md](docs/replay/replay_mock_stack_guide.md).
- **The `joint_states` table does not store joint names** (columns are `position`/`velocity`/`effort` only), so `sensor_msgs/JointState` replayed from the DB arrives with an empty `name`. `target_joint_cmds_executor` falls back to its `joint_names` parameter in that case (message `name` wins when present); the replay launch passes panda_joint1~7. Related trap: declaring a STRING_ARRAY parameter with `[]` as the default makes rclpy infer `BYTE_ARRAY` (`all(isinstance(v, bytes) for v in [])` is vacuously true) and a later `STRING_ARRAY` set silently fails — declare with `Parameter.Type.STRING_ARRAY` (type only) and read via `get_parameter_or`.
- **`panda_jgpc_mock.launch.py` swaps the arm controller type only** (`panda_arm_controller` stays the same *name*, becomes `position_controllers/JointGroupPositionController` via `config/panda_jgpc_ros2_controllers.yaml`). Consequences: `/panda_arm_controller/joint_trajectory` is replaced by `/panda_arm_controller/commands` (`std_msgs/Float64MultiArray`, **no joint names — array order must match the controller's `joints` param**); `move_group` plan & execute for the arm no longer works because `moveit_simple_controller_manager` only speaks `FollowJointTrajectory`/`GripperCommand`. Planning itself is unaffected. `create_move_group_client(node)` detects this stack (by the presence of `/panda_arm_controller/commands`) and returns `MoveGroupJgpcClient`, whose `move_to_named_target()` / `follow_trajectory()` plan with MoveIt and then stream commands — so call sites need no branching. The streaming path is **open loop**: a normal return does not mean the target was reached. The launch also overrides servo's `command_out_type` to `std_msgs/Float64MultiArray` — and **must** set `publish_joint_velocities: false`, since servo's parameter validation returns `nullopt` (node fails to start) when both positions and velocities are published in that mode.
- **`teleop_retarget` 의 `pedal_timeout` 은 기본 0(비활성)** — 켜지 않으면 풋페달 hold 모드가 데드맨으로 동작하지 않는다. 페달 노드가 크래시하거나 USB 가 빠지면 disengage 를 보낼 주체가 사라져 클러치가 물린 채로 남기 때문. `pedal_timeout > 0` 이면 하트비트(`~/pedal_heartbeat`) 가 끊길 때 자동 해제하며, **하트비트 없이 engage 한 경우도 즉시 해제된다**. 클러치 상태는 `~/clutch_state` (`rdfp_msgs/ClutchState`, TRANSIENT_LOCAL) 구독 또는 `~/get_clutch_state` (`std_srvs/Trigger`) 조회로 확인한다 — `~/clutch` 는 SetBool 이라 호출 자체가 상태를 바꾸므로 조회에 쓸 수 없다. 코드에서는 `ClutchClient` 를 쓴다(QoS·비동기 처리를 캡슐화).
- **그리퍼 상태를 `/joint_states` 에서 읽으면 안 된다 — `/gripper_states` 를 쓴다.** mock·Isaac 은 손가락 관절을 실제로 싣지만 **펑션베이는 TF 성립용 고정값(`0.04`)을 주입**해 "항상 열려 있다"고 거짓말을 한다. `GripperState.width` 는 관절값이 아니라 **개구 폭(m)** 이라 (Panda 는 관절값의 2배) `scene/objects` 치수와 직접 비교되며, 못 구하면 **`NaN`** 이다 (0 은 '닫혀 있다'는 거짓말이 된다). 설계: [docs/moveit/GripperNode_Design.md](docs/moveit/GripperNode_Design.md).
- **`GripperCommand` 는 심볼만 싣는다 (`goal`: open/close/grasp).** `position`/`max_effort` 는 2026-09-01 에 제거했다 — 그리퍼 종속이라 다른 기구로 옮기면 틀린 값이 된다. 숫자는 `GripperNode` 의 `targets.<goal>` 파라미터가 갖는다. **명령은 심볼, 관측은 물리량**이라는 비대칭이 의도적이다. 모르는 심볼은 노드가 **거부**한다 (조용히 무시하면 팔만 움직이고 원인이 안 보인다).
- **`GripperState.at_goal` 이 성공 판정이다 — `stalled` 은 관측일 뿐이다.** 판정식이 `goal` 마다 다르고 (`open`/`close` = 목표 폭 도달 AND NOT `stalled`, `grasp` = `stalled`) 노드가 이미 적용하므로 **소비자는 `at_goal` 하나만 본다.** `grasp` 는 목표 폭에 닿으면 오히려 실패(헛닫힘)이므로 `control_msgs` 의 `reached_goal`(위치 도달)과 뜻이 다르다 — 이름이 다른 것이 그 표시다. `false` 는 '실패'와 '진행 중'을 구분하지 못하니 `width` 를 함께 본다. 두 가지 함정: **① mock 은 `stalled` 관측 수단이 없어 `grasp` 의 `at_goal` 이 영영 서지 않는다** (트윈의 `move_gripper_to_target grasp` 가 mock 에서 타임아웃하는 이유). **② `targets` 는 관절값이고 `width` 는 폭이라 그대로 비교하면 안 된다** — `width_scale` 을 곱해야 하며, 곱하지 않아 `open` 판정이 뒤집힌 적이 있다.
- **`/gripper_states` 는 주기 발행이라 '갱신됨'이 '명령이 끝남'을 뜻하지 않는다.** 완료를 기다리는 코드(트윈 `_send_gripper_command`)는 스냅샷 **세대**가 아니라 `goal` 일치 + `at_goal` 로 판정해야 한다. 세대만 보면 그리퍼가 움직이기도 전에 다음 틱이 성공을 돌려준다. `GripperActionState`(이벤트 채널)는 2026-09-02 에 삭제됐고, 그것이 갖던 `CANCELED`(선점) vs `ABORTED`(실패) 구분은 **되살릴 방법이 미정**이다.
- **그리퍼 채널은 `~/` 가 아니라 루트 상대다** (`gripper_cmds` / `gripper_states`). `~/` 는 네임스페이스가 아니라 **노드 이름**을 붙이므로 채널이 구현에 묶이고(`gripper_control_node` → `GripperActionNode` 교체 때 실제로 깨졌다) `PushRosNamespace` 도 타지 못한다. 근거: [docs/topic_naming_contract.md](docs/topic_naming_contract.md) §2.2.
- **`/session` topic uses `TRANSIENT_LOCAL` durability** so late-joining recorders see the current state. Subscribers and `ros2 topic echo` must match this QoS.
- **rosbag2 sessions without `metadata.yaml` are silently skipped by `discover_splits`** (treated as "still recording / abnormally terminated"). If `rosbag2` was killed with SIGKILL or crashed, `ros2 run rdfp import` exits successfully with an empty summary and no error. Recovery: `ros2 bag reindex -s mcap <session_dir>` rebuilds `metadata.yaml` from the `.mcap` file order. Whenever `import` logs `found 0 finalized split(s)` despite `.mcap` files being present in `rosbag_dir`, this is almost always the cause.
- **The scene write path is the `/scene/reset` service (`rdfp_msgs/srv/ResetScene`), not a topic pair — and reverting to topics would repeat a mistake.** Until 2026-09-01 it was `SceneCommand` on `/scene/commands` + `SceneCommandResult` on `/scene/command_results`; both msg types are now deleted. The stated reason for topics had been "a service response never lands in rosbag2, so a failed reset can't be audited later" — but **neither topic was ever in `config/recording_topics.list`**, so nothing was being recorded either way. The cost was real: the twin carried a result variable, a generation snapshot, and a 20 ms polling loop just to imitate a response. **If someone proposes going back to topics, two things must hold that did not hold before**: (1) the topics have to actually be added to `recording_topics.list`, and (2) there must be a reason to record the *command* — note that what the dataset needs is the *achieved* layout (`/scene/objects`), because in a physics sim blocks roll and settle away from where they were commanded. User-facing docs deliberately no longer carry this history; the rationale for the current design is in [docs/scene/scene_objects_guide.md](docs/scene/scene_objects_guide.md) §5.
- **`/ee_pose` is forward kinematics from `/output/panda_joint`, not the simulator's own EE value.** Chain: `/output/panda_joint` → `joint_state_fusion` → `/joint_states` → `robot_state_publisher` → `/tf` → `ee_pose_publisher` (`lookup_transform`) → `/ee_pose`; the node subscribes to `/tf` and `/tf_static` only. `/output/endeffector` is never read (and is currently not published at all — vendor request B-4). Two consequences: it is **not an independent check** (bad joint values give equally bad EE), and its **model differs from the simulator's** (our URDF has a Panda Hand, the sim has a Robotiq 2F-85, so `panda_hand` is not the real fingertip). It also means **`/ee_pose` freezes when the sim clock jumps backwards** — restarting Unity resets sim time, the TF buffer stalls, and the topic keeps publishing one constant value with a frozen `header.stamp`. Symptom to watch for: joints move but EE does not. Restart the ROS stack after restarting Unity.

- **FunctionBay's servo velocity scale is `0.8`, not MoveIt's stock `0.4` — and it cannot be changed at runtime.** Measured: `0.4` gives 25 mm/s of EE motion with a 6.7× up/down asymmetry; `0.8` gives 54 mm/s with the asymmetry down to 2.1× while staying proportional to command magnitude (0.25/0.5/1.0 → 26/53/103 mm); `3.2` **saturates** — 0.25/0.5/1.0 all yield ~100 mm, so fine positioning is lost and x/y uniformity breaks. `0.8` sits just under the ~100 mm/s ceiling, i.e. the largest value that keeps proportionality. `ros2 param set /servo_node moveit_servo.scale.linear` returns success and the read-back changes, but **behavior does not** — servo reads it only at startup; relaunch with `servo_linear_scale:=<v>`.

- **`moveit_servo` output must be bridged on stacks without ros2_control, or every teleop motion key is silently dead.** Servo defaults to publishing `trajectory_msgs/JointTrajectory` on `/panda_arm_controller/joint_trajectory`; with no controller there, that topic has **zero subscribers** and the computation still runs, so the symptom is only "keys do nothing". FunctionBay now overrides servo to `std_msgs/Float64MultiArray` on `/servo_node/commands` and runs `servo_command_bridge` to convert it to `sensor_msgs/JointState` on `/input/panda_joint` — the same node Isaac uses, which is backend-neutral because its topics are relative and set by remap (a neutral `servo_command_bridge` console script was added alongside `isaac_servo_bridge`). **`publish_joint_velocities: false` is mandatory**, not a preference: with Float64MultiArray output and both positions and velocities enabled, servo's parameter validation fails and the node never starts.

- **The FunctionBay arm latches its last commanded setpoint — you do not need to keep publishing to hold a pose.** Measured 2026-09-01: command a target for 1 s then stop, and it sits there for 25 s with 0.19° of steady-state error; at `ready` it held 30 s at 0.00000 rad. An earlier note in these docs claimed the opposite ("the arm sags to gravity equilibrium when commands stop, so publish at 50 Hz continuously") — **that was wrong and is retracted.** Its single supporting observation (EE dropping 40 mm across a gap between scripts) happened right after opening a gripper that had been closed and pressed against something, so contact was holding the arm up and released it. The same contact explains a "stuck IK loop" diagnosed at the same time. What survives is the triage order: check gripper `effort` > 1 N·m for contact, then command `joint1`/`joint7` (no gravity torque) to test the command path, and only then suspect the A-4 lockup.

- **`teleop_keyboard`'s `/` (move to 'ready') needs the arm command channel declared for non-mock backends.** It builds its MoveGroup client from `arm_command_mode` / `arm_command_topic` / `arm_command_format` / `arm_command_joint_names` parameters, defaulting to `auto` — and `auto` only probes for `/panda_arm_controller/commands`, so a stack without ros2_control (FunctionBay, Isaac) is **misdetected as JTC**. Those stacks have no `FollowJointTrajectory` action server, so the move plans and never executes: pressing `/` looks like a dead key. The node now detects that combination at startup and logs an error naming the fix. **Checking `ros2 action list` is not enough** — an action appears there when only a client exists (`moveit_simple_controller_manager` is exactly that); the node counts publishers on `<action>/_action/status`, which is a hidden topic and so absent from `get_topic_names_and_types()`. For FunctionBay pass `jgpc` + `/input/panda_joint` + `joint_state`.

- **`teleop_keyboard` runs without `session_control`; `session_teleop` does not.** The session node belongs to the collection layer (`rdfp`), so a control-only stack has no `session_control` services. `teleop_keyboard` therefore builds its client with `wait_timeout_sec=0` and probes with `SessionControlClient.wait_until_ready()` (returns False instead of raising); session/episode/task keys then log `ignored: session_control not available` rather than silently doing nothing, and the help text marks the block DISABLED. `session_teleop` keeps the strict `create()` because session control is its only purpose. Related fix: `SessionControlClient._call_async` now checks `service_is_ready()` like the sync path already did — `call_async` on an absent service returns a future that **never completes**, so the done_callback never fires and the caller waits forever with nothing in the log.

- **Nothing from `/scene/objects` reaches the MoveIt planning scene, and that is deliberate (2026-09-01).** Every object on that topic is a manipulation target; publishers filter environment objects out (Isaac ships only `dynamic: true` entries from `isaac_scene.json`, and mock only holds what `/scene/reset` placed). Manipulation targets must not enter the planning scene — a grasp pose then reads as a start-state collision and joint-space planning is rejected with `INVALID_MOTION_PLAN` (-2). With environment objects gone from the topic there is nothing left to insert, so the `planning_scene_sync` node and the `SceneObject.fixture` field were both deleted. **The accepted cost is that the arm plans through the table during joint-space motion** — that is an observed failure, not a hypothesis (fingers once ended up inside the tabletop and it was misread as a gripper fault), and it was accepted for simplicity. Exposure is narrow: self-collision checking still works (ACM, independent of world objects), and grasping is unaffected because approach/descend/lift are cartesian and `GetCartesianPath` defaults `avoid_collisions` to `False`. If it must come back, register the table alone as one box at launch time rather than reviving the topic→scene node.
- **`image_streams` / `image_frames` are inserted via the FrameRouter path, not via `WriterBase`** — so they don't show up in `writers.values()` when the pipeline aggregates summary counts. Both `pipeline._run_ingestion` (serial) and `episode_worker._process_episode_inner` (parallel) must call `router.consume_inserted_count()` and merge it into the `inserted` dict. Skipping that merge leaves the JSON summary reporting `0` for image rows even though DB has them.

## External docs

- **`docs/topic_naming_contract.md` — 토픽 이름 규약.** 논리 채널별 정규 이름(상대 경로), arm 명령을 규약에서 제외한 이유, remap 은 백엔드 경계에서만, 로봇별 네임스페이스 설계(미구현).
- **`docs/INDEX.md` — index of every markdown doc in the repo** (topic-grouped table with content summary, path, and a 현행/설계안/이력/구식 status column). Start here when looking for which document covers something; it also lists known stale docs and broken cross-references.
- `src/rdfp/README.md` — package usage walkthrough: MoveGroup client API, both recorder nodes, `session_control_node` state machine + QoS, dataset CLI reference.
- `src/rdfp/CLAUDE.md` — package-level hints (two recorder nodes, rosbag2 metadata.yaml caveat, replay subsystem coupling).
- `src/robot_control/launch/README.md` — control-layer launches + **full helper inventory** (self-contained).
- `src/rdfp/launch/README.md` — collection-layer (`rdfp_*`) launches + YAML↔argument tables.
- `src/rdfp/rdfp/recorder/README.md` — `FFMpegMp4Recorder` core only (the two ROS adapter nodes are covered in the package README).
- `docs/recorder/` (workspace root) — user guides: `image_recorder_node_guide.md`, `rdfp_image_recorder_node_guide.md`, `ffmpeg_mp4_recorder_guide.md`.
- `docs/session/` — `session_control_guide.md`, `session_control_client_guide.md`.
- `docs/moveit/`, `docs/replay/`, `docs/camera/`, `docs/teleop/` — additional per-subsystem user docs (Korean).
- `docs/rosbag2/` — dataset post-processor user/design/runbook docs and sample YAML configs (referenced from package README).
