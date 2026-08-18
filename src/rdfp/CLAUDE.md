# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Scope of this file

This is a package-level CLAUDE.md scoped to the `rdfp` ament_python package. The
workspace-root [`../../CLAUDE.md`](../../CLAUDE.md) is authoritative for build
commands, cross-package architecture, the dataset pipeline overview, coding
conventions, and the "important non-obvious behaviors" list — read that first.
This file only adds package-internal details that aren't covered there.

## Authoritative READMEs (more detail than CLAUDE summaries)

When in doubt, the in-tree READMEs are the source of truth — CLAUDE.md is a
hint sheet, not a spec.

- [README.md](README.md) — full package walkthrough: `MoveGroupClient` API
  surface, `camera_node`/`image_recorder_node`/`rdfp_image_recorder` parameter
  tables, the `session_control_node` state machine + topic QoS, the dataset
  CLI reference, and the replay GUI overview. Update this file (not just
  CLAUDE.md) when public behavior changes.
- [launch/README.md](launch/README.md) — per-launch / per-helper inventory and
  the Panda startup ordering rationale.
- [rdfp/recorder/README.md](rdfp/recorder/README.md) — `FFMpegMp4Recorder`
  internals; the two ROS adapter nodes (`image_recorder_node`,
  `rdfp_image_recorder`) are documented in the package-level README.

Detailed user/dev guides live under the workspace `docs/` tree (not in the
package), e.g. [docs/recorder/](../../docs/recorder/),
[docs/session/](../../docs/session/),
[docs/rosbag2/](../../docs/rosbag2/).

## Adding a console script

`setup.py` is the single source of truth for `ros2 run rdfp <name>` entries —
there is no `entry_points.txt` or plugin registry. After editing the
`console_scripts` list:

1. Run `colcon build --packages-select rdfp` from the workspace root (not from
   here — see workspace CLAUDE for why).
2. Re-`source install/setup.bash`. The new name will not be discoverable via
   `ros2 run` until the `lib/rdfp/` shim is regenerated.

Reminder from the workspace CLAUDE: dataset CLIs are deliberately split into
sibling `*_cmd.py` modules; do **not** re-introduce a `dataset <sub>` argparse
dispatcher.

## setup.py data_files — what gets installed to share/

The `data_files` list installs three globs into `share/rdfp/`:

| Source glob | Installed to | Purpose |
|---|---|---|
| `launch/*.py` | `share/rdfp/launch/` | launch entry points + helper modules |
| `config/*` | `share/rdfp/config/` | YAML / RViz configs (`image_pipeline.yaml`, `panda_robot.yaml`, `replay_panda_mock.yaml`, `teleop_mirror.yaml`, `panda.rviz`) |
| `rdfp/dataset/sql/*.sql` | `share/rdfp/dataset/sql/` | DB schema bootstrap scripts used by `init-db` |

If you add a new top-level config directory or a new SQL file location, extend
`data_files` — `find_packages` does not pick non-Python assets. The SQL glob in
particular is easy to miss because the source path is under the Python source
tree.

## Dependency split (apt vs pip)

`package.xml` declares apt-installable deps (`python3-yaml`, `python3-opencv`,
`ffmpeg`, ROS message packages). The dataset post-processor additionally
requires pip-only packages that are intentionally **not** in `package.xml`
because no current apt version satisfies them:

```bash
pip install --user 'mcap' 'mcap-ros2-support' 'pydantic>=2' 'psycopg[binary]>=3'
```

`rosdep install` will not catch these. New code under `rdfp/dataset/` or
`rdfp/rosbag/` that pulls in additional pip-only deps should also be documented
in the README.md "Post-processor 의존성" section, not just installed locally.

## Two image recorder nodes — don't confuse them

There are **two** `sensor_msgs/Image` → MP4 recorder nodes in the package, both
backed by `FFMpegMp4Recorder` but with different control surfaces:

- `image_recorder_node` (`rdfp.recorder.image_recorder_node`) — external
  start/stop services (`/image_recorder/start_session`,
  `/image_recorder/stop_session`); single recording per service call. Used by
  `rdfp_panda_mock.launch.py`.
- `rdfp_image_recorder` (`rdfp.recorder.rdfp_image_recorder_node`) —
  subscribes `/session` (`SessionCommand`) and **auto-records IN_EPISODE
  windows**, timestamp-based segmentation via a `pending_image_queue`. Also
  emits a per-frame `.jsonl` sidecar and per-prefix `metadata.json`. Used by
  `rdfp_advanced.launch.py`.

Both names show up across the codebase — match the launch file to the right
node when reviewing changes.

## rosbag2 sessions without metadata.yaml are silently skipped

`rdfp.rosbag.catalog.discover_splits` treats any session directory without a
`metadata.yaml` as "still recording or abnormally terminated" and excludes it
from import. Consequence: if `rosbag2` was killed with SIGKILL (or crashed),
`ros2 run rdfp import` finishes successfully with an **empty summary** and no
error. To recover, run:

```bash
ros2 bag reindex -s mcap /data/rdfp/rosbag/<YYYY-MM-DD>/<session_dir>
```

This rebuilds `metadata.yaml` from the `.mcap` file order; the bag is then
importable. If `import` ever logs `found 0 finalized split(s)` despite the
rosbag_dir containing `.mcap` files, this is almost always the cause.

## Replay subsystem coupling

`rdfp/dataset/replay_gui_cmd.py` (Tk GUI, exposed as the `replay_gui`
console_script) wires together two single-shot replayers from
`rdfp/dataset/db/`:

- `TopicMessageReplayer` — non-image topics, single worker over a heap merge.
  Rejects image topics in `__init__` (`ValueError`); use `Mp4ImageReplayer`
  instead for images.
- `Mp4ImageReplayer` — per-image-topic, decoder thread + publisher thread +
  bounded queue.

Both expose the same `start(start_time, first_history_time)` /
`stop()` / `close()` / `error` / `join()` interface so the GUI can drive them
in lock-step. Lifecycle is **one-shot** — calling `start()` twice raises
`RuntimeError('… already started')`. `close()` skips publisher
destroy / `cv2.VideoCapture.release()` if a worker thread is still alive after
the 2 s join timeout (leak is the lesser evil vs. use-after-destroy).

The "Topics to replay" listbox in the GUI starts with **all entries
unchecked** by design; do not flip the default back to `BooleanVar(value=True)`
without rationale.

`replay_panda_mock.launch.py` spawns exactly one arm adapter, selected by the
`replay_arm_path` argument — `ee_twist` (default), `target_joint_cmds`, or
`none`. Both non-`none` paths end at `/panda_arm_controller/joint_trajectory`,
so they must never run together; the launch enforces this in `_build_actions`
and raises `RuntimeError` on an unknown value. The `ee_twist` path additionally
needs `servo_auto_start_node` because `moveit_servo` ignores input until
`start_servo` is called. Full structure / usage / pitfalls:
[docs/replay/replay_mock_stack_guide.md](../../docs/replay/replay_mock_stack_guide.md).

## Tests live next to source

Subpackage tests are colocated under `rdfp/<sub>/tests/` — six suites
(`camera`, `dataset`, `recorder`, `rosbag`, `teleop`, `twin`). The `test/` dir
that `package.xml` and `setup.py` still reference **does not exist**, so the
ament linters never run; see the workspace CLAUDE.md "Tests". When adding
tests:

- Pure-Python tests (no `rclpy` / `sensor_msgs`) should stay importable without
  sourcing ROS — many existing dataset tests guard ROS imports behind
  `pytest.importorskip` so `PYTHONPATH=.:$PYTHONPATH pytest …` works in a clean
  shell. **Always append `:$PYTHONPATH`**; a bare `PYTHONPATH=.` replaces the
  ROS entry and breaks ROS imports even in a sourced shell (workspace CLAUDE.md
  "Tests").
- ROS-dependent tests should `pytest.importorskip('rclpy')` (or the specific
  message package) at module top so they self-skip outside a sourced env rather
  than erroring during collection — an unguarded module aborts the **whole run**,
  not just itself. Put the guard **above** the `rdfp.*` imports and mark those
  `# noqa: E402`; use `pytest.mark.skipif` on a single test when the rest of the
  module is ROS-free (`test_dataset_import_cmd.py`).
- Check the dependency *transitively*, not by what the test file imports:
  `ingest/test_filters.py` exercises pure-Python filter logic but `pipeline`
  pulls in `sensor_msgs`, and `test_topic_message_replayer.py` stubs every ROS
  object while the module under test imports `builtin_interfaces` at top level.
