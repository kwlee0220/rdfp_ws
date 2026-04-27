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
  surface, `camera_node`/`image_recorder_node` parameter tables, the
  `session_control_node` state machine + topic QoS, the dataset CLI reference,
  and the replay GUI overview. Update this file (not just CLAUDE.md) when
  public behavior changes.
- [launch/README.md](launch/README.md) — per-launch / per-helper inventory and
  the Panda startup ordering rationale.
- [rdfp/recorder/README.md](rdfp/recorder/README.md) — `FFMpegMp4Recorder`
  internals and the `image_recorder_node` ROS adapter.

> Note: README.md links to `rdfp/recorder/docs/image_recorder_node_srs.md`,
> `rdfp/session/session_control_srs.md`, and `session_control_plan.md` as
> "상세 명세 / 개발 절차" but **none of these files currently exist** in the
> tree (`recorder/docs/` only has `prompt.md`; `session/` has no `.md` at
> all). Treat those links as aspirational placeholders until restored.

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
| `config/*` | `share/rdfp/config/` | YAML / RViz configs (`rdfp_panda_mock.yaml`, `panda.rviz`) |
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

## Tests live next to source

Subpackage tests are colocated under `rdfp/<sub>/tests/` (not under the
top-level `test/` dir, which is reserved for ament linters only). When adding
tests:

- Pure-Python tests (no `rclpy` / `sensor_msgs`) should stay importable without
  sourcing ROS — many existing dataset tests guard ROS imports behind
  `pytest.importorskip` so `PYTHONPATH=. pytest …` works in a clean shell.
- ROS-dependent tests should `pytest.importorskip('rclpy')` (or the specific
  message package) at module top so they self-skip outside a sourced env rather
  than erroring during collection.
