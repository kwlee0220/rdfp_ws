# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Scope of this file

`rdfp_msgs` is a pure interface package (`ament_cmake` +
`rosidl_default_generators`) — no runtime Python/C++ source, only `msg/` and
`srv/` IDL files.

For wider context, read these first:

- Workspace-root [`../../CLAUDE.md`](../../CLAUDE.md) — build commands,
  workspace layout, the `rdfp` package's architecture, and the dataset
  pipeline that consumes these interfaces.
- [`README.md`](README.md) — authoritative per-interface table (which node
  publishes/serves which type, what the response fields mean). Update the
  README, not just CLAUDE.md, when adding or changing an interface.

## Adding or changing an interface — the dual-update gotcha

A new `.msg` or `.srv` file does **not** get generated unless registered in
both:

1. `CMakeLists.txt` — add the relative path to `rosidl_generate_interfaces(...)`.
2. `package.xml` — add a `<depend>` entry if the new interface references a
   type from a package that isn't already declared (currently `std_msgs`,
   `trajectory_msgs`).

If you add a new external dependency, also extend `find_package(...)` and the
`DEPENDENCIES` argument of `rosidl_generate_interfaces` in `CMakeLists.txt`.
The generator silently produces no Python/C++ binding for unregistered files —
the failure mode is a missing import on the `rdfp` side, not a build error
here.

## Always rebuild rdfp together

Changes to interface fields (rename, type swap, removal) are not picked up by
the `rdfp` package until **both** are rebuilt and the env is re-sourced:

```bash
colcon build --packages-select rdfp_msgs rdfp
source install/setup.bash
```

Building only `rdfp_msgs` leaves `install/rdfp/` referencing the old
generated bindings — node imports may succeed but with stale fields.

## Conventions for new interfaces

- Comments inside `.msg` / `.srv` files are written in Korean (matches the
  `rdfp` package convention from the workspace CLAUDE).
- Response fields for command-style services follow the existing pattern: a
  `bool success` plus either a `string message` (validation-style services
  like `SetString`) or a domain-specific payload like `string mp4_path`
  (resource-creation services like `StartSession` / `StopSession`). Reuse
  this shape rather than inventing a new error-reporting convention.
- Use `std_srvs/srv/Trigger` for parameterless commands that only need
  `success` + `message`; do not add a new local empty-request service when
  `Trigger` would suffice (precedent: the four `/session_control/start_*` /
  `/stop_*` services on the `rdfp` side use `Trigger` instead of a local type).
