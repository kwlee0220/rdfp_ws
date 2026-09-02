# -*- coding: utf-8 -*-
"""Isaac Sim **안에서** 실행하는 로봇 적재 스크립트.

Franka Panda 를 스테이지에 올린다. 이것이 없으면 `setup_graph.py` 가

    no articulation root found on stage

로 멈춘다 — 그래프가 관절 상태를 읽을 대상을 못 찾기 때문이다.

**이 단계가 오랫동안 스크립트에 없었다.** 문서에 "Isaac UI 의 asset browser 에서
Franka 를 스테이지에 올린다"는 **수동 절차**로만 적혀 있어, 스크립트만으로는 스테이지를
재현할 수 없었다(2026-09-02 헤드리스 기동을 만들다 드러났다).

    path = "/home/kwlee/development/ros/rdfp_ws"
    exec(open(path + "/scripts/isaac/sim_side/load_robot.py", encoding="utf-8").read())

**멱등이다** — 이미 있으면 아무것도 하지 않는다. 자세를 고쳐 놓았거나 튜닝한 뒤에
다시 돌려도 그것을 덮어쓰지 않는다.

**Stop 상태에서 실행한다.** 그다음 순서는 `setup_scene` → `setup_graph` →
`place_robot` → `set_home_pose` → `tune_drive` → `tune_grasp` 이며, 마지막에 Play 다.

자산 경로
--------

**Isaac 6.0 에서 재편됐다.** 5.x 의 ``/Isaac/Robots/Franka/franka.usd`` 가
``/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd`` 로 옮겨졌다. 루트는
``get_assets_root_path()`` 가 주며 기본은 원격(S3)이라 **첫 실행은 내려받는다.**
"""
import os as _os

_IS_WINDOWS = _os.name == "nt"
_DEFAULT_WORKSPACE = ("//wsl.localhost/Ubuntu-22.04/home/kwlee/development/ros/rdfp_ws"
                      if _IS_WINDOWS else "/home/kwlee/development/ros/rdfp_ws")
_DEFAULT_LOG_DIR = "//wsl.localhost/Ubuntu-22.04/tmp" if _IS_WINDOWS else "/tmp"
WORKSPACE = _os.environ.get("RDFP_WORKSPACE") or _DEFAULT_WORKSPACE
LOG_DIR = _os.environ.get("RDFP_LOG_DIR") or _DEFAULT_LOG_DIR
LOG_PATH = LOG_DIR + "/isaac_load_robot.log"

# 스테이지에 올릴 자리. `setup_graph.py` 가 articulation root 를 여기서 찾는다.
ROBOT_PRIM = "/World/franka"
# 자산 루트 아래의 상대 경로 (Isaac 6.0 배치).
FRANKA_USD_RELPATH = "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"

_LOG_LINES = []


def _log(message: str) -> None:
    print(message)
    _LOG_LINES.append(message)


def _flush_log() -> None:
    try:
        with open(LOG_PATH, "w", encoding="utf-8") as f:
            f.write("\n".join(_LOG_LINES) + "\n")
        print(f"[load] log written: {LOG_PATH}")
    except Exception as exc:                              # noqa: BLE001
        print(f"[load] could not write log: {type(exc).__name__}: {exc}")


def main() -> None:
    import omni.usd
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.storage.native import get_assets_root_path

    try:
        stage = omni.usd.get_context().get_stage()
        existing = stage.GetPrimAtPath(ROBOT_PRIM)
        if existing and existing.IsValid():
            _log(f"[load] {ROBOT_PRIM} already on stage - nothing to do")
            _log("[load] next: setup_scene.py -> setup_graph.py -> place_robot.py")
            return

        root = get_assets_root_path()
        if root is None:
            _log("[load] ERROR: asset root not reachable - is the network up?")
            return

        usd = root + FRANKA_USD_RELPATH
        _log(f"[load] asset root : {root}")
        _log(f"[load] reference  : {usd}")
        add_reference_to_stage(usd_path=usd, prim_path=ROBOT_PRIM)
        _log(f"[load] placed at  : {ROBOT_PRIM}")
        _log("[load] the reference resolves asynchronously - the robot appears "
             "once the download finishes (first run only)")
        _log("[load] next: setup_scene.py -> setup_graph.py -> place_robot.py "
             "-> set_home_pose.py -> tune_drive.py -> tune_grasp.py -> Play")
    finally:
        _flush_log()


main()
