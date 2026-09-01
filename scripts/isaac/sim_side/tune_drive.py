# -*- coding: utf-8 -*-
"""Isaac Sim **안에서** 실행하는 팔 관절 drive 튜닝 스크립트.

Phase 1 실측에서 두 가지가 기준을 넘었다.

    시정수 τ      195.6 ms   (기준 50 ms)
    정착 오차     0.0698 rad @extended, 0.0015 @ready  (기준 0.005)

**둘은 같은 파라미터에 걸려 있다.** 위치 제어 관절에서

    정착 오차 = 중력토크 / K       시정수 τ = B / K

이므로 K(stiffness)가 분모로 양쪽에 들어간다. 오차가 자세에 따라 46배로 커진 것
(`ready` 0.0015 → `extended` 0.0698)이 분자가 중력 토크라는 증거다.

**K 만 올리면 감쇠비가 떨어져 진동한다.** ζ = B / (2√(KJ)) 이므로 ζ 를 유지하려면
B 는 √K 에 비례해 올린다 — K 를 ``STIFFNESS_SCALE`` 배 하면 B 는 그 제곱근 배.

    K ×25, B ×5  →  τ 195.6 → 39 ms,  처짐 0.0698 → 0.0028 rad

단위는 실측으로 확정됐다 — ``maxForce`` 가 87/12 Nm 로 실제 Franka 스펙과 같고,
τ = B/K 가 실측과 2% 안에서 맞는다. 즉 **라디안 기준 Nm/rad** 이다.

사용법 — **Stop 상태에서 실행하고 다시 Play 한다.** 재생 중 변경은 물리 엔진에
반영되지 않을 수 있다.

    path = "//wsl.localhost/Ubuntu-22.04/home/kwlee/development/ros/rdfp_ws"
    exec(open(path + "/scripts/isaac/sim_side/tune_drive.py", encoding="utf-8").read())

``DRY_RUN = True`` 로 두면 **현재 값만 읽어 로그에 남기고 아무것도 바꾸지 않는다.**
처음에는 이 상태로 한 번 돌려 기준값을 남기는 것을 권한다.
"""

import os as _os

# 워크스페이스·로그 경로. 배포 구성 네 가지를 모두 지원한다 (문서 §1).
#
#   환경변수 RDFP_WORKSPACE / RDFP_LOG_DIR 가 있으면 그것을 쓴다. **Isaac 머신과
#   스택 머신이 다른 구성(§1 C·D)에서는 반드시 지정한다** — 그때는 Isaac 쪽에
#   저장소 사본이 따로 있고, 로그도 Isaac 머신에 떨어진다.
#
#   없으면 같은 머신을 가정한 기본값을 쓴다.
#     Windows : UNC 로 WSL 파일시스템 (§1 A)
#     Linux   : 로컬 경로 (§1 B)
_IS_WINDOWS = _os.name == "nt"
_DEFAULT_WORKSPACE = ("//wsl.localhost/Ubuntu-22.04/home/kwlee/development/ros/rdfp_ws"
                      if _IS_WINDOWS else "/home/kwlee/development/ros/rdfp_ws")
_DEFAULT_LOG_DIR = "//wsl.localhost/Ubuntu-22.04/tmp" if _IS_WINDOWS else "/tmp"
WORKSPACE = _os.environ.get("RDFP_WORKSPACE") or _DEFAULT_WORKSPACE
LOG_DIR = _os.environ.get("RDFP_LOG_DIR") or _DEFAULT_LOG_DIR
LOG_PATH = LOG_DIR + "/isaac_tune_drive.log"


# **절대값으로 설정한다 (배수 아님).** 배수 방식은 두 번 실행하면 값이 제곱으로
# 튄다 — 실제로 그렇게 400 → 10000 → 250000 까지 갔다. 절대값은 몇 번을 돌려도
# 같은 상태로 수렴한다.
#
# 에셋 기본값은 stiffness=400 / damping=80 이고, 그 상태의 실측이 아래와 같았다.
#   τ = B/K = 80/400 = 0.200 s   (실측 0.1956 s — 모델과 2% 오차)
# 목표 τ 50 ms 와 처짐 0.005 rad 을 만족하는 값이 아래다. 감쇠비 ζ = B/(2√(KJ)) 는
# K 를 25배 할 때 B 를 √25 = 5배로 맞춰 그대로 유지된다.
TARGET_STIFFNESS = 10000.0     # 400 x 25
TARGET_DAMPING = 400.0         # 80 x 5

# True 면 읽기만 하고 쓰지 않는다.
DRY_RUN = False

ARM_JOINT_NAMES = [f"panda_joint{i}" for i in range(1, 8)]

_LOG_LINES = []


def _log(message: str) -> None:
    print(message)
    _LOG_LINES.append(message)


def _flush_log() -> None:
    try:
        with open(LOG_PATH, "w", encoding="utf-8") as f:
            f.write("\n".join(_LOG_LINES) + "\n")
        print(f"[tune] log written: {LOG_PATH}")
    except Exception as exc:
        print(f"[tune] could not write log to {LOG_PATH}: {type(exc).__name__}: {exc}")


def _use_root_layer() -> None:
    """편집 대상을 **root layer** 로 고정한다.

    Isaac 은 상황에 따라 edit target 이 **session layer** 로 가 있다. 그 상태에서
    만든 prim 과 바꾼 속성은 화면에서는 멀쩡히 동작하지만 ``File > Save As`` 에
    **담기지 않는다** — 실제로 그렇게 저장한 scene 에 ActionGraph 도 drive gain 도
    들어 있지 않았다(8.8 KB, Franka 참조뿐).

    저장해서 재사용하려면 root layer 에 써야 한다.
    """
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    root = stage.GetRootLayer()
    current = stage.GetEditTarget().GetLayer()
    if current != root:
        stage.SetEditTarget(root)
        _log(f"[tune] edit target: {current.identifier} -> root layer")
    else:
        _log("[tune] edit target: root layer (ok)")


def _find_joint_prims():
    """스테이지에서 팔 관절 prim 을 이름으로 찾는다.

    경로 구조는 에셋마다 다르지만 관절 **이름**은 URDF 와 맞춰져 있다
    (Phase 0 에서 `/joint_states` 로 확인했다).
    """
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    found = {}
    for prim in stage.Traverse():
        name = prim.GetName()
        if name in ARM_JOINT_NAMES and name not in found:
            found[name] = prim
    return found


def _drive(prim):
    """관절 prim 의 angular DriveAPI 를 돌려준다 (없으면 None)."""
    from pxr import UsdPhysics

    drive = UsdPhysics.DriveAPI.Get(prim, "angular")
    return drive if drive else None


def main() -> None:
    _log(f"[tune] arm drive tuning (stiffness={TARGET_STIFFNESS}, "
         f"damping={TARGET_DAMPING}, dry_run={DRY_RUN})")

    if not DRY_RUN:
        _use_root_layer()

    prims = _find_joint_prims()
    missing = [j for j in ARM_JOINT_NAMES if j not in prims]
    if missing:
        _log(f"[tune] WARNING: joint prims not found: {missing}")

    for joint in ARM_JOINT_NAMES:
        prim = prims.get(joint)
        if prim is None:
            continue
        drive = _drive(prim)
        if drive is None:
            _log(f"[tune] {joint}: no angular DriveAPI at {prim.GetPath()}")
            continue

        stiffness_attr = drive.GetStiffnessAttr()
        damping_attr = drive.GetDampingAttr()
        stiffness = stiffness_attr.Get()
        damping = damping_attr.Get()
        max_force = drive.GetMaxForceAttr().Get()

        _log(f"[tune] {joint}: stiffness={stiffness} damping={damping} maxForce={max_force}")

        if DRY_RUN:
            continue

        stiffness_attr.Set(TARGET_STIFFNESS)
        damping_attr.Set(TARGET_DAMPING)
        _log(f"[tune]   -> stiffness={TARGET_STIFFNESS} damping={TARGET_DAMPING}")

    if DRY_RUN:
        _log("[tune] dry run - nothing written")
    else:
        _log("[tune] done. Press Stop then Play so PhysX picks up the new gains")


try:
    main()
finally:
    _flush_log()
