# -*- coding: utf-8 -*-
"""Isaac Sim **안에서** 실행하는 OmniGraph 구성 스크립트 (단계 누적형).

이 파일만 ROS 노드가 아니라 **시뮬레이터 쪽**에서 돈다. Isaac Sim 의
``Window > Script Editor`` 에서 **붙여넣지 말고 아래 두 줄로 불러 실행한다.**
붙여넣기는 인코딩을 타고(Windows 기본 cp949) 사본이 저장소보다 낡는다.

    path = "//wsl.localhost/Ubuntu-22.04/home/kwlee/development/ros/rdfp_ws"
    exec(open(path + "/scripts/isaac/sim_side/setup_graph.py", encoding="utf-8").read())

경로 구분자는 슬래시를 쓴다 — Windows 의 open() 이 UNC 경로에서도 그대로 받는다.
배포판 이름이 ``Ubuntu`` 가 아니라 **``Ubuntu-22.04``** 인 점에 주의한다.

**실행 결과는 화면과 로그 파일 양쪽에 남는다.** Script Editor 출력창은 복사가
되지 않으므로, 진단이 필요하면 WSL 쪽에서 ``LOG_PATH`` 파일을 읽는다:

    cat /tmp/isaac_setup_graph.log

런타임 출력은 전부 영어다 — Windows 콘솔이 cp949 라 한글이 깨진다. 주석과
docstring 은 규약대로 한국어이며, 위 방식으로 실행하면 편집기에 뜨지 않는다.

**단계는 누적된다.** 파일 상단의 ``PHASE`` 만 바꿔 다시 실행한다 — 그래프를 통째로
지우고 그 단계까지 다시 만든다.

    PHASE 0  상태 경로 (읽기 전용)
        OnPlaybackTick ─┬─> ROS2PublishClock       → /clock
                        └─> ROS2PublishJointState  → /joint_states
        IsaacReadSimulationTime ─> 두 노드의 timeStamp
        ROS2Context(domain_id=31) ─> 모든 ROS2 노드의 context

    PHASE 1  팔 명령 경로 (+ 위 전부)
        OnPlaybackTick ──> ROS2SubscribeJointState  ← /isaac/arm_command
                              └─> IsaacArticulationController (/World/franka)

    PHASE 2  그리퍼 명령 경로 (+ 위 전부)
        OnPlaybackTick ──> ROS2SubscribeJointState  ← /isaac/gripper_command
                              └─> IsaacArticulationController (/World/franka)

    PHASE 3  scene 물체 pose (+ 위 전부)
        OnPlaybackTick ──> ROS2PublishTransformTree → /tf  (world 기준)
                              targetPrims = isaac_scene.json 의 물체들

    PHASE 4  고정 시점 카메라 (+ 위 전부)
        OnPlaybackTick ──> IsaacCreateRenderProduct
                              ├─> ROS2CameraHelper      → /isaac/camera/image_raw
                              └─> ROS2CameraInfoHelper  → /isaac/camera/camera_info

**로봇은 이 스크립트가 불러오지 않는다.** 스테이지에 이미 올라와 있는
articulation 을 찾아 쓴다. 에셋 경로는 Isaac 버전마다 바뀌지만 "articulation 이
하나 있다"는 사실은 변하지 않기 때문이다. 먼저 UI 에서 Franka(Panda) 를 스테이지에
올린 뒤 이 스크립트를 실행한다.

실행 후 **Play(▶) 를 눌러야** 그래프가 돈다 — ``OnPlaybackTick`` 은 재생 중에만
tick 을 낸다. 정지 상태에서는 토픽이 하나도 보이지 않는 것이 정상이다.
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
SCENE_JSON = WORKSPACE + "/src/robot_control/config/isaac_scene.json"
LOG_PATH = LOG_DIR + "/isaac_setup_graph.log"


# 어느 단계까지 만들 것인가. 0=상태만, 1=+팔 명령, 2=+그리퍼 명령, 3=+scene TF, 4=+카메라.
PHASE = 4

# ROS 쪽 계약. docs/simulation/isaac_backend_skeleton.md §3 토픽 계약표와 일치해야 한다.
ROS_DOMAIN_ID = 31
CLOCK_TOPIC = "/clock"
JOINT_STATE_TOPIC = "/joint_states"
ARM_COMMAND_TOPIC = "/isaac/arm_command"
GRIPPER_COMMAND_TOPIC = "/isaac/gripper_command"

# 렌더 틱 주파수. 카메라 frameSkipCount 를 계산하는 기준이다.
RENDER_HZ = 60.0

# Phase 3 — scene 물체를 TF 로 내보낸다. 물체 정의는 ROS 쪽과 **같은 JSON** 을 읽는다.
# parentPrim 을 비우면 Isaac 이 TF 부모를 "world" 로 쓴다. 우리 스택의
# static TF(world -> panda_link0)와 이름이 맞으므로 트리가 그대로 이어진다.

# 스테이지에 articulation 이 여럿일 때 쓸 명시적 경로. None 이면 자동 탐색한다.
ROBOT_PRIM_PATH = None

# Script Editor 출력창은 복사가 되지 않으므로 파일로도 남긴다. WSL 쪽 경로라
# 리눅스에서 `cat /tmp/isaac_phase0.log` 로 바로 읽힌다.

# 타임라인 최소 길이(타임코드). 60 fps 기준 약 46 시간.
TIMELINE_MIN_CODES = 10_000_000.0

GRAPH_PATH = "/ActionGraph"
ROS2_BRIDGE_EXT = "isaacsim.ros2.bridge"

_LOG_LINES = []


def _log(message: str) -> None:
    """화면과 로그 버퍼 양쪽에 남긴다."""
    print(message)
    _LOG_LINES.append(message)


def _flush_log() -> None:
    """버퍼를 LOG_PATH 에 쓴다. 실패해도 그래프 구성 자체는 유효하다."""
    try:
        with open(LOG_PATH, "w", encoding="utf-8") as f:
            f.write("\n".join(_LOG_LINES) + "\n")
        print(f"[phase0] log written: {LOG_PATH}")
    except Exception as exc:
        print(f"[phase0] could not write log to {LOG_PATH}: {type(exc).__name__}: {exc}")


def _log_environment() -> None:
    """진단에 필요한 환경 정보를 남긴다."""
    import sys

    _log(f"[phase0] python: {sys.version.split()[0]}")
    try:
        import omni.kit.app

        version = omni.kit.app.get_app().get_build_version()
        _log(f"[phase0] kit build: {version}")
    except Exception as exc:
        _log(f"[phase0] kit build: unavailable ({type(exc).__name__})")


def _enable_ros2_bridge() -> None:
    """ROS 2 bridge 확장이 꺼져 있으면 켠다."""
    import omni.kit.app

    manager = omni.kit.app.get_app().get_extension_manager()
    if manager.is_extension_enabled(ROS2_BRIDGE_EXT):
        _log(f"[phase0] {ROS2_BRIDGE_EXT}: already enabled")
        return
    manager.set_extension_enabled_immediate(ROS2_BRIDGE_EXT, True)
    _log(f"[phase0] {ROS2_BRIDGE_EXT}: enabled")


def _find_articulation_root() -> str:
    """스테이지에서 articulation root prim 경로를 찾는다.

    Returns:
        prim 경로 문자열.

    Raises:
        RuntimeError: articulation 이 없거나 둘 이상이라 특정할 수 없을 때.
    """
    import omni.usd
    from pxr import UsdPhysics

    if ROBOT_PRIM_PATH:
        return ROBOT_PRIM_PATH

    stage = omni.usd.get_context().get_stage()
    roots = [p.GetPath().pathString for p in stage.Traverse()
             if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    _log(f"[phase0] articulation roots on stage: {roots}")

    if not roots:
        raise RuntimeError(
            "no articulation root found on stage; add the robot first "
            "(drag Franka from the Isaac asset browser)")
    if len(roots) > 1:
        raise RuntimeError(
            f"found {len(roots)} articulations: {roots}; "
            f"set ROBOT_PRIM_PATH at the top of this file to pick one")
    return roots[0]


def _dump_attributes(node_name: str) -> None:
    """노드의 입력 속성 이름을 로그에 남긴다 (속성명이 버전마다 다를 때 쓴다)."""
    import omni.graph.core as og

    path = f"{GRAPH_PATH}/{node_name}"
    try:
        node = og.Controller.node(path)
        names = [a.get_name() for a in node.get_attributes() if a.get_name().startswith("inputs:")]
        _log(f"[phase0]   {path} input attributes: {names}")
    except Exception as exc:  # 진단용이라 실패해도 넘어간다
        _log(f"[phase0]   {path}: failed to read attributes: {type(exc).__name__}: {exc}")


def _verify(node_name: str, attribute: str) -> None:
    """SET_VALUES 가 실제로 먹었는지 되읽어 확인한다."""
    import omni.graph.core as og

    target = f"{GRAPH_PATH}/{node_name}.{attribute}"
    try:
        value = og.Controller.get(og.Controller.attribute(target))
        _log(f"[phase0]   {node_name}.{attribute} = {value!r}")
    except Exception as exc:
        _log(f"[phase0]   {node_name}.{attribute}: read back FAILED "
             f"({type(exc).__name__}: {exc})")


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
        _log(f"[phase0] edit target: {current.identifier} -> root layer")
    else:
        _log("[phase0] edit target: root layer (ok)")


def _ensure_timeline_range() -> None:
    """타임라인 재생 구간을 충분히 길게 잡는다.

    **Play 는 endTimeCode 에 닿으면 스스로 멈춘다.** 스테이지를 저장했다가 다시
    열면 그때의 구간이 그대로 남는데, 기본 스테이지의 구간이 짧으면 Play 를 눌러도
    몇 프레임 만에 정지한다 — sim time 이 0.16667 s(60 fps 에서 10 프레임)에서 굳는
    식이다. 정지 원인이 화면에 드러나지 않아 "명령이 안 먹는다"로 오해하기 쉽다.

    로봇 시뮬레이션에는 끝이 없어야 하므로 구간을 크게 잡는다.
    """
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    start = stage.GetStartTimeCode()
    end = stage.GetEndTimeCode()
    fps = stage.GetTimeCodesPerSecond() or 60.0
    _log(f"[phase0] timeline: start={start} end={end} fps={fps} "
         f"({(end - start) / fps:.2f}s)")

    if end - start < TIMELINE_MIN_CODES:
        stage.SetStartTimeCode(0.0)
        stage.SetEndTimeCode(TIMELINE_MIN_CODES)
        _log(f"[phase0]   -> extended to 0..{TIMELINE_MIN_CODES} "
             f"({TIMELINE_MIN_CODES / fps / 3600.0:.1f} hours)")


def _reset_graph() -> None:
    """기존 ``/ActionGraph`` 프림을 지운다.

    ``og.Controller.edit`` 에 생성 스펙(dict)을 주면 **이미 그래프가 있을 때
    ``Failed to wrap graph in node`` 로 실패한다.** 재실행할 때마다 이 상태에
    빠지므로, 매번 지우고 새로 만들어 결과를 결정적으로 만든다.
    """
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(GRAPH_PATH)
    if prim and prim.IsValid():
        stage.RemovePrim(GRAPH_PATH)
        _log(f"[phase0] removed existing graph at {GRAPH_PATH}")
    else:
        _log(f"[phase0] no existing graph at {GRAPH_PATH}")


def _ensure_target_prim(robot_prim: str) -> None:
    """``targetPrim`` 은 관계(relationship) 속성이라 SET_VALUES 로 안 먹을 수 있다.

    되읽어 비어 있으면 ``set_target_prims`` 로 다시 건다.
    """
    import omni.graph.core as og

    node_path = f"{GRAPH_PATH}/PublishJointState"
    try:
        value = og.Controller.get(og.Controller.attribute(f"{node_path}.inputs:targetPrim"))
    except Exception as exc:
        value = None
        _log(f"[phase0]   targetPrim read back failed ({type(exc).__name__}: {exc})")

    if value:
        _log(f"[phase0]   PublishJointState.inputs:targetPrim = {value!r}")
        return

    try:
        og.Controller.set_target_prims(primPath=node_path, targetPrimPaths=[robot_prim],
                                       inputName="inputs:targetPrim")
        _log(f"[phase0]   targetPrim re-bound via set_target_prims -> {robot_prim}")
    except Exception as exc:
        _log(f"[phase0]   targetPrim BIND FAILED: {type(exc).__name__}: {exc}")


def _phase1_spec(robot_prim: str) -> tuple[list, list, list]:
    """Phase 1 (팔 명령 경로) 의 노드·연결·값을 만든다.

    ``IsaacArticulationController`` 는 구독한 ``JointState`` 의 **이름**을 그대로
    쓰므로, 배열 순서 계약이 필요 없다. 우리 쪽은
    ``MoveGroupJgpcClient(arm_command_format='joint_state')`` 로 발행한다.
    """
    nodes = [
        ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
        ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
    ]
    connections = [
        ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
        ("Context.outputs:context", "SubscribeJointState.inputs:context"),
        ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
        ("SubscribeJointState.outputs:positionCommand",
         "ArticulationController.inputs:positionCommand"),
        ("SubscribeJointState.outputs:velocityCommand",
         "ArticulationController.inputs:velocityCommand"),
        ("SubscribeJointState.outputs:effortCommand",
         "ArticulationController.inputs:effortCommand"),
    ]
    values = [
        ("SubscribeJointState.inputs:topicName", ARM_COMMAND_TOPIC),
        ("ArticulationController.inputs:robotPath", robot_prim),
    ]
    return nodes, connections, values


def _phase2_spec(robot_prim: str) -> tuple[list, list, list]:
    """Phase 2 (그리퍼 명령 경로) 의 노드·연결·값을 만든다.

    팔과 **별도의 구독자·컨트롤러 쌍**을 둔다. 하나의 컨트롤러에 두 토픽을 물릴 수
    없기도 하고, articulation drive 의 목표값은 관절별로 유지되므로 각 컨트롤러가
    자기 메시지에 실린 관절만 건드려도 서로 지워지지 않는다.
    """
    nodes = [
        ("SubscribeGripper", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
        ("GripperController", "isaacsim.core.nodes.IsaacArticulationController"),
    ]
    connections = [
        ("OnPlaybackTick.outputs:tick", "SubscribeGripper.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "GripperController.inputs:execIn"),
        ("Context.outputs:context", "SubscribeGripper.inputs:context"),
        ("SubscribeGripper.outputs:jointNames", "GripperController.inputs:jointNames"),
        ("SubscribeGripper.outputs:positionCommand",
         "GripperController.inputs:positionCommand"),
        ("SubscribeGripper.outputs:velocityCommand",
         "GripperController.inputs:velocityCommand"),
        ("SubscribeGripper.outputs:effortCommand", "GripperController.inputs:effortCommand"),
    ]
    values = [
        ("SubscribeGripper.inputs:topicName", GRIPPER_COMMAND_TOPIC),
        ("GripperController.inputs:robotPath", robot_prim),
    ]
    return nodes, connections, values


def _scene_prim_paths() -> list[str]:
    """scene JSON 에서 물체 prim 경로를 읽는다."""
    import json

    with open(SCENE_JSON, encoding="utf-8") as f:
        config = json.load(f)
    root = config["root_prim"]
    return [f"{root}/{obj['name']}" for obj in config["objects"]]


def _phase3_spec(robot_prim: str) -> tuple[list, list, list]:
    """Phase 3 (scene 물체 pose) 의 노드·연결·값을 만든다.

    **TF 로 내보낸다.** `PoseArray` 대신 TF 를 고르는 이유가 둘이다.

    1. Isaac 이 `ROS2PublishTransformTree` 를 기본 제공한다 — 커스텀 노드가 없다.
    2. **좌표 변환과 쿼터니언 순서를 tf2 가 대신한다.** `SceneObjects` 는 pose 를
       로봇 베이스(`panda_link0`) 기준으로 요구하는데 Isaac 은 world 기준이고,
       Isaac 의 쿼터니언은 wxyz 다. TF 를 타면 우리 노드가 `lookup_transform` 한
       번으로 둘 다 해결한다 — 손으로 뒤집다 틀릴 자리가 사라진다.
    """
    scene_prims = _scene_prim_paths()
    nodes = [
        ("PublishSceneTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
    ]
    connections = [
        ("OnPlaybackTick.outputs:tick", "PublishSceneTF.inputs:execIn"),
        ("ReadSimTime.outputs:simulationTime", "PublishSceneTF.inputs:timeStamp"),
        ("Context.outputs:context", "PublishSceneTF.inputs:context"),
    ]
    values = [
        ("PublishSceneTF.inputs:targetPrims", scene_prims),
        # `parentPrim` 은 **비워 둔다.** 기본값이 곧 world 프레임이다. 문자열
        # "world" 를 넣으면 관계(relationship) 속성이라 노드 기준 상대 경로로
        # 해석되어 `/ActionGraph/PublishSceneTF/world` 라는 없는 prim 을 가리킨다.
    ]
    return nodes, connections, values


def _phase4_spec(robot_prim: str) -> tuple[list, list, list]:
    """Phase 4 (고정 시점 카메라) 의 노드·연결·값을 만든다.

    렌더 프로덕트를 만들고 그것을 ROS 헬퍼 두 개에 물린다.

        IsaacCreateRenderProduct(cameraPrim, width, height)
            └─> ROS2CameraHelper(type=rgb)      → image_topic
            └─> ROS2CameraInfoHelper            → camera_info_topic

    **`frameSkipCount` 로 주파수를 낮춘다.** 렌더 틱은 60 Hz 인데 5 Hz 만 필요하므로
    11 을 건너뛴다. 60 Hz 로 렌더한 뒤 버리는 것이 아니라 **렌더 자체를 건너뛰므로**
    GPU 부담이 그만큼 준다 — VRAM 이 빠듯한 호스트에서 특히 중요하다.
    """
    import json

    with open(SCENE_JSON, encoding="utf-8") as f:
        camera = json.load(f)["camera"]

    width, height = camera["resolution"]
    fps = float(camera["fps"])
    # 렌더 틱(60 Hz) 대비 몇 개를 건너뛸지. 5 Hz -> 11.
    frame_skip = max(int(round(RENDER_HZ / fps)) - 1, 0)

    nodes = [
        ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
        ("CameraRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ("CameraInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
    ]
    connections = [
        ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
        # 렌더 프로덕트가 만들어진 **뒤에** 발행 노드가 돌아야 한다.
        ("CreateRenderProduct.outputs:execOut", "CameraRgb.inputs:execIn"),
        ("CreateRenderProduct.outputs:execOut", "CameraInfo.inputs:execIn"),
        ("CreateRenderProduct.outputs:renderProductPath", "CameraRgb.inputs:renderProductPath"),
        ("CreateRenderProduct.outputs:renderProductPath", "CameraInfo.inputs:renderProductPath"),
        ("Context.outputs:context", "CameraRgb.inputs:context"),
        ("Context.outputs:context", "CameraInfo.inputs:context"),
    ]
    values = [
        ("CreateRenderProduct.inputs:cameraPrim", camera["prim"]),
        ("CreateRenderProduct.inputs:width", width),
        ("CreateRenderProduct.inputs:height", height),
        ("CameraRgb.inputs:type", "rgb"),
        ("CameraRgb.inputs:topicName", camera["image_topic"]),
        ("CameraRgb.inputs:frameId", camera["frame_id"]),
        ("CameraRgb.inputs:frameSkipCount", frame_skip),
        ("CameraInfo.inputs:topicName", camera["camera_info_topic"]),
        ("CameraInfo.inputs:frameId", camera["frame_id"]),
        ("CameraInfo.inputs:frameSkipCount", frame_skip),
    ]
    return nodes, connections, values


def build_graph(robot_prim: str) -> None:
    """``PHASE`` 까지의 action graph 를 만든다 (_reset_graph 가 먼저 지운다)."""
    import omni.graph.core as og

    nodes = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("Context", "isaacsim.ros2.bridge.ROS2Context"),
        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
    ]
    connections = [
        ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
        ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
        ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
        ("Context.outputs:context", "PublishClock.inputs:context"),
        ("Context.outputs:context", "PublishJointState.inputs:context"),
    ]
    values = [
        # 환경변수 대신 값을 못박는다. Windows 에서 Isaac 을 GUI 로 띄우면
        # ROS_DOMAIN_ID 가 프로세스에 없을 수 있고, 그러면 도메인 0 으로
        # 붙어 **아무 에러 없이** 스택과 서로 보이지 않는다.
        ("Context.inputs:useDomainIDEnvVar", False),
        ("Context.inputs:domain_id", ROS_DOMAIN_ID),
        ("PublishClock.inputs:topicName", CLOCK_TOPIC),
        # Isaac 이 이름과 9관절을 모두 채우므로 중계 노드 없이 스택 내부
        # 토픽으로 바로 발행한다 (skeleton 문서 §7 Q1). 기본값도 같은
        # 이름이지만, 계약을 코드에 남기려고 명시한다.
        ("PublishJointState.inputs:topicName", JOINT_STATE_TOPIC),
        ("PublishJointState.inputs:targetPrim", robot_prim),
        # Stop 할 때 sim time 을 0 으로 되돌리지 않는다.
        #
        # 되돌리면 **시간이 거꾸로 간다.** 오래 떠 있는 ROS 스택의 TF 버퍼는 이미
        # 미래 시각의 변환을 갖고 있으므로 새로 오는 데이터를 과거로 보고 버린다
        #   TF_OLD_DATA ignoring data from the past for frame panda_link6
        # 그러면 TF 조회가 조용히 실패하고 move_group 이 현재 상태를 못 읽는다.
        # Isaac 을 Stop/Play 할 때마다 스택을 재시작하지 않으려면 이 값이 False 여야 한다.
        ("ReadSimTime.inputs:resetOnStop", False),
    ]

    for phase, spec in ((1, _phase1_spec), (2, _phase2_spec), (3, _phase3_spec),
                        (4, _phase4_spec)):
        if PHASE >= phase:
            extra_nodes, extra_connections, extra_values = spec(robot_prim)
            nodes += extra_nodes
            connections += extra_connections
            values += extra_values

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: nodes,
            keys.CONNECT: connections,
            keys.SET_VALUES: values,
        },
    )


def main() -> None:
    _log(f"[phase0] building Isaac action graph (PHASE={PHASE})")
    try:
        _log_environment()
        _enable_ros2_bridge()

        robot_prim = _find_articulation_root()
        _log(f"[phase0] using articulation: {robot_prim}")

        _use_root_layer()
        _ensure_timeline_range()
        _reset_graph()

        try:
            build_graph(robot_prim)
        except Exception as exc:
            _log(f"[phase0] graph build FAILED: {type(exc).__name__}: {exc}")
            _log("[phase0] attribute names differ between versions; actual inputs below:")
            dump_targets = ["Context", "PublishClock", "PublishJointState", "ReadSimTime"]
            if PHASE >= 1:
                dump_targets += ["SubscribeJointState", "ArticulationController"]
            if PHASE >= 2:
                dump_targets += ["SubscribeGripper", "GripperController"]
            if PHASE >= 3:
                dump_targets += ["PublishSceneTF"]
            if PHASE >= 4:
                dump_targets += ["CreateRenderProduct", "CameraRgb", "CameraInfo"]
            for name in dump_targets:
                _dump_attributes(name)
            raise

        _log(f"[phase0] done - {GRAPH_PATH} (PHASE={PHASE})")
        _verify("Context", "inputs:domain_id")
        _verify("Context", "inputs:useDomainIDEnvVar")
        _verify("PublishClock", "inputs:topicName")
        _verify("PublishJointState", "inputs:topicName")
        _verify("ReadSimTime", "inputs:resetOnStop")
        _ensure_target_prim(robot_prim)
        if PHASE >= 1:
            _verify("SubscribeJointState", "inputs:topicName")
            _verify("ArticulationController", "inputs:robotPath")
        if PHASE >= 2:
            _verify("SubscribeGripper", "inputs:topicName")
            _verify("GripperController", "inputs:robotPath")
        if PHASE >= 3:
            _verify("PublishSceneTF", "inputs:targetPrims")
        if PHASE >= 4:
            _verify("CreateRenderProduct", "inputs:width")
            _verify("CreateRenderProduct", "inputs:height")
            _verify("CameraRgb", "inputs:topicName")
            _verify("CameraRgb", "inputs:frameSkipCount")
        _log("[phase0] IMPORTANT: press Stop then Play (a rebuilt graph does not "
             "attach to a playback already running)")
    finally:
        _flush_log()


main()
