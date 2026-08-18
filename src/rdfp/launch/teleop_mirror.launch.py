"""Leader-follower 동작 미러링 파이프라인 런치.

follower Panda + MoveIt2 스택 (예: ``panda_mock.launch.py`` /
``rdfp_panda_mock.launch.py``) 이 **이미 실행 중** 이라는 전제 하에, 미러링
체인 2개 노드만 추가로 기동한다. 설계 문서:
docs/teleop/leader_follower_mirroring_design.md

기동 노드:

- ``teleop_retarget`` — 클러치 앵커 기반 상대 매핑으로 follower 목표 pose 를
  생성한다. 클러치: ``/teleop_retarget/clutch`` (std_srvs/SetBool).
- ``ee_twist_publisher`` (``ee_twist_node``, source=ee_pose) — 목표 pose 를
  차분해 ``twist_topic`` (기본 ``/servo_node/delta_twist_cmds``) 으로 발행한다.
- ``clutch_pedal`` — ``clutch_pedal.enabled`` 가 true 일 때만 (선택).

설정 파일
---------

모든 argument 의 기본값은 ``<rdfp share>/config/teleop_mirror.yaml`` 에서 온다.
``rdfp_panda_mock.launch.py`` 와 같은 방식이다.

.. code-block:: bash

   ros2 launch rdfp teleop_mirror.launch.py                          # YAML 기본값
   ros2 launch rdfp teleop_mirror.launch.py align_yaw:=3.14159       # 개별 덮어쓰기
   ros2 launch rdfp teleop_mirror.launch.py config_file:=/path/my.yaml

YAML 키 ↔ argument 대응표는 ``src/rdfp/launch/README.md`` 의 "Launch 인자" 절에
있다. ``config_file`` 이 ``OpaqueFunction`` 안에서 resolve 된 **뒤에야** 나머지
argument 가 선언되므로 ``--show-args`` 는 ``config_file`` 하나만 출력한다 —
전체 목록은 그 표를 본다.

leader pose 는 외부에서 공급된다
--------------------------------

본 런치는 ``leader_pose_topic`` (기본 ``/leader/ee_pose``) 의 **발행자를 만들지
않는다.** 외부 입력 어댑터가 이미 그 토픽을 발행하고 있다고 전제한다.

- OMY-L100 (ROS 2 Jazzy / rmw_zenoh) — 별도 저장소 ``omy_leader_bridge`` 가 UDP
  relay 를 거쳐 호스트에 pose 를 재발행한다.
- 그 외 어댑터가 지켜야 할 토픽·QoS·stamp·주기 계약은
  ``docs/teleop/external_input_adapters.md`` 에 정의되어 있다.

leader 의 ``/tf`` 가 **같은 ROS 그래프에 있어** pose 를 직접 뽑아야 한다면
``ee_pose_node`` 를 따로 띄운다. 본 런치에 넣지 않는 이유는, 외부 공급이 기본
구성인데 런치가 발행자를 함께 만들면 **발행자가 둘이 되어 충돌** 하기 때문이다.

.. code-block:: bash

   ros2 run rdfp ee_pose_node --ros-args \\
       -p base_frame:=leader_base -p ee_frame:=leader_ee -p publish_rate:=50.0 \\
       -r ee_pose:=/leader/ee_pose

USB 풋페달로 클러치를 잡으려면
------------------------------

``enable_clutch_pedal:=true`` (YAML ``clutch_pedal.enabled``) 를 주면
``clutch_pedal`` 노드까지 함께 기동한다. 밟는 동안만 engage 하는 **데드맨**
구성이다.

.. code-block:: bash

   ros2 launch rdfp teleop_mirror.launch.py \\
       enable_clutch_pedal:=true pedal_device_name:=pedal pedal_key_code:=KEY_A

기본값은 ``false`` 다 — ``python3-evdev`` 와 실제 장치가 없으면 노드가 기동에
실패하므로, 페달을 쓰지 않는 구성에 부담을 주지 않기 위함이다. 페달 노드를 따로
띄우고 싶으면 ``pedal_timeout`` 만 켜면 된다.

.. code-block:: bash

   ros2 launch rdfp teleop_mirror.launch.py pedal_timeout:=0.3
   ros2 run rdfp clutch_pedal --ros-args -p device_name:=pedal -p key_code:=KEY_A

데드맨은 **두 조각이 함께 있어야** 성립한다 — 페달 노드(뗌 감지)와
``pedal_timeout``(하트비트 감시). 페달 노드가 크래시하거나 USB 가 빠지면
disengage 를 보낼 주체가 사라지는데, 하트비트가 끊기는 것으로 그 상황을 잡는다.

본 런치는 두 값의 정합을 자동으로 맞춘다.

- ``enable_clutch_pedal:=true`` + ``pedal_mode:=hold`` 인데 ``pedal_timeout`` 이
  0 이면 → **0.3 으로 채운다** (데드맨이 아닌 채로 뜨는 것을 막는다)
- ``pedal_mode:=toggle`` 인데 ``pedal_timeout`` 이 0 보다 크면 → **0 으로 되돌린다**
  (toggle 은 하트비트를 보내지 않으므로 감시를 켜면 engage 즉시 풀린다)

상세: ``docs/teleop/clutch_pedal_guide.md``.

Servo 단위 주의
---------------

본 워크스페이스의 servo 설정(``panda_simulated_config.yaml``)은
``command_in_type: unitless`` (joystick 식, scale.linear=0.4 m/s,
scale.rotational=0.8 rad/s) 이므로, 물리 단위(m/s, rad/s) twist 를 unitless 로
변환하기 위해 기본 게인을 ``linear 1/0.4=2.5``, ``angular 1/0.8=1.25`` 로
설정한다. servo 를 ``speed_units`` 로 운용한다면
``twist_linear_gain:=1.0 twist_angular_gain:=1.0`` 을 지정한다.

사용 예
-------

.. code-block:: bash

   ros2 launch rdfp teleop_mirror.launch.py \\
       position_scale:=1.0 align_yaw:=0.0 \\
       workspace_min:="[0.1, -0.5, 0.05]" workspace_max:="[0.8, 0.5, 0.9]"

   # 클러치 engage / disengage
   ros2 service call /teleop_retarget/clutch std_srvs/srv/SetBool "{data: true}"
"""

from __future__ import annotations

from typing import Any

import os

import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# YAML 설정 파일의 기본 경로. setup.py 가 ``config/*`` 를 ``share/rdfp/config/``
# 로 설치하므로 share 에서 읽는다.
DEFAULT_CONFIG_RELPATH = os.path.join("config", "teleop_mirror.yaml")

# enable_clutch_pedal + hold 모드인데 pedal_timeout 이 0 일 때 대신 넣는 값.
# clutch_pedal 의 heartbeat_rate 기본값 10Hz 기준 3배 여유다.
_DEFAULT_PEDAL_TIMEOUT = 0.3

# 페달 동작 모드. hold 만 데드맨이다.
_PEDAL_MODES = ("hold", "toggle")


def _default_config_path() -> str:
    """패키지 share 경로의 기본 YAML 위치를 반환한다."""
    return os.path.join(get_package_share_directory("rdfp"), DEFAULT_CONFIG_RELPATH)


def _load_config(config_path: str) -> dict[str, Any]:
    """YAML 설정 파일을 로드하여 dict 로 반환한다."""
    with open(config_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _as_launch_str(value: Any) -> str:
    """Python 값을 DeclareLaunchArgument 의 default_value 로 쓰이는 문자열로 변환한다.

    - bool 은 ROS launch 관례에 맞춰 소문자 ``"true"`` / ``"false"`` 로 변환한다.
    - list 는 ``str()`` 결과가 그대로 YAML 리스트 문법이므로 그대로 쓴다
      (``[]`` -> ``"[]"``, ``[0.1, 0.2, 0.3]`` -> ``"[0.1, 0.2, 0.3]"``).
    - 그 외는 ``str()`` 으로 변환한다.
    """
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def _arg_defaults(config: dict[str, Any]) -> dict[str, str]:
    """YAML 을 ``{launch argument 이름: 기본값 문자열}`` 로 평탄화한다.

    argument 이름은 YAML 키와 1:1 이 아니다 (``clutch_pedal.enabled`` ->
    ``enable_clutch_pedal`` 등). 대응표는 launch/README.md 의 "Launch 인자" 절.
    """
    leader = config["leader"]
    follower = config["follower"]
    rt = config["retarget"]
    pedal = config["clutch_pedal"]
    twist = config["twist"]

    return {
        # --- leader EE pose 입력 (외부 공급) ---
        "leader_pose_topic": _as_launch_str(leader["pose_topic"]),
        # --- follower ---
        "follower_pose_topic": _as_launch_str(follower["pose_topic"]),
        "target_pose_topic": _as_launch_str(follower["target_pose_topic"]),
        "follower_base_frame": _as_launch_str(follower["base_frame"]),
        # --- retargeting ---
        "position_scale": _as_launch_str(rt["position_scale"]),
        "align_roll": _as_launch_str(rt["align_roll"]),
        "align_pitch": _as_launch_str(rt["align_pitch"]),
        "align_yaw": _as_launch_str(rt["align_yaw"]),
        "lpf_cutoff_hz": _as_launch_str(rt["lpf_cutoff_hz"]),
        "watchdog_timeout": _as_launch_str(rt["watchdog_timeout"]),
        "max_sample_jump": _as_launch_str(rt["max_sample_jump"]),
        "workspace_min": _as_launch_str(rt["workspace_min"]),
        "workspace_max": _as_launch_str(rt["workspace_max"]),
        # --- USB 풋페달 (선택) ---
        "enable_clutch_pedal": _as_launch_str(pedal["enabled"]),
        "pedal_device_path": _as_launch_str(pedal["device_path"]),
        "pedal_device_name": _as_launch_str(pedal["device_name"]),
        "pedal_key_code": _as_launch_str(pedal["key_code"]),
        "pedal_mode": _as_launch_str(pedal["mode"]),
        "pedal_grab": _as_launch_str(pedal["grab"]),
        "pedal_timeout": _as_launch_str(pedal["timeout"]),
        # --- twist 변환 ---
        "twist_topic": _as_launch_str(twist["topic"]),
        "twist_linear_gain": _as_launch_str(twist["linear_gain"]),
        "twist_angular_gain": _as_launch_str(twist["angular_gain"]),
    }


def _declare_arguments(defaults: dict[str, str]) -> list[DeclareLaunchArgument]:
    """YAML 값을 default 로 사용하는 launch argument 들을 선언한다."""
    return [
        # --- leader EE pose 입력 (외부 공급) ---
        DeclareLaunchArgument(
            "leader_pose_topic",
            default_value=defaults["leader_pose_topic"],
            description=(
                "Leader EE pose input topic (PoseStamped, leader base frame). "
                "외부 어댑터가 발행한다 — 본 런치는 이 토픽을 만들지 않는다. "
                "계약: docs/teleop/external_input_adapters.md"
            ),
        ),
        # --- retargeting ---
        DeclareLaunchArgument(
            "follower_pose_topic",
            default_value=defaults["follower_pose_topic"],
            description="Follower current EE pose topic (from the panda stack ee_pose_publisher)",
        ),
        DeclareLaunchArgument(
            "target_pose_topic",
            default_value=defaults["target_pose_topic"],
            description="Retargeted follower target pose topic",
        ),
        DeclareLaunchArgument(
            "follower_base_frame",
            default_value=defaults["follower_base_frame"],
            description="Follower base frame (frame_id of target pose / twist)",
        ),
        DeclareLaunchArgument(
            "position_scale",
            default_value=defaults["position_scale"],
            description="Leader-to-follower position displacement scale",
        ),
        DeclareLaunchArgument(
            "align_roll", default_value=defaults["align_roll"],
            description="R_align roll (rad): leader base -> follower base orientation alignment",
        ),
        DeclareLaunchArgument(
            "align_pitch", default_value=defaults["align_pitch"],
            description="R_align pitch (rad)",
        ),
        DeclareLaunchArgument(
            "align_yaw", default_value=defaults["align_yaw"],
            description="R_align yaw (rad); pi flips x/y for mirror-image mapping",
        ),
        DeclareLaunchArgument(
            "lpf_cutoff_hz",
            default_value=defaults["lpf_cutoff_hz"],
            description="Low-pass filter cutoff for the target pose (<=0 disables)",
        ),
        DeclareLaunchArgument(
            "watchdog_timeout",
            default_value=defaults["watchdog_timeout"],
            description="Auto-disengage if leader pose stream is stale for this long (sec)",
        ),
        DeclareLaunchArgument(
            "max_sample_jump",
            default_value=defaults["max_sample_jump"],
            description="Auto-disengage if leader moves more than this between samples (m)",
        ),
        DeclareLaunchArgument(
            "pedal_timeout",
            default_value=defaults["pedal_timeout"],
            description=(
                "USB 풋페달 데드맨. 0 보다 크면 clutch_pedal 노드의 하트비트를 감시해 "
                "끊길 때 자동 disengage 한다. **0(기본)이면 비활성** — 페달을 쓸 때는 "
                "반드시 켜야 노드 크래시·USB 분리 상황에서 클러치가 물린 채 남지 않는다. "
                "권장값은 heartbeat_rate 주기의 3배 (10Hz -> 0.3). "
                "enable_clutch_pedal:=true + pedal_mode:=hold 인데 이 값이 0 이면 "
                "런치가 자동으로 0.3 을 넣는다. "
                "docs/teleop/clutch_pedal_guide.md 참고"
            ),
        ),
        # --- USB 풋페달 (선택) ---
        DeclareLaunchArgument(
            "enable_clutch_pedal",
            default_value=defaults["enable_clutch_pedal"],
            choices=["true", "false"],
            description=(
                "clutch_pedal 노드를 함께 기동한다. python3-evdev 와 실제 페달 장치가 "
                "있어야 하며, 없으면 노드가 기동에 실패한다. 기본 false"
            ),
        ),
        DeclareLaunchArgument(
            "pedal_device_path",
            default_value=defaults["pedal_device_path"],
            description=(
                "페달 장치 경로. 비우면 pedal_device_name 으로 탐색한다. "
                "/dev/input/eventN 번호는 재연결 시 바뀌므로 /dev/input/by-id/... 권장"
            ),
        ),
        DeclareLaunchArgument(
            "pedal_device_name",
            default_value=defaults["pedal_device_name"],
            description="페달 장치 이름 부분 문자열 (대소문자 무시)",
        ),
        DeclareLaunchArgument(
            "pedal_key_code",
            default_value=defaults["pedal_key_code"],
            description=(
                "페달이 보내는 키 이름 (예: KEY_A, BTN_LEFT). "
                "비우면 해당 장치의 **아무 키나** 페달로 취급한다"
            ),
        ),
        DeclareLaunchArgument(
            "pedal_mode",
            default_value=defaults["pedal_mode"],
            choices=list(_PEDAL_MODES),
            description=(
                "hold = 밟는 동안만 engage (데드맨, 권장). "
                "toggle = 밟을 때마다 전환 — 데드맨이 아니며 하트비트도 보내지 않는다"
            ),
        ),
        DeclareLaunchArgument(
            "pedal_grab",
            default_value=defaults["pedal_grab"],
            choices=["true", "false"],
            description=(
                "true 면 장치를 독점(EVIOCGRAB)해 페달 입력이 데스크톱·터미널로 "
                "새지 않는다. 페달이 일반 키보드로 인식될 때 켠다"
            ),
        ),
        DeclareLaunchArgument(
            "workspace_min",
            default_value=defaults["workspace_min"],
            description="Optional follower workspace box min '[x, y, z]' (YAML list)",
        ),
        DeclareLaunchArgument(
            "workspace_max",
            default_value=defaults["workspace_max"],
            description="Optional follower workspace box max '[x, y, z]' (YAML list)",
        ),
        # --- twist 변환 ---
        DeclareLaunchArgument(
            "twist_topic",
            default_value=defaults["twist_topic"],
            description="Output TwistStamped topic consumed by MoveIt Servo",
        ),
        DeclareLaunchArgument(
            "twist_linear_gain",
            default_value=defaults["twist_linear_gain"],
            description="Linear gain for twist output (1/scale.linear for unitless servo)",
        ),
        DeclareLaunchArgument(
            "twist_angular_gain",
            default_value=defaults["twist_angular_gain"],
            description="Angular gain for twist output (1/scale.rotational for unitless servo)",
        ),
    ]


def _resolve(context: LaunchContext, defaults: dict[str, str], name: str) -> str:
    """CLI 로 준 값이 있으면 그 값을, 없으면 YAML 기본값을 문자열로 반환한다.

    argument 선언을 이 launch 의 ``OpaqueFunction`` 반환 목록에 넣으므로, 같은
    함수 안에서 ``LaunchConfiguration(name).perform(context)`` 은 아직 값을 보지
    못한다 (선언 action 이 아직 실행되지 않았다). CLI ``arg:=value`` 는 launch
    실행 전에 이미 context 에 들어와 있으므로 그것만 조회하고, 없으면 YAML
    기본값을 쓴다. ``config_file`` 은 상위에서 선언하므로 이 함수를 거치지 않고
    ``LaunchConfiguration`` 으로 직접 읽는다.
    """
    return context.launch_configurations.get(name, defaults[name])


def _parse_workspace(raw: str, name: str) -> list[float]:
    """workspace 박스 값(YAML 리스트 문자열)을 파싱한다.

    빈 문자열과 빈 리스트(``[]``) 를 모두 "비활성" 으로 취급한다 — 전자는 CLI 로
    비워 준 경우, 후자는 YAML 기본값이다.
    """
    raw = raw.strip()
    if not raw:
        return []
    value = yaml.safe_load(raw)
    if value is None or (isinstance(value, list) and not value):
        return []
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"'{name}' must be a 3-element YAML list, got {raw!r}")
    return [float(v) for v in value]


def _build_actions(context: LaunchContext) -> list:
    """`config_file` 이 resolve 된 뒤 YAML 을 로드해 argument/노드를 구성한다.

    workspace 박스처럼 문자열 substitution 으로 전달할 수 없는(double array)
    파라미터가 있고 페달 정합 로직도 실제 값이 필요하므로, 값을 여기서 파싱해
    실제 타입으로 넘긴다.
    """
    config_path = LaunchConfiguration("config_file").perform(context)
    config = _load_config(config_path)
    defaults = _arg_defaults(config)

    def sval(name: str) -> str:
        return _resolve(context, defaults, name)

    def fval(name: str) -> float:
        return float(sval(name))

    def bval(name: str) -> bool:
        return sval(name).strip().lower() == "true"

    workspace_min = _parse_workspace(sval("workspace_min"), "workspace_min")
    workspace_max = _parse_workspace(sval("workspace_max"), "workspace_max")

    # leader pose 는 외부 어댑터가 `leader_pose_topic` 으로 공급한다. 본 런치는
    # 그 토픽의 발행자를 만들지 않는다 — 만들면 발행자가 둘이 되어 충돌한다.
    # leader 의 /tf 가 같은 그래프에 있어 직접 뽑아야 하는 경우에는 별도로
    # `ros2 run rdfp ee_pose_node` 를 띄운다 (docstring 참고).

    # --- USB 풋페달 (선택) ---
    # 데드맨은 "페달 노드" 와 "retarget 의 하트비트 감시" 두 조각이 함께 있어야
    # 성립한다. 한쪽만 켠 구성은 조용히 잘못 동작하므로 여기서 정합을 맞춘다.
    enable_pedal = bval("enable_clutch_pedal")
    pedal_mode = sval("pedal_mode")
    pedal_timeout = fval("pedal_timeout")

    # 아래 정합 로직이 mode 문자열을 그대로 비교하므로, 오타를 조용히 흘리면
    # hold 도 toggle 도 아닌 상태로 떠서 데드맨이 사라진다. 여기서 막는다
    # (DeclareLaunchArgument 의 choices 는 이 함수보다 늦게 실행된다).
    if pedal_mode not in _PEDAL_MODES:
        raise RuntimeError(
            f"pedal_mode must be one of {list(_PEDAL_MODES)}, got {pedal_mode!r}"
        )

    if enable_pedal and pedal_mode == "hold" and pedal_timeout <= 0.0:
        # hold 모드인데 감시가 꺼져 있으면 데드맨이 아니다. 기본값을 넣어 준다
        # (heartbeat_rate 10Hz 기준 3배 여유).
        pedal_timeout = _DEFAULT_PEDAL_TIMEOUT
    elif pedal_mode == "toggle" and pedal_timeout > 0.0:
        # toggle 은 하트비트를 보내지 않으므로 감시를 켜면 engage 즉시 풀린다.
        pedal_timeout = 0.0

    retarget_params: dict = {
        "leader_pose_topic": sval("leader_pose_topic"),
        "follower_pose_topic": sval("follower_pose_topic"),
        "target_pose_topic": sval("target_pose_topic"),
        "follower_base_frame": sval("follower_base_frame"),
        "position_scale": fval("position_scale"),
        "align_roll": fval("align_roll"),
        "align_pitch": fval("align_pitch"),
        "align_yaw": fval("align_yaw"),
        "lpf_cutoff_hz": fval("lpf_cutoff_hz"),
        "watchdog_timeout": fval("watchdog_timeout"),
        "max_sample_jump": fval("max_sample_jump"),
        "pedal_timeout": pedal_timeout,
    }
    if workspace_min and workspace_max:
        retarget_params["workspace_min"] = workspace_min
        retarget_params["workspace_max"] = workspace_max

    teleop_retarget_node = Node(
        package="rdfp",
        executable="teleop_retarget",
        name="teleop_retarget",
        output="screen",
        emulate_tty=True,
        parameters=[retarget_params],
    )

    ee_twist_node = Node(
        package="rdfp",
        executable="ee_twist_node",
        name="ee_twist_publisher",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "source": "ee_pose",
            "ee_pose_topic": sval("target_pose_topic"),
            "twist_topic": sval("twist_topic"),
            "base_frame": sval("follower_base_frame"),
            "linear_gain": fval("twist_linear_gain"),
            "angular_gain": fval("twist_angular_gain"),
        }],
    )

    pedal_nodes: list[Node] = []
    if enable_pedal:
        pedal_nodes.append(Node(
            package="rdfp",
            executable="clutch_pedal",
            name="clutch_pedal",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "device_path": sval("pedal_device_path"),
                "device_name": sval("pedal_device_name"),
                "key_code": sval("pedal_key_code"),
                "mode": pedal_mode,
                "grab": bval("pedal_grab"),
                "clutch_node_name": "teleop_retarget",
            }],
        ))

    return [
        # --- YAML 기본값을 가진 argument 들 (config_file resolve 후 결정) ---
        *_declare_arguments(defaults),
        # --- 미러링 체인 ---
        teleop_retarget_node,
        ee_twist_node,
        *pedal_nodes,
    ]


def generate_launch_description() -> LaunchDescription:
    # `config_file` 만 declaration 시점에 노출하고, 그 값에 의존하는 YAML 로딩과
    # 나머지 argument / 노드 생성은 `OpaqueFunction` 안에서 수행한다.
    # rdfp_panda_mock.launch.py 와 같은 구조다.
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=_default_config_path(),
        description=(
            "Path to the teleop_mirror YAML configuration file. "
            "기본값은 <rdfp share>/config/teleop_mirror.yaml 이며 "
            "CLI 또는 IncludeLaunchDescription launch_arguments 로 "
            "'config_file:=<path>' 를 주면 덮어쓸 수 있다 "
            "($HOME 등 쉘 확장은 쉘에 맡긴다)."
        ),
    )
    return LaunchDescription([
        config_file_arg,
        OpaqueFunction(function=_build_actions),
    ])
