"""펑션베이 수집 배선 — **압축을 직접 받는가.**

레코더가 둘이고 능력이 다르다.

| 노드 | 구동 | `input_format` |
|---|---|---|
| `image_recorder_node` | 서비스 | 없다 — `sensor_msgs/Image` 만 구독한다 |
| `rdfp_image_recorder` | `/session` | 있다 — 압축을 직접 받거나 내부 디코드 |

**앞의 것을 쓰면 `camera_republish` 에 매달린다.** 그 노드는 `enable_camera` 가 켜고
그 기본값은 뷰어에서 파생되므로, 헤드리스 수집(`enable_image_viewer:=false`)에서는
raw 발행자가 없어 **레코더가 조용히 0 프레임을 담는다** — 오류도 경고도 없다.
2026-09-08 까지 실제로 그랬다.

> **검사 대상이 바뀌었다 (2026-09-13).** 예전에는 묶음 launch
> `rdfp_panda_functionbay.launch.py` 를 AST 로 읽었다. 그 묶음을 삭제하면서 배선이
> `rdfp_collect` 의 **백엔드 프로파일 파생**으로 옮겨갔으므로, 이제 그 파생 함수를
> 직접 본다 — 소스 텍스트가 아니라 값을 보므로 더 정확하다.
"""

from __future__ import annotations

import importlib.util
import pathlib

import pytest

pytest.importorskip('launch')
pytest.importorskip('robot_control')

from ament_index_python.packages import get_package_share_directory  # noqa: E402

_LAUNCH_FILE = (pathlib.Path(get_package_share_directory('rdfp'))
                / 'launch' / 'rdfp_collect.launch.py')


@pytest.fixture(scope='module')
def collect():
    if not _LAUNCH_FILE.is_file():
        pytest.skip(f'{_LAUNCH_FILE} 없음 (colcon build 를 먼저 돌린다)')
    spec = importlib.util.spec_from_file_location('rdfp_collect_launch', _LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_functionbay_defaults_to_the_compressed_stream(collect):
    """`camera.source: compressed` 면 레코더가 압축을 **직접** 받는다.

    raw 로 되살리면 압축으로 아낀 대역폭이 그대로 돌아온다 (0.27 → 9.2 MB/s, 36배).
    """
    defaults = collect._backend_defaults('functionbay')

    assert defaults['image_recorder_input_format'] == 'mjpeg'
    # 원본(`/camera_image/compressed`)이 아니라 stamp 를 관절과 같은 벽시계 축으로 옮긴
    # 중계 출력이어야 한다 — 레코더는 이미지 stamp 를 `/session` stamp 와 비교해 녹화
    # 구간을 가르므로, 시뮬레이터가 경과시간을 싣는 회차에는 원본으로는 구간이 성립하지 않는다.
    assert defaults['camera_compressed_topic'] == '/camera_image_aligned/compressed'


def test_raw_backends_do_not_ask_for_mjpeg(collect):
    """압축을 안 쓰는 백엔드에까지 mjpeg 이 새면 0 프레임이 된다."""
    for name in ('mock', 'mock_jgpc', 'isaac'):
        defaults = collect._backend_defaults(name)
        assert defaults['image_recorder_input_format'] == 'rawvideo', name


def test_only_the_session_driven_recorder_is_launched(collect):
    """서비스 구동 레코더는 압축을 못 받는다 — 이 launch 가 띄우면 안 된다.

    한때 `recorder:=service` 로 고를 수 있었는데(삭제된 묶음 launch 와의 등가용),
    2026-09-13 에 그 인자와 함께 뺐다. 되살리면 압축 백엔드에서 `camera_republish` 에
    매달리게 되고, 그 발행자가 없으면 **조용히 0 프레임**이다.
    """
    body = _LAUNCH_FILE.read_text(encoding='utf-8').split('"""', 2)[2]

    assert 'executable="rdfp_image_recorder"' in body
    assert 'executable="image_recorder_node"' not in body
    assert '"recorder"' not in body, 'recorder 인자는 2026-09-13 에 없앴다'


def test_functionbay_arm_channel_comes_from_the_profile(collect):
    """펑션베이는 명령이 이미 `JointState` 다 — 컨트롤러 토픽이 아니다."""
    defaults = collect._backend_defaults('functionbay')

    assert defaults['arm_cmd_source'] == 'joint_state'
    assert defaults['target_joint_cmds_input_topic'] == '/input/panda_joint'


# ----- 카메라 해상도·fps 는 정본이 하나다 --------------------------------------
#
# **레코더는 CFR 이라 이 값이 곧 영상의 시간축이다.** 어긋나면 오류가 아니라 재생
# 속도가 밀리므로, 백엔드마다 정본을 한 곳에만 두고 launch 가 그것을 찾아온다.


def test_isaac_geometry_comes_from_the_scene_file(collect):
    """Isaac 은 `isaac_scene.json` 이 정본이다 — 프로파일에 복사하지 않는다.

    시뮬레이터 자신이 그 JSON 을 읽어 렌더링하므로, 복사해 두면 한쪽만 고쳤을 때
    **조용히 어긋난다.**
    """
    from robot_control.backends import get_backend

    defaults = collect._backend_defaults('isaac')

    assert defaults['camera_resolution'] == '640x480'
    assert defaults['image_recorder_fps'] == 10
    # 프로파일에는 없다 — 있으면 정본이 둘이 된다.
    assert get_backend('isaac').value('camera', 'resolution') is None
    assert get_backend('isaac').value('camera', 'fps') is None


def test_functionbay_geometry_comes_from_the_profile(collect):
    """펑션베이는 공유 정의 파일이 없어 프로파일이 정본이다.

    fps 는 시뮬레이터 카메라 XML 의 `Publish_Frequency`(10)와 같아야 한다 — 5 로 두었다가
    소스(약 9.6 Hz)와 어긋나 영상이 1.9배 느리게 재생됐다 (2026-09-17).
    """
    defaults = collect._backend_defaults('functionbay')

    assert defaults['camera_resolution'] == '640x480'
    assert defaults['image_recorder_fps'] == 10


def test_mock_geometry_falls_back_to_the_yaml(collect):
    """mock 은 OpenCV 카메라라 `image_pipeline.yaml` 이 값을 *정하는* 쪽이다."""
    for name in ('mock', 'mock_jgpc'):
        defaults = collect._backend_defaults(name)
        assert defaults['camera_resolution'] is None, name
        assert defaults['image_recorder_fps'] is None, name
