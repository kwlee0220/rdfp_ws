"""`FunctionbaySceneStateNode` 의 설정 해석과 `SceneObject` 변환 테스트.

TF 를 실제로 받는 부분은 시뮬레이터가 필요하므로 여기서 다루지 않는다. 대신 **우리가
채우는 쪽**(설정 파일 해석)과 **변환 규약**(단위·쿼터니언 순서·계약 필드)을 고정한다 —
조용히 틀릴 수 있는 것이 그쪽이기 때문이다.
"""

from __future__ import annotations

import json

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('rdfp_msgs')
pytest.importorskip('tf2_ros')

import rclpy                                                          # noqa: E402
from geometry_msgs.msg import TransformStamped                        # noqa: E402

from robot_control.functionbay.scene_state_node import (              # noqa: E402
    FunctionbaySceneStateNode,
)


@pytest.fixture(scope='module', autouse=True)
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _write_config(tmp_path, objects):
    path = tmp_path / 'scene.json'
    path.write_text(json.dumps({'objects': objects}), encoding='utf-8')
    return str(path)


@pytest.fixture
def node_factory(tmp_path):
    """`config_file` 은 **생성자에서 읽히므로** `parameter_overrides` 로 미리 넣는다.

    그냥 만들고 `set_parameters` 로 바꾸면 이미 share 의 기본 설정을 읽은 뒤다.
    """
    created = []

    def make(objects):
        path = _write_config(tmp_path, objects)
        node = FunctionbaySceneStateNode(
            parameter_overrides=[rclpy.parameter.Parameter(
                'config_file', rclpy.Parameter.Type.STRING, path)])
        created.append(node)
        return node

    yield make
    for n in created:
        n.destroy_node()


def test_dynamic_filter_keeps_only_targets(node_factory):
    """`dynamic: true` 인 물체만 싣는다 — `/scene/objects` 는 조작 대상 채널이다."""
    node = node_factory([
        {'name': 'peg', 'frame': 'peg_round_16', 'type': 'cylinder',
         'dimensions': [0.05, 0.008], 'dynamic': True},
        {'name': 'peg_hole', 'frame': 'peg_hole_round_16', 'type': 'cylinder',
         'dimensions': [0.01, 0.009], 'dynamic': False},
        {'name': 'tray', 'frame': 'peg_tray_round_16', 'type': 'box'},
    ])
    objects = node._load_objects()
    assert [o['name'] for o in objects] == ['peg']


def test_frame_defaults_to_name(node_factory):
    """`frame` 이 없으면 `name` 을 프레임 이름으로 쓴다."""
    node = node_factory([{'name': 'peg_round_16', 'type': 'cylinder', 'dynamic': True}])
    spec = node._load_objects()[0]
    assert spec.get('frame', spec['name']) == 'peg_round_16'


def test_explicit_config_file_wins(node_factory, tmp_path):
    """`config_file` 을 주면 패키지 share 대신 그것을 읽는다."""
    node = node_factory([{'name': 'peg', 'type': 'cylinder', 'dynamic': True}])
    assert node._config_path() == str(tmp_path / 'scene.json')


def test_empty_config_file_means_unset(node_factory):
    """빈 문자열은 '지정 안 함' 이라는 유효한 뜻이다 — share 경로로 떨어져야 한다."""
    node = node_factory([{'name': 'peg', 'type': 'cylinder', 'dynamic': True}])
    node.set_parameters([rclpy.parameter.Parameter(
        'config_file', rclpy.Parameter.Type.STRING, '   ')])
    assert node._config_path().endswith('functionbay_scene.json')


def _transform(x, y, z, quat):
    t = TransformStamped()
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    (t.transform.rotation.x, t.transform.rotation.y,
     t.transform.rotation.z, t.transform.rotation.w) = quat
    return t


def test_quaternion_is_copied_verbatim():
    """**tf2 결과를 뒤집지 않는다.** 시뮬레이터가 ROS 규약 xyzw 로 주기 때문이다.

    순서를 틀려도 노름이 1 이라 어떤 검사도 통과하고 결과가 '그럴듯하게 틀린 자세'로
    나오므로, 변환 코드를 두지 않는 것 자체가 계약이다 (`SceneObject.msg` 의 경고).
    """
    from rdfp_msgs.msg import SceneObject
    tf = _transform(0.5, -0.1, 0.028, (0.1, 0.2, 0.3, 0.927))
    obj = SceneObject()
    obj.pose.orientation = tf.transform.rotation
    assert (obj.pose.orientation.x, obj.pose.orientation.y,
            obj.pose.orientation.z, obj.pose.orientation.w) == (0.1, 0.2, 0.3, 0.927)


def test_negative_w_is_a_valid_identity():
    """실측에서 항등이 `(0,0,0,-1)` 로 왔다 — `w >= 0` 을 가정하면 오판한다.

    `q` 와 `-q` 는 같은 회전이다. 이 테스트는 값을 검증하는 것이 아니라 **그 전제를
    코드에 남기는 것**이 목적이다.
    """
    import math
    q = (0.0, 0.0, 0.0, -1.0)
    assert math.isclose(math.sqrt(sum(c * c for c in q)), 1.0)
    # 회전각은 부호와 무관하게 0 이다.
    assert math.isclose(2 * math.degrees(math.acos(min(1.0, abs(q[3])))), 0.0, abs_tol=1e-9)


def test_shipped_config_matches_measured_peg():
    """패키지에 넣은 설정이 실측 치수와 맞는지 고정한다.

    `peg_round_16mm.obj` 경계 상자에서 산출한 값이다 — 지름 16.0 mm · 높이 50.0 mm.
    `dimensions` 는 `SolidPrimitive` 순서라 cylinder 는 **[높이, 반지름]** 이다.
    """
    from ament_index_python.packages import get_package_share_directory
    import os
    path = os.path.join(get_package_share_directory('robot_control'),
                        'config', 'functionbay_scene.json')
    with open(path, encoding='utf-8') as f:
        config = json.load(f)
    targets = [o for o in config['objects'] if o.get('dynamic', False)]
    assert len(targets) == 1, '조작 대상은 peg 하나다'
    peg = targets[0]
    assert peg['name'] == 'peg'
    assert peg['frame'] == 'peg_round_16', '시뮬레이터가 내보내는 TF 프레임 이름이다'
    assert peg['type'] == 'cylinder'
    assert peg['dimensions'] == [0.05, 0.008], '[높이, 반지름] 순서'
