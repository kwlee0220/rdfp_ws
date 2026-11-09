"""`robot_twin.api` 단위 테스트.

가짜 런타임을 주입하므로 ROS 없이 동작한다 — API 계층이 ROS 를 직접 모르게 만든
설계의 이점이다.
"""

from __future__ import annotations

from typing import Any, Optional

import pytest

# fastapi.testclient 는 httpx 를 요구한다. 없는 환경에서도 이 모듈만 조용히 빠져야
# 한다.
#
# **`pytest.importorskip` 을 최상단에 두면 안 된다.** 이 조합(pytest 6.2.5 + 패키지
# 형태의 tests 디렉터리)에서는 그 skip 이 **디렉터리 수집 자체를 끝내버려서**, 뒤따르는
# 열한 개 파일 190여 개가 통째로 사라진다. 실행 결과는 `1 skipped` 한 줄뿐이라
# **테스트가 없어진 것을 아무도 눈치채지 못한다.**
#
# 그래서 import 를 직접 감싸고 `pytestmark` 로 건너뛴다 — 모듈은 정상적으로 수집되고
# 그 안의 테스트만 skip 된다.
try:
    import httpx  # noqa: F401

    from fastapi.testclient import TestClient
    _HAS_HTTPX = True
except ImportError:  # pragma: no cover - httpx 가 있는 환경에서는 실행되지 않는다
    TestClient = None
    _HAS_HTTPX = False

pytestmark = pytest.mark.skipif(not _HAS_HTTPX, reason='fastapi.testclient requires httpx')

from robot_twin.api import create_app  # noqa: E402
from robot_twin.config import TwinConfig  # noqa: E402
from robot_twin.errors import precondition_failed  # noqa: E402
from robot_twin.session import Session, SessionStore, Status  # noqa: E402

BASE = '/api/v1/robot_twins/panda01'

RAW: dict[str, Any] = {
    'twin': {'id': 'panda01', 'description': 'test twin'},
    'moveit': {'move_group_mode': 'jtc'},
    'variables': [
        {'name': 'joint_states',
         'source': {'type': 'topic', 'topic': '/joint_states',
                    'msg': 'sensor_msgs/msg/JointState'},
         'staleness_ms': 500, 'units': {'position': 'rad'}}
    ],
    'operations': [
        {'name': 'move_to_pose', 'kind': 'async', 'resource': 'arm'},
        # 목표 표와 액션은 생략할 수 없다 — 없으면 설정 검증이 기동을 막는다.
        {'name': 'move_gripper_to_target', 'kind': 'sync', 'resource': 'gripper',
         'backend': {'topic': '/gripper_cmds',
                     'topic_type': 'rdfp_msgs/msg/GripperCommand',
                     'result_variable': 'gripper_last_command_result',
                     'labels': ['open', 'close']}}
    ]
}


class FakeRuntime:
    """API 가 기대하는 표면만 구현한 테스트용 런타임."""

    def __init__(self, config: TwinConfig) -> None:
        self.config = config
        self.sessions = SessionStore(endpoint_prefix=BASE)
        self.etag = '"7"'
        self.precondition_error: Optional[Exception] = None
        self.cancelled: list[str] = []
        self.estop_released = False
        # `?refresh=` 가 런타임까지 전달되는지 확인하기 위한 기록.
        self.refresh_calls: list[bool] = []

    def read_variable(self, name: str, *, refresh: bool = False) -> Optional[dict[str, Any]]:
        if name != 'joint_states':
            return None
        self.refresh_calls.append(refresh)
        return {'name': name, 'quality': 'OK', 'age_ms': 18, 'schema_version': 1,
                'value': {'position': {'panda_joint1': 0.1}}}

    def read_variables(self, names: list[str]) -> dict[str, Any]:
        return {n: self.read_variable(n) for n in names}

    def variable_etag(self, name: str) -> Optional[str]:
        return self.etag

    def read_variable_with_etag(self, name: str, *, refresh: bool = False):
        body = self.read_variable(name, refresh=refresh)
        return None if body is None else (body, self.etag)

    def health(self) -> dict[str, Any]:
        return {'ros': 'CONNECTED', 'move_group': 'READY'}

    def start_operation(self, name: str, inputs: dict[str, Any]) -> Session:
        if self.precondition_error is not None:
            raise self.precondition_error
        op = self.config.operation(name)
        session = self.sessions.create(operation=name, resources=op.resource_names,
                                       inputs=inputs)
        if op.kind == 'sync':
            self.sessions.finish(session.id, Status.COMPLETED, outputs={'ok': True})
        return session

    def cancel_session(self, session: Session) -> None:
        self.cancelled.append(session.id)
        self.sessions.request_cancel(session.id)

    def engage_estop(self) -> dict[str, Any]:
        affected = self.sessions.engage_estop()
        # 가짜 런타임은 정지 수단이 있다고 가정한다.
        for s in affected:
            self.sessions.finish(s.id, Status.FAILED)
        return {'locked': True, 'halted': True, 'in_flight_sessions': len(affected)}

    def release_estop(self) -> None:
        self.estop_released = True
        self.sessions.release_estop()


@pytest.fixture()
def runtime() -> FakeRuntime:
    return FakeRuntime(TwinConfig.model_validate(RAW))


@pytest.fixture()
def client(runtime: FakeRuntime) -> TestClient:
    return TestClient(create_app(runtime))


@pytest.mark.parametrize('status', ['running', 'UNKNOWN', ''])
def test_invalid_session_status_returns_400(client, status) -> None:
    response = client.get(f'{BASE}/operations/sessions', params={'status': status})

    assert response.status_code == 400
    assert response.json()['error']['code'] == 'INVALID_INPUT'


@pytest.mark.parametrize('status', list(Status))
def test_session_status_filter_accepts_terminal_and_running_states(client, runtime, status):
    session = runtime.start_operation('move_to_pose', {})
    if status is not Status.RUNNING:
        runtime.sessions.finish(session.id, status)

    response = client.get(f'{BASE}/operations/sessions', params={'status': status.value})

    assert response.status_code == 200
    assert [s['id'] for s in response.json()['sessions']] == [session.id]


@pytest.mark.parametrize('kind', ['sync', 'async'])
def test_nonfinite_outputs_are_json_null_on_all_session_routes(client, runtime, monkeypatch, kind):
    """그리퍼의 미측정 폭을 포함한 연산 결과가 POST·조회·취소에서 직렬화되어야 한다."""
    import math

    name = 'move_gripper_to_target' if kind == 'sync' else 'move_to_pose'
    start = runtime.start_operation
    created = []

    def start_with_outputs(name, inputs):
        session = start(name, inputs)
        session.outputs = {'width': float('nan'), 'nested': [float('inf'), -float('inf')]}
        created.append(session)
        return session

    monkeypatch.setattr(runtime, 'start_operation', start_with_outputs)
    response = client.post(f'{BASE}/operations/{name}', json={'inputs': {'target': 'open'}})
    assert response.status_code == (200 if kind == 'sync' else 202)
    expected = {'width': None, 'nested': [None, None]}
    assert response.json()['outputs'] == expected
    session = created[0]
    endpoint = f'{BASE}/operations/sessions/{session.id}'
    assert client.get(endpoint).json()['outputs'] == expected
    for path in ('operations/sessions', f'operations/{name}/sessions'):
        assert client.get(f'{BASE}/{path}').json()['sessions'][0]['outputs'] == expected
    if kind == 'async':
        cancelled = client.delete(endpoint)
        assert cancelled.status_code == 202
        assert cancelled.json()['outputs'] == expected
    assert math.isnan(session.outputs['width'])  # 응답 변환이 저장된 결과를 바꾸지 않는다.


# ----------------------------------------------------------------------
# 디스커버리
# ----------------------------------------------------------------------

def test_twin_list_returns_only_itself(client: TestClient) -> None:
    """1 트윈 = 1 프로세스이므로 레지스트리가 아니다 (설계서 2.4)."""
    body = client.get('/api/v1/robot_twins').json()

    assert len(body['twins']) == 1
    assert body['twins'][0]['id'] == 'panda01'
    assert 'backend' in body['twins'][0]


def test_twin_meta_exposes_the_backend() -> None:
    """id 는 환경을 담지 않으므로(세 환경이 같은 id) 환경은 메타의 `backend` 로 안다."""
    pytest.importorskip('robot_control.backend_profiles')
    raw = {**RAW, 'twin': {**RAW['twin'], 'backend': 'panda_mock'}, 'moveit': {}}
    body = TestClient(create_app(FakeRuntime(TwinConfig.model_validate(raw)))).get(BASE).json()

    assert body['backend'] == 'panda_mock'
    assert body['move_group_mode'] == 'jtc'


def test_twin_meta_exposes_closed_loop(client: TestClient) -> None:
    """jgpc 는 open loop 라 클라이언트가 호출 전에 알아야 한다 (설계서 6.7)."""
    body = client.get(BASE).json()

    assert body['move_group_mode'] == 'jtc'
    assert body['closed_loop'] is True


def test_health(client: TestClient) -> None:
    assert client.get(f'{BASE}/health').json()['ros'] == 'CONNECTED'


# ----------------------------------------------------------------------
# 상태 변수
# ----------------------------------------------------------------------

def test_variable_catalog(client: TestClient) -> None:
    body = client.get(f'{BASE}/variables').json()

    assert body['variables'][0]['name'] == 'joint_states'
    assert body['variables'][0]['units'] == {'position': 'rad'}


def test_get_variable_sets_etag_and_no_cache(client: TestClient) -> None:
    res = client.get(f'{BASE}/variables/joint_states')

    assert res.status_code == 200
    assert res.headers['etag'] == '"7"'
    # 프록시가 로봇 상태를 캐시하면 재앙이다 (설계서 5.6).
    assert res.headers['cache-control'] == 'no-cache, must-revalidate'


def test_conditional_get_returns_304(client: TestClient) -> None:
    res = client.get(f'{BASE}/variables/joint_states', headers={'If-None-Match': '"7"'})

    assert res.status_code == 304


def test_conditional_get_with_stale_etag_returns_body(client: TestClient) -> None:
    res = client.get(f'{BASE}/variables/joint_states', headers={'If-None-Match': '"6"'})

    assert res.status_code == 200
    assert res.json()['quality'] == 'OK'


def test_etag_comes_from_the_same_read_as_the_body(client: TestClient,
                                                   runtime: FakeRuntime) -> None:
    """ETag 를 본문과 따로 읽으면 그 사이 새 메시지로 본문보다 새 ETag 가 나간다.

    그러면 그 ETag 의 조건부 조회가 `304` 로 막혀 새 값을 놓친다 (설계서 5.6).
    """
    runtime.read_variable_with_etag = lambda name, refresh=False: (
        {'name': name, 'quality': 'OK', 'schema_version': 1, 'value': 1}, '"5"')
    runtime.etag = '"9"'                       # 그 뒤에 세대가 올라갔다

    res = client.get(f'{BASE}/variables/joint_states')
    assert res.headers['etag'] == '"5"'

    res = client.get(f'{BASE}/variables/joint_states', headers={'If-None-Match': '"9"'})
    assert res.status_code == 200              # 본문보다 새 ETag 로 304 가 나면 안 된다


def test_head_variable(client: TestClient) -> None:
    res = client.head(f'{BASE}/variables/joint_states')

    assert res.status_code == 200
    assert res.headers['etag'] == '"7"'


# ----- 요청 형식 오류 — 다른 거부와 같은 `error` 형식 ---------------------------
#
# FastAPI 기본값(`422 {"detail": [...]}`)이면 `error` 만 읽는 클라이언트(robot-twin-client 의
# `TwinError`, MCP 서버)에서 사유가 통째로 사라진다 — `HTTP 422 None: None`.

@pytest.mark.parametrize('kwargs', [
    {'content': b'{not json', 'headers': {'Content-Type': 'application/json'}},   # JSON 문법
    {'json': [1, 2]},                                                            # 객체 대신 배열
], ids=['bad-json', 'array-body'])
def test_malformed_body_is_400_invalid_input(client: TestClient, kwargs) -> None:
    res = client.post(f'{BASE}/operations/move_to_pose', **kwargs)

    assert res.status_code == 400
    error = res.json()['error']
    assert error['code'] == 'INVALID_INPUT'
    assert 'body' in error['message']
    assert error['details']                      # FastAPI 의 원본 목록을 남긴다
    assert 'detail' not in res.json()


def test_malformed_query_is_400_invalid_input(client: TestClient) -> None:
    res = client.get(f'{BASE}/variables/joint_states', params={'refresh': 'not-a-bool'})

    assert res.status_code == 400
    assert res.json()['error']['code'] == 'INVALID_INPUT'
    assert 'query.refresh' in res.json()['error']['message']


def test_refresh_query_reaches_runtime(client: TestClient, runtime: FakeRuntime) -> None:
    """``?refresh=true`` 는 정적 소스의 유일한 갱신 수단이다 (설계서 4.3)."""
    client.get(f'{BASE}/variables/joint_states')
    client.get(f'{BASE}/variables/joint_states', params={'refresh': 'true'})

    assert runtime.refresh_calls == [False, True]


def test_unknown_variable_is_404(client: TestClient) -> None:
    """정의되지 않은 변수만 404 다 — 값 없음은 200 + NO_DATA (설계서 5.2)."""
    res = client.get(f'{BASE}/variables/nope')

    assert res.status_code == 404
    assert res.json()['error']['code'] == 'NOT_FOUND'


def test_batch_read(client: TestClient) -> None:
    res = client.get(f'{BASE}/variables', params={'names': 'joint_states'})

    assert res.status_code == 200
    assert res.json()['values']['joint_states']['quality'] == 'OK'


def test_batch_read_with_unknown_name_is_404(client: TestClient) -> None:
    res = client.get(f'{BASE}/variables', params={'names': 'joint_states,nope'})

    assert res.status_code == 404
    assert 'nope' in res.json()['error']['message']


# ----------------------------------------------------------------------
# 연산
# ----------------------------------------------------------------------

def test_operation_catalog_exposes_kind_and_idempotent(client: TestClient) -> None:
    ops = {o['name']: o for o in client.get(f'{BASE}/operations').json()['operations']}

    assert ops['move_to_pose']['kind'] == 'async'
    assert ops['move_gripper_to_target']['kind'] == 'sync'
    assert ops['move_to_pose']['idempotent'] is True


def test_async_operation_returns_202_with_location(client: TestClient) -> None:
    res = client.post(f'{BASE}/operations/move_to_pose', json={'inputs': {'x': 1}})

    assert res.status_code == 202
    body = res.json()
    assert body['status'] == 'RUNNING'
    assert body['session_endpoint'].startswith(f'{BASE}/operations/sessions/')
    assert res.headers['location'] == body['session_endpoint']


def test_sync_operation_returns_200_completed(client: TestClient) -> None:
    res = client.post(f'{BASE}/operations/move_gripper_to_target', json={})

    assert res.status_code == 200
    assert res.json()['status'] == 'COMPLETED'
    # 동기 연산은 세션 엔드포인트를 발급하지 않는다.
    assert 'session_endpoint' not in res.json()


def test_resource_conflict_returns_409_with_retry_after(client: TestClient,
                                                        runtime: FakeRuntime) -> None:
    first = client.post(f'{BASE}/operations/move_to_pose', json={}).json()
    sid = first['session_endpoint'].rsplit('/', 1)[-1]
    runtime.sessions.update(sid, planned_duration_ms=20000)

    res = client.post(f'{BASE}/operations/move_to_pose', json={})

    assert res.status_code == 409
    assert res.json()['error']['code'] == 'RESOURCE_BUSY'
    assert res.json()['error']['occupied_by'].endswith(sid)
    assert 'retry-after' in res.headers


def test_precondition_failure_returns_503(client: TestClient, runtime: FakeRuntime) -> None:
    """실행이 시작조차 되지 않았으므로 200 + FAILED 가 아니다 (설계서 6.8)."""
    runtime.precondition_error = precondition_failed('move_group is not ready')

    res = client.post(f'{BASE}/operations/move_to_pose', json={})

    assert res.status_code == 503
    assert res.json()['error']['code'] == 'PRECONDITION_FAILED'


def test_unknown_operation_is_404(client: TestClient) -> None:
    assert client.post(f'{BASE}/operations/nope', json={}).status_code == 404


def test_get_session(client: TestClient) -> None:
    endpoint = client.post(f'{BASE}/operations/move_to_pose', json={}).json()['session_endpoint']

    body = client.get(endpoint).json()

    assert body['status'] == 'RUNNING'
    assert body['phase'] == 'EXECUTING'


def test_cancel_returns_202_and_keeps_session(client: TestClient) -> None:
    """감속 정지에 시간이 걸리므로 즉시 확정하지 않는다 (설계서 6.4)."""
    endpoint = client.post(f'{BASE}/operations/move_to_pose', json={}).json()['session_endpoint']

    res = client.delete(endpoint)

    assert res.status_code == 202
    assert res.json()['status'] == 'RUNNING'
    assert res.json()['phase'] == 'CANCELING'
    # 세션은 유지되어야 최종 CANCELLED 를 확인할 수 있다.
    assert client.get(endpoint).status_code == 200


def test_cancel_terminal_session_is_409(client: TestClient, runtime: FakeRuntime) -> None:
    endpoint = client.post(f'{BASE}/operations/move_to_pose', json={}).json()['session_endpoint']
    sid = endpoint.rsplit('/', 1)[-1]
    runtime.sessions.finish(sid, Status.COMPLETED)

    assert client.delete(endpoint).status_code == 409


def test_missing_session_404_warns_outcome_unknown(client: TestClient) -> None:
    """폴링 중 404 를 완료로 가정하지 않도록 안내한다 (설계서 6.13)."""
    res = client.get(f'{BASE}/operations/sessions/deadbeef')

    assert res.status_code == 404
    assert res.json()['error']['code'] == 'SESSION_NOT_FOUND'
    assert 'unknown' in res.json()['error']['message']


def test_session_listings(client: TestClient) -> None:
    client.post(f'{BASE}/operations/move_to_pose', json={})

    assert len(client.get(f'{BASE}/operations/sessions').json()['sessions']) >= 1
    assert len(client.get(f'{BASE}/operations/move_to_pose/sessions').json()['sessions']) == 1
    running = client.get(f'{BASE}/operations/sessions', params={'status': 'RUNNING'}).json()
    assert len(running['sessions']) == 1


# ----------------------------------------------------------------------
# 자원 · 안전
# ----------------------------------------------------------------------

def test_resources_view(client: TestClient) -> None:
    client.post(f'{BASE}/operations/move_to_pose', json={})

    body = client.get(f'{BASE}/resources').json()

    assert body['arm']['state'] == 'BUSY'
    assert body['gripper']['state'] == 'FREE'


def test_estop_blocks_then_releases(client: TestClient) -> None:
    client.post(f'{BASE}/operations/move_to_pose', json={})

    engaged = client.post(f'{BASE}/estop').json()
    assert engaged['estop'] == 'ENGAGED'
    assert engaged['locked'] is True
    assert engaged['in_flight_sessions'] == 1

    blocked = client.post(f'{BASE}/operations/move_gripper_to_target', json={})
    assert blocked.status_code == 409
    assert blocked.json()['error']['code'] == 'ESTOP_ENGAGED'

    assert client.delete(f'{BASE}/estop').json()['estop'] == 'RELEASED'
    assert client.post(f'{BASE}/operations/move_gripper_to_target', json={}).status_code == 200
