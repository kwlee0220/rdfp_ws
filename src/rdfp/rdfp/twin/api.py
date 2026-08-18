"""REST API 계층 (설계서 7장).

extern_op 프로토콜을 따르되 본 설계의 확장을 얹는다. HTTP 핸들러는 **캐시와 세션
보관소만 읽고 쓴다** — rclpy 동기 API 를 직접 호출하면 executor 스레드를 기다리다
데드락에 빠질 수 있다 (설계서 2.3).

경로에 ``variables/`` / ``operations/`` 세그먼트를 두어 변수·연산 이름이 트윈 하위
리소스(`health`, `resources`)와 같은 경로 공간을 쓰지 않게 한다.
"""

from __future__ import annotations

from typing import Any, Optional

from fastapi import FastAPI, Request, Response
from fastapi.responses import JSONResponse

from rdfp.twin.config import TwinConfig
from rdfp.twin.errors import TwinError
from rdfp.twin.session import Session, SessionStore, Status

API_PREFIX = '/api/v1'

# 로봇 상태 응답이 중간 프록시에 캐시되면 재앙이다. no-store 가 아니라 no-cache
# 여야 ETag 재검증이 동작한다 (설계서 5.6).
_NO_CACHE = 'no-cache, must-revalidate'


class TwinRuntime:
    """API 가 의존하는 런타임 표면.

    구현체는 :mod:`rdfp.twin.main` 이 조립한다. API 계층이 ROS 를 직접 모르게 하여
    테스트에서 가짜 런타임으로 대체할 수 있다.
    """

    config: TwinConfig
    sessions: SessionStore

    def read_variable(self, name: str) -> Optional[dict[str, Any]]:
        """단일 변수 envelope. 정의되지 않은 이름이면 ``None``."""
        raise NotImplementedError

    def read_variables(self, names: list[str]) -> dict[str, Any]:
        """배치 조회 결과."""
        raise NotImplementedError

    def variable_etag(self, name: str) -> Optional[str]:
        """변수의 현재 ETag."""
        raise NotImplementedError

    def health(self) -> dict[str, Any]:
        """ROS 연결·준비 상태."""
        raise NotImplementedError

    def start_operation(self, name: str, inputs: dict[str, Any]) -> Session:
        """연산을 시작한다. 거부 시 :class:`TwinError` 를 던진다."""
        raise NotImplementedError

    def cancel_session(self, session: Session) -> None:
        """세션 취소를 백엔드에 전달한다."""
        raise NotImplementedError

    def engage_estop(self) -> dict[str, Any]:
        """비상정지를 건다. ``halted`` 로 실제 정지 여부를 알린다."""
        raise NotImplementedError

    def release_estop(self) -> None:
        """비상정지를 해제한다."""
        raise NotImplementedError


def session_body(session: Session, *, endpoint: Optional[str] = None) -> dict[str, Any]:
    """세션을 extern_op 응답 본문으로 변환한다.

    ``CANCELLED`` 에는 ``error`` 를 붙이지 않는다 — 클라이언트 요청에 의한 정상
    종료이므로 오류가 아니다 (설계서 6.8).
    """
    body: dict[str, Any] = {'status': session.status.value}
    if session.phase is not None:
        body['phase'] = session.phase.value
    if session.progress is not None:
        body['progress'] = session.progress
    if session.message is not None:
        body['message'] = session.message
    if session.outputs:
        body['outputs'] = session.outputs
    if session.error_code is not None:
        body['error'] = {'code': session.error_code.value,
                         'message': session.error_message or ''}
    if endpoint is not None:
        body['session_endpoint'] = endpoint
    return body


def create_app(runtime: TwinRuntime) -> FastAPI:
    """트윈의 FastAPI 애플리케이션을 만든다."""
    cfg = runtime.config
    twin_id = cfg.twin.id
    base = f'{API_PREFIX}/robot_twins/{twin_id}'

    app = FastAPI(
        title=f'Robot Twin — {twin_id}',
        description=cfg.twin.description or 'ROS 2 robot gateway (extern_op protocol)',
        version='0.1.0'
    )

    @app.exception_handler(TwinError)
    async def _twin_error_handler(request: Request, exc: TwinError) -> JSONResponse:
        headers = {}
        if exc.retry_after_sec is not None:
            headers['Retry-After'] = str(int(exc.retry_after_sec))
        return JSONResponse(exc.to_body(), status_code=exc.http_status, headers=headers)

    # ------------------------------------------------------------------
    # 디스커버리
    # ------------------------------------------------------------------

    @app.get(f'{API_PREFIX}/robot_twins')
    def list_twins() -> dict[str, Any]:
        """트윈 목록.

        **항상 자기 자신 하나만 반환한다** — 1 트윈 = 1 프로세스이므로 이 엔드포인트는
        트윈 레지스트리가 아니다 (설계서 2.4).
        """
        return {'twins': [{'id': twin_id, 'description': cfg.twin.description,
                           'endpoint': base}]}

    @app.get(base)
    def twin_meta() -> dict[str, Any]:
        """트윈 메타와 변수·연산 카탈로그."""
        return {
            'id': twin_id,
            'description': cfg.twin.description,
            'move_group_mode': cfg.moveit.move_group_mode,
            # jgpc 스트리밍은 open loop 다 — 정상 종료가 도달을 보장하지 않는다.
            # 클라이언트가 연산 호출 전에 알 수 있게 노출한다 (설계서 6.7).
            'closed_loop': cfg.moveit.move_group_mode == 'jtc',
            'variables': [v.name for v in cfg.variables],
            'operations': [o.name for o in cfg.operations]
        }

    @app.get(f'{base}/health')
    def health() -> dict[str, Any]:
        """ROS 연결·준비 상태.

        변수 품질(`quality`)과는 별개다. `quality: OK` 라도 컨트롤러가 없으면 연산은
        받을 수 없으므로 클라이언트는 둘 다 확인해야 한다 (설계서 5.1).
        """
        return runtime.health()

    # ------------------------------------------------------------------
    # 상태 변수
    # ------------------------------------------------------------------

    @app.get(f'{base}/variables')
    def list_variables(names: Optional[str] = None) -> Any:
        """변수 목록 또는 배치 조회.

        ``names`` 가 있으면 배치 조회다. 보장 수준은 "동일 시점에 읽은 각 변수의
        최신값"이며, 각 값이 같은 시각에 생성되었다는 뜻이 아니다 (설계서 5.4).
        """
        if names is None:
            return {'variables': [
                {'name': v.name, 'description': v.description,
                 'source_type': v.source.type,
                 'staleness_ms': v.staleness_ms, 'units': v.units,
                 'schema_version': v.schema_version}
                for v in cfg.variables
            ]}

        requested = [n.strip() for n in names.split(',') if n.strip()]
        results = runtime.read_variables(requested)
        unknown = sorted(n for n, v in results.items() if v is None)
        if unknown:
            # 정의되지 않은 변수는 프로토콜 오류다. 반면 정의된 변수의 값 없음이나
            # 변환 실패는 200 + quality 로 표현한다 (설계서 5.3).
            return JSONResponse(
                {'error': {'code': 'NOT_FOUND',
                           'message': f"unknown variable(s): {', '.join(unknown)}"}},
                status_code=404
            )
        return {'values': results}

    @app.get(f'{base}/variables/{{name}}')
    def get_variable(name: str, response: Response, request: Request,
                     refresh: bool = False) -> Any:
        """단일 변수 조회. ETag 가 일치하면 ``304`` 를 반환한다.

        ``?refresh=true`` 는 정적 소스(``source.type: static``)의 캐시를 버리고 다시
        조회한다. 정적 소스는 자동 무효화를 하지 않으므로 트윈 재시작 외에는 이것이
        유일한 갱신 수단이다 (설계서 4.3). 다른 소스 타입에는 영향이 없다.
        """
        body = runtime.read_variable(name, refresh=refresh)
        if body is None:
            return JSONResponse(_unknown_variable_body(name), status_code=404)

        etag = runtime.variable_etag(name) or '"0"'
        response.headers['Cache-Control'] = _NO_CACHE
        response.headers['ETag'] = etag
        if request.headers.get('if-none-match') == etag:
            return Response(status_code=304, headers={'ETag': etag,
                                                      'Cache-Control': _NO_CACHE})
        return body

    @app.head(f'{base}/variables/{{name}}')
    def head_variable(name: str) -> Response:
        """값 없이 ETag 만 확인한다."""
        if runtime.read_variable(name) is None:
            return Response(status_code=404)
        etag = runtime.variable_etag(name) or '"0"'
        return Response(status_code=200, headers={'ETag': etag, 'Cache-Control': _NO_CACHE})

    # ------------------------------------------------------------------
    # 자원 점유
    # ------------------------------------------------------------------

    @app.get(f'{base}/resources')
    def resources() -> dict[str, Any]:
        """자원별 점유 상태.

        시작 전 확인 편의를 위한 것이며 배타성 보장 수단이 아니다 — 조회와 시작
        사이의 경합은 남으므로 클라이언트는 ``409`` 처리를 생략할 수 없다.
        """
        return runtime.sessions.resources()

    # ------------------------------------------------------------------
    # 연산
    # ------------------------------------------------------------------

    @app.get(f'{base}/operations')
    def list_operations() -> dict[str, Any]:
        """연산 카탈로그.

        ``kind`` 는 연산별로 고정된 값이므로, 클라이언트는 호출 전에 폴링 루프
        필요 여부를 알 수 있다 (설계서 6.2).
        """
        return {'operations': [
            {'name': o.name, 'description': o.description,
             'kind': o.kind, 'resource': list(o.resource_names),
             'idempotent': o.idempotent, 'inputs_schema': o.inputs_schema,
             'endpoint': f'{base}/operations/{o.name}'}
            for o in cfg.operations
        ]}

    @app.post(f'{base}/operations/{{name}}')
    def start_operation(name: str, payload: Optional[dict[str, Any]] = None) -> Any:
        """연산을 시작한다.

        동기 연산은 ``200`` + 종료 상태, 비동기 연산은 ``202`` + ``RUNNING`` +
        ``session_endpoint`` 를 반환한다. 거부(``409``/``503``/``400``)는 세션을
        만들지 않는다.
        """
        op = cfg.operation(name)
        if op is None:
            return JSONResponse(
                {'error': {'code': 'NOT_FOUND', 'message': f'unknown operation: {name}'}},
                status_code=404
            )

        inputs = (payload or {}).get('inputs') or {}
        session = runtime.start_operation(name, inputs)

        if op.kind == 'sync':
            return session_body(session)

        endpoint = runtime.sessions.endpoint_of(session.id)
        return JSONResponse(
            session_body(session, endpoint=endpoint), status_code=202,
            # body 의 session_endpoint 와 중복이지만 표준 클라이언트가 자동 추적한다.
            headers={'Location': endpoint}
        )

    @app.get(f'{base}/operations/sessions')
    def list_sessions(status: Optional[str] = None) -> dict[str, Any]:
        """트윈 전체 세션 목록 (운영·디버깅용)."""
        wanted = Status(status) if status else None
        return {'sessions': [
            {'id': s.id, 'operation': s.operation, 'resource': list(s.resources),
             'endpoint': runtime.sessions.endpoint_of(s.id), **session_body(s)}
            for s in runtime.sessions.list(status=wanted)
        ]}

    @app.get(f'{base}/operations/{{name}}/sessions')
    def list_operation_sessions(name: str) -> dict[str, Any]:
        """특정 연산의 세션 목록."""
        return {'sessions': [
            {'id': s.id, 'endpoint': runtime.sessions.endpoint_of(s.id), **session_body(s)}
            for s in runtime.sessions.list(operation=name)
        ]}

    @app.get(f'{base}/operations/sessions/{{session_id}}')
    def get_session(session_id: str) -> Any:
        """세션 상태 조회 (``session_endpoint``)."""
        session = runtime.sessions.get(session_id)
        if session is None:
            return JSONResponse(_session_not_found_body(session_id), status_code=404)
        return session_body(session)

    @app.delete(f'{base}/operations/sessions/{{session_id}}')
    def cancel_session(session_id: str) -> Any:
        """세션 취소.

        로봇은 감속 정지에 시간이 걸리므로 ``202`` + ``phase: CANCELING`` 을 반환하고,
        **세션을 유지**한다. 최종 ``CANCELLED`` 는 ``GET`` 으로 확인한다 (설계서 6.4).
        """
        session = runtime.sessions.get(session_id)
        if session is None:
            return JSONResponse(_session_not_found_body(session_id), status_code=404)
        if session.is_terminal:
            return JSONResponse(
                {'error': {'code': 'NOT_CANCELLABLE',
                           'message': f'session already {session.status.value}'}},
                status_code=409
            )

        runtime.cancel_session(session)
        return JSONResponse(session_body(session), status_code=202)

    # ------------------------------------------------------------------
    # 안전
    # ------------------------------------------------------------------

    @app.post(f'{base}/estop')
    def engage_estop() -> dict[str, Any]:
        """비상정지.

        세션 id 를 몰라도 호출할 수 있어야 하므로 세션과 무관한 채널이다. 권한
        개념을 두지 않으므로 누구든 호출할 수 있다 (설계서 6.4, 8.4).
        """
        result = runtime.engage_estop()
        body: dict[str, Any] = {'estop': 'ENGAGED', **result}
        if not result.get('halted') and result.get('in_flight_sessions'):
            # 잠금은 걸렸지만 로봇은 계속 움직인다. 이 사실을 숨기면 클라이언트가
            # 멈췄다고 오인한다 (설계서 9장 격차).
            body['warning'] = (
                'in-flight motion could NOT be halted; the robot keeps moving. '
                'Use a physical emergency stop'
            )
        return body

    @app.delete(f'{base}/estop')
    def release_estop() -> dict[str, Any]:
        """비상정지 해제."""
        runtime.release_estop()
        return {'estop': 'RELEASED'}

    return app


def _unknown_variable_body(name: str) -> dict[str, Any]:
    """정의되지 않은 변수의 ``404`` 본문.

    변수는 있고 값만 없는 경우(``NO_DATA``)와 구분해야 한다 (설계서 5.2).
    """
    return {'error': {'code': 'NOT_FOUND', 'message': f'unknown variable: {name}'}}


def _session_not_found_body(session_id: str) -> dict[str, Any]:
    """세션 ``404`` 본문.

    만료·축출·재시작을 구분하지 않는다 (설계서 6.13). 클라이언트는 활성 폴링 중
    ``404`` 를 받으면 결과를 알 수 없는 것으로 간주하고 상태 변수를 확인해야 한다.
    """
    return {'error': {
        'code': 'SESSION_NOT_FOUND',
        'message': (f'session {session_id} not found; it may have expired, been evicted, '
                    'or the twin restarted. Treat the operation outcome as unknown and '
                    'verify robot state via variables.')
    }}


__all__ = ['API_PREFIX', 'TwinRuntime', 'create_app', 'session_body']
