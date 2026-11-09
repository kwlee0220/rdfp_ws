"""장시간 작업 = 기존 CLI 를 **서브프로세스로** (설계안 §6.3 · §6.4).

GUI 는 적재 · export 코드를 직접 부르지 않는다. `ros2 run rdfp import …` 를 띄우고 출력을 흘려 보일
뿐이다 — 그래서 GUI 에서 한 조작이 **그대로 CLI 명령**이 되고(하단 패널에 보인다 · 복사된다),
GUI 가 죽어도 적재 트랜잭션은 자기 프로세스에서 끝난다.

`ros2` 가 PATH 에 없으면(소싱 안 한 셸) `python -m rdfp.dataset.import_cmd` 로 대신 부른다. 적재는
어차피 ROS 메시지 패키지를 요구하므로 그 경우는 export 처럼 ROS 없이 도는 명령에나 쓸모가 있다.
"""

from __future__ import annotations

from typing import Iterable, Optional, Sequence

import json
import os
import re
import shlex
import shutil
import sys
from dataclasses import dataclass, field

from PySide6.QtCore import QObject, QProcess, QProcessEnvironment, QTimer, Signal

#: `ros2 run rdfp <script>` 가 안 될 때의 모듈 대체.
RDFP_SCRIPT_MODULES = {
    'import': 'rdfp.dataset.import_cmd',
    'export': 'rdfp.dataset.export_cmd',
    'episode-delete': 'rdfp.dataset.episode_delete_cmd',
}

KIND_IMPORT = 'import'
KIND_EXPORT = 'export'
KIND_REINDEX = 'reindex'
KIND_EPISODE_DELETE = 'episode-delete'


@dataclass
class JobSpec:
    """실행할 명령 하나. `argv` 가 곧 사용자에게 보이는 CLI 다."""

    title: str
    argv: list
    kind: str
    cwd: Optional[str] = None
    total_hint: Optional[int] = None    # 진행률의 분모를 미리 알면 (에피소드 수)
    notes: list = field(default_factory=list)

    def command_line(self) -> str:
        return shlex.join(self.argv)


def rdfp_command(script: str, *, prefer_ros2: Optional[bool] = None) -> list:
    """`ros2 run rdfp <script>` 또는 `python -m <module>`."""
    use_ros2 = shutil.which('ros2') is not None if prefer_ros2 is None else prefer_ros2
    if use_ros2:
        return ['ros2', 'run', 'rdfp', script]
    return [sys.executable, '-m', RDFP_SCRIPT_MODULES[script]]


def episodes_spec(ids: Iterable[int]) -> str:
    """`[1, 2, 3, 7]` → `1-3,7` — `export --episodes` · `episode-delete --ids` 가 읽는 표기."""
    values = sorted(set(int(i) for i in ids))
    if not values:
        return ''
    parts: list = []
    start = prev = values[0]
    for v in values[1:]:
        if v == prev + 1:
            prev = v
            continue
        parts.append(f'{start}-{prev}' if start != prev else str(start))
        start = prev = v
    parts.append(f'{start}-{prev}' if start != prev else str(start))
    return ','.join(parts)


def build_import_command(config_path: str, *, start_ns: Sequence[int] = (),
                         topics: Sequence[str] = (), on_existing: Optional[str] = None,
                         prefer_ros2: Optional[bool] = None) -> JobSpec:
    """`ros2 run rdfp import --config … [--start-ns …] [--topics …] [--on-existing …]`.

    선택이 없으면 인자도 없다 — 설정 파일만으로 돌리는 평소의 CLI 와 같은 명령이 된다.
    """
    argv = rdfp_command('import', prefer_ros2=prefer_ros2) + ['--config', str(config_path)]
    for ns in start_ns:
        argv += ['--start-ns', str(int(ns))]
    if topics:
        argv += ['--topics', *topics]
    if on_existing:
        argv += ['--on-existing', on_existing]
    return JobSpec(title='import', argv=argv, kind=KIND_IMPORT,
                   total_hint=len(start_ns) or None)


def build_export_command(config_path: str, out_dir: str, *, episode_ids: Sequence[int],
                         label: str = 'joint_plan', chunk_len: int = 20, delta_sec: float = 0.1,
                         step_sec: float = 0.1, camera_topic: str = '',
                         keep_untrustworthy: bool = False,
                         prefer_ros2: Optional[bool] = None) -> JobSpec:
    argv = rdfp_command('export', prefer_ros2=prefer_ros2) + [
        '--config', str(config_path), '--out', str(out_dir)]
    if episode_ids:
        argv += ['--episodes', episodes_spec(episode_ids)]
    argv += ['--label', label, '--chunk-len', str(int(chunk_len)),
             '--delta-sec', repr(float(delta_sec)), '--step-sec', repr(float(step_sec))]
    if camera_topic:
        argv += ['--camera-topic', camera_topic]
    if keep_untrustworthy:
        argv.append('--keep-untrustworthy')
    return JobSpec(title='export', argv=argv, kind=KIND_EXPORT, total_hint=len(episode_ids) or None)


def build_reindex_command(session_dir: str) -> JobSpec:
    """`ros2 bag reindex -s mcap <세션>` — `metadata.yaml` 을 다시 만든다. `ros2` 가 필요하다."""
    return JobSpec(title='reindex', argv=['ros2', 'bag', 'reindex', '-s', 'mcap', str(session_dir)],
                   kind=KIND_REINDEX)


def build_episode_delete_command(config_path: str, ids: Sequence[int], *,
                                 delete_files: bool = False,
                                 prefer_ros2: Optional[bool] = None) -> JobSpec:
    """`--yes` 를 붙인다 — 확인은 GUI 다이얼로그가 이미 받았다 (타이핑 확인, §6.6)."""
    argv = rdfp_command('episode-delete', prefer_ros2=prefer_ros2) + [
        '--config', str(config_path), '--ids', episodes_spec(ids), '--yes']
    if delete_files:
        argv.append('--delete-files')
    return JobSpec(title='episode-delete', argv=argv, kind=KIND_EPISODE_DELETE)


# ----- 진행률 ---------------------------------------------------------------

_RE_DETECTED = re.compile(r'detected (\d+) episode')
_RE_REMAIN = re.compile(r'(\d+) episode\(s\) remain after filters')
_RE_COMMITTED = re.compile(r'episode committed: id=(\d+)')
_RE_SKIPPED = re.compile(r'episode already present')
_RE_FAILED = re.compile(r'failed to open episode|insert failed|commit failed|image encoding failed'
                        r'|post-processing failed')
_RE_EXPORTING = re.compile(r'exporting (\d+) episode')
_RE_EXPORTED = re.compile(r'episode (\d+): \d+ sample')


class ImportProgress:
    """`import` · `export` 로그 줄에서 진행률을 읽는다 — CLI 에 진행 출력 옵션을 더하지 않고도 된다.

    분모는 `N episode(s) remain after filters`(import) · `exporting N episode(s)`(export) 에서,
    분자는 `episode committed` · `already present` · 실패 줄에서 센다.
    """

    def __init__(self, total_hint: Optional[int] = None) -> None:
        self.total: Optional[int] = total_hint
        self.committed = 0
        self.skipped = 0
        self.failed = 0
        self.last_episode_id: Optional[int] = None

    @property
    def done(self) -> int:
        return self.committed + self.skipped + self.failed

    def feed(self, line: str) -> bool:
        """한 줄을 읽고 무엇이든 바뀌면 참."""
        m = _RE_REMAIN.search(line) or _RE_EXPORTING.search(line)
        if m:
            self.total = int(m.group(1))
            return True
        if _RE_DETECTED.search(line) and self.total is None:
            self.total = int(_RE_DETECTED.search(line).group(1))
            return True
        m = _RE_COMMITTED.search(line) or _RE_EXPORTED.search(line)
        if m:
            self.committed += 1
            self.last_episode_id = int(m.group(1))
            return True
        if _RE_SKIPPED.search(line):
            self.skipped += 1
            return True
        if _RE_FAILED.search(line):
            self.failed += 1
            return True
        return False

    def status(self) -> str:
        total = f'/{self.total}' if self.total is not None else ''
        extra = []
        if self.skipped:
            extra.append(f'skip {self.skipped}')
        if self.failed:
            extra.append(f'fail {self.failed}')
        tail = f' ({", ".join(extra)})' if extra else ''
        return f'{self.done}{total} 에피소드{tail}'


def parse_summary_json(stdout_text: str) -> Optional[dict]:
    """stdout 의 마지막 JSON 객체 — `import` 가 끝에 찍는 요약."""
    text = stdout_text.strip()
    start = text.rfind('\n{')
    candidate = text[start + 1:] if start >= 0 else text
    if not candidate.startswith('{'):
        return None
    try:
        parsed = json.loads(candidate)
    except json.JSONDecodeError:
        return None
    return parsed if isinstance(parsed, dict) else None


# ----- 실행기 ---------------------------------------------------------------

class JobRunner(QObject):
    """한 번에 하나의 `JobSpec` 을 `QProcess` 로 돈다. 출력은 줄 단위 시그널로 흘린다."""

    started = Signal(object)            # JobSpec
    line = Signal(str)                  # stdout · stderr 한 줄
    progress = Signal(int, int, str)    # done, total(0 = 모름), 상태 문구
    finished = Signal(int, object)      # exit code, 요약 dict | None

    KILL_GRACE_MS = 5000

    def __init__(self, parent: Optional[QObject] = None) -> None:
        super().__init__(parent)
        self._proc: Optional[QProcess] = None
        self._spec: Optional[JobSpec] = None
        self._progress: Optional[ImportProgress] = None
        self._stdout_buf = ''
        self._stderr_buf = ''
        self._stdout_all: list = []

    @property
    def busy(self) -> bool:
        return self._proc is not None and self._proc.state() != QProcess.ProcessState.NotRunning

    @property
    def current(self) -> Optional[JobSpec]:
        return self._spec

    def start(self, spec: JobSpec) -> bool:
        if self.busy:
            return False
        self._spec = spec
        self._progress = ImportProgress(spec.total_hint)
        self._stdout_buf = self._stderr_buf = ''
        self._stdout_all = []
        proc = QProcess(self)
        env = QProcessEnvironment.systemEnvironment()
        env.insert('PYTHONUNBUFFERED', '1')
        proc.setProcessEnvironment(env)
        if spec.cwd:
            proc.setWorkingDirectory(spec.cwd)
        proc.setProgram(spec.argv[0])
        proc.setArguments(spec.argv[1:])
        proc.readyReadStandardOutput.connect(self._on_stdout)
        proc.readyReadStandardError.connect(self._on_stderr)
        proc.finished.connect(self._on_finished)
        proc.errorOccurred.connect(self._on_error)
        self._proc = proc
        self.started.emit(spec)
        self.line.emit(f'$ {spec.command_line()}')
        proc.start()
        return True

    def cancel(self) -> None:
        """SIGTERM 을 보내고, 유예 뒤에도 살아 있으면 SIGKILL. 적재는 에피소드 단위 트랜잭션이라
        진행 중인 에피소드만 되돌아간다."""
        if not self.busy:
            return
        self.line.emit('[gui] cancel requested (SIGTERM)')
        self._proc.terminate()
        QTimer.singleShot(self.KILL_GRACE_MS, self._kill_if_alive)

    def _kill_if_alive(self) -> None:
        if self.busy:
            self.line.emit('[gui] still running — SIGKILL')
            self._proc.kill()

    def _emit_lines(self, text: str, buf_name: str) -> None:
        buf = getattr(self, buf_name) + text
        *lines, rest = buf.split('\n')
        setattr(self, buf_name, rest)
        for ln in lines:
            if buf_name == '_stdout_buf':
                self._stdout_all.append(ln)
            self.line.emit(ln)
            if self._progress is not None and self._progress.feed(ln):
                self.progress.emit(self._progress.done, self._progress.total or 0,
                                   self._progress.status())

    def _on_stdout(self) -> None:
        data = bytes(self._proc.readAllStandardOutput()).decode('utf-8', errors='replace')
        self._emit_lines(data, '_stdout_buf')

    def _on_stderr(self) -> None:
        data = bytes(self._proc.readAllStandardError()).decode('utf-8', errors='replace')
        self._emit_lines(data, '_stderr_buf')

    def _on_error(self, error) -> None:
        if error == QProcess.ProcessError.FailedToStart:
            self.line.emit(f'[gui] failed to start: {self._spec.argv[0]!r} not found '
                           f'(ROS 를 소싱한 셸에서 GUI 를 띄웠는가?)')

    def _on_finished(self, code: int, status) -> None:
        for buf_name in ('_stdout_buf', '_stderr_buf'):
            rest = getattr(self, buf_name)
            if rest:
                self._emit_lines(rest + '\n', buf_name)
        if status == QProcess.ExitStatus.CrashExit:
            code = -1
        summary = parse_summary_json('\n'.join(self._stdout_all))
        self.line.emit(f'[gui] finished: exit code {code}')
        self.finished.emit(code, summary)


def workspace_cwd() -> Optional[str]:
    """CLI 는 `RDFP_HOME` 기준 상대 경로를 쓰므로 거기서 띄운다."""
    home = os.environ.get('RDFP_HOME')
    return home if home and os.path.isdir(home) else None


__all__ = [
    'ImportProgress', 'JobRunner', 'JobSpec', 'KIND_EPISODE_DELETE', 'KIND_EXPORT', 'KIND_IMPORT',
    'KIND_REINDEX', 'RDFP_SCRIPT_MODULES', 'build_episode_delete_command', 'build_export_command',
    'build_import_command', 'build_reindex_command', 'episodes_spec', 'parse_summary_json',
    'rdfp_command', 'workspace_cwd',
]
