"""Shared contracts for the desktop application and its workers."""
from __future__ import annotations

import base64
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Callable


class Cancelled(Exception):
    """The current uncommitted unit must be rolled back."""


class Control:
    def __init__(self, cancelled: Callable = lambda: False,
                 report: Callable = lambda event: None):
        self.cancelled = cancelled
        self.report = report

    def check(self):
        if self.cancelled():
            raise Cancelled('사용자가 작업을 취소했습니다.')

    def progress(self, message, **fields):
        self.check()
        self.report({'message': message, **fields})


def json_value(value: Any, *, limit: int | None = None) -> Any:
    """Loss-aware JSON representation; preview limits never apply to stored data."""
    if value is None or isinstance(value, (str, bool)):
        return value
    if isinstance(value, int):
        # Python's JSON decoder and PostgreSQL JSONB both retain integer precision.
        # In particular, nanosecond timestamps in saved job plans must stay numbers.
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else {'$float': str(value)}
    if isinstance(value, (bytes, bytearray, memoryview)):
        data = bytes(value)
        return {'$bytes': base64.b64encode(data if limit is None else data[:limit]).decode(),
                'length': len(data)}
    if isinstance(value, dict):
        return {str(k): json_value(v, limit=limit) for k, v in value.items()}
    if isinstance(value, (list, tuple)) or hasattr(value, 'tolist'):
        values = value.tolist() if hasattr(value, 'tolist') else value
        result = [json_value(v, limit=limit) for v in
                  (values if limit is None else values[:limit])]
        if limit is not None and len(values) > limit:
            result.append({'$omitted': len(values) - limit})
        return result
    if hasattr(value, 'get_fields_and_field_types'):
        return json_value({k: getattr(value, k) for k in value.get_fields_and_field_types()},
                          limit=limit)
    if hasattr(type(value), '__slots__'):
        return json_value({k: getattr(value, k) for k in type(value).__slots__
                           if not k.startswith('_')}, limit=limit)
    if hasattr(value, '__dict__'):
        return json_value(vars(value), limit=limit)
    return str(value)


def dumps(value):
    return json.dumps(json_value(value), ensure_ascii=False, indent=2, allow_nan=False)


def fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, ensure_ascii=False,
                                    separators=(',', ':')).encode()).hexdigest()


def hash_file(path: Path, control: Control):
    digest = hashlib.sha256()
    before = path.stat()
    with path.open('rb') as stream:
        while block := stream.read(1024 * 1024):
            control.check()
            digest.update(block)
    after = path.stat()
    if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
        raise ValueError(f'읽는 중 원본 파일이 변경되었습니다: {path}')
    return digest.hexdigest()


def parse_ids(spec: str) -> list[int]:
    """Inclusive DB ID ranges; reject empty, inverted or unbounded selections."""
    result = set()
    for part in spec.split(','):
        bits = part.strip().split('-')
        if len(bits) not in (1, 2) or any(not b.strip().isdigit() for b in bits):
            raise ValueError('에피소드는 1-5,8처럼 지정해 주세요.')
        start, end = int(bits[0]), int(bits[-1])
        if start < 1 or end < start or end - start > 100000:
            raise ValueError('에피소드 범위를 확인해 주세요 (최대 100001개).')
        result.update(range(start, end + 1))
    return sorted(result)
