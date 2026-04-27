"""SELECT 결과 한 행을 ROS 2 메시지로 복원하는 reader 베이스."""

from __future__ import annotations

from typing import Any


class ReaderBase:
    """DB row → ROS 2 메시지 복원기.

    서브클래스는 다음을 정의한다.
      * 클래스 속성 `select_cols`: SELECT 할 컬럼 순서
        (`build()` 가 기대하는 row 튜플 순서와 일치).
      * classmethod `build(row)`: row 튜플을 메시지 인스턴스로 복원.

    reader 는 상태를 가지지 않으므로 classmethod 만으로 충분하며 인스턴스화
    하지 않는다. 대상 테이블 이름은 reader 가 알지 못하고 `registry` 의
    `TypeBinding.table` 이 단일 공급원이다. ROS 2 메시지 패키지 import 는
    `build()` 안에서 lazy 로 수행해 rclpy 부재 환경에서도 모듈 import 만큼은
    가능하게 한다.
    """

    select_cols: tuple[str, ...] = ()

    @classmethod
    def build(cls, row: tuple[Any, ...]) -> Any:
        raise NotImplementedError


__all__ = ['ReaderBase']
