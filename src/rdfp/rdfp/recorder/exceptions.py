from __future__ import annotations


class RecorderError(RuntimeError):
    """FFMpegMp4Recorder 관련 오류의 기본 클래스."""


class RecorderStateError(RecorderError):
    """현재 상태에서 허용되지 않는 연산이 요청되었을 때 발생.

    SHUTDOWN 을 포함한 모든 상태 오류에 사용. 예외 메시지에 현재 상태명을
    포함하여 호출자가 원인을 구분할 수 있게 한다.
    """


class EncoderUnavailableError(RecorderError):
    """요구된 인코더(특히 GPU 인코더)를 사용할 수 없을 때 발생."""
