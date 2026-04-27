#!/usr/bin/env python3

from __future__ import annotations

from typing import Any, Callable

import subprocess

import pytest

from rdfp.recorder import encoder_probe
from rdfp.recorder.encoder_probe import (
    DEFAULT_HW_CODEC_PRIORITY,
    ENCODER_MODE_AUTO,
    ENCODER_MODE_CPU,
    ENCODER_MODE_GPU,
    build_probe_command,
    parse_build_encoders,
    probe_available_hw_encoders,
    probe_runtime_encoder,
    select_encoder,
)
from rdfp.recorder.exceptions import EncoderUnavailableError
from rdfp.recorder.ffmpeg_command import (
    CODEC_H264_NVENC,
    CODEC_H264_QSV,
    CODEC_H264_VAAPI,
    CODEC_LIBX264,
)


# ---------- parse_build_encoders --------------------------------------------


_SAMPLE_ENCODERS_OUTPUT = """Encoders:
 V..... = Video
 A..... = Audio
 S..... = Subtitle
 .F.... = Frame-level multithreading
 ..S... = Slice-level multithreading
 ...X.. = Codec is experimental
 ....B. = Supports draw_horiz_band
 .....D = Supports direct rendering method 1
 ------
 V....D libx264              libx264 H.264 / AVC / MPEG-4 AVC / MPEG-4 part 10 (codec h264)
 V....D h264_nvenc           NVIDIA NVENC H.264 encoder (codec h264)
 V....D h264_qsv             Intel Quick Sync Video H.264 encoder (codec h264)
 V....D h264_vaapi           H.264/AVC (VAAPI) (codec h264)
 V....D hevc_nvenc           NVIDIA NVENC hevc encoder (codec hevc)
 A....D aac                  AAC (Advanced Audio Coding)
 A....D libmp3lame           libmp3lame MP3 (MPEG audio layer 3)
"""


def test_parse_build_encoders_extracts_all_video_codecs() -> None:
    codecs = parse_build_encoders(_SAMPLE_ENCODERS_OUTPUT)
    assert CODEC_LIBX264 in codecs
    assert CODEC_H264_NVENC in codecs
    assert CODEC_H264_QSV in codecs
    assert CODEC_H264_VAAPI in codecs
    assert "hevc_nvenc" in codecs


def test_parse_build_encoders_excludes_audio_codecs() -> None:
    codecs = parse_build_encoders(_SAMPLE_ENCODERS_OUTPUT)
    assert "aac" not in codecs
    assert "libmp3lame" not in codecs


def test_parse_build_encoders_excludes_legend_lines() -> None:
    # "V..... = Video" 같은 범례는 코덱으로 오해하지 않아야 한다
    codecs = parse_build_encoders(_SAMPLE_ENCODERS_OUTPUT)
    assert "=" not in codecs
    assert "Video" not in codecs


def test_parse_build_encoders_empty_output() -> None:
    assert parse_build_encoders("") == set()


# ---------- build_probe_command ----------------------------------------------


def test_build_probe_command_libx264_uses_lavfi_source() -> None:
    cmd = build_probe_command(
        ffmpeg_binary="ffmpeg",
        codec=CODEC_LIBX264,
        vaapi_device="/dev/dri/renderD128",
    )
    assert cmd[0] == "ffmpeg"
    assert "lavfi" in cmd
    assert cmd[cmd.index("-c:v") + 1] == CODEC_LIBX264
    assert cmd[-3:] == ["-f", "null", "-"]
    # VAAPI 전용 옵션은 없어야 한다
    assert "-vaapi_device" not in cmd
    assert "-init_hw_device" not in cmd


def test_build_probe_command_nvenc_has_no_hw_setup() -> None:
    cmd = build_probe_command(
        ffmpeg_binary="ffmpeg",
        codec=CODEC_H264_NVENC,
        vaapi_device="/dev/dri/renderD128",
    )
    assert cmd[cmd.index("-c:v") + 1] == CODEC_H264_NVENC
    assert "-vaapi_device" not in cmd


def test_build_probe_command_vaapi_includes_device_and_filter() -> None:
    cmd = build_probe_command(
        ffmpeg_binary="ffmpeg",
        codec=CODEC_H264_VAAPI,
        vaapi_device="/dev/dri/renderD129",
    )
    assert cmd[cmd.index("-vaapi_device") + 1] == "/dev/dri/renderD129"
    assert cmd[cmd.index("-vf") + 1] == "format=nv12,hwupload"
    assert cmd[cmd.index("-c:v") + 1] == CODEC_H264_VAAPI


def test_build_probe_command_qsv_init_hw_device() -> None:
    cmd = build_probe_command(
        ffmpeg_binary="ffmpeg",
        codec=CODEC_H264_QSV,
        vaapi_device="/dev/dri/renderD128",
    )
    assert cmd[cmd.index("-init_hw_device") + 1] == "qsv=hw"
    assert cmd[cmd.index("-filter_hw_device") + 1] == "hw"
    assert cmd[cmd.index("-c:v") + 1] == CODEC_H264_QSV


# ---------- subprocess.run monkeypatch 헬퍼 ---------------------------------


def _make_run_stub(
    *,
    encoders_stdout: str = "",
    encoders_rc: int = 0,
    runtime_success_codecs: frozenset[str] = frozenset(),
    on_call: Callable[[list[str]], None] | None = None,
) -> Callable[..., subprocess.CompletedProcess]:
    """subprocess.run 을 대체할 stub 생성.

    - `-encoders` 호출은 `encoders_stdout` 을 반환
    - runtime probe 호출은 cmd 내 코덱 이름이 `runtime_success_codecs` 에 포함되면
      rc=0, 아니면 rc=1
    """

    def _run(cmd: list[str], *_args: Any, **_kwargs: Any) -> subprocess.CompletedProcess:
        if on_call is not None:
            on_call(cmd)
        if "-encoders" in cmd:
            return subprocess.CompletedProcess(
                args=cmd,
                returncode=encoders_rc,
                stdout=encoders_stdout,
                stderr="",
            )
        # runtime probe: -c:v 다음 인자가 codec 이름
        try:
            codec = cmd[cmd.index("-c:v") + 1]
        except (ValueError, IndexError):
            codec = ""
        rc = 0 if codec in runtime_success_codecs else 1
        return subprocess.CompletedProcess(args=cmd, returncode=rc, stdout="", stderr="")

    return _run


# ---------- list_build_encoders ----------------------------------------------


def test_list_build_encoders_returns_empty_when_ffmpeg_missing(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def _raise(*_args: Any, **_kwargs: Any) -> Any:
        raise FileNotFoundError("ffmpeg not found")

    monkeypatch.setattr(subprocess, "run", _raise)
    assert encoder_probe.list_build_encoders("ffmpeg-nonexistent") == set()


def test_list_build_encoders_returns_empty_on_nonzero_rc(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    stub = _make_run_stub(encoders_stdout="", encoders_rc=1)
    monkeypatch.setattr(subprocess, "run", stub)
    assert encoder_probe.list_build_encoders() == set()


def test_list_build_encoders_parses_output(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    stub = _make_run_stub(encoders_stdout=_SAMPLE_ENCODERS_OUTPUT)
    monkeypatch.setattr(subprocess, "run", stub)
    codecs = encoder_probe.list_build_encoders()
    assert CODEC_LIBX264 in codecs
    assert CODEC_H264_NVENC in codecs


# ---------- probe_runtime_encoder --------------------------------------------


def test_probe_runtime_encoder_success(monkeypatch: pytest.MonkeyPatch) -> None:
    stub = _make_run_stub(runtime_success_codecs=frozenset({CODEC_LIBX264}))
    monkeypatch.setattr(subprocess, "run", stub)
    assert probe_runtime_encoder(ffmpeg_binary="ffmpeg", codec=CODEC_LIBX264)


def test_probe_runtime_encoder_failure(monkeypatch: pytest.MonkeyPatch) -> None:
    stub = _make_run_stub(runtime_success_codecs=frozenset())
    monkeypatch.setattr(subprocess, "run", stub)
    assert not probe_runtime_encoder(ffmpeg_binary="ffmpeg", codec=CODEC_H264_NVENC)


def test_probe_runtime_encoder_handles_ffmpeg_not_found(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def _raise(*_args: Any, **_kwargs: Any) -> Any:
        raise FileNotFoundError("ffmpeg not found")

    monkeypatch.setattr(subprocess, "run", _raise)
    assert not probe_runtime_encoder(ffmpeg_binary="missing", codec=CODEC_LIBX264)


def test_probe_runtime_encoder_handles_timeout(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def _raise(*_args: Any, **_kwargs: Any) -> Any:
        raise subprocess.TimeoutExpired(cmd="ffmpeg", timeout=1)

    monkeypatch.setattr(subprocess, "run", _raise)
    assert not probe_runtime_encoder(ffmpeg_binary="ffmpeg", codec=CODEC_LIBX264)


# ---------- probe_available_hw_encoders --------------------------------------


def test_probe_available_hw_encoders_returns_priority_order(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # 빌드에는 nvenc 와 vaapi 포함, runtime 도 둘 다 성공
    stub = _make_run_stub(
        encoders_stdout=(
            " V....D h264_nvenc   NVENC\n"
            " V....D h264_vaapi   VAAPI\n"
        ),
        runtime_success_codecs=frozenset({CODEC_H264_NVENC, CODEC_H264_VAAPI}),
    )
    monkeypatch.setattr(subprocess, "run", stub)
    available = probe_available_hw_encoders()
    # nvenc 가 우선순위에서 앞서므로 첫 번째여야 한다
    assert available == [CODEC_H264_NVENC, CODEC_H264_VAAPI]


def test_probe_available_hw_encoders_skips_not_in_build(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    stub = _make_run_stub(
        encoders_stdout=" V....D h264_qsv   QSV\n",
        runtime_success_codecs=frozenset({CODEC_H264_QSV}),
    )
    monkeypatch.setattr(subprocess, "run", stub)
    available = probe_available_hw_encoders()
    assert available == [CODEC_H264_QSV]


def test_probe_available_hw_encoders_empty_when_all_fail(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    stub = _make_run_stub(
        encoders_stdout=(
            " V....D h264_nvenc   NVENC\n"
            " V....D h264_qsv     QSV\n"
            " V....D h264_vaapi   VAAPI\n"
        ),
        runtime_success_codecs=frozenset(),
    )
    monkeypatch.setattr(subprocess, "run", stub)
    assert probe_available_hw_encoders() == []


# ---------- select_encoder ---------------------------------------------------


def test_select_encoder_cpu_mode_skips_subprocess(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def _fail(*_args: Any, **_kwargs: Any) -> Any:
        raise AssertionError("subprocess.run must not be called in cpu mode")

    monkeypatch.setattr(subprocess, "run", _fail)
    assert select_encoder(encoder_mode=ENCODER_MODE_CPU) == CODEC_LIBX264


def test_select_encoder_gpu_mode_raises_when_no_hw(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    stub = _make_run_stub(
        encoders_stdout=" V....D h264_nvenc   NVENC\n",
        runtime_success_codecs=frozenset(),  # runtime 실패
    )
    monkeypatch.setattr(subprocess, "run", stub)
    with pytest.raises(EncoderUnavailableError, match="no working hardware encoder"):
        select_encoder(encoder_mode=ENCODER_MODE_GPU)


def test_select_encoder_auto_fallback_to_libx264(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    stub = _make_run_stub(
        encoders_stdout=_SAMPLE_ENCODERS_OUTPUT,
        runtime_success_codecs=frozenset(),  # 모든 HW runtime 실패
    )
    monkeypatch.setattr(subprocess, "run", stub)
    assert select_encoder(encoder_mode=ENCODER_MODE_AUTO) == CODEC_LIBX264


def test_select_encoder_auto_picks_first_available_hw(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    stub = _make_run_stub(
        encoders_stdout=_SAMPLE_ENCODERS_OUTPUT,
        runtime_success_codecs=frozenset({CODEC_H264_NVENC, CODEC_H264_VAAPI}),
    )
    monkeypatch.setattr(subprocess, "run", stub)
    # 우선순위 nvenc > qsv > vaapi. nvenc 가 선택되어야 한다
    assert select_encoder(encoder_mode=ENCODER_MODE_AUTO) == CODEC_H264_NVENC


def test_select_encoder_gpu_skips_failing_nvenc_and_uses_qsv(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    stub = _make_run_stub(
        encoders_stdout=_SAMPLE_ENCODERS_OUTPUT,
        runtime_success_codecs=frozenset({CODEC_H264_QSV, CODEC_H264_VAAPI}),
    )
    monkeypatch.setattr(subprocess, "run", stub)
    assert select_encoder(encoder_mode=ENCODER_MODE_GPU) == CODEC_H264_QSV


def test_select_encoder_preferred_hw_codec_probes_only_that_codec(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    probe_calls: list[list[str]] = []
    stub = _make_run_stub(
        encoders_stdout=_SAMPLE_ENCODERS_OUTPUT,
        runtime_success_codecs=frozenset({CODEC_H264_VAAPI}),
        on_call=lambda cmd: probe_calls.append(cmd),
    )
    monkeypatch.setattr(subprocess, "run", stub)
    result = select_encoder(
        encoder_mode=ENCODER_MODE_GPU,
        preferred_hw_codec=CODEC_H264_VAAPI,
    )
    assert result == CODEC_H264_VAAPI
    # runtime probe 는 vaapi 에 대해서만 수행되어야 한다
    runtime_calls = [c for c in probe_calls if "-c:v" in c]
    assert len(runtime_calls) == 1
    assert runtime_calls[0][runtime_calls[0].index("-c:v") + 1] == CODEC_H264_VAAPI


def test_select_encoder_invalid_mode_raises() -> None:
    with pytest.raises(ValueError, match="invalid encoder_mode"):
        select_encoder(encoder_mode="hybrid")


def test_select_encoder_invalid_preferred_hw_codec_raises() -> None:
    with pytest.raises(ValueError, match="invalid preferred_hw_codec"):
        select_encoder(encoder_mode=ENCODER_MODE_AUTO, preferred_hw_codec=CODEC_LIBX264)


def test_default_hw_codec_priority_matches_plan() -> None:
    # plan.md §6 에 명시된 우선순위: nvenc → qsv → vaapi
    assert DEFAULT_HW_CODEC_PRIORITY == (
        CODEC_H264_NVENC,
        CODEC_H264_QSV,
        CODEC_H264_VAAPI,
    )
