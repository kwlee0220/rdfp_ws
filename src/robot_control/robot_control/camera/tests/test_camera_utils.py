#!/usr/bin/env python3

"""`camera_utils` 의 순수 함수 테스트 — ROS 없이 돈다.

**마스킹이 이 파일의 핵심이다.** RTSP 자격증명이 로그에 그대로 찍히면 rosbag·이슈
첨부·화면 공유로 계속 새어 나가고, 되돌릴 방법이 없다.
"""

from __future__ import annotations

import pytest

from robot_control.camera.camera_utils import mask_camera_id_for_log, parse_camera_id


# ---------- parse_camera_id ----------

@pytest.mark.parametrize('value,expected', [(0, 0), (4, 4), ('0', 0), ('4', 4), ('  7  ', 7)])
def test_numeric_becomes_int(value, expected):
    """숫자 문자열은 int 로 바뀐다 — OpenCV 가 장치 인덱스를 int 로만 받는다."""
    assert parse_camera_id(value, 'camera_id') == expected


@pytest.mark.parametrize('value', ['/data/clip.mp4', 'rtsp://host/stream', '  /a/b.mp4  '])
def test_path_and_uri_stay_str(value):
    assert parse_camera_id(value, 'camera_id') == value.strip()


def test_negative_index_is_rejected():
    """음수 인덱스는 열리지 않는다 — 기동 단계에서 막는다."""
    with pytest.raises(ValueError, match='cannot be negative'):
        parse_camera_id(-1, 'camera_id')


@pytest.mark.parametrize('value', ['', '   '])
def test_empty_string_is_rejected(value):
    with pytest.raises(ValueError, match='cannot be empty'):
        parse_camera_id(value, 'camera_id')


@pytest.mark.parametrize('value', [None, 1.5, [], {}])
def test_wrong_type_is_rejected(value):
    with pytest.raises(ValueError, match='must be int or str'):
        parse_camera_id(value, 'camera_id')


# ---------- mask_camera_id_for_log ----------

def test_password_is_masked():
    """**비밀번호가 남으면 안 된다.** 로그는 rosbag·이슈로 계속 퍼진다."""
    masked = mask_camera_id_for_log('rtsp://admin:s3cret@cam.local:554/stream')
    assert 's3cret' not in masked
    assert 'admin' in masked, '사용자 이름은 남겨야 어느 계정인지 알 수 있다'
    assert 'cam.local:554' in masked and '/stream' in masked


def test_uri_without_credentials_is_unchanged():
    uri = 'rtsp://cam.local:554/stream'
    assert mask_camera_id_for_log(uri) == uri


@pytest.mark.parametrize('value', [0, 4, '/data/clip.mp4'])
def test_non_uri_passes_through(value):
    """장치 인덱스와 파일 경로는 가릴 것이 없다."""
    assert mask_camera_id_for_log(value) == value


def test_query_and_fragment_survive():
    """가리느라 URI 의 다른 부분을 잃으면 로그가 쓸모없어진다."""
    masked = mask_camera_id_for_log('rtsp://u:p@h/s?channel=2#f')
    assert 'channel=2' in masked and '#f' in masked and 'p@' not in masked
