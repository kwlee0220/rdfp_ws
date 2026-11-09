"""앱 컨텍스트 — 설정 로드 · DSN 가리기 · 시각 표기 (Qt 없음)."""

from __future__ import annotations

from rdfp_gui.context import AppContext, fmt_duration, fmt_time, mask_dsn


def test_mask_dsn_hides_only_the_password() -> None:
    assert mask_dsn('postgresql://rdfp:s3cr3t@127.0.0.1:5432/rdfp') == \
        'postgresql://rdfp:***@127.0.0.1:5432/rdfp'
    assert mask_dsn('postgresql://rdfp@localhost/rdfp') == 'postgresql://rdfp@localhost/rdfp'


def test_fmt_time_tells_sim_clock_from_wall_clock() -> None:
    assert fmt_time(None) == '-'
    assert fmt_time(54_566_666_666) == '54.567 s (sim)'
    assert fmt_time(1_789_889_301_979_199_888).startswith('2026-')
    assert fmt_duration(12_600_000_000) == '12.6 s'
    assert fmt_duration(150_000_000_000) == '2.5 min'


def test_context_loads_config_and_survives_a_missing_one(tmp_path, monkeypatch) -> None:
    monkeypatch.delenv('RDFP_DB_DSN', raising=False)
    cfg = tmp_path / 'dataset_config.yaml'
    cfg.write_text('rosbag_dir: /tmp/bags\noutput_mp4_dir: /tmp/videos\n')
    ctx = AppContext.load(str(cfg))
    assert ctx.loaded and ctx.error is None
    assert str(ctx.rosbag_dir) == '/tmp/bags'
    assert ctx.db_configured() is False
    assert 'RDFP_DB_DSN' in ctx.masked_dsn()
    assert [r[0] for r in ctx.path_rows()] == ['rosbag_dir', 'output_mp4_dir', 'recordings_dir']

    ctx.switch(str(tmp_path / 'nope.yaml'))
    assert not ctx.loaded
    assert 'not found' in ctx.error
    assert ctx.mp4_roots() == []


def test_masked_dsn_never_shows_the_password(tmp_path, monkeypatch) -> None:
    monkeypatch.setenv('RDFP_DB_DSN', 'postgresql://u:topsecret@h/db')
    cfg = tmp_path / 'c.yaml'
    cfg.write_text('rosbag_dir: /tmp/bags\n')
    ctx = AppContext.load(str(cfg))
    assert ctx.db_configured()
    assert 'topsecret' not in ctx.masked_dsn()
