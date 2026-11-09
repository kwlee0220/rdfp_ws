"""Serializable application requests shared by GUI and the request-file CLI."""
from pathlib import Path
import base64
import os

from rdfp.dataset.config import load_dataset_config, DatasetConfig
from . import catalog
from .common import Control
from .database import Database


def handle(config_path, request, control=None):
    control = control or Control()
    action = request['action']
    args = request.get('args', {})
    cfg = load_dataset_config(config_path) if config_path else DatasetConfig(rosbag_dir='.')
    db = Database(cfg.db)
    if action == 'config':
        return {'rosbag_dir': cfg.rosbag_dir, 'recordings_dir': cfg.recordings_dir,
                'schema': cfg.db.schema_, 'dsn_env': cfg.db.dsn_env,
                'db_configured': bool(os.environ.get(cfg.db.dsn_env)),
                'planned_topics': cfg.effective_topics()}
    if action == 'discover':
        return catalog.discover_recordings(args['root'])
    if action == 'scan':
        topics = catalog.scan(args['source'], control)
        if os.environ.get(cfg.db.dsn_env):
            try:
                topics = db.mappings(topics)
            except Exception as exc:
                control.report({'message': f'DB 연결 없이 bag 목록을 표시합니다 ({type(exc).__name__}).'})
        return topics
    if action == 'bag_rows':
        return catalog.browse(control=control, **args)
    if action == 'bag_episodes':
        return catalog.episodes(args['source'], control)
    if action == 'tables':
        return db.tables()
    if action == 'table_schema':
        return db.describe(**args)
    if action == 'table_rows':
        return db.browse(**args)
    if action == 'episodes':
        return db.episodes(**args)
    if action == 'episode_detail':
        return db.episode_detail(**args)
    if action == 'initialize':
        return db.initialize()
    if action == 'register':
        return db.register(**args)
    if action == 'mapping_active':
        return db.set_mapping_active(**args)
    if action == 'drop_custom':
        return db.drop_custom(**args)
    if action == 'media':
        from .ingest import media_catalog
        return media_catalog(args.get('root') or cfg.recordings_dir)
    if action == 'preview_frame':
        import cv2
        path = Path(args['path']).expanduser().resolve()
        capture = cv2.VideoCapture(str(path))
        try:
            capture.set(cv2.CAP_PROP_POS_FRAMES, max(0, int(args.get('frame_index', 0))))
            ok, frame = capture.read()
            if not ok:
                raise ValueError('영상 프레임을 읽을 수 없습니다.')
            height, width = frame.shape[:2]
            if width > 960:
                frame = cv2.resize(frame, (960, round(height * 960 / width)))
            ok, png = cv2.imencode('.png', frame)
            if not ok:
                raise ValueError('미리보기 인코딩 실패')
            return {'png': base64.b64encode(png).decode()}
        finally:
            capture.release()
    if action == 'plan_import':
        from .ingest import plan_import
        return plan_import(db, recordings_dir=cfg.recordings_dir, control=control, **args)
    if action == 'import':
        from .ingest import execute_import
        return execute_import(db, args['plan'], control)
    if action == 'export':
        from .exporter import export_dataset
        return export_dataset(db, media_roots=cfg.mp4_roots(), control=control, **args)
    raise ValueError(f'알 수 없는 명령: {action}')
