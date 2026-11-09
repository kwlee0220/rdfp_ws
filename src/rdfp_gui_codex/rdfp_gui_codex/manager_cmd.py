"""Dataset Manager entry point; --request also works without Qt."""
import argparse
import json
from pathlib import Path
import sys


def main(argv=None):
    parser = argparse.ArgumentParser(description='RDFP Dataset Manager')
    parser.add_argument('--config', help='dataset_config.yaml 경로 (DSN은 환경변수로 지정)')
    parser.add_argument('--state-dir', help='로컬 작업 이력 디렉터리')
    parser.add_argument('--request', help='GUI 없이 실행할 JSON 요청 파일: {action, args}')
    args = parser.parse_args(argv)
    if args.request:
        from .management.service import handle
        from .management.common import Control, dumps
        from .management.jobs import safe_error
        try:
            request = json.loads(Path(args.request).read_text())
            result = handle(args.config, request, Control(report=lambda e: print(e['message'], file=sys.stderr)))
            print(dumps(result))
            return 0
        except Exception as exc:
            print(safe_error(exc), file=sys.stderr)
            return 1
    try:
        from PySide6.QtWidgets import QApplication
        from .gui.window import ManagerWindow
    except ImportError as exc:
        print(f'GUI 의존성이 없습니다: {exc}\n'
              'rdfp_gui_codex의 requirements.txt와 Dataset Manager 운영 가이드를 확인하세요.', file=sys.stderr)
        return 2
    app = QApplication([sys.argv[0]])
    app.setApplicationName('RDFP Dataset Manager')
    window = ManagerWindow(args.config or '', args.state_dir)
    window.show()
    return app.exec()


if __name__ == '__main__':
    raise SystemExit(main())
