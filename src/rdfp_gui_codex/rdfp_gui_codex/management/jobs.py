"""Durable local job history and cooperative worker-process cancellation."""
from __future__ import annotations

import json
import multiprocessing
import os
from pathlib import Path
import sqlite3
import time
import uuid

from .common import Cancelled, Control, dumps


class Journal:
    def __init__(self, path):
        self.path = str(path)
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        with self.connect() as conn:
            conn.execute('CREATE TABLE IF NOT EXISTS jobs (id TEXT PRIMARY KEY, action TEXT, state TEXT, '
                         'started REAL, finished REAL, message TEXT, request TEXT)')

    def connect(self):
        return sqlite3.connect(self.path, timeout=10)

    def start(self, request):
        job_id = uuid.uuid4().hex
        with self.connect() as conn:
            conn.execute('INSERT INTO jobs VALUES(?,?,?,?,?,?,?)',
                         (job_id, request['action'], 'RUNNING', time.time(), None, '', dumps(request)))
        return job_id

    def finish(self, job_id, state, message):
        with self.connect() as conn:
            conn.execute('UPDATE jobs SET state=?,finished=?,message=? WHERE id=?',
                         (state, time.time(), message, job_id))

    def list(self):
        with self.connect() as conn:
            conn.row_factory = sqlite3.Row
            return [dict(r) for r in conn.execute('SELECT id,action,state,started,finished,message FROM jobs '
                                                 'ORDER BY started DESC LIMIT 200')]


def safe_error(exc):
    text = str(exc)
    # Never persist a DSN if a driver or another dependency included it in an error.
    for key, value in os.environ.items():
        if ('DSN' in key or 'PASSWORD' in key or 'TOKEN' in key) and value:
            text = text.replace(value, '[redacted]')
    return f'{type(exc).__name__}: {text}'


def run_worker(config_path, request, cancel, queue, journal_path, job_id):
    from .service import handle
    journal = Journal(journal_path)
    control = Control(cancel.is_set, lambda event: queue.put(('progress', event)))
    try:
        result = handle(config_path, request, control)
        # Do not re-check cancellation after an action committed/published its result.
        journal.finish(job_id, 'SUCCEEDED', '완료')
        queue.put(('result', result))
    except Cancelled as exc:
        journal.finish(job_id, 'CANCELLED', str(exc))
        queue.put(('cancelled', str(exc)))
    except Exception as exc:
        message = safe_error(exc)
        journal.finish(job_id, 'FAILED', message)
        queue.put(('error', message))


class Worker:
    def __init__(self, config_path, request, journal):
        self.journal = journal
        self.job_id = journal.start(request)
        ctx = multiprocessing.get_context('spawn')
        self.queue, self.cancel_event = ctx.Queue(), ctx.Event()
        self.process = ctx.Process(target=run_worker, args=(config_path, request,
            self.cancel_event, self.queue, journal.path, self.job_id))
        self.process.start()

    def cancel(self):
        self.cancel_event.set()

    def close(self):
        self.process.join(timeout=0)
        self.queue.close()
        self.queue.join_thread()
