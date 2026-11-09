import importlib.util
import os
from queue import Empty
import time

import pytest

from rdfp_gui_codex.management.jobs import Journal, Worker, safe_error


def test_spawned_worker_and_durable_history(bag, tmp_path):
    journal=Journal(tmp_path/'jobs.sqlite3')
    worker=Worker('', {'action':'scan','args':{'source':str(bag)}}, journal)
    deadline=time.monotonic()+15
    result=None
    try:
        while time.monotonic()<deadline:
            try:
                kind,value=worker.queue.get(timeout=.1)
                if kind=='error':
                    pytest.fail(value)
                if kind=='result':
                    result=value
                    break
            except Empty:
                if not worker.process.is_alive():
                    break
        worker.process.join(timeout=5)
        assert result and any(r['topic']=='/joint_states' for r in result)
        assert journal.list()[0]['state']=='SUCCEEDED'
    finally:
        if worker.process.is_alive():
            worker.process.terminate()
            worker.process.join(timeout=5)
        worker.close()


def test_error_redacts_secrets(monkeypatch):
    monkeypatch.setenv('SOME_DB_DSN', 'postgresql://private:secret@host/db')
    assert 'secret' not in safe_error(ValueError('postgresql://private:secret@host/db failed'))


@pytest.mark.skipif(importlib.util.find_spec('PySide6') is None, reason='optional Qt environment')
def test_offscreen_views_selection_and_export_dialog(tmp_path):
    os.environ['QT_QPA_PLATFORM']='offscreen'
    from PySide6.QtWidgets import QApplication
    from rdfp_gui_codex.gui.window import ManagerWindow, ExportDialog
    app=QApplication.instance() or QApplication([])
    window=ManagerWindow(state_dir=tmp_path)
    window.topics.replace([{'topic':'/a','table_name':'joint_states','type':'sensor_msgs/msg/JointState'}])
    window.tables.replace([{'table_name':'joint_states','estimated_rows':2}])
    window.topics.table.selectRow(0)
    assert window.tables.model.highlight=={0}
    window.show_rows([{'id':1,'value':{'x':.4}}])
    assert window.rows.model.rows[0]['id']==1
    dialog=ExportDialog([1,2],window)
    dialog.output.setText(str(tmp_path/'out'))
    assert dialog.request()['episode_ids']==[1,2]
    window.show();app.processEvents()
    window.close();dialog.close()
