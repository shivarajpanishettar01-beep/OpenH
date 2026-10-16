"""Annotation tool for expert / trainee / model-assisted labels (Open-H V2 proposal, section 7).

  ros2 run usrig_dashboard annotator <session_dir> --annotator <id> --role expert|trainee|model_assisted [--blind]

Offline: reads the episode bags, never touches the ROS graph, so it can run anywhere (also while a
session is being recorded on another machine).

Per episode: B-mode and wrist colour at the playhead, probe-axis force trace, and timeline tracks for
phases, gestures, status (g_t), narration and rig events. The annotator
  * adds segments (layer + value, Mark start / Mark end at the playhead),
  * sets QA status on any entry (entries of others are never deleted, only rejected; accepting an automated
    entry counts as this annotator's rating in label_agreement),
  * sets episode labels: failure class, motion quality 1-5, intent, outcome,
  * adds narration at the playhead.
Save validates and writes only <ep>/annotation.<annotator>.json (own entries, QA decisions, episode labels,
history): raters working at the same time never overwrite each other, and annotation.json (automated,
auto_annotate) is never rewritten. --blind hides every other rater's entries, QA decisions and episode
labels (automated ones stay visible): use it for the second rating of double-rated episodes.
Keys: Space play/pause, Left/Right one frame, [ ] start/end mark, Ctrl+S save.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

from usrig_common import annotation as A
from usrig_common import config

TRACKS = [('phases', 'phase'), ('gestures', 'gesture'), ('status_intervals', 'status')]
COLORS = {
    'approach_and_contact': '#90a4ae', 'target_localisation': '#4fc3f7', 'view_acquisition': '#81c784',
    'sweep_scan': '#ffb74d', 'reacquisition_recovery': '#ba68c8',
    'slide': '#4db6ac', 'tilt': '#7986cb', 'rock': '#f06292', 'rotate': '#ffd54f', 'compress': '#a1887f',
    'decompress': '#bcaaa4', 'nominal': '#c8e6c9', 'failure': '#e57373', 'recovery': '#ffb74d',
}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session')
    ap.add_argument('--annotator', required=True)
    ap.add_argument('--role', required=True, choices=['expert', 'trainee', 'model_assisted'])
    ap.add_argument('--blind', action='store_true', help="hide other raters' labels (double rating for kappa)")
    ap.add_argument('--config-dir')
    a, _ = ap.parse_known_args()
    A.annotator_file('.', a.annotator)                 # refuses an id that is not a safe file name
    only = a.annotator if a.blind else None

    from PySide6 import QtCore, QtGui, QtWidgets
    import pyqtgraph as pg

    from .annot_data import load_episode

    scfg = config.load_config('session', a.config_dir)
    color_topic = scfg['wrist_camera']['topics']['color']
    sdir = Path(a.session).expanduser()
    eps = sorted(p.parent for p in sdir.glob('ep_*/episode.json'))
    if not eps:
        sys.exit(f'no episodes in {sdir}')

    pg.setConfigOptions(imageAxisOrder='row-major', antialias=False)
    app = QtWidgets.QApplication(sys.argv)
    win = QtWidgets.QMainWindow()
    win.setWindowTitle(f'usrig annotator: {sdir.name} ({a.annotator}, {a.role}{", BLIND" if a.blind else ""})')
    state = {'data': None, 'ed': None, 't': 0, 'mark': None, 'playing': False}

    split = QtWidgets.QSplitter()
    win.setCentralWidget(split)

    # ---------- episode list
    ep_list = QtWidgets.QListWidget()
    for d in eps:
        e = json.loads((d / 'episode.json').read_text())
        ann = A.load_merged(d, only=only, episode=e)
        n_human = sum(1 for layer, _ in TRACKS for s in ann.get(layer, []) if s.get('provenance') != 'automated')
        it = QtWidgets.QListWidgetItem(f'{d.name}  {e.get("type") or ""}  {e.get("outcome") or ""}'
                                       f'{"  [sim]" if e.get("sim") else ""}  human:{n_human}')
        it.setData(QtCore.Qt.UserRole, str(d))
        ep_list.addItem(it)
    split.addWidget(ep_list)

    # ---------- centre: images, timeline
    centre = QtWidgets.QWidget()
    cl = QtWidgets.QVBoxLayout(centre)
    imgs = QtWidgets.QHBoxLayout()
    us_view, cam_view = pg.ImageView(), pg.ImageView()
    for v in (us_view, cam_view):
        v.ui.histogram.hide()
        v.ui.roiBtn.hide()
        v.ui.menuBtn.hide()
        imgs.addWidget(v)
    cl.addLayout(imgs, 3)
    info = QtWidgets.QLabel()
    cl.addWidget(info)
    timeline = pg.PlotWidget()
    timeline.setLabel('bottom', 'time from episode start', units='s')
    timeline.getAxis('left').setTicks([[(i, name) for i, (name, _) in enumerate(TRACKS + [('narration', ''), ('events', '')])]])
    timeline.setYRange(-0.6, 4.6)
    playhead = pg.InfiniteLine(pos=0, angle=90, movable=True, pen=pg.mkPen('r', width=2))
    timeline.addItem(playhead)
    cl.addWidget(timeline, 2)
    force_plot = pg.PlotWidget()
    force_plot.setLabel('left', 'F probe axis', units='N')
    force_plot.setXLink(timeline)
    force_curve = force_plot.plot(pen='y')
    force_head = pg.InfiniteLine(pos=0, angle=90, pen=pg.mkPen('r'))
    force_plot.addItem(force_head)
    cl.addWidget(force_plot, 1)
    slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
    cl.addWidget(slider)
    split.addWidget(centre)

    # ---------- right: editor
    right = QtWidgets.QWidget()
    rl = QtWidgets.QVBoxLayout(right)
    form = QtWidgets.QFormLayout()
    layer_box = QtWidgets.QComboBox()
    layer_box.addItems([name for name, _ in TRACKS])
    value_box = QtWidgets.QComboBox()
    form.addRow('layer', layer_box)
    form.addRow('value', value_box)
    rl.addLayout(form)
    mark_row = QtWidgets.QHBoxLayout()
    btn_start = QtWidgets.QPushButton('Mark start [')
    btn_end = QtWidgets.QPushButton('Mark end ]')
    mark_row.addWidget(btn_start)
    mark_row.addWidget(btn_end)
    rl.addLayout(mark_row)
    seg_table = QtWidgets.QTableWidget(0, 5)
    seg_table.setHorizontalHeaderLabels(['value', 'start s', 'end s', 'provenance', 'QA'])
    seg_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
    rl.addWidget(seg_table, 2)
    qa_row = QtWidgets.QHBoxLayout()
    for qa in ('accepted', 'rejected', 'flagged'):
        b = QtWidgets.QPushButton(qa)
        b.clicked.connect(lambda _=False, q=qa: set_qa(q))
        qa_row.addWidget(b)
    btn_del = QtWidgets.QPushButton('delete')
    qa_row.addWidget(btn_del)
    rl.addLayout(qa_row)

    ep_form = QtWidgets.QFormLayout()
    fc_box = QtWidgets.QComboBox()
    fc_box.addItems([''] + A.FAILURE_CLASSES)
    mq_box = QtWidgets.QComboBox()
    mq_box.addItems([''] + [f'{q} ({A.QUALITY_TIERS[q]})' for q in (5, 4, 3, 2, 1)])
    intent = QtWidgets.QLineEdit()
    outcome_box = QtWidgets.QComboBox()
    outcome_box.addItems(['', 'success', 'recovered', 'fail'])
    ep_form.addRow('failure class', fc_box)
    ep_form.addRow('motion quality', mq_box)
    ep_form.addRow('intent', intent)
    ep_form.addRow('outcome', outcome_box)
    rl.addLayout(ep_form)
    narr = QtWidgets.QLineEdit()
    narr.setPlaceholderText('narration at playhead (Enter)')
    rl.addWidget(narr)
    narr_list = QtWidgets.QListWidget()
    rl.addWidget(narr_list, 1)
    btn_save = QtWidgets.QPushButton('Save (Ctrl+S)')
    rl.addWidget(btn_save)
    status = QtWidgets.QLabel()
    status.setWordWrap(True)
    rl.addWidget(status)
    split.addWidget(right)
    split.setSizes([260, 1000, 420])

    track_items = []
    shown = {}              # episode label -> (value, annotator, provenance) as displayed when loaded

    # ---------- helpers
    def rel(t_ns):
        return (t_ns - state['data'].t0_ns) * 1e-9

    def layer():
        return layer_box.currentText()

    def refresh_values():
        value_box.clear()
        value_box.addItems(A.LAYERS[layer()][1])
        refresh_table()

    def refresh_table():
        ed = state['ed']
        if ed is None:
            return
        key = A.LAYERS[layer()][0]
        rows = ed.ann.get(layer(), [])
        seg_table.setRowCount(len(rows))
        for i, s in enumerate(rows):
            vals = [s.get(key), f'{rel(s["t0_ns"]):.2f}',
                    '' if s.get('t1_ns') is None else f'{rel(s["t1_ns"]):.2f}',
                    f'{s.get("provenance")} {s.get("annotator", "")}'.strip(), s.get('qa_status')]
            for j, v in enumerate(vals):
                seg_table.setItem(i, j, QtWidgets.QTableWidgetItem(str(v)))

    def draw_tracks():
        for it in track_items:
            timeline.removeItem(it)
        track_items.clear()
        d, ed = state['data'], state['ed']
        if d is None:
            return
        t_end = rel(d.t1_ns)
        for row, (layer_name, key) in enumerate(TRACKS):
            for s in ed.ann.get(layer_name, []):
                x0 = rel(s['t0_ns'])
                x1 = t_end if s.get('t1_ns') is None else rel(s['t1_ns'])
                faded = s.get('qa_status') == 'rejected'
                col = QtGui.QColor(COLORS.get(s.get(key), '#9e9e9e'))
                col.setAlpha(60 if faded else (255 if s.get('provenance') != 'automated' else 170))
                h = 0.8 if s.get('provenance') != 'automated' else 0.5
                rect = QtWidgets.QGraphicsRectItem(x0, row - h / 2, max(x1 - x0, 1e-3), h)
                rect.setBrush(pg.mkBrush(col))
                rect.setPen(pg.mkPen(None))
                rect.setToolTip(f'{s.get(key)} ({s.get("provenance")}, {s.get("qa_status")})')
                timeline.addItem(rect)
                track_items.append(rect)
        for n in ed.ann.get('narration', []):
            sc = pg.ScatterPlotItem([rel(n['t_ns'])], [3], symbol='t', size=12, brush='w')
            sc.setToolTip(n['text'])
            timeline.addItem(sc)
            track_items.append(sc)
        for k, (t, kind, v) in enumerate(d.events):
            txt = v.get('phase') if kind == 'phase' else kind
            ti = pg.TextItem(str(txt), anchor=(0, 0.5), color='#bbbbbb')
            ti.setPos(rel(t), 4.25 if k % 2 else 3.75)          # stagger so close events stay readable
            timeline.addItem(ti)
            track_items.append(ti)
        narr_list.clear()
        for n in ed.ann.get('narration', []):
            narr_list.addItem(f'{rel(n["t_ns"]):6.2f} s  [{n["provenance"]}] {n["text"]}')

    def show_episode_labels():
        ann = state['ed'].ann
        for name in ('failure_class', 'motion_quality', 'intent', 'outcome'):
            lab = ann.get(name) or {}
            shown[name] = (lab.get('value'), lab.get('annotator'), lab.get('provenance'))
        fc =(ann.get('failure_class') or {}).get('value') or ''
        fc_box.setCurrentText(fc)
        mq = (ann.get('motion_quality') or {}).get('value')
        mq_box.setCurrentIndex(0 if mq is None else 1 + (5 - int(mq)))
        intent.setText((ann.get('intent') or {}).get('value') or '')
        outcome_box.setCurrentText((ann.get('outcome') or {}).get('value') or '')

    def seek(t_ns):
        d = state['data']
        if d is None:
            return
        t_ns = int(np.clip(t_ns, d.us_t[0] if len(d.us_t) else d.t0_ns, d.us_t[-1] if len(d.us_t) else d.t1_ns))
        state['t'] = t_ns
        im = d.frame_at('us', t_ns)
        if im is not None:
            us_view.setImage(im, autoLevels=False, levels=(0, 255), autoRange=False)
        im = d.frame_at('cam', t_ns)
        if im is not None:
            cam_view.setImage(im, autoLevels=False, levels=(0, 255), autoRange=False)
        x = rel(t_ns)
        playhead.blockSignals(True)
        playhead.setValue(x)
        playhead.blockSignals(False)
        force_head.setValue(x)
        slider.blockSignals(True)
        slider.setValue(int(np.searchsorted(d.us_t, t_ns)))
        slider.blockSignals(False)
        mark = '' if state['mark'] is None else f'   start mark {rel(state["mark"]):.2f} s'
        info.setText(f'{d.episode.get("episode_id")}  t = {x:.2f} s  type {d.episode.get("type")}  '
                     f'instruction: {d.episode.get("instruction") or ""}{mark}')

    def load(item):
        if state['ed'] is not None and state['ed'].dirty:
            if QtWidgets.QMessageBox.question(win, 'Unsaved', 'Discard unsaved changes?') != QtWidgets.QMessageBox.Yes:
                return
        ep_dir = Path(item.data(QtCore.Qt.UserRole))
        dlg = QtWidgets.QProgressDialog(f'Loading {ep_dir.name}', None, 0, 100, win)
        dlg.setWindowModality(QtCore.Qt.WindowModal)
        dlg.show()
        d = load_episode(ep_dir, color_topic, progress=lambda f: (dlg.setValue(int(f * 100)), app.processEvents()))
        dlg.close()
        ann = A.load_merged(ep_dir, only=only, episode=d.episode)
        state.update(data=d, ed=A.AnnotationEditor(ann, a.annotator, a.role), mark=None)
        if len(d.force_t):
            force_curve.setData((d.force_t - d.t0_ns) * 1e-9, d.force)
        slider.setRange(0, max(len(d.us_t) - 1, 0))
        timeline.setXRange(rel(d.us_t[0]) if len(d.us_t) else 0, rel(d.t1_ns))
        draw_tracks()
        refresh_values()
        show_episode_labels()
        seek(d.t0_ns)
        status.setText(f'{len(d.us)} ultrasound frames, {len(d.cam)} wrist frames, '
                       f'{len(d.force_t)} robot samples loaded')

    def mark_start():
        state['mark'] = state['t']
        seek(state['t'])

    def mark_end():
        if state['mark'] is None:
            status.setText('Mark start first')
            return
        t0, t1 = sorted((state['mark'], state['t']))
        try:
            state['ed'].add_segment(layer(), value_box.currentText(), t0, t1)
        except ValueError as e:
            status.setText(str(e))
            return
        state['mark'] = None
        draw_tracks()
        refresh_table()
        seek(state['t'])

    def selected_row():
        rows = seg_table.selectionModel().selectedRows()
        return rows[0].row() if rows else None

    def set_qa(q):
        i = selected_row()
        if i is not None:
            state['ed'].set_qa(layer(), i, q)
            draw_tracks()
            refresh_table()

    def delete():
        i = selected_row()
        if i is None:
            return
        try:
            state['ed'].delete_segment(layer(), i)
        except ValueError as e:
            status.setText(str(e))
            return
        draw_tracks()
        refresh_table()

    def add_narr():
        try:
            state['ed'].add_narration(state['t'], narr.text())
        except ValueError as e:
            status.setText(str(e))
            return
        narr.clear()
        draw_tracks()

    def save():
        ed = state['ed']
        if ed is None:
            return
        try:
            values = {'failure_class': fc_box.currentText() or None,
                      'motion_quality': None if mq_box.currentIndex() == 0 else 6 - mq_box.currentIndex(),
                      'intent': intent.text() or None, 'outcome': outcome_box.currentText() or None}
            for name, v in values.items():
                sv, by, prov = shown.get(name, (None, None, None))
                others = sv is not None and prov != 'automated' and by != a.annotator
                # unchanged own value, or another rater's label left as shown: not a new rating by this annotator;
                # an automated value saved as shown is this annotator accepting it (a rating)
                if v != ed.own_value(name) and not (v == sv and others):
                    ed.set_label(name, v)
            path = A.annotator_file(state['data'].ep_dir, a.annotator)
            ed.save(path, time.time_ns())
            show_episode_labels()
            unmatched = len(ed.ann.get('qa_unmatched') or [])
            status.setText(f'saved {path}' + (f'  ({unmatched} QA decisions refer to entries that changed)'
                                              if unmatched else ''))
        except ValueError as e:
            status.setText(f'not saved: {e}')

    def jump(click_pos):
        if state['data'] is not None:
            seek(state['data'].t0_ns + int(playhead.value() * 1e9))

    def step(n):
        d = state['data']
        if d is None or not len(d.us_t):
            return
        i = int(np.clip(np.searchsorted(d.us_t, state['t'], side='right') - 1 + n, 0, len(d.us_t) - 1))
        seek(int(d.us_t[i]))

    timer = QtCore.QTimer()
    timer.timeout.connect(lambda: step(1) if state['playing'] else None)
    timer.start(37)

    def toggle_play():
        state['playing'] = not state['playing']

    ep_list.itemDoubleClicked.connect(load)
    ep_list.itemActivated.connect(load)
    layer_box.currentTextChanged.connect(lambda _: refresh_values())
    btn_start.clicked.connect(mark_start)
    btn_end.clicked.connect(mark_end)
    btn_del.clicked.connect(delete)
    btn_save.clicked.connect(save)
    narr.returnPressed.connect(add_narr)
    playhead.sigPositionChangeFinished.connect(jump)
    slider.valueChanged.connect(lambda i: state['data'] is not None and len(state['data'].us_t) and
                                seek(int(state['data'].us_t[i])))
    for keyseq, fn in (('Space', toggle_play), ('Left', lambda: step(-1)), ('Right', lambda: step(1)),
                       ('[', mark_start), (']', mark_end), ('Ctrl+S', save)):
        sc = QtGui.QShortcut(QtGui.QKeySequence(keyseq), win)
        sc.setContext(QtCore.Qt.ApplicationShortcut)
        sc.activated.connect(fn)

    win.resize(1700, 1000)
    win.show()
    if '--smoke-test' in sys.argv:          # load the first episode and exit (CI / headless check)
        load(ep_list.item(0))
        seek(state['data'].t0_ns + int(1.7e9))
        if '--smoke-save' in sys.argv:      # also save (writes only this annotator's file)
            save()
        app.processEvents()
        if '--screenshot' in sys.argv:
            win.grab().save(sys.argv[sys.argv.index('--screenshot') + 1])
        print(status.text())
        return
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
