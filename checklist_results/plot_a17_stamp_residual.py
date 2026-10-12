#!/usr/bin/env python3
"""A-17·A-10 비교 그림을 `fb_stamp` 원자료에서 다시 그린다.

워크스페이스 루트에서 실행한다::

    python3 checklist_results/plot_a17_stamp_residual.py

관절 보고(`/output/panda_joint`)의 `수신 시각 − header.stamp` 오프셋에서 1차 추세를
뺀 잔차를 두 씬에 대해 겹쳐 그린다. 정지(A-17)가 있으면 잔차가 톱니를 그리고, 없으면
0 근처에 붙는다. 정지 구간은 수신 공백이 0.3 s 를 넘는 곳으로 정한다(체크리스트 A-17 의
수용 기준과 같은 임계).
"""
from __future__ import annotations

import glob
import json
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib import font_manager  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

RUNS = [
    ('Crisp (t1) · 09-29', 'misc/fb_stamp/t1_20260929.json', '#2a78d6'),
    ('RecurDyn (r2) · 09-30', 'misc/fb_stamp/r2_20260930.json', '#eb6834'),
]
STALL_GAP_SEC = 0.3
SURFACE, INK, INK2, GRID, STALL = '#fcfcfb', '#0b0b0b', '#52514e', '#e4e3df', '#f0efec'
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'fig_a17_stamp_residual.png')


def use_cjk_font() -> None:
    """한글 글리프가 있는 Noto Sans CJK 를 등록한다 (.ttc 는 자동 탐색에 안 잡힌다)."""
    for path in glob.glob('/usr/share/fonts/opentype/noto/NotoSansCJK-*.ttc'):
        font_manager.fontManager.addfont(path)
    names = sorted({f.name for f in font_manager.fontManager.ttflist if f.name.startswith('Noto Sans CJK')})
    if names:
        plt.rcParams['font.family'] = names[0]
    plt.rcParams['axes.unicode_minus'] = False


def load(path: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """경과 시간, 추세를 뺀 오프셋 잔차, 정지 시작 인덱스를 돌려준다."""
    raw = np.array(json.load(open(path))['raw']['/output/panda_joint'])
    recv, stamp = raw[:, 0], raw[:, 1]
    t = recv - recv[0]
    offset = recv - stamp
    slope, intercept = np.polyfit(t, offset, 1)
    resid = offset - (slope * t + intercept)
    stalls = np.where(np.diff(t) > STALL_GAP_SEC)[0]
    return t, resid, stalls


def main() -> None:
    use_cjk_font()
    fig, ax = plt.subplots(figsize=(9.6, 3.9), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)

    handles = []
    for zorder, (label, path, color) in enumerate(RUNS, start=3):
        t, resid, stalls = load(path)
        for i in stalls:
            ax.axvspan(t[i], t[i + 1], color=STALL, zorder=0, lw=0)
        ax.plot(t, resid, color=color, lw=1.6, zorder=zorder)
        text = '%s — 정지 %d회, 잔차 std %.3f s' % (label, len(stalls), resid.std())
        handles.append(plt.Line2D([], [], color=color, lw=2, label=text))
        print(text)
    handles.append(Patch(facecolor=STALL, edgecolor='none', label='정지 구간 (수신 공백 > %.1f s)' % STALL_GAP_SEC))

    ax.set_xlim(0, 180)
    ax.set_ylim(-1.0, 1.0)
    ax.set_xlabel('관측 경과 시간 (s)', color=INK2, fontsize=9)
    ax.set_ylabel('수신 − stamp 오프셋 잔차 (s)', color=INK2, fontsize=9)
    ax.tick_params(colors=INK2, labelsize=8, length=0)
    ax.grid(axis='y', color=GRID, lw=0.8)
    for spine in ax.spines.values():
        spine.set_visible(False)
    ax.axhline(0, color='#c9c8c3', lw=0.8, zorder=1)

    leg = ax.legend(handles=handles, loc='lower left', bbox_to_anchor=(0, 1.01), ncol=3, frameon=False,
                    fontsize=8.2, handlelength=1.6, columnspacing=1.4, borderaxespad=0)
    for txt in leg.get_texts():
        txt.set_color(INK)
    fig.text(0.07, 0.955, 'A-17 · A-10 — 관절 보고 stamp 오프셋 잔차, 180 s 관측 (MMR-Build_260929)', ha='left',
             fontsize=11, color=INK)
    fig.subplots_adjust(left=0.07, right=0.985, top=0.82, bottom=0.14)
    fig.savefig(OUT, facecolor=SURFACE)
    print(OUT)


if __name__ == '__main__':
    main()
