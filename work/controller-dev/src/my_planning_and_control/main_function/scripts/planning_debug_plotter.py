#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""EM planner SL/ST 调试图。

订阅 planning_agent 发布的 /carla/ego_vehicle/planning_debug（std_msgs/String, JSON），
实时画出三张图：
  * SL 图：DP 采样点、DP 路径、凸空间、QP 路径、静态/动态障碍物
  * ST 图：速度 DP 采样、凸空间 s_lb/s_ub、ST 图障碍物、障碍物恒速预测线、DP/QP 速度曲线
  * VT 图：曲率限速 v_ub、DP/QP 速度、参考速度

用法：
  实时（launch 自动启动，或手动）：ros2 run my_pnc planning_debug_plotter.py
  回放记录：python3 planning_debug_plotter.py --replay planning_debug_xxx.jsonl
  环境变量 PNC_DEBUG_DIR：设置后每帧原始 JSON 追加写入该目录的 .jsonl，截图也存到该目录。
  没有 DISPLAY 时：若设置了 PNC_DEBUG_DIR 则只记录不画图，否则直接退出。

快捷键：空格 暂停/继续，←/→ 逐帧，S 保存截图。
"""

import argparse
import collections
import json
import math
import os
import sys
import time

TOPIC = '/carla/ego_vehicle/planning_debug'
MAX_FRAMES = 600  # 规划 10Hz 时约 60 s 历史


# ----------------------------------------------------------------------------
# 记录
# ----------------------------------------------------------------------------
class Recorder(object):
    def __init__(self, directory):
        self.path = None
        self._file = None
        self._count = 0
        if not directory:
            return
        try:
            os.makedirs(directory, exist_ok=True)
            name = time.strftime('planning_debug_%Y%m%d-%H%M%S.jsonl')
            self.path = os.path.join(directory, name)
            self._file = open(self.path, 'a')
        except OSError as exc:
            print('[planning_debug_plotter] 无法创建记录文件: %s' % exc, file=sys.stderr)
            self.path = None
            self._file = None

    def write(self, raw):
        if self._file is None:
            return
        self._file.write(raw.replace('\n', ' ') + '\n')
        self._count += 1
        if self._count % 20 == 0:
            self._file.flush()

    def close(self):
        if self._file is not None:
            self._file.flush()
            self._file.close()
            self._file = None


def finite(v):
    return v is not None and isinstance(v, (int, float)) and math.isfinite(v)


# ----------------------------------------------------------------------------
# 图元构建（与 Qt 无关，纯数据）
# item: dict(kind, xs, ys, color, width, dash, size, fill, label, text)
# ----------------------------------------------------------------------------
GRAY = (150, 150, 150)
LIGHT = (205, 205, 205)
BLUE = (30, 100, 220)
RED = (220, 30, 40)
GREEN = (40, 160, 70)
ORANGE = (240, 140, 0)
DARK = (60, 60, 60)
MAGENTA = (200, 0, 200)
BLACK = (0, 0, 0)


def line(xs, ys, color, width=1.5, dash=False, label=None, marker=0):
    return dict(kind='line', xs=list(xs), ys=list(ys), color=color, width=width,
                dash=dash, label=label, size=marker)


def scatter(xs, ys, color, size=3, label=None, shape='o'):
    return dict(kind='scatter', xs=list(xs), ys=list(ys), color=color, size=size,
                label=label, shape=shape)


def polygon(xs, ys, color, fill_alpha=60, width=1.0, label=None):
    return dict(kind='polygon', xs=list(xs), ys=list(ys), color=color,
                fill_alpha=fill_alpha, width=width, label=label)


def text(x, y, s, color=DARK):
    return dict(kind='text', x=x, y=y, text=s, color=color)


def rect_xy(cx, cy, half_x, half_y):
    xs = [cx - half_x, cx + half_x, cx + half_x, cx - half_x]
    ys = [cy - half_y, cy - half_y, cy + half_y, cy + half_y]
    return xs, ys


def build_sl(frame):
    sl = frame.get('sl', {})
    items = []
    up = sl.get('road_up', 6.0)
    low = sl.get('road_low', -2.0)
    if not finite(up):
        up = 6.0
    if not finite(low):
        low = -2.0
    obs_len = sl.get('obs_length', 5.0) or 5.0
    obs_wid = sl.get('obs_width', 3.0) or 3.0

    dense = sl.get('dense_dp_path', {})
    ds, dl = dense.get('s', []), dense.get('l', [])
    qp = sl.get('qp_path', {})
    dp = sl.get('dp_path', {})
    l_min, l_max = sl.get('l_min', []), sl.get('l_max', [])
    sample_s, sample_l = sl.get('dp_sample_s', []), sl.get('dp_sample_l', [])
    ego = sl.get('ego', [0.0, 0.0])
    start = sl.get('start', [0.0, 0.0])
    static_obs = sl.get('static_obs', [])
    dyn_obs = sl.get('dynamic_obs', [])

    all_s = [v for v in ds + sample_s if finite(v)]
    all_s += [o['s'] for o in static_obs + dyn_obs if finite(o.get('s'))]
    all_s += [v for v in (ego[0], start[0]) if finite(v)]
    x0 = min(all_s + [-5.0]) - 5.0
    x1 = max(all_s + [50.0]) + 5.0
    y0, y1 = low - 1.5, up + 1.5

    # 道路边界、车道中心线、车道分界线
    mid = 0.5 * (up + low)
    items.append(line([x0, x1], [up, up], DARK, 1.5, label='road boundary'))
    items.append(line([x0, x1], [low, low], DARK, 1.5))
    items.append(line([x0, x1], [mid, mid], GRAY, 1.0, dash=True, label='lane divider'))
    items.append(line([x0, x1], [0, 0], LIGHT, 1.0, dash=True))

    # 凸空间
    n = min(len(ds), len(l_min), len(l_max))
    if n >= 2:
        pts = [(ds[i], l_min[i], l_max[i]) for i in range(n)
               if finite(ds[i]) and finite(l_min[i]) and finite(l_max[i])]
        if len(pts) >= 2:
            px = [p[0] for p in pts] + [p[0] for p in reversed(pts)]
            py = [p[1] for p in pts] + [p[2] for p in reversed(pts)]
            items.append(polygon(px, py, GREEN, fill_alpha=40, width=0, label='convex space'))
            items.append(line([p[0] for p in pts], [p[1] for p in pts], GREEN, 1.2))
            items.append(line([p[0] for p in pts], [p[2] for p in pts], GREEN, 1.2))

    # DP 采样点
    if sample_s and sample_l:
        gx, gy = [], []
        for s in sample_s:
            for l in sample_l:
                gx.append(s)
                gy.append(l)
        items.append(scatter(gx, gy, GRAY, 3, label='DP samples'))

    # 障碍物
    for i, o in enumerate(static_obs):
        if finite(o.get('s')) and finite(o.get('l')):
            xs, ys = rect_xy(o['s'], o['l'], obs_len / 2.0, obs_wid / 2.0)
            items.append(polygon(xs, ys, DARK, fill_alpha=120, width=1.5,
                                 label='static obs (%.0fx%.0f assumed)' % (obs_len, obs_wid) if i == 0 else None))
    for i, o in enumerate(dyn_obs):
        if finite(o.get('s')) and finite(o.get('l')):
            xs, ys = rect_xy(o['s'], o['l'], obs_len / 2.0, obs_wid / 2.0)
            items.append(polygon(xs, ys, ORANGE, fill_alpha=90, width=1.5,
                                 label='dynamic obs (not used by path)' if i == 0 else None))
            v = o.get('s_dot')
            if finite(v):
                items.append(text(o['s'] - obs_len / 2.0, o['l'] + obs_wid / 2.0 + 0.25,
                                  'ds/dt %.1f  dl/dt %.2f' % (v, o.get('l_dot') or 0.0), ORANGE))

    # 路径
    items.append(line(dp.get('s', []), dp.get('l', []), BLUE, 1.2, dash=True, label='DP path', marker=4))
    items.append(line(ds, dl, BLUE, 1.0))
    items.append(line(qp.get('s', []), qp.get('l', []), RED, 2.5, label='QP path'))

    # 主车和规划起点
    if finite(ego[0]) and finite(ego[1]):
        items.append(scatter([ego[0]], [ego[1]], BLACK, 7, label='ego', shape='s'))
    if finite(start[0]) and finite(start[1]):
        items.append(scatter([start[0]], [start[1]], MAGENTA, 7, label='planning start', shape='x'))
    return items, (x0, x1), (y0, y1)


def build_st(frame):
    st = frame.get('st', {})
    items = []
    dp = st.get('dp', {})
    qp = st.get('qp', {})
    sample_t, sample_s = st.get('sample_t', []), st.get('sample_s', [])
    s_lb, s_ub = st.get('s_lb', []), st.get('s_ub', [])
    path_len = st.get('path_length', 0.0)
    if not finite(path_len):
        path_len = 0.0
    t_dp = dp.get('t', [])
    t_end = max([t for t in sample_t + t_dp if finite(t)] + [8.0])
    s_top = max([path_len] + [v for v in dp.get('s', []) + qp.get('s', []) if finite(v)] + [10.0])

    # 采样网格
    if sample_t and sample_s:
        gx, gy = [], []
        for t in sample_t:
            for s in sample_s:
                gx.append(t)
                gy.append(s)
        items.append(scatter(gx, gy, LIGHT, 2, label='DP samples'))

    # 凸空间
    n = min(len(t_dp), len(s_lb), len(s_ub))
    pts = [(t_dp[i], s_lb[i], s_ub[i]) for i in range(n)
           if finite(t_dp[i]) and finite(s_lb[i]) and finite(s_ub[i])]
    if len(pts) >= 2:
        px = [p[0] for p in pts] + [p[0] for p in reversed(pts)]
        py = [p[1] for p in pts] + [p[2] for p in reversed(pts)]
        items.append(polygon(px, py, GREEN, fill_alpha=35, width=0, label='s_lb / s_ub'))
        items.append(line([p[0] for p in pts], [p[1] for p in pts], GREEN, 1.2))
        items.append(line([p[0] for p in pts], [p[2] for p in pts], GREEN, 1.2))

    # 参考速度对应的匀速线
    ref = frame.get('ref_speed')
    if finite(ref):
        items.append(line([0, t_end], [0, ref * t_end], GRAY, 1.0, dash=True, label='s = v_ref * t'))

    # 动态障碍物恒速预测线（规划路径附近 |l|<3 的）：用于判断障碍物是否被 ST 图漏掉
    first = True
    for o in st.get('obs_on_path', []):
        s, l, v = o.get('s'), o.get('l'), o.get('s_dot')
        if not (finite(s) and finite(l) and finite(v)) or abs(l) >= 3.0:
            continue
        items.append(line([0, t_end], [s, s + v * t_end], ORANGE, 1.5, dash=True,
                          label='obs s0+v*t (|l|<3)' if first else None))
        items.append(text(0.1, s + 0.8, 'l=%.1f dl/dt=%.2f' % (l, o.get('l_dot') or 0.0), ORANGE))
        first = False

    # ST 图中的障碍物（规划器实际使用的）
    first = True
    for node in st.get('st_graph', []):
        vals = [node.get(k) for k in ('t_in', 't_out', 's_in', 's_out')]
        if not all(finite(v) for v in vals):
            continue
        t_in, t_out, s_in, s_out = vals
        items.append(line([t_in, t_out], [s_in, s_out], RED, 5.0,
                          label='ST graph obstacle' if first else None))
        first = False

    items.append(line(t_dp, dp.get('s', []), BLUE, 1.2, dash=True, label='DP speed', marker=4))
    items.append(line(qp.get('t', []), qp.get('s', []), RED, 2.5, label='QP speed'))
    return items, (0.0, t_end), (0.0, s_top * 1.05)


def build_vt(frame):
    st = frame.get('st', {})
    items = []
    dp = st.get('dp', {})
    qp = st.get('qp', {})
    t_dp = dp.get('t', [])
    ref = frame.get('ref_speed')
    ego_v = frame.get('ego_v')
    vals = [v for v in dp.get('v', []) + qp.get('v', []) if finite(v)]
    vals += [v for v in (ref, ego_v) if finite(v)]
    v_top = max(vals + [2.0]) * 1.3
    t_end = max([t for t in t_dp if finite(t)] + [8.0])

    v_ub = [v if (finite(v) and v < v_top * 3) else None for v in st.get('v_ub', [])]
    items.append(line(t_dp, v_ub, GREEN, 1.5, dash=True, label='v_ub (curvature)'))
    items.append(line(t_dp, st.get('v_lb', []), GREEN, 1.0, dash=True))
    if finite(ref):
        items.append(line([0, t_end], [ref, ref], GRAY, 1.0, dash=True, label='v_ref'))
    items.append(line(t_dp, dp.get('v', []), BLUE, 1.2, dash=True, label='DP v', marker=4))
    items.append(line(qp.get('t', []), qp.get('v', []), RED, 2.5, label='QP v'))
    if finite(ego_v):
        items.append(scatter([0.0], [ego_v], BLACK, 7, label='ego v', shape='s'))
    a = [v for v in qp.get('a', []) if finite(v)]
    if a:
        items.append(text(0.2, v_top * 0.95, 'QP a: min %.2f  max %.2f m/s^2' % (min(a), max(a))))
    return items, (0.0, t_end), (0.0, v_top)


def frame_summary(frame):
    sl, st = frame.get('sl', {}), frame.get('st', {})
    ok = lambda b: 'OK' if b else 'FAIL'
    near = [o for o in st.get('obs_on_path', []) if finite(o.get('l')) and abs(o['l']) < 3.0]
    return ('t=%.2f  compute=%.1f ms  pathQP=%s  speedQP=%s  ego v=%.2f  v_ref=%.2f  |  '
            'static=%d  dynamic=%d (near path %d) -> ST graph %d' % (
                frame.get('t') or 0.0, frame.get('compute_ms') or 0.0,
                ok(frame.get('path_qp_ok')), ok(frame.get('speed_qp_ok')),
                frame.get('ego_v') or 0.0, frame.get('ref_speed') or 0.0,
                len(sl.get('static_obs', [])), len(st.get('obs_on_path', [])), len(near),
                len(st.get('st_graph', []))))


# ----------------------------------------------------------------------------
# Qt 界面
# ----------------------------------------------------------------------------
def run_gui(args, frames, recorder, ros_node):
    from PyQt5 import QtCore, QtGui, QtWidgets
    Qt = QtCore.Qt

    def nice_ticks(lo, hi, n=6):
        span = hi - lo
        if span <= 0 or not math.isfinite(span):
            return []
        raw = span / float(n)
        mag = 10 ** math.floor(math.log10(raw))
        step = mag
        for m in (1, 2, 2.5, 5, 10):
            if raw <= m * mag:
                step = m * mag
                break
        start = math.ceil(lo / step) * step
        ticks = []
        v = start
        while v <= hi + 1e-9 and len(ticks) < 50:
            ticks.append(round(v, 10))
            v += step
        return ticks

    class PlotPanel(QtWidgets.QWidget):
        ML, MR, MT, MB = 58, 12, 24, 38

        def __init__(self, title, xlabel, ylabel):
            super(PlotPanel, self).__init__()
            self.title, self.xlabel, self.ylabel = title, xlabel, ylabel
            self.items, self.xlim, self.ylim = [], (0, 1), (0, 1)
            self.setMinimumSize(320, 220)
            self.setAutoFillBackground(True)
            pal = self.palette()
            pal.setColor(self.backgroundRole(), Qt.white)
            self.setPalette(pal)

        def set_data(self, items, xlim, ylim):
            self.items, self.xlim, self.ylim = items, xlim, ylim
            self.update()

        def paintEvent(self, _event):
            p = QtGui.QPainter(self)
            p.setRenderHint(QtGui.QPainter.Antialiasing, True)
            w, h = self.width(), self.height()
            area = QtCore.QRectF(self.ML, self.MT, max(10, w - self.ML - self.MR), max(10, h - self.MT - self.MB))
            (x0, x1), (y0, y1) = self.xlim, self.ylim
            if x1 - x0 <= 1e-9:
                x1 = x0 + 1.0
            if y1 - y0 <= 1e-9:
                y1 = y0 + 1.0
            xr, yr = x1 - x0, y1 - y0

            def X(x):
                x = min(max(x, x0 - 10 * xr), x1 + 10 * xr)
                return area.left() + (x - x0) / xr * area.width()

            def Y(y):
                y = min(max(y, y0 - 10 * yr), y1 + 10 * yr)
                return area.bottom() - (y - y0) / yr * area.height()

            # 标题与坐标轴
            p.setPen(QtGui.QPen(Qt.black))
            f = p.font()
            f.setPointSizeF(9.5)
            f.setBold(True)
            p.setFont(f)
            p.drawText(QtCore.QRectF(0, 2, w, self.MT - 4), Qt.AlignCenter, self.title)
            f.setBold(False)
            f.setPointSizeF(8)
            p.setFont(f)
            grid_pen = QtGui.QPen(QtGui.QColor(235, 235, 235))
            for t in nice_ticks(x0, x1):
                px = X(t)
                p.setPen(grid_pen)
                p.drawLine(QtCore.QPointF(px, area.top()), QtCore.QPointF(px, area.bottom()))
                p.setPen(Qt.black)
                p.drawText(QtCore.QRectF(px - 30, area.bottom() + 2, 60, 14), Qt.AlignCenter, '%g' % t)
            for t in nice_ticks(y0, y1):
                py = Y(t)
                p.setPen(grid_pen)
                p.drawLine(QtCore.QPointF(area.left(), py), QtCore.QPointF(area.right(), py))
                p.setPen(Qt.black)
                p.drawText(QtCore.QRectF(0, py - 7, self.ML - 4, 14), int(Qt.AlignRight | Qt.AlignVCenter), '%g' % t)
            p.drawText(QtCore.QRectF(area.left(), h - 16, area.width(), 14), Qt.AlignCenter, self.xlabel)
            p.save()
            p.translate(12, area.center().y())
            p.rotate(-90)
            p.drawText(QtCore.QRectF(-area.height() / 2, -8, area.height(), 16), Qt.AlignCenter, self.ylabel)
            p.restore()
            p.setPen(QtGui.QPen(Qt.black, 1))
            p.drawRect(area)

            # 图元
            p.save()
            p.setClipRect(area)
            legend = []
            for it in self.items:
                color = QtGui.QColor(*it.get('color', BLACK))
                kind = it['kind']
                if kind == 'line':
                    pen = QtGui.QPen(color, it.get('width', 1.5))
                    if it.get('dash'):
                        pen.setStyle(Qt.DashLine)
                    p.setPen(pen)
                    seg = []
                    pts_all = []
                    for x, y in zip(it['xs'], it['ys']):
                        if finite(x) and finite(y):
                            pt = QtCore.QPointF(X(x), Y(y))
                            seg.append(pt)
                            pts_all.append(pt)
                        else:
                            if len(seg) >= 2:
                                p.drawPolyline(QtGui.QPolygonF(seg))
                            seg = []
                    if len(seg) >= 2:
                        p.drawPolyline(QtGui.QPolygonF(seg))
                    if it.get('size'):
                        p.setPen(QtGui.QPen(color, 1))
                        p.setBrush(color)
                        r = it['size'] / 2.0
                        for pt in pts_all:
                            p.drawEllipse(pt, r, r)
                        p.setBrush(Qt.NoBrush)
                elif kind == 'scatter':
                    r = it.get('size', 3) / 2.0
                    shape = it.get('shape', 'o')
                    p.setPen(QtGui.QPen(color, 2 if shape == 'x' else 1))
                    p.setBrush(color if shape != 'x' else Qt.NoBrush)
                    for x, y in zip(it['xs'], it['ys']):
                        if not (finite(x) and finite(y)):
                            continue
                        c = QtCore.QPointF(X(x), Y(y))
                        if shape == 's':
                            p.drawRect(QtCore.QRectF(c.x() - r, c.y() - r, 2 * r, 2 * r))
                        elif shape == 'x':
                            p.drawLine(QtCore.QPointF(c.x() - r, c.y() - r), QtCore.QPointF(c.x() + r, c.y() + r))
                            p.drawLine(QtCore.QPointF(c.x() - r, c.y() + r), QtCore.QPointF(c.x() + r, c.y() - r))
                        else:
                            p.drawEllipse(c, r, r)
                    p.setBrush(Qt.NoBrush)
                elif kind == 'polygon':
                    pts = [QtCore.QPointF(X(x), Y(y)) for x, y in zip(it['xs'], it['ys']) if finite(x) and finite(y)]
                    if len(pts) >= 3:
                        fill = QtGui.QColor(color)
                        fill.setAlpha(it.get('fill_alpha', 60))
                        p.setBrush(fill)
                        if it.get('width', 1.0) > 0:
                            p.setPen(QtGui.QPen(color, it.get('width', 1.0)))
                        else:
                            p.setPen(Qt.NoPen)
                        p.drawPolygon(QtGui.QPolygonF(pts))
                        p.setBrush(Qt.NoBrush)
                elif kind == 'text':
                    if finite(it['x']) and finite(it['y']):
                        p.setPen(color)
                        p.drawText(QtCore.QPointF(X(it['x']), Y(it['y'])), it['text'])
                if it.get('label'):
                    legend.append(it)
            p.restore()

            # 图例（右上角）
            if legend:
                fm = p.fontMetrics()
                lw = max(fm.width(it['label']) for it in legend) + 34
                lh = 15 * len(legend) + 6
                box = QtCore.QRectF(area.right() - lw - 6, area.top() + 6, lw, lh)
                p.setPen(QtGui.QPen(QtGui.QColor(180, 180, 180)))
                p.setBrush(QtGui.QColor(255, 255, 255, 215))
                p.drawRect(box)
                p.setBrush(Qt.NoBrush)
                for i, it in enumerate(legend):
                    y = box.top() + 11 + 15 * i
                    color = QtGui.QColor(*it.get('color', BLACK))
                    if it['kind'] == 'polygon':
                        fill = QtGui.QColor(color)
                        fill.setAlpha(max(it.get('fill_alpha', 60), 80))
                        p.setPen(Qt.NoPen)
                        p.setBrush(fill)
                        p.drawRect(QtCore.QRectF(box.left() + 5, y - 5, 20, 10))
                        p.setBrush(Qt.NoBrush)
                    elif it['kind'] == 'scatter':
                        p.setPen(color)
                        p.setBrush(color)
                        p.drawEllipse(QtCore.QPointF(box.left() + 15, y), 3, 3)
                        p.setBrush(Qt.NoBrush)
                    else:
                        pen = QtGui.QPen(color, min(it.get('width', 1.5), 4))
                        if it.get('dash'):
                            pen.setStyle(Qt.DashLine)
                        p.setPen(pen)
                        p.drawLine(QtCore.QPointF(box.left() + 5, y), QtCore.QPointF(box.left() + 25, y))
                    p.setPen(Qt.black)
                    p.drawText(QtCore.QPointF(box.left() + 30, y + 4), it['label'])
            p.end()

    class Window(QtWidgets.QMainWindow):
        def __init__(self):
            super(Window, self).__init__()
            self.setWindowTitle('EM planner debug: SL / ST' + ('  [replay]' if args.replay else ''))
            self.frames = frames
            self.index = len(frames) - 1
            self.paused = bool(args.replay)
            self.dirty = True
            self.last_rx = collections.deque(maxlen=30)
            self.snapshot_done = False

            self.sl = PlotPanel('SL graph (reference line frame, s=0 at ego projection, l>0 = left)', 's [m]', 'l [m]')
            self.st = PlotPanel('ST graph (planned path frame, s=0 at planning start)', 't [s]', 's [m]')
            self.vt = PlotPanel('VT graph', 't [s]', 'v [m/s]')
            self.plots = QtWidgets.QWidget()
            grid = QtWidgets.QVBoxLayout(self.plots)
            grid.setContentsMargins(0, 0, 0, 0)
            grid.addWidget(self.sl, 1)
            bottom = QtWidgets.QHBoxLayout()
            bottom.addWidget(self.st, 3)
            bottom.addWidget(self.vt, 2)
            grid.addLayout(bottom, 1)

            self.pause_btn = QtWidgets.QPushButton()
            self.pause_btn.clicked.connect(self.toggle_pause)
            self.save_btn = QtWidgets.QPushButton('Save PNG')
            self.save_btn.clicked.connect(self.save_png)
            self.slider = QtWidgets.QSlider(Qt.Horizontal)
            self.slider.setMinimum(0)
            self.slider.sliderMoved.connect(self.on_slider)
            self.frame_label = QtWidgets.QLabel()
            controls = QtWidgets.QHBoxLayout()
            controls.addWidget(self.pause_btn)
            controls.addWidget(self.slider, 1)
            controls.addWidget(self.frame_label)
            controls.addWidget(self.save_btn)
            self.info = QtWidgets.QLabel('waiting for %s ...' % TOPIC)
            self.info.setTextInteractionFlags(Qt.TextSelectableByMouse)

            central = QtWidgets.QWidget()
            layout = QtWidgets.QVBoxLayout(central)
            layout.addWidget(self.plots, 1)
            layout.addLayout(controls)
            layout.addWidget(self.info)
            self.setCentralWidget(central)
            self.resize(1280, 860)
            self.update_pause_text()

            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(self.tick)
            self.timer.start(30)
            self.replay_acc = 0.0

        # ---- 数据 ----
        def on_message(self, raw):
            recorder.write(raw)
            try:
                frame = json.loads(raw)
            except ValueError:
                return
            self.frames.append(frame)
            self.last_rx.append(time.monotonic())
            if not self.paused:
                self.index = len(self.frames) - 1
            else:
                # deque 满后整体左移，暂停时保持看的是同一帧
                if len(self.frames) == self.frames.maxlen:
                    self.index = max(0, self.index - 1)
            self.dirty = True

        def tick(self):
            if ros_node is not None:
                import rclpy
                for _ in range(20):
                    rclpy.spin_once(ros_node, timeout_sec=0.0)
                    if not ros_node.pending:
                        break
                while ros_node.pending:
                    self.on_message(ros_node.pending.popleft())
            elif not self.paused and self.frames:
                # 回放：按 10Hz 推进
                self.replay_acc += 0.03
                if self.replay_acc >= 0.1:
                    self.replay_acc = 0.0
                    if self.index < len(self.frames) - 1:
                        self.index += 1
                        self.dirty = True
                    else:
                        self.paused = True
                        self.update_pause_text()
            if self.dirty:
                self.dirty = False
                self.redraw()
            if args.snapshot and not self.snapshot_done and len(self.frames) >= args.snapshot_after:
                self.snapshot_done = True
                self.redraw()
                QtWidgets.QApplication.processEvents()
                self.grab().save(args.snapshot)
                print('[planning_debug_plotter] snapshot saved: %s' % args.snapshot)
                QtWidgets.QApplication.quit()

        def redraw(self):
            n = len(self.frames)
            self.slider.setMaximum(max(0, n - 1))
            if n == 0:
                return
            self.index = min(max(self.index, 0), n - 1)
            self.slider.setValue(self.index)
            frame = self.frames[self.index]
            for panel, builder in ((self.sl, build_sl), (self.st, build_st), (self.vt, build_vt)):
                try:
                    items, xl, yl = builder(frame)
                except Exception as exc:  # 单帧数据异常不应让窗口退出
                    items, xl, yl = [], (0, 1), (0, 1)
                    print('[planning_debug_plotter] bad frame: %r' % exc, file=sys.stderr)
                panel.set_data(items, xl, yl)
            rate = ''
            if len(self.last_rx) >= 2 and ros_node is not None:
                span = self.last_rx[-1] - self.last_rx[0]
                if span > 0:
                    rate = '  |  rx %.1f Hz' % ((len(self.last_rx) - 1) / span)
            self.frame_label.setText('frame %d / %d' % (self.index + 1, n))
            self.info.setText(frame_summary(frame) + rate)

        # ---- 交互 ----
        def update_pause_text(self):
            if args.replay:
                self.pause_btn.setText('Play' if self.paused else 'Pause')
            else:
                self.pause_btn.setText('Resume live' if self.paused else 'Pause')

        def toggle_pause(self):
            self.paused = not self.paused
            if not self.paused and not args.replay:
                self.index = len(self.frames) - 1
            if not self.paused and args.replay and self.index >= len(self.frames) - 1:
                self.index = 0
            self.update_pause_text()
            self.dirty = True

        def on_slider(self, value):
            self.paused = True
            self.index = value
            self.update_pause_text()
            self.dirty = True

        def keyPressEvent(self, event):
            key = event.key()
            if key == Qt.Key_Space:
                self.toggle_pause()
            elif key in (Qt.Key_Left, Qt.Key_Right):
                self.paused = True
                self.index += -1 if key == Qt.Key_Left else 1
                self.update_pause_text()
                self.dirty = True
            elif key == Qt.Key_S:
                self.save_png()
            else:
                super(Window, self).keyPressEvent(event)

        def save_png(self):
            directory = args.record_dir or os.getcwd()
            try:
                os.makedirs(directory, exist_ok=True)
            except OSError:
                directory = '/tmp'
            path = os.path.join(directory, time.strftime('planning_debug_%Y%m%d-%H%M%S') +
                                '_f%d.png' % (self.index + 1))
            if not self.plots.grab().save(path):
                path = os.path.join('/tmp', os.path.basename(path))
                self.plots.grab().save(path)
            self.info.setText('saved: %s' % path)

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
    win = Window()
    win.show()
    try:
        return app.exec_()
    finally:
        recorder.close()


# ----------------------------------------------------------------------------
# ROS
# ----------------------------------------------------------------------------
def make_ros_node():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class DebugListener(Node):
        def __init__(self):
            super(DebugListener, self).__init__('planning_debug_plotter')
            self.pending = collections.deque(maxlen=50)
            self.create_subscription(String, TOPIC, self._cb, 10)

        def _cb(self, msg):
            self.pending.append(msg.data)

    rclpy.init(args=sys.argv)
    return DebugListener()


def run_record_only(recorder):
    import rclpy
    node = make_ros_node()
    print('[planning_debug_plotter] no DISPLAY: record only -> %s' % recorder.path)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            while node.pending:
                recorder.write(node.pending.popleft())
    except KeyboardInterrupt:
        pass
    finally:
        recorder.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def main():
    argv = sys.argv[1:]
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    parser = argparse.ArgumentParser(description='EM planner SL/ST debug plotter')
    parser.add_argument('--replay', help='回放 .jsonl 记录文件（不需要 ROS）')
    parser.add_argument('--record-dir', default=os.environ.get('PNC_DEBUG_DIR', ''),
                        help='记录 JSONL 与截图的目录（默认取环境变量 PNC_DEBUG_DIR）')
    parser.add_argument('--snapshot', help='收到足够帧后保存截图并退出（用于无界面检查）')
    parser.add_argument('--snapshot-after', type=int, default=5)
    args = parser.parse_args(argv)

    frames = collections.deque(maxlen=MAX_FRAMES)
    has_display = bool(os.environ.get('DISPLAY')) or os.environ.get('QT_QPA_PLATFORM') == 'offscreen'

    if args.replay:
        with open(args.replay) as f:
            for raw in f:
                raw = raw.strip()
                if raw:
                    try:
                        frames.append(json.loads(raw))
                    except ValueError:
                        pass
        frames = collections.deque(frames)  # 回放不限长度
        if not frames:
            print('记录文件中没有有效帧: %s' % args.replay, file=sys.stderr)
            return 1
        if args.snapshot:
            args.snapshot_after = 0
        return run_gui(args, frames, Recorder(''), None)

    recorder = Recorder(args.record_dir)
    if recorder.path:
        print('[planning_debug_plotter] recording -> %s' % recorder.path)
    if not has_display:
        if recorder.path:
            return run_record_only(recorder)
        print('[planning_debug_plotter] no DISPLAY and no PNC_DEBUG_DIR, exit.')
        return 0

    node = make_ros_node()
    try:
        return run_gui(args, frames, recorder, node)
    finally:
        import rclpy
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main() or 0)
