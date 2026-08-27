#!/usr/bin/env python3
"""分析 drift_test rosbag 中 /Odometry 的 z 漂移（体素层窗口定参用）

用法: python3 analyze_drift.py [bag_db3路径]
"""
import sqlite3
import sys

from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

DB = sys.argv[1] if len(sys.argv) > 1 else \
    '/home/nvidia/dog_ws/rosbag_manual/drift_test_20260827_114208/drift_test_20260827_114208_0.db3'


def load(con, topic):
    tid = con.execute('SELECT id FROM topics WHERE name=?', (topic,)).fetchone()
    if tid is None:
        return []
    rows = con.execute(
        'SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp',
        (tid[0],))
    out = []
    for t, data in rows:
        m = deserialize_message(data, Odometry)
        out.append((t * 1e-9, m.pose.pose.position.x,
                    m.pose.pose.position.y, m.pose.pose.position.z))
    return out


def summarize(name, seq):
    if not seq:
        print(f'{name}: 无数据')
        return
    t0 = seq[0][0]
    zs = [p[3] for p in seq]
    imin, imax = zs.index(min(zs)), zs.index(max(zs))
    print(f'\n=== {name} ({len(seq)} 帧, 时长 {seq[-1][0]-t0:.1f}s) ===')
    print(f'起点 z = {zs[0]:+.3f} m   终点 z = {zs[-1]:+.3f} m   '
          f'净漂移 = {zs[-1]-zs[0]:+.3f} m')
    print(f'z 范围: [{min(zs):+.3f}, {max(zs):+.3f}] 跨度 {max(zs)-min(zs):.3f} m')
    print(f'z_min 出现于 t={seq[imin][0]-t0:6.1f}s   '
          f'z_max 出现于 t={seq[imax][0]-t0:6.1f}s')
    # 每 30s 采样一次
    print('时间采样 (t, x, y, z):')
    nxt = 0.0
    for t, x, y, z in seq:
        rt = t - t0
        if rt >= nxt:
            print(f'  t={rt:6.1f}s  x={x:+7.2f} y={y:+7.2f} z={z:+7.3f}')
            nxt += 30.0
    # 闭环检查: 终点距起点
    dx = seq[-1][1] - seq[0][1]
    dy = seq[-1][2] - seq[0][2]
    print(f'闭环检查: 终点距起点 {((dx*dx+dy*dy)**0.5):.2f} m')


con = sqlite3.connect(DB)
summarize('/Odometry (FAST-LIO, z漂移源头)', load(con, '/Odometry'))
summarize('/localization (融合, 交叉校验)', load(con, '/localization'))
con.close()
