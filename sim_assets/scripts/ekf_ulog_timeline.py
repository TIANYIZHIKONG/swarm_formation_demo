#!/usr/bin/env python3
"""时间线: cs_ev_yaw / cs_mag_hdg / mag_test_ratio / heading / EKF 位置。"""
import sys
import numpy as np
from pyulog import ULog

u = ULog(sys.argv[1])
names = {d.name: d for d in u.data_list}

def series(name, col):
    d = names[name].data
    return np.asarray(d["timestamp"]) / 1e6, np.asarray(d[col], dtype=float)

# cs_ev_yaw / cs_mag_hdg 的置位区间 (estimator_status_flags, 约 1Hz)
f = names["estimator_status_flags"].data
t_f = np.asarray(f["timestamp"]) / 1e6
print("== estimator_status_flags 采样时段: %.1f .. %.1f s (共 %d)" % (t_f[0], t_f[-1], len(t_f)))
for col in ("cs_ev_yaw", "cs_mag_hdg", "cs_ev_pos", "cs_gps", "cs_yaw_align"):
    v = np.asarray(f[col], dtype=float)
    on = t_f[v > 0.5]
    print(f"   {col:14s} 置位 {len(on)}/{len(t_f)}  首次={on[0]:.1f}s 末次={on[-1]:.1f}s" if len(on) else
          f"   {col:14s} 从未置位")

# mag_test_ratio (estimator_status)
t_s, mtr = series("estimator_status", "mag_test_ratio")
mtr = np.nan_to_num(mtr, nan=-1.0, posinf=1e9, neginf=-1.0)
bad = t_s[(mtr > 0.5) & (mtr < 1e8)]
if len(bad):
    print(f"== mag_test_ratio > COM_ARM_EKF_YAW(0.5): {len(bad)} 个采样, 时段 {bad[0]-t_s[0]:.1f} .. {bad[-1]-t_s[0]:.1f} s (相对开机)")
else:
    print("== mag_test_ratio 从未 > 0.5")
print("   mag_test_ratio: 有限最大值 = %.4f" % mtr[mtr < 1e8].max())

# heading 时间线
t_l, hd = series("vehicle_local_position", "heading")
hd = np.degrees(hd)
idx = np.linspace(0, len(hd) - 1, 8).astype(int)
print("== vehicle_local_position.heading (deg) ==")
for i in idx:
    print(f"   t={t_l[i]-t_l[0]:7.1f}s  heading={hd[i]:8.2f}  x={names['vehicle_local_position'].data['x'][i]:7.3f}"
          f" y={names['vehicle_local_position'].data['y'][i]:7.3f} z={names['vehicle_local_position'].data['z'][i]:7.3f}")

# EV 位置/偏航原始量
for nm, col in (("estimator_aid_src_ev_pos", "observation"), ("estimator_aid_src_ev_yaw", "observation"),
                ("estimator_aid_src_ev_pos", "innovation"), ("estimator_aid_src_ev_yaw", "innovation")):
    if nm in names and col in names[nm].data:
        tt, vv = series(nm, col)
        vv = np.asarray(vv)
        print(f"== {nm}.{col}: shape={vv.shape} 首={vv[0]} 末={vv[-1]}")
