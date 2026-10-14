#!/usr/bin/env python3
"""Dump EKF2 fusion flags + test ratios from a PX4 .ulg.

用法: /usr/bin/python3 /tmp/ekf_dump.py <file.ulg>
"""
import sys
from pyulog import ULog

path = sys.argv[1]
u = ULog(path)
names = {d.name: d for d in u.data_list}
print("== 包含的 EKF 相关 topic ==")
for n in sorted(names):
    if "estimator" in n or n in ("vehicle_attitude", "vehicle_local_position", "vehicle_gps_position"):
        print("  ", n, len(names[n].data["timestamp"]))

# ---- estimator_status (老 message, 含 test ratio) ----
if "estimator_status" in names:
    d = names["estimator_status"].data
    ts = d["timestamp"]
    cols = [c for c in d if "test_ratio" in c or c in
            ("control_mode_flags", "filter_fault_flags")]
    print("\n== estimator_status (最后 5 个采样) ==")
    print("  样本数:", len(ts))
    for c in cols:
        vals = d[c]
        print(f"   {c:24s} last={vals[-1]}  max={max(vals) if len(vals) else 'NA'}")

# ---- estimator_status_flags (新 message, 位标志) ----
if "estimator_status_flags" in names:
    d = names["estimator_status_flags"].data
    keys = [c for c in d if c != "timestamp"]
    n = len(d["timestamp"])
    print(f"\n== estimator_status_flags ({n} 采样): 各 bit 置位占比 ==")
    for c in sorted(keys):
        col = d[c]
        try:
            print(f"   {c:26s} last={int(col[-1])}  置位比例={sum(1 for v in col if v)/max(n,1)*100:.1f}%")
        except TypeError:
            pass

# ---- 位置/姿态 ----
for t in ("vehicle_local_position", "vehicle_attitude"):
    if t in names:
        d = names[t].data
        print(f"\n== {t} ==")
        for c in d:
            if c == "timestamp":
                continue
            try:
                print(f"   {c:20s} last={d[c][-1]}")
            except TypeError:
                pass
