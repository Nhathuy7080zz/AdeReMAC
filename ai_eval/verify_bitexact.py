"""So bảng tra Python với RTL thật (iverilog) trên toàn bộ 65.536 cặp INT8 có dấu.

Chạy:  python verify_bitexact.py [--rtl-dir ../rtl]
Cần iverilog trong PATH. Kết quả mong đợi: mọi cấu hình đều PASS (0 sai lệch).
"""
import argparse
import os
import subprocess
import sys
import tempfile

import numpy as np

from approx_mul import CFGS, signed_lut

ap = argparse.ArgumentParser()
ap.add_argument("--rtl-dir", default="../rtl")
args = ap.parse_args()
here = os.path.dirname(os.path.abspath(__file__))
rtl = [os.path.join(args.rtl_dir, f) for f in ("adaremac_mul.v", "adaremac_mac_lane.v")]

tmp = tempfile.mkdtemp()
vvp = os.path.join(tmp, "tb.vvp")
subprocess.run(["iverilog", "-g2005", "-o", vvp, os.path.join(here, "tb_signed_dump.v")] + rtl,
               check=True)

ok_all = True
for name, (mode, thr) in CFGS.items():
    out = os.path.join(tmp, f"{name}.txt")
    subprocess.run(["vvp", vvp, f"+mode={mode}", f"+thr={thr}", f"+out={out}"],
                   check=True, stdout=subprocess.DEVNULL)
    got = np.loadtxt(out, dtype=np.int64)
    slut, enr = signed_lut(mode, thr)
    exp_p = slut.reshape(-1).astype(np.int64)          # thứ tự a ngoài, b trong
    exp_e = enr.reshape(-1).astype(np.int64)
    n_bad_p = int((got[:, 0] != exp_p).sum()) if len(got) == 65536 else -1
    n_bad_e = int((got[:, 1] != exp_e).sum()) if len(got) == 65536 else -1
    ok = len(got) == 65536 and n_bad_p == 0 and n_bad_e == 0
    ok_all &= ok
    print(f"{name:8s} mode={mode} thr={thr}  số phép={len(got):6d}  "
          f"sai tích={n_bad_p}  sai ENR={n_bad_e}  {'PASS' if ok else 'FAIL'}")
print("TỔNG:", "PASS" if ok_all else "FAIL")
sys.exit(0 if ok_all else 1)
