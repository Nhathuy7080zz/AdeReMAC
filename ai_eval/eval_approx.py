"""Bước 2-4: PTQ INT8, rồi đo độ chính xác khi thay bộ nhân bằng AdaReMAC (bit-chính-xác).

Chạy:  python eval_approx.py [--n 10000] [--stats-n 500]
In ra: (a) độ chính xác FP32 / INT8 chính xác / từng cấu hình AdaReMAC cho cả 3 lớp,
       (b) độ nhạy từng lớp (chỉ một lớp dùng Mode 0, các lớp khác chính xác),
       (c) thống kê trên dữ liệu thật: % MAC bật residual, NMED, % phép nhân có toán hạng 0.
Kết quả cũng ghi vào results_k3.csv.
"""
import argparse
import csv

import numpy as np

from approx_mul import CFGS, exact_lut, signed_lut
from mnist_data import load_mnist
from qnet import (accuracy, float_forward, forward_int, new_stats, predict,
                  quantize_input, quantize_model)

ap = argparse.ArgumentParser()
ap.add_argument("--n", type=int, default=10000, help="số ảnh test để đo độ chính xác")
ap.add_argument("--stats-n", type=int, default=500, help="số ảnh để thu thống kê MAC")
ap.add_argument("--model", default="model_fp32.npz")
args = ap.parse_args()

xtr, ytr, xte, yte = load_mnist()
x, y = xte[:args.n], yte[:args.n]
p = dict(np.load(args.model))
m = quantize_model(p, xtr[:2000])

fp = float((float_forward(p, (x[:, None] / 255.0).astype(np.float32))[0].argmax(1) == y).mean())
LUT = {n: signed_lut(*c) for n, c in CFGS.items()}
EX = exact_lut().astype(np.int64)
NAMES = ("conv1", "conv2", "fc")

rows = []
print(f"FP32                       : {fp * 100:.2f}%   (n={len(x)})")
acc_exact = accuracy(m, x, y, [None] * 3)
print(f"INT8 chính xác (ma trận)   : {acc_exact * 100:.2f}%")
rows.append(["fp32", "", fp])
rows.append(["int8_exact", "", acc_exact])

# kiểm tra động cơ tra bảng: LUT chính xác phải trùng bit với phép nhân ma trận
xq = quantize_input(x[:50])[:, None]
ref = forward_int(m, xq, [None] * 3)
lut_exact = (exact_lut(), np.ones((256, 256), np.uint8))
assert (forward_int(m, xq, [lut_exact] * 3) == ref).all(), "động cơ tra bảng sai"
assert (forward_int(m, xq, [LUT["exact"]] * 3) == ref).all(), "mode 2 không trùng bit với chính xác"
print("kiểm tra: tra-bảng-chính-xác và Mode 2 trùng bit với INT8 chính xác  -> OK\n")

print(f"{'cấu hình (cả 3 lớp)':24s} {'acc %':>7s} {'Δ vs INT8':>10s}   "
      f"NMED đều(65536 cặp)")
for name, lut in LUT.items():
    acc = accuracy(m, x, y, [lut] * 3)
    nmed_u = np.abs(lut[0].astype(np.int64) - EX).mean() / 16384
    print(f"{name:24s} {acc * 100:7.2f} {(acc - acc_exact) * 100:+10.2f}   {nmed_u:.3e}")
    rows.append(["all_layers", name, acc])

print("\nĐộ nhạy từng lớp (một lớp = mode0, lớp khác chính xác):")
for i, nm in enumerate(NAMES):
    luts = [None] * 3
    luts[i] = LUT["mode0"]
    acc = accuracy(m, x, y, luts)
    print(f"  {nm:6s}: {acc * 100:.2f}%   ({(acc - acc_exact) * 100:+.2f} so với INT8)")
    rows.append(["sensitivity_mode0", nm, acc])

print(f"\nThống kê MAC trên dữ liệu thật ({args.stats_n} ảnh, cả 3 lớp cùng cấu hình):")
print(f"{'cấu hình':10s} {'lớp':6s} {'số MAC':>12s} {'% bật residual':>15s} {'NMED dữ liệu':>13s} {'% MAC có toán hạng 0':>21s}")
for name, lut in LUT.items():
    st = [new_stats() for _ in range(3)]
    predict(m, x[:args.stats_n], [lut] * 3, stats=st)
    for nm, s in zip(NAMES, st):
        print(f"{name:10s} {nm:6s} {s['n_mac']:12d} {100 * s['enr'] / s['n_mac']:15.2f} "
              f"{s['abs_err'] / s['n_mac'] / 16384:13.3e} {100 * s['zero'] / s['n_mac']:21.2f}")
        rows.append(["mac_stats", f"{name}/{nm}", s["enr"] / s["n_mac"], s["abs_err"] / s["n_mac"] / 16384,
                     s["zero"] / s["n_mac"]])

with open("results_k3.csv", "w", newline="") as f:
    csv.writer(f).writerows([["loai", "ten", "gia_tri", "nmed", "ti_le_toan_hang_0"]] + rows)
print("\nĐã ghi results_k3.csv")
