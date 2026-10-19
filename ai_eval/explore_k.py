"""Bước 5 (KHÁM PHÁ, chưa có RTL cho k > 3): xấp xỉ nhiều đường chéo hơn thì độ chính xác tụt ra sao.

k = số đường chéo thấp bị xấp xỉ, q = 2^(k//2) (heuristic chọn hệ số, k = 3 -> q = 2 như RTL).
Chỉ đo Mode 0 (residual luôn tắt = xấp xỉ mạnh nhất).   Chạy: python explore_k.py [--n 10000]
"""
import argparse

import numpy as np

from approx_mul import exact_lut, signed_lut
from mnist_data import load_mnist
from qnet import accuracy, new_stats, predict, quantize_model

ap = argparse.ArgumentParser()
ap.add_argument("--n", type=int, default=10000)
ap.add_argument("--stats-n", type=int, default=500)
ap.add_argument("--model", default="model_fp32.npz")
args = ap.parse_args()

xtr, ytr, xte, yte = load_mnist()
x, y = xte[:args.n], yte[:args.n]
m = quantize_model(dict(np.load(args.model)), xtr[:2000])
EX = exact_lut().astype(np.int64)
base = accuracy(m, x, y, [None] * 3)
print(f"INT8 chính xác: {base * 100:.2f}%   (n={len(x)})")
print(f"{'k':>2s} {'q':>3s} {'acc Mode0 %':>12s} {'Δ vs INT8':>10s} {'NMED đều':>11s} {'NMED dữ liệu':>13s}")
for k in range(3, 9):
    q = 2 ** (k // 2)
    lut = signed_lut(0, 0, k, q)
    acc = accuracy(m, x, y, [lut] * 3)
    st = [new_stats() for _ in range(3)]
    predict(m, x[:args.stats_n], [lut] * 3, stats=st)
    nmed_d = sum(s["abs_err"] for s in st) / sum(s["n_mac"] for s in st) / 16384
    nmed_u = np.abs(lut[0].astype(np.int64) - EX).mean() / 16384
    print(f"{k:2d} {q:3d} {acc * 100:12.2f} {(acc - base) * 100:+10.2f} {nmed_u:11.3e} {nmed_d:13.3e}")
