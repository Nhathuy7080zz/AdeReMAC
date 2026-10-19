"""Bước 1 (bản NumPy thuần, không cần PyTorch): huấn luyện đúng CNN trong qnet.py, lưu model_fp32.npz.

Backprop viết tay (im2col + ma trận). Dùng khi không có PyTorch/GPU; kết quả tương đương train_mnist.py.
Chạy:  python train_mnist_numpy.py [--epochs 6]
"""
import argparse
import time

import numpy as np

from mnist_data import load_mnist
from qnet import float_forward, im2col

ap = argparse.ArgumentParser()
ap.add_argument("--epochs", type=int, default=6)
ap.add_argument("--max-iters", type=int, default=0, help="(đo tốc độ) dừng sau N bước")
args = ap.parse_args()
BATCH = 128
rng = np.random.default_rng(0)


def he(shape, fan_in):
    return (rng.standard_normal(shape) * np.sqrt(2.0 / fan_in)).astype(np.float32)


P = {"conv1_w": he((8, 1, 3, 3), 9), "conv1_b": np.zeros(8, np.float32),
     "conv2_w": he((16, 8, 3, 3), 72), "conv2_b": np.zeros(16, np.float32),
     "fc_w": he((10, 400), 400), "fc_b": np.zeros(10, np.float32)}


def pool_fwd(x):
    B, C, H, W = x.shape
    h, w = H // 2, W // 2
    xr = x[:, :, :h * 2, :w * 2].reshape(B, C, h, 2, w, 2)
    out = xr.max(axis=(3, 5))
    return out, (xr, out, x.shape)


def pool_bwd(dout, cache):
    xr, out, shp = cache
    mask = (xr == out[:, :, :, None, :, None]).astype(np.float32)
    mask /= mask.sum(axis=(3, 5), keepdims=True)
    dxr = mask * dout[:, :, :, None, :, None]
    B, C, H, W = shp
    dx = np.zeros(shp, np.float32)
    dx[:, :, :H // 2 * 2, :W // 2 * 2] = dxr.reshape(B, C, H // 2 * 2, W // 2 * 2)
    return dx


def col2im(dcols, shp, k=3):
    B, C, H, W = shp
    Ho, Wo = H - k + 1, W - k + 1
    d = dcols.reshape(B, Ho, Wo, C, k, k)
    dx = np.zeros(shp, np.float32)
    for i in range(k):
        for j in range(k):
            dx[:, :, i:i + Ho, j:j + Wo] += d[:, :, :, :, i, j].transpose(0, 3, 1, 2)
    return dx


def step(x, y):
    B = len(x)
    W1, W2, Wf = P["conv1_w"].reshape(8, -1), P["conv2_w"].reshape(16, -1), P["fc_w"]
    cols1, Ho1, Wo1 = im2col(x)
    z1 = cols1 @ W1.T + P["conv1_b"]
    a1 = np.maximum(z1, 0).reshape(B, Ho1, Wo1, 8).transpose(0, 3, 1, 2)
    p1, c1 = pool_fwd(a1)
    cols2, Ho2, Wo2 = im2col(np.ascontiguousarray(p1))
    z2 = cols2 @ W2.T + P["conv2_b"]
    a2 = np.maximum(z2, 0).reshape(B, Ho2, Wo2, 16).transpose(0, 3, 1, 2)
    p2, c2 = pool_fwd(a2)
    h = p2.reshape(B, -1)
    logits = h @ Wf.T + P["fc_b"]
    e = np.exp(logits - logits.max(1, keepdims=True))
    sm = e / e.sum(1, keepdims=True)
    loss = float(-np.log(sm[np.arange(B), y] + 1e-9).mean())
    dl = sm
    dl[np.arange(B), y] -= 1
    dl /= B
    g = {"fc_w": dl.T @ h, "fc_b": dl.sum(0)}
    da2 = pool_bwd((dl @ Wf).reshape(B, 16, 5, 5), c2)
    dz2 = da2.transpose(0, 2, 3, 1).reshape(B, Ho2 * Wo2, 16) * (z2 > 0)
    g["conv2_w"] = (dz2.reshape(-1, 16).T @ cols2.reshape(-1, 72)).reshape(16, 8, 3, 3)
    g["conv2_b"] = dz2.sum((0, 1))
    da1 = pool_bwd(col2im(dz2 @ W2, p1.shape), c1)
    dz1 = da1.transpose(0, 2, 3, 1).reshape(B, Ho1 * Wo1, 8) * (z1 > 0)
    g["conv1_w"] = (dz1.reshape(-1, 8).T @ cols1.reshape(-1, 9)).reshape(8, 1, 3, 3)
    g["conv1_b"] = dz1.sum((0, 1))
    return loss, g


xtr, ytr, xte, yte = load_mnist()
Xtr = (xtr[:, None] / 255.0).astype(np.float32)
Xte = (xte[:, None] / 255.0).astype(np.float32)
mom = {k: np.zeros_like(v) for k, v in P.items()}
vel = {k: np.zeros_like(v) for k, v in P.items()}
t_adam, it = 0, 0
for ep in range(args.epochs):
    lr = 2e-3 * (0.5 ** (ep // 2))
    perm = rng.permutation(len(Xtr))
    t0 = time.time()
    for i in range(0, len(perm), BATCH):
        b = perm[i:i + BATCH]
        loss, g = step(Xtr[b], ytr[b])
        t_adam += 1
        for k in P:
            mom[k] = 0.9 * mom[k] + 0.1 * g[k]
            vel[k] = 0.999 * vel[k] + 0.001 * g[k] ** 2
            mh, vh = mom[k] / (1 - 0.9 ** t_adam), vel[k] / (1 - 0.999 ** t_adam)
            P[k] = (P[k] - lr * mh / (np.sqrt(vh) + 1e-8)).astype(np.float32)
        it += 1
        if args.max_iters and it >= args.max_iters:
            print(f"{it} bước mất {time.time() - t0:.1f}s, loss={loss:.3f}")
            raise SystemExit
    acc = float((float_forward(P, Xte)[0].argmax(1) == yte).mean())
    print(f"epoch {ep + 1}/{args.epochs}  test acc = {acc * 100:.2f}%  ({time.time() - t0:.0f}s)", flush=True)
    np.savez("model_fp32.npz", **P)
print("Đã lưu model_fp32.npz")
