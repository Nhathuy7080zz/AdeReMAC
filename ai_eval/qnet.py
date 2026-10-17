"""Lượng tử hóa INT8 (PTQ) + suy luận số nguyên với bộ nhân tra bảng (NumPy thuần).

Mạng:  conv1(1->8, 3x3) -> ReLU -> maxpool2 -> conv2(8->16, 3x3) -> ReLU -> maxpool2
       -> flatten(16*5*5=400) -> fc(400->10)
Quy ước lượng tử hóa (đối xứng, zero-point = 0, khớp phần cứng dấu-độ lớn):
    input : x_q = round(pixel/255 * 127)             (scale 1/127)
    trọng số: w_q = round(w / s_w), s_w = max|w|/127  (từng tensor)
    kích hoạt: s_a = percentile99.99(|a|)/127, a_q = clip(round(a/s_a), 0, 127)
    bias  : b_q = round(b / (s_x * s_w))  (int32, cộng vào bộ cộng dồn)
    requant: out = clip((acc*M0 + 2^(shift-1)) >> shift, 0, 127), M ~= M0 * 2^-shift
Lớp cuối giữ nguyên acc int32 làm logit (argmax), không requant.
"""
import math

import numpy as np

from approx_mul import exact_lut

S_IN = 1.0 / 127.0


# ---------------------------------------------------------------- tiện ích ảnh
def im2col(x, k=3):
    """x [B,C,H,W] -> [B, Ho*Wo, C*k*k]; thứ tự K = (C, kh, kw) khớp w.reshape(O, -1)."""
    B, C, H, W = x.shape
    Ho, Wo = H - k + 1, W - k + 1
    s = x.strides
    win = np.lib.stride_tricks.as_strided(
        x, (B, Ho, Wo, C, k, k), (s[0], s[2], s[3], s[1], s[2], s[3]))
    return win.reshape(B, Ho * Wo, C * k * k), Ho, Wo


def maxpool2(x):
    B, C, H, W = x.shape
    h, w = H // 2, W // 2
    return x[:, :, :h * 2, :w * 2].reshape(B, C, h, 2, w, 2).max(axis=(3, 5))


def _to_map(y, B, Ho, Wo):          # [B, L, O] -> [B, O, Ho, Wo]
    return y.reshape(B, Ho, Wo, -1).transpose(0, 3, 1, 2)


# ---------------------------------------------------------------- FP32 (hiệu chỉnh)
def float_forward(p, x01):
    """x01 [B,1,28,28] float32 trong [0,1]. Trả (logits, act1, act2); act = sau ReLU, trước pool."""
    B = x01.shape[0]
    cols, Ho, Wo = im2col(x01)
    a1 = np.maximum(cols @ p["conv1_w"].reshape(8, -1).T + p["conv1_b"], 0)
    a1 = _to_map(a1, B, Ho, Wo)
    cols, Ho, Wo = im2col(np.ascontiguousarray(maxpool2(a1)))
    a2 = np.maximum(cols @ p["conv2_w"].reshape(16, -1).T + p["conv2_b"], 0)
    a2 = _to_map(a2, B, Ho, Wo)
    h = maxpool2(a2).reshape(B, -1)
    return h @ p["fc_w"].T + p["fc_b"], a1, a2


# ---------------------------------------------------------------- PTQ
def quantize_input(x_u8):
    return ((x_u8.astype(np.int32) * 127 + 127) // 255).astype(np.int8)


def _sym_scale(x, pct=100.0):
    m = np.abs(x).max() if pct >= 100 else np.percentile(np.abs(x), pct)
    return float(m) / 127.0


def _requant_params(M):
    mant, e = math.frexp(M)                      # M = mant * 2^e, mant in [0.5, 1)
    return int(round(mant * 2 ** 31)), 31 - e    # M ~= M0 * 2^-shift


def quantize_model(p, x_calib_u8):
    """Trả về dict số nguyên của mô hình; hiệu chỉnh kích hoạt trên x_calib_u8 [N,28,28]."""
    _, a1, a2 = float_forward(p, (x_calib_u8[:, None] / 255.0).astype(np.float32))
    s_act = [_sym_scale(a1, 99.99), _sym_scale(a2, 99.99)]
    s_in = [S_IN, s_act[0], s_act[1]]
    m = {}
    for i, name in enumerate(("conv1", "conv2", "fc")):
        w = p[name + "_w"].reshape(p[name + "_w"].shape[0], -1)
        sw = _sym_scale(w)
        m[name + "_w"] = np.clip(np.round(w / sw), -127, 127).astype(np.int8)
        m[name + "_b"] = np.round(p[name + "_b"] / (s_in[i] * sw)).astype(np.int32)
        if name != "fc":
            m[name + "_M0"], m[name + "_shift"] = _requant_params(s_in[i] * sw / s_act[i])
    return m


# ---------------------------------------------------------------- MAC tra bảng
_EXACT_FLAT = exact_lut().reshape(-1)


def mac(xc, w, lut, stats=None):
    """xc [B,L,K] int8, w [O,K] int8 -> acc [B,L,O] int32.

    lut = None            : tích chính xác (ma trận số nguyên)
    lut = (slut, enr)     : mỗi phép nhân tra bảng của bộ nhân AdaReMAC
    stats (dict, tùy chọn): cộng dồn n_mac, enr, abs_err, zero (phép có toán hạng 0)
    """
    if lut is None:
        return xc.astype(np.int32) @ w.astype(np.int32).T
    xi = xc.astype(np.int32) + 128
    wi = w.astype(np.int32) + 128
    idx = xi[:, :, None, :] * 256 + wi[None, None, :, :]
    prod = lut[0].reshape(-1).take(idx)
    if stats is not None:
        stats["n_mac"] += idx.size
        stats["enr"] += int(lut[1].reshape(-1).take(idx).sum())
        stats["abs_err"] += int(np.abs(prod - _EXACT_FLAT.take(idx)).sum())
        stats["zero"] += int(((xi == 128)[:, :, None, :] | (wi == 128)[None, None, :, :]).sum())
    return prod.sum(axis=3, dtype=np.int32)


def _requant(acc, M0, shift):
    return (acc.astype(np.int64) * M0 + (1 << (shift - 1))) >> shift


def new_stats():
    return {"n_mac": 0, "enr": 0, "abs_err": 0, "zero": 0}


def forward_int(m, x_q, luts, stats=None):
    """x_q int8 [B,1,28,28]; luts = [lut_conv1, lut_conv2, lut_fc]; stats = [dict]*3 hoặc None."""
    st = stats or [None] * 3
    B = x_q.shape[0]
    h = x_q
    for i, name in enumerate(("conv1", "conv2")):
        cols, Ho, Wo = im2col(np.ascontiguousarray(h))
        acc = mac(cols, m[name + "_w"], luts[i], st[i]) + m[name + "_b"]
        y = np.clip(_requant(acc, m[name + "_M0"], m[name + "_shift"]), 0, 127).astype(np.int8)
        h = maxpool2(_to_map(y, B, Ho, Wo))
    acc = mac(h.reshape(B, 1, -1), m["fc_w"], luts[2], st[2])[:, 0, :] + m["fc_b"]
    return acc


def predict(m, x_u8, luts, bs=100, stats=None):
    xq = quantize_input(x_u8)[:, None]
    out = [forward_int(m, xq[i:i + bs], luts, stats).argmax(1) for i in range(0, len(xq), bs)]
    return np.concatenate(out)


def accuracy(m, x_u8, y, luts, bs=100):
    return float((predict(m, x_u8, luts, bs) == y).mean())
