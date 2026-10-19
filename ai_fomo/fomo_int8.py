"""
Lượng tử hóa INT8 + suy luận SỐ NGUYÊN bit-exact cho FOMO-Micro.
Đây là ĐẶC TẢ CHUNG mà cả 3 bản cài đặt phải khớp từng bit:
  - Python (file này)                    -> mô hình tham chiếu
  - C (firmware ESP32-S3: fomo_micro.c)  -> chạy trên CPU
  - RTL (rtl/fomo_accel*.v)              -> chạy trên Tang Nano 20K (có bộ nhân AdaReMAC)

Đặc tả số học (mỗi lớp conv k x k, stride s, padding k//2, phần tử ngoài biên = 0):
  x  : uint8 (lớp đầu: điểm ảnh xám 0..255; các lớp sau: kích hoạt sau ReLU)
  w  : int8 đối xứng trong [-127, 127] (không dùng -128), theo từng kênh ra
  acc[co] = B[co] + sum x * w            (int32; tích x*w do bộ nhân AdaReMAC tính:
                                          p = sign(w) * adaremac_mul(x, |w|, mode, thr))
  lớp ReLU : y = clamp((acc * M[co] + 2^(SH-1)) >> SH, 0, 255)     (>> là dịch số học)
  lớp cuối : y = clamp((acc * M[co] + 2^(SH-1)) >> SH, -128, 127)  (logit int8 của "person")
  M[co] : uint16 (1..65535), SH : 1..31, cố định theo lớp.
"""
import numpy as np
import torch

from fomo_models import FomoMicro


# ----------------------------------------------------------------------------------------
# Bộ nhân AdaReMAC (khớp rtl/adaremac_mul.v): k = 3 đường chéo thấp, q = 2, R = 2*D2 - D0
# Sai số chỉ phụ thuộc 3 bit thấp của hai toán hạng -> bảng 8x8.
# ----------------------------------------------------------------------------------------
def adaremac_mul_u8(a, b, mode, thr):
    a = np.asarray(a, np.int64); b = np.asarray(b, np.int64)
    bit = lambda v, i: (v >> i) & 1
    d0 = bit(a, 0) & bit(b, 0)
    d2 = (bit(a, 0) & bit(b, 2)) + (bit(a, 1) & bit(b, 1)) + (bit(a, 2) & bit(b, 0))
    r = 2 * d2 - d0
    if mode >= 2:
        enr = np.ones_like(r)
    elif mode == 0:
        enr = np.zeros_like(r)
    else:
        enr = (d2 >= thr).astype(np.int64)
    return a * b - np.where(enr == 1, 0, r)


def err_table(mode, thr):
    """E[u][v] = adaremac(x, a) - x*a với u = x & 7, v = a & 7 (kiểm tra lại trên toàn dải 0..255)."""
    x = np.arange(256)[:, None]; a = np.arange(128)[None, :]
    full = adaremac_mul_u8(x, a, mode, thr) - x * a
    E = full[:8, :8].copy()
    assert (full == E[x & 7, a & 7]).all()
    return E.astype(np.int64)


# ----------------------------------------------------------------------------------------
# Lượng tử hóa sau huấn luyện (PTQ) từ checkpoint PyTorch
# ----------------------------------------------------------------------------------------
def fold_bn(model):
    """Trả về danh sách lớp float: dict(w[co,ci,k,k], b[co], k, s, relu)."""
    layers = []
    mods = list(model.net)
    i = 0
    for (ci, co, k, s) in model.cfg:
        conv = mods[i]; i += 1
        w = conv.weight.detach().double().numpy()
        b = conv.bias.detach().double().numpy() if conv.bias is not None else np.zeros(co)
        relu = False
        if i < len(mods) and isinstance(mods[i], torch.nn.BatchNorm2d):
            bn = mods[i]; i += 1
            g = bn.weight.detach().double().numpy() / np.sqrt(bn.running_var.detach().double().numpy() + bn.eps)
            w = w * g[:, None, None, None]
            b = (b - bn.running_mean.detach().double().numpy()) * g + bn.bias.detach().double().numpy()
        if i < len(mods) and isinstance(mods[i], torch.nn.ReLU):
            relu = True; i += 1
        layers.append(dict(w=w, b=b, k=k, s=s, relu=relu))
    return layers


def conv_float(x, L):
    t = torch.nn.functional.conv2d(torch.from_numpy(x), torch.from_numpy(L["w"]), torch.from_numpy(L["b"]),
                                   stride=L["s"], padding=L["k"] // 2).numpy()
    return np.maximum(t, 0) if L["relu"] else t


def quantize(model, calib_u8, pct=99.99):
    """calib_u8: [N,96,96] uint8. Trả về danh sách lớp INT8 theo đặc tả ở đầu file."""
    fl = fold_bn(model)
    x = calib_u8[:, None].astype(np.float64) / 255.0
    s_in = 1.0 / 255.0
    q = []
    for li, L in enumerate(fl):
        y = conv_float(x, L)
        last = li == len(fl) - 1
        s_out = (np.percentile(np.abs(y), pct) / (127.0 if last else 255.0)) or 1e-6
        sw = np.abs(L["w"]).reshape(L["w"].shape[0], -1).max(1) / 127.0
        sw = np.where(sw == 0, 1e-8, sw)
        wq = np.clip(np.round(L["w"] / sw[:, None, None, None]), -127, 127).astype(np.int8)
        bq = np.round(L["b"] / (s_in * sw)).astype(np.int64)
        m = s_in * sw / s_out
        sh = int(min(31, np.floor(np.log2(65535.0 / m.max()))))
        M = np.clip(np.round(m * 2.0 ** sh), 1, 65535).astype(np.int64)
        q.append(dict(w=wq, b=bq, M=M, sh=sh, k=L["k"], s=L["s"], relu=L["relu"],
                      s_in=s_in, s_out=s_out))
        # lớp tiếp theo nhận kích hoạt đã lượng tử hóa (mô phỏng sai số lượng tử tích lũy)
        x = np.clip(np.round(y / s_out), 0, 255) * s_out if not last else y
        s_in = s_out
    return q


# ----------------------------------------------------------------------------------------
# Suy luận số nguyên (vector hóa) — tham chiếu bit-exact
# ----------------------------------------------------------------------------------------
def _im2col(x, k, s):
    """x int64 [B,C,H,W] -> cols [B, Ho*Wo, C*k*k] (thứ tự C, ky, kx)."""
    B, C, H, W = x.shape
    p = k // 2
    xp = np.pad(x, ((0, 0), (0, 0), (p, p), (p, p)))
    Ho, Wo = (H + 2 * p - k) // s + 1, (W + 2 * p - k) // s + 1
    st = xp.strides
    win = np.lib.stride_tricks.as_strided(xp, (B, Ho, Wo, C, k, k),
                                          (st[0], st[2] * s, st[3] * s, st[1], st[2], st[3]))
    return win.reshape(B, Ho * Wo, C * k * k), Ho, Wo


def layer_int(x, L, mode=2, thr=2, bias_comp=None):
    """x int64 [B,C,H,W] (0..255) -> y int64 [B,Co,Ho,Wo]. mode: 0/1/2 (AdaReMAC)."""
    B = x.shape[0]
    cols, Ho, Wo = _im2col(x, L["k"], L["s"])
    W = L["w"].reshape(L["w"].shape[0], -1).astype(np.int64)        # [Co, K]
    acc = cols @ W.T                                                  # tích chính xác
    if mode != 2:
        E = err_table(mode, thr)
        sgn, mag = np.sign(W), np.abs(W) & 7
        xl = cols & 7
        for u in range(8):                                            # cộng sai số AdaReMAC
            Eu = (E[u][mag] * sgn).T                                  # [K, Co]
            if Eu.any():
                acc += (xl == u).astype(np.int64) @ Eu
    acc += L["b"][None, None, :] if bias_comp is None else (L["b"] + bias_comp)[None, None, :]
    y = (acc * L["M"][None, None, :] + (1 << (L["sh"] - 1))) >> L["sh"]
    y = np.clip(y, 0, 255) if L["relu"] else np.clip(y, -128, 127)
    return y.reshape(B, Ho, Wo, -1).transpose(0, 3, 1, 2)


def forward_int(q, imgs_u8, modes=None, thr=2, comps=None, return_all=False, chunk=128):
    """imgs_u8 [B,96,96] -> logit int8 [B,12,12]. modes: list mode cho từng lớp (mặc định toàn 2)."""
    if len(imgs_u8) > chunk and not return_all:
        return np.concatenate([forward_int(q, imgs_u8[i:i + chunk], modes, thr, comps, chunk=chunk)
                               for i in range(0, len(imgs_u8), chunk)])
    x = imgs_u8[:, None].astype(np.int64)
    outs = []
    for i, L in enumerate(q):
        m = 2 if modes is None else modes[i]
        x = layer_int(x, L, m, thr, None if comps is None else comps[i])
        outs.append(x)
    return outs if return_all else x[:, 0]


def logit_to_prob(q, logit_i8):
    return 1.0 / (1.0 + np.exp(-logit_i8 * q[-1]["s_out"]))


def mode0_bias_compensation(q, calib_u8, thr=2):
    """Bù độ lệch hệ thống của Mode 0: E[sai số tích] theo từng kênh ra, đo trên dữ liệu hiệu chỉnh,
    trừ vào bias (không tốn phần cứng). Trả về list mảng int64 (cộng thêm vào B)."""
    comps = []
    x = calib_u8[:, None].astype(np.int64)
    E = err_table(0, thr)
    for L in q:
        W = L["w"].reshape(L["w"].shape[0], -1).astype(np.int64)
        sgn, mag = np.sign(W), np.abs(W) & 7
        e = np.zeros(W.shape[0]); n = 0
        for c0 in range(0, len(x), 64):
            cols, _, _ = _im2col(x[c0:c0 + 64], L["k"], L["s"])
            xl = cols & 7
            for u in range(8):
                Eu = (E[u][mag] * sgn).T
                if Eu.any():
                    e += ((xl == u).astype(np.int64) @ Eu).sum(axis=(0, 1))
            n += cols.shape[0] * cols.shape[1]
        comps.append(-np.round(e / n).astype(np.int64))
        x = np.concatenate([layer_int(x[c0:c0 + 64], L, 2) for c0 in range(0, len(x), 64)])
    return comps
