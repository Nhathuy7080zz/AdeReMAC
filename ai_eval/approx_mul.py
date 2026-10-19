"""Mô hình bit-chính-xác của adaremac_mul (RTL) dưới dạng bảng tra 256x256.

Phép nhân không dấu 8x8, các đường chéo thấp:
    D_i = tổng các a_j & b_(i-j), j = 0..i          (i = 0 .. k-1)
    PL  = sum 2^i * D_i          (phần thấp, chính xác)
    ML  = sum D_i ;  PL^ = q * ML (đường chính xấp xỉ)
    R   = PL - PL^  = sum (2^i - q) * D_i  (phần dư)
Cấu hình RTL đã kiểm chứng: k = 3, q = 2  ->  R = 2*D2 - D0.
Khi residual TẮT:  tích = a*b - R.   Khi BẬT: tích = a*b.
    mode 0     : residual luôn tắt
    mode 1     : residual bật khi D_(k-1) >= thr
    mode 2 / 3 : residual luôn bật (chính xác tuyệt đối)
Số có dấu: dấu-độ lớn (sign-magnitude) như adaremac_mac_lane.v.
k != 3 là phần KHÁM PHÁ thêm, chưa có RTL tương ứng.
"""
import numpy as np

# tên cấu hình: (mode, thr)
CFGS = {
    "mode0":   (0, 0),
    "m1_thr3": (1, 3),
    "m1_thr2": (1, 2),
    "m1_thr1": (1, 1),
    "exact":   (2, 0),
}


def _unsigned_tables(mode, thr, k=3, q=2):
    """Trả về (tích xấp xỉ, residual-bật) cho mọi cặp (a, b) không dấu 0..255."""
    a = np.arange(256, dtype=np.int64)[:, None]
    b = np.arange(256, dtype=np.int64)[None, :]
    bit = lambda v, i: (v >> i) & 1
    D = [sum(bit(a, j) & bit(b, i - j) for j in range(i + 1)) for i in range(k)]
    exact = a * b
    r = sum(((1 << i) - q) * D[i] for i in range(k))
    if mode >= 2:
        enr = np.ones_like(exact)
    elif mode == 0:
        enr = np.zeros_like(exact)
    else:
        enr = (D[k - 1] >= thr).astype(np.int64)
    return exact - np.where(enr == 1, 0, r), enr


def signed_lut(mode, thr, k=3, q=2):
    """Bảng tra có dấu, chỉ số (x+128, w+128), x/w trong -128..127.

    Trả về (slut int32 [256,256], enr uint8 [256,256]).
    """
    p, enr = _unsigned_tables(mode, thr, k, q)
    v = np.arange(-128, 128)
    mag = np.abs(v)
    neg = (v[:, None] < 0) ^ (v[None, :] < 0)
    pm = p[mag[:, None], mag[None, :]]
    em = enr[mag[:, None], mag[None, :]]
    return np.where(neg, -pm, pm).astype(np.int32), em.astype(np.uint8)


def exact_lut():
    v = np.arange(-128, 128, dtype=np.int32)
    return (v[:, None] * v[None, :]).astype(np.int32)


if __name__ == "__main__":
    # ví dụ tay: 5 x 3  (cùng ví dụ trong tài liệu): mode 0 -> 14, mode 2 -> 15
    for name, (m, t) in CFGS.items():
        s, _ = signed_lut(m, t)
        print(f"{name:8s} 5*3 = {s[5 + 128, 3 + 128]}   -5*3 = {s[-5 + 128, 3 + 128]}")
    assert (signed_lut(2, 0)[0] == exact_lut()).all()
    print("mode 2 == tích chính xác: OK")
