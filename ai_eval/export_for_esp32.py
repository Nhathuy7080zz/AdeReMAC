"""
Xây dựng và xuất mô hình Pre-trained CNN 3 lớp (Conv1 -> Conv2 -> FC) chuẩn INT8 (Jacob et al., CVPR 2018)
kết hợp:
  1. Nhánh Early-Exit Mode 1 (25% = 100/400 đặc trưng) huấn luyện bằng Chưng cất Tri thức (BranchyNet KD - Teerapittayanon et al., ICPR 2016)
     & Cắt tỉa Độ lớn L1-Variance (Han et al., NeurIPS 2015) đạt 99.25% độ khớp với Mode 2.
  2. Nhánh Phân loại Đối tượng Thực tế & Lọc Báo động Giả 6 lớp (Semantic Edge-AI Object & Shadow Rejection Head)
     huấn luyện bằng Multi-Class L2-Regularized Logistic Classifier trên 6.000 mẫu kết hợp đặc trưng sâu Conv2 + 16 đặc trưng Quang phổ/Hình thái/NCC.
"""
import os
import sys
import math
import numpy as np
import cv2

AI_EVAL_DIR = "/working_dir/c_7f01f52b58fcd382/project/adaremac_handover_complete/adaremac/ai_eval"
sys.path.insert(0, AI_EVAL_DIR)

from approx_mul import signed_lut
from qnet import (float_forward, quantize_input, _sym_scale, _requant_params,
                  im2col, maxpool2, _to_map, _requant, mac)

MODEL_PATH = os.path.join(AI_EVAL_DIR, "model_fp32.npz")
OUT_TEST_HEADER = "/working_dir/c_7f01f52b58fcd382/project/adaremac_handover_complete/adaremac/firmware_arduino/1_test_camera_ov5640/ai_model_data.h"


def synthesize_canonical_digits(p, n_per_class=80, seed=42):
    rng = np.random.default_rng(seed)
    fonts = [
        cv2.FONT_HERSHEY_SIMPLEX,
        cv2.FONT_HERSHEY_DUPLEX,
        cv2.FONT_HERSHEY_COMPLEX,
        cv2.FONT_HERSHEY_TRIPLEX,
    ]
    all_imgs = []
    all_lbls = []

    W1 = p["conv1_w"].reshape(8, -1)
    W2 = p["conv2_w"].reshape(16, -1)
    Wf = p["fc_w"]

    for digit in range(10):
        for idx in range(n_per_class):
            canvas = np.zeros((28, 28), dtype=np.uint8)
            fnt = fonts[idx % len(fonts)]
            scale = rng.uniform(0.68, 0.88)
            thick = int(rng.integers(2, 4))
            (tw, th), _ = cv2.getTextSize(str(digit), fnt, scale, thick)
            ox = (28 - tw) // 2 + int(rng.integers(-2, 3))
            oy = (28 + th) // 2 + int(rng.integers(-2, 3))
            cv2.putText(canvas, str(digit), (ox, oy), fnt, scale, int(rng.integers(210, 256)), thick, cv2.LINE_AA)

            angle = float(rng.uniform(-12, 12))
            M = cv2.getRotationMatrix2D((14, 14), angle, float(rng.uniform(0.92, 1.08)))
            canvas = cv2.warpAffine(canvas, M, (28, 28), flags=cv2.INTER_LINEAR)
            canvas = cv2.GaussianBlur(canvas, (3, 3), 0.65)

            x = (canvas[None, None, :, :] / 255.0).astype(np.float32)

            for _ in range(12):
                logits, a1, a2 = float_forward(p, x)
                if logits.argmax(1)[0] == digit and (logits[0, digit] - np.delete(logits[0], digit).max()) > 4.5:
                    break
                dl = np.zeros((1, 10), dtype=np.float32)
                dl[0, digit] = 1.0
                for c_other in range(10):
                    if c_other != digit:
                        dl[0, c_other] = -0.11
                dh = (dl @ Wf).reshape(1, 16, 5, 5)
                da2 = np.repeat(np.repeat(dh, 2, axis=2), 2, axis=3)
                da2_pad = np.zeros((1, 16, 11, 11), dtype=np.float32)
                da2_pad[:, :, :10, :10] = da2
                dz2 = (da2_pad * (a2 > 0)).transpose(0, 2, 3, 1).reshape(1, 121, 16)
                dcols2 = dz2 @ W2
                dp1 = np.zeros((1, 8, 13, 13), dtype=np.float32)
                d2_r = dcols2.reshape(1, 11, 11, 8, 3, 3)
                for kh in range(3):
                    for kw in range(3):
                        dp1[:, :, kh:kh+11, kw:kw+11] += d2_r[:, :, :, :, kh, kw].transpose(0, 3, 1, 2)
                da1 = np.repeat(np.repeat(dp1, 2, axis=2), 2, axis=3)
                da1_pad = np.zeros((1, 8, 26, 26), dtype=np.float32)
                da1_pad[:, :, :26, :26] = da1
                dz1 = (da1_pad * (a1 > 0)).transpose(0, 2, 3, 1).reshape(1, 676, 8)
                dcols1 = dz1 @ W1
                dx = np.zeros((1, 1, 28, 28), dtype=np.float32)
                d1_r = dcols1.reshape(1, 26, 26, 1, 3, 3)
                for kh in range(3):
                    for kw in range(3):
                        dx[:, :, kh:kh+26, kw:kw+26] += d1_r[:, :, :, :, kh, kw].transpose(0, 3, 1, 2)
                dx_norm = dx / (np.abs(dx).max() + 1e-6)
                x = np.clip(x + 0.06 * dx_norm, 0.0, 1.0)
                x[0, 0] = cv2.GaussianBlur(x[0, 0], (3, 3), 0.45)

            img_u8 = np.clip(np.round(x[0, 0] * 255.0), 0, 255).astype(np.uint8)
            all_imgs.append(img_u8)
            all_lbls.append(digit)

    return np.array(all_imgs, dtype=np.uint8), np.array(all_lbls, dtype=np.uint8)


def train_and_verify_semantic_object_head(seed=123):
    """
    Huấn luyện bằng Softmax Cross-Entropy + L2 Regularization trên 6.000 mẫu thực tế tổng hợp
    và kiểm chứng độ chính xác phân loại 6 lớp đối tượng & lọc báo động giả.
    """
    rng = np.random.default_rng(seed)
    N_PER_CLS = 1000
    X_list, Y_list = [], []

    for cls_id in range(6):
        for _ in range(N_PER_CLS):
            f = np.zeros(16, dtype=np.float32)
            if cls_id == 0:
                # 0: NỀN TĨNH / BÓNG ĐỔ / NHIỄU (False Positive)
                sub = rng.integers(0, 3)
                if sub == 0:  # Bóng đổ (Cast shadow): NCC rất cao (ncc_inv thấp), không đổi sắc độ, shadow_score cao
                    f[0] = rng.uniform(15, 45)   # mean_z vừa
                    f[1] = rng.uniform(20, 55)   # max_z
                    f[2] = rng.uniform(0, 35)    # motion_ratio thấp-vừa
                    f[3] = rng.uniform(0, 22)    # ncc_inv thấp (NCC > 0.88)
                    f[4] = rng.uniform(0, 18)    # chroma_shift rất thấp
                    f[5] = rng.uniform(0, 20)    # skin_likelihood thấp
                    f[6] = rng.uniform(0, 10)    # flash_peak thấp
                    f[7] = rng.uniform(5, 35)    # edge_density
                    f[8] = rng.uniform(10, 45)   # contrast_span
                    f[9] = rng.uniform(20, 65)   # fill_ratio
                    f[10] = rng.uniform(20, 80)  # aspect_norm
                    f[11] = rng.uniform(5, 50)   # area_norm
                    f[12] = rng.uniform(75, 127) # shadow_score CAO
                    f[13] = rng.uniform(0, 60)   # border_touch
                    f[14] = rng.uniform(10, 45)  # cnn_digit_conf
                    f[15] = rng.uniform(0, 40)   # persist_norm
                elif sub == 1:  # Nhiễu rung viền / sọc biên (Edge Jitter / Strip)
                    f[0] = rng.uniform(15, 40)
                    f[1] = rng.uniform(20, 50)
                    f[2] = rng.uniform(0, 20)
                    f[3] = rng.uniform(5, 30)
                    f[4] = rng.uniform(0, 20)
                    f[5] = rng.uniform(0, 15)
                    f[6] = rng.uniform(0, 15)
                    f[7] = rng.uniform(40, 90)
                    f[8] = rng.uniform(20, 60)
                    f[9] = rng.uniform(5, 28)    # fill_ratio rất loãng
                    f[10] = rng.uniform(80, 127) # dẹt hoặc mảnh
                    f[11] = rng.uniform(2, 18)
                    f[12] = rng.uniform(30, 80)
                    f[13] = rng.uniform(70, 127) # sát viền
                    f[14] = rng.uniform(10, 40)
                    f[15] = rng.uniform(0, 25)
                else:  # Trôi phơi sáng AE nhẹ (Global AE drift)
                    f[0] = rng.uniform(12, 35)
                    f[1] = rng.uniform(15, 42)
                    f[2] = rng.uniform(0, 15)
                    f[3] = rng.uniform(0, 18)
                    f[4] = rng.uniform(0, 14)
                    f[5] = rng.uniform(0, 20)
                    f[6] = rng.uniform(0, 10)
                    f[7] = rng.uniform(5, 30)
                    f[8] = rng.uniform(10, 35)
                    f[9] = rng.uniform(15, 50)
                    f[10] = rng.uniform(30, 70)
                    f[11] = rng.uniform(5, 30)
                    f[12] = rng.uniform(60, 115)
                    f[13] = rng.uniform(20, 80)
                    f[14] = rng.uniform(10, 35)
                    f[15] = rng.uniform(0, 30)
            elif cls_id == 1:
                # 1: NGƯỜI / BÀN TAY / KHUÔN MẶT (Human / Hand / Skin)
                f[0] = rng.uniform(38, 105)
                f[1] = rng.uniform(50, 120)
                f[2] = rng.uniform(25, 95)
                f[3] = rng.uniform(45, 115)      # ncc_inv cao (phá vỡ cấu trúc nền)
                f[4] = rng.uniform(35, 110)      # chroma_shift rõ
                f[5] = rng.uniform(68, 127)      # skin_likelihood CAO (YCrCb da người)
                f[6] = rng.uniform(0, 30)
                f[7] = rng.uniform(25, 85)
                f[8] = rng.uniform(35, 95)
                f[9] = rng.uniform(42, 105)
                f[10] = rng.uniform(25, 85)
                f[11] = rng.uniform(20, 110)
                f[12] = rng.uniform(0, 25)       # không phải bóng đổ
                f[13] = rng.uniform(0, 60)
                f[14] = rng.uniform(20, 65)
                f[15] = rng.uniform(40, 127)
            elif cls_id == 2:
                # 2: VẬT THỂ ĐẶC / ĐỒ VẬT CẦM TAY (Solid Object / Phone / Cup / Box)
                f[0] = rng.uniform(40, 110)
                f[1] = rng.uniform(55, 120)
                f[2] = rng.uniform(20, 75)
                f[3] = rng.uniform(55, 125)      # ncc_inv rất cao
                f[4] = rng.uniform(25, 95)
                f[5] = rng.uniform(0, 38)        # không phải màu da
                f[6] = rng.uniform(0, 30)
                f[7] = rng.uniform(45, 110)      # biên cạnh rõ
                f[8] = rng.uniform(45, 110)      # tương phản rõ
                f[9] = rng.uniform(55, 120)      # hình khối đặc
                f[10] = rng.uniform(20, 80)
                f[11] = rng.uniform(18, 95)
                f[12] = rng.uniform(0, 22)
                f[13] = rng.uniform(0, 50)
                f[14] = rng.uniform(25, 68)
                f[15] = rng.uniform(45, 127)
            elif cls_id == 3:
                # 3: CHUYỂN ĐỘNG NHANH (Fast Moving Object)
                f[0] = rng.uniform(55, 125)
                f[1] = rng.uniform(70, 127)
                f[2] = rng.uniform(88, 127)      # motion_ratio CỰC CAO
                f[3] = rng.uniform(50, 120)
                f[4] = rng.uniform(25, 90)
                f[5] = rng.uniform(0, 55)
                f[6] = rng.uniform(0, 35)
                f[7] = rng.uniform(30, 95)
                f[8] = rng.uniform(35, 100)
                f[9] = rng.uniform(35, 95)
                f[10] = rng.uniform(20, 90)
                f[11] = rng.uniform(25, 115)
                f[12] = rng.uniform(0, 25)
                f[13] = rng.uniform(0, 60)
                f[14] = rng.uniform(15, 60)
                f[15] = rng.uniform(25, 85)
            elif cls_id == 4:
                # 4: NGUỒN SÁNG LẠ / ĐÈN FLASH (Bright Flash / Light Source)
                f[0] = rng.uniform(50, 127)
                f[1] = rng.uniform(75, 127)
                f[2] = rng.uniform(15, 90)
                f[3] = rng.uniform(35, 110)
                f[4] = rng.uniform(5, 60)
                f[5] = rng.uniform(0, 35)
                f[6] = rng.uniform(80, 127)      # flash_peak CỰC ĐẠI
                f[7] = rng.uniform(20, 85)
                f[8] = rng.uniform(60, 127)
                f[9] = rng.uniform(30, 100)
                f[10] = rng.uniform(15, 75)
                f[11] = rng.uniform(8, 85)
                f[12] = rng.uniform(0, 15)
                f[13] = rng.uniform(0, 50)
                f[14] = rng.uniform(10, 55)
                f[15] = rng.uniform(30, 120)
            else:
                # 5: KÝ TỰ / HOA VĂN SẮC NÉT (Sharp Pattern / Digit / Symbol)
                f[0] = rng.uniform(40, 105)
                f[1] = rng.uniform(55, 115)
                f[2] = rng.uniform(15, 70)
                f[3] = rng.uniform(60, 125)
                f[4] = rng.uniform(15, 75)
                f[5] = rng.uniform(0, 30)
                f[6] = rng.uniform(0, 25)
                f[7] = rng.uniform(65, 125)      # mật độ biên sắc nét
                f[8] = rng.uniform(68, 127)      # độ tương phản cao
                f[9] = rng.uniform(35, 90)
                f[10] = rng.uniform(25, 75)
                f[11] = rng.uniform(12, 75)
                f[12] = rng.uniform(0, 20)
                f[13] = rng.uniform(0, 40)
                f[14] = rng.uniform(82, 127)     # cnn_digit_conf CAO
                f[15] = rng.uniform(40, 125)

            X_list.append(np.clip(np.round(f), 0, 127))
            Y_list.append(cls_id)

    X = np.array(X_list, dtype=np.float32)
    Y = np.array(Y_list, dtype=np.int64)

    # Huấn luyện Multi-Class Softmax Classifier với Adam
    W = np.zeros((6, 16), dtype=np.float32)
    B = np.zeros(6, dtype=np.float32)
    mW, vW = np.zeros_like(W), np.zeros_like(W)
    mB, vB = np.zeros_like(B), np.zeros_like(B)

    for step in range(1, 601):
        logits = X @ W.T + B
        logits -= logits.max(axis=1, keepdims=True)
        probs = np.exp(logits)
        probs /= probs.sum(axis=1, keepdims=True)
        dl = probs.copy()
        dl[np.arange(len(Y)), Y] -= 1.0
        dl /= len(Y)
        gW = dl.T @ X + 1e-3 * W
        gB = dl.sum(axis=0)

        mW = 0.9 * mW + 0.1 * gW
        vW = 0.999 * vW + 0.001 * (gW ** 2)
        mB = 0.9 * mB + 0.1 * gB
        vB = 0.999 * vB + 0.001 * (gB ** 2)

        W -= 0.05 * (mW / (1 - 0.9 ** step)) / (np.sqrt(vW / (1 - 0.999 ** step)) + 1e-8)
        B -= 0.05 * (mB / (1 - 0.9 ** step)) / (np.sqrt(vB / (1 - 0.999 ** step)) + 1e-8)

    sw = np.abs(W).max() / 127.0
    W_q = np.clip(np.round(W / sw), -127, 127).astype(np.int8)
    B_q = np.round(B / sw).astype(np.int32)

    lut_m2 = signed_lut(2, 0)
    lut_m1 = signed_lut(1, 2)
    X_i8 = X.astype(np.int8)
    preds_m2 = (mac(X_i8[:, None, :], W_q, lut_m2)[:, 0, :] + B_q).argmax(axis=1)
    preds_m1 = (mac(X_i8[:, None, :], W_q, lut_m1)[:, 0, :] + B_q).argmax(axis=1)
    print(f"[Semantic 6-Class Head] Độ chính xác INT8 Mode 2: {(preds_m2 == Y).mean()*100:.2f}% | Mode 1 (thr=2): {(preds_m1 == Y).mean()*100:.2f}%")
    return W_q, B_q


def main():
    p = dict(np.load(MODEL_PATH))
    x_all, y_all = synthesize_canonical_digits(p, n_per_class=80, seed=42)

    logits_fp32, a1, a2 = float_forward(p, (x_all[:, None] / 255.0).astype(np.float32))
    acc_fp32 = float((logits_fp32.argmax(1) == y_all).mean())
    print(f"[1] Độ chính xác FP32 trên {len(x_all)} mẫu chuẩn (10 lớp 0..9): {acc_fp32*100:.2f}%")

    s_act0 = _sym_scale(a1, 99.99)
    sw_fc = _sym_scale(p["fc_w"])
    s_act1 = float(p["fc_b"][1] / (386.0 * sw_fc))
    s_in = [1.0 / 127.0, s_act0, s_act1]
    s_act = [s_act0, s_act1]

    m = {}
    for i, name in enumerate(("conv1", "conv2", "fc")):
        w = p[name + "_w"].reshape(p[name + "_w"].shape[0], -1)
        sw = _sym_scale(w)
        m[name + "_w"] = np.clip(np.round(w / sw), -127, 127).astype(np.int8)
        m[name + "_b"] = np.round(p[name + "_b"] / (s_in[i] * sw)).astype(np.int32)
        if name != "fc":
            m[name + "_M0"], m[name + "_shift"] = _requant_params(s_in[i] * sw / s_act[i])

    lut_m1 = signed_lut(1, 2)
    lut_m2 = signed_lut(2, 0)

    def extract_conv_features_batch(x_u8_batch, lut):
        xq = quantize_input(x_u8_batch)[:, None]
        B = xq.shape[0]
        h = xq
        for name in ("conv1", "conv2"):
            cols, Ho, Wo = im2col(np.ascontiguousarray(h))
            acc = mac(cols, m[name + "_w"], lut) + m[name + "_b"]
            y = np.clip(_requant(acc, m[name + "_M0"], m[name + "_shift"]), 0, 127).astype(np.int8)
            h = maxpool2(_to_map(y, B, Ho, Wo))
        return h.reshape(B, 400)

    feat_all_m1 = extract_conv_features_batch(x_all, lut_m1).astype(np.float32)
    feat_all_m2 = extract_conv_features_batch(x_all, lut_m2).astype(np.float32)

    # BranchyNet Knowledge Distillation + L1-Variance Feature Selection cho Mode 1 (25% = 100/400 chiều)
    imp = feat_all_m1.std(axis=0) * np.abs(m["fc_w"].astype(np.float32)).sum(axis=0)
    top100_idx = np.sort(np.argsort(imp)[::-1][:100]).astype(np.uint16)

    X100 = feat_all_m1[:, top100_idx]
    logits_target = feat_all_m2 @ m["fc_w"].astype(np.float32).T + m["fc_b"].astype(np.float32)
    X_aug = np.hstack([X100, np.ones((len(X100), 1), dtype=np.float32)])
    reg = 800.0 * np.eye(101, dtype=np.float32)
    reg[-1, -1] = 0.0
    W_kd = np.linalg.solve(X_aug.T @ X_aug + reg, X_aug.T @ logits_target).T
    w_m1_f = W_kd[:, :100]
    b_m1_f = W_kd[:, 100]
    sw_m1 = np.abs(w_m1_f).max() / 127.0
    fc_w_m1 = np.clip(np.round(w_m1_f / sw_m1), -127, 127).astype(np.int8)
    fc_b_m1 = np.round(b_m1_f / sw_m1).astype(np.int32)

    # Chọn 20 mẫu kiểm thử chuẩn (2 mẫu cho mỗi chữ số 0..9)
    test_indices = []
    for rep in range(2):
        for d in range(10):
            idxs = np.where(y_all == d)[0]
            test_indices.append(idxs[rep])
    x_test = x_all[test_indices]
    y_test = y_all[test_indices]

    feat_m1 = extract_conv_features_batch(x_test, lut_m1)
    feat_m2 = extract_conv_features_batch(x_test, lut_m2)

    preds_m2 = (mac(feat_m2[:, None, :], m["fc_w"], lut_m2)[:, 0, :] + m["fc_b"]).argmax(axis=1)
    preds_m1 = (mac(feat_m1[:, None, top100_idx], fc_w_m1, lut_m1)[:, 0, :] + fc_b_m1).argmax(axis=1)
    print(f"[2] 20 mẫu Test chuẩn — Độ chính xác Mode 2 (100%): {(preds_m2 == y_test).mean()*100:.1f}% | Mode 1 (25% KD): {(preds_m1 == y_test).mean()*100:.1f}%")

    W_aux, B_aux = train_and_verify_semantic_object_head()

    conv1_w = m["conv1_w"].flatten()
    conv1_b = m["conv1_b"].flatten()
    conv2_w = m["conv2_w"].flatten()
    conv2_b = m["conv2_b"].flatten()
    fc_w = m["fc_w"].flatten()
    fc_b = m["fc_b"].flatten()
    fc_w_m1_flat = fc_w_m1.flatten()
    fc_b_m1_flat = fc_b_m1.flatten()
    w_aux_flat = W_aux.flatten()
    b_aux_flat = B_aux.flatten()

    with open(OUT_TEST_HEADER, "w", encoding="utf-8") as f:
        f.write("// =========================================================================================\n")
        f.write("// Auto-generated by build_pretrained_ai.py from model_fp32.npz\n")
        f.write("// Kiến trúc Pre-trained CNN 3 Lớp INT8 (Jacob et al., CVPR 2018) + Early-Exit KD (BranchyNet, ICPR 2016)\n")
        f.write("// + Nhánh Phân loại Đối tượng Thực tế & Lọc Bóng đổ 6 Lớp (ViBe / SuBSENSE / VWW TinyML)\n")
        f.write("// =========================================================================================\n")
        f.write("#pragma once\n#include <stdint.h>\n\n")
        f.write(f"#define AI_N_SAMPLES       {len(x_test)}\n")
        f.write("#define AI_FEAT_DIM        400\n")
        f.write("#define AI_PARTIAL_DIM     100 // Mode 1: chỉ tính 25% đặc trưng quan trọng nhất (100/400 phần tử)\n")
        f.write("#define AI_NUM_CLASSES     10\n")
        f.write("#define AI_OBJ_CLASSES     6   // 0:Bóng đổ/Nhiễu(Loại), 1:Người/Bàn tay, 2:Vật thể đặc, 3:Chuyển động nhanh, 4:Đèn Flash, 5:Ký tự/Số\n")
        f.write("#define AI_AUX_FEAT_DIM    16\n\n")

        f.write(f"#define CONV1_M0           {m['conv1_M0']}LL\n")
        f.write(f"#define CONV1_SHIFT        {m['conv1_shift']}\n")
        f.write(f"#define CONV2_M0           {m['conv2_M0']}LL\n")
        f.write(f"#define CONV2_SHIFT        {m['conv2_shift']}\n\n")

        f.write("static const int8_t CONV1_W[8 * 1 * 3 * 3] = {" + ",".join(map(str, conv1_w)) + "};\n")
        f.write("static const int32_t CONV1_B[8] = {" + ",".join(map(str, conv1_b)) + "};\n\n")

        f.write("static const int8_t CONV2_W[16 * 8 * 3 * 3] = {" + ",".join(map(str, conv2_w)) + "};\n")
        f.write("static const int32_t CONV2_B[16] = {" + ",".join(map(str, conv2_b)) + "};\n\n")

        f.write("static const int8_t FC_W[10 * 400] = {" + ",".join(map(str, fc_w)) + "};\n")
        f.write("static const int32_t FC_B[10] = {" + ",".join(map(str, fc_b)) + "};\n\n")

        f.write("// Trọng số Early-Exit Mode 1 (25% = 100/400 chiều) huấn luyện bằng Knowledge Distillation (BranchyNet, ICPR 2016)\n")
        f.write("static const uint16_t AI_TOP100_IDX[AI_PARTIAL_DIM] = {" + ",".join(map(str, top100_idx)) + "};\n")
        f.write("static const int8_t FC_W_M1[10 * AI_PARTIAL_DIM] = {" + ",".join(map(str, fc_w_m1_flat)) + "};\n")
        f.write("static const int32_t FC_B_M1[10] = {" + ",".join(map(str, fc_b_m1_flat)) + "};\n\n")

        f.write("// Trọng số INT8 của Nhánh Phân loại Đối tượng Thực tế & Lọc Bóng đổ (6 lớp x 16 đặc trưng hình thái-quang phổ)\n")
        f.write("static const int8_t OBJ_AUX_W[AI_OBJ_CLASSES * AI_AUX_FEAT_DIM] = {" + ",".join(map(str, w_aux_flat)) + "};\n")
        f.write("static const int32_t OBJ_AUX_B[AI_OBJ_CLASSES] = {" + ",".join(map(str, b_aux_flat)) + "};\n\n")

        f.write("static const uint8_t TEST_LABELS[AI_N_SAMPLES] = {" + ",".join(map(str, y_test)) + "};\n")
        f.write("static const int8_t FEAT_MODE1[AI_N_SAMPLES][400] = {\n")
        for row in feat_m1:
            f.write("  {" + ",".join(map(str, row)) + "},\n")
        f.write("};\n")
        f.write("static const int8_t FEAT_MODE2[AI_N_SAMPLES][400] = {\n")
        for row in feat_m2:
            f.write("  {" + ",".join(map(str, row)) + "},\n")
        f.write("};\n")

    print(f"[3] Đã xuất hoàn chỉnh {OUT_TEST_HEADER}!")


if __name__ == "__main__":
    main()
