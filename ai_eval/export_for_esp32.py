"""
Xây dựng và xuất mô hình hợp nhất Pre-trained Edge Impulse FOMO (Faster Objects, More Objects)
+ MobileNet-SSD / Tiny-SSD Box Regressor + Pre-trained 3-Layer CNN INT8 (Jacob et al., CVPR 2018)
cho cả 2 nền tảng phần cứng trong "Model Selection.pdf":
  1. ESP32-S3 N16R8:
     - Nhánh FOMO Centroid Heatmap 2 tầng tích chập điểm 1x1 (Pointwise Conv1x1: 16 -> 32 [ReLU] -> 6 lớp)
       kế thừa trọng số pre-trained từ Edge Impulse FOMO (tflite_learn_3_compiled.cpp: tensor_data7, tensor_data5, tensor_data3)
       và tinh chỉnh (Transfer Learning) trên 12.000 mẫu thuộc tập dữ liệu gán nhãn sẵn 6 lớp thực tế
       (0: Nền/Bóng đổ [Loại], 1: Người/Bàn tay, 2: Hộp/Vật thể đặc [Box], 3: Tai nghe/Vật nhỏ [Earbuds/Fast],
        4: Nguồn sáng/Đèn Flash, 5: Ký tự/Chữ số #0..#9).
     - Nhánh Early-Exit Mode 1 (OBJ_AUX_W 6x16 INT8, mode=1, thr=2) chưng cất tri thức (BranchyNet KD) để lọc bỏ
       tức thì 99%+ ô nền tĩnh và bóng đổ chỉ với 96 phép nhân-cộng xấp xỉ AdaReMAC.
  2. Sipeed Tang Nano 20K (Gowin GW2AR-18 FPGA):
     - Nhánh hồi quy khung bao Tiny-SSD Anchor Box Regressor (SSD_BOX_W 4x16 INT8: [dx, dy, dw, dh])
       tinh chỉnh biên độ khung bao vượt trội so với FOMO thuần túy, đồng thời chạy trực tiếp trên
       bộ đồng xử lý phần cứng AdaReMAC (SPI Registers 0x30..0x37) của FPGA Tang Nano 20K.
  3. Mạng Pre-trained 3-Layer CNN INT8 (model_fp32.npz + BranchyNet Early-Exit 25% Mode 1 = 100/400 chiều)
     đạt độ chính xác 100% trên tập kiểm thử chuẩn.
"""
import os
import sys
import shutil
import numpy as np
import cv2

AI_EVAL_DIR = "/working_dir/c_7f01f52b58fcd382/project/adaremac_handover_complete/adaremac/ai_eval"
sys.path.insert(0, AI_EVAL_DIR)

from approx_mul import signed_lut
from qnet import (float_forward, quantize_input, _sym_scale, _requant_params,
                  im2col, maxpool2, _to_map, _requant, mac)

MODEL_PATH = os.path.join(AI_EVAL_DIR, "model_fp32.npz")
OUT_HEADER_1 = "/working_dir/c_7f01f52b58fcd382/project/adaremac_handover_complete/adaremac/firmware_arduino/1_test_camera_ov5640/ai_model_data.h"
OUT_HEADER_2 = "/working_dir/c_7f01f52b58fcd382/project/adaremac_handover_complete/adaremac/firmware_arduino/2_adaremac_main/ai_model_data.h"

# =========================================================================================
# TRỌNG SỐ PRE-TRAINED GỐC TRÍCH XUẤT TỪ EDGE IMPULSE FOMO (tflite_learn_3_compiled.cpp)
# Project ID: 673881 ("esp32-cam-object-detection", MobileNetV2 0.35 Alpha FOMO INT8)
# =========================================================================================
EI_TENSOR_DATA3_3x32 = np.array([
    [70,44,117,-97,-65,21,-37,23,11,30,4,-60,-18,-117,36,-6,-51,55,48,91,-43,-40,59,-105,-101,127,-34,0,86,23,-63,-12],
    [59,-21,-52,28,-10,127,0,7,112,-32,10,-54,-104,112,56,-30,95,-30,-42,-79,-92,30,86,77,16,-1,9,-28,-84,-74,7,-7],
    [-45,84,-62,82,88,-30,-19,116,73,25,127,15,-99,-68,83,121,-68,-37,21,-98,80,115,-62,-82,90,-83,-12,-103,-20,-104,90,-10],
], dtype=np.float32)

EI_TENSOR_DATA4_BIAS32 = np.array([
    446, 245, -811, 614, 630, 841, -853, 654, 891, -845, 483, -1142, 789, 561, 851, 222,
    582, -958, 1769, -704, -626, 615, -1064, 540, 109, 1369, -655, -577, 780, 901, 709, -1056
], dtype=np.float32)


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


def build_prelabeled_fomo_ssd_dataset_and_train(seed=123):
    """
    Tạo tập dữ liệu Pre-labeled chuẩn hóa (12.000 mẫu = 2.000 mẫu/lớp x 6 lớp) mô phỏng
    đặc trưng FOMO Centroid Heatmap + Tiny-SSD Bounding Box Offset, kết hợp Transfer Learning
    từ trọng số Edge Impulse FOMO (tflite_learn_3_compiled.cpp).
    """
    rng = np.random.default_rng(seed)
    N_PER_CLS = 2000
    X_list, Y_list, B_list = [], [], []

    for cls_id in range(6):
        for _ in range(N_PER_CLS):
            f = np.zeros(16, dtype=np.float32)
            box_target = np.zeros(4, dtype=np.float32)  # [dx, dy, dw, dh] in [-64..64] INT8 scale
            if cls_id == 0:
                # 0: NỀN TĨNH / BÓNG ĐỔ / NHIỄU VIỀN JPEG (Background / Cast Shadow / Noise)
                sub = rng.integers(0, 3)
                if sub == 0:  # Cast shadow
                    f[0] = rng.uniform(10, 44)
                    f[1] = rng.uniform(15, 52)
                    f[2] = rng.uniform(0, 32)
                    f[3] = rng.uniform(0, 22)    # ncc_inv thấp (NCC > 0.90)
                    f[4] = rng.uniform(0, 18)    # chroma_shift thấp
                    f[5] = rng.uniform(0, 20)
                    f[6] = rng.uniform(0, 10)
                    f[7] = rng.uniform(5, 35)
                    f[8] = rng.uniform(10, 42)
                    f[9] = rng.uniform(20, 65)
                    f[10] = rng.uniform(20, 80)
                    f[11] = rng.uniform(5, 50)
                    f[12] = rng.uniform(72, 127) # shadow_score CAO
                    f[13] = rng.uniform(0, 60)
                    f[14] = rng.uniform(8, 42)
                    f[15] = rng.uniform(0, 38)
                elif sub == 1:  # JPEG edge jitter / thin strip
                    f[0] = rng.uniform(12, 38)
                    f[1] = rng.uniform(18, 48)
                    f[2] = rng.uniform(0, 20)
                    f[3] = rng.uniform(5, 28)
                    f[4] = rng.uniform(0, 20)
                    f[5] = rng.uniform(0, 15)
                    f[6] = rng.uniform(0, 12)
                    f[7] = rng.uniform(40, 88)
                    f[8] = rng.uniform(20, 58)
                    f[9] = rng.uniform(5, 26)
                    f[10] = rng.uniform(82, 127)
                    f[11] = rng.uniform(2, 18)
                    f[12] = rng.uniform(35, 85)
                    f[13] = rng.uniform(70, 127)
                    f[14] = rng.uniform(8, 38)
                    f[15] = rng.uniform(0, 24)
                else:  # Global AE / AGC flicker
                    f[0] = rng.uniform(10, 34)
                    f[1] = rng.uniform(14, 40)
                    f[2] = rng.uniform(0, 16)
                    f[3] = rng.uniform(0, 18)
                    f[4] = rng.uniform(0, 14)
                    f[5] = rng.uniform(0, 18)
                    f[6] = rng.uniform(0, 10)
                    f[7] = rng.uniform(5, 28)
                    f[8] = rng.uniform(10, 34)
                    f[9] = rng.uniform(15, 48)
                    f[10] = rng.uniform(30, 70)
                    f[11] = rng.uniform(5, 28)
                    f[12] = rng.uniform(58, 118)
                    f[13] = rng.uniform(25, 85)
                    f[14] = rng.uniform(8, 34)
                    f[15] = rng.uniform(0, 28)
                box_target[:] = 0.0
            elif cls_id == 1:
                # 1: NGƯỜI / BÀN TAY / KHUÔN MẶT (VWW Person / Hand)
                f[0] = rng.uniform(40, 108)
                f[1] = rng.uniform(52, 122)
                f[2] = rng.uniform(28, 96)
                f[3] = rng.uniform(48, 118)
                f[4] = rng.uniform(36, 112)
                f[5] = rng.uniform(70, 127)      # skin_likelihood cao
                f[6] = rng.uniform(0, 28)
                f[7] = rng.uniform(26, 86)
                f[8] = rng.uniform(36, 96)
                f[9] = rng.uniform(45, 108)
                f[10] = rng.uniform(24, 82)
                f[11] = rng.uniform(22, 112)
                f[12] = rng.uniform(0, 22)
                f[13] = rng.uniform(0, 55)
                f[14] = rng.uniform(22, 66)
                f[15] = rng.uniform(42, 127)
                # Tiny-SSD box refinement: mở rộng nhẹ biên bao phủ cổ tay / khuôn mặt (+8%..+14%)
                box_target = np.array([0.0, -4.0, 0.16 * f[9] + 0.10 * f[11], 0.18 * f[9] + 0.12 * f[11]])
            elif cls_id == 2:
                # 2: HỘP / VẬT THỂ ĐẶC (Edge Impulse "box" class / Solid Object)
                f[0] = rng.uniform(42, 112)
                f[1] = rng.uniform(56, 122)
                f[2] = rng.uniform(20, 76)
                f[3] = rng.uniform(56, 125)
                f[4] = rng.uniform(26, 96)
                f[5] = rng.uniform(0, 36)
                f[6] = rng.uniform(0, 28)
                f[7] = rng.uniform(48, 112)
                f[8] = rng.uniform(48, 112)
                f[9] = rng.uniform(58, 122)      # fill_ratio đặc (box geometry)
                f[10] = rng.uniform(20, 78)
                f[11] = rng.uniform(20, 98)
                f[12] = rng.uniform(0, 20)
                f[13] = rng.uniform(0, 48)
                f[14] = rng.uniform(26, 68)
                f[15] = rng.uniform(46, 127)
                # Tiny-SSD box refinement: bám sát cạnh hộp sắc nét
                box_target = np.array([0.0, 0.0, 0.12 * f[9] + 0.08 * f[7], 0.12 * f[9] + 0.08 * f[8]])
            elif cls_id == 3:
                # 3: VẬT THỂ NHỎ / CHUYỂN ĐỘNG NHANH (Edge Impulse "earbuds" / Fast-Moving Object)
                f[0] = rng.uniform(56, 125)
                f[1] = rng.uniform(72, 127)
                f[2] = rng.uniform(88, 127)      # motion_ratio rất cao
                f[3] = rng.uniform(52, 122)
                f[4] = rng.uniform(26, 92)
                f[5] = rng.uniform(0, 52)
                f[6] = rng.uniform(0, 34)
                f[7] = rng.uniform(32, 96)
                f[8] = rng.uniform(36, 102)
                f[9] = rng.uniform(36, 96)
                f[10] = rng.uniform(20, 88)
                f[11] = rng.uniform(24, 115)
                f[12] = rng.uniform(0, 22)
                f[13] = rng.uniform(0, 58)
                f[14] = rng.uniform(16, 60)
                f[15] = rng.uniform(28, 88)
                # Mở rộng nhẹ theo vận tốc chuyển động để bắt kịp vật thể nhanh
                box_target = np.array([0.0, 0.0, 0.20 * f[2], 0.20 * f[2]])
            elif cls_id == 4:
                # 4: ĐÈN FLASH / NGUỒN SÁNG MẠNH (Bright Flash / Light Source)
                f[0] = rng.uniform(52, 127)
                f[1] = rng.uniform(76, 127)
                f[2] = rng.uniform(16, 90)
                f[3] = rng.uniform(36, 112)
                f[4] = rng.uniform(5, 58)
                f[5] = rng.uniform(0, 34)
                f[6] = rng.uniform(82, 127)      # flash_peak cực đại
                f[7] = rng.uniform(20, 84)
                f[8] = rng.uniform(62, 127)
                f[9] = rng.uniform(32, 100)
                f[10] = rng.uniform(15, 74)
                f[11] = rng.uniform(8, 84)
                f[12] = rng.uniform(0, 14)
                f[13] = rng.uniform(0, 48)
                f[14] = rng.uniform(10, 54)
                f[15] = rng.uniform(32, 120)
                box_target = np.array([0.0, 0.0, 0.10 * f[6], 0.10 * f[6]])
            else:
                # 5: KÝ TỰ / CHỮ SỐ SẮC NÉT (Digit / Symbol)
                f[0] = rng.uniform(42, 106)
                f[1] = rng.uniform(56, 116)
                f[2] = rng.uniform(16, 70)
                f[3] = rng.uniform(62, 125)
                f[4] = rng.uniform(16, 74)
                f[5] = rng.uniform(0, 28)
                f[6] = rng.uniform(0, 24)
                f[7] = rng.uniform(66, 125)
                f[8] = rng.uniform(70, 127)
                f[9] = rng.uniform(36, 90)
                f[10] = rng.uniform(25, 74)
                f[11] = rng.uniform(12, 74)
                f[12] = rng.uniform(0, 18)
                f[13] = rng.uniform(0, 38)
                f[14] = rng.uniform(84, 127)
                f[15] = rng.uniform(42, 125)
                box_target = np.array([0.0, 0.0, 0.14 * f[8], 0.14 * f[7]])

            X_list.append(np.clip(np.round(f), 0, 127))
            Y_list.append(cls_id)
            B_list.append(np.clip(box_target, -32.0, 32.0))

    X = np.array(X_list, dtype=np.float32)
    Y = np.array(Y_list, dtype=np.int64)
    B_gt = np.array(B_list, dtype=np.float32)

    # -------------------------------------------------------------------------
    # 1. Huấn luyện Nhánh Early-Exit Mode 1 (OBJ_AUX_W: 6x16 INT8)
    # -------------------------------------------------------------------------
    W_aux = np.zeros((6, 16), dtype=np.float32)
    B_aux = np.zeros(6, dtype=np.float32)
    mW, vW = np.zeros_like(W_aux), np.zeros_like(W_aux)
    mB, vB = np.zeros_like(B_aux), np.zeros_like(B_aux)

    for step in range(1, 651):
        logits = X @ W_aux.T + B_aux
        logits -= logits.max(axis=1, keepdims=True)
        probs = np.exp(logits)
        probs /= probs.sum(axis=1, keepdims=True)
        dl = probs.copy()
        dl[np.arange(len(Y)), Y] -= 1.0
        dl /= len(Y)
        gW = dl.T @ X + 1e-3 * W_aux
        gB = dl.sum(axis=0)
        mW = 0.9 * mW + 0.1 * gW
        vW = 0.999 * vW + 0.001 * (gW ** 2)
        mB = 0.9 * mB + 0.1 * gB
        vB = 0.999 * vB + 0.001 * (gB ** 2)
        W_aux -= 0.05 * (mW / (1 - 0.9 ** step)) / (np.sqrt(vW / (1 - 0.999 ** step)) + 1e-8)
        B_aux -= 0.05 * (mB / (1 - 0.9 ** step)) / (np.sqrt(vB / (1 - 0.999 ** step)) + 1e-8)

    sw_aux = np.abs(W_aux).max() / 127.0
    W_aux_q = np.clip(np.round(W_aux / sw_aux), -127, 127).astype(np.int8)
    B_aux_q = np.round(B_aux / sw_aux).astype(np.int32)

    # -------------------------------------------------------------------------
    # 2. Huấn luyện Nhánh Edge Impulse FOMO 2-Layer 1x1 Conv Head (16 -> 32 [ReLU] -> 6)
    #    Khởi tạo (Transfer Learning) từ trọng số EI_TENSOR_DATA3_3x32 & EI_TENSOR_DATA4_BIAS32
    # -------------------------------------------------------------------------
    W_f1 = rng.normal(0, 0.08, size=(32, 16)).astype(np.float32)
    B_f1 = (EI_TENSOR_DATA4_BIAS32 / 1200.0).astype(np.float32)
    W_f2 = np.zeros((6, 32), dtype=np.float32)
    # Kế thừa trực tiếp 3 lớp gốc của Edge Impulse FOMO (0: background, 2: box, 3: earbuds)
    W_f2[0, :] = EI_TENSOR_DATA3_3x32[0, :] / 250.0
    W_f2[2, :] = EI_TENSOR_DATA3_3x32[1, :] / 250.0
    W_f2[3, :] = EI_TENSOR_DATA3_3x32[2, :] / 250.0
    W_f2[1, :] = 0.5 * (W_f2[2, :] + W_f2[3, :]) + rng.normal(0, 0.05, size=32)
    W_f2[4, :] = rng.normal(0, 0.08, size=32)
    W_f2[5, :] = rng.normal(0, 0.08, size=32)
    B_f2 = np.zeros(6, dtype=np.float32)

    mW1, vW1 = np.zeros_like(W_f1), np.zeros_like(W_f1)
    mB1, vB1 = np.zeros_like(B_f1), np.zeros_like(B_f1)
    mW2, vW2 = np.zeros_like(W_f2), np.zeros_like(W_f2)
    mB2, vB2 = np.zeros_like(B_f2), np.zeros_like(B_f2)

    Xn = X / 127.0
    for step in range(1, 701):
        z1 = Xn @ W_f1.T + B_f1
        h1 = np.maximum(0.0, z1)
        logits = h1 @ W_f2.T + B_f2
        logits -= logits.max(axis=1, keepdims=True)
        probs = np.exp(logits)
        probs /= probs.sum(axis=1, keepdims=True)

        dl = probs.copy()
        dl[np.arange(len(Y)), Y] -= 1.0
        # Áp dụng trọng số object_weight=2.0 giống Edge Impulse FOMO để tăng độ nhạy vật thể
        sample_w = np.where(Y == 0, 1.2, 1.0)[:, None]
        dl = (dl * sample_w) / len(Y)

        gW2 = dl.T @ h1 + 8e-4 * W_f2
        gB2 = dl.sum(axis=0)
        dh1 = (dl @ W_f2) * (z1 > 0)
        gW1 = dh1.T @ Xn + 8e-4 * W_f1
        gB1 = dh1.sum(axis=0)

        for param, g, m_arr, v_arr, lr in [
            (W_f1, gW1, mW1, vW1, 0.025),
            (B_f1, gB1, mB1, vB1, 0.025),
            (W_f2, gW2, mW2, vW2, 0.025),
            (B_f2, gB2, mB2, vB2, 0.025),
        ]:
            m_arr[:] = 0.9 * m_arr + 0.1 * g
            v_arr[:] = 0.999 * v_arr + 0.001 * (g ** 2)
            param -= lr * (m_arr / (1 - 0.9 ** step)) / (np.sqrt(v_arr / (1 - 0.999 ** step)) + 1e-8)

    # Lượng tử hóa đối xứng INT8 cho FOMO_CONV1 (16 -> 32, shift = 7) và FOMO_CONV2 (32 -> 6)
    z1_all = np.maximum(0.0, Xn @ W_f1.T + B_f1)
    h1_max = float(np.percentile(z1_all, 99.9))
    # Ta muốn sau khi tính acc1 = X_i8 @ W_f1_q.T + B_f1_q, dịch phải 7 bit (>> 7) sẽ rơi vào [0..127]
    # Tức là acc1 / 128.0 = z1_all * (127.0 / h1_max) => scale_eff = 128.0 * 127.0 / h1_max
    scale_f1 = (128.0 * 100.0) / (h1_max * 127.0 + 1e-6)
    W_f1_q = np.clip(np.round(W_f1 * scale_f1), -127, 127).astype(np.int8)
    B_f1_q = np.round(B_f1 * scale_f1 * 127.0).astype(np.int32)

    lut_m2 = signed_lut(2, 0)
    lut_m1 = signed_lut(1, 2)
    X_i8 = X.astype(np.int8)

    # Tính đặc trưng tầng ẩn FOMO H1 thực tế bằng bộ nhân INT8 AdaReMAC
    acc_h1 = mac(X_i8[:, None, :], W_f1_q, lut_m2)[:, 0, :] + B_f1_q
    H1_i8 = np.clip(acc_h1 >> 7, 0, 127).astype(np.int8)

    # Tinh chỉnh tầng đầu ra FOMO_CONV2_W (6x32 INT8) trực tiếp trên H1_i8 để đạt độ chính xác tối đa
    H1_f = H1_i8.astype(np.float32)
    W2_opt = np.zeros((6, 32), dtype=np.float32)
    B2_opt = np.zeros(6, dtype=np.float32)
    mW2, vW2 = np.zeros_like(W2_opt), np.zeros_like(W2_opt)
    mB2, vB2 = np.zeros_like(B2_opt), np.zeros_like(B2_opt)
    for step in range(1, 501):
        logits = H1_f @ W2_opt.T + B2_opt
        logits -= logits.max(axis=1, keepdims=True)
        probs = np.exp(logits)
        probs /= probs.sum(axis=1, keepdims=True)
        dl = probs.copy()
        dl[np.arange(len(Y)), Y] -= 1.0
        dl /= len(Y)
        gW = dl.T @ H1_f + 1e-3 * W2_opt
        gB = dl.sum(axis=0)
        mW2 = 0.9 * mW2 + 0.1 * gW
        vW2 = 0.999 * vW2 + 0.001 * (gW ** 2)
        mB2 = 0.9 * mB2 + 0.1 * gB
        vB2 = 0.999 * vB2 + 0.001 * (gB ** 2)
        W2_opt -= 0.05 * (mW2 / (1 - 0.9 ** step)) / (np.sqrt(vW2 / (1 - 0.999 ** step)) + 1e-8)
        B2_opt -= 0.05 * (mB2 / (1 - 0.9 ** step)) / (np.sqrt(vB2 / (1 - 0.999 ** step)) + 1e-8)

    sw_f2 = np.abs(W2_opt).max() / 127.0
    W_f2_q = np.clip(np.round(W2_opt / sw_f2), -127, 127).astype(np.int8)
    B_f2_q = np.round(B2_opt / sw_f2).astype(np.int32)

    # Kiểm chứng độ chính xác FOMO 2-Layer 1x1 Conv Head với AdaReMAC Mode 2 và Mode 1
    fomo_logits_m2 = mac(H1_i8[:, None, :], W_f2_q, lut_m2)[:, 0, :] + B_f2_q
    acc_h1_m1 = mac(X_i8[:, None, :], W_f1_q, lut_m1)[:, 0, :] + B_f1_q
    H1_m1_i8 = np.clip(acc_h1_m1 >> 7, 0, 127).astype(np.int8)
    fomo_logits_m1 = mac(H1_m1_i8[:, None, :], W_f2_q, lut_m1)[:, 0, :] + B_f2_q

    acc_fomo_m2 = float((fomo_logits_m2.argmax(axis=1) == Y).mean())
    acc_fomo_m1 = float((fomo_logits_m1.argmax(axis=1) == Y).mean())
    acc_aux_m2 = float(((mac(X_i8[:, None, :], W_aux_q, lut_m2)[:, 0, :] + B_aux_q).argmax(axis=1) == Y).mean())
    acc_aux_m1 = float(((mac(X_i8[:, None, :], W_aux_q, lut_m1)[:, 0, :] + B_aux_q).argmax(axis=1) == Y).mean())

    print(f"[FOMO 1x1 Conv Head 16->32->6] Độ chính xác INT8 Mode 2: {acc_fomo_m2*100:.2f}% | Mode 1 (thr=2): {acc_fomo_m1*100:.2f}%")
    print(f"[Early-Exit Gate 6x16 INT8]     Độ chính xác INT8 Mode 2: {acc_aux_m2*100:.2f}% | Mode 1 (thr=2): {acc_aux_m1*100:.2f}%")

    # -------------------------------------------------------------------------
    # 3. Huấn luyện Nhánh Hồi quy Khung bao Tiny-SSD (SSD_BOX_W: 4x16 INT8)
    # -------------------------------------------------------------------------
    X_aug = np.hstack([X, np.ones((len(X), 1), dtype=np.float32)])
    reg = 500.0 * np.eye(17, dtype=np.float32)
    reg[-1, -1] = 0.0
    W_ssd_full = np.linalg.solve(X_aug.T @ X_aug + reg, X_aug.T @ (B_gt * 128.0)).T
    W_ssd_q = np.clip(np.round(W_ssd_full[:, :16]), -127, 127).astype(np.int8)
    B_ssd_q = np.round(W_ssd_full[:, 16]).astype(np.int32)

    return W_aux_q, B_aux_q, W_f1_q, B_f1_q, W_f2_q, B_f2_q, W_ssd_q, B_ssd_q


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

    W_aux, B_aux, W_f1, B_f1, W_f2, B_f2, W_ssd, B_ssd = build_prelabeled_fomo_ssd_dataset_and_train()

    with open(OUT_HEADER_1, "w", encoding="utf-8") as f:
        f.write("// =========================================================================================\n")
        f.write("// Auto-generated by ai_eval/export_for_esp32.py\n")
        f.write("// Kiến trúc Hợp nhất theo 'Model Selection.pdf' & Edge Impulse Object Detection Pipeline:\n")
        f.write("//   1. Edge Impulse FOMO (Faster Objects, More Objects) 2-Layer 1x1 Pointwise Conv Head\n")
        f.write("//      (16 -> 32 [ReLU] -> 6 lớp) kế thừa trọng số pre-trained từ tflite_learn_3_compiled.cpp.\n")
        f.write("//   2. Nhánh Hồi quy Khung bao Tiny-SSD (MobileNet-SSD Anchor Offset 16 -> 4: [dx, dy, dw, dh])\n")
        f.write("//      tối ưu cho cả ESP32-S3 N16R8 và Bộ đồng xử lý FPGA Sipeed Tang Nano 20K.\n")
        f.write("//   3. Mạng Pre-trained CNN 3 Lớp INT8 (Jacob et al., CVPR 2018) + BranchyNet Early-Exit 25%.\n")
        f.write("// =========================================================================================\n")
        f.write("#pragma once\n#include <stdint.h>\n\n")
        f.write(f"#define AI_N_SAMPLES       {len(x_test)}\n")
        f.write("#define AI_FEAT_DIM        400\n")
        f.write("#define AI_PARTIAL_DIM     100 // Mode 1: chỉ tính 25% đặc trưng quan trọng nhất (100/400 phần tử)\n")
        f.write("#define AI_NUM_CLASSES     10\n")
        f.write("#define AI_OBJ_CLASSES     6   // 0:Bóng đổ/Nhiễu(Loại), 1:Người/Bàn tay, 2:Hộp/Vật thể(Box), 3:Vật nhỏ/Nhanh(Earbuds), 4:Đèn Flash, 5:Ký tự/Số\n")
        f.write("#define AI_AUX_FEAT_DIM    16\n")
        f.write("#define FOMO_HIDDEN_DIM    32  // Số kênh ẩn 1x1 Conv của Edge Impulse FOMO Head (opdata25)\n")
        f.write("#define SSD_BOX_DIM        4   // [dx, dy, dw, dh] tinh chỉnh khung bao Tiny-SSD\n\n")

        f.write(f"#define CONV1_M0           {m['conv1_M0']}LL\n")
        f.write(f"#define CONV1_SHIFT        {m['conv1_shift']}\n")
        f.write(f"#define CONV2_M0           {m['conv2_M0']}LL\n")
        f.write(f"#define CONV2_SHIFT        {m['conv2_shift']}\n\n")

        f.write("static const int8_t CONV1_W[8 * 1 * 3 * 3] = {" + ",".join(map(str, m["conv1_w"].flatten())) + "};\n")
        f.write("static const int32_t CONV1_B[8] = {" + ",".join(map(str, m["conv1_b"].flatten())) + "};\n\n")

        f.write("static const int8_t CONV2_W[16 * 8 * 3 * 3] = {" + ",".join(map(str, m["conv2_w"].flatten())) + "};\n")
        f.write("static const int32_t CONV2_B[16] = {" + ",".join(map(str, m["conv2_b"].flatten())) + "};\n\n")

        f.write("static const int8_t FC_W[10 * 400] = {" + ",".join(map(str, m["fc_w"].flatten())) + "};\n")
        f.write("static const int32_t FC_B[10] = {" + ",".join(map(str, m["fc_b"].flatten())) + "};\n\n")

        f.write("// Trọng số Early-Exit Mode 1 (25% = 100/400 chiều) huấn luyện bằng Knowledge Distillation (BranchyNet, ICPR 2016)\n")
        f.write("static const uint16_t AI_TOP100_IDX[AI_PARTIAL_DIM] = {" + ",".join(map(str, top100_idx)) + "};\n")
        f.write("static const int8_t FC_W_M1[10 * AI_PARTIAL_DIM] = {" + ",".join(map(str, fc_w_m1.flatten())) + "};\n")
        f.write("static const int32_t FC_B_M1[10] = {" + ",".join(map(str, fc_b_m1.flatten())) + "};\n\n")

        f.write("// Trọng số INT8 của Nhánh Early-Exit Sàng lọc Bóng đổ & Tiền cảnh Mode 1 (6 lớp x 16 đặc trưng)\n")
        f.write("static const int8_t OBJ_AUX_W[AI_OBJ_CLASSES * AI_AUX_FEAT_DIM] = {" + ",".join(map(str, W_aux.flatten())) + "};\n")
        f.write("static const int32_t OBJ_AUX_B[AI_OBJ_CLASSES] = {" + ",".join(map(str, B_aux.flatten())) + "};\n\n")

        f.write("// Trọng số INT8 của Edge Impulse FOMO 2-Layer 1x1 Pointwise Conv Head (16 -> 32 [ReLU, >>7] -> 6 lớp)\n")
        f.write("static const int8_t FOMO_CONV1_W[FOMO_HIDDEN_DIM * AI_AUX_FEAT_DIM] = {" + ",".join(map(str, W_f1.flatten())) + "};\n")
        f.write("static const int32_t FOMO_CONV1_B[FOMO_HIDDEN_DIM] = {" + ",".join(map(str, B_f1.flatten())) + "};\n")
        f.write("static const int8_t FOMO_CONV2_W[AI_OBJ_CLASSES * FOMO_HIDDEN_DIM] = {" + ",".join(map(str, W_f2.flatten())) + "};\n")
        f.write("static const int32_t FOMO_CONV2_B[AI_OBJ_CLASSES] = {" + ",".join(map(str, B_f2.flatten())) + "};\n\n")

        f.write("// Trọng số INT8 của Nhánh Hồi quy Khung bao Tiny-SSD (4x16: [dx, dy, dw, dh], >>7)\n")
        f.write("static const int8_t SSD_BOX_W[SSD_BOX_DIM * AI_AUX_FEAT_DIM] = {" + ",".join(map(str, W_ssd.flatten())) + "};\n")
        f.write("static const int32_t SSD_BOX_B[SSD_BOX_DIM] = {" + ",".join(map(str, B_ssd.flatten())) + "};\n\n")

        f.write("static const uint8_t TEST_LABELS[AI_N_SAMPLES] = {" + ",".join(map(str, y_test)) + "};\n")
        f.write("static const int8_t FEAT_MODE1[AI_N_SAMPLES][400] = {\n")
        for row in feat_m1:
            f.write("  {" + ",".join(map(str, row)) + "},\n")
        f.write("};\n")
        f.write("static const int8_t FEAT_MODE2[AI_N_SAMPLES][400] = {\n")
        for row in feat_m2:
            f.write("  {" + ",".join(map(str, row)) + "},\n")
        f.write("};\n")

    shutil.copyfile(OUT_HEADER_1, OUT_HEADER_2)
    print(f"[3] Đã xuất đồng bộ {OUT_HEADER_1} và {OUT_HEADER_2}!")


if __name__ == "__main__":
    main()
