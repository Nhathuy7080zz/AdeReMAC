"""
Lượng tử hóa FOMO-Micro đã huấn luyện -> INT8, đánh giá trên COCO val 2017 với các chế độ AdaReMAC,
và xuất cho cả hai nền tảng:
  firmware_arduino/adaremac_fomo_cam/fomo_micro_model.h   (ESP32-S3, C thuần)
  rtl/fomo_model.vh + rtl/mem/fomo_*.hex                  (Tang Nano 20K)
  sim/vectors/*.hex                                        (vector vàng cho testbench RTL)
  ai_fomo/results/int8_eval.json                           (số liệu đo)

  python export_fomo_micro.py --ckpt runs/micro/model.pt --train data/coco_person_train96.npz --val data/coco_person_val96.npz
"""
import argparse
import json
import os
import numpy as np
import torch

from fomo_models import FomoMicro, MICRO_LAYERS
from fomo_data import boxes_by_image, eval_f1, image_level
from fomo_int8 import quantize, forward_int, logit_to_prob, mode0_bias_compensation

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
LANES = 16
FRAME_BASE = 18432
REGION_A, REGION_B = 0, 18432          # ping-pong trong act_mem (FRAME_BASE = REGION_B)


def scene_subset(bxs, min_px=12, max_n=3):
    """Tập con 'cảnh kiểu camera an ninh': 1..max_n người, mỗi người cạnh ngắn >= min_px ở 96x96,
    hoặc ảnh không có người. Phù hợp giới hạn của FOMO (vật cùng cỡ, không quá sát nhau)."""
    keep = []
    for i, b in enumerate(bxs):
        if len(b) == 0:
            keep.append(i)
            continue
        sides = np.minimum(b[:, 2] - b[:, 0], b[:, 3] - b[:, 1]) * 96
        if len(b) <= max_n and (sides >= min_px).all():
            keep.append(i)
    return np.array(keep)


def evaluate(q, imgs, bxs, thr_prob, **kw):
    lg = forward_int(q, imgs, **kw)
    pr = logit_to_prob(q, lg)
    return eval_f1(pr, bxs, thr_prob), image_level(pr, bxs, thr_prob), lg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", required=True)
    ap.add_argument("--train", required=True)
    ap.add_argument("--val", required=True)
    ap.add_argument("--n-calib", type=int, default=512)
    ap.add_argument("--n-vectors", type=int, default=3)
    ap.add_argument("--skip-eval", action="store_true", help="chỉ xuất file, không đánh giá lại")
    args = ap.parse_args()

    ck = torch.load(args.ckpt, map_location="cpu", weights_only=False)
    model = FomoMicro()
    model.load_state_dict(ck["state"])
    model.eval()
    thr = float(ck["threshold"])

    tr = np.load(args.train)
    rng = np.random.default_rng(1)
    calib = tr["images"][rng.choice(len(tr["images"]), args.n_calib, replace=False)]
    va = np.load(args.val)
    vimgs = va["images"]
    vbxs = boxes_by_image(va["boxes"], len(vimgs))

    q = quantize(model, calib)
    comps = mode0_bias_compensation(q, calib)
    nL = len(q)

    # ---------------- đánh giá ----------------
    with torch.no_grad():
        pf = torch.sigmoid(model(torch.from_numpy(vimgs).float().div(255).unsqueeze(1))).numpy()
    sub = scene_subset(vbxs)
    res = {"threshold_prob": thr, "n_test": int(len(vimgs)), "n_scene_subset": int(len(sub)),
           "fp32": {"all": eval_f1(pf, vbxs, thr), "inbox": eval_f1(pf, vbxs, thr, mode="inbox"), "image": image_level(pf, vbxs, thr),
                    "scene": eval_f1(pf[sub], [vbxs[i] for i in sub], thr),
                    "scene_image": image_level(pf[sub], [vbxs[i] for i in sub], thr)}}
    configs = {
        "int8_exact (Mode 2)": dict(modes=[2] * nL),
        "Mode 0 mọi lớp, không bù": dict(modes=[0] * nL),
        "Mode 0 mọi lớp, bù bias": dict(modes=[0] * nL, comps=comps),
        "Mode 1 (thr=2) mọi lớp": dict(modes=[1] * nL, thr=2),
        "Mode 0 lớp 1-5 + bù, lớp cuối Mode 2": dict(modes=[0] * (nL - 1) + [2], comps=comps),
    }
    for name, kw in ({} if args.skip_eval else configs).items():
        f, il, lg = evaluate(q, vimgs, vbxs, thr, **kw)
        pr = logit_to_prob(q, lg)
        res[name] = {"all": f, "inbox": eval_f1(pr, vbxs, thr, mode="inbox"), "image": il,
                     "scene": eval_f1(pr[sub], [vbxs[i] for i in sub], thr),
                     "scene_image": image_level(pr[sub], [vbxs[i] for i in sub], thr)}
        print(f"{name:40s} F1 {f['f1']:.3f} (P {f['precision']:.3f} R {f['recall']:.3f}) trong-hộp {res[name]['inbox']['f1']:.3f} | "
              f"cảnh-an-ninh F1 {res[name]['scene']['f1']:.3f} | ảnh: P {il['precision']:.3f} R {il['recall']:.3f} FPR {il['fpr']:.3f}")
    print(f"{'fp32':40s} F1 {res['fp32']['all']['f1']:.3f} | cảnh-an-ninh F1 {res['fp32']['scene']['f1']:.3f}")

    # ---------------- bảng lớp (dùng chung C + RTL) ----------------
    layers = []
    H = 96
    wbase, cbase = 0, 0
    in_base, out_base = FRAME_BASE, REGION_A
    for i, (L, (ci, co, k, s)) in enumerate(zip(q, MICRO_LAYERS)):
        Ho = (H + 2 * (k // 2) - k) // s + 1
        ng = (co + LANES - 1) // LANES
        kt = ci * k * k
        last = i == nL - 1
        layers.append(dict(cin=ci, cout=co, k=k, s=s, hi=H, wi=H, ho=Ho, wo=Ho, relu=int(L["relu"]),
                           last=int(last), sh=int(L["sh"]), wbase=wbase, cbase=cbase,
                           inb=in_base, outb=out_base, ng=ng, kt=kt))
        wbase += ng * kt
        cbase += co
        H = Ho
        in_base, out_base = out_base, (REGION_B if out_base == REGION_A else REGION_A)
        if in_base == REGION_B:
            in_base = FRAME_BASE
    assert wbase <= 2048 and cbase <= 128
    res["layers"] = layers
    res["weight_words"] = wbase
    os.makedirs(os.path.join(ROOT, "ai_fomo", "results"), exist_ok=True)
    if not args.skip_eval:
      json.dump(res, open(os.path.join(ROOT, "ai_fomo", "results", "int8_eval.json"), "w"), indent=2, ensure_ascii=False)

    # ---------------- RTL ----------------
    rtl = os.path.join(ROOT, "rtl")
    os.makedirs(os.path.join(rtl, "mem"), exist_ok=True)
    with open(os.path.join(rtl, "fomo_model.vh"), "w") as f:
        f.write("// TU DONG SINH boi ai_fomo/export_fomo_micro.py - khong sua tay\n")
        f.write(f"localparam NUM_LAYERS = {nL};\nlocalparam WMEM_DEPTH = 2048;\nlocalparam CMEM_DEPTH = 128;\n")
        f.write("always @* begin\n    L_CIN = 0; L_COUT = 0; L_K = 0; L_S = 0; L_HI = 0; L_WI = 0; L_HIWI = 0;\n"
                "    L_HO = 0; L_WO = 0; L_HOWO = 0; L_RELU = 0; L_LAST = 0; L_SH = 0; L_WBASE = 0;\n"
                "    L_CBASE = 0; L_INB = 0; L_OUTB = 0; L_NG = 0; L_KT = 0;\n    case (li)\n")
        for i, l in enumerate(layers):
            f.write(f"    3'd{i}: begin L_CIN = {l['cin']}; L_COUT = {l['cout']}; L_K = {l['k']}; L_S = {l['s']}; "
                    f"L_HI = {l['hi']}; L_WI = {l['wi']}; L_HIWI = {l['hi']*l['wi']}; L_HO = {l['ho']}; L_WO = {l['wo']}; "
                    f"L_HOWO = {l['ho']*l['wo']}; L_RELU = {l['relu']}; L_LAST = {l['last']}; L_SH = {l['sh']}; "
                    f"L_WBASE = {l['wbase']}; L_CBASE = {l['cbase']}; L_INB = {l['inb']}; L_OUTB = {l['outb']}; "
                    f"L_NG = {l['ng']}; L_KT = {l['kt']}; end\n")
        f.write("    default: ;\n    endcase\nend\n")
    words = np.zeros((2048, LANES), np.int64)
    for l, L in zip(layers, q):
        W = L["w"].reshape(l["cout"], -1)
        for g in range(l["ng"]):
            for lane in range(LANES):
                co = g * LANES + lane
                if co < l["cout"]:
                    words[l["wbase"] + g * l["kt"]: l["wbase"] + (g + 1) * l["kt"], lane] = W[co]
    with open(os.path.join(rtl, "mem", "fomo_w.hex"), "w") as f:
        for w in words:
            # dạng dấu-độ lớn: bit7 = dấu, bit6..0 = |w| (khớp fomo_mac_lane.v)
            f.write("".join(f"{(0x80 | -int(v)) if v < 0 else int(v):02x}" for v in w[::-1]) + "\n")   # lane 15 ... lane 0
    bias = np.zeros(128, np.int64); comp = np.zeros(128, np.int64); mm = np.zeros(128, np.int64)
    for l, L, c in zip(layers, q, comps):
        bias[l["cbase"]:l["cbase"] + l["cout"]] = L["b"]
        comp[l["cbase"]:l["cbase"] + l["cout"]] = c
        mm[l["cbase"]:l["cbase"] + l["cout"]] = L["M"]
    for name, arr, nb in (("fomo_bias.hex", bias, 8), ("fomo_comp.hex", comp, 8), ("fomo_m.hex", mm, 4)):
        with open(os.path.join(rtl, "mem", name), "w") as f:
            for v in arr:
                f.write(f"{int(v) & ((1 << (4 * nb)) - 1):0{nb}x}\n")

    # ---------------- vector vàng cho testbench ----------------
    vec = os.path.join(ROOT, "sim", "vectors")
    os.makedirs(vec, exist_ok=True)
    pick = [i for i in sub if len(vbxs[i]) > 0][:args.n_vectors]
    tests = []
    for t, i in enumerate(pick):
        img = vimgs[i:i + 1]
        exp2 = forward_int(q, img, modes=[2] * nL)[0]
        exp0 = forward_int(q, img, modes=[0] * nL, comps=comps)[0]
        np.savetxt(os.path.join(vec, f"frame{t}.hex"), img.reshape(-1), fmt="%02x")
        np.savetxt(os.path.join(vec, f"expect{t}_m2.hex"), exp2.reshape(-1) & 0xFF, fmt="%02x")
        np.savetxt(os.path.join(vec, f"expect{t}_m0c.hex"), exp0.reshape(-1) & 0xFF, fmt="%02x")
        tests.append(dict(index=int(i), coco_id=int(va["ids"][i]), persons=int(len(vbxs[i]))))
    json.dump(tests, open(os.path.join(vec, "vectors.json"), "w"), indent=2)

    # ---------------- C header cho ESP32-S3 ----------------
    fw = os.path.join(ROOT, "firmware_arduino", "adaremac_fomo_cam")
    os.makedirs(fw, exist_ok=True)
    with open(os.path.join(fw, "fomo_micro_model.h"), "w") as f:
        f.write("// TU DONG SINH boi ai_fomo/export_fomo_micro.py - FOMO-Micro INT8 (person), huan luyen tren COCO 2017\n")
        f.write("#pragma once\n#include <stdint.h>\n\n")
        f.write(f"#define FOMO_NUM_LAYERS {nL}\n#define FOMO_IN_W 96\n#define FOMO_IN_H 96\n#define FOMO_GRID 12\n")
        f.write(f"#define FOMO_LOGIT_SCALE {q[-1]['s_out']:.8e}f  // prob = sigmoid(logit_i8 * scale)\n")
        thr_logit = int(np.ceil(np.log(thr / (1 - thr)) / q[-1]["s_out"]))
        f.write(f"#define FOMO_DEFAULT_THR_LOGIT {thr_logit}  // tuong ung xac suat {thr:.2f} (chon tren tap tune)\n\n")
        f.write("typedef struct { uint8_t cin, cout, k, s, hi, wi, ho, wo, relu, sh; const int8_t *w; const int32_t *b; const uint16_t *m; } fomo_layer_t;\n\n")
        for i, L in enumerate(q):
            Wf = L["w"].reshape(-1)
            f.write(f"static const int8_t FOMO_W{i}[{Wf.size}] = {{" + ",".join(str(int(v)) for v in Wf) + "};\n")
            f.write(f"static const int32_t FOMO_B{i}[{L['b'].size}] = {{" + ",".join(str(int(v)) for v in L["b"]) + "};\n")
            f.write(f"static const uint16_t FOMO_M{i}[{L['M'].size}] = {{" + ",".join(str(int(v)) for v in L["M"]) + "};\n")
        f.write("\nstatic const fomo_layer_t FOMO_LAYERS[FOMO_NUM_LAYERS] = {\n")
        for i, l in enumerate(layers):
            f.write(f"    {{{l['cin']}, {l['cout']}, {l['k']}, {l['s']}, {l['hi']}, {l['wi']}, {l['ho']}, {l['wo']}, "
                    f"{l['relu']}, {l['sh']}, FOMO_W{i}, FOMO_B{i}, FOMO_M{i}}},\n")
        f.write("};\n")
    print("đã xuất RTL/C/vectors; ngưỡng logit mặc định", thr_logit)


if __name__ == "__main__":
    main()
