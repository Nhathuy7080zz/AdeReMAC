"""
Tổng hợp số liệu cho báo cáo: với mỗi checkpoint, tính trên COCO val 2017
  - F1 theo tâm (ngưỡng chọn trên tập tune lúc huấn luyện)
  - mức ảnh (có người / không) tại ngưỡng đó
  - điểm vận hành kiểu camera an ninh: ngưỡng chọn TRÊN TẬP TUNE sao cho tỉ lệ báo động giả ≤ 10%,
    rồi báo cáo recall / FPR trên tập test
  python report_metrics.py runs/micro runs/micro_kd runs/teacher runs/mbv2 --train data/... --val data/...
"""
import argparse
import json
import os
import numpy as np
import torch

from fomo_models import FomoMicro, FomoMbv2, FomoTeacher, count_macs
from fomo_data import boxes_by_image, eval_f1, image_level, IMG, MIN_OBJ_PX


@torch.no_grad()
def predict(model, imgs, bs=256):
    out = []
    for s in range(0, len(imgs), bs):
        out.append(torch.sigmoid(model(torch.from_numpy(imgs[s:s + bs]).float().div(255).unsqueeze(1))).numpy())
    return np.concatenate(out)


def img_scores(probs, bxs):
    s = probs.reshape(len(probs), -1).max(1)
    pos = np.array([any(min(x2 - x1, y2 - y1) * IMG >= MIN_OBJ_PX for x1, y1, x2, y2 in b) for b in bxs])
    neg = np.array([len(b) == 0 for b in bxs])
    return s, pos, neg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--train", required=True)
    ap.add_argument("--val", required=True)
    ap.add_argument("--fpr", type=float, default=0.10)
    ap.add_argument("--out", default="results/summary.json")
    args = ap.parse_args()
    tr = np.load(args.train)
    n = len(tr["images"])
    rng = np.random.default_rng(0)                     # cùng cách tách tune như train_fomo.py
    tune_idx = rng.permutation(n)[: n // 10]
    t_imgs = tr["images"][tune_idx]
    t_bxs_all = boxes_by_image(tr["boxes"], n)
    t_bxs = [t_bxs_all[i] for i in tune_idx]
    va = np.load(args.val)
    v_imgs, v_bxs = va["images"], boxes_by_image(va["boxes"], len(va["images"]))
    summary = {}
    for run in args.runs:
        ck = torch.load(os.path.join(run, "model.pt"), map_location="cpu", weights_only=False)
        model = {"micro": FomoMicro, "mbv2": FomoMbv2, "teacher": lambda: FomoTeacher(pretrained=False)}[ck["model"]]()
        model.load_state_dict(ck["state"]); model.eval()
        macs, params = count_macs(model)
        thr = ck["threshold"]
        vp, tp = predict(model, v_imgs), predict(model, t_imgs)
        f = eval_f1(vp, v_bxs, thr)
        fin = eval_f1(vp, v_bxs, thr, mode="inbox")
        il = image_level(vp, v_bxs, thr)
        ts, tpos, tneg = img_scores(tp, t_bxs)
        thr_op = float(np.quantile(ts[tneg], 1 - args.fpr))          # chọn trên tune
        vs, vpos, vneg = img_scores(vp, v_bxs)
        big = np.array([len(b) > 0 and (np.minimum(b[:, 2] - b[:, 0], b[:, 3] - b[:, 1]) * IMG).max() >= 24 for b in v_bxs])
        op = dict(threshold=thr_op, recall=float((vs[vpos] > thr_op).mean()), fpr=float((vs[vneg] > thr_op).mean()),
                  recall_big=float((vs[big] > thr_op).mean()), n_big=int(big.sum()))
        summary[os.path.basename(run.rstrip("/"))] = dict(model=ck["model"], macs=macs, params=params, threshold=thr,
                                                          centroid=f, inbox=fin, image=il, operating_point=op)
        print(f"{run:22s} MAC {macs/1e6:5.2f}M  F1 {f['f1']:.3f} (P {f['precision']:.3f} R {f['recall']:.3f}) "
              f"F1-trong-hộp {fin['f1']:.3f} | "
              f"ảnh @thr: R {il['recall']:.3f} FPR {il['fpr']:.3f} | @FPR≤{args.fpr:.0%} (tune): R {op['recall']:.3f} "
              f"FPR {op['fpr']:.3f} R(người ≥24px) {op['recall_big']:.3f}")
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    json.dump(summary, open(args.out, "w"), indent=2)


if __name__ == "__main__":
    main()
