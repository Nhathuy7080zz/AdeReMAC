"""
Huấn luyện FOMO "person" trên COCO-person (đã gán nhãn sẵn) — tái hiện pipeline Edge Impulse:
  bbox -> nhãn tâm đối tượng trên lưới 1/8, loss cross-entropy có trọng số cho ô đối tượng.

  python train_fomo.py --model micro --train data/coco_person_train96.npz --val data/coco_person_val96.npz --out runs/micro
  python train_fomo.py --model mbv2  ...

Kết quả: runs/<tên>/model.pt, metrics.json (F1 theo tâm trên COCO val 2017 — tập test không dùng để chọn tham số).
"""
import argparse
import json
import os
import time
import numpy as np
import torch
import torch.nn.functional as F

from fomo_models import FomoMicro, FomoMbv2, FomoTeacher, count_macs
from fomo_data import boxes_by_image, make_target, augment, eval_f1, image_level


def load(path):
    d = np.load(path)
    imgs = d["images"]
    return imgs, boxes_by_image(d["boxes"], len(imgs))


def batch_iter(imgs, bxs, idx, bs, rng, aug):
    for s in range(0, len(idx), bs):
        sel = idx[s:s + bs]
        X, T, M = [], [], []
        for k in sel:
            im, b = (augment(imgs[k], bxs[k], rng) if aug else (imgs[k], bxs[k]))
            t, m = make_target(b)
            X.append(im); T.append(t); M.append(m)
        yield (torch.from_numpy(np.stack(X)).float().div_(255.0).unsqueeze(1),
               torch.from_numpy(np.stack(T)), torch.from_numpy(np.stack(M)))


@torch.no_grad()
def predict(model, imgs, bs=256):
    model.eval()
    out = []
    for s in range(0, len(imgs), bs):
        x = torch.from_numpy(imgs[s:s + bs]).float().div_(255.0).unsqueeze(1)
        out.append(torch.sigmoid(model(x)).numpy())
    return np.concatenate(out)


def best_threshold(probs, bxs):
    best = (0.5, -1, None)
    for thr in np.arange(0.3, 0.91, 0.05):
        r = eval_f1(probs, bxs, thr)
        if r["f1"] > best[1]:
            best = (float(thr), r["f1"], r)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", choices=["micro", "mbv2", "teacher"], default="micro")
    ap.add_argument("--distill", default=None, help="checkpoint mô hình thầy (FomoTeacher) để chưng cất tri thức")
    ap.add_argument("--kd-weight", type=float, default=1.0)
    ap.add_argument("--label", choices=["center", "region"], default="center")
    ap.add_argument("--init", default=None, help="khởi tạo từ checkpoint (tinh chỉnh)")
    ap.add_argument("--train", required=True)
    ap.add_argument("--val", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--bs", type=int, default=128)
    ap.add_argument("--lr", type=float, default=3e-3)
    ap.add_argument("--obj-weight", type=float, default=20.0,
                    help="trọng số ô có đối tượng (Edge Impulse gọi là object_weight)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--threads", type=int, default=2)
    args = ap.parse_args()
    import fomo_data
    fomo_data.LABEL_MODE = args.label
    torch.manual_seed(args.seed)
    torch.set_num_threads(args.threads)
    rng = np.random.default_rng(args.seed)
    os.makedirs(args.out, exist_ok=True)

    imgs, bxs = load(args.train)
    vimgs, vbxs = load(args.val)
    # 10% cuối của tập train làm tập "tune" để chọn ngưỡng; COCO val chỉ dùng để báo cáo
    n = len(imgs)
    perm = rng.permutation(n)
    n_tune = n // 10
    tune_idx, train_idx = perm[:n_tune], perm[n_tune:]

    model = {"micro": FomoMicro, "mbv2": FomoMbv2, "teacher": FomoTeacher}[args.model]()
    if args.init:
        model.load_state_dict(torch.load(args.init, map_location="cpu", weights_only=False)["state"])
    teacher = None
    if args.distill:
        teacher = FomoTeacher(pretrained=False)
        teacher.load_state_dict(torch.load(args.distill, map_location="cpu", weights_only=False)["state"])
        teacher.eval()
    macs, params = count_macs(model)
    print(f"model={args.model} MAC={macs/1e6:.2f}M params={params} train={len(train_idx)} tune={n_tune} test={len(vimgs)}")
    opt = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4)
    steps = args.epochs * ((len(train_idx) + args.bs - 1) // args.bs)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=args.lr, total_steps=steps, pct_start=0.1)

    log = []
    best_f1, best_state = -1, None
    for ep in range(args.epochs):
        model.train()
        t0, tot, nb = time.time(), 0.0, 0
        idx = rng.permutation(train_idx)
        for X, T, M in batch_iter(imgs, bxs, idx, args.bs, rng, aug=True):
            logit = model(X)
            w = M * (1.0 + (args.obj_weight - 1.0) * T)
            loss = (F.binary_cross_entropy_with_logits(logit, T, reduction="none") * w).sum() / w.sum()
            if teacher is not None:
                with torch.no_grad():
                    soft = torch.sigmoid(teacher(X))
                wk = M * (1.0 + (args.obj_weight - 1.0) * torch.maximum(T, soft))
                loss = loss + args.kd_weight * (F.binary_cross_entropy_with_logits(logit, soft, reduction="none") * wk).sum() / wk.sum()
            opt.zero_grad()
            loss.backward()
            opt.step()
            sched.step()
            tot += loss.item(); nb += 1
        tp = predict(model, imgs[tune_idx])
        thr, f1, r = best_threshold(tp, [bxs[k] for k in tune_idx])
        log.append(dict(epoch=ep + 1, loss=tot / nb, tune_f1=f1, thr=thr, sec=time.time() - t0))
        print(f"ep {ep+1:2d} loss {tot/nb:.4f} tune F1 {f1:.3f} (thr {thr:.2f}) P {r['precision']:.3f} R {r['recall']:.3f} [{time.time()-t0:.0f}s]", flush=True)
        if f1 > best_f1:
            best_f1, best_thr = f1, thr
            best_state = {k: v.clone() for k, v in model.state_dict().items()}

    model.load_state_dict(best_state)
    vp = predict(model, vimgs)
    test = eval_f1(vp, vbxs, best_thr)
    test_inbox = eval_f1(vp, vbxs, best_thr, dist_cells=1.0)
    img_lvl = image_level(vp, vbxs, best_thr)
    res = dict(model=args.model, label=args.label, macs=macs, params=params, threshold=best_thr, tune_f1=best_f1,
               test_coco_val2017=test, test_dist1cell=test_inbox, test_image_level=img_lvl,
               n_train=int(len(train_idx)), n_tune=int(n_tune), n_test=int(len(vimgs)),
               obj_weight=args.obj_weight, epochs=args.epochs, log=log)
    torch.save(dict(state=best_state, model=args.model, threshold=best_thr), os.path.join(args.out, "model.pt"))
    json.dump(res, open(os.path.join(args.out, "metrics.json"), "w"), indent=2)
    print(json.dumps({k: v for k, v in res.items() if k != "log"}, indent=2))


if __name__ == "__main__":
    main()
