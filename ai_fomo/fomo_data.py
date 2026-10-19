"""Tạo nhãn FOMO (bản đồ tâm 12x12), tăng cường dữ liệu và đánh giá F1 theo tâm đối tượng."""
import numpy as np
import cv2

GRID = 12           # 96 / 8
IMG = 96
MIN_OBJ_PX = 6.0    # người nhỏ hơn ngưỡng (cạnh ngắn, đơn vị pixel ở 96x96) -> ô của nó bị bỏ qua khi tính loss/F1


def boxes_by_image(boxes, n):
    out = [[] for _ in range(n)]
    for b in boxes:
        out[int(b[0])].append(b[1:5])
    return [np.array(o, np.float32).reshape(-1, 4) for o in out]


LABEL_MODE = "center"   # "center": chỉ ô chứa tâm (Edge Impulse); "region": vùng lõi quanh tâm


def make_target(bxs, grid=GRID, img=IMG):
    """bxs: [K,4] chuẩn hóa. Trả về (target[grid,grid], mask[grid,grid]).
    center : ô chứa tâm hộp = 1 (đúng cách Edge Impulse huấn luyện FOMO).
    region : các ô có tâm nằm trong lõi hộp (1/3 rộng x 1/3 cao quanh tâm) = 1, phần còn lại
             của hộp bị bỏ qua khỏi loss (không phạt) -> cụm ổn định hơn với người lớn."""
    t = np.zeros((grid, grid), np.float32)
    m = np.ones((grid, grid), np.float32)
    if LABEL_MODE == "region":
        cc = (np.arange(grid) + 0.5) / grid
        for x1, y1, x2, y2 in bxs:
            if min(x2 - x1, y2 - y1) * img < MIN_OBJ_PX:
                continue
            inx = (cc >= x1) & (cc <= x2); iny = (cc >= y1) & (cc <= y2)
            m[np.ix_(iny, inx)] = 0.0
        for x1, y1, x2, y2 in bxs:
            w, h = (x2 - x1) * img, (y2 - y1) * img
            cx = min(grid - 1, max(0, int((x1 + x2) / 2 * grid)))
            cy = min(grid - 1, max(0, int((y1 + y2) / 2 * grid)))
            if min(w, h) < MIN_OBJ_PX:
                if t[cy, cx] == 0:
                    m[cy, cx] = 0.0
                continue
            mx, my = (x1 + x2) / 2, (y1 + y2) / 2
            hw, hh = (x2 - x1) / 6, (y2 - y1) / 6
            inx = (cc >= mx - hw) & (cc <= mx + hw); iny = (cc >= my - hh) & (cc <= my + hh)
            t[np.ix_(iny, inx)] = 1.0; m[np.ix_(iny, inx)] = 1.0
            t[cy, cx] = 1.0; m[cy, cx] = 1.0
        return t, m
    for x1, y1, x2, y2 in bxs:
        w, h = (x2 - x1) * img, (y2 - y1) * img
        cx = min(grid - 1, max(0, int((x1 + x2) / 2 * grid)))
        cy = min(grid - 1, max(0, int((y1 + y2) / 2 * grid)))
        if min(w, h) < MIN_OBJ_PX:
            if t[cy, cx] == 0:
                m[cy, cx] = 0.0        # đối tượng quá nhỏ: không phạt, không thưởng
        else:
            t[cy, cx] = 1.0
            m[cy, cx] = 1.0
    return t, m


def augment(img, bxs, rng):
    """img uint8 [96,96], bxs [K,4] chuẩn hóa -> ảnh/hộp đã tăng cường."""
    S = img.shape[0]
    bxs = bxs.copy()
    if rng.random() < 0.5:
        img = img[:, ::-1]
        if len(bxs):
            bxs[:, [0, 2]] = 1.0 - bxs[:, [2, 0]]
    # tỉ lệ + tịnh tiến ngẫu nhiên (affine)
    sc = rng.uniform(0.75, 1.25)
    tx, ty = rng.uniform(-0.15, 0.15, 2) * S
    M = np.array([[sc, 0, (1 - sc) * S / 2 + tx], [0, sc, (1 - sc) * S / 2 + ty]], np.float32)
    img = cv2.warpAffine(np.ascontiguousarray(img), M, (S, S), flags=cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_REFLECT_101)
    if len(bxs):
        p = bxs * S
        x1 = p[:, 0] * sc + M[0, 2]; x2 = p[:, 2] * sc + M[0, 2]
        y1 = p[:, 1] * sc + M[1, 2]; y2 = p[:, 3] * sc + M[1, 2]
        nb = np.stack([x1, y1, x2, y2], 1) / S
        cx, cy = (nb[:, 0] + nb[:, 2]) / 2, (nb[:, 1] + nb[:, 3]) / 2
        keep = (cx >= 0) & (cx < 1) & (cy >= 0) & (cy < 1)
        bxs = np.clip(nb[keep], 0, 1)
    # độ sáng / tương phản / gamma / nhiễu cảm biến
    f = img.astype(np.float32) / 255.0
    f = f ** rng.uniform(0.7, 1.4)
    f = (f - 0.5) * rng.uniform(0.6, 1.4) + 0.5 + rng.uniform(-0.2, 0.2)
    f += rng.normal(0, rng.uniform(0, 0.03), f.shape)
    if rng.random() < 0.2:
        f = cv2.GaussianBlur(f, (3, 3), rng.uniform(0.3, 1.0))
    img = np.clip(f * 255.0 + 0.5, 0, 255).astype(np.uint8)
    return img, bxs


# ----------------------------------------------------------------------------------------
# Hậu xử lý FOMO + đánh giá
# ----------------------------------------------------------------------------------------
def fomo_decode(prob, thr=0.5):
    """prob [12,12] -> danh sách (cx, cy, score) theo đơn vị ô; gộp các ô kề nhau (8-liên thông),
    tâm = trung bình có trọng số. Đây cũng là thuật toán chạy trên ESP32 (fomo_postprocess.c)."""
    on = (prob > thr).astype(np.uint8)
    n, lab = cv2.connectedComponents(on, connectivity=8)
    out = []
    for k in range(1, n):
        ys, xs = np.nonzero(lab == k)
        w = prob[ys, xs]
        out.append(((xs + 0.5) @ w / w.sum(), (ys + 0.5) @ w / w.sum(), float(w.max())))
    return out


def eval_f1(prob_maps, gt_boxes, thr=0.5, mode="dist", dist_cells=1.5):
    """Đếm TP/FP/FN theo tâm đối tượng.
    mode="dist" : TP nếu tâm dự đoán cách tâm GT <= dist_cells ô (khớp tham lam 1-1) — chặt.
    mode="inbox": TP nếu tâm dự đoán nằm TRONG hộp GT chưa khớp (gần tâm nhất) — như yêu cầu
                  "chấm đúng vào người", hợp với người lớn.
    GT nhỏ hơn MIN_OBJ_PX là 'bỏ qua': khớp với nó không tính TP cũng không tính FP."""
    if mode == "inbox":
        return _eval_inbox(prob_maps, gt_boxes, thr)
    tp = fp = fn = 0
    for prob, bxs in zip(prob_maps, gt_boxes):
        preds = fomo_decode(prob, thr)
        gts, ign = [], []
        for x1, y1, x2, y2 in bxs:
            c = ((x1 + x2) / 2 * GRID, (y1 + y2) / 2 * GRID)
            (ign if min(x2 - x1, y2 - y1) * IMG < MIN_OBJ_PX else gts).append(c)
        used = [False] * len(gts)
        for px, py, _ in sorted(preds, key=lambda p: -p[2]):
            best, bd = -1, 1e9
            for i, (gx, gy) in enumerate(gts):
                d = np.hypot(px - gx, py - gy)
                if not used[i] and d <= dist_cells and d < bd:
                    best, bd = i, d
            if best >= 0:
                used[best] = True
                tp += 1
            elif any(np.hypot(px - gx, py - gy) <= dist_cells for gx, gy in ign + gts):
                pass   # trùng người nhỏ / người đã khớp (đám đông) -> không tính FP
            else:
                fp += 1
        fn += used.count(False)
    p = tp / max(1, tp + fp)
    r = tp / max(1, tp + fn)
    f1 = 2 * p * r / max(1e-9, p + r)
    return dict(tp=tp, fp=fp, fn=fn, precision=p, recall=r, f1=f1)


def _eval_inbox(prob_maps, gt_boxes, thr):
    tp = fp = fn = 0
    for prob, bxs in zip(prob_maps, gt_boxes):
        preds = fomo_decode(prob, thr)
        g = np.asarray(bxs, np.float64).reshape(-1, 4) * GRID
        small = np.minimum(g[:, 2] - g[:, 0], g[:, 3] - g[:, 1]) * (IMG / GRID) < MIN_OBJ_PX
        used = np.zeros(len(g), bool)
        for px, py, _ in sorted(preds, key=lambda p: -p[2]):
            inside = (g[:, 0] <= px) & (px <= g[:, 2]) & (g[:, 1] <= py) & (py <= g[:, 3])
            cand = np.nonzero(inside & ~used & ~small)[0]
            if len(cand):
                c = cand[np.argmin(np.hypot((g[cand, 0] + g[cand, 2]) / 2 - px, (g[cand, 1] + g[cand, 3]) / 2 - py))]
                used[c] = True
                tp += 1
            elif not inside.any():
                fp += 1
        fn += int((~used & ~small).sum())
    p = tp / max(1, tp + fp)
    r = tp / max(1, tp + fn)
    return dict(tp=tp, fp=fp, fn=fn, precision=p, recall=r, f1=2 * p * r / max(1e-9, p + r))


def image_level(prob_maps, gt_boxes, thr=0.5):
    """Đánh giá mức ảnh (có người / không) — phù hợp bài toán báo động camera an ninh."""
    tp = fp = fn = tn = 0
    for prob, bxs in zip(prob_maps, gt_boxes):
        has = any(min(x2 - x1, y2 - y1) * IMG >= MIN_OBJ_PX for x1, y1, x2, y2 in bxs)
        pred = bool((prob > thr).any())
        neg = len(bxs) == 0                     # ảnh chỉ có người quá nhỏ: bỏ qua
        tp += has and pred; fn += has and not pred
        fp += neg and pred; tn += neg and not pred
    return dict(tp=tp, fp=fp, fn=fn, tn=tn,
                precision=tp / max(1, tp + fp), recall=tp / max(1, tp + fn),
                fpr=fp / max(1, fp + tn))
