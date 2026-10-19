"""
Xuất tập COCO-person (đã gán nhãn sẵn) sang định dạng tải lên Edge Impulse Studio — KHÔNG cần gán nhãn tay.

  python export_edge_impulse.py --parquet "data/coco_parquet/train_0.parquet" --out ei_upload --max 3000
  edge-impulse-uploader --category split --directory ei_upload

Mỗi ảnh được thu nhỏ (giữ tỉ lệ) để cạnh dài = --long-side (mặc định 320 px) cho nhẹ khi tải lên;
bounding_boxes.labels (JSON) đặt cùng thư mục, đúng định dạng của edge-impulse-uploader:
  {"version": 1, "type": "bounding-box-labels", "boundingBoxes": {"x.jpg": [{"label","x","y","width","height"}]}}
Sau đó trong Studio: Image (Grayscale) 96x96 -> Object Detection -> FOMO (MobileNet V2 0.35).
"""
import argparse
import glob
import json
import os
import numpy as np
import cv2
import duckdb

PERSON = 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--parquet", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--max", type=int, default=3000, help="số ảnh tối đa (gói miễn phí của EI giới hạn dung lượng)")
    ap.add_argument("--long-side", type=int, default=320)
    ap.add_argument("--min-box-px", type=float, default=12.0, help="bỏ người nhỏ hơn ngưỡng (theo ảnh đã thu nhỏ)")
    ap.add_argument("--neg-ratio", type=float, default=0.3, help="tỉ lệ ảnh không có người (giảm báo động giả)")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    rng = np.random.default_rng(args.seed)
    labels, n_pos, n_neg = {}, 0, 0
    con = duckdb.connect()
    for f in sorted(glob.glob(args.parquet)):
        q = con.execute(f"SELECT image_id, image.bytes, width, height, objects.category, objects.bbox FROM '{f}'")
        while n_pos + n_neg < args.max:
            rows = q.fetchmany(256)
            if not rows:
                break
            for image_id, jpg, w, h, cats, bbs in rows:
                if n_pos + n_neg >= args.max:
                    break
                sc = args.long_side / max(w, h)
                persons = [b for c, b in zip(cats, bbs) if c == PERSON]
                boxes = [dict(label="person", x=int(b[0] * sc), y=int(b[1] * sc),
                              width=int((b[2] - b[0]) * sc), height=int((b[3] - b[1]) * sc))
                         for b in persons if min(b[2] - b[0], b[3] - b[1]) * sc >= args.min_box_px]
                if persons and not boxes:
                    continue            # chỉ có người quá nhỏ: bỏ cả ảnh để nhãn không mâu thuẫn
                if not persons:
                    if n_neg >= args.neg_ratio * args.max or rng.random() > 0.5:
                        continue
                img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                if img is None:
                    continue
                img = cv2.resize(img, (round(w * sc), round(h * sc)), interpolation=cv2.INTER_AREA)
                name = f"coco_{image_id}.jpg"
                cv2.imwrite(os.path.join(args.out, name), img, [cv2.IMWRITE_JPEG_QUALITY, 90])
                labels[name] = boxes
                n_pos += bool(boxes)
                n_neg += not boxes
    json.dump({"version": 1, "type": "bounding-box-labels", "boundingBoxes": labels},
              open(os.path.join(args.out, "bounding_boxes.labels"), "w"))
    print(f"{args.out}: {n_pos} ảnh có người, {n_neg} ảnh không có người, "
          f"{sum(len(v) for v in labels.values())} hộp person")


if __name__ == "__main__":
    main()
