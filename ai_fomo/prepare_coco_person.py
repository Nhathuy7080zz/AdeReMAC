"""
Chuẩn bị tập dữ liệu ĐÃ GÁN NHÃN SẴN cho FOMO "person" từ COCO 2017 (không gán nhãn thủ công).

Nguồn: COCO 2017 (Lin et al., ECCV 2014), bản parquet trên Hugging Face
       https://huggingface.co/datasets/detection-datasets/coco  (bbox dạng [x1, y1, x2, y2], lớp 0 = person)

Đầu ra (mỗi split một file .npz):
  images : uint8 [N, S, S]      ảnh xám đã co (squash) về S x S (mặc định 96 x 96, giống Edge Impulse)
  boxes  : float32 [M, 5]       [img_idx, x1, y1, x2, y2] chuẩn hóa 0..1 (chỉ lớp person)
  ids    : int64 [N]            COCO image_id để truy vết

Cách chạy:
  python prepare_coco_person.py --parquet "data/coco_parquet/train_*.parquet" --out data/coco_person_train96.npz
  python prepare_coco_person.py --parquet "data/coco_parquet/val_*.parquet"   --out data/coco_person_val96.npz --neg-ratio 1.0

Cần: duckdb (đọc parquet), opencv-python, numpy.
"""
import argparse
import glob
import numpy as np
import cv2
import duckdb

PERSON = 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--parquet", required=True, help="glob tới các file parquet COCO")
    ap.add_argument("--out", required=True)
    ap.add_argument("--size", type=int, default=96)
    ap.add_argument("--neg-ratio", type=float, default=0.5,
                    help="số ảnh KHÔNG có người giữ lại / số ảnh có người (ảnh âm giúp giảm báo động giả)")
    ap.add_argument("--min-box-px", type=float, default=4.0,
                    help="bỏ qua (không tính) ảnh chỉ có người nhỏ hơn ngưỡng này sau khi co về S x S")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    files = sorted(glob.glob(args.parquet))
    assert files, "không tìm thấy parquet"
    rng = np.random.default_rng(args.seed)
    S = args.size
    imgs, boxes, ids = [], [], []
    n_pos = n_neg = n_skip = 0
    con = duckdb.connect()
    for f in files:
        q = con.execute(
            f"SELECT image_id, image.bytes, width, height, objects.category, objects.bbox FROM '{f}'")
        while True:
            rows = q.fetchmany(256)
            if not rows:
                break
            for image_id, jpg, w, h, cats, bbs in rows:
                persons = [b for c, b in zip(cats, bbs) if c == PERSON]
                if persons:
                    # chỉ giữ ảnh có ít nhất 1 người đủ lớn để FOMO 1/8 nhìn thấy
                    big = [b for b in persons
                           if (b[2] - b[0]) / w * S >= args.min_box_px and (b[3] - b[1]) / h * S >= args.min_box_px]
                    if not big:
                        n_skip += 1
                        continue
                    is_pos = True
                else:
                    if rng.random() > args.neg_ratio * 0.9:   # COCO có ~45% ảnh có người
                        continue
                    is_pos = False
                arr = np.frombuffer(jpg, np.uint8)
                g = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
                if g is None:
                    continue
                g = cv2.resize(g, (S, S), interpolation=cv2.INTER_AREA)
                k = len(imgs)
                imgs.append(g)
                ids.append(image_id)
                if is_pos:
                    n_pos += 1
                    for b in persons:   # giữ cả người nhỏ: lúc train sẽ được che (ignore) khỏi loss
                        boxes.append([k, b[0] / w, b[1] / h, b[2] / w, b[3] / h])
                else:
                    n_neg += 1
        print(f"{f}: pos={n_pos} neg={n_neg} skip_small_only={n_skip}", flush=True)
    np.savez_compressed(args.out,
                        images=np.stack(imgs).astype(np.uint8),
                        boxes=np.array(boxes, np.float32).reshape(-1, 5),
                        ids=np.array(ids, np.int64))
    print(f"saved {args.out}: {len(imgs)} images ({n_pos} có người, {n_neg} không có người), {len(boxes)} hộp person")


if __name__ == "__main__":
    main()
