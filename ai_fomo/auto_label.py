"""
Gán nhãn TỰ ĐỘNG ảnh chụp từ chính camera của nhóm bằng mô hình phát hiện đã huấn luyện sẵn trên COCO
(torchvision Faster R-CNN MobileNetV3-FPN, lớp "person") — dùng để tinh chỉnh FOMO cho đúng bối cảnh lắp đặt
mà không phải vẽ khung bằng tay. Kết quả: bounding_boxes.labels (định dạng Edge Impulse) + file .npz cho train_fomo.py.

  python auto_label.py --images captures/ --out captures_labeled --score 0.6

Ảnh lấy từ ESP32: nút "Chụp ảnh" trên web hoặc http://192.168.4.1/capture (mỗi lần tải 1 khung JPEG). Nên kiểm tra nhanh
bằng mắt vài chục ảnh trong captures_labeled/preview/ (khung xanh) trước khi huấn luyện.
"""
import argparse
import glob
import json
import os
import numpy as np
import cv2
import torch
import torchvision

PERSON = 1   # chỉ số lớp "person" trong COCO của torchvision (0 = background)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", required=True, help="thư mục ảnh .jpg/.png")
    ap.add_argument("--out", required=True)
    ap.add_argument("--score", type=float, default=0.6, help="ngưỡng tin cậy của mô hình gán nhãn")
    ap.add_argument("--size", type=int, default=96, help="kích thước ảnh xám xuất ra .npz")
    args = ap.parse_args()
    os.makedirs(os.path.join(args.out, "preview"), exist_ok=True)
    model = torchvision.models.detection.fasterrcnn_mobilenet_v3_large_fpn(weights="DEFAULT").eval()
    files = sorted(glob.glob(os.path.join(args.images, "*.jpg")) + glob.glob(os.path.join(args.images, "*.png")))
    labels, imgs, boxes = {}, [], []
    for k, path in enumerate(files):
        bgr = cv2.imread(path)
        if bgr is None:
            continue
        h, w = bgr.shape[:2]
        x = torch.from_numpy(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)).permute(2, 0, 1).float() / 255
        with torch.no_grad():
            det = model([x])[0]
        keep = (det["labels"] == PERSON) & (det["scores"] >= args.score)
        bb = det["boxes"][keep].numpy()
        name = os.path.basename(path)
        labels[name] = [dict(label="person", x=int(b[0]), y=int(b[1]), width=int(b[2] - b[0]), height=int(b[3] - b[1]))
                        for b in bb]
        idx = len(imgs)
        imgs.append(cv2.resize(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), (args.size, args.size), interpolation=cv2.INTER_AREA))
        for b in bb:
            boxes.append([idx, b[0] / w, b[1] / h, b[2] / w, b[3] / h])
            cv2.rectangle(bgr, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 255, 0), 2)
        cv2.imwrite(os.path.join(args.out, "preview", name), bgr)
        cv2.imwrite(os.path.join(args.out, name), cv2.imread(path))
        if k % 50 == 0:
            print(f"{k}/{len(files)}", flush=True)
    json.dump({"version": 1, "type": "bounding-box-labels", "boundingBoxes": labels},
              open(os.path.join(args.out, "bounding_boxes.labels"), "w"))
    np.savez_compressed(os.path.join(args.out, f"labeled{args.size}.npz"), images=np.stack(imgs).astype(np.uint8),
                        boxes=np.array(boxes, np.float32).reshape(-1, 5), ids=np.arange(len(imgs)))
    print(f"xong: {len(imgs)} ảnh, {len(boxes)} người -> {args.out}")


if __name__ == "__main__":
    main()
