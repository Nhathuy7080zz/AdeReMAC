# Hướng dẫn từng bước: đo AdaReMAC trên một mạng nơ-ron thật (MNIST)

M��c tiêu: trả lời bằng **số liệu thật** câu hỏi "thay bộ nhân chính xác bằng AdaReMAC thì
mạng nhận dạng còn đúng bao nhiêu, và đường chéo bị xấp xỉ ảnh hưởng thế nào lên dữ liệu thật?".
Toàn bộ chạy trên laptop, **chưa cần FPGA**.

## Trạng thái kiểm chứng (đọc trước)

| Thành phần | Đã chạy thật? | Kết quả |
|---|---|---|
| `mnist_data.py` | Có | tải được 60.000 + 10.000 ảnh MNIST thật |
| `approx_mul.py` | Có | ví dụ 5×3: Mode 0 → 14, Mode 2 → 15 |
| `verify_bitexact.py` (so với RTL) | Có | **PASS 5/5 cấu hình, 0 sai lệch trên 65.536 cặp INT8 có dấu** |
| `train_mnist_numpy.py` | Có | FP32 đạt **98,48%** sau 6 epoch |
| `qnet.py`, `eval_approx.py`, `explore_k.py` | Có | số liệu ở mục 7–8 |
| `train_mnist.py` (PyTorch) | **Chưa** | môi trường của mình không cài nổi PyTorch; cách tự kiểm ở Bước 2 |

## Bước 0 — Chuẩn bị

```
pip install numpy                 # bắt buộc
pip install torch                 # chỉ nếu dùng train_mnist.py (tận dụng GPU RTX 4050)
# iverilog (chỉ cho Bước 4): Linux: sudo apt install iverilog ; Windows: cài bản iverilog cho Windows
#                            rồi đảm bảo `iverilog` và `vvp` gọi được trong terminal
cd ai_eval
```

| File | Việc nó làm |
|---|---|
| `mnist_data.py` | tải/đọc MNIST (NumPy thuần, tự thử 3 mirror) |
| `train_mnist.py` / `train_mnist_numpy.py` | huấn luyện CNN FP32 → `model_fp32.npz` |
| `approx_mul.py` | mô hình bit-chính-xác của bộ nhân AdaReMAC dưới dạng bảng tra |
| `tb_signed_dump.v` + `verify_bitexact.py` | chứng minh bảng tra **trùng từng bit** với RTL |
| `qnet.py` | lượng tử hóa INT8 + suy luận số nguyên dùng bảng tra |
| `eval_approx.py` | đo độ chính xác, độ nhạy từng lớp, thống kê MAC trên dữ liệu thật |
| `explore_k.py` | quét số đường chéo bị xấp xỉ (khám phá, chưa có RTL) |

## Bước 1 — Dữ liệu

MNIST: ảnh xám 28×28, giá trị 0–255, 10 chữ số. `python mnist_data.py` phải in
`(60000, 28, 28) (60000,) (10000, 28, 28) (10000,) ...`. Ảnh được giữ nguyên ở dạng
`uint8`; việc chia 255 chỉ làm khi cần, để bước lượng tử hóa kiểm soát được từng bit.

## Bước 2 — Huấn luyện mạng FP32

M��ng (cố ý nhỏ và đơn giản, 5.258 tham số, 192.064 phép nhân mỗi ảnh):

```
conv1(1→8, 3×3) → ReLU → maxpool2 → conv2(8→16, 3×3) → ReLU → maxpool2 → flatten(400) → fc(400→10)
28×28 → 26×26 → 13×13 → 11×11 → 5×5
```

Lý do thiết kế: **không BatchNorm, không padding, ảnh đưa vào ở thang [0,1] (không trừ mean/chia std)**.
Nhờ vậy lượng tử hóa ở Bước 3 chỉ là phép chia scale, không phải gộp thêm lớp nào.

```
python train_mnist.py            # PyTorch (GPU nếu có)
python train_mnist_numpy.py      # bản NumPy thuần, khoảng 30–40 giây/epoch trên 1 core
```

Kết quả mình đo được (bản NumPy): epoch 1 → 97,02%, epoch 6 → **98,48%**.
Lưu ra `model_fp32.npz` (6 mảng: `conv1_w`, `conv1_b`, `conv2_w`, `conv2_b`, `fc_w`, `fc_b`).

**Tự kiểm khi dùng bản PyTorch (mình chưa chạy được bản này):** `eval_approx.py` ở Bước 7 in
dòng `FP32 : xx.xx%` tính bằng NumPy từ chính file `.npz`. Con số đó phải **bằng** test acc cuối cùng
`train_mnist.py` in ra (chênh không quá ~0,02%). Nếu lệch nhiều hoặc INT8 tụt về ~10% thì quy ước
trọng số bị sai: kiểm tra lại thứ tự `[out, in, kh, kw]` và thứ tự flatten kênh–hàng–cột.

## Bước 3 — Lượng tử hóa INT8 (hàm `quantize_model` trong `qnet.py`)

Ý tưởng: thay số thực bằng số nguyên 8-bit `q = round(x / scale)`, để mọi phép nhân trong mạng
là **nhân INT8 × INT8**, đúng thứ AdaReMAC xử lý. Chọn kiểu **đối xứng, zero-point = 0**
(giá trị 0 ↔ số nguyên 0) vì mạch dùng dấu-độ lớn.

1. **Ngõ vào:** `x_q = round(pixel/255 × 127)` (scale = 1/127). Pixel 200 → 100, pixel 255 → 127.
2. **Trọng số** (từng tensor): `s_w = max|w| / 127`, `w_q = clip(round(w/s_w), −127, 127)`.
   Ví dụ mô hình của mình: conv1 có `max|w| = 1,2035` nên `s_w = 0,009476`.
3. **Kích hoạt** (sau ReLU): chạy mạng FP32 trên 2.000 ảnh huấn luyện để đo biên độ, lấy
   **phân vị 99,99%** thay vì max để vài giá trị cực lớn không làm phí thang đo
   (conv2: max = 10,065 nhưng p99,99 = 7,784). `s_a = p99,99 / 127`.
4. **Bias:** `b_q = round(b / (s_x × s_w))`, kiểu int32, cộng thẳng vào bộ cộng dồn.
5. **Bộ cộng dồn** là int32 (khớp `ACC_W = 32` trong RTL). Tích INT8×INT8 lớn nhất 16.384
   nên cộng 72 hoặc 400 tích vẫn rất xa mức tràn.
6. **Requant** (đưa int32 về int8 cho lớp sau): hệ số thực `M = s_x × s_w / s_a` được xấp xỉ
   bằng số nguyên: `M ≈ M0 × 2^(−shift)`, rồi
   `out = clip((acc × M0 + 2^(shift−1)) >> shift, 0, 127)`  (clip ở 0 chính là ReLU).
   Ví dụ thật: conv1 có `M0 = 1176059091`, `shift = 39`, tức `M = 0,002139`;
   một `acc = 12345` cho `out = 26` (12345 × 0,002139 ≈ 26,4).
   Dùng số nguyên thuần để phần cứng sau này tái tạo **đúng từng bit** mà không cần số thực.
7. **Lớp cuối (fc)** giữ nguyên `acc` int32 làm logit, chỉ cần `argmax`.

Đây là PTQ (lượng tử hóa sau huấn luyện, không huấn luyện lại). INT8 chính xác đạt **98,51%**, ngang FP32 (98,48%).

## Bước 4 — Mô hình bit-chính-xác của bộ nhân (`approx_mul.py`) và đối chiếu với RTL

**Vì sao dùng được bảng tra:** tích `x × w` chỉ phụ thuộc đúng 4 thứ: hai toán hạng, `mode`, `thr`.
Hai toán hạng INT8 chỉ có 256 giá trị nên **mọi** kết quả có thể là một bảng 256×256 = 65.536 ô,
tính một lần cho mỗi cấu hình. Thay phép nhân trong mạng bằng một lần tra bảng là xong.

Cách tính một ô, bám đúng RTL (`adaremac_mul.v` + dấu-độ lớn của `adaremac_mac_lane.v`):

1. Lấy độ lớn `|x|`, `|w|` (−128 có độ lớn 128 vẫn nằm trong 8 bit không dấu).
2. Với 3 đường chéo thấp: `D0 = a0·b0`, `D1 = a0·b1 + a1·b0`, `D2 = a0·b2 + a1·b1 + a2·b0`
   (a_j, b_j là bit thứ j).
3. Phần dư `R = 2·D2 − D0`. Residual **tắt** → tích = `|x|·|w| − R`; **bật** → tích = `|x|·|w|`.
   Mode 0: luôn tắt. Mode 1: bật khi `D2 ≥ thr`. Mode 2: luôn bật.
4. Gắn dấu: âm nếu `x` và `w` khác dấu.

`python approx_mul.py` in ví dụ 5×3: Mode 0 → 14, Mode 1 (thr=1) → 15, Mode 2 → 15; và −5×3 → −14 / −15.

**Chứng minh bảng không sai dù chỉ 1 bit:** `verify_bitexact.py` biên dịch RTL thật bằng iverilog,
cho `adaremac_mac_lane` chạy hết 65.536 cặp (a, b) có dấu ở từng cấu hình, rồi so với bảng Python:

```
python verify_bitexact.py --rtl-dir ../rtl
mode0    mode=0 thr=0  số phép= 65536  sai tích=0  sai ENR=0  PASS
m1_thr3  mode=1 thr=3  ...             sai tích=0  sai ENR=0  PASS
m1_thr2  mode=1 thr=2  ...             sai tích=0  sai ENR=0  PASS
m1_thr1  mode=1 thr=1  ...             sai tích=0  sai ENR=0  PASS
exact    mode=2 thr=0  ...             sai tích=0  sai ENR=0  PASS
TỔNG: PASS
```

`ENR` là cờ "residual có bật cho phép nhân này không", cũng được so từng ô.
Nếu có `FAIL`: đừng tin mọi kết quả phía sau; so lại công thức `D0..D2` giữa RTL và `approx_mul.py`.
(Thư mục `standalone_tangnano_only/` cũng có sẵn hai file RTL này, dùng `--rtl-dir ../standalone_tangnano_only`.)

## Bước 5 — Suy luận số nguyên bằng bảng tra (`qnet.py`)

- `im2col` biến mỗi cửa sổ 3×3 thành một hàng, nên conv thành "mỗi hàng nhân với mỗi bộ lọc rồi cộng".
- `mac(...)`: với `lut=None` dùng nhân ma trận số nguyên (tham chiếu chính xác); với `lut=(slut, enr)`
  thì **từng phép nhân** được thay bằng `slut[x+128, w+128]`, sau đó cộng dồn int32.
  Đây đúng là phép nhân-cộng mà mảng MAC trên FPGA sẽ làm.
- Hai phép kiểm tra tự động (chạy đầu `eval_approx.py`): bảng *chính xác* và **Mode 2** đều phải
  cho logit **trùng bit** với nhân ma trận số nguyên. Nếu một trong hai sai, động cơ tra bảng có lỗi.

## Bước 6 — Cấu hình từng lớp

`forward_int(m, x_q, luts)` nhận một danh sách 3 bảng, mỗi lớp một bảng (`None` = chính xác).
Nhờ đó thử được ý tưởng của báo cáo gốc: lớp đầu dùng Mode 0, lớp cuối giữ chính xác.

## Bước 7 — Đo độ chính xác và thống kê (`eval_approx.py`)

```
python eval_approx.py --n 10000 --stats-n 500      # vài phút trên CPU
```

**Kết quả mình đo (10.000 ảnh test, 3 lớp cùng cấu hình):**

| Cấu hình | Độ chính xác | Chênh so với INT8 chính xác |
|---|---|---|
| FP32 | 98,48% | |
| INT8 chính xác | 98,51% | |
| Mode 0 | 98,50% | −0,01% |
| Mode 1, thr=3 | 98,50% | −0,01% |
| Mode 1, thr=2 | 98,53% | +0,02% |
| Mode 1, thr=1 | 98,52% | +0,01% |
| Mode 2 | 98,51% | 0,00% (trùng bit, đúng kỳ vọng) |

Độ nhạy từng lớp (một lớp Mode 0, các lớp khác chính xác): conv1 98,53%, conv2 98,51%, fc 98,53%.

**Cách đọc:** mọi chênh lệch chỉ ±1–3 ảnh trên 10.000. Đó là nhiễu, **không phải** "xấp xỉ làm mạng tốt hơn".
Kết luận đúng: với 3 đường chéo thấp (k = 3), **độ chính xác không đổi** trên mạng này.

**Thống kê trên dữ liệu thật** (500 ảnh; NMED chuẩn hóa theo 16.384 = 128×128, như firmware):

| Lớp | Số MAC/500 ảnh | % MAC có toán hạng bằng 0 | NMED dữ liệu, Mode 0 | % bật residual, thr=2 |
|---|---|---|---|---|
| conv1 | 24.336.000 | **79,54%** | 1,92e-5 | 3,57% |
| conv2 | 69.696.000 | **44,88%** | 4,49e-5 | 7,35% |
| fc | 2.000.000 | **38,57%** | 5,02e-5 | 8,89% |

Ba điều đáng chú ý:
1. NMED trên dữ liệu thật (≈2–5e-5) thấp hơn NMED khi quét đều 65.536 cặp (8,2e-5), vì dữ liệu thật
   có rất nhiều số 0 (nền ảnh đen, đầu ra ReLU = 0) và phép nhân với 0 không có sai số.
2. Khi hạ ngưỡng, tỉ lệ bật residual tăng đúng như thiết kế: thr=3 ≈ 0,5–0,9%, thr=2 ≈ 3,6–8,9%, thr=1 ≈ 13–36%.
3. **Phát hiện bất ngờ:** 39–80% phép nhân có một toán hạng bằng 0. Nếu sau này đo công suất, cơ hội
   *bỏ qua / khóa các phép nhân với 0* có thể lớn hơn nhiều so với xấp xỉ 3 đường chéo thấp.
   Đây là **quan sát từ dữ liệu**, chưa phải kết luận về năng lượng; năng lượng chỉ biết được khi đo trên phần cứng.

## Bước 8 — Quét số đường chéo bị xấp xỉ (`explore_k.py`, khám phá)

Vì k = 3 không làm mất độ chính xác, thử xấp xỉ nhiều đường chéo hơn để tìm điểm đánh đổi
(Mode 0, `q = 2^(k//2)`; **chưa có RTL cho k > 3**: phần dư có thêm số hạng `(2^i − q)·D_i` nên khối residual sẽ lớn hơn):

| k | q | Độ chính xác Mode 0 | Chênh vs INT8 | NMED dữ liệu |
|---|---|---|---|---|
| 3 | 2 | 98,50% | −0,01% | 3,8e-5 |
| 4 | 4 | 98,53% | +0,02% | 8,1e-5 |
| 5 | 4 | 98,47% | −0,04% | 4,3e-4 |
| 6 | 8 | 98,43% | −0,08% | 8,5e-4 |
| 7 | 8 | 98,24% | **−0,27%** | 2,4e-3 |
| 8 | 16 | 98,02% | **−0,49%** | 3,8e-3 |

Độ chính xác chỉ bắt đầu tụt rõ từ k = 7 (≈27 ảnh sai thêm), k = 8 mất ≈49 ảnh. Đây mới là vùng có
đánh đổi thật giữa độ chính xác và mức xấp xỉ.

## Bước 9 — Tinh chỉnh lại mạng với xấp xỉ (chưa viết code)

Chỉ cần khi chọn k lớn (từ k ≈ 7 trở lên). Cách làm tiêu chuẩn: khi huấn luyện lại, **lượt đi** dùng
bảng tra AdaReMAC, **lượt về** dùng gradient của phép nhân chính xác (straight-through estimator);
cho vài epoch với learning rate nhỏ. Với k = 3 không cần, vì độ chính xác đã không đổi.
Phần này mình chưa viết và chưa chạy thử.

## Bước 10 — Sang phần cứng (chưa làm)

Đã có sẵn: trọng số INT8, tham số requant, và bảng chuẩn để so từng bit. Còn thiếu: RTL bộ tăng tốc
(bộ nhớ trọng số/kích hoạt, nhiều lane `adaremac_mac_lane`, bộ điều khiển chạy từng lớp), xuất
trọng số ra file `.mem`, và vector chuẩn (ảnh + logit mong đợi) để testbench so với đầu ra RTL.

## Giới hạn cần nói thẳng

- Chỉ **một mạng nhỏ, một tập dữ liệu dễ** (MNIST). Kết luận "độ chính xác không đổi ở k = 3" cần đo lại
  cho mô hình phục vụ video thật (ví dụ phát hiện người) trước khi dùng trong báo cáo.
- `qnet.py` viết cứng cho đúng mạng này. Mô hình khác (DS-CNN, ResNet-8, MobileNet) cần mở rộng:
  conv depthwise, phép cộng residual, nhiều dạng lớp hơn.
- PTQ không huấn luyện lại; mô hình lượng tử hóa lúc huấn luyện (QAT) có thể cho kết quả khác.
- Mọi số ở đây là **độ chính xác và sai số**. Chưa có một con số **năng lượng** nào, vì năng lượng cần đo trên phần cứng.
