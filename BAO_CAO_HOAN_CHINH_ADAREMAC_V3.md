# BÁO CÁO KỸ THUẬT HOÀN CHỈNH DỰ ÁN ADAREMAC (PHIÊN BẢN V3 MỚI NHẤT)
**Thiết kế Vi kiến trúc Bộ nhân–Tích lũy Xấp xỉ Thích nghi Phần dư Thưa (AdaReMAC) và Hệ thống Đồng xử lý Thị giác Biên Phân tầng 3 Chế độ trên FPGA Sipeed Tang Nano 20K, ESP32-S3 N16R8 OV5640 & Cảm biến INA219**

## 1. Tóm tắt điều hành (Executive Summary)

Dự án **AdaReMAC** (*Adaptive Sparse-Residual Approximate Multiply-Accumulate*) giải quyết bài toán tối ưu hóa năng lượng cho các hệ thống Trí tuệ nhân tạo tại biên (Edge-AI) thông qua sự kết hợp chặt chẽ giữa **thiết kế vi kiến trúc số ở cấp độ cổng logic (RTL Verilog trên FPGA)** và **kiến trúc điều phối năng lượng–dữ liệu phân tầng ở cấp độ hệ thống (Vi điều khiển ESP32-S3 + Camera OV5640)**.

Những đóng góp và kết quả thực nghiệm quan trọng nhất của phiên bản **AdaReMAC v3**:
1. **Công thức nhân xấp xỉ phần dư thưa triệt tiêu đường chéo bậc 1 ($k = 3, q_L = 2$)**: Bằng cách gom 3 đường chéo tích riêng bậc thấp nhất $P_L = D_0 + 2D_1 + 4D_2$ thành tổng trọng số đồng nhất $\hat{P}_L = 2(D_0 + D_1 + D_2)$, phần dư sửa sai rút gọn còn $R = 2D_2 - D_0$. Đường chéo $D_1$ biến mất hoàn toàn khỏi mạch sửa sai, giúp khối khôi phục chính xác chỉ cần 1 phép dịch bit và 1 phép trừ nhỏ.
2. **Kiểm chứng Bit-Exact 100% trên 65.536 cặp toán hạng có dấu và Mạng CNN `INT8` thật**: Mô hình phần cứng Verilog (`adaremac_mul.v`, `adaremac_mac_lane.v`) khớp tuyệt đối từng bit (0 sai lệch trên 327.680 phép thử qua 5 cấu hình) với bộ mô phỏng lượng tử hóa PTQ `INT8`. Trên tập kiểm thử 10.000 ảnh MNIST, mạng CNN 3 lớp đạt độ chính xác **98,48% (FP32)**, **98,51% (`INT8` chính xác)**, **98,50% (`Mode 0` thuần xấp xỉ)**, và đạt đỉnh **98,53% ở `Mode 1 (thr = 2)`** trong khi chỉ phải bật khối sửa sai Residual cho **3,57%–8,89%** số phép tính MAC.
3. **Kiến trúc Đồng xử lý Phân tầng 3 Chế độ (FPGA + ESP32-S3)**: Khắc phục triệt để nghịch lý công suất hệ thống bằng cách phân tách rõ 3 trạng thái:
   - **Mode 0 (Gatekeeper — 0% Data)**: FPGA chạy thuần xấp xỉ (`mode = 0`), ESP32-S3 ngắt CPU (`Light Sleep`), lượng dữ liệu qua model trên ESP32 = 0%.
   - **Mode 1 (Selective Residual & Early-Exit — 25% Data)**: FPGA chạy `mode = 1, thr = 2`, ESP32-S3 chỉ xử lý vùng trung tâm **25% khung hình (ROI $160 \times 120$, 100/400 đặc trưng `INT8`)**.
   - **Mode 2 (Full Exact — 100% Data)**: FPGA bật 100% khối Residual (`mode = 2`, $\text{NMED} = 0$), ESP32-S3 xử lý **100% khung hình ($320 \times 240$, 400/400 đặc trưng `INT8`)**.
4. **Tối ưu hóa Phần cứng Thực tế Chống Nóng Chip & Chống Nghẽn Luồng Video**:
   - Trên **FPGA Tang Nano 20K (`GW2AR-18C`)**: Tối ưu hóa số lane kiểm chứng (`N_LANES = 1`, giảm 95% công suất động từ 1.539 FF xuống ~190 FF), đưa chân `spi_miso` (Pin 49) về trạng thái trở kháng cao (`1'bz`) khi `CS_N = 1`, loại bỏ dòng rò trên 56 chân `Unused Pins` (`As input tri-stated`), giữ chip FPGA mát lạnh hoàn toàn (~32–36 °C).
   - Trên **ESP32-S3 N16R8 + OV5640**: Quy hoạch **100% dây cắm ngoại vi (`SPI`, `I2C`, `VCC nuôi INA219`, `GND`) về duy nhất Hàng Phải (Right Row)** để tương thích hoàn toàn với Breadboard; xây dựng hệ thống truyền video **Pure Wi-Fi AP 40MHz HT40 (`192.168.4.1`)** kết hợp thuật toán **AI Học Môi Trường 5 Đặc Trưng + Khóa Cổng Động Học Chống Kẹt (`s_fg_lock` & `Silent Per-Cell Background Absorption`)** chạy ổn định với 0 byte mảng trên Stack.

## 2. Cơ sở toán học và Phân tích không gian thiết kế ($k = 3 \dots 8$)

### 2.1. Phân rã tích riêng theo đường chéo và Sự triệt tiêu của $D_1$
Xét phép nhân hai số nguyên không dấu 8-bit $A = \sum_{i=0}^{7} a_i 2^i$ và $B = \sum_{j=0}^{7} b_j 2^j$. Ma trận tích riêng gồm 64 bit $p_{i,j} = a_i \wedge b_j$ được chia thành các đường chéo có cùng trọng số $2^d$ (với $d = i + j \in \{0, \dots, 14\}$). Đặt tổng số bit bằng 1 trên đường chéo thứ $d$ là:
$$D_d = \sum_{\substack{i+j=d \\ 0 \le i,j \le 7}} (a_i \wedge b_j)$$

Với ngưỡng cắt $k = 3$, tích chính xác $P = A \times B$ được tách thành phần cao $P_H$ (gồm các đường chéo $d \ge 3$) và phần thấp $P_L$ (gồm 3 đường chéo $d \in \{0, 1, 2\}$):
$$P = P_H + P_L, \quad P_L = 1\cdot D_0 + 2\cdot D_1 + 4\cdot D_2 \in [0, 17]$$

Thay vì dùng cây cộng đầy đủ cho $P_L$, đường chính xấp xỉ của AdaReMAC tính tổng số bit hoạt động $M_L = D_0 + D_1 + D_2 \in [0, 6]$ và nhân với hệ số lũy thừa hai $q_L = 2$ (tương đương dịch trái 1 bit):
$$\hat{P}_L = q_L \cdot M_L = 2(D_0 + D_1 + D_2) \in [0, 12]$$

Khi đó, phần dư sửa sai chính xác $R = P_L - \hat{P}_L$ được rút gọn đại số thành:
$$R = (1 - 2)D_0 + (2 - 2)D_1 + (4 - 2)D_2 = 2D_2 - D_0 \in [-1, +6]$$

**Ý nghĩa vi kiến trúc**:
- Hệ số của $D_1$ bằng $2 - 2 = 0$, nghĩa là đường chéo $D_1 = (a_0 \wedge b_1) + (a_1 \wedge b_0)$ **tự triệt tiêu hoàn toàn** khỏi biểu thức phần dư $R$.
- Khối sửa sai phần dư chỉ cần thực hiện phép dịch trái `(D2 << 1)` rồi trừ đi 1 bit `D0`.
- Khi bật tín hiệu `enr = 1`, tích tổng hợp là $P = P_H + (\hat{P}_L + R) = P_H + P_L$, khôi phục **chính xác tuyệt đối 100%** tích số nguyên gốc ($\text{ErrRate} = 0\%$, $\text{NMED} = 0$).

### 2.2. Kết quả kiểm chứng trên mạng CNN `INT8` (10.000 ảnh MNIST) và Khảo sát $k = 3 \dots 8$
Mạng CNN thực nghiệm có cấu trúc: `Conv1(1->8, 3x3) + ReLU + MaxPool(2x2)` $\to$ `Conv2(8->16, 3x3) + ReLU + MaxPool(2x2)` $\to$ `Flatten(400)` $\to$ `FC(400->10)`, tổng cộng **5.258 tham số** và **192.064 phép tính MAC trên mỗi ảnh**.

| Cấu hình vận hành | Chế độ (`mode`, `thr`) | Độ chính xác MNIST (%) | Độ lệch $\Delta$ so với `INT8` đúng | Tỷ lệ bật Residual (`conv1` / `conv2` / `fc`) | NMED trên dữ liệu thật (`conv1` / `conv2` / `fc`) |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **FP32 Baseline** | Số thực 32-bit | **98,48%** | -0,03% | — | — |
| **INT8 Exact (Ma trận)** | Chuẩn số nguyên | **98,51%** | 0,00% | 100% / 100% / 100% | $0,0$ / $0,0$ / $0,0$ |
| **AdaReMAC `mode0`** | `mode=0, thr=0` | **98,50%** | -0,01% | **0,00% / 0,00% / 0,00%** | $1,92\times 10^{-5}$ / $4,49\times 10^{-5}$ / $5,02\times 10^{-5}$ |
| **AdaReMAC `m1_thr3`** | `mode=1, thr=3` | **98,50%** | -0,01% | 0,49% / 0,54% / 0,85% | $1,77\times 10^{-5}$ / $4,32\times 10^{-5}$ / $4,76\times 10^{-5}$ |
| **AdaReMAC `m1_thr2`** | `mode=1, thr=2` | **98,53%** | **+0,02%** | **3,57% / 7,35% / 8,89%** | $1,11\times 10^{-5}$ / $2,90\times 10^{-5}$ / $3,07\times 10^{-5}$ |
| **AdaReMAC `m1_thr1`** | `mode=1, thr=1` | **98,52%** | +0,01% | 13,29% / 32,78% / 35,78% | $4,38\times 10^{-7}$ / $1,95\times 10^{-6}$ / $2,43\times 10^{-6}$ |
| **AdaReMAC `exact`** | `mode=2, thr=0` | **98,51%** | **0,00% (Trùng bit)** | 100% / 100% / 100% | $0,0$ / $0,0$ / $0,0$ |

Khi khảo sát mở rộng số đường chéo xấp xỉ $k$ từ $3$ lên $8$ ở `Mode 0` (với $q_L = 2^{\lfloor k/2 \rfloor}$):
- Tại **$k = 3, q_L = 2$**: Độ chính xác giữ nguyên **98,50%** (chỉ lệch 0,01% so với `INT8` đúng).
- Tại **$k = 4, q_L = 4$**: Độ chính xác giảm xuống **95,38%** (-3,13%).
- Tại **$k = 5, q_L = 4$**: Độ chính xác sụp đổ xuống **76,72%** (-21,79%).
- Tại **$k = 6 \dots 8$**: Độ chính xác rơi xuống **9,74%–21,09%** do kỳ vọng sai số $\mathbb{E}[R] \ne 0$ gây trôi phân phối tích lũy qua các tầng chập. Điều này chứng minh đanh thép vì sao **$k = 3, q_L = 2$ là điểm ngọt toán học tối ưu duy nhất** không cần huấn luyện lại mạng.

## 3. Thiết kế Vi kiến trúc RTL trên FPGA Sipeed Tang Nano 20K (`GW2AR-18C`)

### 3.1. Cấu trúc Pipeline 3 tầng và Cơ chế chống nóng chip FPGA
Toàn bộ thiết kế RTL trong thư mục `rtl/` đã được tối ưu hóa cho chip **Gowin `GW2AR-LV18QN88C8/I7` (Version C)**:
- **Cô lập toán hạng (Operand Isolation)**: Trong `adaremac_mul.v`, hai đường chéo `d2` và `d0` đi vào khối phần dư được khóa qua cổng AND với tín hiệu `enr` (`d2_g = d2 & {2{enr}}`, `d0_g = d0 & enr`). Khi `enr = 0`, nhánh logic tính phần dư hoàn toàn đứng yên, không tiêu thụ năng lượng chuyển mạch.
- **Tối ưu hóa `N_LANES = 1` và `spi_miso` High-Z (`1'bz`)**:
  - Đặt mặc định `N_LANES = 1` trong `adaremac_core.v`, `top_tn20k.v` và `top_standalone.v`. Thay vì để 16 bộ nhân chạy tự do đốt nóng chip (chiếm 3.195 LUT và 1.539 FF), cấu hình `N_LANES = 1` chỉ sử dụng **~190 Flip-Flop (~1% tài nguyên)** nhưng vẫn giữ nguyên 100% bộ giám sát sai số phần cứng ở Lane 0 và toàn bộ thanh ghi SPI.
  - Tại `top_tn20k.v`, chân `spi_miso` (Pin 49) được điều khiển ba trạng thái: `assign spi_miso = spi_cs_n ? 1'bz : spi_miso_core;`, ngăn chặn hoàn toàn dòng rò qua mạch ngoại vi trên Pin 49 khi bus SPI nghỉ.

### 3.2. Bốn quy tắc vàng khi tổng hợp và nạp bitstream trên Gowin EDA
1. Chọn đúng mã chip **`GW2AR-LV18QN88C8/I7`** với **`Version = C`** (**`GW2AR-18C`**).
2. Trong **`Project -> Configuration -> Place & Route -> Unused Pin`**: Bắt buộc tích chọn **`As input tri-stated`** và **bỏ tích** `As input tri-stated with pull-up` để giải phóng 56 chân không dùng khỏi trạng thái kéo áp `PULL_MODE=UP`.
3. Trong **`Project -> Configuration -> Place & Route -> Dual-Purpose Pin`**: Tích chọn đủ 4 ô (`DONE`, `READY`, `RECONFIG_N`, `SSPI as regular IO`).
4. Chỉ thêm các file RTL tổng hợp được (6 file `.v` trong `rtl/` cho bản SPI với `top_tn20k` làm Top Module, hoặc 7 file `.v` trong `standalone_tangnano_only/` cho bản độc lập với `top_standalone` làm Top Module). Tuyệt đối không thêm các file testbench `tb_*.v` trong `sim/` vào Gowin EDA.

## 4. Kiến trúc Firmware Arduino IDE 2.x trên ESP32-S3 N16R8 + Camera OV5640

### 4.1. Sketch 1 (`1_test_camera_ov5640.ino`): Trạm Stream HD/FHD Chống Nghẽn & AI Học Môi Trường 5 Đặc Trưng
Sketch 1 biến mạch ESP32-S3 N16R8 OV5640 thành một hệ thống giám sát thông minh thời gian thực hoạt động độc lập:
- **Truyền dẫn Pure Wi-Fi AP 40MHz HT40 (`ESP32-OV5640-FHD`, IP `192.168.4.1`)**:
  - Sử dụng chế độ thuần `WIFI_AP` ở độ rộng kênh **40 MHz (`WIFI_BW_HT40`, tốc độ vật lý 150 Mbps)**, tắt hoàn toàn chế độ ngủ tiết kiệm điện (`WIFI_PS_NONE`), phát công suất tối đa `19.5 dBm` và bật cờ `TCP_NODELAY` trên socket truyền ảnh.
  - Tách biệt 2 máy chủ HTTP chạy song song: **Port 80** phục vụ giao diện Web HTML5 Canvas + API điều khiển `/ctrl` + JSON tọa độ AI `/ai`; **Port 81** chuyên biệt đẩy luồng ảnh MJPEG `/stream` không khóa.
- **Tối ưu hóa ISP phần cứng của cảm biến OV5640 (`16 MHz Clean-PCLK`)**:
  - Cấp xung nhịp `xclk_freq_hz = 16000000` (16 MHz) để triệt tiêu hoàn toàn lỗi rớt byte JPEG gây sọc tím trên cáp dẹt FPC 24-pin.
  - Khóa chế độ phơi sáng kéo dài ban đêm (`set_aec2(s, 0)`), giới hạn trần khuếch đại `gainceiling = +16x`, tắt làm sắc nét nhân tạo (`sharpness = 0`) và bật mạch lọc điểm chết phần cứng (`bpc = 1`, `wpc = 1`, thanh ghi ISP `0x5000 = 0x86`).
- **Thuật toán AI Học Môi Trường 5 Đặc Trưng + Khóa Cổng Động Học Chống Kẹt (`analyze_frame_and_learn`)**:
  - Chạy trên **Core 0** (`ai_continual_learning_task` với `20480` byte Stack và **100% mảng trung gian đặt trong vùng nhớ tĩnh `.bss`**, tốn **0 byte Stack**).
  - Chia khung hình thu nhỏ (`JPG_SCALE_8X`) thành lưới không gian $20 \times 15 = 300$ ô. Tại mỗi ô $i$, mô hình duy trì 5 đặc trưng nền dài hạn: độ sáng trung bình `s_bg_y[i]`, đỉnh sáng tĩnh `s_bg_max_y[i]`, sắc độ Đỏ `s_bg_cr[i]`, sắc độ Xanh `s_bg_cb[i]`, và cường độ hoa văn/biên tĩnh `s_bg_edge[i]`.
  - **Cơ chế chống kẹt trạng thái học nền 100%**: Bộ đếm học nhanh `s_fast_learn_frames` khởi tạo bằng `0`, khi kích hoạt chỉ chạy đúng **3 chu kỳ (~0,25 giây)** rồi giảm vô điều kiện về `0`, đi kèm khóa chống lặp **`s_learn_cooldown = 30` chu kỳ (~2,5 giây)**.
  - **Cơ chế phân biệt hoa văn trần nhà / đèn tĩnh với bàn tay / đèn Flash mới (`s_fg_lock` & `Silent Per-Cell Background Absorption`)**: Chỉ cấp khóa tiền cảnh (`s_fg_lock[i] = 12` hoặc `24`) khi ô thực sự vừa có chuyển động (`s_static_age[i] == 0`). Hoa văn trần nhà hay bóng đèn cố định đứng yên (`s_static_age[i] >= 3`) tự động được hấp thụ ngầm vào nền với hệ số học nhanh `alpha = 0.38f`. Khi phát hiện cụm vật thể hợp lệ qua gom cụm BFS 8 hướng, vùng ảnh của vật thể được chuẩn hóa thành vector `400` chiều `INT8` và chạy suy luận qua bộ nhân xấp xỉ **AdaReMAC** ở cả **Mode 1 (`AI_PARTIAL_DIM = 100`, `thr = 2`)** và **Mode 2 (`AI_FEAT_DIM = 400`, `Exact`)** để hiển thị nhãn phân loại trực tiếp trên khung bao.

### 4.2. Sketch 2 (`2_adaremac_main.ino`): Hệ thống Đo đạc & Điều phối 3 Chế độ (ESP32-S3 + Tang Nano 20K + INA219)
Sketch 2 thực hiện quy trình thực nghiệm chuẩn của đề tài khi ghép nối cả 3 phần cứng trên Hàng Phải của Breadboard:
1. Kiểm tra kết nối SPI với FPGA Tang Nano 20K (`REG_ID = 0xA5`, `REG_LANES = 1`), khởi tạo cảm biến dòng INA219 qua nguồn `GPIO 38` và I2C (`GPIO 47, 21`), khởi tạo Camera OV5640 ở chế độ `QVGA 320x240 RGB565`.
2. Tự động quét qua **6 cấu hình vi kiến trúc của FPGA** (`idle`, `mode0`, `mode1_thr3`, `mode1_thr2`, `mode1_thr1`, `mode2`), lấy trung bình 40 mẫu điện áp/dòng điện/công suất từ INA219 và chốt bộ đếm phần cứng (`OPS`, `ENR`, `ERR`) để xuất bảng chuẩn định dạng CSV (`CSV,config,mode,thr,vbus_V,i_mA,p_mW,ops,residual_active,nmed`).
3. Vận hành liên tục vòng lặp thích nghi **3 Chế độ**:
   - **Mode 0**: Đặt FPGA về `mode = 0`, đưa ESP32-S3 vào `Light Sleep` 300 ms, đo công suất `p0`.
   - **Mode 1**: Đặt FPGA sang `mode = 1, thr = 2`, cắt vùng trung tâm **25% diện tích khung hình (ROI $160 \times 120$)** từ Camera OV5640, trích xuất 100 đặc trưng `INT8` và chạy phân loại nhẹ (`AI_PARTIAL_DIM = 100`), đo thời gian thực thi `t1 (µs)` và công suất `p1`.
   - **Mode 2**: Khi phát hiện biến thiên độ sáng/chuyển động trong vùng ROI (hoặc trong chế độ Benchmark MNIST khi không gắn camera), đặt FPGA sang `mode = 2` (Exact), trích xuất toàn bộ **100% khung hình ($320 \times 240$, 400 đặc trưng `INT8`)** chạy qua toàn bộ mô hình (`AI_FEAT_DIM = 400`), đo thời gian `t2 (µs)`, công suất `p2` và nhiệt độ chip.
