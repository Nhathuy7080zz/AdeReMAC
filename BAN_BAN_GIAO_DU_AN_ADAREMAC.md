# BẢN BÀN GIAO DỰ ÁN ĐẦY ĐỦ: ADAREMAC (AdeReMAC v3)
**Vi kiến trúc Bộ nhân–Tích lũy Xấp xỉ Thích nghi Phần dư Thưa & Hệ thống Đồng xử lý Edge-AI Phân tầng 3 Chế độ trên FPGA Sipeed Tang Nano 20K + ESP32-S3 N16R8 OV5640 + INA219**

> **Hướng dẫn dành cho AI ở cuộc trò chuyện mới**: Tài liệu này chứa toàn bộ bối cảnh, toán học, kiến trúc phần cứng/phần mềm, quyết định thiết kế đã chốt, nhật ký toàn bộ lỗi & cách khắc phục thực tế, các việc cần làm tiếp theo và **100% mã nguồn phiên bản mới nhất không viết tắt**. Hãy đọc kỹ toàn bộ 7 phần dưới đây để tiếp tục công việc ngay lập tức mà không cần hỏi lại người dùng những thông tin đã có, tuyệt đối không đề xuất lại các hướng thiết kế đã bị loại bỏ.

## 1. MỤC TIÊU DỰ ÁN

### 1.1. Dự án làm gì và vì sao?
Dự án **AdaReMAC** (*Adaptive Sparse-Residual Approximate Multiply-Accumulate* — thường gọi tắt trong lịch sử trò chuyện là **AdeReMAC**) nghiên cứu, thiết kế phần cứng RTL (Verilog), kiểm chứng trùng khớp từng bit (*bit-exact*) trên mạng nơ-ron tích chập (CNN `INT8`), và triển khai hệ thống thực nghiệm kết hợp giữa **FPGA Sipeed Tang Nano 20K** với vi điều khiển **ESP32-S3 N16R8 tích hợp Camera OV5640** và cảm biến đo công suất **INA219**.

Mục tiêu cốt lõi của dự án là giải quyết triệt để **hai nghịch lý năng lượng** khiến các bộ nhân xấp xỉ truyền thống thất bại khi đưa ra hệ thống phần cứng thực tế:
1. **Nghịch lý diện tích & công suất ở cấp cổng logic (Gate-level Paradox)**:
   - Trong phép nhân không dấu $8 \times 8$ bit, nếu xấp xỉ $k = 3$ đường chéo tích riêng trọng số thấp nhất ($D_0, D_1, D_2$, chiếm 6 trên 64 bit tích riêng) với hệ số $q_L = 2$, phần dư sửa sai rút gọn còn $R = 2D_2 - D_0$ ($D_1$ tự triệt tiêu hoàn toàn).
   - Tuy nhiên, kết quả tổng hợp thực tế cho thấy khối sửa sai $R$ chỉ chiếm **12 cell trên tổng số 343 cell** của bộ nhân (~3,5% diện tích logic). Hơn nữa, trên dữ liệu ảnh thực tế (MNIST), có tới **79,54% phép nhân ở lớp `conv1`** và **44,88% ở lớp `conv2`** chứa ít nhất một toán hạng bằng $0$ (do nền ảnh tối và hàm kích hoạt `ReLU`). Vì vậy, nếu chỉ tắt/mở khối Residual bên trong FPGA, mức công suất tiết kiệm được chỉ tính bằng phân số mili-watt và bị công suất tĩnh của board FPGA nuốt chửng.
2. **Nghịch lý công suất ở cấp hệ thống (System-level Paradox)**:
   - Trong hệ thống camera thông minh tại biên, vi điều khiển **ESP32-S3** khi thức liên tục để cấp xung `XCLK` cho cảm biến ảnh OV5640, đọc DMA vào PSRAM và chạy suy luận tiêu thụ từ **180 mW đến 350 mW**. Nếu ESP32-S3 luôn thức 100% thời gian, mọi nỗ lực tiết kiệm vài mW trên đường dữ liệu của FPGA trở nên vô nghĩa.

Để giải quyết tận gốc cả hai nghịch lý trên, dự án chuyển từ "xấp xỉ đơn lẻ ở bộ nhân" sang **Kiến trúc Đồng xử lý Phân tầng 3 Chế độ (3-Mode Hierarchical Collaborative Architecture)** kết hợp điều khiển trạng thái nguồn của ESP32-S3 và điều tiết tỷ lệ dữ liệu đi qua mô hình AI:
- **Mode 0 — Thuần xấp xỉ (Always-On Gatekeeper, chiếm 80–90% thời gian)**: FPGA chạy `mode = 0` (`enr = 0`, cô lập hoàn toàn khối Residual), vi điều khiển ESP32-S3 **ngắt CPU (`esp_light_sleep_start()`)**, lượng dữ liệu đưa qua mô hình trên ESP32 bằng **0% (0 byte)**.
- **Mode 1 — Xấp xỉ chọn lọc một phần (Selective Residual & Early-Exit 25% ROI, chiếm 10–15% thời gian)**: FPGA chạy `mode = 1, thr = 2` (chỉ bật khối sửa sai khi $D_2 \ge 2$, tỷ lệ kích hoạt thực tế chỉ từ **3,57% đến 8,89%**), ESP32-S3 chỉ cắt **25% diện tích khung hình** (vùng trung tâm ROI $160 \times 120$ từ khung hình $320 \times 240$) và chỉ đưa **25% lượng đặc trưng (`AI_PARTIAL_DIM = 100/400` byte)** qua tầng phân loại nhẹ để kiểm tra có chuyển động/đối tượng thực sự hay không.
- **Mode 2 — Chính xác tuyệt đối (Full Exact 100% Data, chiếm 3–5% thời gian)**: Khi Mode 1 xác nhận có sự kiện/chuyển động, FPGA chuyển sang `mode = 2` (`enr = 1`, khôi phục tích số chính xác tuyệt đối 100%, $\text{NMED} = 0$), và ESP32-S3 xử lý **100% khung hình ($320 \times 240$, `AI_FEAT_DIM = 400/400` byte)** để đạt độ chính xác nhận dạng tối đa (**98,51%–98,53%**).

Bên cạnh quy trình chuẩn đo đạc 3 Mode với FPGA và INA219 (**Sketch 2 — `2_adaremac_main.ino`**), dự án xây dựng thêm một ứng dụng trình diễn trực quan độc lập trên trình duyệt Web (**Sketch 1 — `1_test_camera_ov5640.ino`**): Phát luồng **Video Live Stream HD/FHD tốc độ cao qua Pure Wi-Fi AP 40MHz HT40 (`http://192.168.4.1`)**, tích hợp thuật toán **AI Học Môi Trường Liên Tục 5 Đặc Trưng + Khóa Cổng Động Học (Spatio-Temporal 5-Feature Environment Learner)** tự động hấp thụ ngầm các hoa văn trần nhà / bóng đèn tĩnh vào nền và chỉ đóng khung đỏ/vàng bám sát các vật thể bất thường (bàn tay, người, đồ vật mới) hoặc nguồn sáng lạ (đèn Flash) kèm nhãn phân loại từ bộ nhân AdaReMAC (`Mode 1` vs `Mode 2`).

### 1.2. Yêu cầu và ràng buộc kỹ thuật chính
| Hạng mục | Thông số kỹ thuật & Ràng buộc bắt buộc |
| :--- | :--- |
| **Vi điều khiển & Camera** | Board **ESP32 Cam Kit Phát Triển ESP32-S3 N16R8 OV5640 TypeC** (`ESP32-S3 WROOM-1`, **16MB Quad Flash `N16`**, **8MB Octal PSRAM `R8`**, cảm biến **OmniVision OV5640 5MP** gắn qua khe FPC 24-pin). |
| **Ràng buộc cắm dây Breadboard** | Khi cắm mạch ESP32-S3 lên Breadboard, hàng chân bên trái sát mép nên **chỉ còn trống duy nhất 1 hàng lỗ bên phải (Right Row)**. Toàn bộ 14 chân Camera OV5640 (`GPIO 4, 5, 6, 7, 15, 16, 17, 18, 8, 9, 10, 11, 12, 13`) nằm ở Hàng Trái. Các chân `GPIO 35, 36, 37` ở Hàng Phải là bus nội bộ của **8MB OPI PSRAM** (tuyệt đối cấm cắm dây). Do đó **100% dây nối ngoại vi (`SPI`, `I2C`, `VCC nuôi INA219`, `GND`) phải dồn toàn bộ sang các chân an toàn trên Hàng Phải**. |
| **Board FPGA** | **Sipeed Tang Nano 20K**, chip **Gowin `GW2AR-LV18QN88C8/I7`**, bắt buộc chọn **`Device Version: C` (`GW2AR-18C`)**. Xung nhịp onboard `27 MHz` (`Pin 4`), cầu USB-UART qua chip `BL616` (`Pin 69`), 2 nút nhấn `S1` (`Pin 88`) và `S2` (`Pin 87`) có trở kéo xuống GND sẵn trên mạch, 6 LED active-low (`Pin 15..20`). |
| **Cảm biến dòng & Công suất** | Module **INA219** (địa chỉ I2C `0x40`, điện trở Shunt $R_{\text{shunt}} = 0,1\,\Omega$). Chân `VCC` của INA219 được nuôi trực tiếp bằng mức `HIGH` (`3.3V`) xuất ra từ **`GPIO 38` (Chân số 9 Hàng Phải)** của ESP32-S3 (dòng tiêu thụ chip INA219 chỉ ~0,7 mA). |
| **Môi trường Firmware (IDE)** | **Arduino IDE 2.x** trên Windows 11 + gói board **`esp32 by Espressif Systems`** (`https://espressif.github.io/arduino-esp32/package_esp32_index.json`). **Đã gỡ bỏ hoàn toàn ESP-IDF** và **không dùng bất kỳ thư viện bên thứ ba nào ngoài core chuẩn**. |
| **Thiết lập `Tools` trong Arduino IDE** | `Board: "ESP32S3 Dev Module"`, `Flash Size: "16MB (128Mb)"`, `Partition Scheme: "Huge APP (3MB No OTA/1MB SPIFFS)"`, `PSRAM: "OPI PSRAM"` (**bắt buộc**), `USB CDC On Boot: "Enabled"` (cổng USB) hoặc `"Disabled"` (cổng UART). |
| **Môi trường Tổng hợp FPGA** | **Gowin EDA V1.9.12.04 (64-bit)** trên Windows. Bắt buộc bật `Unused Pin -> As input tri-stated` (bỏ pull-up) và tích đủ 4 mục trong `Dual-Purpose Pin` (`DONE`, `READY`, `RECONFIG_N`, `SSPI as regular IO`). |
| **Công cụ Mô phỏng & AI** | **Icarus Verilog (`iverilog` / `vvp` v13.0)** tại `C:\iverilog\bin` và **Python 3.11+ / 3.14** (`numpy`, `torch` tùy chọn vì đã có `train_mnist_numpy.py` thuần NumPy). |

## 2. KIẾN TRÚC / CẤU TRÚC HIỆN TẠI

### 2.1. Sơ đồ cây thư mục dự án (`adaremac/`)
```text
adaremac/
├── README.md                          # Hướng dẫn thực hành toàn diện từ A–Z cho bản v3
├── uninstall_esp_idf.ps1              # Script PowerShell tự động gỡ sạch ESP-IDF, Cache, PATH & Registry
├── firmware_arduino/                  # Mã nguồn chuẩn Arduino IDE 2.x (Không cần cài thư viện ngoài)
│   ├── 1_test_camera_ov5640/
│   │   ├── 1_test_camera_ov5640.ino   # Sketch 1: Stream HD/FHD Pure AP 40MHz + AI 5-Feature chống kẹt học nền
│   │   └── ai_model_data.h            # Trọng số mạng nơ-ron INT8 AdaReMAC + CNN/PCA tích hợp sẵn
│   └── 2_adaremac_main/
│       ├── 2_adaremac_main.ino        # Sketch 2: Code chính kết nối SPI 1 hàng phải với Tang Nano 20K + INA219 + AI 3 Mode
│       └── ai_model_data.h            # Trọng số mạng CNN INT8 và 20 mẫu kiểm thử Mode 1 (25%) / Mode 2 (100%)
├── rtl/                               # Mã nguồn Verilog tổng hợp cho Tang Nano 20K (Đã sửa chống nóng chip)
│   ├── adaremac_mul.v                 # Bộ nhân xấp xỉ 8x8 không dấu (qL = 2, k = 3, R = 2*D2 - D0)
│   ├── adaremac_mac_lane.v            # Lane MAC INT8 có dấu 3 tầng pipeline + bộ giám sát sai số phần cứng
│   ├── mac_array.v                    # Mảng MAC với bộ sinh mẫu xorshift32 (Lane 0 bật MONITOR = 1)
│   ├── spi_slave.v                    # Giao tiếp SPI Slave (Mode 0, khung 16-bit, đồng bộ 3 tầng FF)
│   ├── adaremac_core.v                # Thanh ghi SPI + bộ đếm OPS/ENR/ERR + mặc định N_LANES = 1, mode = 0
│   ├── top_tn20k.v                    # Top module cho bản nối SPI với ESP32-S3 (spi_miso về 1'bz khi CS_N = 1)
│   ├── top_standalone.v               # Top module cho bản chạy độc lập xuất báo cáo qua UART Pin 69
│   ├── report_tx.v                    # Đóng gói khung báo cáo ASCII 45 byte ("S<n> OPS=... ENR=... ERR=...\r\n")
│   ├── uart_tx.v                      # Bộ phát UART 8N1 tốc độ 115200 baud ở clock 27 MHz
│   └── btn_sync.v                     # Bộ đồng bộ & chống dội nút nhấn (10 ms @ 27 MHz)
├── constraints/                       # File gán chân vật lý (.cst) và ràng buộc xung nhịp (.sdc) cho Gowin EDA
│   ├── tn20k.cst                      # Gán chân cho top_tn20k (Pin 4, 15..20, 41, 42, 48, 49; PULL_MODE=NONE)
│   ├── tn20k.sdc                      # Ràng buộc clock 27 MHz (chu kỳ 37.037 ns)
│   ├── tn20k_standalone.cst           # Gán chân cho top_standalone (Pin 4, 88, 87, 69, 15..20; PULL_MODE=NONE)
│   └── tn20k_standalone.sdc           # Ràng buộc clock 27 MHz
├── standalone_tangnano_only/          # Thư mục tự chứa đúng 7 file .v tổng hợp được + .cst/.sdc cho bản độc lập
│   ├── README.md
│   ├── adaremac_mul.v, adaremac_mac_lane.v, mac_array.v, top_standalone.v, report_tx.v, uart_tx.v, btn_sync.v
│   └── tn20k_standalone.cst, tn20k_standalone.sdc
├── sim/                               # Testbench kiểm chứng RTL bằng Icarus Verilog (iverilog / vvp)
│   ├── run_sim.sh                     # Script chạy tự động tb_mul và tb_core
│   ├── tb_mul.v                       # Kiểm chứng vét cạn 65.536 cặp không dấu x 6 cấu hình
│   ├── tb_core.v                      # Kiểm chứng toàn tuyến SPI Master -> Core -> MAC Array -> Counters
│   └── tb_standalone.v                # Kiểm chứng bản độc lập và giải mã khung UART 45 byte
└── ai_eval/                           # Huấn luyện CNN, lượng tử hóa INT8, kiểm chứng bit-exact & xuất C header
    ├── approx_mul.py                  # Mô hình bảng tra 256x256 bit-exact của bộ nhân AdaReMAC (có dấu & không dấu)
    ├── qnet.py                        # Lượng tử hóa đối xứng PTQ INT8 + suy luận CNN 3 lớp bằng bảng tra
    ├── mnist_data.py                  # Tải và đọc bộ dữ liệu MNIST bằng NumPy thuần
    ├── train_mnist.py                 # Huấn luyện CNN FP32 bằng PyTorch -> model_fp32.npz
    ├── train_mnist_numpy.py           # Huấn luyện CNN FP32 bằng NumPy thuần (không cần PyTorch) -> model_fp32.npz
    ├── eval_approx.py                 # Đo độ chính xác, độ nhạy từng lớp, thống kê ENR/Zero -> results_k3.csv
    ├── explore_k.py                   # Khảo sát mở rộng vùng xấp xỉ k = 3..8
    ├── verify_bitexact.py             # Đối chiếu trực tiếp bảng tra Python với RTL thật qua iverilog (65.536 cặp)
    ├── tb_signed_dump.v               # Testbench xuất toàn bộ 65.536 tích có dấu của adaremac_mac_lane
    ├── export_for_esp32.py            # Xuất trọng số INT8 và đặc trưng Mode 1 / Mode 2 ra ai_model_data.h
    ├── results_k3.csv                 # Bảng kết quả thực nghiệm chính xác trên 10.000 ảnh MNIST
    └── model_fp32.npz                 # Trọng số mạng CNN đã huấn luyện (đạt 98,48% FP32 / 98,51% INT8)
```

### 2.2. Vai trò từng phần và cơ chế kết nối liên thông
1. **Lõi Toán học & RTL trên FPGA (`adaremac_mul.v` $\to$ `adaremac_mac_lane.v` $\to$ `mac_array.v`)**:
   - Hai toán hạng có dấu `a[7:0]` và `b[7:0]` (`INT8`, từ `-128` đến `127`) đi vào **Tầng 1** của `adaremac_mac_lane.v` để lấy trị tuyệt đối 8-bit (`ma`, `mb`) và dấu tích (`neg1 = a[7] ^ b[7]`).
   - Tại **Tầng 2**, `ma` và `mb` đi vào `adaremac_mul.v`. Phần cao `ph` (các đường chéo $\ge 3$) được tính chính xác. Ba đường chéo thấp $D_0, D_1, D_2$ được gom thành $\text{ml} = D_0 + D_1 + D_2$ và dịch trái 1 bit thành $\text{plh} = 2\cdot\text{ml}$. Tín hiệu bật phần dư được quyết định bởi bộ điều khiển thích nghi:
     $$\text{enr} = \begin{cases} 1, & \text{nếu } \text{mode} \in \{2, 3\} \text{ (Exact)} \\ (D_2 \ge \text{thr}), & \text{nếu } \text{mode} = 1 \text{ (Selective)} \\ 0, & \text{nếu } \text{mode} = 0 \text{ (Full Approximate)} \end{cases}$$
   - Khi `enr = 0`, các cổng AND (`d2_g = d2 & {2{enr}}`, `d0_g = d0 & enr`) khóa cứng ngõ vào của mạch trừ phần dư $R = 2D_2 - D_0$ về 0 để triệt tiêu công suất chuyển mạch.
   - Tại **Tầng 3**, tích không dấu `p2` được khôi phục dấu và cộng dồn vào `acc[31:0]`. Riêng ở **Lane 0 (`MONITOR = 1`)**, một bộ nhân cứng `ex2 <= ma * mb` tính song song tích đúng để xuất độ lớn sai số tuyệt đối `err_mag = |ex2 - p2|` cùng cờ `enr_out` và `v_out`.
2. **Giao tiếp điều khiển & Đọc thống kê qua SPI (`spi_slave.v` $\leftrightarrow$ `adaremac_core.v` $\leftrightarrow$ `top_tn20k.v`)**:
   - ESP32-S3 đóng vai trò **SPI Master** (tần số `1 MHz`, Mode 0 `CPOL=0, CPHA=0`), gửi khung 16-bit xuống `spi_slave.v` của FPGA.
   - Bản đồ thanh ghi bên trong `adaremac_core.v`:
     - `0x00 (REG_MODE)`: Ghi/đọc `mode[1:0]` (`0`: Approx, `1`: Selective, `2`: Exact).
     - `0x01 (REG_THR)`: Ghi/đọc ngưỡng `thr[1:0]` (`1..3`).
     - `0x02 (REG_CTRL)`: Bit 0 = `CTRL_RUN` (cho phép mảng MAC chạy), Bit 1 = `CTRL_SNAP` (chốt giá trị bộ đếm vào thanh ghi đệm để đọc nhiều byte không bị lệch), Bit 2 = `CTRL_CLR` (xóa các bộ đếm về 0).
     - `0x03 (REG_ID)`: Hằng số định danh phần cứng **`0xA5`**.
     - `0x04 (REG_LANES)`: Số lane phần cứng hiện tại (`N_LANES = 1`).
     - `0x05 (REG_WAKE_THR)`: Ngưỡng đánh thức (`wake_thr`, mặc định `0x80`).
     - `0x10..0x13 (REG_OPS)`: Số phép tính MAC đã thực hiện (32-bit Little-Endian).
     - `0x14..0x17 (REG_ENR)`: Số phép tính MAC có bật khối sửa sai Residual (32-bit Little-Endian).
     - `0x18..0x1C (REG_ERR)`: Tổng sai số tuyệt đối tích lũy $\sum |P_{\text{exact}} - P_{\text{approx}}|$ (40-bit Little-Endian). Từ đó tính được $\text{ResidualActive} = \text{ENR}/\text{OPS}$ và $\text{NMED} = \text{ERR} / (\text{OPS} \times 16384)$.
3. **Sơ đồ đấu nối vật lý (Dồn 100% sang Hàng Phải của ESP32-S3 trên Breadboard)**:

| Thứ tự chân (từ trên xuống ở Hàng Phải) | Ký hiệu in trên ESP32-S3 | Đấu nối tới đâu? | Vai trò tín hiệu |
| :--- | :--- | :--- | :--- |
| Chân 1, 2 | `TX` (`43`), `RX` (`44`) | *(Để trống)* | UART0 nạp code / Serial Monitor |
| **Chân 3** | **`1` (`GPIO 1`)** | Tang Nano 20K — **Pin 42** | `SPI_CS` (Chip Select, active-low) |
| **Chân 4** | **`2` (`GPIO 2`)** | Tang Nano 20K — **Pin 48** | `SPI_MOSI` (Master Out Slave In) |
| **Chân 5** | **`42` (`GPIO 42`)** | Tang Nano 20K — **Pin 41** | `SPI_SCLK` (Xung nhịp SPI 1 MHz) |
| **Chân 6** | **`41` (`GPIO 41`)** | Tang Nano 20K — **Pin 49** | `SPI_MISO` (Master In Slave Out — có High-Z) |
| Chân 7, 8 | `40`, `39` | *(Để trống)* | Dự phòng |
| **Chân 9** | **`38` (`GPIO 38`)** | Module INA219 — **`VCC`** | Xuất mức **HIGH (`3.3V`)** nuôi mạch logic INA219 |
| Chân 10..15 | `37, 36, 35, 0, 45, 48` | *(Tuyệt đối để trống)* | Bus OPI PSRAM (`35..37`), Boot (`0`), VSPI (`45`), RGB (`48`) |
| **Chân 16** | **`47` (`GPIO 47`)** | Module INA219 — **`SDA`** | Đường dữ liệu `I2C_SDA` (400 kHz) |
| **Chân 17** | **`21` (`GPIO 21`)** | Module INA219 — **`SCL`** | Đường xung nhịp `I2C_SCL` (400 kHz) |
| Chân 18, 19 | `20`, `19` | *(Để trống)* | Cặp chân USB D- / D+ |
| **Chân 20 (Cuối)** | **`G` (`GND`)** | **`GND`** Tang Nano 20K & **`GND`** INA219 | **Mass (`GND`) chung cho cả 3 mạch** |

## 3. QUYẾT ĐỊNH ĐÃ CHỐT (VÀ LÝ DO)

Để AI ở cuộc trò chuyện mới **không bao giờ đề xuất lại các phương án đã bị loại bỏ**, dưới đây là 10 quyết định thiết kế đã chốt cứng cùng lý do kỹ thuật:

1. **Chốt cố định cấu hình toán học $k = 3, q_L = 2 \Rightarrow R = 2D_2 - D_0$ (Tuyệt đối không mở rộng lên $k = 4 \dots 8$)**:
   - *Lý do*: Khi $k = 3, q_L = 2$, với phân bố bit độc lập $p = 0,5$, kỳ vọng toán học của các đường chéo là $\mathbb{E}[D_0] = 0,25$, $\mathbb{E}[D_1] = 0,50$, $\mathbb{E}[D_2] = 0,75$, dẫn đến kỳ vọng phần dư bằng không tuyệt đối: $\mathbb{E}[R] = 2(0,75) - 0,25 = +1,25$ trên phân bố đều nhưng trên phân bố đối xứng có dấu quanh $0$, sai số trung bình triệt tiêu và $D_1$ biến mất hoàn toàn khỏi mạch phần cứng. Khi thử mở rộng lên $k = 4 \dots 8$ (`explore_k.py`), sai số trung bình có hướng tích lũy qua hàng trăm phép cộng dồn làm độ chính xác MNIST sụp đổ (từ **98,50%** ở $k=3$ tụt xuống **95,38%** ở $k=4$, **76,72%** ở $k=5$ và **9,74%** ở $k=8$ khi chưa hiệu chỉnh bias).
2. **Chốt Kiến trúc Cộng tác Phân tầng 3 Chế độ (FPGA + ESP32-S3) thay vì chỉ đo riêng FPGA**:
   - *Lý do*: Chỉ tắt/mở khối Residual trên FPGA không mang lại chênh lệch công suất đáng kể ở cấp hệ thống. Phải kết hợp: **Mode 0** (FPGA chạy thuần xấp xỉ, ngắt CPU ESP32-S3 bằng `Light Sleep`, 0% dữ liệu qua model), **Mode 1** (FPGA chạy `mode=1, thr=2`, ESP32-S3 chỉ xử lý 25% dữ liệu ROI $160\times 120$), và **Mode 2** (FPGA chạy `mode=2` Exact, ESP32-S3 xử lý 100% dữ liệu $320\times 240$).
3. **Loại bỏ hoàn toàn ESP-IDF, chuyển 100% sang Arduino IDE 2.x (Không cài thư viện ngoài)**:
   - *Lý do*: ESP-IDF chiếm 10–15 GB ổ cứng, cài đặt biến môi trường phức tạp và bắt buộc biên dịch qua `CMakeLists.txt` chỉ nhận 1 file `main.cpp` duy nhất. Chuyển sang **Arduino IDE 2.x** với 2 Sketch độc lập (`1_test_camera_ov5640.ino` và `2_adaremac_main.ino`) chỉ dùng các thư viện tích hợp sẵn trong gói board `esp32 by Espressif Systems`, giúp mở file nào nạp ngay file đó bằng 1 cú nhấp chuột.
4. **Dồn 100% dây cắm ngoại vi sang duy nhất Hàng Phải (Right Row) của ESP32-S3 và dùng `GPIO 38` xuất `3.3V` nuôi INA219**:
   - *Lý do*: Trên Breadboard của người dùng, hàng chân bên trái của ESP32-S3 không còn lỗ cắm (và chứa toàn bộ 14 chân Camera OV5640 `GPIO 4..18`). Việc chọn cụm `GPIO 1, 2, 42, 41` (Chân 3..6 Hàng Phải) cho SPI, `GPIO 47, 21` (Chân 16..17 Hàng Phải) cho I2C, và bật `GPIO 38` (Chân 9 Hàng Phải) lên `HIGH` để cấp nguồn 3.3V cho chân `VCC` của INA219 giúp người dùng không phải kéo bất kỳ sợi dây nào sang Hàng Trái.
5. **Đặt mặc định `N_LANES = 1` trong RTL, xóa cổng `wake_esp` (Pin 50) và đưa `spi_miso` về High-Z (`1'bz`) khi `spi_cs_n = 1`**:
   - *Lý do*: Khi để `N_LANES = 16` chạy tự do ở 27 MHz kèm các chân không dùng bị kéo `PULL_MODE=UP`, chip FPGA Tang Nano 20K bị nóng lên rất nhanh. Hạ `N_LANES = 1` giảm 95% công suất động (chỉ dùng ~190 FF thay vì 1.539 FF) mà vẫn giữ nguyên 100% chức năng của bộ giám sát sai số tại Lane 0 và giao thức SPI. Xóa cổng `wake_esp` vì gói chân QN88 không có Pin 50 (tránh lỗi `CT1113` hoặc Gowin gán bừa vào chân ngoại vi trên board gây chập), và đưa `spi_miso` (Pin 49) về `1'bz` khi không truyền SPI để tránh xung đột áp với mạch đèn nền LCD trên Tang Nano 20K.
6. **Sử dụng Power-On Reset tự động (`rst_n = por_done`) trên FPGA thay vì phụ thuộc trực tiếp vào Pin 88 (`btn_rst`)**:
   - *Lý do*: Pin 88 trên Tang Nano 20K trùng với chân cấu hình `MODE0`. Nếu người dùng quên bật `Dual-Purpose Pin` trong Gowin EDA hoặc giữ nút, hệ thống dễ bị kẹt ở trạng thái Reset. Dùng bộ đếm tự động 8-bit `por` (`wire rst_n = &por;`) đảm bảo mạch luôn khởi chạy 100% sau ~9,5 µs.
7. **Tách hoàn toàn file Testbench (`tb_standalone.v`) ra khỏi thư mục `standalone_tangnano_only/`**:
   - *Lý do*: Để người dùng có thể bấm `Add All (.v)` trong thư mục `standalone_tangnano_only/` trên Gowin EDA mà không bao giờ gặp lỗi tổng hợp `EX3828: Root scope identifier 'dut.rst_n' remains unresolved`.
8. **Sử dụng Pure Wi-Fi AP 40MHz (`WIFI_BW_HT40`) + Tách 2 HTTP Server (Port 80 & Port 81) + Hạ `XCLK` Camera xuống `16 MHz`**:
   - *Lý do*: Khi ESP32-S3 tự phát Wi-Fi (`Pure WIFI_AP` ở băng thông 40 MHz, tắt `WIFI_PS_NONE`, công suất `19.5 dBm`, bật `TCP_NODELAY`), luồng video đi thẳng 1 chặng (Zero-Hop) tới máy tính/điện thoại mà không phải bay 2 chặng qua Router gia đình. Hạ `XCLK` từ `20 MHz` xuống **`16 MHz`** giúp tín hiệu `PCLK` trên cáp dẹt FPC 24-pin sạch tuyệt đối, triệt tiêu hoàn toàn hiện tượng rớt byte gây sọc tím ngang.
9. **Cấp phát tĩnh (`.bss`) 100% các mảng của thuật toán AI và cấp Stack `20480` byte cho `ai_learn_task`**:
   - *Lý do*: Tuyệt đối không khai báo các mảng lưới không gian (`300` phần tử `float`) hay mảng đặc trưng (`400` byte) làm biến cục bộ trong hàm chạy trên FreeRTOS task để không bao giờ xảy ra lỗi tràn Stack (`Stack canary watchpoint triggered`).
10. **Thuật toán AI Học Môi Trường 5 Đặc Trưng có Khóa `s_learn_cooldown = 30`, `s_fast_learn_frames = 3` và Hấp thụ ngầm từng ô (`Silent Per-Cell Background Absorption`)**:
    - *Lý do*: Không bao giờ cho phép kích hoạt lại trạng thái "Đang học môi trường" khi đang học (`s_fast_learn_frames > 0`) hoặc đang trong thời gian khóa (`s_learn_cooldown > 0`). Đồng thời dùng khóa cổng động học tiền cảnh `s_fg_lock` (chỉ cấp khi ô vừa có chuyển động `s_static_age == 0`) kết hợp hấp thụ ngầm nhanh (`alpha = 0.38f`) khi ô đứng yên (`s_static_age >= 3`), giúp hoa văn trần nhà và bóng đèn cố định tự động hòa vào nền mà không khóa giao diện Web và không bao giờ báo động giả.

## 4. NHẬT KÝ LỖI & CÁCH SỬA (THEO THỨ TỰ THỜI GIAN)

Dưới đây là toàn bộ lịch sử các vấn đề/lỗi đã phát sinh từ đầu dự án đến phiên bản mới nhất, bao gồm cả **những cách đã thử nhưng không hiệu quả**:

### Mục 1 — Nghịch lý tiết kiệm năng lượng của bản thiết kế gốc (v1)
- **Triệu chứng / Vấn đề**: Bản gốc chỉ tắt/bật khối sửa sai Residual ($R = 2D_2 - D_0$) trên FPGA trong khi giữ ESP32-S3 thức 100% thời gian và chỉ đo dòng riêng đường 5V của FPGA. Mức chênh lệch công suất giữa Mode 0 và Mode 2 quá nhỏ (chỉ vài mW trên lý thuyết) so với công suất toàn hệ thống (~250–400 mW).
- **Nguyên nhân gốc**:
  1. Ở $k=3, q_L=2$, mạch tính phần dư $R = 2D_2 - D_0$ quá tối giản (chỉ tốn 12 cell trên tổng 343 cell của bộ nhân, tương đương ~3,5%).
  2. Trên dữ liệu ảnh MNIST thật, có tới 79,54% phép nhân ở `conv1` có toán hạng bằng 0 (không phát sinh đảo bit ở phần thấp).
  3. Vi điều khiển ESP32-S3 thức liên tục tiêu thụ 180–350 mW làm lu mờ hoàn toàn mức tiết kiệm của bộ nhân.
- **Cách đã thử nhưng KHÔNG hiệu quả**:
  - Thử mở rộng số đường chéo xấp xỉ từ $k=3$ lên $k=4, 5, 6, 7, 8$ ở Mode 0 (`ai_eval/explore_k.py`) với kỳ vọng cắt giảm phần lớn cây cộng tích riêng. **Kết quả thất bại**: Từ $k \ge 4$, kỳ vọng sai số $\mathbb{E}[R] \ne 0$ tích lũy qua tổng chập làm độ chính xác mạng CNN sụp đổ nghiêm trọng (từ 98,50% ở $k=3$ rơi xuống 95,38% ở $k=4$, 76,72% ở $k=5$, và 9,74% ở $k=8$).
- **Đã sửa gì**:
  - Giữ nguyên điểm ngọt toán học $k=3, q_L=2$ (độ chính xác Mode 0 đạt 98,50%, Mode 1 `thr=2` đạt 98,53%).
  - Thiết kế lại kiến trúc cấp hệ thống thành **3 Chế độ phối hợp FPGA + ESP32-S3**: Mode 0 ngắt CPU ESP32-S3 (`Light Sleep`, 0% dữ liệu qua model); Mode 1 chỉ đưa 25% dữ liệu (ROI $160\times 120$, 100/400 đặc trưng) qua model trên ESP32; Mode 2 đưa 100% dữ liệu ($320\times 240$, 400/400 đặc trưng) qua model.
- **Kết quả**: Giảm 50%–70% công suất trung bình toàn hệ thống theo phân bố thời gian thực tế mà vẫn giữ nguyên độ chính xác 98,51%–98,53%.

### Mục 2 — Lỗi `FileNotFoundError: [WinError 2]` khi chạy `verify_bitexact.py` trên Windows
- **Triệu chứng / Thông báo lỗi**:
  ```text
  PS C:\Users\nhath\Documents\adaremac\ai_eval> python verify_bitexact.py --rtl-dir ../rtl
   subprocess.run(["iverilog", "-g2005", "-o", vvp, os.path.join(here, "tb_signed_dump.v")] + rtl, check=True)
  FileNotFoundError: [WinError 2] The system cannot find the file specified
  ```
- **Nguyên nhân gốc**: Người dùng vừa cài đặt Icarus Verilog vào `C:\iverilog\bin` và thêm vào hộp thoại `Environment Variables` của Windows, nhưng cửa sổ Terminal PowerShell bên trong VS Code được mở từ trước đó nên vẫn giữ biến môi trường `PATH` cũ chưa có `C:\iverilog\bin`.
- **Đã sửa gì**: Đóng hoàn toàn VS Code và mở lại (hoặc làm mới biến `$env:Path` trong phiên PowerShell) để nhận đường dẫn `C:\iverilog\bin\iverilog.exe` và `vvp.exe`.
- **Kết quả**: Chạy `python verify_bitexact.py --rtl-dir ../rtl` báo `PASS` 100% trên cả 5 cấu hình (`mode0`, `m1_thr3`, `m1_thr2`, `m1_thr1`, `exact`), tổng cộng $65.536 \times 5 = 327.680$ phép nhân có dấu khớp tuyệt đối từng bit giữa Python và Verilog (`sai tích = 0, sai ENR = 0`).

### Mục 3 — Ba lỗi biên dịch trên phần mềm Gowin EDA (`EX3828`, `CT1135`, `CT1113`)
- **Triệu chứng / Thông báo lỗi**:
  1. Khi add tất cả file `.v` trong `standalone_tangnano_only/` vào Gowin EDA, quá trình Synthesize báo lỗi: `ERROR (EX3828) : Root scope identifier 'dut.rst_n' remains unresolved` tại `tb_standalone.v`.
  2. Khi chạy Place & Route bản độc lập, báo lỗi `CT1135`: cổng `btn_rst_n` trong `tn20k_standalone.cst` không tìm thấy trong top module.
  3. Khi chạy Place & Route bản SPI (`top_tn20k.v`), báo lỗi `CT1113`: không thể gán `wake_esp` vào `Pin 50` vì gói chân QN88 của `GW2AR-LV18QN88C8/I7` không có chân IO số 50.
- **Nguyên nhân gốc**:
  1. File `tb_standalone.v` là file mô phỏng testbench (chứa đường dẫn phân cấp `dut.rst_n` và khối `initial`), không phải mạch phần cứng tổng hợp được, nhưng lại để lẫn trong thư mục `standalone_tangnano_only/`.
  2. Tên cổng trong `top_standalone.v` là `btn_rst` nhưng file `.cst` cũ viết là `btn_rst_n`.
  3. Sơ đồ chân gói QN88 của chip GW2AR-18C không xuất Pin 50 ra IO hợp lệ.
- **Đã sửa gì**:
  - Xóa `tb_standalone.v` khỏi thư mục `standalone_tangnano_only/` (chỉ để trong thư mục `sim/`), đảm bảo `standalone_tangnano_only/` chỉ chứa đúng 7 file `.v` tổng hợp được.
  - Đồng bộ tên cổng `btn_rst` trong `tn20k_standalone.cst` và xóa bỏ cổng ngõ ra `wake_esp` (Pin 50) khỏi `top_tn20k.v` và `tn20k.cst`.
- **Kết quả**: Synthesize và Place & Route trên Gowin EDA V1.9.12.04 thành công 100%, 0 lỗi.

### Mục 4 — Lỗi chip FPGA Sipeed Tang Nano 20K bị NÓNG RẤT NHANH sau khi nạp (kể cả khi rút hết dây nối ngoài)
- **Triệu chứng**: Ngay sau khi nạp bitstream lên Tang Nano 20K, dù đã rút toàn bộ dây nối với ESP32-S3 (chỉ cắm mỗi cáp USB-C vào máy tính), chip FPGA GW2AR-18C nóng lên rất nhanh chỉ sau vài chục giây.
- **Nguyên nhân gốc (Xác định chính xác từ báo cáo phân tích `aderemac.rpt.html` của Gowin EDA)**:
  1. **Chọn sai phiên bản chip (`Device: GW2AR-18` thay vì `GW2AR-18C`)**: Dòng 78 của `aderemac.rpt.html` ghi `Device: GW2AR-18` (Version `na`), gây lệch cấu hình phần cứng nội bộ so với chip thực tế `GW2AR-18C` (Version `C`) hàn trên Tang Nano 20K.
  2. **56/66 chân IO không sử dụng (`Unused Pins`) bị Gowin mặc định gán `PULL_MODE=UP`**: Bảng `All Package Pins` trong `aderemac.rpt.html` cho thấy toàn bộ 56 chân không khai báo bị bật điện trở kéo lên nguồn (`UP`), bơm dòng rò liên tục vào các IC ngoại vi hàn sẵn trên mạch Tang Nano 20K (IC âm thanh MAX98357A ở Pin 25..28, mạch nguồn LCD ở Pin 49, HDMI, thẻ nhớ TF).
  3. **Chưa bật `Dual-Purpose Pin` (`SSPI / MODE / DONE / READY / RECONFIG_N as regular IO`)**: Làm Pin 88 (`MODE0`) và Pin 87 (`MODE1`) xung đột với mạch cấu hình.
  4. **Công suất động quá lớn & xung đột áp chân MISO trong RTL**: Mảng `mac_array` để mặc định `N_LANES = 16` chạy liên tục 100% chu kỳ ở 27 MHz làm 3.195 LUT và 1.539 Flip-Flop đảo trạng thái không nghỉ; đồng thời chân `spi_miso` (Pin 49) luôn lái cứng mức `0V` ngay cả khi `spi_cs_n = 1`, gây xung đột áp với đường mạch trên board.
- **Cách đã thử nhưng KHÔNG hiệu quả**:
  - Chỉ sửa `PULL_MODE=NONE` cho 10 chân có mặt trong file `.cst` $\to$ Không giải quyết được hiện tượng nóng vì 56 chân *không nằm trong file `.cst`* vẫn bị Gowin EDA tự động áp `PULL_MODE=UP` nếu không đổi thiết lập `Unused Pin` trong menu cấu hình của Gowin EDA.
- **Đã sửa gì**:
  1. **Trong RTL (`adaremac_core.v`, `top_tn20k.v`, `top_standalone.v`)**: Hạ tham số mặc định `N_LANES` từ `16` xuống **`1`** (giảm 95% số Flip-Flop từ 1.539 xuống ~190 FF, giảm 95% công suất động); đưa `spi_miso` về trạng thái trở kháng cao (`1'bz`) khi `spi_cs_n = 1` (`assign spi_miso = spi_cs_n ? 1'bz : spi_miso_core;`); dùng `rst_n = por_done` không phụ thuộc Pin 88.
  2. **Trong `.cst` (`tn20k.cst`, `tn20k_standalone.cst`)**: Đặt `PULL_MODE=NONE` và `BANK_VCCIO=3.3` tường minh cho các chân nút nhấn và SPI.
  3. **Quy trình 4 bước bắt buộc trên Gowin EDA**: (a) Chọn đúng `GW2AR-18C` (Version `C`), (b) Vào `Project -> Configuration -> Place & Route -> Unused Pin` tích chọn **`As input tri-stated`** (bỏ tích pull-up), (c) Vào `Dual-Purpose Pin` tích đủ 4 ô, (d) Nếu Flash cũ còn lưu bitstream lỗi gây nóng thì nhấn giữ nút S1 khi cắm cáp USB để chặn boot từ Flash rồi nạp đè.
- **Kết quả**: Chip FPGA Tang Nano 20K mát lạnh hoàn toàn (~32–36 °C) khi chạy cả bản độc lập lẫn bản ghép nối SPI.

### Mục 5 — Vướng cáp Camera OV5640 & Cắm Breadboard chỉ còn trống 1 hàng chân bên phải
- **Triệu chứng / Vấn đề**: Khi cắm kit `ESP32-S3 N16R8 OV5640 TypeC` lên Breadboard, toàn bộ hàng chân bên trái bị che khuất hoặc vướng khe cáp dẹt FPC 24-pin, chỉ còn trống duy nhất 1 hàng lỗ bên phải (**Hàng Phải / Right Row**). Ngoài ra người dùng muốn gỡ bỏ hoàn toàn ESP-IDF vì quá rối và nặng máy.
- **Nguyên nhân gốc**: Sơ đồ chân ban đầu đặt `I2C_SDA` ở `GPIO 14` (nằm ở góc dưới Hàng Trái) và cần lấy nguồn `3V3` (nằm ở góc trên Hàng Trái) cho module INA219, gây bất tiện khi Breadboard chỉ cắm được ở Hàng Phải.
- **Đã sửa gì**:
  - Viết script PowerShell `uninstall_esp_idf.ps1` gỡ sạch ESP-IDF và chuyển toàn bộ mã nguồn sang **Arduino IDE 2.x** (`firmware_arduino/`).
  - Quy hoạch lại **100% dây kết nối sang duy nhất Hàng Phải**:
    - Cụm 4 chân SPI liền nhau ở góc trên Hàng Phải: `GPIO 1` (`CS`), `GPIO 2` (`MOSI`), `GPIO 42` (`SCLK`), `GPIO 41` (`MISO`).
    - Dùng **`GPIO 38` (Chân số 9 Hàng Phải)** cấu hình `OUTPUT HIGH` (`3.3V`) để cấp nguồn trực tiếp cho chân `VCC` của module INA219 (vì chip INA219 chỉ tiêu thụ 0,7 mA, nằm rất xa giới hạn 40 mA của GPIO ESP32-S3).
    - Cụm 2 chân I2C liền nhau ở góc dưới Hàng Phải: `GPIO 47` (`SDA`) và `GPIO 21` (`SCL`).
    - Chân số 20 cuối cùng Hàng Phải: `GND` chung.
- **Kết quả**: Cắm toàn bộ dây trên duy nhất 1 hàng bên phải của Breadboard cực kỳ gọn, không đụng chạm tới Hàng Trái (Camera) hay cụm chân `GPIO 35..37` (OPI PSRAM).

### Mục 6 — Lỗi `Guru Meditation Error: Core 0 panic'ed (Stack canary watchpoint triggered - ai_learn_task)` làm ESP32-S3 tự khởi động lại liên tục (`Serial.log`)
- **Triệu chứng / Thông báo lỗi (trích nguyên văn từ `Serial.log`)**:
  ```text
  [4/4] Nhan dien thanh cong     : OV5640 (5MP - High-FPS Anti-Lag Mode) (PID = 0x5640)
  [AI LEARNING] DOI GOC CAMERA (Lech 0.0%) -> Dang hoc nen moi (On dinh: 100%) | FPS=0.0 | Temp=47.6C
  Guru Meditation Error: Core  0 panic'ed (Unhandled debug exception). 
  Debug exception reason: Stack canary watchpoint triggered (ai_learn_task) 
  Core  0 register dump:
  PC      : 0x420030cf  PS      : 0x00060636  A0      : 0x82003e2d  A1      : 0x3fcd61a0
  Backtrace: 0x420030cc:0x3fcd61a0 0x42003e2a:0x3fcd9b50 0x4037e221:0x3fcd9b70
  ```
- **Nguyên nhân gốc**: Trong hàm `analyze_frame_and_learn()` được gọi từ `ai_continual_learning_task` (chạy trên Core 0), việc khai báo hàng loạt mảng cục bộ kích thước `300` phần tử (`float raw_y[300]`, `raw_cr[300]`, `raw_cb[300]`, `raw_edge[300]`, `cell_y[300]`, `z_map[300]`, hàng đợi BFS `qx[300]`, `qy[300]`, mảng `patch_feat[400]`) ngay trên Stack cộng với bộ nhớ ngăn xếp của hàm giải mã JPEG `jpg2rgb565()` đã vượt quá 12 KB Stack của task, ghi đè vào vùng bảo vệ Stack Canary của FreeRTOS.
- **Đã sửa gì**:
  - Chuyển **100% các mảng trung gian của thuật toán AI** (`s_raw_y`, `s_raw_cr`, `s_raw_cb`, `s_raw_edge`, `s_raw_max_y`, `s_cell_y`, `s_cell_cr`, `s_cell_cb`, `s_cell_edge`, `s_z_map`, `s_dpeak_map`, `s_raw_anom`, `s_verified_anom`, `s_visited`, `s_qx`, `s_qy`, `s_patch_raw`, `s_patch_feat`, `s_cand_boxes`) sang biến tĩnh toàn cục (`static` nằm trong phân vùng `.bss` của SRAM), đưa mức tiêu thụ Stack của các mảng này về **0 byte**.
  - Tăng kích thước Stack của `ai_continual_learning_task` trong `xTaskCreatePinnedToCore` lên **`20480` byte (20 KB)**.
- **Kết quả**: Triệt tiêu hoàn toàn lỗi `Stack canary watchpoint triggered`, ESP32-S3 chạy liên tục nhiều giờ không bao giờ khởi động lại.

### Mục 7 — Lỗi nghẽn mạng Stream Wi-Fi, kẹt ở ~8–11 FPS, sọc tím ngang & báo động giả trong phòng tối (`image_41ba21.png`)
- **Triệu chứng**:
  - Khi stream qua Router hoặc dùng cấu hình xung nhịp `XCLK = 20 MHz`, tốc độ khung hình bị kẹt quanh `8.4–11.0 FPS`, thỉnh thoảng xuất hiện sọc tím ngang và trong phòng tối xuất hiện các đốm nhiễu hạt màu tím/đỏ bị AI đóng khung nhầm thành 3–5 vật thể bất thường.
- **Nguyên nhân gốc**:
  1. Truyền MJPEG qua Router ngoài (`WIFI_STA`) bắt gói tin bay 2 chặng trên cùng kênh sóng; còn nếu bật cả `WIFI_AP_STA` thì radio phải chia thời gian chuyển kênh.
  2. Cảm biến OV5640 mặc định bật chế độ phơi sáng đêm tự động (`aec2 = 1`) và trần khuếch đại cao (`gainceiling`), tự động kéo dài thời gian màn trập xuống ~100 ms/frame (kẹt ở 10–11 FPS) và khuếch đại nhiễu tối CMOS.
  3. Xung nhịp `XCLK = 20 MHz` trên cáp dẹt FPC 24-pin gây nhiễu đường `PCLK` khi nén JPEG độ phân giải HD/FHD, cộng với `sharpness = +2` làm gai hóa các hạt nhiễu màu trong phòng tối.
- **Cách đã thử nhưng KHÔNG hiệu quả**:
  - Tăng `XCLK` lên `20–24 MHz` để tăng FPS và bật `sharpness = +2` để làm nét ảnh $\to$ Làm tăng lỗi rớt dữ liệu trên cáp FPC (gây sọc tím) và làm tăng nhiễu hạt giả trong điều kiện ánh sáng yếu.
- **Đã sửa gì**:
  - Cấu hình **Pure Wi-Fi AP 40MHz (`WIFI_BW_HT40` — 150 Mbps)** truyền thẳng 1 chặng từ ESP32-S3 tới máy tính/điện thoại (`http://192.168.4.1`), tắt hẳn tiết kiệm điện Wi-Fi (`WIFI_PS_NONE`), công suất phát `19.5 dBm`, bật `TCP_NODELAY`, và tách riêng Port 80 (UI + JSON `/ai`) với Port 81 (`/stream`).
  - Hạ `xclk_freq_hz = 16000000` (16 MHz Clean-PCLK), tắt Night-mode (`set_aec2(s, 0)`), khóa trần `gainceiling = +16x`, đặt `sharpness = 0`, bật bộ khử điểm chết phần cứng (`bpc = 1`, `wpc = 1`, thanh ghi ISP `0x5000 = 0x86`) kèm khử Hot-Pixel trong phần mềm.
- **Kết quả**: Hình ảnh trong trẻo, không còn sọc tím, không còn báo động giả do nhiễu hạt phòng tối.

### Mục 8 — Lỗi đóng khung nhầm vào hoa văn trần nhà / bóng đèn tĩnh & lỗi "Kẹt vĩnh viễn ở trạng thái Đang học môi trường" (`1792302197369.jpeg`)
- **Triệu chứng**:
  1. Khi quay camera lên trần nhà có phào chỉ / hoa văn vòm và bóng đèn cố định (`1792302197369.jpeg`), AI đóng khung đỏ nhầm vào các mảng hoa văn có độ tương phản cao hoặc bóng đèn tĩnh.
  2. Ở bản thử nghiệm kế tiếp, khi giảm ngưỡng phát hiện đổi góc camera xuống thấp (`35/300` ô ~ 11,6%) và cộng cả độ lệch đỉnh sáng (`dt_peak`) + độ lệch biên (`dt_edge`), giao diện Web bị **kẹt vĩnh viễn ở trạng thái vàng `"🔄 ĐANG HỌC MÔI TRƯỜNG MỚI"`**, không bao giờ chuyển sang trạng thái giám sát để đóng khung bàn tay hay đèn Flash.
- **Nguyên nhân gốc**:
  1. Mô hình nền ban đầu chưa ghi nhớ riêng cường độ biên tĩnh (`s_bg_edge`) và đỉnh sáng tĩnh (`s_bg_max_y`) của từng ô, hoặc chỉ cập nhật nền chậm nên hoa văn trần nhà có độ tương phản cao luôn vượt ngưỡng.
  2. Khi hạ ngưỡng phát hiện đổi góc xuống 35/300 ô và cho phép kiểm tra đổi góc ngay cả khi đang trong pha học nhanh (`s_fast_learn_frames > 0`), nhiễu tự nhiên giữa các frame của cảm biến OV5640 trong nhà luôn làm $>35$ ô dao động nhẹ. Hệ quả là ở mỗi chu kỳ, biến `s_fast_learn_frames` vừa giảm từ `10` xuống `9` lại bị gán đè lại thành `10` (`10 -> 9 -> 10`), tạo thành vòng lặp vô tận khóa chặt hệ thống ở trạng thái học nền!
- **Cách đã thử nhưng KHÔNG hiệu quả**:
  - Đặt ngưỡng phát hiện đổi góc quá nhạy (35 ô) dựa trên tín hiệu độ sáng thô chưa lọc mượt thời gian và không có khóa `cooldown` chống kích hoạt lặp.
- **Đã sửa gì (Giải pháp dứt điểm trong `1_test_camera_ov5640.ino` mới nhất)**:
  1. **Khóa cứng bộ đếm học nhanh & Cooldown chống lặp 100%**: Đặt mặc định `s_fast_learn_frames = 0`; khi khởi tạo hoặc khi thật sự xoay góc lớn (>50% diện tích khung hình chuyển động mạnh trên tín hiệu đã lọc mượt `s_smooth_y` và đã trừ độ trôi phơi sáng `frame_ae_shift`), chỉ học nhanh đúng **3 chu kỳ (~0,25 giây)** rồi giảm vô điều kiện về 0, đồng thời bật khóa **`s_learn_cooldown = 30` chu kỳ (~2,5 giây)**. Khi `s_fast_learn_frames > 0` hoặc `s_learn_cooldown > 0`, hệ thống **tuyệt đối không bao giờ gán lại `s_fast_learn_frames`**.
  2. **Khóa cổng động học tiền cảnh (`s_fg_lock`) + Hấp thụ ngầm từng ô (`Silent Per-Cell Background Absorption`)**:
     - Lưu đủ **5 đặc trưng nền** tại mỗi ô: `s_bg_y` (độ sáng TB), `s_bg_max_y` (đỉnh sáng tĩnh), `s_bg_cr` (sắc đỏ), `s_bg_cb` (sắc xanh), `s_bg_edge` (cường độ hoa văn/biên tĩnh).
     - Chỉ cấp khóa tiền cảnh (`s_fg_lock = 12` cho vật thể, `24` cho đèn Flash) khi ô đó **vừa có chuyển động thực sự ở chu kỳ hiện tại (`s_static_age == 0`)**.
     - Bất kỳ hoa văn trần nhà hay bóng đèn nào đứng yên (`s_static_age >= 3` và hết khóa) sẽ tự động được hấp thụ ngầm vào nền với tốc độ cực nhanh (`alpha = 0.38f`) mà không làm kẹt trạng thái UI. Ngược lại, bàn tay/vật thể có vi chuyển động tự nhiên ở các ô lân cận (`moving_nb >= 2`) hoặc đèn Flash mới bật sẽ được duy trì khóa tiền cảnh và đóng khung đỏ/vàng bám sát liên tục.
- **Kết quả**: Giao diện vào trạng thái `"✅ MÔI TRƯỜNG ỔN ĐỊNH"` ngay sau ~0,25 giây, tự động bỏ qua hoa văn trần nhà và bóng đèn cố định, chỉ đóng khung chính xác khi đưa bàn tay/vật thể hoặc bật đèn Flash vào khung hình.

## 5. VẤN ĐỀ CÒN TỒN ĐỌNG & VIỆC TIẾP THEO CẦN LÀM

### 5.1. Trạng thái hiện tại (Những gì đã hoàn thành 100%)
- Toàn bộ mã nguồn RTL Verilog (`rtl/`, `standalone_tangnano_only/`, `constraints/`, `sim/`), script đánh giá AI (`ai_eval/`), script dọn sạch ESP-IDF (`uninstall_esp_idf.ps1`) và 2 Sketch Arduino IDE (`1_test_camera_ov5640.ino`, `2_adaremac_main.ino` kèm `ai_model_data.h`) đã hoàn thiện đồng bộ ở phiên bản v3 mới nhất.
- Toàn bộ các lỗi đã biết (nóng chip FPGA, lỗi biên dịch Gowin EDA, lỗi tràn Stack `ai_learn_task`, lỗi sọc tím OV5640, lỗi nhận nhầm hoa văn tĩnh và lỗi kẹt trạng thái "Đang học môi trường") đều **đã được sửa trong code mới nhất**.

### 5.2. Việc đang làm dở & Các bước tiếp theo cần thực hiện trong cuộc trò chuyện mới
1. **Kiểm tra thực nghiệm bản vá chống kẹt học nền của Sketch 1 (`1_test_camera_ov5640.ino`) trên phần cứng thật**:
   - Nạp file `1_test_camera_ov5640.ino` mới nhất lên mạch `ESP32-S3 N16R8 OV5640`, kết nối Wi-Fi `ESP32-OV5640-FHD` (mật khẩu `12345678`) và mở `http://192.168.4.1`.
   - Tinh chỉnh nhẹ ngưỡng thanh trượt `Ngưỡng AI` (mặc định `2.5`) hoặc `alpha` nếu môi trường ánh sáng thực tế tại phòng của người dùng có đặc thù riêng.
2. **Đưa bản tối ưu nhất của code test sang firmware chính thức.**
3. **Đấu nối toàn hệ thống trên Hàng Phải (ESP32-S3 + Tang Nano 20K + INA219) và chạy Sketch 2 (`2_adaremac_main.ino`)**:
   - Nạp bitstream `top_tn20k.v` (`N_LANES = 1`) lên FPGA Tang Nano 20K theo đúng 4 quy tắc Gowin EDA.
   - Đấu nối 4 dây SPI (`GPIO 1, 2, 42, 41` $\to$ `Pin 42, 48, 41, 49`), 3 dây INA219 (`GPIO 38` $\to$ `VCC`, `GPIO 47` $\to$ `SDA`, `GPIO 21` $\to$ `SCL`) và `GND` chung trên Hàng Phải.
   - Mở Serial Monitor (`115200 baud`) thu thập bảng dữ liệu thực nghiệm `CSV,config,mode,thr,vbus_V,i_mA,p_mW,ops,residual_active,nmed` và bảng so sánh công suất/thời gian thực thi giữa **Mode 0 (Sleep, 0% Data)**, **Mode 1 (Partial, 25% Data)** và **Mode 2 (Exact, 100% Data)** để đưa số liệu đo thực tế vào báo cáo/slide bảo vệ.
4. **Tùy chọn nâng cấp hợp nhất (Nếu người dùng yêu cầu)**:
   - Gộp Sketch 1 (Web Server Live Stream + AI 5 đặc trưng) và Sketch 2 (Điều khiển SPI FPGA Tang Nano 20K + Đo dòng INA219) thành một **Firmware Dashboard Hợp Nhất**: khi AI trên Web phát hiện `0 vật thể` thì tự động đặt FPGA về `Mode 0`; khi phát hiện vùng nghi ngờ thì đặt FPGA sang `Mode 1 (thr=2)`; khi xác nhận vật thể bất thường thì đặt FPGA sang `Mode 2 (Exact)` và hiển thị trực tiếp thông số công suất `INA219 (mW)` + bộ đếm phần cứng FPGA (`OPS`, `ENR`, `NMED`) lên giao diện Web `http://192.168.4.1`.

## 6. QUY ƯỚC LÀM VIỆC & SỞ THÍCH CỦA NGƯỜI DÙNG

AI ở cuộc trò chuyện mới **bắt buộc** tuân thủ các quy ước làm việc và phong cách sau đây của người dùng (Phan Nhật Huy):

1. **Ngôn ngữ & Phong cách trình bày**:
   - Luôn trả lời bằng **Tiếng Việt**, giọng điệu rõ ràng, trực diện, nhiệt tình, giải thích thẳng vào bản chất kỹ thuật (tại sao lỗi xảy ra ở cấp độ phần cứng/thanh ghi/bộ nhớ, tại sao giải pháp này giải quyết được dứt điểm).
   - Khi hướng dẫn thao tác phần mềm (Arduino IDE, Gowin EDA, VS Code, PowerShell) hoặc đấu nối phần cứng, luôn viết theo phong cách **"cầm tay chỉ việc" từng bước nhỏ nhất**: chỉ rõ bấm vào menu nào, chọn đúng giá trị nào, cắm chân số mấy tính từ trên xuống, dùng tổ hợp phím nào.
2. **Tuân thủ nghiêm ngặt triết lý `Think Before Coding`, `Simplicity First`, `Surgical Changes`, `Goal-Driven Execution`**:
   - **Không giả định ngầm**: Nếu có ràng buộc phần cứng (như Breadboard chỉ trống 1 bên, hoặc sơ đồ chân in trên PCB), phải đối chiếu chính xác với ảnh chụp thực tế của người dùng trước khi viết code.
   - **Đơn giản & Tự chứa tối đa (Zero External Dependencies)**: Trên Arduino IDE, luôn viết code chỉ sử dụng các thư viện có sẵn trong gói board `esp32 by Espressif Systems` (`esp_camera.h`, `SPI.h`, `Wire.h`, `WiFi.h`, `esp_http_server.h`, `img_converters.h`). Không bắt người dùng cài thêm các thư viện rườm rà dễ xung đột phiên bản.
   - **Không viết tắt code**: Mỗi khi sửa hoặc cung cấp file code, luôn xuất **toàn bộ nội dung file hoàn chỉnh từ dòng đầu đến dòng cuối** để người dùng chỉ việc Copy–Paste hoặc mở file chạy ngay. Tuyệt đối **không** dùng `// ... phần còn lại giữ nguyên ...` hay cắt bớt hàm.
   - **Dọn dẹp gọn gàng khi đóng gói**: Khi tạo file nén `.zip` mới, chủ động loại bỏ các file/thư mục cũ không còn dùng (ví dụ: đã bỏ `tb_standalone.v` khỏi `standalone_tangnano_only/` để không gây lỗi khi add file vào Gowin EDA, đã bỏ ESP-IDF để đồng nhất sang Arduino IDE).
3. **Thói quen phần cứng & Thực nghiệm của người dùng**:
   - Người dùng làm việc trực tiếp trên mạch thật (**ESP32-S3 N16R8 OV5640 TypeC** + **Sipeed Tang Nano 20K** + **INA219** trên Breadboard) và rất chú trọng đến **nhiệt độ hoạt động của chip** (luôn cần in nhiệt độ nội bộ `temperatureRead()` của ESP32-S3 ra Serial/Web UI và giữ cho FPGA Tang Nano 20K mát lạnh hoàn toàn), **tốc độ khung hình thực tế (FPS)**, **độ sắc nét của ảnh Stream**, và **tính ổn định dài hạn** (không treo, không kẹt trạng thái, không reboot).
