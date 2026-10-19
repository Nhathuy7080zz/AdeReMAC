# BẢN BÀN GIAO TOÀN DIỆN DỰ ÁN ADAREMAC-EDGEVISION (PHIÊN BẢN v4.3 — HỢP NHẤT ĐẦY ĐỦ)

Tài liệu này hợp nhất toàn bộ nội dung của **`hand_over.md`** (chuẩn 7 phần theo hướng dẫn bàn giao), **`BAN_BAN_GIAO_DU_AN_ADAREMAC.md`** (hướng dẫn đấu nối, nạp code, nạp bitstream FPGA, huấn luyện & tinh chỉnh mô hình, quản lý nhiệt) và **`README`** vào một tệp duy nhất. Bạn chỉ cần mang tệp `hand_over.md` này (kèm `README.md` chứa Báo cáo kỹ thuật) sang cuộc trò chuyện mới là AI tiếp nhận có đầy đủ 100% thông tin và toàn bộ mã nguồn để làm việc tiếp ngay lập tức, không cần hỏi lại.

---

## 1. MỤC TIÊU DỰ ÁN & BẢNG TÓM TẮT TRẠNG THÁI

### 1.1. Dự án làm gì và vì sao
- **Tên dự án**: **AdaReMAC-EdgeVision** (*Adaptive Reconfigurable Multiply-Accumulate Co-Design for Real-Time Embedded Vision*).
- **Mục tiêu cốt lõi**: Thiết kế hệ thống đồng xử lý Phần cứng – Phần mềm (*Hardware–Software Co-Design*) nhận dạng đối tượng/người thời gian thực theo kiến trúc **FOMO (Faster Objects, More Objects — Edge Impulse)** kết hợp **Lưới Siêu Mịn Đa Tỷ Lệ (`Fine-Grid 48×48 = 2.304 ô`)** và truyền phát luồng video MJPEG sắc nét (mục tiêu $\ge 30\text{ FPS}$) trên vi điều khiển **ESP32-S3 N16R8 CAM (cảm biến OV5640)** phối hợp với bộ tăng tốc suy luận **INT8** dùng **Bộ nhân xấp xỉ đa chế độ AdaReMAC** trên **FPGA Sipeed Tang Nano 20K (Gowin GW2AR-LV18QN88C8/I7)**.
- **Động lực khoa học & kỹ thuật**:
  1. Đánh giá và chứng minh hiệu quả của **Bộ nhân xấp xỉ tái cấu hình thời gian thực AdaReMAC** (`Mode 0`: xấp xỉ tiết kiệm năng lượng kèm bù độ lệch bias theo kênh; `Mode 1`: kích hoạt nhánh bù chọn lọc khi $D_2 \ge \text{thr}$; `Mode 2`: chính xác từng bit 100%) trên mạng tích chập phát hiện đối tượng thực tế.
  2. Đạt đồng thời tốc độ truyền phát luồng video MJPEG sắc nét ($\ge 30\text{ FPS}$, tách rời hoàn toàn khỏi chu kỳ suy luận AI) và khả năng đóng khung bao (*Bounding Box*) bám sát đường biên vật thể trên giao diện Web UI trực quan.
  3. Giải quyết bài toán quá nhiệt và mất nét của cảm biến **OV5640** hoàn toàn bằng phần mềm (qua thanh ghi SCCB/I2C: điều khiển DAC cuộn dây lấy nét VCM, làm nét cạnh phần cứng ISP CIP, đưa PLL về dải chuẩn, quản lý dòng đệm chân DVP và chế độ ngủ xen kẽ khi rảnh) mà không cần độ chế phần cứng cáp FPC.

### 1.2. Bảng Tóm tắt Trạng thái Dự án (Cập nhật tới v4.3)

| Hạng mục | Trạng thái v4.3 | Đã kiểm chứng bằng |
|---|---|---|
| **Phát hiện người FOMO (Edge Impulse pipeline) huấn luyện trên COCO 2017 đã gán nhãn** | Xong — F1 theo tâm `0,307` (trong hộp `0,401`) trên COCO val (`4.535` ảnh); đi kèm quy trình tinh chỉnh theo cảnh thật bằng `auto_label.py` | Chạy đánh giá thật (`ai_fomo/results/int8_eval.json`) |
| **Cùng mô hình INT8 chạy trên ESP32-S3 (C) và Tang Nano 20K (RTL)** | Xong — trùng khớp từng bit (`0/144` logit lệch) với Python | `sim/test_fomo_c.c`, `sim/tb_fomo_top.v` |
| **Bộ tăng tốc FPGA dùng bộ nhân AdaReMAC** | `350.784` chu kỳ = `12,99 ms/khung` ở `27 MHz` (mô phỏng RTL) | Mô phỏng Icarus Verilog; Yosys ước tính tài nguyên (`~30%` LUT4, `67%` BSRAM) |
| **Làm mịn khả năng nhận dạng (Lưới siêu mịn `48×48 = 2.304` ô trên cùng diện tích `96×96`)** | Xong — tăng gấp `16×` mật độ ô lưới so với `12×12`, kết hợp lan truyền vùng song phương (Bilateral Silhouette Growing) & khóa biên Sobel giúp đóng khung sát biên vật thể | `sim/test_fomo_c.c` (mở rộng từ `5–8` ô thô lên `135–162` ô mịn bám sát biên) |
| **Làm nét luồng Stream OV5640 & Sửa lỗi giải mã `esp_new_jpeg` ở SVGA/HD** | Xong — mặc định `SVGA 800×600` (`jpeg_quality = 8`), bật ISP CIP Sharpness + LENC + RAW_GMA, điều khiển lấy nét cuộn dây VCM DAC + Auto-Focus Sweep, sửa hàm `choose_idct_scale()` | Kiểm tra mã nguồn & biên dịch chuẩn `esp32` core 3.x |
| **Giao diện Web UI Dashboard 2 cột & Khung bao tương phản kép** | Xong — bố cục 2 cột không cuộn tràn màn hình, Canvas HiDPI vẽ khung viền kép + 4 góc chữ L + hiển thị ô lưới mịn bám theo hình dáng vật thể | `firmware_arduino/adaremac_fomo_cam/web_ui.cpp` |
| **Giảm nhiệt OV5640 bằng phần mềm** | Đã cài đặt 6 biện pháp an toàn (không chạm `0x3031[3]`, không ghi `0x4202 = 0x0F`, khóa ngủ khi đang xem stream) | Cần đo nhiệt độ & dòng thực tế trên board theo quy trình mục 2.7 |

### 1.3. Yêu cầu và Ràng buộc Phần cứng & Môi trường
- **Phần cứng mục tiêu**:
  - **Vi điều khiển & Camera**: Bo mạch **ESP32-S3-CAM N16R8** (Dual-core Xtensa LX7 @ 240 MHz, 16 MB Flash, **8 MB Octal-SPI PSRAM**, ~512 KB Internal SRAM) gắn cảm biến **OmniVision OV5640 (5 MP, giao tiếp song song 8-bit DVP, thấu kính có cuộn dây lấy nét VCM)**.
  - **FPGA Đồng xử lý**: **Sipeed Tang Nano 20K** (chip **Gowin GW2AR-LV18QN88C8/I7**, package **QFN88**, Version **C**, 20.736 LUT4, 46 khối BSRAM 18 Kbit, xung hệ thống 27 MHz tại Pin 4).
  - **Cảm biến dòng điện (tùy chọn)**: **INA219** giao tiếp I2C (`SDA = GPIO 47`, `SCL = GPIO 21`, địa chỉ `0x40`).
- **Môi trường & Phiên bản công cụ**:
  - **Arduino IDE 2.x** / `arduino-cli` với gói **esp32 by Espressif $\ge 3.0$** (đã kiểm chứng biên dịch với bản **3.3.12**).
  - **Gowin EDA**: Tổng hợp RTL và nạp bitstream `.fs` cho `GW2AR-LV18QN88C8/I7`.
  - **Mô phỏng & Huấn luyện trên máy tính**: `Icarus Verilog >= 11` (`iverilog`, `vvp`), `gcc` / `g++`, `Python 3.10+` (`numpy`, `opencv-python`, `torch`, `torchvision`, `duckdb`).

---

## 2. KIẾN TRÚC HIỆN TẠI & HƯỚNG DẪN VẬN HÀNH TOÀN DIỆN (ESP32-S3 + TANG NANO 20K + AI)

### 2.1. Sơ đồ khối & Luồng dữ liệu hệ thống (v4.3)
1. **Task Camera (`cam_task` trên Core 1, ưu tiên 5)**:
   - Đọc khung JPEG từ DMA của OV5640 (`esp_camera_fb_get()`), chép tức thì vào kho đệm vòng 4 ô trong PSRAM (`frame_hub.cpp`, mất `~0,3 ms`) rồi trả framebuffer ngay lập tức (`esp_camera_fb_return()`).
   - Xử lý yêu cầu quét lấy nét tự động VCM (`cam_auto_focusSweep()`) an toàn giữa hai khung hình khi người dùng bấm nút **🎯 Tự lấy nét (AF)** trên Web UI.
   - Quản lý chế độ ngủ tiết kiệm nhiệt (`cam_standby(true)` -> `0x3008 = 0x42`) **chỉ khi không có trình duyệt nào đang xem (`stream_clients == 0`)**.
2. **Server Luồng Video MJPEG (`stream_h` tại cổng HTTP `:81` trên Core 1)**:
   - Đọc khung JPEG mới nhất từ `frame_hub` mà không chặn `cam_task` hay `det_task`, gửi qua socket TCP bật `TCP_NODELAY`.
3. **Đường ống Nhận dạng 2 Lõi (`prep_task` trên Core 1 + `det_task` trên Core 0)**:
   - **`prep_task` (Core 1, ưu tiên 3)**: Lấy khung JPEG mới nhất từ `frame_hub`, dùng bộ giải mã SIMD phần cứng `esp_new_jpeg` thu nhỏ trực tiếp ở tầng IDCT theo tỷ lệ lũy thừa 2 hợp lệ (`choose_idct_scale()`: `1/8`, `1/4` hoặc `1/2` ra định dạng `CbYCrY`), trích thẳng kênh độ sáng `Y` chuẩn BT.601 và nội suy song tuyến về ảnh xám `96×96` trong PSRAM (`s_gray[0..2]`), đẩy chỉ số qua hàng đợi `s_ready_q`.
   - **`det_task` (Core 0, ưu tiên 4)**:
     - Nhường CPU mỗi vòng (`vTaskDelay(1)`) để nuôi `IDLE0` watchdog và stack Wi-Fi/lwIP.
     - Chạy suy luận mạng **FOMO-Micro INT8** (6 lớp Conv, 4,15 triệu MAC, 24,5 nghìn tham số) trên **1 trong 3 Backend**:
       1. **FPGA Tang Nano 20K (`BK_FPGA`)**: Gửi 9.216 byte ảnh `96×96` qua SPI 20 MHz (`CMD 0x01`), FPGA chạy song song 16 làn MAC AdaReMAC trong `350.784 chu kỳ` (`12,99 ms @ 27 MHz`), trả về 144 logit `12×12` (`CMD 0x02`) theo cơ chế đường ống chồng lấp (*Pipelined Overlap*: trong lúc FPGA tính khung $n$, ESP32 giải mã khung $n+1$).
       2. **ESP32-S3 CPU (`BK_CPU`)**: Chạy bằng thư viện vector SIMD **`esp-nn` (`fomo_espnn.c`)** hoặc bản C chuẩn bit-exact **`fomo_micro.c`**.
       3. **Edge Impulse SDK (`BK_EI`, tùy chọn)**: Gọi `run_classifier()` của thư viện Arduino xuất từ Edge Impulse Studio.
     - **Giải mã Lưới Siêu Mịn Đa Tỷ Lệ (`fomo_decode_fine()` trong `fomo_micro.c`)**:
       - Chia khung ảnh `96×96` thành lưới siêu mịn **`48×48` (`2.304` ô, mật độ gấp 16 lần lưới `12×12`)** hoặc `24×24` (`576` ô).
       - Nội suy song tuyến logit `12×12` lên lưới `48×48`, kết hợp **Lan truyền vùng song phương (Bilateral Silhouette Region Growing)** và **Khóa biên Gradient Sobel** để xác định chính xác tọa độ 4 biên `[x0, y0, x1, y1]` bám sát viền vật thể và xuất bản đồ lưới mịn `fgrid[576]` (`24×24`).
     - **Bám vết mượt (`track_update()`)**: Khớp láng giềng gần nhất giữa các khung, lọc thông thấp EMA (`alpha = 0.60`) trên cả tâm `(cx, cy)` và 4 biên `(x0, y0, w, h)` để khung bao bám sát vật thể mượt mà.
4. **Server Điều khiển & Giao diện Web UI (`s_web` tại cổng HTTP `:80` trên Core 1)**:
   - Phục vụ giao diện Dashboard 2 cột (`/`), JSON kết quả nhận dạng + bản đồ lưới mịn (`/det?heat=1` ở chu kỳ `55 ms`), JSON trạng thái phần cứng (`/status` ở chu kỳ `1000 ms`), API điều khiển trực tiếp (`/ctrl?var=...&val=...`), và chụp ảnh tải về (`/capture`).

### 2.2. Cấu trúc thư mục dự án

```text
adaremac/
├── hand_over.md                           # Bản bàn giao hợp nhất đầy đủ 7 mục kèm 100% code v4.3 (File này)
├── README.md                              # Báo cáo khoa học & kỹ thuật toàn diện (v4.0 -> v4.3)
├── BAN_BAN_GIAO_DU_AN_ADAREMAC.md         # Bản tóm tắt hướng dẫn vận hành
├── BAO_CAO_KY_THUAT_ADAREMAC_V4.md        # Bản sao báo cáo kỹ thuật v4.3
├── Model Selection.pdf                    # Tài liệu gốc định hướng lựa chọn mô hình
├── firmware_arduino/adaremac_fomo_cam/    # Firmware duy nhất cho ESP32-S3 N16R8 CAM (OV5640)
│   ├── adaremac_fomo_cam.ino              # Điểm vào setup()/loop(), cam_task, quản lý nhiệt & VCM AF
│   ├── app_config.h                       # Cấu hình chân, Wi-Fi, SVGA Q8, ISP Sharpness, VCM, Fine-Grid 48x48
│   ├── cam_ov5640.h / cam_ov5640.cpp      # Driver OV5640: ISP CIP Clarity, VCM DAC Focus, Auto-Focus Sweep, PLL
│   ├── frame_hub.h / frame_hub.cpp        # Kho đệm vòng 4 khung JPEG không khóa lâu trong PSRAM
│   ├── detector.h / detector.cpp          # Pipeline 2 lõi: esp_new_jpeg IDCT -> FOMO -> Fine-Grid 48x48 -> Tracker
│   ├── fomo_micro.h / fomo_micro.c        # Suy luận FOMO-Micro INT8 C chuẩn + Giải mã lưới siêu mịn fomo_decode_fine()
│   ├── fomo_espnn.c                       # Tăng tốc suy luận FOMO-Micro bằng SIMD ESP-NN trên ESP32-S3
│   ├── fomo_micro_model.h                 # Trọng số, bias và hệ số lượng tử INT8 của 6 lớp FOMO-Micro
│   ├── fpga_link.h / fpga_link.cpp        # Giao thức SPI Master 20MHz giao tiếp với Tang Nano 20K (0x01..0x04)
│   └── web_ui.h / web_ui.cpp              # HTTP Server cổng 80/81 + Giao diện Web UI 2 cột + Canvas HiDPI
├── rtl/                                   # Mã nguồn Verilog RTL cho FPGA Sipeed Tang Nano 20K
│   ├── fomo_top_tn20k.v                   # Top-level bộ tăng tốc FOMO-Micro qua SPI (Pin 41, 42, 48, 49)
│   ├── fomo_spi_if.v                      # Giao tiếp SPI Slave 20MHz dịch bit trực tiếp trong miền SCLK
│   ├── fomo_accel_core.v                  # Lõi tăng tốc 6 lớp Conv INT8 với 16 làn MAC AdaReMAC song song
│   ├── fomo_mac_lane.v                    # Làn nhân-cộng dấu-độ lớn sử dụng bộ nhân adaremac_mul
│   ├── adaremac_mul.v                     # Bộ nhân xấp xỉ đa chế độ AdaReMAC 8x8 không dấu (Mode 0/1/2)
│   ├── fomo_model.vh & mem/fomo_*.hex     # Cấu hình lớp và bộ nhớ BSRAM trọng số/bias/bù Mode 0/M
│   └── microbench/                        # Demo đo vi kiến trúc AdaReMAC chỉ dùng Tang Nano 20K qua UART 115200
├── constraints/                           # Ràng buộc chân (.cst) và xung nhịp (.sdc) cho Gowin EDA
│   ├── tn20k_fomo.cst / tn20k_fomo.sdc
│   └── tn20k_standalone.cst / tn20k_standalone.sdc
├── sim/                                   # Kiểm chứng tự động C và RTL (run_sim.sh, tb_mul, tb_standalone, tb_fomo_top, test_fomo_c.c)
├── ai_fomo/                               # Dữ liệu COCO, huấn luyện, lượng tử INT8, gán nhãn tự động & xuất mô hình FOMO
├── ai_eval/                               # Đánh giá bộ nhân xấp xỉ AdaReMAC trên MNIST thật (từ v3)
└── legacy/                                # Lưu trữ mã nguồn và tài liệu bản v3 cũ
```

### 2.3. Sơ đồ Đấu nối Phần cứng (ESP32-S3 $\leftrightarrow$ Tang Nano 20K & INA219)

Dây nối SPI giữa ESP32-S3 và Sipeed Tang Nano 20K giữ nguyên:

| ESP32-S3 N16R8 | Sipeed Tang Nano 20K | Tín hiệu | Ghi chú |
|---|---|---|---|
| **GPIO 1** | **Pin 42** | `SPI_CS` | Chip Select (Active Low) |
| **GPIO 2** | **Pin 48** | `SPI_MOSI` | Master Out Slave In |
| **GPIO 42** | **Pin 41** | `SPI_SCLK` | Xung nhịp SPI (`20 MHz`) |
| **GPIO 41** | **Pin 49** | `SPI_MISO` | Master In Slave Out |
| **GND** | **GND** | `GND` | **Bắt buộc nối chung GND** |

- Dùng dây cắm ngắn ($\le 10\text{ cm}$). Nếu kết quả đọc từ FPGA chập chờn do nhiễu dây cắm, giảm `FPGA_SPI_HZ` trong `app_config.h` từ `20000000` xuống `10000000` (`10 MHz`).
- **Cảm biến dòng INA219** (tùy chọn, dùng để đo dòng kiểm chứng giảm nhiệt): nối `SDA = GPIO 47`, `SCL = GPIO 21`, đặt `#define INA219_ENABLE 1` trong `app_config.h`. Lấy nguồn `VCC` cho INA219 từ chân `3V3` của bo mạch (không cấp nguồn từ chân GPIO).

### 2.4. Hướng dẫn Nạp & Chạy Firmware ESP32-S3
1. Cài đặt **Arduino IDE 2.x**, cài gói **esp32 by Espressif $\ge 3.0$** (đã kiểm chứng biên dịch với bản `3.3.12`).
2. Mở tệp `firmware_arduino/adaremac_fomo_cam/adaremac_fomo_cam.ino`.
3. Thiết lập trong menu **Tools**:
   - **Board**: `ESP32S3 Dev Module`
   - **PSRAM**: **`OPI PSRAM`** *(Bắt buộc bật OPI PSRAM)*
   - **Flash Size**: `16MB (128Mb)`
   - **Partition Scheme**: `Huge APP (3MB No OTA/1MB SPIFFS)`
4. Nhấn **Upload**, sau đó mở **Serial Monitor (`115200 baud`)**. Mỗi `2 giây` firmware in ra tốc độ `cam fps`, `stream fps`, `AI fps`, mật độ lưới mịn (`48x48`), thời gian giải mã (`esp_new_jpeg`) và suy luận (`esp-nn` / `FPGA`), số đối tượng phát hiện và nhiệt độ chip.
5. Kết nối vào mạng Wi-Fi **`AdaReMAC-FOMO`** (mật khẩu `12345678`), mở trình duyệt truy cập **`http://192.168.4.1`**:
   - Chọn **Mật độ ô lưới nhận dạng (Fine-Grid)**: `48×48 Siêu mịn (2.304 ô)` (mặc định, bám sát biên vật thể nhất), `24×24 Mịn (576 ô)` hoặc `12×12 Chuẩn FOMO (144 ô)`.
   - Chỉnh **Ngưỡng phát hiện**, **Backend** (`Tự động`, `ESP32-S3 CPU`, `Tang Nano 20K`, `Edge Impulse`) và **Mode AdaReMAC** (`Mode 2`, `Mode 1`, `Mode 0`).
   - Chỉnh **Độ phân giải Stream** (`HD 1280×720`, `XGA 1024×768`, `SVGA 800×600`, `VGA 640×480`, `QVGA 320×240`), **Độ sắc nét cạnh ISP (`0..6`)**, **Chất lượng nén JPEG (`6..20`)**, **Tiêu cự thấu kính VCM (`0..255`)** hoặc bấm nút **🎯 Tự lấy nét (AF)**.
   - Bấm nút **📷 Chụp ảnh (.jpg)** để tải khung hình JPEG gốc về máy tính phục vụ thu thập dữ liệu.

#### Dùng mô hình Edge Impulse Studio thay cho FOMO-Micro (tùy chọn):
1. Tạo thư mục dữ liệu chuẩn định dạng Edge Impulse từ tập COCO:
   ```bash
   python ai_fomo/export_edge_impulse.py --parquet "data/coco_parquet/train_0.parquet" --out ei_upload --max 3000
   edge-impulse-uploader --category split --directory ei_upload
   ```
2. Trong Edge Impulse Studio: Tạo Impulse **Image 96×96 (Grayscale) $\to$ Object Detection (Images)**, chọn **FOMO (MobileNet V2 0.35)**, nhấn Train, sau đó vào **Deployment $\to$ Arduino library** (bật EON Compiler) và cài tệp `.zip` vào Arduino IDE.
3. Trong `app_config.h`, bỏ chú thích `#define USE_EDGE_IMPULSE 1` và sửa `EI_HEADER` đúng tên tệp header của dự án Edge Impulse, rồi chọn backend `"Edge Impulse"` trên giao diện web.

### 2.5. Hướng dẫn Nạp Bitstream lên FPGA Tang Nano 20K (Gowin EDA)
1. Mở **Gowin EDA**, tạo project mới cho chip **`GW2AR-LV18QN88C8/I7`**:
   - Package: **QFN88** (chú ý không chọn QFN88P); Device: **GW2AR-18**; Version: **C**.
2. Thêm 5 tệp RTL và 2 tệp ràng buộc:
   - `rtl/fomo_top_tn20k.v`, `rtl/fomo_spi_if.v`, `rtl/fomo_accel_core.v`, `rtl/fomo_mac_lane.v`, `rtl/adaremac_mul.v`.
   - `constraints/tn20k_fomo.cst`, `constraints/tn20k_fomo.sdc`.
   - *(Không thêm các tệp trong thư mục `rtl/microbench/` khi tổng hợp bộ tăng tốc FOMO).*
3. Lưu ý đường dẫn bộ nhớ BSRAM: Tệp `fomo_model.vh` phải nằm cùng thư mục với `fomo_accel_core.v`. Bốn tệp `rtl/mem/fomo_*.hex` được nạp qua `$readmemh`; nếu Gowin EDA trên Windows báo không tìm thấy tệp `.hex`, hãy sửa dòng `` `define FOMO_MEM_DIR "" `` ở đầu `fomo_accel_core.v` thành đường dẫn tuyệt đối (dùng dấu `/`, ví dụ `"D:/adaremac/rtl/mem/"`).
4. Vào **Project $\to$ Configuration**:
   - Đặt **Top Module** = `fomo_top_tn20k`.
   - Trong mục **Dual-Purpose Pin**: Tích chọn *Use MSPI as regular IO* / *Use SSPI as regular IO* / *Use DONE/READY as regular IO* nếu trình tổng hợp báo trùng chân 41/42/48/49.
5. Nhấn **Run All (Synthesize + Place & Route)**, mở **Timing Analysis Report** kiểm tra $F_{\max}$ của `sys_clk` phải đạt $> 27\text{ MHz}$, sau đó nạp xuống mạch qua **Gowin Programmer**.
6. **Bảng ý nghĩa 6 đèn LED trên Tang Nano 20K (tích cực mức thấp — sáng = 1)**:

| LED | Chân Pin | Ý nghĩa trạng thái |
|---|---|---|
| **LED 0** | Pin 15 | FPGA đang bận chạy suy luận (`acc_busy = 1`) |
| **LED 1** | Pin 16 | Có kết quả suy luận mới chưa được ESP32 đọc (`res_new = 1`) |
| **LED 2** | Pin 17 | Đã nhận giao dịch SPI từ ESP32-S3 (`spi_seen = 1`) |
| **LED 3** | Pin 18 | Nhịp tim xung hệ thống 27 MHz (`hb[24]`) |
| **LED 4** | Pin 19 | Báo lỗi: ESP32 ghi khung mới khi FPGA đang bận (`err_wr_busy = 1`) |
| **LED 5** | Pin 20 | Lớp 1 đang chạy ở chế độ xấp xỉ (`Mode 0` hoặc `Mode 1`, khác `Mode 2`) |

*(Lưu ý: Gói demo đo kiểm vi kiến trúc độc lập chỉ dùng Tang Nano 20K xuất số liệu qua cáp USB-C UART 115200 vẫn được giữ nguyên trong `rtl/microbench/` kèm `constraints/tn20k_standalone.*`).*

### 2.6. Quy trình Huấn luyện lại, Xuất Mô hình & Tinh chỉnh theo Cảnh thực tế (`auto_label.py`)
```bash
# 1) Chuẩn bị dữ liệu từ các shard COCO 2017 parquet:
python ai_fomo/prepare_coco_person.py --parquet "data/coco_parquet/train_*.parquet" --out data/coco_person_train96.npz
python ai_fomo/prepare_coco_person.py --parquet "data/coco_parquet/val_*.parquet" --out data/coco_person_val96.npz --neg-ratio 1.0

# 2) Huấn luyện mô hình thầy (tùy chọn) và mô hình FOMO-Micro:
python ai_fomo/train_fomo.py --model teacher --lr 1e-3 --epochs 8 --train data/coco_person_train96.npz --val data/coco_person_val96.npz --out ai_fomo/runs/teacher
python ai_fomo/train_fomo.py --model micro --epochs 30 --train data/coco_person_train96.npz --val data/coco_person_val96.npz --out ai_fomo/runs/micro

# 3) Lượng tử hóa INT8 + đánh giá các chế độ AdaReMAC + tự động xuất fomo_micro_model.h, rtl/fomo_model.vh, rtl/mem/*.hex và sim/vectors/:
python ai_fomo/export_fomo_micro.py --ckpt ai_fomo/runs/micro/model.pt --train data/coco_person_train96.npz --val data/coco_person_val96.npz

# 4) Kiểm chứng bit-exact toàn bộ RTL và C + kiểm tra lưới siêu mịn 48x48:
cd sim
./run_sim.sh 3
gcc -O2 -I../firmware_arduino/adaremac_fomo_cam test_fomo_c.c ../firmware_arduino/adaremac_fomo_cam/fomo_micro.c -lm -o test_fomo_c
./test_fomo_c vectors
```

- **Tinh chỉnh cho đúng bối cảnh lắp đặt camera của nhóm (không cần gán nhãn tay)**:
  1. Thu vài trăm khung ảnh ở các điều kiện ánh sáng khác nhau bằng nút **📷 Chụp ảnh (.jpg)** trên Web UI (`http://192.168.4.1/capture`).
  2. Chạy `python ai_fomo/auto_label.py --images captures --out captures_labeled` (dùng Faster R-CNN MobileNetV3-FPN tự động đóng khung nhãn `person`).
  3. Kiểm tra nhanh thư mục `captures_labeled/preview/`, sau đó dùng `captures_labeled/labeled96.npz` để tinh chỉnh (`--init ai_fomo/runs/micro/model.pt`) bằng `train_fomo.py` rồi chạy lại `export_fomo_micro.py`.

### 2.7. Bảng Cấu hình Quản lý Nhiệt & Độ nét OV5640 bằng Phần mềm (`app_config.h`)

| Tùy chọn trong `app_config.h` / Web UI | Mặc định v4.3 | Tác dụng kỹ thuật |
|---|---|---|
| `STREAM_FRAMESIZE` | `FRAMESIZE_SVGA` (`800×600`) | Tăng 56% độ chi tiết so với VGA nhưng vẫn giữ chế độ **2×2 hardware binning** (`<= 1280×960`) giúp cảm biến mát và đạt $\ge 30\text{ FPS}$ |
| `STREAM_JPEG_QUALITY` | `8` | Giảm mạnh nhiễu khối DCT `8×8` so với mức `12` của v4.2 |
| `CAM_DEFAULT_SHARPNESS` | `3` (trên thang `0..6`) | Kích hoạt bộ tăng cường biên cạnh phần cứng **ISP CIP** (`0x5308`, `0x5302`, `0x5303`) + bật `LENC`, `RAW_GMA`, `BPC/WPC`, `DCW` |
| `CAM_DEFAULT_VCM_STEP` & Nút `🎯 Tự lấy nét (AF)` | `140` (trên thang `0..255`) | Cấp dòng DAC 10-bit (`0x3602`, `0x3603`) cho cuộn dây lấy nét VCM đúng cự ly trong phòng (`0,4 m – 2,5 m`) hoặc tự động quét tìm đỉnh nét |
| `THERMAL_ECO_DEFAULT` | `0` (Tắt lúc khởi động) | Giữ luồng video liên tục sắc nét; khi bật "Ngủ tiết kiệm nhiệt" và **không có người xem (`stream_clients == 0`)**, cảm biến ngủ (`0x3008 = 0x42`) và thức mỗi `1 s` (bỏ `6` khung để AE/AWB hội tụ) |
| `THERMAL_PLL_IN_SPEC` | `1` | Hạ tần số VCO PLL1 từ `1,8 GHz` (`x180 / 4`) về `900 MHz` (`x90 / 2`) nằm chuẩn trong dải `500–1000 MHz` của driver Linux mà giữ nguyên FPS |
| `THERMAL_PAD_DRIVE_1X` | `0` (Dùng mức `2x` = `0x42`) | Giảm một nửa dòng lái I/O so với mức `4x` (`0xC3`) mặc định nhưng vẫn đảm bảo sườn xung `PCLK` sắc nét ở SVGA/HD |

---

## 3. QUYẾT ĐỊNH ĐÃ CHỐT (VÀ LÝ DO KỸ THUẬT)

1. **Chọn kiến trúc FOMO (Faster Objects, More Objects) trên cả ESP32-S3 và FPGA Tang Nano 20K (thay vì YOLOv5-nano hay YOLOv2-Tiny)**:
   - *Lý do*: Trên ESP32-S3, YOLOv5-nano chỉ đạt `2–5 FPS`, không đáp ứng mục tiêu thời gian thực $\ge 30\text{ FPS}$. Trên FPGA Tang Nano 20K (`GW2AR-18`), YOLOv2-Tiny cần `3,49 GMAC/khung` và `15,9 MB` trọng số, vượt quá `103 KB` BSRAM nội bộ và chỉ đạt tối đa `~1,4 FPS` với 48 bộ nhân DSP. Kiến trúc **FOMO-Micro** (`4,15 triệu MAC/khung`, `24,5 KB` trọng số INT8, chỉ gồm `Conv 3×3 / 1×1 + ReLU`) nằm gọn 100% trong BSRAM của Tang Nano 20K và chạy hết chỉ `12,99 ms/khung` ở `27 MHz`.
2. **Dùng dữ liệu thật đã gán nhãn sẵn từ COCO 2017 (`person`) kết hợp công cụ tự động gán nhãn cảnh thật `auto_label.py` (Faster R-CNN)**:
   - *Lý do*: Loại bỏ hoàn toàn dữ liệu tổng hợp giả tạo (`rng.uniform`) của bản v3 cũ, đồng thời đáp ứng yêu cầu không phải vẽ nhãn thủ công. (Lưu ý: Không sử dụng lại các con số không có cơ sở đo của bản v3 cũ như `99,97%`, `12.000 mẫu`, `< 1,25 ms`, `110 -> 58 mA` khi viết báo cáo hay thuyết minh).
3. **Bù độ lệch hệ thống cho Mode 0 của AdaReMAC trực tiếp vào Bias (`mode0_bias_compensation`)**:
   - *Lý do*: Sai số của bộ nhân AdaReMAC ở Mode 0 chỉ phụ thuộc vào 3 bit thấp của hai toán hạng. Trừ kỳ vọng sai số theo từng kênh vào mảng `comp_mem` ngay khi lượng tử hóa giúp **Mode 0 (tắt hoàn toàn mạch bù phần dư trên toàn mạng)** giữ nguyên điểm F1 (`0,310` so với `0,307` của Mode 2 chính xác) mà không tốn thêm cổng logic phần cứng.
4. **Tách riêng tác vụ giải mã JPEG (`prep_task` trên Core 1, dùng `esp_new_jpeg` SIMD) và tác vụ suy luận (`det_task` trên Core 0, dùng `esp-nn` SIMD), đặt toàn bộ đệm ảnh lớn trong OPI PSRAM**:
   - *Lý do*: Giải mã bằng `tjpgd` trong ROM mất `216 ms` và suy luận C thuần mất `150 ms`. Chuyển sang `esp_new_jpeg` (lấy thẳng kênh `Y` từ `CbYCrY`) + `esp-nn` SIMD và chạy song song trên 2 nhân giúp tăng tốc vượt bậc; đặt bộ đệm ảnh `s_yuv` và `s_gray` trong PSRAM giúp giữ `> 40 KB` Internal SRAM trống cho Wi-Fi và 2 HTTP Server.
5. **Tăng độ mịn nhận dạng bằng tầng Hậu xử lý Lưới Siêu Mịn `48×48` (`2.304` ô, `fomo_decode_fine`) tách biệt với nhân CNN `12×12`**:
   - *Lý do*: Nhân CNN `FOMO-Micro` xuất bản đồ `12×12` (`144` logit) là kích thước cố định trong phần cứng FPGA (`rtl/fomo_accel_core.v`) và chỉ kích hoạt mạnh ở tâm đối tượng. Việc nội suy song tuyến bản đồ `12×12` lên lưới siêu mịn `48×48` (`2.304` ô, gấp 16 lần số ô trên cùng diện tích `96×96`) kết hợp **Lan truyền vùng song phương (Bilateral Silhouette Region Growing)** và **Khóa biên Sobel** giúp đóng khung sát biên vật thể với độ phân giải `1/48` (`2,08%`) mà chỉ mất `< 0,4 ms` trên CPU và **không phải sửa lại RTL hay huấn luyện lại từ đầu**.
6. **Tuyệt đối KHÔNG ghi `0x3031[3] = 1` (tắt LDO 1,5V nội) và KHÔNG ghi `0x4202 = 0x0F` trên OV5640**:
   - *Lý do*: Bo mạch ESP32-S3-CAM cố định không cấp nguồn 1,5V rời từ ngoài vào chân `DVDD`; tắt LDO nội (`0x3031[3] = 1`) sẽ làm sập nguồn lõi số của camera. Thanh ghi `0x4202` là bộ đếm che khung hình (`FRAME CTRL02`); ghi `0x4202 = 0x0F` ở chế độ DVP sẽ khóa 100% khung hình làm mất hoàn toàn luồng stream.

---

## 4. NHẬT KÝ LỖI & CÁCH SỬA (THEO THỨ TỰ THỜI GIAN)

### Mục 1 (Bản v3.0 $\to$ v3.1): Mất hoàn toàn luồng video stream sau khi thêm hàm hạ nhiệt OV5640
- **Triệu chứng**: Sau khi nạp bản cập nhật hạ nhiệt camera, trang web không hiển thị video, `esp_camera_fb_get()` luôn trả về `NULL`.
- **Nguyên nhân gốc**: Ghi nhầm thanh ghi `s->set_reg(s, 0x4202, 0xFF, 0x0F)` (tưởng là tắt riêng MIPI, nhưng theo Datasheet OV5640 mục 7.14 thì `0x4202` là `FRAME CTRL02 - Control Masked Frame Number` làm che toàn bộ 15/15 khung hình DVP) và gọi `0x3008 = 0x42` (Software Standby) khi DMA của `LCD_CAM` đang chạy.
- **Cách đã thử nhưng không hiệu quả**: Bấm nút Reset trên bo ESP32-S3 nhưng không rút cáp nguồn USB-C (do `CAM_PIN_RESET = -1`, chip OV5640 không được reset phần cứng nên vẫn giữ `0x4202 = 0x0F` trong RAM).
- **Đã sửa**: Bỏ hoàn toàn lệnh ghi `0x4202 = 0x0F`, đồng thời chủ động ghi tường minh `0x3008 = 0x02`, `0x4201 = 0x00`, `0x4202 = 0x00` ngay đầu hàm khởi tạo camera.
- **Kết quả**: Luồng video MJPEG khôi phục ngay lập tức sau khi nạp code mà không cần rút nguồn.

### Mục 2 (Bản v4.0): Tốc độ AI chỉ đạt 2,7 FPS và bo mạch tự khởi động lại sau 6 giây (`task_wdt: IDLE0`)
- **Triệu chứng**: Serial in `giai ma 216 ms, suy luan 150 ms, AI 2.7 fps`, sau ~6 giây thì báo lỗi `task_wdt: Task watchdog got triggered. The following tasks did not reset the watchdog in time: IDLE0` và khởi động lại.
- **Nguyên nhân gốc**:
  1. Task AI trên Core 0 chạy vòng lặp tính toán liên tục không gọi `vTaskDelay(1)`, chiếm 100% CPU Core 0 khiến `IDLE0` không bao giờ được chạy để xóa Watchdog Timer.
  2. Giải mã JPEG bằng `tjpgd` trong ROM (`jpg2rgb565`) mất `216 ms` và suy luận mạng FOMO-Micro bằng C thuần mất `150 ms` chạy nối tiếp nhau trên cùng 1 luồng.
- **Đã sửa (trong v4.1)**:
  1. Thêm `vTaskDelay(1)` ở đầu mỗi vòng lặp của `det_task` trên Core 0.
  2. Tách giải mã JPEG sang `prep_task` trên Core 1 chạy song song với `det_task` trên Core 0 qua hàng đợi 3 bộ đệm.
  3. Thay `tjpgd` bằng `esp_new_jpeg` (SIMD, lấy thẳng kênh `Y` từ `CbYCrY`) và thay C thuần bằng `esp-nn` SIMD (`fomo_espnn.c`).
- **Kết quả**: Hết hoàn toàn lỗi Watchdog reset, tốc độ suy luận tăng vọt nhờ SIMD và đường ống 2 nhân.

### Mục 3 (Bản v4.1 $\to$ v4.2): Báo lỗi `esp_new_jpeg loi`, AI 0 FPS và trang web bị treo không vào được
- **Triệu chứng**: Khi khởi động bản v4.1, Serial báo `esp_new_jpeg loi -> dung tjpgd`, AI tụt về `0 fps` và trình duyệt không mở được trang điều khiển cổng 80.
- **Nguyên nhân gốc** (đã tái hiện bằng QEMU ESP32-S3):
  1. Khi bật cấu hình thu nhỏ (`cfg.scale`), hàm `jpeg_dec_parse_header()` trả về `info.width` và `info.height` là kích thước **sau khi thu nhỏ** (`tw × th`), nhưng code v4.1 lại đem so sánh với kích thước gốc `640×480` (`f.w × f.h`), khiến điều kiện kiểm tra luôn sai!
  2. Đặt bộ đệm `s_yuv` (`61 KB`) và `s_gray` (`27 KB`) trong Internal SRAM làm SRAM nội bộ chỉ còn `< 40 KB`, không đủ để cấp phát stack cho task AI và khởi tạo 2 HTTP Server (`httpd_start`).
- **Đã sửa (trong v4.2)**:
  1. Sửa điều kiện kiểm tra header thành `info.width == tw && info.height == th`.
  2. Chuyển `s_yuv` và `s_gray[0..2]` sang cấp phát trên **OPI PSRAM (`MALLOC_CAP_SPIRAM`)**, giải phóng `> 120 KB` Internal SRAM cho `esp-nn`, Wi-Fi và HTTP Server.
- **Kết quả**: `esp_new_jpeg` và cả 2 HTTP Server (cổng 80 và 81) chạy ổn định.

### Mục 4 (Bản v4.2 $\to$ v4.3): Luồng Stream mờ tịt, Giao diện Web UI khó nhìn và Khung bao vật thể quá nhỏ/không bám sát biên
- **Triệu chứng**:
  1. Hình ảnh video stream trên trình duyệt bị mờ nhòe ("mờ tịt"), thiếu chi tiết cạnh.
  2. Giao diện Web UI đẩy các thẻ thông số và thanh điều khiển xuống tuốt phía dưới màn hình; khung bao vẽ mờ nhạt và chỉ là một ô vuông nhỏ (`1/12` khung hình) nằm lơ lửng ở tâm đối tượng chứ không đóng khung quanh biên vật thể.
- **Nguyên nhân gốc**:
  1. *Về luồng Stream bị mờ*: (a) Thấu kính OV5640 có cuộn dây lấy nét VCM nhưng chưa được cấp dòng DAC (`0x3602/0x3603 = 0`, lấy nét ở vô cực nên cảnh trong phòng bị mất nét quang học); (b) Khối làm sắc nét cạnh phần cứng ISP CIP (`0x5308`, `0x5302`), `LENC` và `RAW_GMA` chưa được bật; (c) Độ phân giải mặc định `VGA 640×480` với mức nén `jpeg_quality = 12` bị kéo giãn lên `960 px`; (d) Chế độ ngủ xen kẽ `eco_enabled = true` và `g_stats.hot` ngắt camera ngay cả khi đang xem stream và chỉ bỏ qua 3 khung khi thức dậy (chưa đủ để AE/AWB hội tụ); (e) Hàm `scaled_dim(800, 160)` tính tỷ lệ `1/5` không hợp lệ cho `esp_new_jpeg` khi chuyển lên SVGA.
  2. *Về độ mịn nhận dạng & đóng khung*: Mạng FOMO cắt ở `stride = 8` chỉ xuất lưới `12×12` (`144` ô thô, mỗi ô rộng tới `8,33%` khung hình) và được huấn luyện với nhãn tâm (`center`), nên chỉ có 1–2 ô ở tâm đối tượng bật lên, không phản ánh kích thước và biên thật của vật thể.
  3. *Về Web UI*: Bố cục 1 cột dọc làm tràn chiều cao màn hình; Canvas không nhân `devicePixelRatio` và vẽ hộp đối xứng quanh `(cx, cy)` với nét mảnh `2px` không có viền tương phản.
- **Đã sửa (trong v4.3)**:
  1. *Làm nét luồng Stream*: Bổ sung điều khiển DAC VCM 10-bit (`cam_set_vcm_focus`, mặc định `140`) + thuật toán quét lấy nét tự động `cam_auto_focusSweep()`; bật `LENC`, `RAW_GMA`, `BPC/WPC`, `DCW` và làm sắc nét cạnh phần cứng CIP (`sharpness = 3`); nâng mặc định lên `SVGA (800×600)` với `jpeg_quality = 8` và dòng lái DVP `2x` (`0x302C = 0x42`); khóa không cho camera ngủ khi `stream_clients > 0`; sửa `choose_idct_scale()` chọn đúng tỷ lệ `1/8`, `1/4`, `1/2` cho `esp_new_jpeg`.
  2. *Làm mịn lưới nhận dạng gấp 16 lần (`48×48 = 2.304` ô)*: Xây dựng hàm `fomo_decode_fine()` nội suy bản đồ nhiệt `12×12` lên lưới siêu mịn `48×48` (`2.304` ô trên cùng diện tích `96×96`), kết hợp **Lan truyền vùng song phương (Bilateral Silhouette Region Growing)** và **Khóa biên Gradient Sobel** để tìm chính xác 4 cạnh `[x0, y0, x1, y1]` ôm sát biên vật thể với độ phân giải `1/48` (`2,08%`).
  3. *Thiết kế lại Web UI Dashboard 2 cột*: Hiển thị song song khung video và bảng điều khiển trên cùng 1 màn hình; nâng cấp Canvas HiDPI vẽ khung bao tương phản kép (viền đen + viền neon `#00e676` + 4 góc ngắm chữ L + lưới mịn `24×24`/`48×48` bám theo hình dáng vật thể).
- **Kết quả**: Luồng stream sắc nét, tốc độ cao, khung bao bám sát đường biên thực tế của vật thể và giao diện Web UI rất dễ quan sát, điều chỉnh.

---

## 5. VẤN ĐỀ CÒN TỒN ĐỌNG & VIỆC TIẾP THEO CẦN LÀM (ƯU TIÊN TRƯỚC DEMO)

| STT | Công việc cần thực hiện | Thời điểm | Ghi chú kỹ thuật |
|---|---|---|---|
| **1** | **Nạp firmware v4.3 lên ESP32-S3, ghi lại log Serial thực tế** (`cam fps`, `stream fps`, `AI fps`, thời gian giải mã `esp_new_jpeg` và suy luận `esp-nn` + `Fine-Grid 48×48`) | Trước buổi Demo | Kiểm tra dòng `AI tren CPU: esp-nn` và `(esp_new_jpeg)` trên Serial Monitor ở cả `SVGA` và `VGA` |
| **2** | **Tổng hợp project `fomo_top_tn20k` trên Gowin EDA** để lấy bảng Timing Analysis ở `27 MHz` và bảng Resource Utilization thực tế | Trước buổi Demo | Điền số liệu thực tế từ Gowin vào mục 6.3 của `README.md` (Báo cáo kỹ thuật) |
| **3** | **Nối dây SPI giữa ESP32-S3 và Tang Nano 20K**, chạy backend `Tang Nano 20K` và đối chiếu kết quả với backend `ESP32-S3 CPU` (`CPU_USE_ESPNN 0` phải khớp 100%; bản `esp-nn` được phép lệch `1–2 LSB`) | Trước buổi Demo | Nếu dữ liệu SPI bị nhiễu trên dây cắm, hạ `FPGA_SPI_HZ` trong `app_config.h` xuống `10 MHz` |
| **4** | **Đo nhiệt độ bề mặt cảm biến OV5640 và dòng tiêu thụ qua INA219** theo quy trình 10 phút/cấu hình (mục 7.5 trong `README.md`) | Trước báo cáo cuối | Hoàn thiện bảng thực nghiệm nhiệt và năng lượng |
| **5** | **Thu thập ảnh cảnh thật qua nút `📷 Chụp ảnh (.jpg)`**, chạy `auto_label.py` và tinh chỉnh lại `FOMO-Micro` | Sau Demo / Mở rộng | Cải thiện mạnh nhất độ chính xác F1 và giảm báo động giả cho góc lắp đặt cố định |
| **6** | **Đo công suất tiêu thụ của FPGA theo Mode 0 / Mode 1 / Mode 2** (bằng Gowin Power Analyzer hoặc INA219) | Trước báo cáo cuối | Cung cấp số liệu thực nghiệm chứng minh lợi thế tiết kiệm năng lượng của bộ nhân AdaReMAC |

---

## 6. QUY ƯỚC LÀM VIỆC

1. **Ngôn ngữ & Phong cách giao tiếp**:
   - Trả lời hoàn toàn bằng **tiếng Việt**, ngắn gọn, đi thẳng vào nguyên nhân kỹ thuật và kết quả thực thi, không dùng lời lẽ sáo rỗng hay tự khen.
   - Luôn phân biệt rõ ràng giữa **số liệu đã đo/kiểm chứng bằng trình biên dịch & mô phỏng** với **số liệu ước tính cần đo trên mạch thật**. Tuyệt đối không bịa đặt số liệu hay dùng dữ liệu tổng hợp giả (`rng.uniform`) để báo cáo độ chính xác ảo.
2. **Phong cách viết Code & Thiết kế Hệ thống**:
   - **Ưu tiên sự đơn giản, gọn sạch và phẫu thuật chính xác (*Surgical & Minimalist*)**: Chỉ sửa đúng những phần giải quyết trực tiếp yêu cầu; không viết các lớp trừu tượng thừa thãi.
   - **Tự chứa (*Self-Contained*) & Biên dịch tức thì**: Code Arduino cho ESP32-S3 và Verilog RTL cho Tang Nano 20K phải biên dịch được ngay mà không bắt người dùng cài đặt chuỗi công cụ phức tạp hay sửa đường dẫn thủ công.
   - **Bảo toàn tính tương thích Bit-Exact**: Mọi thay đổi ở tầng tiền xử lý hoặc hậu xử lý (như `fomo_decode_fine` trên lưới `48×48`) phải giữ nguyên 100% tính khớp bit (`0/144` logit lệch) của nhân suy luận `fomo_micro_run` giữa Python, C và RTL FPGA.
   - **Khi xuất bản bàn giao (`hand_over.md`)**: Tuyệt đối không viết tắt hay dùng `"...phần còn lại giữ nguyên"`. Không in nội dung code vào hand_over, chỉ phân tích vai trò file code (đã có trong mục 2).
