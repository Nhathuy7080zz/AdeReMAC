# Dự án AdaReMAC (Bản v3 — Chuẩn Arduino IDE 2.x, Stream FHD@30p Chống Nghẽn + AI Học Liên Tục & Chống Nóng Chip)

Tài liệu này hướng dẫn toàn diện từ bước **dọn sạch ESP-IDF cũ**, **cài đặt Arduino IDE + Board Manager**, **chạy Code Test Stream FHD@30p siêu nét theo phương án Pure Wi-Fi AP 40MHz HT40 chống nghẽn + AI học môi trường liên tục & đóng khung vật thể bất thường**, **nạp FPGA Tang Nano 20K mát lạnh 100%**, đến **đấu dây trên 1 hàng chân phải duy nhất cho Breadboard**.

## 1. Cấu trúc thư mục trong gói v3

```text
adaremac/
├── README.md                          # Hướng dẫn toàn diện từ A–Z (file này)
├── uninstall_esp_idf.ps1              # Script tự động gỡ sạch ESP-IDF, Cache, PATH & Registry
├── firmware_arduino/                  # Toàn bộ mã nguồn cho Arduino IDE 2.x (Không cần cài thư viện ngoài)
│   ├── 1_test_camera_ov5640/
│   │   ├── 1_test_camera_ov5640.ino   # Sketch 1: Stream FHD@30p Pure AP 40MHz + ISP chống mờ + AI đóng khung bất thường
│   │   └── ai_model_data.h            # Trọng số mạng nơ-ron INT8 AdaReMAC tích hợp sẵn cho Sketch 1
│   └── 2_adaremac_main/
│       ├── 2_adaremac_main.ino        # Sketch 2: Code chính ghép nối SPI 1 hàng phải với Tang Nano 20K + INA219 + AI 3 Mode
│       └── ai_model_data.h            # Trọng số mạng CNN INT8 và mẫu kiểm thử Mode 1 (25%) / Mode 2 (100%)
├── rtl/                               # Mã nguồn Verilog cho Tang Nano 20K (Đã sửa chống nóng chip)
│   ├── adaremac_mul.v                 # Bộ nhân xấp xỉ 8x8 không dấu (qL = 2, k = 3, R = 2*D2 - D0)
│   ├── adaremac_mac_lane.v            # Lane MAC INT8 có dấu 3 tầng pipeline + bộ giám sát sai số phần cứng
│   ├── mac_array.v                    # Mảng MAC điều khiển bằng tín hiệu run
│   ├── spi_slave.v                    # Giao tiếp SPI Slave (Mode 0, khung 16-bit)
│   ├── adaremac_core.v                # Đặt mặc định N_LANES = 1 và mode = 0 khi khởi động
│   ├── top_tn20k.v                    # Bỏ chân wake_esp thừa, đưa spi_miso về High-Z (1'bz) khi CS_N = 1
│   └── top_standalone.v, report_tx.v, uart_tx.v, btn_sync.v
├── constraints/                       # File gán chân vật lý (.cst) và ràng buộc xung nhịp (.sdc) cho Gowin EDA
│   ├── tn20k.cst                      # Bỏ chân 50 không tồn tại, đặt PULL_MODE=NONE và BANK_VCCIO=3.3
│   ├── tn20k.sdc                      # Ràng buộc clock 27 MHz
│   ├── tn20k_standalone.cst           # Đổi btn_rst_n -> btn_rst, PULL_MODE=NONE, BANK_VCCIO=3.3
│   └── tn20k_standalone.sdc
├── standalone_tangnano_only/          # Chỉ chứa đúng 7 file .v tổng hợp được (đã tách tb_standalone.v sang sim/)
├── sim/                               # Testbench kiểm chứng RTL bằng Icarus Verilog (tb_mul.v, tb_core.v, tb_standalone.v)
└── ai_eval/                           # Huấn luyện CNN, lượng tử hóa INT8, kiểm chứng trùng bit RTL & xuất C header
```

## 2. Quy trình cài đặt và chạy từng bước từ A–Z (Với Arduino IDE 2.x)

### Bước 1: Cài đặt Arduino IDE 2.x và gói Board ESP32

1. Tải và cài đặt **Arduino IDE 2.x** từ `https://www.arduino.cc/en/software`.
2. Vào **File** $\to$ **Preferences...** (`Ctrl + ,`) $\to$ tại ô **Additional boards manager URLs**, dán:
   ```text
   https://espressif.github.io/arduino-esp32/package_esp32_index.json
   ```
3. Mở **Boards Manager** (`Ctrl + Shift + B`) $\to$ tìm **`esp32 by Espressif Systems`** và nhấn **Install**.

### Bước 2: Thiết lập menu `Tools` trong Arduino IDE cho Kit ESP32-S3 N16R8 OV5640 TypeC

| Mục trong menu `Tools` | Giá trị bắt buộc chọn | Lưu ý quan trọng |
| :--- | :--- | :--- |
| **Board** | `ESP32S3 Dev Module` | Board chuẩn cho ESP32-S3 WROOM-1 |
| **Port** | `COMx` | Cổng COM của mạch ESP32-S3 |
| **USB CDC On Boot** | `Enabled` (cổng USB) / `Disabled` (cổng UART) | Nếu mở Serial Monitor không thấy chữ, chỉ cần đổi trạng thái mục này |
| **Flash Size** | **`16MB (128Mb)`** | Khớp với bộ nhớ Flash 16MB (`N16`) |
| **Partition Scheme** | **`Huge APP (3MB No OTA/1MB SPIFFS)`** | Đủ chỗ cho Web Server Stream và mô hình AI |
| **PSRAM** | **`OPI PSRAM`** | **Bắt buộc chọn `OPI PSRAM`** (khớp với 8MB Octal PSRAM `R8`) |

### Bước 3: Nạp Sketch 1 (`1_test_camera_ov5640.ino`) — Stream FHD@30p Siêu Nét + AI Học Môi Trường & Đóng Khung Bất Thường

#### 3.1. Tại sao phương án chính ESP32-S3 tự phát Wi-Fi (`Pure WIFI_AP 40MHz HT40`) đỡ nghẽn nhất?

* **Truyền thẳng 1 chặng (Zero-Hop Direct Link)**: Khi ESP32-S3 tự phát Wi-Fi (`WIFI_AP`), mỗi khung ảnh FHD đi **thẳng 1 chặng** từ anten ESP32-S3 sang máy tính/điện thoại của bạn. Ngược lại, nếu đi qua Router Wi-Fi ngoài (`WIFI_STA`), mỗi khung ảnh phải bay **2 chặng** trên cùng kênh tần số (ESP32-S3 $\to$ Router $\to$ Máy tính), làm tốn gấp đôi thời gian chiếm sóng (Airtime) và dễ nghẽn bởi các thiết bị khác trong nhà.
* **Không chia đôi thời gian Anten**: Code đã cấu hình thuần **`WIFI_AP`** (tắt hẳn chế độ kép `WIFI_AP_STA`), bật băng thông kênh rộng **40 MHz (`WIFI_BW_HT40` — 150 Mbps)**, công suất phát cực đại **`19.5 dBm`**, tắt hoàn toàn ngủ tiết kiệm điện (`WIFI_PS_NONE`) và bật cờ **`TCP_NODELAY`** trên socket truyền video.

#### 3.2. Cách nạp và mở giao diện Stream FHD@30p + AI

1. **Mở Sketch 1**: Trong Arduino IDE, vào **File** $\to$ **Open...** và mở:
   `adaremac/firmware_arduino/1_test_camera_ov5640/1_test_camera_ov5640.ino`
   *(File `ai_model_data.h` nằm cùng thư mục sẽ tự động mở kèm làm tab thứ 2).*
2. **Nhấn nút Upload (`→`)** (không cần sửa bất kỳ dòng code nào).
3. **Kết nối Wi-Fi và xem Stream**:
   * Trên máy tính hoặc điện thoại, kết nối vào mạng Wi-Fi **`ESP32-OV5640-FHD`** (mật khẩu: **`12345678`**).
   * Mở trình duyệt Web truy cập địa chỉ: **`http://192.168.4.1`**
   * *(Lưu ý: Nếu bạn muốn đổi sang nối qua Router Wi-Fi nhà bạn, chỉ cần đổi `#define USE_DIRECT_AP_MODE 0` ở dòng 36 và điền `WIFI_SSID` / `WIFI_PASS`).*
4. **Cách chỉnh nét căng cho ống kính OV5640**:
   * Trong code đã bật sẵn bộ xử lý ảnh phần cứng ISP của OV5640.
   * Nếu thấu kính OV5640 của bạn đang lấy nét ở khoảng cách xa/gần chưa khớp với vật thể trước mặt, bạn chỉ cần **vặn nhẹ vành ren nhựa đen ở đầu ống kính OV5640** khoảng $15^\circ - 30^\circ$ trong lúc nhìn màn hình Live Stream để lấy nét tiêu cự chuẩn xác 100%.

### Bước 4: Nạp mạch FPGA Sipeed Tang Nano 20K (Đảm bảo mát lạnh 100%)

1. **Mẹo Reset cứng nếu mạch Tang Nano 20K còn lưu bitstream lỗi cũ gây nóng**: Rút cáp USB chờ nguội $\to$ **nhấn giữ chặt nút S1 (hoặc S2) trong lúc cắm cáp USB vào máy tính** (giữ 2–3 giây) $\to$ nạp đè bằng Gowin Programmer.
2. **Bốn quy tắc bắt buộc trên Gowin EDA**:
   * Chọn đúng chip **`GW2AR-LV18QN88C8/I7`** với **`Version = C`** (**`GW2AR-18C`**).
   * Trong **Project $\to$ Configuration $\to$ Place & Route $\to$ Unused Pin**: Tích chọn **`As input tri-stated`** (bỏ tích pull-up).
   * Trong **Project $\to$ Configuration $\to$ Place & Route $\to$ Dual-Purpose Pin**: Tích chọn đủ 4 ô (`DONE`, `READY`, `RECONFIG_N`, `SSPI / MODE as regular IO`).
   * Dùng bộ file trong `rtl/` + `constraints/tn20k.cst` & `tn20k.sdc` (đặt `top_tn20k` làm Top Module).

### Bước 5: Cắm dây trên 1 Hàng Bên Phải (Breadboard) & Nạp Sketch 2 (`2_adaremac_main.ino`)

Khi cắm mạch ESP32-S3 lên Breadboard, **bỏ trống hoàn toàn Hàng Trái** (chứa 14 chân Camera OV5640) và chỉ cắm dây ở **Hàng Phải** theo thứ tự từ trên xuống:

| Thứ tự chân (từ trên xuống ở Hàng Phải) | Ký hiệu in trên mạch ESP32-S3 | Đấu nối tới đâu? | Chức năng |
| :--- | :--- | :--- | :--- |
| Chân số 1, 2 | `TX` (`43`), `RX` (`44`) | *(Để trống)* | Cổng nạp / Serial |
| **Chân số 3** | **`1` (`GPIO 1`)** | Tang Nano 20K — **Pin 42** | `SPI_CS` (Cụm 4 dây SPI nằm sát liền nhau) |
| **Chân số 4** | **`2` (`GPIO 2`)** | Tang Nano 20K — **Pin 48** | `SPI_MOSI` |
| **Chân số 5** | **`42` (`GPIO 42`)** | Tang Nano 20K — **Pin 41** | `SPI_SCLK` |
| **Chân số 6** | **`41` (`GPIO 41`)** | Tang Nano 20K — **Pin 49** | `SPI_MISO` |
| Chân số 7, 8 | `40`, `39` | *(Để trống)* | Dự phòng |
| **Chân số 9** | **`38` (`GPIO 38`)** | Module INA219 — **`VCC`** | Xuất **3.3V (HIGH)** nuôi INA219 ngay bên phải |
| Chân số 10..15 | `37, 36, 35, 0, 45, 48` | *(Tuyệt đối để trống)* | Chân bộ nhớ OPI PSRAM (`35..37`), Boot (`0`) và LED RGB (`48`) |
| **Chân số 16** | **`47` (`GPIO 47`)** | Module INA219 — **`SDA`** | `I2C_SDA` (Cụm 2 dây I2C nằm sát liền nhau) |
| **Chân số 17** | **`21` (`GPIO 21`)** | Module INA219 — **`SCL`** | `I2C_SCL` |
| Chân số 18, 19 | `20`, `19` | *(Để trống)* | Chân USB Type-C |
| **Chân số 20 (Cuối cùng)** | **`G` (`GND`)** | **`GND`** của Tang Nano 20K & **`GND`** của INA219 | **Mass (GND) chung cả 3 mạch** |
