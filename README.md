# Dự án AdaReMAC (Bản v3 — Chuẩn Arduino IDE 2.x, Dồn 1 Hàng Chân Phải & Chống Nóng Chip)

Tài liệu này hướng dẫn toàn diện từ bước **dọn sạch ESP-IDF cũ**, **cài đặt Arduino IDE + Board Manager**, **nạp FPGA Tang Nano 20K mát lạnh 100%**, **đấu dây trên 1 hàng chân phải duy nhất cho Breadboard**, đến bước **nạp code Test Camera OV5640 và code Chính vận hành 3 Chế độ AI**.

## 1. Cấu trúc thư mục trong gói v3 này

```text
adaremac/
├── README.md                          # Hướng dẫn toàn diện từ A–Z (file này)
├── uninstall_esp_idf.ps1              # [MỚI] Script tự động gỡ sạch ESP-IDF, Cache, PATH & Registry
├── firmware_arduino/                  # [MỚI] Toàn bộ mã nguồn cho Arduino IDE 2.x (Không cần cài thư viện ngoài)
│   ├── 1_test_camera_ov5640/
│   │   └── 1_test_camera_ov5640.ino   # Sketch 1: Test cô lập Camera OV5640 + PSRAM + Đo nhiệt độ chip + Web Stream
│   └── 2_adaremac_main/
│       ├── 2_adaremac_main.ino        # Sketch 2: Code chính (SPI 1 hàng phải + INA219 + Camera OV5640 + AI 3 Mode)
│       └── ai_model_data.h            # Trọng số mạng CNN INT8 và mẫu kiểm thử Mode 1 (25%) / Mode 2 (100%)
├── rtl/                               # Mã nguồn Verilog cho Tang Nano 20K (Đã sửa chống nóng chip)
│   ├── adaremac_mul.v                 # Bộ nhân xấp xỉ 8x8 không dấu (qL = 2, k = 3, R = 2*D2 - D0)
│   ├── adaremac_mac_lane.v            # Lane MAC INT8 có dấu 3 tầng pipeline + bộ giám sát sai số phần cứng
│   ├── mac_array.v                    # Mảng MAC điều khiển bằng tín hiệu run
│   ├── spi_slave.v                    # Giao tiếp SPI Slave (Mode 0, khung 16-bit)
│   ├── adaremac_core.v                # [ĐÃ SỬA] Đặt mặc định N_LANES = 1 và mode = 0 khi khởi động
│   ├── top_tn20k.v                    # [ĐÃ SỬA] Bỏ chân wake_esp thừa, đưa spi_miso về High-Z (1'bz) khi CS_N = 1
│   └── top_standalone.v, report_tx.v, uart_tx.v, btn_sync.v
├── constraints/                       # File gán chân vật lý (.cst) và ràng buộc xung nhịp (.sdc) cho Gowin EDA
│   ├── tn20k.cst                      # [ĐÃ SỬA] Bỏ chân 50 không tồn tại, đặt PULL_MODE=NONE và BANK_VCCIO=3.3
│   ├── tn20k.sdc                      # Ràng buộc clock 27 MHz
│   ├── tn20k_standalone.cst           # [ĐÃ SỬA] Đổi btn_rst_n -> btn_rst, PULL_MODE=NONE, BANK_VCCIO=3.3
│   └── tn20k_standalone.sdc
├── standalone_tangnano_only/          # [ĐÃ SỬA] Chỉ chứa đúng 7 file .v tổng hợp được (đã bỏ tb_standalone.v ra)
├── sim/                               # Testbench kiểm chứng RTL bằng Icarus Verilog (tb_mul.v, tb_core.v, tb_standalone.v)
├── ai_eval/                           # Huấn luyện CNN, lượng tử hóa INT8, kiểm chứng trùng bit RTL & xuất C header
└── docs/                              # Báo cáo kỹ thuật PDF/DOCX và hướng dẫn đánh giá AI
```

## 2. Các lỗi phần cứng & code đã được sửa triệt để trong bản v3 này (Đảm bảo không nóng chip)

1. **Sửa lỗi nóng chip FPGA ở cả bản `standalone` và bản `rtl` (`top_tn20k.v`)**:
   * Hạ tham số `N_LANES` từ `16` xuống **`1`** trong cả `top_standalone.v`, `top_tn20k.v` và `adaremac_core.v` (giảm 95% số LUT/Flip-Flop đảo trạng thái).
   * Xóa bỏ cổng ngõ ra `output wire wake_esp` bị bỏ trống trong `top_tn20k.v` (tránh việc Gowin EDA tự gán ngẫu nhiên ngõ ra này vào chân SDRAM nội bộ hoặc IC ngoại vi gây chập nguồn 3.3V xuống GND).
   * Đưa chân `spi_miso` (Pin 49) về trạng thái **trở kháng cao (`1'bz` / High-Z)** khi `spi_cs_n = 1` (`assign spi_miso = spi_cs_n ? 1'bz : spi_miso_core;`) để không bao giờ xung đột áp với ESP32-S3 hoặc mạch đèn nền LCD trên Tang Nano 20K.
2. **Sửa lỗi biên dịch trên Gowin EDA**:
   * Đã dời file testbench `tb_standalone.v` ra khỏi thư mục `standalone_tangnano_only/` (chỉ để trong `sim/`) để bạn có thể chọn toàn bộ file `.v` trong `standalone_tangnano_only/` mà không bị lỗi `EX3828: dut.rst_n remains unresolved`.
   * Đã đổi `btn_rst_n` thành `btn_rst` trong `tn20k_standalone.cst` (hết lỗi `CT1135`) và bỏ chân `Pin 50` không tồn tại trên gói QN88 trong `tn20k.cst` (hết lỗi `CT1113`).
   * Đổi `PULL_MODE=NONE` trên các chân nút bấm và chân SPI để không gây dòng rò qua điện trở kéo nội bộ.
3. **Dồn 100% chân kết nối của ESP32-S3 sang duy nhất Hàng Phải (Right Row)**:
   * Toàn bộ 14 chân Camera OV5640 nằm bên Hàng Trái, còn toàn bộ các dây nối ra ngoài (`SPI`, `I2C`, `VCC 3.3V cho INA219`, `GND`) đều nằm ở **Hàng Phải** để khi cắm lên Breadboard chỉ cần cắm dây ở 1 bên.

## 3. Quy trình cài đặt và chạy từng bước từ A–Z (Với Arduino IDE 2.x)

### Bước 1: Cài đặt Arduino IDE 2.x và gói Board ESP32

1. Tải và cài đặt **Arduino IDE 2.x** từ trang chủ `https://www.arduino.cc/en/software`.
2. Mở Arduino IDE $\to$ vào **File $\to$ Preferences...** (`Ctrl + ,`) $\to$ tại ô **Additional boards manager URLs**, dán dòng sau rồi nhấn **OK**:
   ```text
   https://espressif.github.io/arduino-esp32/package_esp32_index.json
   ```
3. Mở biểu tượng **Boards Manager** ở thanh bên trái (`Ctrl + Shift + B`) $\to$ gõ **`esp32`** $\to$ tìm **`esp32 by Espressif Systems`** và nhấn **Install**.
   *(Gói này đã tích hợp sẵn thư viện Camera `"esp_camera.h"`, `<SPI.h>`, `<Wire.h>` và cảm biến nhiệt độ `temperatureRead()` — bạn **không cần cài thêm bất kỳ thư viện nào khác**).*

### Bước 2: Thiết lập menu `Tools` trong Arduino IDE cho Kit ESP32-S3 N16R8 OV5640 TypeC

Trước khi nạp bất kỳ code nào cho mạch **ESP32-S3 WROOM-1 N16R8**, hãy vào menu **Tools** và chọn đúng **6 thông số bắt buộc** sau (chọn **Flash Size** trước rồi mới chọn **Partition Scheme**):

| Mục trong menu `Tools` | Giá trị bắt buộc chọn | Lưu ý quan trọng |
| :--- | :--- | :--- |
| **Board** | `ESP32S3 Dev Module` | Board chuẩn cho ESP32-S3 WROOM-1 |
| **Port** | `COMx` | Cổng COM của mạch ESP32-S3 |
| **USB CDC On Boot** | `Enabled` (cổng USB) / `Disabled` (cổng UART) | Nếu mở Serial Monitor không thấy chữ, chỉ cần đổi trạng thái mục này |
| **Flash Size** | **`16MB (128Mb)`** | Khớp với bộ nhớ Flash 16MB (`N16`) |
| **Partition Scheme** | **`Huge APP (3MB No OTA/1MB SPIFFS)`** | Đủ chỗ cho Camera và mô hình AI |
| **PSRAM** | **`OPI PSRAM`** | **Bắt buộc chọn `OPI PSRAM`** (khớp với 8MB Octal PSRAM `R8`; chọn sai Camera sẽ báo lỗi!) |

### Bước 3: Nạp Sketch 1 để test riêng Camera OV5640 và Nhiệt độ ESP32-S3

1. **Rút cáp USB** khỏi mạch ESP32-S3. Mở lẫy đen của khe cắm FPC 24-pin trên mạch, cắm cáp dẹt của **Camera OV5640** sao cho **mặt có lá đồng vàng úp xuống sát mặt bo mạch PCB** (mặt nhựa màu xanh dương hướng lên trên), rồi ấn nhẹ lẫy đen xuống khóa chặt. **Chưa cắm bất kỳ sợi dây nào sang FPGA.**
2. Cắm cáp USB Type-C nối ESP32-S3 vào máy tính.
3. Trong Arduino IDE, vào **File $\to$ Open...** và mở file:
   `adaremac/firmware_arduino/1_test_camera_ov5640/1_test_camera_ov5640.ino`
4. Nhấn nút **Upload (`→`)** ở góc trên bên trái.
5. Khi báo `Done uploading`, mở **Serial Monitor (`🔍`)** ở góc trên bên phải, chọn tốc độ **`115200 baud`** và nhấn nút **RST** trên mạch ESP32-S3. Màn hình sẽ in ra:
   ```text
   [1/4] Nhiet do noi bo ESP32-S3 : 42.5 C [BINH THUONG]
   [2/4] Bo nho OPI PSRAM         : 8188 KB trong / 8192 KB tong
   [3/4] Dang khoi tao Camera OV5640 (XCLK=GPIO15, SIOD=GPIO4, SIOC=GPIO5)...
   [4/4] Nhan dien thanh cong     : OV5640 (5MP - max 2592x1944 QSXGA) (PID = 0x5640)
   [Frame #0001] OK | 320x240 | 153600 bytes | Capture: 18.20 ms | Do sang TB: 118/255 | Temp: 43.1 C
   ```
   *(Mẹo: Nếu bạn điền tên Wi-Fi vào `WIFI_SSID` và `WIFI_PASS` ở đầu file `1_test_camera_ov5640.ino`, bạn có thể mở địa chỉ `http://<IP>/` trên trình duyệt để xem ảnh chụp trực tiếp từ Camera OV5640).*

### Bước 4: Nạp mạch FPGA Sipeed Tang Nano 20K (Đảm bảo mát lạnh 100%)

#### 4.1. Mẹo Reset cứng nếu mạch Tang Nano 20K còn lưu bitstream lỗi cũ gây nóng
Nếu vừa cắm cáp USB vào Tang Nano 20K mà chip nóng do bitstream cũ trong Flash:
* Rút cáp USB ra chờ nguội $\to$ **dùng ngón tay nhấn giữ chặt nút S1 (hoặc S2) trong lúc cắm cáp USB vào máy tính** (giữ 2–3 giây rồi thả ra) để ngăn FPGA tự boot từ Flash $\to$ dùng Gowin Programmer nạp đè code mới vào.

#### 4.2. Bốn quy tắc bắt buộc khi tạo Project trên Gowin EDA (Cho cả bản `standalone` và bản `rtl`)
1. **Chọn đúng phiên bản chip `Device Version: C`**:
   * Khi tạo Project hoặc khi nhấp đúp vào tên chip `GW2AR-LV18QN88C8/I7` ở cửa sổ **Design**, bắt buộc chọn dòng có **`Version = C`** (**`GW2AR-18C`**), tuyệt đối không chọn dòng `na` (`GW2AR-18`).
2. **Thả nổi toàn bộ các chân không dùng (`Unused Pin`)**:
   * Vào menu **Project $\to$ Configuration $\to$ Place & Route $\to$ Unused Pin**:
   * Tích chọn **`As input tri-stated`** (và **bỏ tích** ô `As input tri-stated with pull-up`).
3. **Bật `Dual-Purpose Pin`**:
   * Vào menu **Project $\to$ Configuration $\to$ Place & Route $\to$ Dual-Purpose Pin**:
   * Tích chọn đủ 4 ô: **Use DONE as regular IO**, **Use READY as regular IO**, **Use RECONFIG_N as regular IO**, **Use SSPI as regular IO**.
4. **Thêm đúng các file của bản SPI (`rtl/`)**:
   * Để chạy độc lập qua UART: thêm các file trong `standalone_tangnano_only/` (`top_standalone` làm Top Module).
   * Để ghép nối SPI với ESP32-S3: thêm **6 file `.v`** trong `rtl/` (`adaremac_mul.v`, `adaremac_mac_lane.v`, `mac_array.v`, `spi_slave.v`, `adaremac_core.v`, `top_tn20k.v` — đặt `top_tn20k` làm Top Module) + **2 file ràng buộc** trong `constraints/` (`tn20k.cst`, `tn20k.sdc`).
   * Chạy **Synthesize** $\to$ **Place & Route** $\to$ mở **PnR Report** xác nhận `Register` chỉ dùng **`~190 FF (1%)`** và `I/O Port` đúng bằng **`11/66`**.
   * Mở **Gowin Programmer**, nạp vào SRAM (`SRAM Program`) hoặc Flash ngoài (`External Flash Mode` $\to$ `exFlash Erase, Program thru GAO-Bridge` $\to$ `Generic Flash` ở tần số `<= 2.5MHz`). Đèn LED thứ 4 (`Pin 18`) sẽ chớp đều đặn mỗi ~0.6 giây và chip hoàn toàn mát.

### Bước 5: Cắm dây trên 1 Hàng Bên Phải (Breadboard) & Nạp Sketch 2 (`2_adaremac_main.ino`)

#### 5.1. Sơ đồ cắm dây 100% trên Hàng Phải của ESP32-S3
Khi cắm mạch ESP32-S3 lên Breadboard, hãy đặt sát **Hàng Trái** vào mép không còn lỗ của Breadboard (không cắm bất kỳ dây nào vào Hàng Trái). Ở phía **Hàng Phải** còn trống lỗ, bạn cắm dây theo đúng thứ tự từ trên xuống dưới:

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
| **Chân số 20 (Cuối cùng)** | **`GND`** | **`GND`** của Tang Nano 20K & **`GND`** của INA219 | **Mass (GND) chung cả 3 mạch** |

#### 5.2. Nạp Sketch 2 (`2_adaremac_main.ino`) trên Arduino IDE
1. Trong Arduino IDE, vào **File $\to$ Open...** và mở file:
   `adaremac/firmware_arduino/2_adaremac_main/2_adaremac_main.ino`
   *(File `ai_model_data.h` đã nằm sẵn cùng thư mục nên Arduino IDE sẽ tự động mở kèm làm tab thứ 2).*
2. Nhấn nút **Upload (`→`)**.
3. Mở **Serial Monitor (`115200 baud`)**:
   * Mạch sẽ tự động kiểm tra kết nối SPI với Tang Nano 20K (`ID = 0xA5`), xuất bảng đo `CSV,config,mode,thr,...` qua 6 cấu hình vi kiến trúc.
   * Tiếp theo, hệ thống bước vào vòng lặp thích nghi **3 Chế độ**:
     * **Mode 0 (Thuần xấp xỉ)**: FPGA chạy `mode=0`, ESP32-S3 tự ngắt CPU (`Light Sleep` 300 ms), lượng dữ liệu qua model = `0%`.
     * **Mode 1 (Xấp xỉ một phần)**: FPGA chạy `mode=1, thr=2`, ESP32-S3 chỉ cắt vùng trung tâm **25% khung hình (ROI 160×120)** từ Camera OV5640 và đưa **25% lượng dữ liệu đặc trưng (100/400 byte)** qua mô hình phân loại nhẹ.
     * **Mode 2 (Chính xác 100%)**: Khi phát hiện thay đổi độ sáng/chuyển động trong vùng ROI (hoặc trong chu kỳ kiểm chứng), FPGA chuyển sang `mode=2` (Exact) và ESP32-S3 đưa **100% khung hình (320×240, 400/400 byte)** qua toàn bộ mô hình, đồng thời in thời gian xử lý, công suất INA219 và nhiệt độ chip theo thời gian thực.
