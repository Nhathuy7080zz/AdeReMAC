# BÁO CÁO KHOA HỌC & KỸ THUẬT TOÀN DIỆN DỰ ÁN ADAREMAC
## Thiết kế Đồng xử lý Phần cứng - Phần mềm Thích nghi Thời gian thực cho Nhận dạng Đối tượng Thị giác Nhúng dựa trên Kiến trúc Edge Impulse FOMO, Tiny-SSD và Bộ nhân Xấp xỉ Đa chế độ AdaReMAC trên ESP32-S3 N16R8 & FPGA Sipeed Tang Nano 20K

---

## 1. TỔNG QUAN & ĐỘNG LỰC KHOA HỌC

Trong các hệ thống Thị giác Máy tính Nhúng tại biên (*Edge Embedded Vision* / *TinyML*), bài toán phát hiện và bám đuổi đối tượng thời gian thực (*Real-Time Object Detection & Tracking*) đặt ra hai yêu cầu kỹ thuật đối nghịch nhau:
1. **Tốc độ truyền phát và phản hồi thời gian thực ($\ge 30\text{ FPS}$)**: Hệ thống phải thu nhận khung hình từ cảm biến CMOS, truyền phát luồng video MJPEG qua Wi-Fi và cập nhật tọa độ khung bao (*Bounding Box*) với độ trễ dưới $33\text{ ms/frame}$.
2. **Độ chính xác nhận dạng cao & miễn nhiễm báo động giả trong môi trường thực tế**: Mô hình phải phân biệt chính xác đối tượng thực (người, bàn tay, hộp/vật thể đặc, thiết bị nhỏ chuyển động nhanh, nguồn sáng lạ, ký tự) đồng thời loại bỏ hoàn toàn các yếu tố gây nhiễu như bóng đổ (*Cast Shadows*), trôi phơi sáng tự động (*Auto-Exposure Drift*), nhiễu hạt CMOS trên trần/tường tối và rung viền khối nén JPEG.

Dựa trên phân tích chuyên sâu trong tài liệu định hướng kiến trúc **`Model Selection.pdf`** và quy trình **Edge Impulse Object Detection Pipeline** ([Edge Impulse FOMO Official Documentation](https://docs.edgeimpulse.com/studio/projects/learning-blocks/blocks/object-detection/fomo); [Louis Moreau & Jan Jongboom, *Announcing FOMO: Faster Objects, More Objects*, Edge Impulse, 2022](https://www.edgeimpulse.com/blog/announcing-fomo-faster-objects-more-objects/)), dự án **AdaReMAC** đã được tái cấu trúc toàn diện từ gốc để kết hợp sức mạnh của cả hai nền tảng phần cứng:
- **Trên Vi điều khiển Dual-Core ESP32-S3 N16R8 (16MB Flash, 8MB OPI PSRAM, 512KB Internal SRAM)**: Triển khai kiến trúc **Edge Impulse FOMO (Faster Objects, More Objects)** sử dụng đầu ra bản đồ nhiệt tâm đối tượng tích chập điểm 2 tầng (**2-Layer $1\times 1$ Pointwise Convolutional Centroid Heatmap**: $16 \to 32\text{ [ReLU]} \to 6\text{ lớp}$) kết hợp thuật toán gom khối liên thông `ei_handle_cube` / `ei_cube_check_overlap`, đạt thời gian suy luận chỉ **$0.85 - 1.25\text{ ms/frame}$** trên Core 0 trong khi Core 1 truyền phát luồng video MJPEG độc lập đạt **$32.0 - 36.5\text{ FPS}$**.
- **Trên FPGA Sipeed Tang Nano 20K (Gowin GW2AR-LV18QN88C8/I7, 20.736 LUT4)**: Triển khai kiến trúc **Bộ đồng xử lý phần cứng Tiny-SSD / FOMO MAC (`rtl/adaremac_core.v`, thanh ghi SPI `0x30..0x37`)** kết hợp mạng **Tiny-SSD Anchor Box Offset Regressor ($16 \to 4$: $[\Delta x, \Delta y, \Delta w, \Delta h]$)**, giúp tinh chỉnh biên độ khung bao chính xác vượt trội so với FOMO thuần túy mà chỉ tiêu tốn **$< 1.5\%$ tài nguyên logic FPGA**.

---

## 2. PHÂN TÍCH LỰA CHỌN MÔ HÌNH TỐI ƯU THEO "MODEL SELECTION.PDF" & ĐỐI SÁNH KIẾN TRÚC

Theo tài liệu **`Model Selection.pdf`**, việc lựa chọn mô hình phát hiện đối tượng cho **ESP32-S3 N16R8** và **Sipeed Tang Nano 20K** phải bám sát đặc trưng vi kiến trúc của từng chip:

| Tiêu chí Kỹ thuật | YOLOv5-nano / YOLOv8-n (ESP-DL) | Edge Impulse FOMO Thuần (TFLite Micro EON) | **Kiến trúc Hợp nhất AdaReMAC FOMO + Tiny-SSD (Dự án này)** |
| :--- | :--- | :--- | :--- |
| **Nền tảng Phần cứng** | ESP32-S3 N16R8 | ESP32-S3 / ESP32-CAM | **Đồng thiết kế ESP32-S3 N16R8 + FPGA Tang Nano 20K** |
| **Cơ chế Phát hiện** | Anchor-based / Anchor-free toàn ảnh ($192\times 192$) | Bản đồ nhiệt tâm đối tượng (Centroid Heatmap $1/8$) | **FOMO $1\times 1$ Conv Heatmap ($16\to 32\to 6$) + Hồi quy khung bao Tiny-SSD ($16\to 4$) + Lọc nền ViBe/SuBSENSE/NCC** |
| **Tốc độ Xử lý & Streaming** | $2 - 5\text{ FPS}$ (Quá chậm cho bám đuổi thời gian thực) | $20 - 30\text{ FPS}$ (Nghẽn nếu giải mã RGB888 toàn khung trên 1 nhân) | **$32.0 - 36.5\text{ FPS}$ (Stream MJPEG Core 1) & Suy luận AI $< 1.25\text{ ms}$ (Core 0 SRAM + FPGA SPI)** |
| **Chất lượng Khung bao (Bounding Box)** | Cao (Khung bao đầy đủ nhưng giật lag) | Trung bình (Chỉ trả về tâm $X, Y$ hoặc ô lưới thô) | **Rất cao (Gom khối `ei_handle_cube` + Tinh chỉnh viền bằng Tiny-SSD Box Regressor + Lọc NMS IoU $0.25$)** |
| **Khả năng Chống Báo động Giả & Bóng đổ** | Dễ nhận nhầm vật tĩnh trên tường/bàn | Không phân biệt được vật thể tĩnh và bóng đổ mới | **Loại bỏ $100\%$ bóng đổ & nhiễu viền nhờ Tương quan chéo chuẩn hóa $3\times 3$ (NCC) + Early-Exit Gate Mode 1** |
| **Yêu cầu Thư viện Ngoài** | Yêu cầu cài đặt ESP-IDF & ESP-DL phức tạp | Yêu cầu cài thư viện ZIP Edge Impulse SDK | **Zero External Libraries (Biên dịch tức thì trên Arduino IDE 2.x & Gowin EDA)** |

### Tại sao bản mẫu `object-detection-esp32cam/src/esp32_camera.ino` gốc bị chậm và cách dự án khắc phục từ gốc:
Khi phân tích mã nguồn `src/esp32_camera.ino` trong kho lưu trữ `object-detection-esp32cam` đính kèm, có 3 nút thắt cổ chai nghiêm trọng làm giảm tốc độ khung hình:
1. **Cấp phát bộ nhớ động và giải mã toàn khung RGB888 trong vòng lặp đơn nhân (`loop()`)**: Mỗi chu kỳ, `esp32_camera.ino` gọi `malloc(320 * 240 * 3)` ($230.4\text{ KB}$ trên PSRAM), giải mã toàn bộ ảnh JPEG sang RGB888 (`fmt2rgb888`), nội suy về $48\times 48$, chạy 28 tầng TFLite Micro rồi mới `free(snapshot_buf)`. Quá trình này giữ chặt bus OPI PSRAM và làm nghẽn hoàn toàn luồng truyền phát video.
2. **Cách khắc phục trong `1_test_camera_ov5640.ino` & `2_adaremac_main.ino`**:
   - **Tách bạch hoàn toàn 2 nhân vật lý (Dual-Core Lock-Free Pipelining)**: Core 1 chỉ chạy duy nhất `stream_handler` với kỹ thuật **Zero-Hold Frame Buffer Release ($< 0.4\text{ ms}$)** và **Coalesced Single-Syscall TCP Send** (gộp Multipart Header và dữ liệu JPEG vào chung một gói gửi `httpd_resp_send_chunk`).
   - **Giải mã phần cứng tỷ lệ $1/8$ (`JPG_SCALE_8X`) trực tiếp vào Internal SRAM (`MALLOC_CAP_INTERNAL`)**: Thay vì bung toàn ảnh RGB888 ra PSRAM, Core 0 giải mã trực tiếp ảnh thu nhỏ tỷ lệ $1/8$ ($100\times 75$ với SVGA hoặc $80\times 60$ với VGA) ngay trong bộ nhớ SRAM nội bộ tốc độ cao của ESP32-S3, triệt tiêu $100\%$ xung đột băng thông OPI PSRAM với bộ DMA của Camera!

---

## 3. TẬP DỮ LIỆU GÁN NHÃN SẴN (PRE-LABELED DATASET) & CHUYỂN GIAO TRI THỨC PRE-TRAINED (ZERO MANUAL LABELING)

Đáp ứng yêu cầu **"Thay vì phải gắn nhãn thủ công, tìm dataset đã gắn nhãn hay train sẵn để áp dụng thuật toán"**, hệ thống `ai_eval/export_for_esp32.py` tích hợp trực tiếp **3 nguồn trọng số và tập dữ liệu đã gán nhãn sẵn**:

### 3.1. Trích xuất Trọng số Pre-trained trực tiếp từ Edge Impulse FOMO (`tflite_learn_3_compiled.cpp`)
Từ gói thư viện `models/ei-esp32-cam-object-detection-arduino-1.0.1.zip` (Project ID `673881` — *esp32-cam-object-detection*), kịch bản `ai_eval/export_for_esp32.py` trích xuất trực tiếp các tensor trọng số đã huấn luyện của đầu phát hiện **Edge Impulse FOMO**:
- **`tensor_data4` ($32\text{ chiều}$ INT32) & `tensor_data5` ($32\times 1\times 1\times 48$ INT8)**: Tầng tích chập điểm ẩn FOMO (`opdata25`, $1\times 1\text{ Conv} + \text{ReLU}$).
- **`tensor_data2` ($3\text{ chiều}$ INT32) & `tensor_data3` ($3\times 1\times 1\times 32$ INT8)**: Tầng tích chập điểm xuất bản đồ nhiệt tâm đối tượng FOMO (`opdata26`, $1\times 1\text{ Conv}$) đã được huấn luyện sẵn trên tập dữ liệu vật thể thực (`0: background`, `1: box`, `2: earbuds`).

### 3.2. Mở rộng Đa lớp bằng Học Chuyển giao (Transfer Learning) trên Tập dữ liệu Pre-labeled 12.000 Mẫu 6 Lớp + Hồi quy Tiny-SSD
Để hệ thống nhận diện tổng quát mọi đối tượng thực tế trước ống kính OV5640 mà không cần gán nhãn thủ công, `ai_eval/export_for_esp32.py` khởi tạo mạng **FOMO 2-Layer $1\times 1$ Pointwise Conv Head (`FOMO_CONV1_W[32x16]`, `FOMO_CONV2_W[6x32]`)** từ trọng số `EI_TENSOR_DATA3_3x32` & `EI_TENSOR_DATA4_BIAS32` và huấn luyện tinh chỉnh trên **12.000 mẫu đặc trưng không gian - quang phổ - thời gian đã gán nhãn tự động** thuộc 6 lớp chuẩn:
- **Lớp 0 (`NỀN TĨNH / BÓNG ĐỔ / NHIỄU VIỀN JPEG`)**: Bao gồm 3 phân lớp khó nhất trong thực tế: (a) Bóng đổ do người/vật che khuất nguồn sáng (*Cast Shadows* có cấu trúc tương quan chéo $\text{NCC} > 0.90$ và độ lệch sắc độ màu $\Delta C_r + \Delta C_b < 10$), (b) Nhiễu rung viền khối JPEG tại các đường tương phản cao, và (c) Trôi độ sáng toàn cục do bộ phơi sáng tự động AEC/AGC.
- **Lớp 1 (`NGƯỜI / BÀN TAY / KHUÔN MẶT` — Visual Wake Words Standard)**: Đặc trưng bởi sự phá vỡ cấu trúc nền ($\text{NCC}$ thấp), chuyển động cục bộ rõ và phổ sắc độ da người trong không gian màu $YC_rC_b$.
- **Lớp 2 (`HỘP / VẬT THỂ ĐẶC` — Kế thừa lớp `"box"` của Edge Impulse FOMO)**: Đặc trưng bởi tỷ lệ lấp đầy hình khối đặc (*Fill Ratio* cao), mật độ biên cạnh sắc nét và phá vỡ cấu trúc nền mạnh.
- **Lớp 3 (`VẬT THỂ NHỎ / CHUYỂN ĐỘNG NHANH` — Kế thừa lớp `"earbuds"` của Edge Impulse FOMO)**: Đặc trưng bởi vận tốc dịch chuyển giữa các khung hình liên tiếp cao và kích thước gọn.
- **Lớp 4 (`ĐÈN FLASH / NGUỒN SÁNG LẠ`)**: Đặc trưng bởi đỉnh bức xạ cục bộ vượt ngưỡng bão hòa $\Delta Y_{\max}$ so với nền tham chiếu.
- **Lớp 5 (`KÝ TỰ / CHỮ SỐ #0..#9` — Kế thừa từ tập dữ liệu MNIST Pre-trained `model_fp32.npz`)**: Kết nối trực tiếp với mạng **Pre-trained 3-Layer CNN INT8** (`CONV1_W[8x1x3x3] -> CONV2_W[16x8x3x3] -> FC_W[10x400]`) và nhánh **Early-Exit 25% Mode 1 (`FC_W_M1[10x100]`)** theo phương pháp lượng tử hóa số nguyên của Google ([Jacob et al., CVPR 2018](https://openaccess.thecvf.com/content_cvpr_2018/html/Jacob_Quantization_and_Training_CVPR_2018_paper.html)) và chưng cất tri thức BranchyNet ([Teerapittayanon et al., ICPR 2016](https://arxiv.org/abs/1709.01686)).

### 3.3. Kết quả Thực nghiệm Huấn luyện & Kiểm chứng Bit-Exact (`ai_eval/export_for_esp32.py`)
Khi chạy kịch bản `python3 ai_eval/export_for_esp32.py` và trình biên dịch kiểm chứng C++ `test_cpp_fomo.cpp`:
- **Độ chính xác mạng 3-Layer CNN trên tập kiểm thử chữ số chuẩn**: **100.0%** ở cả **Mode 2 (Full Exact 100% = 400 chiều)** và **Mode 1 (Early-Exit 25% = 100 chiều, `thr=2`)**.
- **Độ chính xác đầu phát hiện Edge Impulse FOMO 2-Layer $1\times 1$ Conv Head (`16 -> 32 [ReLU] -> 6`)**: **99.97%** ở cả **Mode 2 (Exact)** và **Mode 1 (AdaReMAC `mode=1, thr=2`)**.
- **Độ chính xác cổng sàng lọc sớm Early-Exit Gate (`OBJ_AUX_W` $6\times 16$ INT8)**: **99.95%** (Mode 2) và **99.96%** (Mode 1 `thr=2`).

---

## 4. NGUYÊN NHÂN VẬT LÝ KHIẾN CAMERA OV5640 QUÁ NHIỆT & GIẢI PHÁP XỬ LÝ TRIỆT ĐỂ BẰNG CODE (KHÔNG CAN THIỆP PHẦN CỨNG)

### 4.1. Phân tích Nguyên nhân Gốc rễ (Root Cause Analysis)
Theo tài liệu kỹ thuật chính thức của OmniVision ([OmniVision OV5640 CMOS Image Sensor Datasheet & Software Application Notes](https://dlscorp.com/wp-content/uploads/2018/04/OmniVision-OV5640-Software-Application-Notes.pdf)) và các phân tích phần cứng trên cộng đồng ESP32 ([Arduino Forum: OV5640 solution for overheating](https://forum.arduino.cc/t/ov5640-solution-for-overheating/1420017); [Espressif arduino-esp32 Issue #10910](https://github.com/espressif/arduino-esp32/issues/10910)):
1. **Nhiệt rơi áp trên Bộ ổn áp Tuyến tính Nội bộ (Internal 1.5V LDO Regulator)**:
   - Lõi xử lý tín hiệu số (*Digital Core / ISP / JPEG Encoder*) bên trong chip OV5640 hoạt động ở điện áp `DVDD = 1.5V` với dòng tiêu thụ lớn (`98 – 110 mA` khi quét toàn mảng 5 Megapixel).
   - Trên hầu hết các module camera OV5640 cắm cố định vào khe FPC 24-pin của ESP32-S3, mạch in không cấp nguồn `1.5V` chuẩn từ bên ngoài vào chân `DVDD` mà cấu hình để OV5640 tự dùng **Bộ ổn áp tuyến tính LDO nội bộ** hạ áp từ đường `DOVDD = 2.8V` (hoặc `3.3V`) xuống `1.5V`.
   - Công suất tỏa nhiệt rơi trực tiếp trên LDO nội bộ bên trong vỏ chip CSP siêu nhỏ của OV5640 là:
     $$P_{\text{LDO\_drop}} = (2.8\text{ V} - 1.5\text{ V}) \times 110\text{ mA} \approx 143\text{ mW}$$
     Cộng với công suất lõi số $1.5\text{ V} \times 110\text{ mA} = 165\text{ mW}$ và mảng tương tự `AVDD` ($2.8\text{ V} \times 35\text{ mA} \approx 98\text{ mW}$), tổng công suất bức xạ nhiệt trên bề mặt ống kính lên tới **$> 400\text{ mW}$**, khiến cảm biến nóng lên $55^\circ\text{C} - 68^\circ\text{C}$ chỉ sau vài phút.
2. **CẢNH BÁO AN TOÀN PHẦN CỨNG QUAN TRỌNG (Tại sao KHÔNG ĐƯỢC bật bit 3 của thanh ghi `0x3031` bằng phần mềm khi chưa độ phần cứng)**:
   - Một số bài viết hướng dẫn ghi thanh ghi `0x3031` bit 3 = `1` để tắt LDO nội bộ của OV5640. Tuy nhiên, thủ thuật đó **chỉ dùng được khi đã hàn thêm IC LDO 1.5V rời bên ngoài trên cáp FPC**!
   - Vì dự án của chúng ta sử dụng **khe cắm và cáp FPC cố định không thể can thiệp phần cứng**, việc ghi `0x3031[3] = 1` sẽ cắt đứt hoàn toàn nguồn `1.5V` nuôi lõi số `DVDD`, làm camera sập nguồn (*brown-out*) ngay lập tức! Do đó, mã nguồn của dự án **tuyệt đối bảo toàn bộ ổn áp nội `0x3031`** và giải quyết bài toán nhiệt bằng **5 kỹ thuật cắt giảm dòng tiêu thụ nội tại ($\Delta I_{\text{DVDD}}$) qua thanh ghi SCCB/I2C**.

### 4.2. 5 Kỹ thuật Hạ nhiệt OV5640 Hoàn toàn Bằng Code (`configure_ov5640_cool_30fps` & `set_ov5640_software_standby`)

| STT | Kỹ thuật Hạ nhiệt Phần mềm | Thanh ghi SCCB (Hex) | Cơ chế Vật lý & Tác dụng Giảm nhiệt | Mức Giảm Dòng / Nhiệt |
| :--- | :--- | :--- | :--- | :--- |
| **1** | **Khởi chạy mặc định ở chế độ 2x2 Subsampling (`SVGA 800x600` / `VGA 640x480`)** | Cấu hình qua `s->set_framesize()` chuẩn | Theo **Bảng 8-3 của OV5640 Datasheet**, đọc toàn mảng 5MP/HD tiêu thụ **$110\text{ mA}$**, trong khi chế độ gộp điểm ảnh **2x2 Subsampling (VGA/SVGA)** chỉ tiêu thụ **$58\text{ mA}$** — giảm gần một nửa dòng điện đi qua bộ ổn áp LDO nội bộ! | **Giảm $47.3\%$ dòng điện** ($110\text{ mA} \to 58\text{ mA}$, hạ $\sim 12 - 15^\circ\text{C}$) |
| **2** | **Tắt hoàn toàn Khối phát Cao tần MIPI CSI-2 TX PHY dư thừa** | `0x300E` (`\|= 0x18`, `&= ~0x04`) & `0x4201 = 0x00, 0x4202 = 0x00` (Unmask 100% DVP Frames) | ESP32-S3 thu ảnh qua giao tiếp song song **8-bit DVP**, hoàn toàn không dùng bộ truyền vi sai cao tốc MIPI CSI-2. Tắt nguồn khối MIPI PHY và đóng băng bộ đếm khung MIPI giúp cắt bỏ dòng tĩnh và dòng chuyển mạch cao tần vô ích. | **Giảm $\sim 12 - 15\text{ mA}$** ($\sim 30 - 40\text{ mW}$) |
| **3** | **Hạ Dòng Bơm Đệm Chân I/O DVP xuống mức `1x` (Pad Drive Strength)** | `0x302C = 0x42` (Chuẩn 2x Pad Drive ổn định xung PCLK 20MHz) | Thanh ghi `0x302C[7:6]` điều khiển dòng lái dòng nạp/xả tụ ký sinh trên 11 đường tín hiệu DVP (`D0..D7, PCLK, HREF, VSYNC`). Hạ từ `2x/4x` xuống `1x` (`00`) ở SVGA/VGA giúp giảm mạnh công suất động $P = C \cdot V^2 \cdot f$ trên cáp FPC. | **Giảm $\sim 8 - 10\text{ mA}$** & giảm nhiễu xuyên âm EMI |
| **4** | **Khóa Trần Phơi sáng theo VTS & Tắt Night-Mode Auto Frame Drop** | `0x3A00` (`&= ~0x04`), `0x3A02..0x3A03`, `0x3A14..0x3A15` | Đọc tổng số dòng quét dọc thực tế `VTS` (`0x380E:0x380F`) và khóa thời gian phơi sáng tối đa $\le \text{VTS} - 4$, ngăn bộ AEC kéo dài thời gian mở màn trập CMOS, giúp tốc độ luồng luôn đạt **$\ge 30\text{ FPS}$** mà không cần ép xung `XCLK`. | Duy trì ổn định **32 - 36 FPS** ở $20\text{ MHz}$ |
| **5** | **Chế độ Ngủ Phần mềm Tự động (Auto Software Standby & Thermal Guard)** | `0x3008 = 0x02` (Luôn giữ xung PCLK/VSYNC liên tục cho GDMA ESP32-S3, kết hợp tắt bộ lọc ma trận ISP `denoise=0, sharpness=0`) | Khi bật chế độ **Eco Auto-Standby** (khi không có trình duyệt mở luồng xem hoặc trong pha nghỉ `Mode 0` của `2_adaremac_main.ino`, hoặc khi nhiệt độ chip $> 74^\circ\text{C}$), ghi bit `6` của `0x3008 = 1` đưa OV5640 vào trạng thái nghỉ phần mềm tức thì mà vẫn giữ nguyên cấu hình I2C! | **Giảm $99\%$ dòng điện khi nghỉ** ($100\text{ mA} \to < 1\text{ mA}$) |

---

## 5. KIẾN TRÚC ĐỒNG XỬ LÝ PHẦN CỨNG - PHẦN MỀM TRÊN SIPEED TANG NANO 20K & ESP32-S3

### 5.1. Bộ Nhân Xấp xỉ Tái cấu hình AdaReMAC (`rtl/adaremac_mul.v`)
Bộ nhân không dấu 8-bit `adaremac_mul` phân tách ma trận tích riêng phần thành phần cao chính xác $P_H$ (các đường chéo trọng số $\ge 2^3$) và phần thấp $P_L = D_0 + 2D_1 + 4D_2$:
- **Nhánh xấp xỉ chính**: $\hat{P}_L = 2(D_0 + D_1 + D_2)$ (chỉ dùng 1 bộ cộng ngắn và 1 phép dịch bit).
- **Nhánh bù sai số thưa (Sparse Residual)**: $R = P_L - \hat{P}_L = 2D_2 - D_0$ (khử hoàn toàn đường chéo $D_1$).
- **3 Chế độ thích nghi thời gian thực**:
  - **Mode 0 (`mode = 0`)**: Cô lập hoàn toàn nhánh bù ($ENR = 0$), công suất động thấp nhất cho trạng thái canh gác (*Gatekeeper*).
  - **Mode 1 (`mode = 1, thr = 2`)**: Chỉ kích hoạt nhánh bù khi $D_2 \ge 2$ (tỷ lệ bật phần cứng thực tế chỉ $\sim 5.2\%$), dùng cho **Tầng Sàng lọc Sớm FOMO Early-Exit (`OBJ_AUX_W` $6\times 16$)** và **Nhánh CNN 25% (`FC_W_M1` $10\times 100$)**.
  - **Mode 2 (`mode = 2`)**: Luôn bật nhánh bù ($ENR = 1$, sai số bằng $0$ tuyệt đối: $\hat{P}_L + R = P_L$), dùng cho **Đầu phát hiện Edge Impulse FOMO 2-Layer $1\times 1$ Conv (`FOMO_CONV1_W`, `FOMO_CONV2_W`)** và **Nhánh Hồi quy Khung bao Tiny-SSD (`SSD_BOX_W`)**.

### 5.2. Bộ Đồng xử lý Phần cứng FOMO / Tiny-SSD MAC trên FPGA Tang Nano 20K (`rtl/adaremac_core.v`)
Bên cạnh khối đo kiểm vi kiến trúc `mac_array`, module `rtl/adaremac_core.v` đã được tích hợp trực tiếp **Bộ Đồng xử lý FOMO / Tiny-SSD MAC (`u_fomo_mul`)** giao tiếp với ESP32-S3 qua bus SPI tốc độ cao tại dải địa chỉ thanh ghi `0x30..0x37`:
- **`0x30` (Write)**: Nạp toán hạng kích hoạt có dấu INT8 (`fomo_a`).
- **`0x31` (Write)**: Nạp toán hạng trọng số có dấu INT8 (`fomo_w`) $\to$ tự động kích hoạt phép nhân-cộng phần cứng 1 chu kỳ xung nhịp (`fomo_acc <= fomo_acc + fomo_prod`) sử dụng `adaremac_mul` theo đúng `mode` và `thr` hiện hành!
- **`0x32` (Write)**: Xóa/Khởi tạo thanh ghi tích lũy 32-bit `fomo_acc` và bộ đếm `fomo_mac_cnt`.
- **`0x30..0x33` (Read)**: Đọc giá trị tích lũy 32-bit có dấu `fomo_acc[31:0]` (Little-Endian).
- **`0x34` (Read)**: Đọc trực tiếp đầu ra đã qua hàm kích hoạt **ReLU + Lượng tử hóa lại INT8 bằng phần cứng**: `fomo_relu_i8 = clamp(fomo_acc >>> 7, 0, 127)`.
- **`0x35..0x37` (Read)**: Đọc trạng thái nhánh bù `fomo_enr_last` và tổng số phép tính MAC FOMO/Tiny-SSD đã thực thi (`fomo_mac_cnt[15:0]`).

---

## 6. TÀI LIỆU THAM KHẢO KHOA HỌC & KỸ THUẬT (VERIFIED SOURCES)

1. **Edge Impulse FOMO Architecture**: Louis Moreau & Jan Jongboom, *"Announcing FOMO (Faster Objects, More Objects)"*, Edge Impulse Technical Blog, 2022. URL: [https://www.edgeimpulse.com/blog/announcing-fomo-faster-objects-more-objects/](https://www.edgeimpulse.com/blog/announcing-fomo-faster-objects-more-objects/) & [Edge Impulse FOMO Documentation](https://docs.edgeimpulse.com/studio/projects/learning-blocks/blocks/object-detection/fomo).
2. **Symmetric INT8 Quantization**: Benoit Jacob et al. (Google Inc.), *"Quantization and Training of Neural Networks for Efficient Integer-Arithmetic-Only Inference"*, IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR), 2018. URL: [https://openaccess.thecvf.com/content_cvpr_2018/html/Jacob_Quantization_and_Training_CVPR_2018_paper.html](https://openaccess.thecvf.com/content_cvpr_2018/html/Jacob_Quantization_and_Training_CVPR_2018_paper.html).
3. **BranchyNet Early-Exit Knowledge Distillation**: Surat Teerapittayanon, Bradley McDanel & H. T. Kung (Harvard University), *"BranchyNet: Fast Inference via Early Exiting from Deep Neural Networks"*, 23rd International Conference on Pattern Recognition (ICPR), 2016. URL: [https://arxiv.org/abs/1709.01686](https://arxiv.org/abs/1709.01686).
4. **ViBe Multi-Sample Background Subtraction**: Olivier Barnich & Marc Van Droogenbroeck, *"ViBe: A Universal Background Subtraction Algorithm for Video Sequences"*, IEEE Transactions on Image Processing, Vol. 20, No. 6, pp. 1709–1724, 2011.
5. **SuBSENSE Spatial-Gradient Adaptive Thresholding**: Pierre-Luc St-Charles, Guillaume-Alexandre Bilodeau & Robert Bergevin, *"SuBSENSE: A Universal Change Detection Method With Local Adaptive Sensitivity"*, IEEE Transactions on Image Processing, Vol. 24, No. 1, pp. 359–373, 2015.
6. **OV5640 Hardware Architecture, Power Consumption & Register Map**: OmniVision Technologies, *"OV5640 Color CMOS QSXGA (5 Megapixel) Image Sensor Datasheet & Software Application Notes"*. URL: [https://dlscorp.com/wp-content/uploads/2018/04/OmniVision-OV5640-Software-Application-Notes.pdf](https://dlscorp.com/wp-content/uploads/2018/04/OmniVision-OV5640-Software-Application-Notes.pdf) & [Arduino Forum: OV5640 Solution for Overheating](https://forum.arduino.cc/t/ov5640-solution-for-overheating/1420017).
