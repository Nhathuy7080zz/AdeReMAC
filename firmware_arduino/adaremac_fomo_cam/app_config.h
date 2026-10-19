// =============================================================================
// Cấu hình phần cứng & tùy chọn — ESP32-S3 N16R8 CAM + OV5640 (+ Tang Nano 20K tùy chọn)
// Phiên bản v4.3: Tối ưu độ nét luồng Stream (ISP Sharpness + VCM Focus + SVGA Q8),
//                 Lưới nhận dạng siêu mịn 48x48 (2.304 ô) bám sát biên vật thể & Web UI 2 cột.
// Arduino IDE: Board "ESP32S3 Dev Module", PSRAM "OPI PSRAM", Flash 16MB,
//              Partition "Huge APP (3MB No OTA/1MB SPIFFS)", core esp32 >= 3.0
// =============================================================================
#pragma once

// ---------- Wi-Fi ----------
#define WIFI_AP_MODE        1            // 1: ESP32 phát Wi-Fi riêng; 0: kết nối router
#define WIFI_AP_SSID        "AdaReMAC-FOMO"
#define WIFI_AP_PASS        "12345678"
#define WIFI_STA_SSID       ""
#define WIFI_STA_PASS       ""
#define WIFI_TX_POWER       WIFI_POWER_15dBm   // cân bằng giữa tốc độ truyền stream SVGA/HD mượt và nhiệt độ bo

// ---------- Chân camera (ESP32-S3-CAM N16R8 phổ biến) ----------
#define CAM_PIN_PWDN    -1
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK    15
#define CAM_PIN_SIOD     4
#define CAM_PIN_SIOC     5
#define CAM_PIN_D7      16
#define CAM_PIN_D6      17
#define CAM_PIN_D5      18
#define CAM_PIN_D4      12
#define CAM_PIN_D3      10
#define CAM_PIN_D2       8
#define CAM_PIN_D1       9
#define CAM_PIN_D0      11
#define CAM_PIN_VSYNC    6
#define CAM_PIN_HREF     7
#define CAM_PIN_PCLK    13
#define CAM_XCLK_HZ     20000000

// ---------- Luồng video & Độ nét quang học ISP / VCM ----------
#define STREAM_FRAMESIZE       FRAMESIZE_SVGA  // 800x600 sắc nét (vẫn ở chế độ 2x2 binning <=1280x960 mát & nhanh)
#define STREAM_JPEG_QUALITY    8               // 6..20: giảm từ 12 xuống 8 để triệt tiêu nhòe khối DCT 8x8
#define CAM_DEFAULT_SHARPNESS  3               // 0..6: bật bộ làm sắc nét cạnh phần cứng ISP CIP (0x5302/0x5308)
#define CAM_DEFAULT_CONTRAST   1               // -2..2: tăng nhẹ tương phản cục bộ giúp biên vật thể rõ nét
#define CAM_DEFAULT_VCM_STEP   140             // 0..255: bước dòng cuộn dây lấy nét VCM mặc định cho cự ly 0.4m - 2.5m
#define FRAME_SLOT_BYTES       (192 * 1024)    // bộ đệm JPEG trong PSRAM (đủ cho cả SVGA/XGA/HD chất lượng cao)

// ---------- Độ mịn lưới nhận dạng FOMO Fine-Grid ----------
#define FOMO_DEFAULT_FINE_GRID 48              // 48: lưới siêu mịn 48x48 (2.304 ô, gấp 16 lần 12x12); hỗ trợ 12, 24, 48

// ---------- Tang Nano 20K (SPI) — giữ nguyên dây nối ----------
#define FPGA_ENABLE      1
#define PIN_FPGA_CS      1     // -> Pin 42 Tang Nano 20K
#define PIN_FPGA_MOSI    2     // -> Pin 48
#define PIN_FPGA_SCLK    42    // -> Pin 41
#define PIN_FPGA_MISO    41    // <- Pin 49
#define FPGA_SPI_HZ      20000000

// ---------- AI trên ESP32-S3 ----------
#define CPU_USE_ESPNN    1     // 1: esp-nn (SIMD, nhanh ~vài lần); 0: bản C chuẩn bit-exact với FPGA Mode 2

// ---------- Edge Impulse (tùy chọn) ----------
// #define USE_EDGE_IMPULSE 1
// #define EI_HEADER <AdaReMAC_person_inferencing.h>

// ---------- Quản lý nhiệt OV5640 (chỉ bằng phần mềm) ----------
#define THERMAL_ECO_DEFAULT       0       // mặc định tắt ngủ xen kẽ lúc mới bật để luồng stream luôn liên tục, sắc nét
#define THERMAL_IDLE_AFTER_MS     15000   // không người xem + không phát hiện trong 15 s -> chế độ nghỉ
#define THERMAL_IDLE_PERIOD_MS    1000    // chế độ nghỉ: đánh thức OV5640 mỗi 1 s để kiểm tra 1 khung
#define THERMAL_WAKE_DISCARD      6       // bỏ 6 khung đầu sau khi đánh thức để AE/AGC/AWB hội tụ hoàn toàn (tránh ảnh mờ/tối)
#define THERMAL_HOT_C             75.0f   // nhiệt độ chip ESP32-S3 vượt ngưỡng -> cảnh báo (chỉ ép nghỉ khi không có người xem)
#define THERMAL_PAD_DRIVE_1X      0       // 0: dùng mức 2x (0x42) giữ sườn xung PCLK/DVP sắc nét ở SVGA/HD; 1: hạ xuống 1x
#define THERMAL_PLL_IN_SPEC       1       // PLL tương đương nhưng VCO trong dải 500-1000 MHz
#define CAM_FPS_BOOST             0       // 1: PLL1 610 MHz tăng SYSCLK ~35% nếu cần ép thêm FPS

// ---------- INA219 (tùy chọn, đo dòng để kiểm chứng giảm nhiệt) ----------
#define INA219_ENABLE   0
#define PIN_INA_SDA     47
#define PIN_INA_SCL     21
#define INA219_ADDR     0x40
#define INA_SHUNT_OHMS  0.1f
