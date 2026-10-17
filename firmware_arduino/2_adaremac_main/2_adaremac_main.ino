// =========================================================================================
// SKETCH 2: FIRMWARE CHÍNH TOÀN HỆ THỐNG ADAREMAC TRÊN ARDUINO IDE 2.x
// Phần cứng: ESP32-S3 N16R8 + Camera OV5640 + FPGA Tang Nano 20K (SPI) + INA219 (I2C)
//
// ĐẶC ĐIỂM NỔI BẬT CỦA BẢN NÀY:
//   1. Không cần ESP-IDF, không cần cài thêm bất kỳ thư viện ngoài nào trên Arduino IDE!
//      (Chỉ dùng <SPI.h>, <Wire.h>, "esp_camera.h" có sẵn trong gói board esp32 của Arduino).
//   2. DỒN 100% DÂY NỐI SANG DUY NHẤT HÀNG BÊN PHẢI (RIGHT ROW) CỦA ESP32-S3:
//      Khi cắm ESP32-S3 lên Breadboard chỉ còn trống 1 bên lỗ bên phải, bạn cắm toàn bộ dây
//      ngay trên Hàng Phải theo thứ tự từ trên xuống:
//        - GPIO 1  (Chân 3 Hàng Phải)  -> Tang Nano 20K Pin 42 (SPI_CS)
//        - GPIO 2  (Chân 4 Hàng Phải)  -> Tang Nano 20K Pin 48 (SPI_MOSI)
//        - GPIO 42 (Chân 5 Hàng Phải)  -> Tang Nano 20K Pin 41 (SPI_SCLK)
//        - GPIO 41 (Chân 6 Hàng Phải)  -> Tang Nano 20K Pin 49 (SPI_MISO)
//        - GPIO 38 (Chân 9 Hàng Phải)  -> Cấp nguồn 3.3V (HIGH) cho chân VCC của INA219
//        - GPIO 47 (Chân 16 Hàng Phải) -> Chân SDA của INA219
//        - GPIO 21 (Chân 17 Hàng Phải) -> Chân SCL của INA219
//        - GND     (Chân 20 Hàng Phải) -> Nối chung GND của Tang Nano 20K và INA219
//   3. Vận hành Kiến trúc 3 Chế độ thích nghi:
//        - Mode 0 (Thuần xấp xỉ)    : FPGA chạy mode=0, ESP32-S3 ngắt CPU (Light Sleep), Data = 0%.
//        - Mode 1 (Xấp xỉ một phần) : FPGA chạy mode=1 (thr=2), ESP32-S3 chỉ xử lý 25% dữ liệu
//                                     (cắt ROI 160x120 từ Camera OV5640 + 25% đặc trưng 100/400 byte).
//        - Mode 2 (Chính xác 100%)  : FPGA chạy mode=2 (Exact), ESP32-S3 xử lý 100% dữ liệu
//                                     (toàn khung hình 320x240 + 100% đặc trưng 400/400 byte).
// =========================================================================================

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "esp_camera.h"
#include "esp_sleep.h"
#include "ai_model_data.h"

// ---------------- 1. SƠ ĐỒ CHÂN DỒN 100% SANG HÀNG BÊN PHẢI (RIGHT ROW) ------------------
// Cụm 4 chân SPI đứng liền kề nhau từ trên xuống ở góc trên bên phải (chân số 3, 4, 5, 6):
#define PIN_SPI_CS       1    // Hàng Phải -> Nối tới Pin 42 trên Tang Nano 20K
#define PIN_SPI_MOSI     2    // Hàng Phải -> Nối tới Pin 48 trên Tang Nano 20K
#define PIN_SPI_SCLK     42   // Hàng Phải -> Nối tới Pin 41 trên Tang Nano 20K
#define PIN_SPI_MISO     41   // Hàng Phải -> Nối tới Pin 49 trên Tang Nano 20K

// Chân cấp nguồn 3.3V phụ trên Hàng Phải nuôi mạch logic của INA219 (chỉ tiêu thụ 0.7mA):
#define PIN_INA_VCC_OUT  38   // Hàng Phải (chân số 9) -> Nối tới VCC của INA219

// Cụm 2 chân I2C đứng liền kề nhau ở góc dưới bên phải (chân số 16, 17):
#define PIN_I2C_SDA      47   // Hàng Phải -> Nối tới SDA của INA219
#define PIN_I2C_SCL      21   // Hàng Phải -> Nối tới SCL của INA219

#define INA219_ADDR      0x40
#define SHUNT_OHMS       0.1f
#define SPI_HZ           1000000

// Sơ đồ 14 chân cố định của khe cắm cáp dẹt FPC 24-pin (nằm hoàn toàn bên Hàng Trái):
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

// Bản đồ thanh ghi FPGA Tang Nano 20K (rtl/adaremac_core.v)
#define REG_MODE      0x00
#define REG_THR       0x01
#define REG_CTRL      0x02
#define REG_ID        0x03
#define REG_LANES     0x04
#define REG_WAKE_THR  0x05
#define REG_OPS       0x10
#define REG_ENR       0x14
#define REG_ERR       0x18
#define CTRL_RUN      0x01
#define CTRL_SNAP     0x02
#define CTRL_CLR      0x04

static SPIClass s_spi(FSPI);
static bool s_cam_ready = false;
static bool s_ina_ready = false;
static uint32_t s_prev_roi_lum = 0;

// ---------------- 2. GIAO TIẾP SPI VỚI FPGA TANG NANO 20K ----------------
static uint8_t fpga_xfer(uint8_t b0, uint8_t b1) {
    s_spi.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_SPI_CS, LOW);
    s_spi.transfer(b0);
    uint8_t rx1 = s_spi.transfer(b1);
    digitalWrite(PIN_SPI_CS, HIGH);
    s_spi.endTransaction();
    return rx1;
}

static void fpga_write(uint8_t addr, uint8_t val) {
    fpga_xfer(0x80 | (addr & 0x7F), val);
}

static uint8_t fpga_read(uint8_t addr) {
    return fpga_xfer(addr & 0x7F, 0x00);
}

static uint64_t fpga_read_le(uint8_t addr, int nbytes) {
    uint64_t v = 0;
    for (int i = 0; i < nbytes; i++) {
        v |= ((uint64_t)fpga_read(addr + i)) << (8 * i);
    }
    return v;
}

static void fpga_set_mode(uint8_t mode, uint8_t thr) {
    fpga_write(REG_MODE, mode & 0x03);
    fpga_write(REG_THR,  thr  & 0x03);
}

// ---------------- 3. ĐỌC DÒNG ĐIỆN & CÔNG SUẤT QUA CẢM BIẾN INA219 ----------------
static bool ina219_init() {
    // Bật GPIO 38 trên Hàng Phải xuất 3.3V để nuôi chân VCC của INA219
    pinMode(PIN_INA_VCC_OUT, OUTPUT);
    digitalWrite(PIN_INA_VCC_OUT, HIGH);
    delay(10);

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
    Wire.beginTransmission(INA219_ADDR);
    Wire.write(0x00);
    Wire.write(0x09); // Config 0x09DF: 16V bus, +/-40mV shunt, 128-sample averaging
    Wire.write(0xDF);
    return (Wire.endTransmission() == 0);
}

static bool ina219_read16(uint8_t reg, uint16_t *val) {
    Wire.beginTransmission(INA219_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)INA219_ADDR, (uint8_t)2) != 2) return false;
    *val = ((uint16_t)Wire.read() << 8) | Wire.read();
    return true;
}

static void ina219_sample(float *v_bus, float *i_ma, float *p_mw) {
    if (!s_ina_ready) {
        *v_bus = 0.0f; *i_ma = 0.0f; *p_mw = 0.0f;
        return;
    }
    uint16_t sh = 0, bus = 0;
    if (!ina219_read16(0x01, &sh) || !ina219_read16(0x02, &bus)) {
        *v_bus = 0.0f; *i_ma = 0.0f; *p_mw = 0.0f;
        return;
    }
    *v_bus = (float)(bus >> 3) * 0.004f;
    *i_ma  = (float)((int16_t)sh) * 0.01f / SHUNT_OHMS;
    *p_mw  = (*v_bus) * (*i_ma);
}

// ---------------- 4. QUÉT CSV 6 CẤU HÌNH VI KIẾN TRÚC FPGA ----------------
typedef struct { const char *name; uint8_t run, mode, thr; } cfg_t;
static const cfg_t CFGS[] = {
    { "idle",       0, 2, 0 },
    { "mode0",      1, 0, 0 },
    { "mode1_thr3", 1, 1, 3 },
    { "mode1_thr2", 1, 1, 2 },
    { "mode1_thr1", 1, 1, 1 },
    { "mode2",      1, 2, 0 },
};
#define NCFG (sizeof(CFGS) / sizeof(CFGS[0]))

static void measure_fpga_microarch_csv() {
    Serial.println("\nCSV,config,mode,thr,vbus_V,i_mA,p_mW,ops,residual_active,nmed");
    for (size_t k = 0; k < NCFG; k++) {
        const cfg_t *c = &CFGS[k];
        fpga_set_mode(c->mode, c->thr);
        fpga_write(REG_CTRL, CTRL_CLR | (c->run ? CTRL_RUN : 0));
        delay(300);

        double sv = 0, si = 0;
        const int NSAMPLES = 40;
        for (int n = 0; n < NSAMPLES; n++) {
            float v, i, p;
            ina219_sample(&v, &i, &p);
            sv += v; si += i;
            delay(5);
        }
        float v = sv / NSAMPLES;
        float i = si / NSAMPLES;
        float p_mw = v * i;

        fpga_write(REG_CTRL, CTRL_SNAP | (c->run ? CTRL_RUN : 0));
        uint64_t ops = fpga_read_le(REG_OPS, 4);
        uint64_t enr = fpga_read_le(REG_ENR, 4);
        uint64_t err = fpga_read_le(REG_ERR, 5);
        float act  = ops ? ((float)enr / (float)ops) : 0.0f;
        float nmed = ops ? ((float)err / (float)ops / 16384.0f) : 0.0f;

        Serial.printf("CSV,%s,%u,%u,%.3f,%.2f,%.1f,%llu,%.4f,%.3e\n",
                      c->name, c->mode, c->thr, v, i, p_mw,
                      (unsigned long long)ops, act, nmed);
    }
}

// ---------------- 5. KHỞI TẠO CAMERA OV5640 ----------------
static bool camera_init_ov5640() {
    if (!psramFound()) {
        Serial.println("[WARN] Chua bat OPI PSRAM trong menu Tools -> Tu dong chay Benchmark MNIST INT8.");
        return false;
    }
    camera_config_t config = {};
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0 = CAM_PIN_D0; config.pin_d1 = CAM_PIN_D1;
    config.pin_d2 = CAM_PIN_D2; config.pin_d3 = CAM_PIN_D3;
    config.pin_d4 = CAM_PIN_D4; config.pin_d5 = CAM_PIN_D5;
    config.pin_d6 = CAM_PIN_D6; config.pin_d7 = CAM_PIN_D7;
    config.pin_xclk = CAM_PIN_XCLK; config.pin_pclk = CAM_PIN_PCLK;
    config.pin_vsync = CAM_PIN_VSYNC; config.pin_href = CAM_PIN_HREF;
    config.pin_sccb_sda = CAM_PIN_SIOD; config.pin_sccb_scl = CAM_PIN_SIOC;
    config.pin_pwdn = CAM_PIN_PWDN; config.pin_reset = CAM_PIN_RESET;
    config.xclk_freq_hz = 16000000;
    config.pixel_format = PIXFORMAT_RGB565;
    config.frame_size   = FRAMESIZE_QVGA; // 320x240 RGB565
    config.fb_count     = 2;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
    config.grab_mode    = CAMERA_GRAB_LATEST;

    if (esp_camera_init(&config) != ESP_OK) {
        Serial.println("[INFO] Khong phat hien Camera OV5640 -> Tu dong chay Benchmark MNIST INT8.");
        return false;
    }
    sensor_t *s = esp_camera_sensor_get();
    Serial.printf("[OK] Camera OV5640 san sang (PID = 0x%04X, 320x240 RGB565)\n", s ? s->id.PID : 0);
    return true;
}

// ---------------- 6. SUY LUẬN MÔ HÌNH AI INT8 THEO LƯỢNG DỮ LIỆU (25% vs 100%) -----------
static uint8_t esp32_run_mnist_classifier(const int8_t *feat, int n_dims) {
    int32_t best_score = INT32_MIN;
    uint8_t best_cls = 0;
    for (int c = 0; c < AI_NUM_CLASSES; c++) {
        int32_t acc = FC_B[c];
        const int8_t *w_row = &FC_W[c * AI_FEAT_DIM];
        for (int k = 0; k < n_dims; k++) {
            acc += (int32_t)feat[k] * (int32_t)w_row[k];
        }
        if (acc > best_score) {
            best_score = acc;
            best_cls = (uint8_t)c;
        }
    }
    return best_cls;
}

// Trích xuất đặc trưng từ vùng trung tâm 25% diện tích khung hình (ROI 160x120) của Camera OV5640
static void process_camera_roi_25pct(const uint8_t *buf, int w, int h,
                                     int8_t *out_feat_100, uint32_t *roi_lum_out) {
    int roi_w = w / 2; // 160
    int roi_h = h / 2; // 120
    int x0 = (w - roi_w) / 2;
    int y0 = (h - roi_h) / 2;
    uint64_t lum_sum = 0;
    int idx = 0;
    for (int by = 0; by < 10; by++) {
        for (int bx = 0; bx < 10; bx++) {
            int py = y0 + (by * roi_h) / 10;
            int px = x0 + (bx * roi_w) / 10;
            uint8_t v = buf[(py * w + px) * 2];
            lum_sum += v;
            out_feat_100[idx++] = (int8_t)((int)v - 128);
        }
    }
    *roi_lum_out = (uint32_t)(lum_sum / 100);
}

// Trích xuất đặc trưng trên 100% toàn khung hình (320x240) của Camera OV5640
static void process_camera_full_100pct(const uint8_t *buf, int w, int h, int8_t *out_feat_400) {
    int idx = 0;
    for (int by = 0; by < 20; by++) {
        for (int bx = 0; bx < 20; bx++) {
            int py = (by * h) / 20;
            int px = (bx * w) / 20;
            uint8_t v = buf[(py * w + px) * 2];
            out_feat_400[idx++] = (int8_t)((int)v - 128);
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println("\n==================================================================");
    Serial.println("  AdaReMAC (Arduino IDE) - ESP32-S3 + Tang Nano 20K + OV5640");
    Serial.println("  So do 1 hang chan phai: SPI(1,2,42,41) | I2C(47,21) | VCC(38)");
    Serial.println("==================================================================");

    // 1. Khởi tạo SPI nối tới FPGA Tang Nano 20K
    pinMode(PIN_SPI_CS, OUTPUT);
    digitalWrite(PIN_SPI_CS, HIGH);
    s_spi.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS);

    // 2. Khởi tạo I2C đọc INA219 và Camera OV5640
    s_ina_ready = ina219_init();
    s_cam_ready = camera_init_ov5640();

    uint8_t fpga_id = fpga_read(REG_ID);
    uint8_t lanes   = fpga_read(REG_LANES);
    Serial.printf("[FPGA] ID = 0x%02X (ky vong 0xA5), So lane MAC = %u | INA219 = %s | Temp ESP32 = %.1f C\n",
                  fpga_id, lanes, s_ina_ready ? "ONLINE" : "OFFLINE", temperatureRead());

    if (fpga_id == 0xA5) {
        measure_fpga_microarch_csv();
    } else {
        Serial.println("[WARN] Chua nhan duoc ID 0xA5 tu FPGA. Kiem tra 4 day SPI (GPIO 1,2,42,41) va GND chung!");
    }
}

void loop() {
    float v_bus = 0, i_ma = 0, p0 = 0, p1 = 0, p2 = 0;

    Serial.println("\n=== [CHU KY VAN HANH 3 CHE DO THICH NGHI ADAREMAC] ===");
    Serial.printf("%-16s | %-18s | %-14s | %-10s | %-12s | %-12s\n",
                  "Che do", "Trang thai ESP32", "Data qua Model", "P_INA(mW)", "Time_ESP(us)", "Ket qua");

    // -------------------------------------------------------------------------------------
    // 1. MODE 0 — THUẦN XẤP XỈ: FPGA chạy mode=0, ESP32-S3 ngắt CPU (Light Sleep), Data = 0%
    // -------------------------------------------------------------------------------------
    fpga_set_mode(0, 0);
    fpga_write(REG_CTRL, CTRL_CLR | CTRL_RUN);
    esp_sleep_enable_timer_wakeup(300 * 1000); // Ngủ nhẹ 300ms tiết kiệm điện
    esp_light_sleep_start();
    ina219_sample(&v_bus, &i_ma, &p0);
    Serial.printf("%-16s | %-18s | %-14s | %10.1f | %12d | %-12s\n",
                  "Mode 0 (Approx)", "SLEEP (Ngat CPU)", "0% (0 B)", p0, 0, "Gatekeeper");

    // -------------------------------------------------------------------------------------
    // 2. MODE 1 — XẤP XỈ MỘT PHẦN: FPGA chạy mode=1 (thr=2), ESP32-S3 xử lý 25% lượng dữ liệu
    // -------------------------------------------------------------------------------------
    fpga_set_mode(1, 2);
    fpga_write(REG_CTRL, CTRL_CLR | CTRL_RUN);
    delay(80);
    ina219_sample(&v_bus, &i_ma, &p1);

    if (s_cam_ready) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            int8_t feat_roi[AI_PARTIAL_DIM];
            uint32_t roi_lum = 0;
            uint32_t t1 = micros();
            process_camera_roi_25pct(fb->buf, fb->width, fb->height, feat_roi, &roi_lum);
            uint8_t cls_m1 = esp32_run_mnist_classifier(feat_roi, AI_PARTIAL_DIM);
            t1 = micros() - t1;

            int32_t diff_lum = (int32_t)roi_lum - (int32_t)s_prev_roi_lum;
            if (diff_lum < 0) diff_lum = -diff_lum;
            bool motion_detected = (diff_lum >= 4) || (roi_lum > 40);
            s_prev_roi_lum = roi_lum;

            Serial.printf("%-16s | %-18s | %-14s | %10.1f | %12lu | Lum=%lu (%s)\n",
                          "Mode 1 (Partial)", "ROI 25% (160x120)", "25% (100 B)", p1,
                          (unsigned long)t1, (unsigned long)roi_lum,
                          motion_detected ? "Len Mode 2" : "Ve Mode 0");

            // -----------------------------------------------------------------------------
            // 3. MODE 2 — CHÍNH XÁC 100%: Đưa 100% khung hình (320x240) & 400/400B qua Model
            // -----------------------------------------------------------------------------
            if (motion_detected) {
                fpga_set_mode(2, 0);
                fpga_write(REG_CTRL, CTRL_CLR | CTRL_RUN);
                delay(80);
                ina219_sample(&v_bus, &i_ma, &p2);

                int8_t feat_full[AI_FEAT_DIM];
                uint32_t t2 = micros();
                process_camera_full_100pct(fb->buf, fb->width, fb->height, feat_full);
                uint8_t cls_m2 = esp32_run_mnist_classifier(feat_full, AI_FEAT_DIM);
                t2 = micros() - t2;

                Serial.printf("%-16s | %-18s | %-14s | %10.1f | %12lu | Class=%u | T=%.1fC\n",
                              "Mode 2 (Exact)", "Full 100%(320x240)", "100% (400 B)", p2,
                              (unsigned long)t2, cls_m2, temperatureRead());
            }
            esp_camera_fb_return(fb);
        }
    } else {
        // Khi chưa gắn Camera OV5640, tự động chạy Benchmark tập dữ liệu MNIST INT8
        int correct_m1 = 0;
        uint32_t t1 = micros();
        for (int s = 0; s < AI_N_SAMPLES; s++) {
            if (esp32_run_mnist_classifier(FEAT_MODE1[s], AI_PARTIAL_DIM) == TEST_LABELS[s]) correct_m1++;
        }
        t1 = (micros() - t1) / AI_N_SAMPLES;
        Serial.printf("%-16s | %-18s | %-14s | %10.1f | %12lu | Acc=%.1f%%\n",
                      "Mode 1 (Partial)", "Active Nhe (25%)", "25% (100 B)", p1,
                      (unsigned long)t1, 100.0f * correct_m1 / AI_N_SAMPLES);

        fpga_set_mode(2, 0);
        fpga_write(REG_CTRL, CTRL_CLR | CTRL_RUN);
        delay(80);
        ina219_sample(&v_bus, &i_ma, &p2);

        int correct_m2 = 0;
        uint32_t t2 = micros();
        for (int s = 0; s < AI_N_SAMPLES; s++) {
            if (esp32_run_mnist_classifier(FEAT_MODE2[s], AI_FEAT_DIM) == TEST_LABELS[s]) correct_m2++;
        }
        t2 = (micros() - t2) / AI_N_SAMPLES;
        Serial.printf("%-16s | %-18s | %-14s | %10.1f | %12lu | Acc=%.1f%% | T=%.1fC\n",
                      "Mode 2 (Exact)", "Full Model (100%)", "100% (400 B)", p2,
                      (unsigned long)t2, 100.0f * correct_m2 / AI_N_SAMPLES, temperatureRead());
    }

    delay(1500);
}
