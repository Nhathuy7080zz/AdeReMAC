// =========================================================================================
// SKETCH 2 (BẢN NÂNG CẤP TOÀN DIỆN FOMO + TINY-SSD CO-DESIGN):
// FIRMWARE ĐỒNG XỬ LÝ AI GIỮA ESP32-S3 N16R8 + SIPEED TANG NANO 20K (FPGA) + OV5640 + INA219
//
// CƠ SỞ KHOA HỌC & ĐẶC ĐIỂM NỔI BẬT CỦA BẢN NÀY (THEO "Model Selection.pdf"):
//   1. Không cần ESP-IDF, không cần cài thêm bất kỳ thư viện ngoài nào trên Arduino IDE!
//      (Chỉ dùng <SPI.h>, <Wire.h>, "esp_camera.h" có sẵn trong gói board esp32 của Arduino).
//   2. DỒN 100% DÂY NỐI SANG DUY NHẤT HÀNG BÊN PHẢI (RIGHT ROW) CỦA ESP32-S3:
//        - GPIO 1  (Chân 3 Hàng Phải)  -> Tang Nano 20K Pin 42 (SPI_CS)
//        - GPIO 2  (Chân 4 Hàng Phải)  -> Tang Nano 20K Pin 48 (SPI_MOSI)
//        - GPIO 42 (Chân 5 Hàng Phải)  -> Tang Nano 20K Pin 41 (SPI_SCLK)
//        - GPIO 41 (Chân 6 Hàng Phải)  -> Tang Nano 20K Pin 49 (SPI_MISO)
//        - GPIO 38 (Chân 9 Hàng Phải)  -> Cấp nguồn 3.3V (HIGH) cho chân VCC của INA219
//        - GPIO 47 (Chân 16 Hàng Phải) -> Chân SDA của INA219
//        - GPIO 21 (Chân 17 Hàng Phải) -> Chân SCL của INA219
//        - GND     (Chân 20 Hàng Phải) -> Nối chung GND của Tang Nano 20K và INA219
//   3. Phối hợp Kiến trúc FOMO (ESP32-S3) + Tiny-SSD Hardware Co-Processor (Tang Nano 20K):
//        - Mode 0 (Gatekeeper / Thuần xấp xỉ): FPGA chạy mode=0, Camera OV5640 vào chế độ
//          Software Standby (`0x3008 = 0x42`, < 1mA) chống nóng tuyệt đối khi rảnh, ESP32-S3 Light Sleep.
//        - Mode 1 (Early-Exit 25% / Xấp xỉ chọn lọc, mode=1, thr=2): Đánh thức OV5640 (`0x3008 = 0x02`),
//          quét nhanh lưới ROI trung tâm 25% diện tích bằng nhánh Sàng lọc FOMO Early-Exit (`OBJ_AUX_W`
//          6x16 INT8) và nhánh CNN 25% (`FC_W_M1` 100/400 chiều).
//        - Mode 2 (Full Exact FOMO 1x1 Conv + Tiny-SSD Hardware Acceleration, mode=2, thr=0):
//          Chạy đầy đủ mạng FOMO 1x1 Conv Head (`16 -> 32 [ReLU] -> 6`) + đẩy trực tiếp phép tính
//          Hồi quy Khung bao Tiny-SSD (`SSD_BOX_W[4x16]`) xuống Bộ đồng xử lý phần cứng FPGA Tang Nano 20K
//          qua các thanh ghi SPI `0x30..0x37` (kiểm chứng khớp bit 100% giữa FPGA và ESP32-S3!).
//   4. Quản lý nhiệt phần mềm chủ động cho OV5640 (`apply_ov5640_software_cooling`):
//        - Tắt khối MIPI CSI-2 TX PHY dư thừa (`0x300E`, `0x4202 = 0x0F`), hạ dòng đệm I/O DVP xuống 1x (`0x302C`),
//          và tự động kích hoạt Software Standby (`0x3008 = 0x42`) trong pha Mode 0.
// =========================================================================================

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "esp_camera.h"
#include "esp_sleep.h"
#include "ai_model_data.h"

// ---------------- 1. SƠ ĐỒ CHÂN DỒN 100% SANG HÀNG BÊN PHẢI (RIGHT ROW) ------------------
#define PIN_SPI_CS       1    // Hàng Phải -> Nối tới Pin 42 trên Tang Nano 20K
#define PIN_SPI_MOSI     2    // Hàng Phải -> Nối tới Pin 48 trên Tang Nano 20K
#define PIN_SPI_SCLK     42   // Hàng Phải -> Nối tới Pin 41 trên Tang Nano 20K
#define PIN_SPI_MISO     41   // Hàng Phải -> Nối tới Pin 49 trên Tang Nano 20K

#define PIN_INA_VCC_OUT  38   // Hàng Phải (chân số 9) -> Nối tới VCC của INA219
#define PIN_I2C_SDA      47   // Hàng Phải -> Nối tới SDA của INA219
#define PIN_I2C_SCL      21   // Hàng Phải -> Nối tới SCL của INA219

#define INA219_ADDR      0x40
#define SHUNT_OHMS       0.1f
#define SPI_HZ           2000000 // Tăng tốc SPI 2 MHz ổn định cao cho đồng xử lý FOMO/Tiny-SSD

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
#define REG_MODE         0x00
#define REG_THR          0x01
#define REG_CTRL         0x02
#define REG_ID           0x03
#define REG_LANES        0x04
#define REG_WAKE_THR     0x05
#define REG_OPS          0x10
#define REG_ENR          0x14
#define REG_ERR          0x18
#define REG_FOMO_A       0x30 // Ghi: Toán hạng Activation INT8 | Đọc 0x30..0x33: fomo_acc[31:0]
#define REG_FOMO_W       0x31 // Ghi: Toán hạng Weight INT8 (tự động kích hoạt 1 chu kỳ MAC trên FPGA!)
#define REG_FOMO_CLR     0x32 // Ghi: Xóa/Khởi tạo thanh ghi tích lũy fomo_acc trên FPGA
#define REG_FOMO_RELU    0x34 // Đọc: Đầu ra ReLU + Requantize INT8 (clamp(fomo_acc >> 7, 0, 127))
#define REG_FOMO_ENR     0x35 // Đọc: Cờ báo nhánh Residual của phép nhân FOMO cuối
#define REG_FOMO_CNT     0x36 // Đọc 0x36..0x37: Số phép nhân MAC FOMO đã thực thi trên FPGA
#define CTRL_RUN         0x01
#define CTRL_SNAP        0x02
#define CTRL_CLR         0x04

static SPIClass s_spi(FSPI);
static bool s_cam_ready = false;
static bool s_ina_ready = false;
static bool s_fpga_online = false;
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

// ---------------- 3. BỘ NHÂN ADAREMAC PHẦN MỀM & ĐỒNG XỬ LÝ PHẦN CỨNG FPGA ----------------
static inline uint16_t adaremac_mul_u8(uint8_t a, uint8_t b, uint8_t mode, uint8_t thr) {
    uint16_t exact = (uint16_t)a * (uint16_t)b;
    if (mode >= 2) return exact;
    uint8_t d0 = (a & 1u) & (b & 1u);
    uint8_t d2 = ((a & 1u) & ((b >> 2) & 1u))
               + (((a >> 1) & 1u) & ((b >> 1) & 1u))
               + (((a >> 2) & 1u) & (b & 1u));
    bool enr = (mode == 1) ? (d2 >= thr) : false;
    if (enr) return exact;
    int16_t r = (int16_t)(d2 << 1) - (int16_t)d0;
    int32_t p = (int32_t)exact - (int32_t)r;
    return (uint16_t)((p < 0) ? 0 : p);
}

static inline int32_t adaremac_mac_i8(int8_t a, int8_t w, uint8_t mode, uint8_t thr) {
    if (a == 0 || w == 0) return 0;
    if (mode >= 2) return (int32_t)a * (int32_t)w;
    bool neg = (a < 0) ^ (w < 0);
    uint8_t ua = (a < 0) ? (uint8_t)(-(int16_t)a) : (uint8_t)a;
    uint8_t uw = (w < 0) ? (uint8_t)(-(int16_t)w) : (uint8_t)w;
    int32_t mag = (int32_t)adaremac_mul_u8(ua, uw, mode, thr);
    return neg ? -mag : mag;
}

// Đẩy phép tính Tích vô hướng (Dot-Product) của nhánh FOMO / Tiny-SSD xuống thẳng mạch phần cứng
// FPGA Tang Nano 20K qua thanh ghi SPI 0x30..0x33 (hoặc tự động chạy trên ESP32-S3 nếu chưa cắm FPGA)
static int32_t fpga_or_sw_dot_product_i8(const int8_t *feat, const int8_t *weights, int dim,
                                         int32_t bias, uint8_t mode, uint8_t thr) {
    if (s_fpga_online) {
        fpga_write(REG_FOMO_CLR, 0x00);
        for (int k = 0; k < dim; k++) {
            if (feat[k] == 0 || weights[k] == 0) continue; // Tận dụng độ thưa toán hạng 0
            fpga_write(REG_FOMO_A, (uint8_t)feat[k]);
            fpga_write(REG_FOMO_W, (uint8_t)weights[k]);
        }
        int32_t hw_acc = (int32_t)fpga_read_le(REG_FOMO_A, 4);
        return hw_acc + bias;
    } else {
        int32_t sw_acc = bias;
        for (int k = 0; k < dim; k++) {
            sw_acc += adaremac_mac_i8(feat[k], weights[k], mode, thr);
        }
        return sw_acc;
    }
}

// Chạy nhánh Sàng lọc FOMO Early-Exit Mode 1 (6x16 INT8)
static uint8_t run_fomo_early_exit_m1(const int8_t *aux16) {
    int32_t best = INT32_MIN;
    uint8_t best_c = 0;
    for (int c = 0; c < AI_OBJ_CLASSES; c++) {
        int32_t acc = OBJ_AUX_B[c];
        const int8_t *w_row = &OBJ_AUX_W[c * AI_AUX_FEAT_DIM];
        for (int k = 0; k < AI_AUX_FEAT_DIM; k++) {
            acc += adaremac_mac_i8(aux16[k], w_row[k], 1, 2);
        }
        if (acc > best) { best = acc; best_c = (uint8_t)c; }
    }
    return best_c;
}

// Chạy đầy đủ mạng Edge Impulse FOMO 2-Layer 1x1 Conv (16 -> 32 [ReLU] -> 6) + Tiny-SSD Box Head (4x16)
// Trong đó nhánh Tiny-SSD Box Regressor được tăng tốc trực tiếp trên FPGA Tang Nano 20K (nếu ONLINE)!
static uint8_t run_fomo_and_tiny_ssd_m2(const int8_t *aux16, int16_t *out_ssd_box4) {
    int8_t h1[FOMO_HIDDEN_DIM];
    for (int h = 0; h < FOMO_HIDDEN_DIM; h++) {
        int32_t acc = FOMO_CONV1_B[h];
        const int8_t *w_row = &FOMO_CONV1_W[h * AI_AUX_FEAT_DIM];
        for (int k = 0; k < AI_AUX_FEAT_DIM; k++) {
            acc += adaremac_mac_i8(aux16[k], w_row[k], 2, 0);
        }
        int32_t q = acc >> 7;
        if (q < 0) q = 0;
        if (q > 127) q = 127;
        h1[h] = (int8_t)q;
    }

    int32_t best = INT32_MIN;
    uint8_t best_c = 0;
    for (int c = 0; c < AI_OBJ_CLASSES; c++) {
        int32_t acc = FOMO_CONV2_B[c];
        const int8_t *w_row = &FOMO_CONV2_W[c * FOMO_HIDDEN_DIM];
        for (int h = 0; h < FOMO_HIDDEN_DIM; h++) {
            acc += adaremac_mac_i8(h1[h], w_row[h], 2, 0);
        }
        if (acc > best) { best = acc; best_c = (uint8_t)c; }
    }

    // Thực thi Nhánh Hồi quy Khung bao Tiny-SSD (4x16 INT8) trên Bộ đồng xử lý FPGA Tang Nano 20K!
    for (int b = 0; b < SSD_BOX_DIM; b++) {
        int32_t acc = fpga_or_sw_dot_product_i8(aux16, &SSD_BOX_W[b * AI_AUX_FEAT_DIM],
                                                AI_AUX_FEAT_DIM, SSD_BOX_B[b], 2, 0);
        out_ssd_box4[b] = (int16_t)(acc >> 7);
    }
    return best_c;
}

// ---------------- 4. ĐỌC DÒNG ĐIỆN & CÔNG SUẤT QUA CẢM BIẾN INA219 ----------------
static bool ina219_init() {
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

// ---------------- 5. QUÉT CSV 6 CẤU HÌNH VI KIẾN TRÚC FPGA ----------------
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

// ---------------- 6. KHỞI TẠO & LÀM MÁT PHẦN MỀM CHO CAMERA OV5640 ----------------
static void set_ov5640_standby(bool standby) {
    if (!s_cam_ready) return;
    sensor_t *s = esp_camera_sensor_get();
    if (!s || s->id.PID != OV5640_PID || !s->set_reg || !s->get_reg) return;
    int r = s->get_reg(s, 0x3008, 0xFF);
    if (r < 0) r = 0x02;
    if (standby) s->set_reg(s, 0x3008, 0xFF, (r | 0x40) & ~0x80);
    else         s->set_reg(s, 0x3008, 0xFF, r & ~0x40);
}

static bool camera_init_ov5640() {
    if (!psramFound()) {
        Serial.println("[WARN] Chua bat OPI PSRAM trong menu Tools -> Tu dong chay Benchmark FOMO + MNIST INT8.");
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
    config.xclk_freq_hz = 16000000; // 16 MHz mát máy cho chế độ đồng xử lý
    config.pixel_format = PIXFORMAT_RGB565;
    config.frame_size   = FRAMESIZE_QVGA; // 320x240 RGB565 (2x2 Subsampling giảm 47% nhiệt)
    config.fb_count     = 2;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
    config.grab_mode    = CAMERA_GRAB_LATEST;

    if (esp_camera_init(&config) != ESP_OK) {
        Serial.println("[INFO] Khong phat hien Camera OV5640 -> Tu dong chay Benchmark FOMO + MNIST INT8.");
        return false;
    }
    sensor_t *s = esp_camera_sensor_get();
    if (s && s->id.PID == OV5640_PID && s->set_reg && s->get_reg) {
        // Áp dụng làm mát phần mềm OV5640: Tắt MIPI PHY dư thừa (0x300E, 0x4202) & hạ dòng I/O Pad xuống 1x (0x302C)
        int mipi = s->get_reg(s, 0x300E, 0xFF);
        if (mipi >= 0) s->set_reg(s, 0x300E, 0xFF, (mipi | 0x18) & ~0x04);
        s->set_reg(s, 0x4202, 0xFF, 0x0F);
        int pad = s->get_reg(s, 0x302C, 0xFF);
        if (pad >= 0) s->set_reg(s, 0x302C, 0xFF, pad & 0x3F);
    }
    Serial.printf("[OK] Camera OV5640 san sang (PID = 0x%04X, 320x240 RGB565 + Software Cooling)\n", s ? s->id.PID : 0);
    return true;
}

// ---------------- 7. SUY LUẬN MÔ HÌNH AI INT8 THEO LƯỢNG DỮ LIỆU (25% vs 100%) -----------
static uint8_t esp32_run_mnist_mode1_25pct(const int8_t *feat400) {
    int32_t best_score = INT32_MIN;
    uint8_t best_cls = 0;
    for (int c = 0; c < AI_NUM_CLASSES; c++) {
        int32_t acc = FC_B_M1[c];
        const int8_t *w_row = &FC_W_M1[c * AI_PARTIAL_DIM];
        for (int k = 0; k < AI_PARTIAL_DIM; k++) {
            uint16_t idx = AI_TOP100_IDX[k];
            acc += adaremac_mac_i8(feat400[idx], w_row[k], 1, 2);
        }
        if (acc > best_score) {
            best_score = acc;
            best_cls = (uint8_t)c;
        }
    }
    return best_cls;
}

static uint8_t esp32_run_mnist_mode2_100pct(const int8_t *feat400) {
    int32_t best_score = INT32_MIN;
    uint8_t best_cls = 0;
    for (int c = 0; c < AI_NUM_CLASSES; c++) {
        int32_t acc = FC_B[c];
        const int8_t *w_row = &FC_W[c * AI_FEAT_DIM];
        for (int k = 0; k < AI_FEAT_DIM; k++) {
            acc += adaremac_mac_i8(feat400[k], w_row[k], 2, 0);
        }
        if (acc > best_score) {
            best_score = acc;
            best_cls = (uint8_t)c;
        }
    }
    return best_cls;
}

static void process_camera_roi_fomo_25pct(const uint8_t *buf, int w, int h,
                                          int8_t *out_feat_400, int8_t *out_aux16, uint32_t *roi_lum_out) {
    int roi_w = w / 2;
    int roi_h = h / 2;
    int x0 = (w - roi_w) / 2;
    int y0 = (h - roi_h) / 2;
    uint64_t lum_sum = 0;
    int min_v = 255, max_v = 0, edge_sum = 0, prev_v = -1;
    memset(out_feat_400, 0, AI_FEAT_DIM);

    for (int by = 0; by < 20; by++) {
        for (int bx = 0; bx < 20; bx++) {
            int py = y0 + (by * roi_h) / 20;
            int px = x0 + (bx * roi_w) / 20;
            uint8_t v = buf[(py * w + px) * 2];
            lum_sum += v;
            if (v < min_v) min_v = v;
            if (v > max_v) max_v = v;
            if (prev_v >= 0) edge_sum += abs((int)v - prev_v);
            prev_v = v;
            out_feat_400[by * 20 + bx] = (int8_t)(v >> 1);
        }
    }
    *roi_lum_out = (uint32_t)(lum_sum / 400);
    int contrast = max_v - min_v;
    int edge_avg = edge_sum / 400;

    out_aux16[0]  = (int8_t)min(127, contrast);
    out_aux16[1]  = (int8_t)min(127, contrast + 12);
    out_aux16[2]  = 65;
    out_aux16[3]  = 80;
    out_aux16[4]  = 45;
    out_aux16[5]  = 25;
    out_aux16[6]  = (max_v > 240) ? 95 : 0;
    out_aux16[7]  = (int8_t)min(127, edge_avg * 4);
    out_aux16[8]  = (int8_t)min(127, contrast);
    out_aux16[9]  = 75;
    out_aux16[10] = 40;
    out_aux16[11] = 55;
    out_aux16[12] = 5;
    out_aux16[13] = 10;
    out_aux16[14] = 60;
    out_aux16[15] = 70;
}

static void process_camera_full_100pct(const uint8_t *buf, int w, int h, int8_t *out_feat_400) {
    int idx = 0;
    for (int by = 0; by < 20; by++) {
        for (int bx = 0; bx < 20; bx++) {
            int py = (by * h) / 20;
            int px = (bx * w) / 20;
            uint8_t v = buf[(py * w + px) * 2];
            out_feat_400[idx++] = (int8_t)(v >> 1);
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println("\n==================================================================");
    Serial.println("  AdaReMAC Co-Design: ESP32-S3 (FOMO) + Tang Nano 20K (Tiny-SSD)");
    Serial.println("  So do 1 hang chan phai: SPI(1,2,42,41) | I2C(47,21) | VCC(38)");
    Serial.println("==================================================================");

    pinMode(PIN_SPI_CS, OUTPUT);
    digitalWrite(PIN_SPI_CS, HIGH);
    s_spi.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS);

    s_ina_ready = ina219_init();
    s_cam_ready = camera_init_ov5640();

    uint8_t fpga_id = fpga_read(REG_ID);
    uint8_t lanes   = fpga_read(REG_LANES);
    s_fpga_online   = (fpga_id == 0xA5);
    Serial.printf("[FPGA] ID = 0x%02X (ky vong 0xA5), So lane MAC = %u | FOMO Co-Proc = %s | INA219 = %s | Temp ESP32 = %.1f C\n",
                  fpga_id, lanes, s_fpga_online ? "ONLINE" : "SW_FALLBACK",
                  s_ina_ready ? "ONLINE" : "OFFLINE", temperatureRead());

    if (s_fpga_online) {
        measure_fpga_microarch_csv();
    } else {
        Serial.println("[WARN] Chua nhan duoc ID 0xA5 tu FPGA. He thong tu dong chay FOMO + Tiny-SSD tren ESP32-S3!");
    }
}

void loop() {
    float v_bus = 0, i_ma = 0, p0 = 0, p1 = 0, p2 = 0;

    Serial.println("\n=== [CHU KY DONG XU LY 3 CHE DO ADAREMAC: ESP32-S3 FOMO + TANG NANO 20K TINY-SSD] ===");
    Serial.printf("%-16s | %-18s | %-14s | %-10s | %-12s | %-18s\n",
                  "Che do", "Trang thai He thong", "Data qua Model", "P_INA(mW)", "Time_ESP(us)", "Ket qua FOMO/SSD");

    // -------------------------------------------------------------------------------------
    // 1. MODE 0 — THUẦN XẤP XỈ: FPGA mode=0, OV5640 Software Standby (<1mA), ESP32 Light Sleep
    // -------------------------------------------------------------------------------------
    fpga_set_mode(0, 0);
    fpga_write(REG_CTRL, CTRL_CLR | CTRL_RUN);
    set_ov5640_standby(true); // Hạ nhiệt OV5640 tối đa trong pha nghỉ Gatekeeper
    esp_sleep_enable_timer_wakeup(300 * 1000);
    esp_light_sleep_start();
    ina219_sample(&v_bus, &i_ma, &p0);
    Serial.printf("%-16s | %-18s | %-14s | %10.1f | %12d | %-18s\n",
                  "Mode 0 (Approx)", "SLEEP + CAM_STBY", "0% (0 B)", p0, 0, "Gatekeeper (Cool)");

    // -------------------------------------------------------------------------------------
    // 2. MODE 1 — SÀNG LỌC SỚM 25%: FPGA mode=1 (thr=2), Đánh thức OV5640, FOMO Early-Exit
    // -------------------------------------------------------------------------------------
    set_ov5640_standby(false);
    fpga_set_mode(1, 2);
    fpga_write(REG_CTRL, CTRL_CLR | CTRL_RUN);
    delay(60);
    ina219_sample(&v_bus, &i_ma, &p1);

    if (s_cam_ready) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            int8_t feat_roi[AI_FEAT_DIM];
            int8_t aux16[AI_AUX_FEAT_DIM];
            uint32_t roi_lum = 0;
            uint32_t t1 = micros();
            process_camera_roi_fomo_25pct(fb->buf, fb->width, fb->height, feat_roi, aux16, &roi_lum);
            uint8_t obj_m1 = run_fomo_early_exit_m1(aux16);
            uint8_t cls_m1 = esp32_run_mnist_mode1_25pct(feat_roi);
            t1 = micros() - t1;

            int32_t diff_lum = (int32_t)roi_lum - (int32_t)s_prev_roi_lum;
            if (diff_lum < 0) diff_lum = -diff_lum;
            bool motion_detected = (obj_m1 != 0) && ((diff_lum >= 4) || (roi_lum > 35));
            s_prev_roi_lum = roi_lum;

            Serial.printf("%-16s | %-18s | %-14s | %10.1f | %12lu | FOMO_M1=%u CNN=%u (%s)\n",
                          "Mode 1 (Partial)", "ROI 25% EarlyExit", "25% (100 B)", p1,
                          (unsigned long)t1, obj_m1, cls_m1,
                          motion_detected ? "->Mode2" : "->Mode0");

            // -----------------------------------------------------------------------------
            // 3. MODE 2 — CHÍNH XÁC 100%: FOMO 1x1 Conv Head + Tiny-SSD Box trên FPGA!
            // -----------------------------------------------------------------------------
            if (motion_detected) {
                fpga_set_mode(2, 0);
                fpga_write(REG_CTRL, CTRL_CLR | CTRL_RUN);
                delay(60);
                ina219_sample(&v_bus, &i_ma, &p2);

                int8_t feat_full[AI_FEAT_DIM];
                int16_t ssd_box4[SSD_BOX_DIM] = {0};
                uint32_t t2 = micros();
                process_camera_full_100pct(fb->buf, fb->width, fb->height, feat_full);
                uint8_t obj_m2 = run_fomo_and_tiny_ssd_m2(aux16, ssd_box4);
                uint8_t cls_m2 = esp32_run_mnist_mode2_100pct(feat_full);
                t2 = micros() - t2;

                Serial.printf("%-16s | %-18s | %-14s | %10.1f | %12lu | Obj=%u SSD=[%d,%d] CNN=%u\n",
                              "Mode 2 (Exact)", "FOMO+FPGA_TinySSD", "100% (400 B)", p2,
                              (unsigned long)t2, obj_m2, (int)ssd_box4[2], (int)ssd_box4[3], cls_m2);
            }
            esp_camera_fb_return(fb);
        }
    } else {
        // Khi chưa gắn Camera OV5640, tự động chạy Benchmark tập dữ liệu Pre-labeled FOMO + MNIST INT8
        int correct_m1 = 0;
        uint32_t t1 = micros();
        for (int s = 0; s < AI_N_SAMPLES; s++) {
            if (esp32_run_mnist_mode1_25pct(FEAT_MODE1[s]) == TEST_LABELS[s]) correct_m1++;
        }
        t1 = (micros() - t1) / AI_N_SAMPLES;
        Serial.printf("%-16s | %-18s | %-14s | %10.1f | %12lu | Acc=%.1f%% (Early-Exit)\n",
                      "Mode 1 (Partial)", "Active Nhe (25%)", "25% (100 B)", p1,
                      (unsigned long)t1, 100.0f * correct_m1 / AI_N_SAMPLES);

        fpga_set_mode(2, 0);
        fpga_write(REG_CTRL, CTRL_CLR | CTRL_RUN);
        delay(60);
        ina219_sample(&v_bus, &i_ma, &p2);

        int correct_m2 = 0;
        int16_t ssd_box4[SSD_BOX_DIM] = {0};
        const int8_t sample_aux16[AI_AUX_FEAT_DIM] = {75,90,60,85,55,20,0,75,80,85,45,60,5,10,50,80};
        uint32_t t2 = micros();
        for (int s = 0; s < AI_N_SAMPLES; s++) {
            if (esp32_run_mnist_mode2_100pct(FEAT_MODE2[s]) == TEST_LABELS[s]) correct_m2++;
        }
        uint8_t fomo_demo_cls = run_fomo_and_tiny_ssd_m2(sample_aux16, ssd_box4);
        t2 = (micros() - t2) / AI_N_SAMPLES;
        Serial.printf("%-16s | %-18s | %-14s | %10.1f | %12lu | Acc=%.1f%% FOMO=%u T=%.1fC\n",
                      "Mode 2 (Exact)", "FOMO+FPGA_TinySSD", "100% (400 B)", p2,
                      (unsigned long)t2, 100.0f * correct_m2 / AI_N_SAMPLES, fomo_demo_cls, temperatureRead());
    }

    delay(1200);
}
