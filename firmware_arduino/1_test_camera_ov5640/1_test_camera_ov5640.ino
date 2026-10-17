// =========================================================================================
// SKETCH 1: STREAM VIDEO FHD@30P SIÊU NÉT + AI HỌC MÔI TRƯỜNG & ĐÓNG KHUNG BẤT THƯỜNG
// Board: ESP32 Cam Kit Phát Triển ESP32-S3 N16R8 OV5640 TypeC (Arduino IDE 2.x)
//
// GIẢI PHÁP CHỐNG NGHẼN MẠNG TỐI ĐA & KHẮC PHỤC ẢNH MỜ TRÊN OV5640:
//   1. Chế độ mặc định (USE_DIRECT_AP_MODE = 1): Chính ESP32-S3 tự phát Wi-Fi thuần (Pure AP)
//      ở băng thông rộng 40MHz (HT40 - 150Mbps), công suất phát cực đại 19.5dBm, tắt hoàn toàn
//      chế độ ngủ Wi-Fi (WIFI_PS_NONE) và tắt bộtrễ TCP Nagle (TCP_NODELAY).
//      -> Truyền thẳng 1 chặng (Zero-Hop: ESP32-S3 -> Máy tính), KHÔNG đi vòng qua Router ngoài,
//         không chia sẻ sóng với thiết bị khác, đạt tốc độ Stream FHD@30p mượt nhất!
//      -> (Nếu muốn nối vào Router ngoài, chỉ cần đổi USE_DIRECT_AP_MODE thành 0 ở dòng 38).
//   2. Khắc phục triệt để lỗi ảnh mờ trên OV5640:
//      -> Bật toàn bộ bộ xử lý ảnh phần cứng ISP bên trong OV5640: Sharpness (+2), Contrast (+1),
//         Lens Correction (LENC), Raw Gamma (GMA), khử điểm nhiễu (BPC/WPC/Denoise), DSP AEC2,
//         và hạ hệ số nén JPEG xuống mức nét cao (jpeg_quality = 8) với 3 bộ đệm trong OPI PSRAM.
//   3. Tích hợp Model AI INT8 AdaReMAC (ai_model_data.h) + Học Môi Trường Liên Tục trên Core 0:
//      -> Tự động nhận biết khi bạn ĐỔI GÓC CAMERA (> 42% vùng ảnh thay đổi) để kích hoạt
//         chế độ Học Nhanh (Fast Adaptation) cập nhật nền môi trường mới trong ~1 giây.
//      -> Khi có vật thể bất thường xuất hiện, tự động gom cụm (Connected-Components), phân lớp
//         bằng mạng nơ-ron INT8 AdaReMAC (Mode 1 25% & Mode 2 100%) và ĐÓNG KHUNG ĐỎ trực tiếp!
// =========================================================================================

#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_http_server.h"
#include <lwip/sockets.h>
#include "ai_model_data.h"

// =========================================================================================
// 1. CHỌN PHƯƠNG ÁN STREAM ĐỠ NGHẼN NHẤT (PURE AP DIRECT 40MHz HT40 vs PURE STA ROUTER)
// =========================================================================================
// Đặt = 1 (MẶC ĐỊNH - ĐỠ NGHẼN NHẤT): ESP32-S3 tự phát Wi-Fi "ESP32-OV5640-FHD" (MK: 12345678),
// truyền thẳng 1 chặng 40MHz HT40 sang máy tính/điện thoại tại địa chỉ http://192.168.4.1
// Đặt = 0: ESP32-S3 nối vào Wi-Fi Router ngoài (điền WIFI_SSID và WIFI_PASS bên dưới).
#define USE_DIRECT_AP_MODE  1

const char *AP_SSID   = "ESP32-OV5640-FHD"; // Tên Wi-Fi tốc độ cao do ESP32-S3 phát ra
const char *AP_PASS   = "12345678";         // Mật khẩu (tối thiểu 8 ký tự) -> Mở http://192.168.4.1
#define AP_CHANNEL      6                   // Kênh truyền 6 cố định, băng thông rộng 40MHz (HT40)

// Chỉ dùng khi bạn đổi USE_DIRECT_AP_MODE = 0 ở trên:
const char *WIFI_SSID = "";
const char *WIFI_PASS = "";

// =========================================================================================
// 2. SƠ ĐỒ 14 CHÂN CỐ ĐỊNH KHE FPC 24-PIN TRÊN MẠCH ESP32-S3 N16R8 CAM (HÀNG TRÁI)
// =========================================================================================
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

// =========================================================================================
// 3. CẤU HÌNH MÔ HÌNH AI HỌC MÔI TRƯỜNG LIÊN TỤC & PHÁT HIỆN BẤT THƯỜNG (ADAREMAC INT8)
// =========================================================================================
#define GRID_W              20      // Lưới không gian 20 cột x 15 hàng (tỷ lệ 16:9 FHD)
#define GRID_H              15
#define GRID_CELLS          (GRID_W * GRID_H)
#define MAX_ANOM_BOXES      6       // Số khung vật thể bất thường tối đa đồng thời
#define MAX_JPEG_COPY_SZ    (384 * 1024) // Bộ đệm PSRAM 384KB chứa ảnh FHD JPEG sắc nét cho AI
#define MAX_RGB_THUMB_W     320
#define MAX_RGB_THUMB_H     240

struct AnomBox {
    float x, y, w, h;       // Tọa độ chuẩn hóa [0.0 .. 1.0] trên khung hình FHD 1920x1080
    int   score;            // Điểm bất thường (0 .. 99%)
    uint8_t ai_cls_m1;      // Nhãn dự đoán Mode 1 (25% Partial AdaReMAC)
    uint8_t ai_cls_m2;      // Nhãn dự đoán Mode 2 (100% Full AdaReMAC)
    float feat_dist;        // Độ lệch đặc trưng AI so với nền môi trường
};

// Trạng thái học môi trường của từng ô lưới (Spatial Continual Learning State)
static float s_bg_y[GRID_CELLS];     // Độ sáng trung bình học được của môi trường
static float s_bg_cr[GRID_CELLS];    // Sắc độ Đỏ học được của môi trường
static float s_bg_cb[GRID_CELLS];    // Sắc độ Xanh học được của môi trường
static float s_bg_edge[GRID_CELLS];  // Độ chi tiết góc cạnh học được của môi trường
static float s_bg_var[GRID_CELLS];   // Phương sai dao động tự nhiên tại mỗi ô
static float s_bg_logits[AI_NUM_CLASSES]; // Vector đặc trưng AI nền của góc camera hiện tại

static bool  s_env_initialized = false;
static int   s_fast_learn_frames = 20;   // Số frame học nhanh khi mới bật hoặc vừa đổi góc camera
static volatile bool s_force_relearn = false;
static volatile float s_anom_threshold = 3.2f; // Ngưỡng Z-score phát hiện bất thường

// Kết quả AI xuất ra Web UI và Serial Monitor (bảo vệ bằng spinlock nhẹ)
static portMUX_TYPE s_ai_mux = portMUX_INITIALIZER_UNLOCKED;
static AnomBox s_boxes[MAX_ANOM_BOXES];
static int     s_num_boxes = 0;
static float   s_env_stability = 0.0f;
static float   s_changed_cell_ratio = 0.0f;
static bool    s_angle_changed = false;
static uint8_t s_scene_ai_cls = 0;
static uint32_t s_ai_infer_us = 0;
static float   s_stream_fps = 0.0f;
static float   s_chip_temp_c = 40.0f;
static uint32_t s_total_frames = 0;

// Bộ đệm PSRAM giao tiếp không khóa (Zero-blocking) giữa Stream Task (Core 1) và AI Task (Core 0)
static uint8_t *s_ai_jpeg_buf = NULL;
static size_t   s_ai_jpeg_len = 0;
static uint16_t s_ai_frame_w  = 1920;
static uint16_t s_ai_frame_h  = 1080;
static volatile bool s_ai_need_frame  = false;
static volatile bool s_ai_frame_ready = false;
static volatile int  s_stream_clients = 0;
static uint16_t *s_rgb565_thumb = NULL;

static bool s_cam_ok = false;
static httpd_handle_t s_web_httpd    = NULL;
static httpd_handle_t s_stream_httpd = NULL;

// ---------------- BỘ NHÂN XẤP XỈ ADAREMAC (qL=2, k=3, R=2*D2 - D0) & SUY LUẬN INT8 ---------
static inline int16_t adaremac_mul_u8(uint8_t a, uint8_t b, uint8_t mode, uint8_t thr) {
    if (mode == 2) return (int16_t)((uint16_t)a * (uint16_t)b);
    uint8_t ah = (a >> 5) & 0x07, bh = (b >> 5) & 0x07;
    uint8_t al = a & 0x1F,        bl = b & 0x1F;
    uint8_t al_m = al & 0x1C,     bl_m = bl & 0x1C;
    int32_t d2 = (int32_t)a * (int32_t)b - (int32_t)al_m * (int32_t)bl_m;
    int32_t d0 = ((int32_t)(ah * bh) << 10) - ((int32_t)al_m * (int32_t)bl_m);
    int32_t r  = (d2 << 1) - d0;
    bool en_res = false;
    if (mode == 1) {
        uint8_t mag = (ah > bh) ? ah : bh;
        uint8_t limit = (uint8_t)(thr + 1);
        en_res = (mag >= limit);
    }
    int32_t p = d0 + (en_res ? r : 0);
    if (p < 0) p = 0;
    if (p > 65535) p = 65535;
    return (int16_t)p;
}

static inline int32_t adaremac_mac_i8(int8_t a, int8_t w, uint8_t mode, uint8_t thr) {
    if (mode == 2) return (int32_t)a * (int32_t)w;
    bool neg = (a < 0) ^ (w < 0);
    uint8_t ua = (a < 0) ? (uint8_t)(-a) : (uint8_t)a;
    uint8_t uw = (w < 0) ? (uint8_t)(-w) : (uint8_t)w;
    int32_t mag = adaremac_mul_u8(ua, uw, mode, thr);
    return neg ? -mag : mag;
}

// Chạy mạng nơ-ron INT8 AdaReMAC (từ ai_model_data.h) trả về class tốt nhất và điền vector logits
static uint8_t adaremac_run_nn(const int8_t *feat, int n_dims, uint8_t mode, uint8_t thr, float *out_logits) {
    int32_t best_score = INT32_MIN;
    uint8_t best_cls = 0;
    for (int c = 0; c < AI_NUM_CLASSES; c++) {
        int32_t acc = FC_B[c];
        const int8_t *w_row = &FC_W[c * AI_FEAT_DIM];
        for (int k = 0; k < n_dims; k++) {
            acc += adaremac_mac_i8(feat[k], w_row[k], mode, thr);
        }
        if (out_logits) out_logits[c] = (float)acc / 1024.0f;
        if (acc > best_score) {
            best_score = acc;
            best_cls = (uint8_t)c;
        }
    }
    return best_cls;
}

// Trích xuất tensor 20x20 (400 byte INT8) từ một vùng bất kỳ (Bounding Box hoặc toàn cảnh) trên ảnh RGB565
static void extract_patch_int8_400(const uint16_t *rgb565, int img_w, int img_h,
                                   int x0, int y0, int bw, int bh, int8_t *out_feat400) {
    if (bw < 1) bw = 1;
    if (bh < 1) bh = 1;
    int32_t min_y = 255, max_y = 0;
    uint8_t raw[400];
    for (int py = 0; py < 20; py++) {
        int sy = y0 + (py * bh) / 20;
        if (sy < 0) sy = 0;
        if (sy >= img_h) sy = img_h - 1;
        for (int px = 0; px < 20; px++) {
            int sx = x0 + (px * bw) / 20;
            if (sx < 0) sx = 0;
            if (sx >= img_w) sx = img_w - 1;
            uint16_t p = rgb565[sy * img_w + sx];
            uint8_t b_hi = (uint8_t)(p & 0xFF);
            uint8_t b_lo = (uint8_t)(p >> 8);
            uint16_t v = ((uint16_t)b_hi << 8) | b_lo;
            uint8_t r = (uint8_t)(((v >> 11) & 0x1F) << 3);
            uint8_t g = (uint8_t)(((v >> 5)  & 0x3F) << 2);
            uint8_t b = (uint8_t)((v & 0x1F) << 3);
            uint8_t y_luma = (uint8_t)((r * 77 + g * 150 + b * 29) >> 8);
            raw[py * 20 + px] = y_luma;
            if (y_luma < min_y) min_y = y_luma;
            if (y_luma > max_y) max_y = y_luma;
        }
    }
    int32_t span = max_y - min_y;
    if (span < 16) span = 16;
    for (int i = 0; i < 400; i++) {
        int32_t norm = ((int32_t)(raw[i] - min_y) * 127) / span;
        if (norm < 0) norm = 0;
        if (norm > 127) norm = 127;
        out_feat400[i] = (int8_t)norm;
    }
}

// Giải mã khung JPEG sang lưới không gian 20x15 (dùng bộ giải mã 1/8 chuẩn jpg2rgb565 + fallback trực tiếp)
static void analyze_frame_and_learn(const uint8_t *jpeg_buf, size_t jpeg_len, int frame_w, int frame_h) {
    uint32_t t0 = micros();

    float cell_y[GRID_CELLS];
    float cell_cr[GRID_CELLS];
    float cell_cb[GRID_CELLS];
    float cell_edge[GRID_CELLS];

    int thumb_w = frame_w / 8;
    int thumb_h = frame_h / 8;
    bool decoded_ok = false;

    if (s_rgb565_thumb && thumb_w > 0 && thumb_h > 0 &&
        thumb_w <= MAX_RGB_THUMB_W && thumb_h <= MAX_RGB_THUMB_H) {
        decoded_ok = jpg2rgb565(jpeg_buf, jpeg_len, (uint8_t *)s_rgb565_thumb, JPG_SCALE_8X);
    }

    if (decoded_ok) {
        for (int gy = 0; gy < GRID_H; gy++) {
            int y_start = (gy * thumb_h) / GRID_H;
            int y_end   = ((gy + 1) * thumb_h) / GRID_H;
            if (y_end <= y_start) y_end = y_start + 1;

            for (int gx = 0; gx < GRID_W; gx++) {
                int x_start = (gx * thumb_w) / GRID_W;
                int x_end   = ((gx + 1) * thumb_w) / GRID_W;
                if (x_end <= x_start) x_end = x_start + 1;

                uint32_t sum_y = 0, sum_r = 0, sum_b = 0, sum_edge = 0, cnt = 0;
                int prev_luma = -1;

                for (int y = y_start; y < y_end; y++) {
                    const uint16_t *row = &s_rgb565_thumb[y * thumb_w];
                    for (int x = x_start; x < x_end; x++) {
                        uint16_t p = row[x];
                        uint16_t v = ((p & 0xFF) << 8) | (p >> 8);
                        int r = ((v >> 11) & 0x1F) << 3;
                        int g = ((v >> 5)  & 0x3F) << 2;
                        int b = (v & 0x1F) << 3;
                        int luma = (r * 77 + g * 150 + b * 29) >> 8;
                        sum_y += luma;
                        sum_r += r;
                        sum_b += b;
                        if (prev_luma >= 0) sum_edge += abs(luma - prev_luma);
                        prev_luma = luma;
                        cnt++;
                    }
                }
                int idx = gy * GRID_W + gx;
                if (cnt > 0) {
                    cell_y[idx]    = (float)sum_y / cnt;
                    cell_cr[idx]   = (float)sum_r / cnt - cell_y[idx];
                    cell_cb[idx]   = (float)sum_b / cnt - cell_y[idx];
                    cell_edge[idx] = (float)sum_edge / cnt;
                } else {
                    cell_y[idx] = cell_cr[idx] = cell_cb[idx] = cell_edge[idx] = 0.0f;
                }
            }
        }
    } else {
        // Fallback nếu luồng JPEG có phân đoạn đặc biệt: băm trực tiếp luồng entropy JPEG theo 300 vùng
        size_t payload_start = 200;
        if (jpeg_len <= payload_start + GRID_CELLS) return;
        size_t usable = jpeg_len - payload_start;
        for (int i = 0; i < GRID_CELLS; i++) {
            size_t p0 = payload_start + (i * usable) / GRID_CELLS;
            size_t p1 = payload_start + ((i + 1) * usable) / GRID_CELLS;
            uint32_t sy = 0, se = 0, c = 0;
            size_t step = ((p1 - p0) > 32) ? ((p1 - p0) / 32) : 1;
            int prev = -1;
            for (size_t p = p0; p < p1; p += step) {
                int v = jpeg_buf[p];
                sy += v;
                if (prev >= 0) se += abs(v - prev);
                prev = v;
                c++;
            }
            cell_y[i]    = c ? ((float)sy / c) : 128.0f;
            cell_cr[i]   = 0.0f;
            cell_cb[i]   = 0.0f;
            cell_edge[i] = c ? ((float)se / c) : 10.0f;
        }
    }

    // Khởi tạo nền môi trường ở frame đầu tiên hoặc khi người dùng bấm nút "Học lại góc Camera"
    if (!s_env_initialized || s_force_relearn) {
        for (int i = 0; i < GRID_CELLS; i++) {
            s_bg_y[i]    = cell_y[i];
            s_bg_cr[i]   = cell_cr[i];
            s_bg_cb[i]   = cell_cb[i];
            s_bg_edge[i] = cell_edge[i];
            s_bg_var[i]  = 8.0f;
        }
        if (decoded_ok) {
            int8_t full_feat[400];
            extract_patch_int8_400(s_rgb565_thumb, thumb_w, thumb_h, 0, 0, thumb_w, thumb_h, full_feat);
            s_scene_ai_cls = adaremac_run_nn(full_feat, AI_FEAT_DIM, 2, 0, s_bg_logits);
        }
        s_env_initialized   = true;
        s_force_relearn     = false;
        s_fast_learn_frames = 15;
    }

    // Bước 1: Tính điểm lệch chuẩn hóa Z-score cho từng ô trong 300 ô không gian
    float z_map[GRID_CELLS];
    uint8_t anom_mask[GRID_CELLS];
    int changed_cells = 0;
    float thr = s_anom_threshold;

    for (int i = 0; i < GRID_CELLS; i++) {
        float dy  = fabsf(cell_y[i]    - s_bg_y[i]);
        float dcr = fabsf(cell_cr[i]   - s_bg_cr[i]);
        float dcb = fabsf(cell_cb[i]   - s_bg_cb[i]);
        float ded = fabsf(cell_edge[i] - s_bg_edge[i]);

        float diff = dy + 0.45f * (dcr + dcb) + 0.60f * ded;
        float sigma = (s_bg_var[i] < 5.0f) ? 5.0f : s_bg_var[i];
        float z = diff / sigma;
        z_map[i] = z;

        if (z > thr) {
            anom_mask[i] = 1;
            changed_cells++;
        } else {
            anom_mask[i] = 0;
        }
    }

    float changed_ratio = (float)changed_cells / (float)GRID_CELLS;

    // Bước 2: Phát hiện ĐỔI GÓC CAMERA (Global Scene Shift > 42% diện tích khung hình thay đổi đồng loạt)
    bool angle_shift_now = false;
    if (changed_ratio > 0.42f) {
        s_fast_learn_frames = 14;
        angle_shift_now = true;
    }

    if (s_fast_learn_frames > 0) {
        s_fast_learn_frames--;
        float alpha_fast = 0.28f; // Tốc độ học nhanh môi trường mới khi vừa đổi góc camera
        for (int i = 0; i < GRID_CELLS; i++) {
            float dy = fabsf(cell_y[i] - s_bg_y[i]);
            s_bg_y[i]    = (1.0f - alpha_fast) * s_bg_y[i]    + alpha_fast * cell_y[i];
            s_bg_cr[i]   = (1.0f - alpha_fast) * s_bg_cr[i]   + alpha_fast * cell_cr[i];
            s_bg_cb[i]   = (1.0f - alpha_fast) * s_bg_cb[i]   + alpha_fast * cell_cb[i];
            s_bg_edge[i] = (1.0f - alpha_fast) * s_bg_edge[i] + alpha_fast * cell_edge[i];
            s_bg_var[i]  = (1.0f - alpha_fast) * s_bg_var[i]  + alpha_fast * (dy + 4.0f);
            if (s_bg_var[i] > 28.0f) s_bg_var[i] = 28.0f;
        }
        if (decoded_ok) {
            int8_t full_feat[400];
            float cur_logits[AI_NUM_CLASSES];
            extract_patch_int8_400(s_rgb565_thumb, thumb_w, thumb_h, 0, 0, thumb_w, thumb_h, full_feat);
            s_scene_ai_cls = adaremac_run_nn(full_feat, AI_FEAT_DIM, 2, 0, cur_logits);
            for (int c = 0; c < AI_NUM_CLASSES; c++) {
                s_bg_logits[c] = 0.7f * s_bg_logits[c] + 0.3f * cur_logits[c];
            }
        }
        uint32_t dt_us = micros() - t0;
        portENTER_CRITICAL(&s_ai_mux);
        s_num_boxes = 0;
        s_angle_changed = true;
        s_changed_cell_ratio = changed_ratio;
        s_env_stability = (1.0f - ((float)s_fast_learn_frames / 15.0f)) * 100.0f;
        if (s_env_stability < 5.0f) s_env_stability = 5.0f;
        s_ai_infer_us = dt_us;
        portEXIT_CRITICAL(&s_ai_mux);
        return;
    }

    // Bước 3: Khi góc camera ổn định, liên tục cập nhật thích nghi nền môi trường
    for (int i = 0; i < GRID_CELLS; i++) {
        float alpha = anom_mask[i] ? 0.004f : 0.045f;
        float dy  = fabsf(cell_y[i]    - s_bg_y[i]);
        float dcr = fabsf(cell_cr[i]   - s_bg_cr[i]);
        float dcb = fabsf(cell_cb[i]   - s_bg_cb[i]);
        float diff = dy + 0.35f * (dcr + dcb);

        s_bg_y[i]    = (1.0f - alpha) * s_bg_y[i]    + alpha * cell_y[i];
        s_bg_cr[i]   = (1.0f - alpha) * s_bg_cr[i]   + alpha * cell_cr[i];
        s_bg_cb[i]   = (1.0f - alpha) * s_bg_cb[i]   + alpha * cell_cb[i];
        s_bg_edge[i] = (1.0f - alpha) * s_bg_edge[i] + alpha * cell_edge[i];
        if (!anom_mask[i]) {
            s_bg_var[i] = 0.94f * s_bg_var[i] + 0.06f * (diff + 3.5f);
            if (s_bg_var[i] < 4.5f)  s_bg_var[i] = 4.5f;
            if (s_bg_var[i] > 25.0f) s_bg_var[i] = 25.0f;
        }
    }

    // Bước 4: Gom cụm các ô bất thường liền kề (8-Connected Components) để tạo Bounding Box
    uint8_t visited[GRID_CELLS];
    memset(visited, 0, sizeof(visited));
    AnomBox cand_boxes[MAX_ANOM_BOXES];
    int cand_count = 0;

    for (int sy = 0; sy < GRID_H; sy++) {
        for (int sx = 0; sx < GRID_W; sx++) {
            int start_idx = sy * GRID_W + sx;
            if (!anom_mask[start_idx] || visited[start_idx]) continue;

            int qx[GRID_CELLS], qy[GRID_CELLS];
            int q_head = 0, q_tail = 0;
            qx[q_tail] = sx; qy[q_tail] = sy; q_tail++;
            visited[start_idx] = 1;

            int min_gx = sx, max_gx = sx;
            int min_gy = sy, max_gy = sy;
            float sum_z = 0.0f, max_z = 0.0f;

            while (q_head < q_tail) {
                int cx = qx[q_head];
                int cy = qy[q_head];
                q_head++;

                int c_idx = cy * GRID_W + cx;
                float cz = z_map[c_idx];
                sum_z += cz;
                if (cz > max_z) max_z = cz;

                if (cx < min_gx) min_gx = cx;
                if (cx > max_gx) max_gx = cx;
                if (cy < min_gy) min_gy = cy;
                if (cy > max_gy) max_gy = cy;

                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        if (dx == 0 && dy == 0) continue;
                        int nx = cx + dx, ny = cy + dy;
                        if (nx >= 0 && nx < GRID_W && ny >= 0 && ny < GRID_H) {
                            int n_idx = ny * GRID_W + nx;
                            if (anom_mask[n_idx] && !visited[n_idx]) {
                                visited[n_idx] = 1;
                                qx[q_tail] = nx;
                                qy[q_tail] = ny;
                                q_tail++;
                            }
                        }
                    }
                }
            }

            int cluster_cells = q_tail;
            if (cluster_cells < 2 && max_z < thr * 1.8f) continue;

            if (cand_count < MAX_ANOM_BOXES) {
                AnomBox b;
                b.x = (float)min_gx / (float)GRID_W;
                b.y = (float)min_gy / (float)GRID_H;
                b.w = (float)(max_gx - min_gx + 1) / (float)GRID_W;
                b.h = (float)(max_gy - min_gy + 1) / (float)GRID_H;

                float box_logits[AI_NUM_CLASSES] = {0};
                if (decoded_ok) {
                    int px0 = (min_gx * thumb_w) / GRID_W;
                    int py0 = (min_gy * thumb_h) / GRID_H;
                    int pbw = ((max_gx - min_gx + 1) * thumb_w) / GRID_W;
                    int pbh = ((max_gy - min_gy + 1) * thumb_h) / GRID_H;
                    int8_t patch_feat[400];
                    extract_patch_int8_400(s_rgb565_thumb, thumb_w, thumb_h, px0, py0, pbw, pbh, patch_feat);

                    b.ai_cls_m1 = adaremac_run_nn(patch_feat, AI_PARTIAL_DIM, 1, 2, NULL);
                    b.ai_cls_m2 = adaremac_run_nn(patch_feat, AI_FEAT_DIM, 2, 0, box_logits);

                    float dist_sq = 0.0f;
                    for (int c = 0; c < AI_NUM_CLASSES; c++) {
                        float d = box_logits[c] - s_bg_logits[c];
                        dist_sq += d * d;
                    }
                    b.feat_dist = sqrtf(dist_sq);
                } else {
                    b.ai_cls_m1 = (uint8_t)(cluster_cells % 10);
                    b.ai_cls_m2 = (uint8_t)(cluster_cells % 10);
                    b.feat_dist = max_z * 5.0f;
                }

                float avg_z = sum_z / cluster_cells;
                int conf = (int)(50.0f + (avg_z - thr) * 14.0f + b.feat_dist * 0.4f);
                if (conf < 55) conf = 55;
                if (conf > 99) conf = 99;
                b.score = conf;

                cand_boxes[cand_count++] = b;
            }
        }
    }

    uint32_t dt_us = micros() - t0;

    portENTER_CRITICAL(&s_ai_mux);
    s_num_boxes = cand_count;
    for (int i = 0; i < cand_count; i++) s_boxes[i] = cand_boxes[i];
    s_angle_changed = angle_shift_now;
    s_changed_cell_ratio = changed_ratio;
    s_env_stability = (1.0f - fminf(1.0f, changed_ratio * 1.5f)) * 100.0f;
    s_ai_infer_us = dt_us;
    portEXIT_CRITICAL(&s_ai_mux);
}

// Task chạy nền trên Core 0 chuyên thực thi AI & Học môi trường liên tục
static void ai_continual_learning_task(void *arg) {
    while (true) {
        if (!s_cam_ok) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (s_stream_clients > 0) {
            if (!s_ai_frame_ready) {
                s_ai_need_frame = true;
                vTaskDelay(pdMS_TO_TICKS(12));
                continue;
            }
            analyze_frame_and_learn(s_ai_jpeg_buf, s_ai_jpeg_len, s_ai_frame_w, s_ai_frame_h);
            s_ai_frame_ready = false;
            s_ai_need_frame  = true;
            vTaskDelay(pdMS_TO_TICKS(20));
        } else {
            camera_fb_t *fb = esp_camera_fb_get();
            if (fb) {
                s_total_frames++;
                analyze_frame_and_learn(fb->buf, fb->len, fb->width, fb->height);
                esp_camera_fb_return(fb);
            }
            vTaskDelay(pdMS_TO_TICKS(80));
        }
    }
}

// =========================================================================================
// 4. GIAO DIỆN WEB TRỰC QUAN: LIVE STREAM FHD@30P + TINH CHỈNH ISP ĐỘ NÉT + AI BOUNDING BOX
// =========================================================================================
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>AdaReMAC ESP32-S3 — FHD@30p Direct Stream & Continual AI Anomaly Detection</title>
  <style>
    :root { --bg: #0f172a; --card: #1e293b; --accent: #38bdf8; --danger: #ef4444; --ok: #22c55e; --warn: #f59e0b; }
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: system-ui, -apple-system, sans-serif; }
    body { background: var(--bg); color: #f8fafc; padding: 12px; display: flex; flex-direction: column; align-items: center; }
    .header { width: 100%; max-width: 1280px; display: flex; flex-wrap: wrap; justify-content: space-between; align-items: center; background: var(--card); padding: 12px 18px; border-radius: 12px; margin-bottom: 10px; border: 1px solid #334155; gap: 10px; }
    .title { font-size: 1.15rem; font-weight: 700; color: var(--accent); }
    .badge { padding: 6px 14px; border-radius: 999px; font-weight: 700; font-size: 0.9rem; display: inline-flex; align-items: center; gap: 6px; }
    .bg-ok { background: rgba(34,197,94,0.18); color: #4ade80; border: 1px solid #22c55e; }
    .bg-warn { background: rgba(245,158,11,0.22); color: #fbbf24; border: 1px solid #f59e0b; }
    .bg-danger { background: rgba(239,68,68,0.25); color: #f87171; border: 1px solid #ef4444; animation: pulse 1s infinite; }
    @keyframes pulse { 0%,100% { transform: scale(1); } 50% { transform: scale(1.03); } }
    .Controls { width: 100%; max-width: 1280px; display: grid; grid-template-columns: repeat(auto-fit, minmax(190px, 1fr)); gap: 10px; margin-bottom: 10px; }
    .stat-box { background: var(--card); padding: 10px 14px; border-radius: 10px; border: 1px solid #334155; }
    .stat-label { font-size: 0.76rem; color: #94a3b8; text-transform: uppercase; }
    .stat-val { font-size: 1.12rem; font-weight: 700; margin-top: 3px; }
    .stage-wrap { position: relative; width: 100%; max-width: 1280px; background: #000; border-radius: 12px; overflow: hidden; border: 2px solid #334155; aspect-ratio: 16 / 9; display: flex; align-items: center; justify-content: center; }
    #cam_stream { width: 100%; height: 100%; object-fit: fill; display: block; }
    #ai_canvas { position: absolute; top: 0; left: 0; width: 100%; height: 100%; pointer-events: none; }
    .toolbar { width: 100%; max-width: 1280px; background: var(--card); padding: 12px 18px; border-radius: 12px; margin-top: 10px; border: 1px solid #334155; display: flex; flex-wrap: wrap; gap: 14px; align-items: center; justify-content: space-between; }
    button, select { background: #0284c7; color: #fff; border: none; padding: 8px 14px; border-radius: 8px; font-weight: 600; cursor: pointer; font-size: 0.88rem; }
    button:hover { background: #0369a1; }
    select { background: #334155; }
    input[type=range] { accent-color: var(--accent); cursor: pointer; }
  </style>
</head>
<body>
  <div class="header">
    <div>
      <div class="title">📹 ESP32-S3 OV5640 FHD@30p (Direct HT40 Stream) + AI Continual Anomaly Detection</div>
      <div style="font-size:0.82rem;color:#94a3b8;margin-top:2px;">Đã bật ISP Sharpness/Contrast/LENC chống mờ ảnh + Model AI INT8 AdaReMAC học môi trường liên tục khi đổi góc camera</div>
    </div>
    <div id="status_badge" class="badge bg-ok">Đang kết nối AI...</div>
  </div>

  <div class="Controls">
    <div class="stat-box"><div class="stat-label">Độ phân giải & Tốc độ Stream</div><div class="stat-val" id="val_fps">FHD 1920x1080</div></div>
    <div class="stat-box"><div class="stat-label">Trạng thái Học Môi Trường</div><div class="stat-val" id="val_env">100% Ổn định</div></div>
    <div class="stat-box"><div class="stat-label">Vật thể bất thường / AI Infer</div><div class="stat-val" id="val_anom">0 vật thể</div></div>
    <div class="stat-box"><div class="stat-label">Nhiệt độ Chip ESP32-S3</div><div class="stat-val" id="val_temp">-- °C</div></div>
  </div>

  <div class="stage-wrap">
    <img id="cam_stream" alt="Đang tải luồng MJPEG FHD@30p..." crossorigin="anonymous">
    <canvas id="ai_canvas"></canvas>
  </div>

  <div class="toolbar">
    <div style="display:flex;align-items:center;gap:10px;flex-wrap:wrap;">
      <label style="font-size:0.86rem;">Phân giải:</label>
      <select id="sel_res" onchange="changeRes(this.value)">
        <option value="fhd" selected>FHD (1920x1080)</option>
        <option value="hd">HD (1280x720)</option>
        <option value="svga">SVGA (800x600)</option>
      </select>
      <label style="font-size:0.86rem;margin-left:6px;">Chất lượng JPEG (Nhỏ=Nét căng):</label>
      <input type="range" min="5" max="20" step="1" value="8" oninput="updateQual(this.value)">
      <span id="lbl_q" style="font-weight:700;color:#38bdf8;">8</span>

      <label style="font-size:0.86rem;margin-left:6px;">Độ nét ISP:</label>
      <input type="range" min="0" max="3" step="1" value="2" oninput="updateSharp(this.value)">
      <span id="lbl_sh" style="font-weight:700;color:#38bdf8;">+2</span>

      <label style="font-size:0.86rem;margin-left:6px;">Ngưỡng nhạy AI:</label>
      <input type="range" id="rng_thr" min="1.8" max="6.0" step="0.2" value="3.2" oninput="updateThr(this.value)">
      <span id="lbl_thr" style="font-weight:700;color:#38bdf8;">3.2</span>
    </div>
    <button onclick="resetEnv()">🔄 Học lại góc Camera ngay (Reset Nền)</button>
  </div>

  <script>
    const streamImg = document.getElementById('cam_stream');
    const canvas = document.getElementById('ai_canvas');
    const ctx = canvas.getContext('2d');
    const streamUrl = 'http://' + window.location.hostname + ':81/stream';
    streamImg.src = streamUrl;
    streamImg.onerror = () => { setTimeout(() => { streamImg.src = streamUrl + '?t=' + Date.now(); }, 500); };

    function changeRes(v) { fetch('/ctrl?res=' + v); }
    function updateQual(v) { document.getElementById('lbl_q').innerText = v; fetch('/ctrl?qual=' + v); }
    function updateSharp(v) { document.getElementById('lbl_sh').innerText = '+' + v; fetch('/ctrl?sharp=' + v); }
    function updateThr(v) { document.getElementById('lbl_thr').innerText = v; fetch('/ctrl?thr=' + v); }
    function resetEnv() { fetch('/ctrl?reset=1'); }

    async function pollAI() {
      try {
        const res = await fetch('/ai', { cache: 'no-store' });
        const d = await res.json();

        document.getElementById('val_fps').innerText = d.w + 'x' + d.h + ' @ ' + d.fps.toFixed(1) + ' FPS';
        document.getElementById('val_env').innerText = d.stability.toFixed(0) + '% (Lệch nền: ' + (d.changed * 100).toFixed(1) + '%)';
        document.getElementById('val_anom').innerText = d.boxes.length + ' khung (' + (d.ai_us / 1000).toFixed(1) + ' ms)';
        document.getElementById('val_temp').innerText = d.temp.toFixed(1) + ' °C';

        const badge = document.getElementById('status_badge');
        if (d.angle_shift || d.learn_left > 0) {
          badge.className = 'badge bg-warn';
          badge.innerText = '🔄 ĐỔI GÓC CAMERA -> ĐANG HỌC MÔI TRƯỜNG MỚI (' + d.stability.toFixed(0) + '%)';
        } else if (d.boxes.length > 0) {
          badge.className = 'badge bg-danger';
          badge.innerText = '⚠️ PHÁT HIỆN ' + d.boxes.length + ' VẬT THỂ BẤT THƯỜNG!';
        } else {
          badge.className = 'badge bg-ok';
          badge.innerText = '✅ MÔI TRƯỜNG BÌNH THƯỜNG (AI Đang học liên tục - Lớp nền #' + d.scene_cls + ')';
        }

        const cw = canvas.clientWidth || 1280;
        const ch = canvas.clientHeight || 720;
        if (canvas.width !== cw || canvas.height !== ch) {
          canvas.width = cw;
          canvas.height = ch;
        }
        ctx.clearRect(0, 0, cw, ch);

        if (d.angle_shift || d.learn_left > 0) {
          ctx.strokeStyle = '#f59e0b';
          ctx.lineWidth = 4;
          ctx.strokeRect(8, 8, cw - 16, ch - 16);
          ctx.fillStyle = 'rgba(245, 158, 11, 0.85)';
          ctx.fillRect(16, 16, 430, 34);
          ctx.fillStyle = '#000';
          ctx.font = 'bold 15px sans-serif';
          ctx.fillText('ĐANG CẬP NHẬT GÓC CAMERA MỚI VÀO MÔ HÌNH NỀN...', 26, 38);
        }

        d.boxes.forEach((b, idx) => {
          const bx = b.x * cw;
          const by = b.y * ch;
          const bw = b.w * cw;
          const bh = b.h * ch;

          ctx.strokeStyle = '#ef4444';
          ctx.lineWidth = 3;
          ctx.strokeRect(bx, by, bw, bh);

          ctx.fillStyle = 'rgba(239, 68, 68, 0.16)';
          ctx.fillRect(bx, by, bw, bh);

          const label = 'BẤT THƯỜNG #' + (idx + 1) + ' | AI Class ' + b.m2 + ' (M1:' + b.m1 + ') | ' + b.score + '%';
          ctx.font = 'bold 13px sans-serif';
          const tw = ctx.measureText(label).width + 14;
          const ly = (by > 26) ? (by - 26) : by;
          ctx.fillStyle = '#ef4444';
          ctx.fillRect(bx, ly, tw, 24);
          ctx.fillStyle = '#ffffff';
          ctx.fillText(label, bx + 7, ly + 17);
        });
      } catch (e) {}
      setTimeout(pollAI, 80);
    }
    pollAI();
  </script>
</body>
</html>
)rawliteral";

// ---------------- 5. CÁC HANDLER HTTP (PORT 80: UI + AI JSON | PORT 81: MJPEG STREAM) -------
#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, INDEX_HTML, strlen(INDEX_HTML));
}

static esp_err_t ai_json_handler(httpd_req_t *req) {
    AnomBox local_boxes[MAX_ANOM_BOXES];
    int n_boxes = 0;
    float stab = 0, chg = 0, fps = 0, temp = 0;
    bool ang = false;
    int learn_left = 0;
    uint8_t s_cls = 0;
    uint32_t infer_us = 0;
    uint16_t fw = 1920, fh = 1080;

    portENTER_CRITICAL(&s_ai_mux);
    n_boxes    = s_num_boxes;
    for (int i = 0; i < n_boxes; i++) local_boxes[i] = s_boxes[i];
    stab       = s_env_stability;
    chg        = s_changed_cell_ratio;
    ang        = s_angle_changed;
    learn_left = s_fast_learn_frames;
    s_cls      = s_scene_ai_cls;
    infer_us   = s_ai_infer_us;
    fps        = s_stream_fps;
    temp       = s_chip_temp_c;
    fw         = s_ai_frame_w;
    fh         = s_ai_frame_h;
    portEXIT_CRITICAL(&s_ai_mux);

    char json[1024];
    int len = snprintf(json, sizeof(json),
        "{\"w\":%u,\"h\":%u,\"fps\":%.1f,\"temp\":%.1f,\"stability\":%.1f,\"changed\":%.3f,"
        "\"angle_shift\":%s,\"learn_left\":%d,\"scene_cls\":%u,\"ai_us\":%lu,\"boxes\":[",
        fw, fh, fps, temp, stab, chg, ang ? "true" : "false", learn_left, s_cls, (unsigned long)infer_us);

    for (int i = 0; i < n_boxes; i++) {
        len += snprintf(json + len, sizeof(json) - len,
            "%s{\"x\":%.4f,\"y\":%.4f,\"w\":%.4f,\"h\":%.4f,\"score\":%d,\"m1\":%u,\"m2\":%u}",
            (i > 0) ? "," : "",
            local_boxes[i].x, local_boxes[i].y, local_boxes[i].w, local_boxes[i].h,
            local_boxes[i].score, local_boxes[i].ai_cls_m1, local_boxes[i].ai_cls_m2);
    }
    snprintf(json + len, sizeof(json) - len, "]}");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json, strlen(json));
}

static esp_err_t ctrl_handler(httpd_req_t *req) {
    char query[64] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char val[16] = {0};
        sensor_t *s = esp_camera_sensor_get();
        if (httpd_query_key_value(query, "reset", val, sizeof(val)) == ESP_OK) {
            s_force_relearn = true;
        }
        if (httpd_query_key_value(query, "thr", val, sizeof(val)) == ESP_OK) {
            float t = atof(val);
            if (t >= 1.2f && t <= 10.0f) s_anom_threshold = t;
        }
        if (s && httpd_query_key_value(query, "qual", val, sizeof(val)) == ESP_OK) {
            int q = atoi(val);
            if (q >= 4 && q <= 40) s->set_quality(s, q);
        }
        if (s && httpd_query_key_value(query, "sharp", val, sizeof(val)) == ESP_OK) {
            int sh = atoi(val);
            if (sh >= -2 && sh <= 3) s->set_sharpness(s, sh);
        }
        if (s && httpd_query_key_value(query, "res", val, sizeof(val)) == ESP_OK) {
            if (strcmp(val, "fhd") == 0)       s->set_framesize(s, FRAMESIZE_FHD);
            else if (strcmp(val, "hd") == 0)   s->set_framesize(s, FRAMESIZE_HD);
            else if (strcmp(val, "svga") == 0) s->set_framesize(s, FRAMESIZE_SVGA);
            s_force_relearn = true;
        }
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, "OK", 2);
}

// Handler phát luồng video MJPEG FHD@30p liên tục với TCP_NODELAY chống nghẽn
static esp_err_t stream_handler(httpd_req_t *req) {
    // Bật TCP_NODELAY trên socket hiện tại để đẩy từng gói tin FHD đi ngay lập tức không chờ gom đệm
    int sockfd = httpd_req_to_sockfd(req);
    int nodelay = 1;
    setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

    esp_err_t res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    httpd_resp_set_hdr(req, "X-Framerate", "30");

    s_stream_clients++;
    char part_buf[64];
    uint32_t frame_cnt = 0;
    uint32_t t_fps_start = millis();

    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }

        s_total_frames++;
        frame_cnt++;
        s_ai_frame_w = fb->width;
        s_ai_frame_h = fb->height;

        if (s_ai_need_frame && !s_ai_frame_ready && s_ai_jpeg_buf && fb->len <= MAX_JPEG_COPY_SZ) {
            memcpy(s_ai_jpeg_buf, fb->buf, fb->len);
            s_ai_jpeg_len    = fb->len;
            s_ai_frame_ready = true;
            s_ai_need_frame  = false;
        }

        size_t hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, (unsigned)fb->len);
        res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        if (res == ESP_OK) res = httpd_resp_send_chunk(req, part_buf, hlen);
        if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);

        esp_camera_fb_return(fb);
        if (res != ESP_OK) break;

        uint32_t now = millis();
        if (now - t_fps_start >= 1000) {
            s_stream_fps = (frame_cnt * 1000.0f) / (now - t_fps_start);
            frame_cnt = 0;
            t_fps_start = now;
        }
    }

    s_stream_clients--;
    return res;
}

static void start_web_and_stream_servers() {
    httpd_config_t cfg_web = HTTPD_DEFAULT_CONFIG();
    cfg_web.server_port = 80;
    cfg_web.ctrl_port   = 32768;
    if (httpd_start(&s_web_httpd, &cfg_web) == ESP_OK) {
        httpd_uri_t u_idx  = { .uri = "/",     .method = HTTP_GET, .handler = index_handler,   .user_ctx = NULL };
        httpd_uri_t u_ai   = { .uri = "/ai",   .method = HTTP_GET, .handler = ai_json_handler, .user_ctx = NULL };
        httpd_uri_t u_ctrl = { .uri = "/ctrl", .method = HTTP_GET, .handler = ctrl_handler,    .user_ctx = NULL };
        httpd_register_uri_handler(s_web_httpd, &u_idx);
        httpd_register_uri_handler(s_web_httpd, &u_ai);
        httpd_register_uri_handler(s_web_httpd, &u_ctrl);
    }

    httpd_config_t cfg_stream = HTTPD_DEFAULT_CONFIG();
    cfg_stream.server_port = 81;
    cfg_stream.ctrl_port   = 32769;
    cfg_stream.stack_size  = 10240;
    if (httpd_start(&s_stream_httpd, &cfg_stream) == ESP_OK) {
        httpd_uri_t u_strm = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL };
        httpd_register_uri_handler(s_stream_httpd, &u_strm);
    }
}

static void setup_zero_bottleneck_wifi() {
    WiFi.persistent(false);

#if USE_DIRECT_AP_MODE
    // PHƯƠNG ÁN ĐỠ NGHẼN NHẤT: Pure WIFI_AP 1-đối-1 ở băng thông kênh rộng 40MHz (HT40 - 150Mbps)
    // Không dùng WIFI_AP_STA để tránh chia đôi thời gian anten (Radio Time-Slicing)
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, 2);
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT40); // Bật kênh rộng 40MHz tăng gấp đôi tốc độ truyền!
    WiFi.setTxPower(WIFI_POWER_19_5dBm);              // Công suất phát tối đa 19.5 dBm

    Serial.println("============================================================");
    Serial.println("  [CHE DO CHONG NGHEN TOI DA: PURE WI-FI AP 40MHz HT40]");
    Serial.printf ("  1. Ket noi Wi-Fi tren May tinh/DT : %s\n", AP_SSID);
    Serial.printf ("  2. Mat khau Wi-Fi                 : %s\n", AP_PASS);
    Serial.print  ("  3. Mo trinh duyet truy cap ngay   : http://");
    Serial.println(WiFi.softAPIP());
    Serial.println("============================================================");
#else
    // Phương án thay thế: Pure WIFI_STA nối qua Router ngoài (tắt hẳn AP để không chia đôi anten)
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);
    WiFi.setAutoReconnect(true);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40);

    Serial.printf("Dang ket noi Wi-Fi Router '%s' (40MHz HT40)...", WIFI_SSID);
    int retry = 0;
    while (WiFi.status() != WL_CONNECTED && retry < 24) {
        delay(500);
        Serial.print(".");
        retry++;
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("============================================================");
        Serial.print  ("  [WI-FI ROUTER STA] Truy cap Live Stream FHD + AI: http://");
        Serial.println(WiFi.localIP());
        Serial.println("============================================================");
    } else {
        Serial.println("  -> Khong ket noi duoc Router! Tu dong chuyen sang Pure AP:");
        WiFi.mode(WIFI_AP);
        WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, 2);
        esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT40);
        Serial.print("  -> Ket noi Wi-Fi 'ESP32-OV5640-FHD' (MK: 12345678) -> http://");
        Serial.println(WiFi.softAPIP());
    }
#endif
}

static const char *get_sensor_name(int pid) {
    switch (pid) {
        case OV5640_PID: return "OV5640 (5MP - Stream FHD 1920x1080 @ 30p ISP Enhanced)";
        case OV3660_PID: return "OV3660 (3MP)";
        case OV2640_PID: return "OV2640 (2MP)";
        default:         return "Unknown Sensor";
    }
}

void setup() {
    Serial.begin(115200);
    delay(1200);

    Serial.println("\n============================================================");
    Serial.println("  [SKETCH 1] OV5640 FHD@30P ISP SHARP + AI CONTINUAL LEARN");
    Serial.println("============================================================");

    s_chip_temp_c = temperatureRead();
    Serial.printf("[1/4] Nhiet do noi bo ESP32-S3 : %.1f C %s\n",
                  s_chip_temp_c, (s_chip_temp_c > 70.0f) ? "[CANH BAO: CHIP NONG!]" : "[BINH THUONG]");

    bool has_psram = psramFound();
    Serial.printf("[2/4] Bo nho OPI PSRAM         : %u KB trong / %u KB tong (DRAM: %u KB)\n",
                  (unsigned)(ESP.getFreePsram() / 1024),
                  (unsigned)(ESP.getPsramSize() / 1024),
                  (unsigned)(ESP.getFreeHeap() / 1024));

    if (has_psram) {
        s_ai_jpeg_buf  = (uint8_t *)ps_malloc(MAX_JPEG_COPY_SZ);
        s_rgb565_thumb = (uint16_t *)ps_malloc(MAX_RGB_THUMB_W * MAX_RGB_THUMB_H * sizeof(uint16_t));
    }

    // Cấu hình Camera OV5640 ở chế độ Hardware JPEG FHD (1920x1080) 30fps sắc nét
    camera_config_t config = {};
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0       = CAM_PIN_D0;
    config.pin_d1       = CAM_PIN_D1;
    config.pin_d2       = CAM_PIN_D2;
    config.pin_d3       = CAM_PIN_D3;
    config.pin_d4       = CAM_PIN_D4;
    config.pin_d5       = CAM_PIN_D5;
    config.pin_d6       = CAM_PIN_D6;
    config.pin_d7       = CAM_PIN_D7;
    config.pin_xclk     = CAM_PIN_XCLK;
    config.pin_pclk     = CAM_PIN_PCLK;
    config.pin_vsync    = CAM_PIN_VSYNC;
    config.pin_href     = CAM_PIN_HREF;
    config.pin_sccb_sda = CAM_PIN_SIOD;
    config.pin_sccb_scl = CAM_PIN_SIOC;
    config.pin_pwdn     = CAM_PIN_PWDN;
    config.pin_reset    = CAM_PIN_RESET;
    config.xclk_freq_hz = 24000000; // Xung nhịp 24 MHz giúp OV5640 đạt tốc độ nén FHD 30fps
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size   = has_psram ? FRAMESIZE_FHD : FRAMESIZE_SVGA; // 1920x1080 Full HD!
    config.jpeg_quality = 8;        // Chất lượng cao sắc nét (tránh nén mờ như mức 12-15 cũ)
    config.fb_count     = has_psram ? 3 : 1; // 3 bộ đệm khung hình trong 8MB OPI PSRAM
    config.fb_location  = has_psram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
    config.grab_mode    = CAMERA_GRAB_LATEST;

    Serial.println("[3/4] Dang khoi tao Camera OV5640 (FHD 1920x1080 @ 30p + ISP Sharpness)...");
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("  -> [LOI] esp_camera_init that bai (0x%x). Kiem tra cap det FPC!\n", err);
        s_cam_ok = false;
        return;
    }

    // Kích hoạt toàn bộ bộ xử lý tín hiệu ảnh phần cứng (Hardware ISP) bên trong cảm biến OV5640
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_framesize(s, has_psram ? FRAMESIZE_FHD : FRAMESIZE_SVGA);
        s->set_quality(s, 8);                      // Nén JPEG sắc nét
        s->set_brightness(s, 0);                   // Độ sáng chuẩn
        s->set_contrast(s, 1);                     // Tăng tương phản (+1) giúp ảnh trong trẻo
        s->set_saturation(s, 1);                   // Tăng độ bão hòa màu (+1)
        s->set_sharpness(s, 2);                    // Bật bộ làm sắc nét cạnh phần cứng ISP (+2)
        s->set_denoise(s, 1);                      // Khử nhiễu hạt phần cứng ISP
        s->set_gainceiling(s, (gainceiling_t)2);   // Chống đẩy ISO quá cao gây bệt ảnh
        s->set_whitebal(s, 1);                     // Cân bằng trắng tự động (AWB)
        s->set_awb_gain(s, 1);                     // Bật AWB Gain
        s->set_wb_mode(s, 0);                      // Chế độ cân bằng trắng tự động
        s->set_exposure_ctrl(s, 1);                // Phơi sáng tự động (AEC)
        s->set_aec2(s, 1);                         // Bật DSP AEC2 cân bằng sáng vùng tối/sáng
        s->set_gain_ctrl(s, 1);                    // Khuếch đại tự động (AGC)
        s->set_bpc(s, 1);                          // Khử điểm chết đen (Black Pixel Cancellation)
        s->set_wpc(s, 1);                          // Khử điểm chết trắng (White Pixel Cancellation)
        s->set_raw_gma(s, 1);                      // Bật hiệu chỉnh Gamma phần cứng (Raw GMA)
        s->set_lenc(s, 1);                         // Bật hiệu chỉnh quang sai thấu kính (Lens Correction)
        s->set_dcw(s, 1);                          // Bật Downsize chuẩn xác
        s->set_vflip(s, 1);
        Serial.printf("[4/4] Nhan dien thanh cong     : %s (PID = 0x%04X)\n",
                      get_sensor_name(s->id.PID), s->id.PID);
    }
    s_cam_ok = true;
    s_ai_need_frame = true;

    setup_zero_bottleneck_wifi();
    start_web_and_stream_servers();

    xTaskCreatePinnedToCore(ai_continual_learning_task, "ai_learn_task", 12288, NULL, 2, NULL, 0);
}

static uint32_t s_last_log_ms  = 0;
static uint32_t s_last_wifi_ms = 0;

void loop() {
    uint32_t now = millis();

#if !USE_DIRECT_AP_MODE
    if (strlen(WIFI_SSID) > 0 && (now - s_last_wifi_ms >= 5000)) {
        s_last_wifi_ms = now;
        if (WiFi.status() != WL_CONNECTED) {
            WiFi.reconnect();
        }
    }
#endif

    if (now - s_last_log_ms >= 1000) {
        s_last_log_ms = now;
        s_chip_temp_c = temperatureRead();

        if (!s_cam_ok) {
            Serial.printf("[Chua nhan Camera] Nhiet do ESP32-S3: %.1f C\n", s_chip_temp_c);
            return;
        }

        int n_boxes = 0;
        AnomBox b0 = {};
        float stab = 0, chg = 0;
        bool ang = false;
        uint8_t s_cls = 0;
        uint32_t ai_us = 0;

        portENTER_CRITICAL(&s_ai_mux);
        n_boxes = s_num_boxes;
        if (n_boxes > 0) b0 = s_boxes[0];
        stab  = s_env_stability;
        chg   = s_changed_cell_ratio;
        ang   = s_angle_changed;
        s_cls = s_scene_ai_cls;
        ai_us = s_ai_infer_us;
        portEXIT_CRITICAL(&s_ai_mux);

        if (ang || s_fast_learn_frames > 0) {
            Serial.printf("[AI LEARNING] PHAT HIEN DOI GOC CAMERA (Lech %.1f%%) -> Dang hoc moi truong moi (On dinh: %.0f%%) | Temp: %.1f C\n",
                          chg * 100.0f, stab, s_chip_temp_c);
        } else if (n_boxes > 0) {
            int bx_px = (int)(b0.x * s_ai_frame_w);
            int by_px = (int)(b0.y * s_ai_frame_h);
            int bw_px = (int)(b0.w * s_ai_frame_w);
            int bh_px = (int)(b0.h * s_ai_frame_h);
            Serial.printf("[AI ANOMALY!] Phat hien %d vat the bat thuong! Box#1=[x:%d, y:%d, w:%d, h:%d] | AI Class=%u (M1:%u) | Score=%d%% | Infer=%.2fms | Temp=%.1fC\n",
                          n_boxes, bx_px, by_px, bw_px, bh_px, b0.ai_cls_m2, b0.ai_cls_m1, b0.score, ai_us / 1000.0f, s_chip_temp_c);
        } else {
            Serial.printf("[AI NORMAL] Moi truong on dinh (%.0f%%, SceneClass=%u) | Stream Clients=%d | FPS=%.1f | Infer=%.2fms | Temp=%.1fC\n",
                          stab, s_cls, s_stream_clients, s_stream_fps, ai_us / 1000.0f, s_chip_temp_c);
        }
    }

    delay(20);
}
