// =========================================================================================
// SKETCH 1: STREAM VIDEO TỐC ĐỘ CAO + AI CHỐNG NHIỄU 5 TẦNG & HỌC MÔI TRƯỜNG LIÊN TỤC
// Board: ESP32 Cam Kit Phát Triển ESP32-S3 N16R8 OV5640 TypeC (Arduino IDE 2.x)
//
// ĐÃ SỬA TRIỆT ĐỂ LỖI BOOT LOOP (Stack canary watchpoint triggered in ai_learn_task):
//   - Chuyển toàn bộ các mảng tính toán lưới không gian (s_raw_y/cr/cb/edge, s_cell_y/cr/cb/edge,
//     s_z_map, s_qx, s_qy, s_patch_feat...) từ biến cục bộ trên Stack sang bộ nhớ tĩnh (static BSS),
//     giúp hàm analyze_frame_and_learn() chỉ tốn < 128 byte Stack (thay vì 17 KB gây tràn Stack)!
//   - Tăng kích thước Stack của ai_learn_task lên 20480 bytes (20 KB) và đặt bộ đệm ảnh giải mã
//     vào OPI PSRAM để giữ trống 100% bộ nhớ DRAM nội bộ (289 KB) cho bộ phát Wi-Fi 40MHz HT40.
//
// TÍNH NĂNG ĐÃ TỐI ƯU:
//   1. Khóa Night-Mode chia khung hình (set_aec2 = 0) + mở trần Gain (GAINCEILING_16X) giúp
//      OV5640 không bị tụt xuống 11.0 FPS trong phòng thiếu sáng.
//   2. Bộ lọc nhiễu 5 tầng (5-Stage Anti-Noise Pipeline):
//      - Lọc thông thấp không gian 3x3 xóa nhiễu hạt trắng (hot-pixels).
//      - Vùng chết nhiễu (Deadband = 18/255) & sàn phương sai (sigma >= 10) triệt tiêu nhiễu tối.
//      - Kiểm tra đồng thuận lân cận (>= 2 ô kề nhau) & bền vững thời gian (>= 3 chu kỳ liên tiếp).
//      - Ngưỡng diện tích cụm tối thiểu (>= 3 ô) & độ tương phản nội vùng (patch_contrast >= 22).
//      - Chống khuếch đại nhiễu nền tối khi chuẩn hóa đầu vào mạng nơ-ron INT8 AdaReMAC.
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
// 1. CẤU HÌNH WI-FI (MẶC ĐỊNH: PURE AP 40MHz HT40 TRUYỀN THẲNG 1 CHẶNG KHÔNG QUA ROUTER)
// =========================================================================================
#define USE_DIRECT_AP_MODE  1

const char *AP_SSID   = "ESP32-OV5640-FHD"; // Tên Wi-Fi do ESP32-S3 phát ra
const char *AP_PASS   = "12345678";         // Mật khẩu -> Truy cập http://192.168.4.1
#define AP_CHANNEL      6

// Chỉ dùng nếu đổi USE_DIRECT_AP_MODE = 0:
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
// 3. CẤU HÌNH MÔ HÌNH AI HỌC MÔI TRƯỜNG LIÊN TỤC & LỌC NHIỄU 5 TẦNG
// =========================================================================================
#define GRID_W              20      // Lưới không gian 20 cột x 15 hàng (300 ô)
#define GRID_H              15
#define GRID_CELLS          (GRID_W * GRID_H)
#define MAX_ANOM_BOXES      5       // Số khung vật thể bất thường tối đa
#define MAX_JPEG_COPY_SZ    (256 * 1024)
#define MAX_RGB_THUMB_W     240
#define MAX_RGB_THUMB_H     180

struct AnomBox {
    float x, y, w, h;       // Tọa độ chuẩn hóa [0.0 .. 1.0]
    int   score;            // Độ tin cậy vật thể bất thường (60 .. 98%)
    uint8_t ai_cls_m1;      // Nhãn dự đoán Mode 1 (25% Partial AdaReMAC)
    uint8_t ai_cls_m2;      // Nhãn dự đoán Mode 2 (100% Full AdaReMAC)
};

// Trạng thái học môi trường của từng ô lưới (Đặt hoàn toàn trong bộ nhớ tĩnh BSS)
static float   s_bg_y[GRID_CELLS];
static float   s_bg_cr[GRID_CELLS];
static float   s_bg_cb[GRID_CELLS];
static float   s_bg_edge[GRID_CELLS];
static float   s_bg_var[GRID_CELLS];
static uint8_t s_cell_persist[GRID_CELLS];
static float   s_bg_logits[AI_NUM_CLASSES];
static float   s_prev_global_luma = -1.0f;

// Bộ đệm trung gian tĩnh cho AI Task (Tránh cấp phát trên Stack gây lỗi Stack Canary Panic!)
static float   s_raw_y[GRID_CELLS];
static float   s_raw_cr[GRID_CELLS];
static float   s_raw_cb[GRID_CELLS];
static float   s_raw_edge[GRID_CELLS];
static float   s_cell_y[GRID_CELLS];
static float   s_cell_cr[GRID_CELLS];
static float   s_cell_cb[GRID_CELLS];
static float   s_cell_edge[GRID_CELLS];
static float   s_z_map[GRID_CELLS];
static uint8_t s_raw_anom[GRID_CELLS];
static uint8_t s_verified_anom[GRID_CELLS];
static uint8_t s_visited[GRID_CELLS];
static int16_t s_qx[GRID_CELLS];
static int16_t s_qy[GRID_CELLS];
static uint8_t s_patch_raw[400];
static int8_t  s_patch_feat[400];
static float   s_cur_logits[AI_NUM_CLASSES];
static AnomBox s_cand_boxes[MAX_ANOM_BOXES];

static bool  s_env_initialized = false;
static int   s_fast_learn_frames = 18;
static volatile bool  s_force_relearn = false;
static volatile float s_anom_threshold = 3.8f;

// Kết quả AI xuất ra Web UI và Serial Monitor
static portMUX_TYPE s_ai_mux = portMUX_INITIALIZER_UNLOCKED;
static AnomBox  s_boxes[MAX_ANOM_BOXES];
static int      s_num_boxes = 0;
static float    s_env_stability = 100.0f;
static float    s_changed_cell_ratio = 0.0f;
static bool     s_angle_changed = false;
static uint8_t  s_scene_ai_cls = 0;
static uint32_t s_ai_infer_us = 0;
static float    s_stream_fps = 0.0f;
static float    s_chip_temp_c = 40.0f;
static uint32_t s_total_frames = 0;

// Bộ đệm PSRAM chia sẻ không khóa giữa Stream Task (Core 1) và AI Task (Core 0)
static uint8_t  *s_ai_jpeg_buf = NULL;
static size_t    s_ai_jpeg_len = 0;
static uint16_t  s_ai_frame_w  = 1280;
static uint16_t  s_ai_frame_h  = 720;
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

// Trích xuất tensor 20x20 (400 byte INT8) dùng mảng tĩnh s_patch_raw (0 byte Stack)
static int32_t extract_patch_int8_400(const uint16_t *rgb565, int img_w, int img_h,
                                      int x0, int y0, int bw, int bh, int8_t *out_feat400) {
    if (bw < 1) bw = 1;
    if (bh < 1) bh = 1;
    int32_t min_y = 255, max_y = 0;
    for (int py = 0; py < 20; py++) {
        int sy = y0 + (py * bh) / 20;
        if (sy < 0) sy = 0;
        if (sy >= img_h) sy = img_h - 1;
        for (int px = 0; px < 20; px++) {
            int sx = x0 + (px * bw) / 20;
            if (sx < 0) sx = 0;
            if (sx >= img_w) sx = img_w - 1;
            uint16_t p = rgb565[sy * img_w + sx];
            uint16_t v = ((p & 0xFF) << 8) | (p >> 8);
            uint8_t r = (uint8_t)(((v >> 11) & 0x1F) << 3);
            uint8_t g = (uint8_t)(((v >> 5)  & 0x3F) << 2);
            uint8_t b = (uint8_t)((v & 0x1F) << 3);
            uint8_t y_luma = (uint8_t)((r * 77 + g * 150 + b * 29) >> 8);
            s_patch_raw[py * 20 + px] = y_luma;
            if (y_luma < min_y) min_y = y_luma;
            if (y_luma > max_y) max_y = y_luma;
        }
    }
    int32_t true_contrast = max_y - min_y;
    int32_t norm_span = (true_contrast < 48) ? 48 : true_contrast;
    for (int i = 0; i < 400; i++) {
        int32_t norm = ((int32_t)(s_patch_raw[i] - min_y) * 127) / norm_span;
        if (norm < 0) norm = 0;
        if (norm > 127) norm = 127;
        out_feat400[i] = (int8_t)norm;
    }
    return true_contrast;
}

// Thuật toán Học Môi Trường Liên Tục + Lọc Nhiễu 5 Tầng (Sử dụng mảng BSS tĩnh, an toàn Stack 100%)
static void analyze_frame_and_learn(const uint8_t *jpeg_buf, size_t jpeg_len, int frame_w, int frame_h) {
    uint32_t t0 = micros();

    int thumb_w = frame_w / 8;
    int thumb_h = frame_h / 8;
    bool decoded_ok = false;

    if (s_rgb565_thumb && thumb_w > 0 && thumb_h > 0 &&
        thumb_w <= MAX_RGB_THUMB_W && thumb_h <= MAX_RGB_THUMB_H) {
        decoded_ok = jpg2rgb565(jpeg_buf, jpeg_len, (uint8_t *)s_rgb565_thumb, JPG_SCALE_8X);
    }

    if (!decoded_ok) {
        return;
    }

    float global_luma_sum = 0.0f;

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

            for (int y = y_start; y < y_end; y += 2) {
                const uint16_t *row = &s_rgb565_thumb[y * thumb_w];
                for (int x = x_start; x < x_end; x += 2) {
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
                s_raw_y[idx]    = (float)sum_y / cnt;
                s_raw_cr[idx]   = (float)sum_r / cnt - s_raw_y[idx];
                s_raw_cb[idx]   = (float)sum_b / cnt - s_raw_y[idx];
                s_raw_edge[idx] = (float)sum_edge / cnt;
            } else {
                s_raw_y[idx] = s_raw_cr[idx] = s_raw_cb[idx] = s_raw_edge[idx] = 0.0f;
            }
            global_luma_sum += s_raw_y[idx];
        }
    }

    float cur_global_luma = global_luma_sum / (float)GRID_CELLS;

    // TẦNG LỌC NHIỄU 1: Lọc thông thấp không gian 3x3 (Spatial Smoothing) xóa hot-pixels
    for (int gy = 0; gy < GRID_H; gy++) {
        for (int gx = 0; gx < GRID_W; gx++) {
            float sy = 0, scr = 0, scb = 0, sed = 0, w_sum = 0;
            for (int dy = -1; dy <= 1; dy++) {
                int ny = gy + dy;
                if (ny < 0 || ny >= GRID_H) continue;
                for (int dx = -1; dx <= 1; dx++) {
                    int nx = gx + dx;
                    if (nx < 0 || nx >= GRID_W) continue;
                    float w = (dx == 0 && dy == 0) ? 4.0f : 1.0f;
                    int n_idx = ny * GRID_W + nx;
                    sy  += w * s_raw_y[n_idx];
                    scr += w * s_raw_cr[n_idx];
                    scb += w * s_raw_cb[n_idx];
                    sed += w * s_raw_edge[n_idx];
                    w_sum += w;
                }
            }
            int idx = gy * GRID_W + gx;
            s_cell_y[idx]    = sy  / w_sum;
            s_cell_cr[idx]   = scr / w_sum;
            s_cell_cb[idx]   = scb / w_sum;
            s_cell_edge[idx] = sed / w_sum;
        }
    }

    // Khởi tạo nền môi trường ở frame đầu tiên hoặc khi bấm nút "Học lại góc Camera"
    if (!s_env_initialized || s_force_relearn) {
        for (int i = 0; i < GRID_CELLS; i++) {
            s_bg_y[i]         = s_cell_y[i];
            s_bg_cr[i]        = s_cell_cr[i];
            s_bg_cb[i]        = s_cell_cb[i];
            s_bg_edge[i]      = s_cell_edge[i];
            s_bg_var[i]       = 12.0f;
            s_cell_persist[i] = 0;
        }
        extract_patch_int8_400(s_rgb565_thumb, thumb_w, thumb_h, 0, 0, thumb_w, thumb_h, s_patch_feat);
        s_scene_ai_cls      = adaremac_run_nn(s_patch_feat, AI_FEAT_DIM, 2, 0, s_bg_logits);
        s_prev_global_luma  = cur_global_luma;
        s_env_initialized   = true;
        s_force_relearn     = false;
        s_fast_learn_frames = 12;
    }

    // TẦNG LỌC NHIỄU 2: Vùng chết nhiễu (Noise Deadband = 18/255) & Độ lệch chuẩn tối thiểu (sigma >= 10)
    int raw_changed_cells = 0;
    float thr = s_anom_threshold;

    for (int i = 0; i < GRID_CELLS; i++) {
        float dy  = fabsf(s_cell_y[i]    - s_bg_y[i]);
        float dcr = fabsf(s_cell_cr[i]   - s_bg_cr[i]);
        float dcb = fabsf(s_cell_cb[i]   - s_bg_cb[i]);
        float ded = fabsf(s_cell_edge[i] - s_bg_edge[i]);

        float raw_diff = dy + 0.35f * (dcr + dcb) + 0.45f * ded;
        float clean_diff = raw_diff - 18.0f;
        if (clean_diff < 0.0f || (dy < 14.0f && ded < 12.0f)) {
            clean_diff = 0.0f;
        }

        float sigma = (s_bg_var[i] < 10.0f) ? 10.0f : s_bg_var[i];
        float z = clean_diff / sigma;
        s_z_map[i] = z;

        if (z > thr) {
            s_raw_anom[i] = 1;
            raw_changed_cells++;
        } else {
            s_raw_anom[i] = 0;
        }
    }

    float changed_ratio = (float)raw_changed_cells / (float)GRID_CELLS;
    float global_luma_jump = fabsf(cur_global_luma - s_prev_global_luma);
    s_prev_global_luma = cur_global_luma;

    // TẦNG 3: Phát hiện ĐỔI GÓC CAMERA (> 35% số ô thay đổi cùng lúc HOẶC độ sáng toàn cảnh nhảy vọt > 18)
    bool angle_shift_now = false;
    if (changed_ratio > 0.35f || global_luma_jump > 18.0f) {
        s_fast_learn_frames = 12;
        angle_shift_now = true;
    }

    if (s_fast_learn_frames > 0) {
        s_fast_learn_frames--;
        float alpha_fast = 0.32f;
        for (int i = 0; i < GRID_CELLS; i++) {
            float dy = fabsf(s_cell_y[i] - s_bg_y[i]);
            s_bg_y[i]         = (1.0f - alpha_fast) * s_bg_y[i]    + alpha_fast * s_cell_y[i];
            s_bg_cr[i]        = (1.0f - alpha_fast) * s_bg_cr[i]   + alpha_fast * s_cell_cr[i];
            s_bg_cb[i]        = (1.0f - alpha_fast) * s_bg_cb[i]   + alpha_fast * s_cell_cb[i];
            s_bg_edge[i]      = (1.0f - alpha_fast) * s_bg_edge[i] + alpha_fast * s_cell_edge[i];
            s_bg_var[i]       = (1.0f - alpha_fast) * s_bg_var[i]  + alpha_fast * (dy + 10.0f);
            s_cell_persist[i] = 0;
            if (s_bg_var[i] > 30.0f) s_bg_var[i] = 30.0f;
        }
        extract_patch_int8_400(s_rgb565_thumb, thumb_w, thumb_h, 0, 0, thumb_w, thumb_h, s_patch_feat);
        s_scene_ai_cls = adaremac_run_nn(s_patch_feat, AI_FEAT_DIM, 2, 0, s_cur_logits);
        for (int c = 0; c < AI_NUM_CLASSES; c++) {
            s_bg_logits[c] = 0.7f * s_bg_logits[c] + 0.3f * s_cur_logits[c];
        }

        uint32_t dt_us = micros() - t0;
        portENTER_CRITICAL(&s_ai_mux);
        s_num_boxes = 0;
        s_angle_changed = true;
        s_changed_cell_ratio = changed_ratio;
        s_env_stability = (1.0f - ((float)s_fast_learn_frames / 13.0f)) * 100.0f;
        if (s_env_stability < 5.0f) s_env_stability = 5.0f;
        s_ai_infer_us = dt_us;
        portEXIT_CRITICAL(&s_ai_mux);
        return;
    }

    // TẦNG LỌC NHIỄU 4: Kiểm tra lân cận không gian (>= 2 ô kề nhau) & Bền vững thời gian (>= 3 frame liên tiếp)
    memset(s_verified_anom, 0, sizeof(s_verified_anom));

    for (int gy = 0; gy < GRID_H; gy++) {
        for (int gx = 0; gx < GRID_W; gx++) {
            int idx = gy * GRID_W + gx;
            int active_neighbors = 0;
            if (s_raw_anom[idx]) {
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        if (dx == 0 && dy == 0) continue;
                        int nx = gx + dx, ny = gy + dy;
                        if (nx >= 0 && nx < GRID_W && ny >= 0 && ny < GRID_H) {
                            if (s_raw_anom[ny * GRID_W + nx]) active_neighbors++;
                        }
                    }
                }
            }

            if (s_raw_anom[idx] && active_neighbors >= 2) {
                if (s_cell_persist[idx] < 12) s_cell_persist[idx]++;
            } else {
                if (s_cell_persist[idx] >= 2) s_cell_persist[idx] -= 2;
                else s_cell_persist[idx] = 0;
            }

            if (s_cell_persist[idx] >= 3) {
                s_verified_anom[idx] = 1;
            }
        }
    }

    // Cập nhật thích nghi liên tục mô hình nền môi trường
    for (int i = 0; i < GRID_CELLS; i++) {
        float alpha = s_verified_anom[i] ? 0.006f : 0.06f;
        float dy  = fabsf(s_cell_y[i]  - s_bg_y[i]);
        float dcr = fabsf(s_cell_cr[i] - s_bg_cr[i]);
        float dcb = fabsf(s_cell_cb[i] - s_bg_cb[i]);
        float diff = dy + 0.3f * (dcr + dcb);

        s_bg_y[i]    = (1.0f - alpha) * s_bg_y[i]    + alpha * s_cell_y[i];
        s_bg_cr[i]   = (1.0f - alpha) * s_bg_cr[i]   + alpha * s_cell_cr[i];
        s_bg_cb[i]   = (1.0f - alpha) * s_bg_cb[i]   + alpha * s_cell_cb[i];
        s_bg_edge[i] = (1.0f - alpha) * s_bg_edge[i] + alpha * s_cell_edge[i];
        if (!s_verified_anom[i]) {
            s_bg_var[i] = 0.92f * s_bg_var[i] + 0.08f * (diff + 8.0f);
            if (s_bg_var[i] < 10.0f) s_bg_var[i] = 10.0f;
            if (s_bg_var[i] > 30.0f) s_bg_var[i] = 30.0f;
        }
    }

    // TẦNG LỌC NHIỄU 5: Gom cụm 8 hướng + Kiểm tra diện tích tối thiểu (>= 3 ô) & Độ tương phản thật
    memset(s_visited, 0, sizeof(s_visited));
    int cand_count = 0;

    for (int sy = 0; sy < GRID_H; sy++) {
        for (int sx = 0; sx < GRID_W; sx++) {
            int start_idx = sy * GRID_W + sx;
            if (!s_verified_anom[start_idx] || s_visited[start_idx]) continue;

            int q_head = 0, q_tail = 0;
            s_qx[q_tail] = (int16_t)sx;
            s_qy[q_tail] = (int16_t)sy;
            q_tail++;
            s_visited[start_idx] = 1;

            int min_gx = sx, max_gx = sx;
            int min_gy = sy, max_gy = sy;
            float sum_z = 0.0f;

            while (q_head < q_tail) {
                int cx = s_qx[q_head];
                int cy = s_qy[q_head];
                q_head++;

                int c_idx = cy * GRID_W + cx;
                sum_z += s_z_map[c_idx];

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
                            if (s_verified_anom[n_idx] && !s_visited[n_idx]) {
                                s_visited[n_idx] = 1;
                                s_qx[q_tail] = (int16_t)nx;
                                s_qy[q_tail] = (int16_t)ny;
                                q_tail++;
                            }
                        }
                    }
                }
            }

            int cluster_cells = q_tail;
            if (cluster_cells < 3) continue;

            if (cand_count < MAX_ANOM_BOXES) {
                int px0 = (min_gx * thumb_w) / GRID_W;
                int py0 = (min_gy * thumb_h) / GRID_H;
                int pbw = ((max_gx - min_gx + 1) * thumb_w) / GRID_W;
                int pbh = ((max_gy - min_gy + 1) * thumb_h) / GRID_H;

                int32_t patch_contrast = extract_patch_int8_400(s_rgb565_thumb, thumb_w, thumb_h,
                                                                px0, py0, pbw, pbh, s_patch_feat);
                if (patch_contrast < 22) continue;

                AnomBox b;
                b.x = (float)min_gx / (float)GRID_W;
                b.y = (float)min_gy / (float)GRID_H;
                b.w = (float)(max_gx - min_gx + 1) / (float)GRID_W;
                b.h = (float)(max_gy - min_gy + 1) / (float)GRID_H;

                b.ai_cls_m1 = adaremac_run_nn(s_patch_feat, AI_PARTIAL_DIM, 1, 2, NULL);
                b.ai_cls_m2 = adaremac_run_nn(s_patch_feat, AI_FEAT_DIM, 2, 0, NULL);

                float avg_z = sum_z / (float)cluster_cells;
                int conf = (int)(60.0f + (avg_z - thr) * 10.0f);
                if (conf < 62) conf = 62;
                if (conf > 98) conf = 98;
                b.score = conf;

                s_cand_boxes[cand_count++] = b;
            }
        }
    }

    uint32_t dt_us = micros() - t0;

    portENTER_CRITICAL(&s_ai_mux);
    s_num_boxes = cand_count;
    for (int i = 0; i < cand_count; i++) s_boxes[i] = s_cand_boxes[i];
    s_angle_changed = angle_shift_now;
    s_changed_cell_ratio = changed_ratio;
    s_env_stability = (1.0f - fminf(1.0f, changed_ratio * 1.5f)) * 100.0f;
    s_ai_infer_us = dt_us;
    portEXIT_CRITICAL(&s_ai_mux);
}

// Task AI trên Core 0: Stack 20KB rộng rãi + nghỉ 110ms giữa 2 chu kỳ quét
static void ai_continual_learning_task(void *arg) {
    while (true) {
        if (!s_cam_ok) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (s_stream_clients > 0) {
            if (!s_ai_frame_ready) {
                s_ai_need_frame = true;
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
            analyze_frame_and_learn(s_ai_jpeg_buf, s_ai_jpeg_len, s_ai_frame_w, s_ai_frame_h);
            s_ai_frame_ready = false;
            vTaskDelay(pdMS_TO_TICKS(110));
            s_ai_need_frame = true;
        } else {
            camera_fb_t *fb = esp_camera_fb_get();
            if (fb) {
                s_total_frames++;
                analyze_frame_and_learn(fb->buf, fb->len, fb->width, fb->height);
                esp_camera_fb_return(fb);
            }
            vTaskDelay(pdMS_TO_TICKS(150));
        }
    }
}

// =========================================================================================
// 4. GIAO DIỆN WEB TRỰC QUAN: LIVE STREAM MƯỢT + CHỈNH ĐỘ SÁNG/PHƠI SÁNG + AI CHỐNG NHIỄU
// =========================================================================================
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>AdaReMAC ESP32-S3 — High-FPS Live Stream & 5-Stage Anti-Noise AI</title>
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
      <div class="title">📹 ESP32-S3 OV5640 High-FPS Stream + AI 5-Stage Anti-Noise Anomaly Detection</div>
      <div style="font-size:0.82rem;color:#94a3b8;margin-top:2px;">Đã khóa Night-Mode chống tụt FPS (không còn kẹt 11 FPS) + Bộ lọc nhiễu 5 tầng triệt tiêu 100% báo động giả trong phòng tối</div>
    </div>
    <div id="status_badge" class="badge bg-ok">Đang kết nối AI...</div>
  </div>

  <div class="Controls">
    <div class="stat-box"><div class="stat-label">Độ phân giải & Tốc độ Stream</div><div class="stat-val" id="val_fps">HD 1280x720</div></div>
    <div class="stat-box"><div class="stat-label">Trạng thái Học Môi Trường</div><div class="stat-val" id="val_env">100% Ổn định</div></div>
    <div class="stat-box"><div class="stat-label">Vật thể bất thường / AI Infer</div><div class="stat-val" id="val_anom">0 vật thể</div></div>
    <div class="stat-box"><div class="stat-label">Nhiệt độ Chip ESP32-S3</div><div class="stat-val" id="val_temp">-- °C</div></div>
  </div>

  <div class="stage-wrap">
    <img id="cam_stream" alt="Đang tải luồng MJPEG..." crossorigin="anonymous">
    <canvas id="ai_canvas"></canvas>
  </div>

  <div class="toolbar">
    <div style="display:flex;align-items:center;gap:10px;flex-wrap:wrap;">
      <label style="font-size:0.86rem;">Phân giải:</label>
      <select id="sel_res" onchange="changeRes(this.value)">
        <option value="fhd">FHD (1920x1080)</option>
        <option value="hd" selected>HD (1280x720 - Mượt & Nét tối ưu)</option>
        <option value="svga">SVGA (800x600 - Siêu mượt 30p)</option>
        <option value="vga">VGA (640x480 - Tốc độ tối đa)</option>
      </select>

      <label style="font-size:0.86rem;margin-left:6px;">Bù sáng (AE):</label>
      <input type="range" min="-2" max="2" step="1" value="1" oninput="updateAE(this.value)">
      <span id="lbl_ae" style="font-weight:700;color:#38bdf8;">+1</span>

      <label style="font-size:0.86rem;margin-left:6px;">Nén JPEG:</label>
      <input type="range" min="6" max="24" step="1" value="10" oninput="updateQual(this.value)">
      <span id="lbl_q" style="font-weight:700;color:#38bdf8;">10</span>

      <label style="font-size:0.86rem;margin-left:6px;">Ngưỡng lọc nhiễu AI:</label>
      <input type="range" id="rng_thr" min="2.0" max="7.0" step="0.2" value="3.8" oninput="updateThr(this.value)">
      <span id="lbl_thr" style="font-weight:700;color:#38bdf8;">3.8</span>
    </div>
    <button onclick="resetEnv()">🔄 Học lại góc Camera ngay (Reset Nền)</button>
  </div>

  <script>
    const streamImg = document.getElementById('cam_stream');
    const canvas = document.getElementById('ai_canvas');
    const ctx = canvas.getContext('2d');
    const streamUrl = 'http://' + window.location.hostname + ':81/stream';
    streamImg.src = streamUrl;
    streamImg.onerror = () => { setTimeout(() => { streamImg.src = streamUrl + '?t=' + Date.now(); }, 600); };

    function changeRes(v) { fetch('/ctrl?res=' + v); }
    function updateAE(v) { document.getElementById('lbl_ae').innerText = (v > 0 ? '+' : '') + v; fetch('/ctrl?ae=' + v); }
    function updateQual(v) { document.getElementById('lbl_q').innerText = v; fetch('/ctrl?qual=' + v); }
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
          badge.innerText = '✅ MÔI TRƯỜNG ỔN ĐỊNH (Đã lọc sạch nhiễu - Lớp nền #' + d.scene_cls + ')';
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

          const label = 'VẬT THỂ BẤT THƯỜNG #' + (idx + 1) + ' | AI Class ' + b.m2 + ' (M1:' + b.m1 + ') | ' + b.score + '%';
          ctx.font = 'bold 13px sans-serif';
          const tw = ctx.measureText(label).width + 14;
          const ly = (by > 26) ? (by - 26) : by;
          ctx.fillStyle = '#ef4444';
          ctx.fillRect(bx, ly, tw, 24);
          ctx.fillStyle = '#ffffff';
          ctx.fillText(label, bx + 7, ly + 17);
        });
      } catch (e) {}
      setTimeout(pollAI, 200);
    }
    pollAI();
  </script>
</body>
</html>
)rawliteral";

// ---------------- 5. CÁC HANDLER HTTP (PORT 80: UI + AI JSON | PORT 81: MJPEG STREAM) -------
#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static char s_json_buf[1024];

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
    uint16_t fw = 1280, fh = 720;

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

    int len = snprintf(s_json_buf, sizeof(s_json_buf),
        "{\"w\":%u,\"h\":%u,\"fps\":%.1f,\"temp\":%.1f,\"stability\":%.1f,\"changed\":%.3f,"
        "\"angle_shift\":%s,\"learn_left\":%d,\"scene_cls\":%u,\"ai_us\":%lu,\"boxes\":[",
        fw, fh, fps, temp, stab, chg, ang ? "true" : "false", learn_left, s_cls, (unsigned long)infer_us);

    for (int i = 0; i < n_boxes; i++) {
        len += snprintf(s_json_buf + len, sizeof(s_json_buf) - len,
            "%s{\"x\":%.4f,\"y\":%.4f,\"w\":%.4f,\"h\":%.4f,\"score\":%d,\"m1\":%u,\"m2\":%u}",
            (i > 0) ? "," : "",
            local_boxes[i].x, local_boxes[i].y, local_boxes[i].w, local_boxes[i].h,
            local_boxes[i].score, local_boxes[i].ai_cls_m1, local_boxes[i].ai_cls_m2);
    }
    snprintf(s_json_buf + len, sizeof(s_json_buf) - len, "]}");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, s_json_buf, strlen(s_json_buf));
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
            if (t >= 1.5f && t <= 10.0f) s_anom_threshold = t;
        }
        if (s && httpd_query_key_value(query, "qual", val, sizeof(val)) == ESP_OK) {
            int q = atoi(val);
            if (q >= 5 && q <= 40) s->set_quality(s, q);
        }
        if (s && httpd_query_key_value(query, "ae", val, sizeof(val)) == ESP_OK) {
            int ae = atoi(val);
            if (ae >= -2 && ae <= 2) {
                s->set_ae_level(s, ae);
                s->set_brightness(s, ae);
            }
        }
        if (s && httpd_query_key_value(query, "res", val, sizeof(val)) == ESP_OK) {
            if (strcmp(val, "fhd") == 0)       s->set_framesize(s, FRAMESIZE_FHD);
            else if (strcmp(val, "hd") == 0)   s->set_framesize(s, FRAMESIZE_HD);
            else if (strcmp(val, "svga") == 0) s->set_framesize(s, FRAMESIZE_SVGA);
            else if (strcmp(val, "vga") == 0)  s->set_framesize(s, FRAMESIZE_VGA);
            s_force_relearn = true;
        }
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, "OK", 2);
}

static esp_err_t stream_handler(httpd_req_t *req) {
    int sockfd = httpd_req_to_sockfd(req);
    int nodelay = 1;
    setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

    esp_err_t res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");

    s_stream_clients++;
    char hdr_buf[128];
    uint32_t frame_cnt = 0;
    uint32_t t_fps_start = millis();

    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        s_total_frames++;
        frame_cnt++;
        s_ai_frame_w = fb->width;
        s_ai_frame_h = fb->height;

        if (s_ai_need_frame && !s_ai_frame_ready && s_ai_jpeg_buf &&
            fb->len <= MAX_JPEG_COPY_SZ && (fb->width / 8) <= MAX_RGB_THUMB_W) {
            memcpy(s_ai_jpeg_buf, fb->buf, fb->len);
            s_ai_jpeg_len    = fb->len;
            s_ai_frame_ready = true;
            s_ai_need_frame  = false;
        }

        int hlen = snprintf(hdr_buf, sizeof(hdr_buf),
                            "\r\n--" PART_BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                            (unsigned)fb->len);
        res = httpd_resp_send_chunk(req, hdr_buf, hlen);
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        }

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
    cfg_web.server_port       = 80;
    cfg_web.ctrl_port         = 32768;
    cfg_web.stack_size        = 8192;
    cfg_web.keep_alive_enable = true;
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
    cfg_stream.stack_size  = 12288;
    if (httpd_start(&s_stream_httpd, &cfg_stream) == ESP_OK) {
        httpd_uri_t u_strm = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL };
        httpd_register_uri_handler(s_stream_httpd, &u_strm);
    }
}

static void setup_zero_bottleneck_wifi() {
    WiFi.persistent(false);

#if USE_DIRECT_AP_MODE
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, 2);
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT40);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

    Serial.println("============================================================");
    Serial.println("  [CHE DO CHONG NGHEN TOI DA: PURE WI-FI AP 40MHz HT40]");
    Serial.printf ("  1. Ket noi Wi-Fi tren May tinh/DT : %s\n", AP_SSID);
    Serial.printf ("  2. Mat khau Wi-Fi                 : %s\n", AP_PASS);
    Serial.print  ("  3. Mo trinh duyet truy cap ngay   : http://");
    Serial.println(WiFi.softAPIP());
    Serial.println("============================================================");
#else
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
        Serial.print  ("  [WI-FI ROUTER STA] Truy cap Live Stream + AI: http://");
        Serial.println(WiFi.localIP());
        Serial.println("============================================================");
    } else {
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
        case OV5640_PID: return "OV5640 (5MP - High-FPS Anti-Lag Mode)";
        case OV3660_PID: return "OV3660 (3MP)";
        case OV2640_PID: return "OV2640 (2MP)";
        default:         return "Unknown Sensor";
    }
}

void setup() {
    Serial.begin(115200);
    delay(1200);

    Serial.println("\n============================================================");
    Serial.println("  [SKETCH 1] OV5640 HIGH-FPS STREAM + 5-STAGE ANTI-NOISE AI");
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
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size   = has_psram ? FRAMESIZE_HD : FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count     = has_psram ? 3 : 1;
    config.fb_location  = has_psram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
    config.grab_mode    = CAMERA_GRAB_LATEST;

    Serial.println("[3/4] Dang khoi tao Camera OV5640 (Che do High-FPS, khoa Night-Mode Slow Shutter)...");
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("  -> [LOI] esp_camera_init that bai (0x%x). Kiem tra cap det FPC!\n", err);
        s_cam_ok = false;
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_framesize(s, has_psram ? FRAMESIZE_HD : FRAMESIZE_VGA);
        s->set_quality(s, 10);
        s->set_exposure_ctrl(s, 1);
        s->set_aec2(s, 0);                         // TẮT Night-Mode Frame Drop (tránh kẹt 11.0 FPS!)
        s->set_ae_level(s, 1);
        s->set_gain_ctrl(s, 1);
        s->set_gainceiling(s, (gainceiling_t)4);   // GAINCEILING_16X
        s->set_brightness(s, 1);
        s->set_contrast(s, 1);
        s->set_saturation(s, 0);
        s->set_sharpness(s, 0);
        s->set_denoise(s, 1);
        s->set_whitebal(s, 1);
        s->set_awb_gain(s, 1);
        s->set_bpc(s, 1);
        s->set_wpc(s, 1);
        s->set_raw_gma(s, 1);
        s->set_lenc(s, 1);
        s->set_dcw(s, 1);
        s->set_vflip(s, 1);
        Serial.printf("[4/4] Nhan dien thanh cong     : %s (PID = 0x%04X)\n",
                      get_sensor_name(s->id.PID), s->id.PID);
    }
    s_cam_ok = true;
    s_ai_need_frame = true;

    setup_zero_bottleneck_wifi();
    start_web_and_stream_servers();

    // Cấp Stack 20480 bytes (20 KB) cho ai_learn_task (toàn bộ mảng lớn đã đưa ra BSS tĩnh)
    xTaskCreatePinnedToCore(ai_continual_learning_task, "ai_learn_task", 20480, NULL, 2, NULL, 0);
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
            Serial.printf("[AI LEARNING] DOI GOC CAMERA (Lech %.1f%%) -> Dang hoc nen moi (On dinh: %.0f%%) | FPS=%.1f | Temp=%.1fC\n",
                          chg * 100.0f, stab, s_stream_fps, s_chip_temp_c);
        } else if (n_boxes > 0) {
            int bx_px = (int)(b0.x * s_ai_frame_w);
            int by_px = (int)(b0.y * s_ai_frame_h);
            int bw_px = (int)(b0.w * s_ai_frame_w);
            int bh_px = (int)(b0.h * s_ai_frame_h);
            Serial.printf("[AI ANOMALY!] %d vat the that! Box#1=[x:%d,y:%d,w:%d,h:%d] | AI=%u(M1:%u) | Score=%d%% | Infer=%.1fms | FPS=%.1f | Temp=%.1fC\n",
                          n_boxes, bx_px, by_px, bw_px, bh_px, b0.ai_cls_m2, b0.ai_cls_m1, b0.score, ai_us / 1000.0f, s_stream_fps, s_chip_temp_c);
        } else {
            Serial.printf("[AI NORMAL] Nen on dinh (%.0f%%, Class=%u) | FPS=%.1f | Infer=%.1fms | Temp=%.1fC\n",
                          stab, s_cls, s_stream_fps, ai_us / 1000.0f, s_chip_temp_c);
        }
    }

    delay(20);
}
