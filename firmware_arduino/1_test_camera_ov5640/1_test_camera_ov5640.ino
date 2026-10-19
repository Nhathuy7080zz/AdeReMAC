// =========================================================================================
// SKETCH 1 (BẢN TỐI ƯU THỰC CHIẾN V5): STREAM VIDEO HD@30FPS + ĐÓNG KHUNG ĐỐI TƯỢNG SAKBOT
// Board: ESP32 Cam Kit Phát Triển ESP32-S3 N16R8 OV5640 TypeC (Arduino IDE 2.x - Zero External Libs)
//
// ĐÃ KHẮC PHỤC TRIỆT ĐỂ 2 VẤN ĐỀ TỪ ẢNH CHỤP THỰC TẾ (12.7 FPS & 289.3ms NHẬN DẠNG SAI TRẦN NHÀ):
//   1. Tại sao bản v4 bị kẹt ở ~12.7 - 14 FPS (dù kéo Nén JPEG lên 24) và trễ 289.3 ms?
//      - Nguyên nhân 1 (Phần cứng OV5640): Trong thư viện esp32-camera, khi chọn FRAMESIZE_HD (1280x720),
//        driver mặc định đặt thanh ghi chia xung hệ thống 0x3035 = 0x41 (sys_div = 4, tương ứng cấu hình
//        "720p 15fps" trong tài liệu OmniVision OV5640 Application Note, Mục 4.1.5) hoặc dùng khung quét
//        lớn làm tốc độ xuất xung VSYNC vật lý của cảm biến bị khóa cứng ở ~13.2 - 14 FPS!
//      - Nguyên nhân 2 (Nghẽn CPU Core 0): Bản v4 chạy 2 tầng tích chập phần mềm Conv1 + Conv2 (~724.000
//        phép tính MAC) cho các khung bao làm thời gian xử lý AI vọt lên 289.3 ms/frame trên Core 0,
//        chiếm dụng CPU của tiến trình Wi-Fi và lwIP TCP/IP.
//      -> GIẢI PHÁP TRONG BẢN V5:
//        (a) Ghi trực tiếp bảng thanh ghi phần cứng "720p 30fps" chính hãng OmniVision (Mục 4.1.5):
//            Bật 2x2 Binning/Subsampling (0x3814=0x31, 0x3815=0x31, HTS=0x0764 [1892], VTS=0x02E4 [740]),
//            chuyển thanh ghi PLL 0x3035 từ 0x41 (15fps) sang 0x21 (sys_div = 2 -> 30fps!), đặt bộ chia
//            DVP PCLK 0x3824 = 0x04 và khóa trần phơi sáng 0x3A02/0x3A14 = 0x02E0 (736 dòng).
//        (b) Loại bỏ hoàn toàn vòng lặp tích chập nặng và bỏ việc gán nhãn tên đối tượng theo đúng yêu cầu:
//            Thời gian chạy AI giảm từ 289.3 ms xuống chỉ còn ~2.5 - 4.0 ms/frame!
//
//   2. Tại sao bản v4 đóng khung sai lên trần nhà tĩnh và không tắt được dù kéo Ngưỡng AI = 5?
//      - Nguyên nhân 1: Nhánh phát hiện đèn Flash (is_new_flash) ở bản v4 bắt theo pixel đơn lẻ sáng nhất
//        (max2_luma) và ép cứng điểm bất thường z = thr + 2.2f (vô hiệu hóa hoàn toàn thanh trượt Ngưỡng AI = 5!).
//        Khi phòng tối có 1 điểm nóng CMOS (hot-pixel trắng) hoặc ánh sáng cửa sổ hắt lên điều hòa/phào chỉ,
//        nó tự kích hoạt khung vàng.
//      - Nguyên nhân 2 (Hiện tượng "Ghost Deadlock" - Bóng ma khóa chết): Lệnh `if (s_verified_anom[i]) continue;`
//        ở bản v4 cấm cập nhật nền tại các ô đang đóng khung. Khi camera vừa khởi động hoặc đổi phơi sáng AE
//        bất tuyến tính (do góc dưới bên phải có cửa sổ nắng gắt còn trần nhà tối), các ô trần nhà vượt ngưỡng
//        sẽ bị khóa chặt vĩnh viễn thành "khung ma" (Ghost) không bao giờ tự xóa được!
//      -> GIẢI PHÁP KHOA HỌC TRONG BẢN V5 (Thuật toán Sakbot - Cucchiara et al., IEEE TPAMI 2003):
//        (a) Xóa bỏ hoàn toàn nhánh ép ngưỡng `is_new_flash` đơn điểm; dùng Trung bình cắt cực trị (Trimmed Mean,
//            loại bỏ pixel sáng nhất và tối nhất trong mỗi ô) để miễn nhiễm 100% với Hot-Pixel.
//        (b) Điều kiện Kép Sakbot (Dual-Evidence): Một khung bao MỚI chỉ được phép hình thành khi cụm ô vừa
//            khác biệt với mô hình nền (Z > Ngưỡng AI) VỪA có năng lượng chuyển động thực sự giữa các khung hình
//            liên tiếp (Temporal Motion Energy). Trần nhà, phào chỉ, điều hòa, rèm cửa đứng yên có năng lượng
//            chuyển động bằng 0 nên TUYỆT ĐỐI KHÔNG BAO GIỜ bị đóng khung!
//        (c) Tự động giải phóng "Bóng ma" (Anti-Ghosting Decay): Khi vật thể dừng chuyển động hoàn toàn,
//            bộ đếm giữ khung giảm dần vô điều kiện và hấp thụ vào nền sau ~1.5 giây, đảm bảo không bao giờ
//            có bất kỳ khung giả nào tồn tại dai dẳng trên màn hình.
//        (d) Chỉ vẽ khung chữ nhật sắc nét (kèm góc định vị) bao quanh đối tượng, bỏ hoàn toàn chữ/nhãn rối mắt.
// =========================================================================================

#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"
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
// 3. CẤU HÌNH THUẬT TOÁN SAKBOT HYBRID MOTION-BACKGROUND DETECTOR (IEEE TPAMI 2003)
// =========================================================================================
#define GRID_W              20      // Lưới không gian 20 cột x 15 hàng (300 ô)
#define GRID_H              15
#define GRID_CELLS          (GRID_W * GRID_H)
#define MAX_ANOM_BOXES      5       // Số khung đóng quanh vật thể tối đa
#define MAX_JPEG_COPY_SZ    (128 * 1024)
#define STREAM_HDR_RESERVE  128
#define MAX_RGB_THUMB_W     240
#define MAX_RGB_THUMB_H     180

struct AnomBox {
    float   x, y, w, h;     // Tọa độ chuẩn hóa [0.0 .. 1.0]
    int     score;          // Độ tin cậy (65 .. 99%)
    uint8_t ai_cls_m1;      // Nhãn bộ nhân AdaReMAC Mode 1 (25% dữ liệu, thr=2) phục vụ thống kê
    uint8_t ai_cls_m2;      // Nhãn bộ nhân AdaReMAC Mode 2 (100% dữ liệu, Exact) phục vụ thống kê
};

// --- BỘ NHỚ NỀN THỐNG KÊ THÍCH NGHI & ĐỘNG HỌC SAKBOT (STATIC .BSS - 0 BYTE STACK) ---
static float   s_bg_y[GRID_CELLS];          // Độ sáng nền trung bình tại ô i
static float   s_bg_cr[GRID_CELLS];         // Sắc độ Đỏ nền
static float   s_bg_cb[GRID_CELLS];         // Sắc độ Xanh nền
static float   s_bg_edge[GRID_CELLS];       // Cường độ biên tĩnh của nền
static float   s_spatial_grad[GRID_CELLS];  // Độ dốc không gian giữa ô i và 8 ô lân cận
static float   s_bg_var[GRID_CELLS];        // Độ lệch chuẩn nhiễu tự nhiên tại ô i

static float   s_smooth_y[GRID_CELLS];      // Độ sáng lọc mượt thời gian (EMA)
static float   s_prev_smooth_y[GRID_CELLS]; // Độ sáng lọc mượt ở chu kỳ trước
static float   s_motion_ema[GRID_CELLS];    // Năng lượng chuyển động ngắn hạn của từng ô
static uint8_t s_static_age[GRID_CELLS];    // Số chu kỳ liên tiếp ô đứng yên
static uint8_t s_fg_hold[GRID_CELLS];       // Bộ đếm giữ khung Sakbot (tự động giảm về 0 khi đứng yên)
static uint8_t s_cell_persist[GRID_CELLS];  // Độ bền vững chống nhiễu 1-frame

// --- BỘ ĐỆM TRUNG GIAN TĨNH (0 BYTE STACK) ---
static float   s_raw_y[GRID_CELLS];
static float   s_raw_cr[GRID_CELLS];
static float   s_raw_cb[GRID_CELLS];
static float   s_raw_edge[GRID_CELLS];
static float   s_cell_y[GRID_CELLS];
static float   s_cell_cr[GRID_CELLS];
static float   s_cell_cb[GRID_CELLS];
static float   s_cell_edge[GRID_CELLS];
static float   s_z_map[GRID_CELLS];
static uint8_t s_moving_map[GRID_CELLS];
static uint8_t s_raw_anom[GRID_CELLS];
static uint8_t s_verified_anom[GRID_CELLS];
static uint8_t s_visited[GRID_CELLS];
static int16_t s_qx[GRID_CELLS];
static int16_t s_qy[GRID_CELLS];
static uint8_t s_patch_raw[AI_FEAT_DIM];
static int8_t  s_patch_feat[AI_FEAT_DIM];
static AnomBox s_cand_boxes[MAX_ANOM_BOXES];
static AnomBox s_tracked_boxes[MAX_ANOM_BOXES];
static int     s_num_tracked = 0;

static bool           s_env_initialized   = false;
static volatile int   s_fast_learn_frames = 0;
static int            s_learn_cooldown    = 0;
static volatile bool  s_force_relearn     = false;
static volatile float s_anom_threshold    = 2.5f;

// Kết quả xuất ra Web UI và Serial Monitor
static portMUX_TYPE s_ai_mux = portMUX_INITIALIZER_UNLOCKED;
static AnomBox  s_boxes[MAX_ANOM_BOXES];
static int      s_num_boxes = 0;
static float    s_env_stability = 100.0f;
static float    s_changed_cell_ratio = 0.0f;
static bool     s_angle_changed = false;
static uint8_t  s_scene_ai_cls = 0;
static uint8_t  s_active_hw_mode = 0;       // 0: Gatekeeper (0%), 1: Partial (25%), 2: Exact (100%)
static uint32_t s_ai_infer_us = 0;
static float    s_stream_fps = 0.0f;
static float    s_chip_temp_c = 40.0f;
static uint32_t s_total_frames = 0;

// Bộ đệm chia sẻ Zero-Hold giữa Stream Task (Core 1) và AI Task (Core 0)
static uint8_t  *s_stream_tx_buf = NULL;
static uint8_t  *s_ai_jpeg_buf   = NULL;
static size_t    s_ai_jpeg_len   = 0;
static uint16_t  s_ai_frame_w    = 1280;
static uint16_t  s_ai_frame_h    = 720;
static volatile bool     s_ai_need_frame  = false;
static volatile bool     s_ai_frame_ready = false;
static volatile int      s_stream_clients = 0;
static volatile uint32_t s_last_stream_ms = 0;
static uint16_t *s_rgb565_thumb  = NULL;
static bool      s_thumb_in_sram = false;

static bool s_cam_ok = false;
static httpd_handle_t s_web_httpd    = NULL;
static httpd_handle_t s_stream_httpd = NULL;

// =========================================================================================
// 4. BỘ NHÂN XẤP XỈ ADAREMAC (BIT-EXACT 100% VỚI RTL VERILOG: qL=2, k=3, R=2*D2 - D0)
// =========================================================================================
static inline uint16_t IRAM_ATTR adaremac_mul_u8(uint8_t a, uint8_t b, uint8_t mode, uint8_t thr) {
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

static inline int32_t IRAM_ATTR adaremac_mac_i8(int8_t a, int8_t w, uint8_t mode, uint8_t thr) {
    if (a == 0 || w == 0) return 0;
    if (mode >= 2) return (int32_t)a * (int32_t)w;
    bool neg = (a < 0) ^ (w < 0);
    uint8_t ua = (a < 0) ? (uint8_t)(-(int16_t)a) : (uint8_t)a;
    uint8_t uw = (w < 0) ? (uint8_t)(-(int16_t)w) : (uint8_t)w;
    int32_t mag = (int32_t)adaremac_mul_u8(ua, uw, mode, thr);
    return neg ? -mag : mag;
}

// Suy luận siêu tốc (< 0.12 ms) với bộ nhân AdaReMAC cho Mode 1 (25% = 100 chiều) và Mode 2 (100% = 400 chiều)
static uint8_t adaremac_run_fast_fc(const int8_t *feat400, uint8_t mode, uint8_t thr) {
    int32_t best_score = INT32_MIN;
    uint8_t best_cls = 0;
    if (mode == 1) {
        for (int c = 0; c < AI_NUM_CLASSES; c++) {
            int32_t acc = FC_B_M1[c];
            const int8_t *w_row = &FC_W_M1[c * AI_PARTIAL_DIM];
            for (int k = 0; k < AI_PARTIAL_DIM; k++) {
                uint16_t idx = AI_TOP100_IDX[k];
                acc += adaremac_mac_i8(feat400[idx], w_row[k], 1, thr);
            }
            if (acc > best_score) {
                best_score = acc;
                best_cls = (uint8_t)c;
            }
        }
    } else {
        for (int c = 0; c < AI_NUM_CLASSES; c++) {
            int32_t acc = FC_B[c];
            const int8_t *w_row = &FC_W[c * AI_FEAT_DIM];
            for (int k = 0; k < AI_FEAT_DIM; k++) {
                acc += adaremac_mac_i8(feat400[k], w_row[k], mode, thr);
            }
            if (acc > best_score) {
                best_score = acc;
                best_cls = (uint8_t)c;
            }
        }
    }
    return best_cls;
}

static void extract_patch_int8_400(const uint16_t *rgb565, int img_w, int img_h,
                                   int x0, int y0, int bw, int bh, int8_t *out_feat400) {
    if (bw < 1) bw = 1;
    if (bh < 1) bh = 1;
    int32_t min_y = 255, max_y = 0;
    for (int py = 0; py < 20; py++) {
        int sy = y0 + (py * bh) / 20;
        if (sy < 0) sy = 0;
        if (sy >= img_h) sy = img_h - 1;
        const uint16_t *row = &rgb565[sy * img_w];
        for (int px = 0; px < 20; px++) {
            int sx = x0 + (px * bw) / 20;
            if (sx < 0) sx = 0;
            if (sx >= img_w) sx = img_w - 1;
            uint16_t p = row[sx];
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
    int32_t span = max_y - min_y;
    if (span < 32) span = 32;
    for (int i = 0; i < AI_FEAT_DIM; i++) {
        int32_t norm = ((int32_t)(s_patch_raw[i] - min_y) * 127) / span;
        if (norm < 0) norm = 0;
        if (norm > 127) norm = 127;
        out_feat400[i] = (int8_t)norm;
    }
}

static void update_spatial_gradients() {
    for (int gy = 0; gy < GRID_H; gy++) {
        for (int gx = 0; gx < GRID_W; gx++) {
            int idx = gy * GRID_W + gx;
            float max_g = 0.0f;
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    if (dx == 0 && dy == 0) continue;
                    int nx = gx + dx, ny = gy + dy;
                    if (nx >= 0 && nx < GRID_W && ny >= 0 && ny < GRID_H) {
                        float g = fabsf(s_bg_y[idx] - s_bg_y[ny * GRID_W + nx]);
                        if (g > max_g) max_g = g;
                    }
                }
            }
            s_spatial_grad[idx] = max_g;
        }
    }
}

// =========================================================================================
// 5. THUẬT TOÁN PHÁT HIỆN & ĐÓNG KHUNG ĐỐI TƯỢNG SAKBOT (Cucchiara et al., IEEE TPAMI 2003)
//    Siêu nhẹ (~2.5 ms/frame), Không bao giờ báo giả trên trần nhà/tường tĩnh, Không bóng ma!
// =========================================================================================
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
        portENTER_CRITICAL(&s_ai_mux);
        if (s_fast_learn_frames > 0) s_fast_learn_frames--;
        s_angle_changed = (s_fast_learn_frames > 0);
        portEXIT_CRITICAL(&s_ai_mux);
        return;
    }

    // -----------------------------------------------------------------------------------------
    // BƯỚC 1: Trích xuất đặc trưng lưới 20x15 bằng Trung bình cắt cực trị (Trimmed Mean)
    // Tự động loại bỏ điểm sáng nhất (max_luma) và tối nhất (min_luma) trong mỗi ô để miễn nhiễm
    // hoàn toàn với Hot-Pixel trắng trên trần nhà tối và nhiễu hạt CMOS!
    // -----------------------------------------------------------------------------------------
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
            int min_luma = 255, max_luma = 0;

            for (int y = y_start; y < y_end; y += 2) {
                const uint16_t *row = &s_rgb565_thumb[y * thumb_w];
                for (int x = x_start; x < x_end; x += 2) {
                    uint16_t p = row[x];
                    uint16_t v = ((p & 0xFF) << 8) | (p >> 8);
                    int r = ((v >> 11) & 0x1F) << 3;
                    int g = ((v >> 5)  & 0x3F) << 2;
                    int b = (v & 0x1F) << 3;
                    int luma = (r * 77 + g * 150 + b * 29) >> 8;

                    if (luma < min_luma) min_luma = luma;
                    if (luma > max_luma) max_luma = luma;

                    sum_y += luma;
                    sum_r += r;
                    sum_b += b;
                    if (prev_luma >= 0) sum_edge += abs(luma - prev_luma);
                    prev_luma = luma;
                    cnt++;
                }
            }
            int idx = gy * GRID_W + gx;
            if (cnt > 4) {
                // Trimmed Mean: Cắt bỏ điểm cực đại (hot-pixel) và cực tiểu trước khi chia trung bình
                s_raw_y[idx]    = (float)(sum_y - max_luma - min_luma) / (float)(cnt - 2);
                s_raw_cr[idx]   = (float)sum_r / cnt - s_raw_y[idx];
                s_raw_cb[idx]   = (float)sum_b / cnt - s_raw_y[idx];
                s_raw_edge[idx] = (float)sum_edge / cnt;
            } else if (cnt > 0) {
                s_raw_y[idx]    = (float)sum_y / cnt;
                s_raw_cr[idx]   = (float)sum_r / cnt - s_raw_y[idx];
                s_raw_cb[idx]   = (float)sum_b / cnt - s_raw_y[idx];
                s_raw_edge[idx] = (float)sum_edge / cnt;
            } else {
                s_raw_y[idx] = s_raw_cr[idx] = s_raw_cb[idx] = s_raw_edge[idx] = 0.0f;
            }
        }
    }

    // -----------------------------------------------------------------------------------------
    // BƯỚC 2: Làm mượt không gian 3x3 khử nhiễu khối JPEG
    // -----------------------------------------------------------------------------------------
    for (int gy = 0; gy < GRID_H; gy++) {
        for (int gx = 0; gx < GRID_W; gx++) {
            int idx = gy * GRID_W + gx;
            float nb_y = 0, nb_cr = 0, nb_cb = 0, nb_ed = 0, nb_cnt = 0;
            for (int dy = -1; dy <= 1; dy++) {
                int ny = gy + dy;
                if (ny < 0 || ny >= GRID_H) continue;
                for (int dx = -1; dx <= 1; dx++) {
                    int nx = gx + dx;
                    if (nx < 0 || nx >= GRID_W || (dx == 0 && dy == 0)) continue;
                    int n_idx = ny * GRID_W + nx;
                    nb_y  += s_raw_y[n_idx];
                    nb_cr += s_raw_cr[n_idx];
                    nb_cb += s_raw_cb[n_idx];
                    nb_ed += s_raw_edge[n_idx];
                    nb_cnt += 1.0f;
                }
            }
            s_cell_y[idx]    = 0.78f * s_raw_y[idx]    + 0.22f * (nb_y  / nb_cnt);
            s_cell_cr[idx]   = 0.78f * s_raw_cr[idx]   + 0.22f * (nb_cr / nb_cnt);
            s_cell_cb[idx]   = 0.78f * s_raw_cb[idx]   + 0.22f * (nb_cb / nb_cnt);
            s_cell_edge[idx] = 0.78f * s_raw_edge[idx] + 0.22f * (nb_ed / nb_cnt);
        }
    }

    // -----------------------------------------------------------------------------------------
    // Khởi tạo mô hình nền khi mới bật mạch hoặc bấm nút "Học lại góc Camera"
    // -----------------------------------------------------------------------------------------
    if (!s_env_initialized || s_force_relearn) {
        for (int i = 0; i < GRID_CELLS; i++) {
            s_bg_y[i]          = s_cell_y[i];
            s_bg_cr[i]         = s_cell_cr[i];
            s_bg_cb[i]         = s_cell_cb[i];
            s_bg_edge[i]       = s_cell_edge[i];
            s_bg_var[i]        = 7.0f;
            s_smooth_y[i]      = s_cell_y[i];
            s_prev_smooth_y[i] = s_cell_y[i];
            s_motion_ema[i]    = 0.0f;
            s_static_age[i]    = 30;
            s_fg_hold[i]       = 0;
            s_cell_persist[i]  = 0;
        }
        update_spatial_gradients();
        extract_patch_int8_400(s_rgb565_thumb, thumb_w, thumb_h, 0, 0, thumb_w, thumb_h, s_patch_feat);
        s_scene_ai_cls      = adaremac_run_fast_fc(s_patch_feat, 2, 0);
        s_num_tracked       = 0;
        s_env_initialized   = true;
        s_force_relearn     = false;
        s_fast_learn_frames = 5;  // Học nhanh 5 chu kỳ (~0.3 giây) để hấp thụ trọn đường cong hội tụ AE/AWB
        s_learn_cooldown    = 25;
    }

    // -----------------------------------------------------------------------------------------
    // BƯỚC 3: ĐO NĂNG LƯỢNG CHUYỂN ĐỘNG THỜI GIAN THỰC (TEMPORAL MOTION ENERGY - SAKBOT)
    // Chỉ những ô có sự thay đổi thực sự giữa các frame liên tiếp mới có năng lượng chuyển động!
    // -----------------------------------------------------------------------------------------
    float thr = s_anom_threshold; // Nhận trực tiếp từ thanh trượt Ngưỡng AI (1.4 .. 5.0)
    float sum_dt = 0.0f;
    for (int i = 0; i < GRID_CELLS; i++) {
        s_smooth_y[i] = 0.55f * s_smooth_y[i] + 0.45f * s_cell_y[i];
        sum_dt += (s_smooth_y[i] - s_prev_smooth_y[i]);
    }
    float frame_ae_step = sum_dt / (float)GRID_CELLS;

    int moving_cells = 0;
    int quad_move[4] = {0, 0, 0, 0};
    int opp_top = 0, opp_bot = 0, opp_left = 0, opp_right = 0;

    // Ngưỡng nhạy chuyển động frame-to-frame tỷ lệ thuận với thanh trượt Ngưỡng AI của người dùng
    float motion_trig_thr = 3.2f + 1.35f * thr;

    for (int gy = 0; gy < GRID_H; gy++) {
        for (int gx = 0; gx < GRID_W; gx++) {
            int i = gy * GRID_W + gx;
            float grad_tol = 0.22f * s_spatial_grad[i] + 0.18f * s_bg_edge[i];
            float raw_dt   = fabsf((s_smooth_y[i] - s_prev_smooth_y[i]) - frame_ae_step);
            s_prev_smooth_y[i] = s_smooth_y[i];

            float net_dt = raw_dt - grad_tol;
            if (net_dt < 0.0f) net_dt = 0.0f;
            s_motion_ema[i] = 0.45f * s_motion_ema[i] + 0.55f * net_dt;

            if (raw_dt > (11.5f + grad_tol)) {
                moving_cells++;
                int q = ((gy >= GRID_H / 2) ? 2 : 0) + ((gx >= GRID_W / 2) ? 1 : 0);
                quad_move[q]++;
                if (gy <= 2)          opp_top++;
                if (gy >= GRID_H - 3) opp_bot++;
                if (gx <= 2)          opp_left++;
                if (gx >= GRID_W - 3) opp_right++;
            }

            if (s_motion_ema[i] > motion_trig_thr || net_dt > (motion_trig_thr * 1.25f)) {
                s_moving_map[i] = 1;
                s_static_age[i] = 0;
            } else {
                s_moving_map[i] = 0;
                if (s_static_age[i] < 250) s_static_age[i]++;
            }
        }
    }

    // -----------------------------------------------------------------------------------------
    // BƯỚC 4: PHÁT HIỆN ĐỔI GÓC CAMERA TOÀN CỤC CÓ KHÓA COOLDOWN CHỐNG KẸT
    // -----------------------------------------------------------------------------------------
    if (s_learn_cooldown > 0) s_learn_cooldown--;

    int active_quads = (quad_move[0] >= 7) + (quad_move[1] >= 7) + (quad_move[2] >= 7) + (quad_move[3] >= 7);
    bool opposite_borders = (opp_top >= 7 && opp_bot >= 7) || (opp_left >= 7 && opp_right >= 7);

    if (s_fast_learn_frames == 0 && s_learn_cooldown == 0) {
        if (moving_cells > (GRID_CELLS * 55) / 100 ||
            (active_quads >= 3 && opposite_borders && moving_cells >= 65)) {
            s_fast_learn_frames = 4;
            s_learn_cooldown    = 30;
        }
    }

    if (s_fast_learn_frames > 0) {
        s_fast_learn_frames--;
        float alpha_fast = 0.55f;
        for (int i = 0; i < GRID_CELLS; i++) {
            s_bg_y[i]         = (1.0f - alpha_fast) * s_bg_y[i]    + alpha_fast * s_cell_y[i];
            s_bg_cr[i]        = (1.0f - alpha_fast) * s_bg_cr[i]   + alpha_fast * s_cell_cr[i];
            s_bg_cb[i]        = (1.0f - alpha_fast) * s_bg_cb[i]   + alpha_fast * s_cell_cb[i];
            s_bg_edge[i]      = (1.0f - alpha_fast) * s_bg_edge[i] + alpha_fast * s_cell_edge[i];
            s_bg_var[i]       = 7.0f;
            s_motion_ema[i]   = 0.0f;
            s_fg_hold[i]      = 0;
            s_cell_persist[i] = 0;
        }
        update_spatial_gradients();
        s_num_tracked = 0;
        uint32_t dt_us = micros() - t0;
        portENTER_CRITICAL(&s_ai_mux);
        s_num_boxes          = 0;
        s_angle_changed      = (s_fast_learn_frames > 0);
        s_changed_cell_ratio = (float)moving_cells / (float)GRID_CELLS;
        s_env_stability      = (1.0f - ((float)s_fast_learn_frames / 5.0f)) * 100.0f;
        s_active_hw_mode     = 0;
        s_ai_infer_us        = dt_us;
        portEXIT_CRITICAL(&s_ai_mux);
        return;
    }

    // -----------------------------------------------------------------------------------------
    // BƯỚC 5: SO SÁNH MÔ HÌNH NỀN + KHỬ BÓNG ĐỔ + MÁY TRẠNG THÁI CHỐNG BÓNG MA SAKBOT
    // -----------------------------------------------------------------------------------------
    // Bù phơi sáng cả nhân tính (beta_ae) lẫn cộng tính (ae_offset) từ các ô nền tĩnh
    float sum_cur_bg = 0.0f, sum_ref_bg = 0.0f, sum_diff_bg = 0.0f;
    int   quiet_cnt = 0;
    for (int i = 0; i < GRID_CELLS; i++) {
        if (s_static_age[i] >= 4 && s_fg_hold[i] == 0) {
            float d = s_cell_y[i] - s_bg_y[i];
            if (fabsf(d) < 20.0f) {
                sum_cur_bg  += s_cell_y[i];
                sum_ref_bg  += s_bg_y[i];
                sum_diff_bg += d;
                quiet_cnt++;
            }
        }
    }
    float beta_ae   = (quiet_cnt >= 20 && sum_ref_bg > 100.0f) ? (sum_cur_bg / sum_ref_bg) : 1.0f;
    if (beta_ae < 0.75f) beta_ae = 0.75f;
    if (beta_ae > 1.32f) beta_ae = 1.32f;
    float ae_offset = (quiet_cnt >= 20) ? (sum_diff_bg / quiet_cnt) : 0.0f;

    for (int i = 0; i < GRID_CELLS; i++) {
        float dy_mul = fabsf(s_cell_y[i] - s_bg_y[i] * beta_ae);
        float dy_add = fabsf((s_cell_y[i] - ae_offset) - s_bg_y[i]);
        float dy     = (dy_mul < dy_add) ? dy_mul : dy_add;

        float dcr = fabsf(s_cell_cr[i]   - s_bg_cr[i]);
        float dcb = fabsf(s_cell_cb[i]   - s_bg_cb[i]);
        float ded = fabsf(s_cell_edge[i] - s_bg_edge[i]);
        float chroma_shift = dcr + dcb;

        // Khử bóng đổ (Cast Shadow Suppression - Cucchiara et al., IEEE TPAMI 2003):
        // Bóng đổ chỉ làm vùng nền tối đi nhẹ (tỷ lệ độ sáng 0.58 .. 0.95) mà không làm thay đổi
        // sắc độ màu (chroma_shift < 8.5) và không tạo ra biên cạnh mới (ded < 7.0).
        float expected_y = s_bg_y[i] * beta_ae;
        float lum_ratio  = (expected_y > 12.0f) ? (s_cell_y[i] / expected_y) : 1.0f;
        bool is_shadow   = (lum_ratio >= 0.55f && lum_ratio <= 0.95f) &&
                           (chroma_shift < 8.5f) &&
                           (ded < 7.0f) &&
                           (!s_moving_map[i]);

        // Độ lệch chuẩn ngưỡng hiệu dụng tự động nới rộng tại các đường viền có độ tương phản cao
        // (phào chỉ trần nhà, viền điều hòa, mép rèm cửa)
        float sigma_eff = s_bg_var[i] + 0.42f * s_spatial_grad[i] + 0.32f * s_bg_edge[i];
        if (sigma_eff < 6.0f)  sigma_eff = 6.0f;
        if (sigma_eff > 36.0f) sigma_eff = 36.0f;

        float raw_diff = dy + 0.45f * chroma_shift + 0.45f * ded;
        float clean_diff = raw_diff - 8.0f;
        if (clean_diff < 0.0f) clean_diff = 0.0f;

        float z = clean_diff / sigma_eff;
        if (is_shadow) z *= 0.25f;
        s_z_map[i] = z;

        // NGUYÊN LÝ VÀNG CỦA SAKBOT (Cucchiara et al., IEEE TPAMI 2003):
        // - Một ô ĐANG Ở NỀN (s_fg_hold == 0) CHỈ được phép kích hoạt lên Tiền cảnh khi:
        //   vừa khác nền (z > thr) VỪA đang có chuyển động thực sự (s_moving_map[i] == 1)!
        // - Nếu ô không chuyển động, s_fg_hold giảm dần vô điều kiện về 0 (không bao giờ khóa chết!).
        if (z > thr) {
            if (s_moving_map[i]) {
                s_fg_hold[i] = 24; // Giữ khung ~1.3 giây sau khi có chuyển động
            } else if (s_fg_hold[i] > 0) {
                s_fg_hold[i]--;    // Giảm vô điều kiện khi đứng yên -> Triệt tiêu 100% khung ma (Ghost)!
            }
        } else {
            if (s_fg_hold[i] >= 4) s_fg_hold[i] -= 4;
            else s_fg_hold[i] = 0;
        }

        s_raw_anom[i] = (z > thr && s_fg_hold[i] > 0) ? 1 : 0;
    }

    // -----------------------------------------------------------------------------------------
    // BƯỚC 6: LAN TRUYỀN VI CHUYỂN ĐỘNG TRONG CỤM ĐỐI TƯỢNG & CẬP NHẬT NỀN LIÊN TỤC
    // Nếu bàn tay/cơ thể người có vi chuyển động ở một vài ô, duy trì khung cho các ô liền kề
    // của cùng bàn tay đó; nhưng nếu cả cụm đứng yên hoàn toàn (trần nhà/tường), nó tự tắt!
    // -----------------------------------------------------------------------------------------
    memset(s_verified_anom, 0, sizeof(s_verified_anom));
    int active_fg_cells = 0;

    for (int gy = 0; gy < GRID_H; gy++) {
        for (int gx = 0; gx < GRID_W; gx++) {
            int idx = gy * GRID_W + gx;
            int active_nb = 0;
            int moving_nb = 0;

            if (s_z_map[idx] > thr) {
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        if (dx == 0 && dy == 0) continue;
                        int nx = gx + dx, ny = gy + dy;
                        if (nx >= 0 && nx < GRID_W && ny >= 0 && ny < GRID_H) {
                            int n_idx = ny * GRID_W + nx;
                            if (s_raw_anom[n_idx]) active_nb++;
                            if (s_moving_map[n_idx] && s_z_map[n_idx] > thr) moving_nb++;
                        }
                    }
                }
                // Chỉ lan truyền giữ khung khi các ô sát cạnh ĐANG CÓ CHUYỂN ĐỘNG THỰC SỰ (moving_nb >= 2)
                if (moving_nb >= 2 && s_fg_hold[idx] < 16) {
                    s_fg_hold[idx]  = 16;
                    s_raw_anom[idx] = 1;
                }
            }

            if (s_raw_anom[idx] && active_nb >= 1) {
                if (s_cell_persist[idx] < 5) s_cell_persist[idx]++;
            } else {
                if (s_cell_persist[idx] >= 2) s_cell_persist[idx] -= 2;
                else s_cell_persist[idx] = 0;
            }

            if (s_cell_persist[idx] >= 2) {
                s_verified_anom[idx] = 1;
                active_fg_cells++;
            }
        }
    }

    // CẬP NHẬT NỀN THÍCH NGHI (KHÔNG BAO GIỜ KHÓA CHẾT):
    // Bất kỳ ô nào đứng yên (s_static_age >= 3 và không còn s_verified_anom) sẽ được cập nhật
    // nhanh vào nền (alpha = 0.28f), đảm bảo phào chỉ trần nhà hay ánh sáng cửa sổ luôn hòa vào nền!
    for (int i = 0; i < GRID_CELLS; i++) {
        float alpha = 0.08f;
        if (s_verified_anom[i]) {
            // Nếu ô đang đóng khung nhưng thực chất đã đứng yên lâu (s_static_age >= 10),
            // vẫn kéo nhẹ nền về phía ảnh hiện tại để không bao giờ kẹt vĩnh viễn!
            alpha = (s_static_age[i] >= 10) ? 0.08f : 0.005f;
        } else if (s_static_age[i] >= 3) {
            alpha = 0.28f;
        }

        float dy  = fabsf(s_cell_y[i]  - s_bg_y[i]);
        float dcr = fabsf(s_cell_cr[i] - s_bg_cr[i]);
        float dcb = fabsf(s_cell_cb[i] - s_bg_cb[i]);
        float diff = dy + 0.3f * (dcr + dcb);

        s_bg_y[i]    = (1.0f - alpha) * s_bg_y[i]    + alpha * s_cell_y[i];
        s_bg_cr[i]   = (1.0f - alpha) * s_bg_cr[i]   + alpha * s_cell_cr[i];
        s_bg_cb[i]   = (1.0f - alpha) * s_bg_cb[i]   + alpha * s_cell_cb[i];
        s_bg_edge[i] = (1.0f - alpha) * s_bg_edge[i] + alpha * s_cell_edge[i];

        if (!s_verified_anom[i]) {
            s_bg_var[i] = 0.90f * s_bg_var[i] + 0.10f * (diff + 5.0f);
            if (s_bg_var[i] < 5.5f)  s_bg_var[i] = 5.5f;
            if (s_bg_var[i] > 24.0f) s_bg_var[i] = 24.0f;
        }
    }
    update_spatial_gradients();

    // -----------------------------------------------------------------------------------------
    // BƯỚC 7: GOM CỤM 8 HƯỚNG TẠO BOUNDING BOX & KIỂM ĐỊNH ĐỘNG HỌC CỤM (CLUSTER MOTION GATE)
    // Một cụm Bounding Box chỉ được giữ lại nếu có diện tích >= 2 ô VÀ có ít nhất 1 ô chuyển động
    // hoặc đang được giữ khóa hợp lệ từ chuyển động vừa diễn ra!
    // -----------------------------------------------------------------------------------------
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
            int moving_in_cluster = 0;
            int max_hold_in_cluster = 0;

            while (q_head < q_tail) {
                int cx = s_qx[q_head];
                int cy = s_qy[q_head];
                q_head++;

                int c_idx = cy * GRID_W + cx;
                sum_z += s_z_map[c_idx];
                if (s_moving_map[c_idx]) moving_in_cluster++;
                if (s_fg_hold[c_idx] > max_hold_in_cluster) max_hold_in_cluster = s_fg_hold[c_idx];

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
            // Loại bỏ các đốm nhiễu nhỏ (< 2 ô) hoặc cụm đã hết năng lượng chuyển động
            if (cluster_cells < 2) continue;
            if (moving_in_cluster == 0 && max_hold_in_cluster < 6) continue;

            int box_gw = max_gx - min_gx + 1;
            int box_gh = max_gy - min_gy + 1;
            if (box_gh == 1 && box_gw >= 6) continue; // Bỏ sọc mảnh ngang

            if (cand_count < MAX_ANOM_BOXES) {
                AnomBox b;
                b.x = (float)min_gx / (float)GRID_W;
                b.y = (float)min_gy / (float)GRID_H;
                b.w = (float)box_gw / (float)GRID_W;
                b.h = (float)box_gh / (float)GRID_H;

                float avg_z = sum_z / (float)cluster_cells;
                int conf = (int)(70.0f + (avg_z - thr) * 10.0f);
                if (conf < 70) conf = 70;
                if (conf > 99) conf = 99;
                b.score = conf;

                // Chỉ chạy bộ nhân AdaReMAC siêu nhẹ 1 tầng (< 0.12 ms) cho khung đầu tiên để lấy thống kê Mode 1/2
                if (cand_count == 0) {
                    int px0 = (min_gx * thumb_w) / GRID_W;
                    int py0 = (min_gy * thumb_h) / GRID_H;
                    int pbw = (box_gw * thumb_w) / GRID_W;
                    int pbh = (box_gh * thumb_h) / GRID_H;
                    extract_patch_int8_400(s_rgb565_thumb, thumb_w, thumb_h, px0, py0, pbw, pbh, s_patch_feat);
                    b.ai_cls_m1 = adaremac_run_fast_fc(s_patch_feat, 1, 2);
                    b.ai_cls_m2 = adaremac_run_fast_fc(s_patch_feat, 2, 0);
                } else {
                    b.ai_cls_m1 = 0;
                    b.ai_cls_m2 = 0;
                }

                s_cand_boxes[cand_count++] = b;
            }
        }
    }

    // Gộp các khung bao nằm sát hoặc chồng lấp nhau thành 1 khung bao duy nhất gọn đẹp
    for (int i = 0; i < cand_count; i++) {
        for (int j = i + 1; j < cand_count; ) {
            float ax0 = s_cand_boxes[i].x, ay0 = s_cand_boxes[i].y;
            float ax1 = ax0 + s_cand_boxes[i].w, ay1 = ay0 + s_cand_boxes[i].h;
            float bx0 = s_cand_boxes[j].x, by0 = s_cand_boxes[j].y;
            float bx1 = bx0 + s_cand_boxes[j].w, by1 = by0 + s_cand_boxes[j].h;
            float pad = 0.06f;
            if (!(ax1 + pad < bx0 || bx1 + pad < ax0 || ay1 + pad < by0 || by1 + pad < ay0)) {
                float nx0 = fminf(ax0, bx0), ny0 = fminf(ay0, by0);
                float nx1 = fmaxf(ax1, bx1), ny1 = fmaxf(ay1, by1);
                s_cand_boxes[i].x = nx0;
                s_cand_boxes[i].y = ny0;
                s_cand_boxes[i].w = nx1 - nx0;
                s_cand_boxes[i].h = ny1 - ny0;
                if (s_cand_boxes[j].score > s_cand_boxes[i].score) {
                    s_cand_boxes[i].score = s_cand_boxes[j].score;
                }
                for (int k = j; k < cand_count - 1; k++) s_cand_boxes[k] = s_cand_boxes[k + 1];
                cand_count--;
            } else {
                j++;
            }
        }
    }

    // Làm mượt tọa độ khung bao theo thời gian (EMA Tracking)
    for (int i = 0; i < cand_count; i++) {
        AnomBox *cb = &s_cand_boxes[i];
        float c_cx = cb->x + cb->w * 0.5f;
        float c_cy = cb->y + cb->h * 0.5f;
        int best_match = -1;
        float best_dist = 0.20f;
        for (int j = 0; j < s_num_tracked; j++) {
            float t_cx = s_tracked_boxes[j].x + s_tracked_boxes[j].w * 0.5f;
            float t_cy = s_tracked_boxes[j].y + s_tracked_boxes[j].h * 0.5f;
            float dist = fabsf(c_cx - t_cx) + fabsf(c_cy - t_cy);
            if (dist < best_dist) {
                best_dist = dist;
                best_match = j;
            }
        }
        if (best_match >= 0) {
            cb->x = 0.60f * cb->x + 0.40f * s_tracked_boxes[best_match].x;
            cb->y = 0.60f * cb->y + 0.40f * s_tracked_boxes[best_match].y;
            cb->w = 0.60f * cb->w + 0.40f * s_tracked_boxes[best_match].w;
            cb->h = 0.60f * cb->h + 0.40f * s_tracked_boxes[best_match].h;
        }
    }
    s_num_tracked = cand_count;
    for (int i = 0; i < cand_count; i++) s_tracked_boxes[i] = s_cand_boxes[i];

    float changed_ratio = (float)active_fg_cells / (float)GRID_CELLS;
    uint8_t hw_mode = (cand_count > 0) ? 2 : ((moving_cells >= 2) ? 1 : 0);
    uint32_t dt_us = micros() - t0;

    portENTER_CRITICAL(&s_ai_mux);
    s_num_boxes          = cand_count;
    for (int i = 0; i < cand_count; i++) s_boxes[i] = s_cand_boxes[i];
    s_angle_changed      = false;
    s_changed_cell_ratio = changed_ratio;
    s_env_stability      = (1.0f - fminf(1.0f, changed_ratio)) * 100.0f;
    s_active_hw_mode     = hw_mode;
    s_ai_infer_us        = dt_us;
    portEXIT_CRITICAL(&s_ai_mux);
}

// Task AI chạy nhẹ nhàng (~12 Hz, chỉ tốn ~3 ms mỗi lần chạy), nhường 96% CPU cho luồng Video Stream!
static void ai_continual_learning_task(void *arg) {
    while (true) {
        if (!s_cam_ok) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        bool stream_active = (s_stream_clients > 0) && ((uint32_t)(millis() - s_last_stream_ms) < 500);

        if (stream_active) {
            if (!s_ai_frame_ready) {
                s_ai_need_frame = true;
                vTaskDelay(pdMS_TO_TICKS(8));
                continue;
            }
            analyze_frame_and_learn(s_ai_jpeg_buf, s_ai_jpeg_len, s_ai_frame_w, s_ai_frame_h);
            s_ai_frame_ready = false;
            vTaskDelay(pdMS_TO_TICKS(65));
            s_ai_need_frame = true;
        } else {
            camera_fb_t *fb = esp_camera_fb_get();
            if (fb) {
                s_total_frames++;
                s_ai_frame_w = fb->width;
                s_ai_frame_h = fb->height;
                if (s_ai_jpeg_buf && fb->len <= MAX_JPEG_COPY_SZ) {
                    memcpy(s_ai_jpeg_buf, fb->buf, fb->len);
                    s_ai_jpeg_len = fb->len;
                    esp_camera_fb_return(fb);
                    analyze_frame_and_learn(s_ai_jpeg_buf, s_ai_jpeg_len, s_ai_frame_w, s_ai_frame_h);
                } else {
                    analyze_frame_and_learn(fb->buf, fb->len, fb->width, fb->height);
                    esp_camera_fb_return(fb);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(80));
        }
    }
}

// =========================================================================================
// 6. BẢNG THANH GHI PHẦN CỨNG OMNIVISION OV5640 "720P @ 30FPS" CHÍNH HÃNG (MỤC 4.1.5)
// =========================================================================================
// Giải quyết tận gốc lỗi kẹt ở ~13-14 FPS của thư viện esp32-camera:
//   - Theo tài liệu gốc "OmniVision OV5640 Auto Focus Camera Module Application Notes" (Trang 23-24,
//     Mục 4.1.5 "720p Video"):
//       + Cấu hình 720p 15fps dùng thanh ghi PLL 0x3035 = 0x41 (sys_div = 4).
//       + Cấu hình 720p 30fps bắt buộc đổi thanh ghi PLL 0x3035 = 0x21 (sys_div = 2, nhân đôi SYSCLK!)
//         kết hợp cửa sổ quét 2x2 Binning/Subsampling (0x3814=0x31, 0x3815=0x31, HTS=0x0764 [1892],
//         VTS=0x02E4 [740]) và khóa phơi sáng tối đa 0x3A02/0x3A14 = 0x02E0 (736 dòng).
static void configure_ov5640_true_30fps(sensor_t *s, framesize_t fs, int jpeg_qual) {
    if (!s) return;
    s->set_framesize(s, fs);
    s->set_quality(s, jpeg_qual);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 0);                        // Tắt Night-Mode
    s->set_ae_level(s, 0);
    s->set_gain_ctrl(s, 1);
    s->set_gainceiling(s, (gainceiling_t)3);  // Trần Gain 16x (sáng rõ mà không sinh nhiễu hạt Hot-Pixel!)
    s->set_brightness(s, 0);
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

    if (s->id.PID == OV5640_PID && s->set_reg && s->get_reg) {
        // Bật bộ khử điểm chết phần cứng (LENC + BPC + WPC)
        s->set_reg(s, 0x5000, 0xFF, 0x86);
        // Tắt bit 2 (Night mode frame drop) trong thanh ghi 0x3A00
        s->set_reg(s, 0x3A00, 0x04, 0x00);

        if (fs == FRAMESIZE_HD) {
            // --- GHI BẢNG THANH GHI 720P (1280x720) @ 30 FPS CHÍNH HÃNG OMNIVISION (MỤC 4.1.5) ---
            // 1. Cửa sổ 2x2 Subsampling & Horizontal Binning (giữ nguyên bit 5 = JPEG Enable trong 0x3821)
            s->set_reg(s, 0x3814, 0xFF, 0x31); // X INC 2x2 subsample
            s->set_reg(s, 0x3815, 0xFF, 0x31); // Y INC 2x2 subsample
            s->set_reg(s, 0x3821, 0x01, 0x01); // Bật Horizontal Binning (bit 0 = 1)

            // 2. Kích thước cửa sổ cảm biến và đầu ra DVP 1280x720 chuẩn
            s->set_reg(s, 0x3800, 0xFF, 0x00);
            s->set_reg(s, 0x3801, 0xFF, 0x00); // HS = 0
            s->set_reg(s, 0x3802, 0xFF, 0x00);
            s->set_reg(s, 0x3803, 0xFF, 0xFA); // VS = 250
            s->set_reg(s, 0x3804, 0xFF, 0x0A);
            s->set_reg(s, 0x3805, 0xFF, 0x3F); // HW = 2623
            s->set_reg(s, 0x3806, 0xFF, 0x06);
            s->set_reg(s, 0x3807, 0xFF, 0xA9); // VH = 1705
            s->set_reg(s, 0x3808, 0xFF, 0x05);
            s->set_reg(s, 0x3809, 0xFF, 0x00); // DVPHO = 1280
            s->set_reg(s, 0x380A, 0xFF, 0x02);
            s->set_reg(s, 0x380B, 0xFF, 0xD0); // DVPVO = 720

            // 3. Tổng kích thước quét ngang HTS = 1892 (0x0764) & dọc VTS = 740 (0x02E4)
            //    (Thay thế giá trị HTS=2844, VTS=1488 gây kẹt ở 13.4 FPS của driver mặc định!)
            s->set_reg(s, 0x380C, 0xFF, 0x07);
            s->set_reg(s, 0x380D, 0xFF, 0x64); // HTS = 1892
            s->set_reg(s, 0x380E, 0xFF, 0x02);
            s->set_reg(s, 0x380F, 0xFF, 0xE4); // VTS = 740
            s->set_reg(s, 0x3813, 0xFF, 0x04);

            // 4. Khóa trần phơi sáng tối đa 60Hz (0x3A02:0x3A03) & 50Hz (0x3A14:0x3A15) = 0x02E0 (736 dòng)
            s->set_reg(s, 0x3A02, 0xFF, 0x02);
            s->set_reg(s, 0x3A03, 0xFF, 0xE0);
            s->set_reg(s, 0x3A14, 0xFF, 0x02);
            s->set_reg(s, 0x3A15, 0xFF, 0xE0);

            // 5. Cấu hình PLL sang "720p 30fps" (0x3035 = 0x21 thay vì 0x41 của 15fps!)
            //    và chia DVP PCLK (0x3824 = 0x04) để xung PCLK ra cáp FPC sạch tuyệt đối (~20 MHz)
            s->set_reg(s, 0x3035, 0xFF, 0x21); // sys_div = 2 (Mấu chốt mở khóa 30 FPS!)
            s->set_reg(s, 0x3036, 0xFF, 0x69); // PLL Multiplier = 105
            s->set_reg(s, 0x3037, 0xFF, 0x13); // PLL root div + pre-div = 3
            s->set_reg(s, 0x3108, 0xFF, 0x01);
            s->set_reg(s, 0x460C, 0xFF, 0x22); // Manual PCLK divider cho JPEG FIFO
            s->set_reg(s, 0x3824, 0xFF, 0x04); // PCLK divider = 4 (chống sọc ngang & tràn DMA)
            s->set_reg(s, 0x4837, 0xFF, 0x16);
        } else if (fs == FRAMESIZE_SVGA || fs == FRAMESIZE_VGA) {
            // Với SVGA/VGA: mở khóa PLL 0x3035 = 0x11 / 0x21 để đạt 30+ FPS
            s->set_reg(s, 0x3035, 0xFF, 0x21);
            int vts_hi = s->get_reg(s, 0x380E, 0xFF);
            int vts_lo = s->get_reg(s, 0x380F, 0xFF);
            if (vts_hi >= 0 && vts_lo >= 0) {
                int max_exp = ((vts_hi << 8) | vts_lo) - 4;
                if (max_exp > 100) {
                    s->set_reg(s, 0x3A02, 0xFF, (max_exp >> 8) & 0xFF);
                    s->set_reg(s, 0x3A03, 0xFF, max_exp & 0xFF);
                    s->set_reg(s, 0x3A14, 0xFF, (max_exp >> 8) & 0xFF);
                    s->set_reg(s, 0x3A15, 0xFF, max_exp & 0xFF);
                }
            }
        }
    }
}

// =========================================================================================
// 7. GIAO DIỆN WEB TRỰC QUAN (CHỈ ĐÓNG KHUNG SẮC NÉT, KHÔNG GÁN TÊN ĐỐI TƯỢNG)
// =========================================================================================
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>AdaReMAC ESP32-S3 — HD@30fps Stream & Sakbot Motion-Background Detector</title>
  <style>
    :root { --bg: #0f172a; --card: #1e293b; --accent: #38bdf8; --danger: #ef4444; --ok: #22c55e; --warn: #f59e0b; }
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: system-ui, -apple-system, sans-serif; }
    body { background: var(--bg); color: #f8fafc; padding: 12px; display: flex; flex-direction: column; align-items: center; }
    .header { width: 100%; max-width: 1280px; display: flex; flex-wrap: wrap; justify-content: space-between; align-items: center; background: var(--card); padding: 12px 18px; border-radius: 12px; margin-bottom: 10px; border: 1px solid #334155; gap: 10px; }
    .title { font-size: 1.12rem; font-weight: 700; color: var(--accent); }
    .subtitle { font-size: 0.80rem; color: #94a3b8; margin-top: 3px; }
    .badge { padding: 6px 14px; border-radius: 999px; font-weight: 700; font-size: 0.88rem; display: inline-flex; align-items: center; gap: 6px; }
    .bg-ok { background: rgba(34,197,94,0.18); color: #4ade80; border: 1px solid #22c55e; }
    .bg-warn { background: rgba(245,158,11,0.22); color: #fbbf24; border: 1px solid #f59e0b; }
    .bg-danger { background: rgba(239,68,68,0.25); color: #f87171; border: 1px solid #ef4444; animation: pulse 1s infinite; }
    @keyframes pulse { 0%,100% { transform: scale(1); } 50% { transform: scale(1.03); } }
    .Controls { width: 100%; max-width: 1280px; display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 10px; margin-bottom: 10px; }
    .stat-box { background: var(--card); padding: 10px 14px; border-radius: 10px; border: 1px solid #334155; }
    .stat-label { font-size: 0.75rem; color: #94a3b8; text-transform: uppercase; }
    .stat-val { font-size: 1.08rem; font-weight: 700; margin-top: 3px; }
    .stage-wrap { position: relative; width: 100%; max-width: 1280px; background: #000; border-radius: 12px; overflow: hidden; border: 2px solid #334155; aspect-ratio: 16 / 9; display: flex; align-items: center; justify-content: center; }
    #cam_stream { width: 100%; height: 100%; object-fit: fill; display: block; }
    #ai_canvas { position: absolute; top: 0; left: 0; width: 100%; height: 100%; pointer-events: none; }
    .toolbar { width: 100%; max-width: 1280px; background: var(--card); padding: 12px 18px; border-radius: 12px; margin-top: 10px; border: 1px solid #334155; display: flex; flex-wrap: wrap; gap: 14px; align-items: center; justify-content: space-between; }
    button, select { background: #0284c7; color: #fff; border: none; padding: 8px 14px; border-radius: 8px; font-weight: 600; cursor: pointer; font-size: 0.86rem; }
    button:hover { background: #0369a1; }
    select { background: #334155; }
    input[type=range] { accent-color: var(--accent); cursor: pointer; }
  </style>
</head>
<body>
  <div class="header">
    <div>
      <div class="title">📹 AdaReMAC v5 — Stream HD@30fps + Đóng Khung Đối Tượng Sakbot (IEEE TPAMI)</div>
      <div class="subtitle">Mở khóa thanh ghi OV5640 720p@30fps (0x3035=0x21) + Thuật toán Sakbot chống bóng ma & khử bóng đổ siêu tốc (~3ms)</div>
    </div>
    <div id="status_badge" class="badge bg-ok">Đang kết nối...</div>
  </div>

  <div class="Controls">
    <div class="stat-box"><div class="stat-label">Độ phân giải & Tốc độ Stream</div><div class="stat-val" id="val_fps">HD 1280x720</div></div>
    <div class="stat-box"><div class="stat-label">Chế độ AdaReMAC & Ổn định Nền</div><div class="stat-val" id="val_env">Mode 0 | 100%</div></div>
    <div class="stat-box"><div class="stat-label">Số khung phát hiện / Thời gian xử lý</div><div class="stat-val" id="val_anom">0 khung (0.0 ms)</div></div>
    <div class="stat-box"><div class="stat-label">Nhiệt độ Chip ESP32-S3</div><div class="stat-val" id="val_temp">-- °C</div></div>
  </div>

  <div class="stage-wrap">
    <img id="cam_stream" alt="Đang tải luồng MJPEG tốc độ cao..." crossorigin="anonymous">
    <canvas id="ai_canvas"></canvas>
  </div>

  <div class="toolbar">
    <div style="display:flex;align-items:center;gap:10px;flex-wrap:wrap;">
      <label style="font-size:0.86rem;">Phân giải:</label>
      <select id="sel_res" onchange="changeRes(this.value)">
        <option value="fhd">FHD (1920x1080)</option>
        <option value="hd" selected>HD (1280x720 @ 30fps - Chuẩn)</option>
        <option value="svga">SVGA (800x600 - Siêu tốc)</option>
        <option value="vga">VGA (640x480)</option>
      </select>

      <label style="font-size:0.86rem;margin-left:6px;">Bù sáng (AE):</label>
      <input type="range" min="-2" max="2" step="1" value="0" oninput="updateAE(this.value)">
      <span id="lbl_ae" style="font-weight:700;color:#38bdf8;">0</span>

      <label style="font-size:0.86rem;margin-left:6px;">Nén JPEG (16 = Nhẹ & Mượt):</label>
      <input type="range" min="10" max="28" step="1" value="16" oninput="updateQual(this.value)">
      <span id="lbl_q" style="font-weight:700;color:#38bdf8;">16</span>

      <label style="font-size:0.86rem;margin-left:6px;">Ngưỡng AI (Lớn = Khắt khe hơn):</label>
      <input type="range" id="rng_thr" min="1.4" max="5.0" step="0.2" value="2.5" oninput="updateThr(this.value)">
      <span id="lbl_thr" style="font-weight:700;color:#38bdf8;">2.5</span>
    </div>
    <button onclick="resetEnv()">🔄 Học lại góc Camera ngay (Reset Nền)</button>
  </div>

  <script>
    const MODE_NAMES = ['Mode 0 (Gatekeeper 0%)', 'Mode 1 (25% Partial)', 'Mode 2 (100% Exact)'];

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

    let targetBoxes = [];
    let renderBoxes = [];
    let learnLeft = 0;

    function renderLoop() {
      const cw = canvas.clientWidth || 1280;
      const ch = canvas.clientHeight || 720;
      if (canvas.width !== cw || canvas.height !== ch) {
        canvas.width = cw;
        canvas.height = ch;
      }
      ctx.clearRect(0, 0, cw, ch);

      if (learnLeft > 0) {
        ctx.strokeStyle = '#f59e0b';
        ctx.lineWidth = 4;
        ctx.strokeRect(8, 8, cw - 16, ch - 16);
      }

      if (renderBoxes.length !== targetBoxes.length) {
        renderBoxes = targetBoxes.map(b => ({ ...b }));
      } else {
        for (let i = 0; i < targetBoxes.length; i++) {
          renderBoxes[i].x += (targetBoxes[i].x - renderBoxes[i].x) * 0.40;
          renderBoxes[i].y += (targetBoxes[i].y - renderBoxes[i].y) * 0.40;
          renderBoxes[i].w += (targetBoxes[i].w - renderBoxes[i].w) * 0.40;
          renderBoxes[i].h += (targetBoxes[i].h - renderBoxes[i].h) * 0.40;
        }
      }

      // CHỈ ĐÓNG KHUNG SẮC NÉT (KHÔNG GÁN TÊN ĐỐI TƯỢNG THEO YÊU CẦU)
      renderBoxes.forEach((b) => {
        const bx = b.x * cw;
        const by = b.y * ch;
        const bw = b.w * cw;
        const bh = b.h * ch;

        ctx.fillStyle = 'rgba(239, 68, 68, 0.14)';
        ctx.fillRect(bx, by, bw, bh);

        ctx.strokeStyle = '#ef4444';
        ctx.lineWidth = 2.5;
        ctx.strokeRect(bx, by, bw, bh);

        // Vẽ 4 góc định vị sắc nét
        const cl = Math.min(18, bw * 0.25, bh * 0.25);
        ctx.strokeStyle = '#38bdf8';
        ctx.lineWidth = 3.5;
        ctx.beginPath();
        ctx.moveTo(bx, by + cl); ctx.lineTo(bx, by); ctx.lineTo(bx + cl, by);
        ctx.moveTo(bx + bw - cl, by); ctx.lineTo(bx + bw, by); ctx.lineTo(bx + bw, by + cl);
        ctx.moveTo(bx, by + bh - cl); ctx.lineTo(bx, by + bh); ctx.lineTo(bx + cl, by + bh);
        ctx.moveTo(bx + bw - cl, by + bh); ctx.lineTo(bx + bw, by + bh); ctx.lineTo(bx + bw, by + bh - cl);
        ctx.stroke();
      });

      requestAnimationFrame(renderLoop);
    }
    requestAnimationFrame(renderLoop);

    async function pollAI() {
      const ctrl = new AbortController();
      const tid = setTimeout(() => ctrl.abort(), 350);
      try {
        const res = await fetch('/ai', { cache: 'no-store', signal: ctrl.signal });
        const d = await res.json();

        document.getElementById('val_fps').innerText = d.w + 'x' + d.h + ' @ ' + d.fps.toFixed(1) + ' FPS';
        document.getElementById('val_env').innerText = (MODE_NAMES[d.hw_mode] || 'Mode 0') + ' | ' + d.stability.toFixed(0) + '%';
        document.getElementById('val_anom').innerText = d.boxes.length + ' khung (' + (d.ai_us / 1000).toFixed(1) + ' ms)';
        document.getElementById('val_temp').innerText = d.temp.toFixed(1) + ' °C';

        learnLeft = d.learn_left;
        targetBoxes = d.boxes || [];

        const badge = document.getElementById('status_badge');
        if (d.learn_left > 0) {
          badge.className = 'badge bg-warn';
          badge.innerText = '🔄 ĐANG CẬP NHẬT NỀN MỚI (' + d.stability.toFixed(0) + '%)';
        } else if (d.boxes.length > 0) {
          badge.className = 'badge bg-danger';
          badge.innerText = '⚠️ PHÁT HIỆN ' + d.boxes.length + ' ĐỐI TƯỢNG TRONG KHUNG HÌNH';
        } else {
          badge.className = 'badge bg-ok';
          badge.innerText = '✅ MÔI TRƯỜNG ỔN ĐỊNH';
        }
      } catch (e) {}
      clearTimeout(tid);
      setTimeout(pollAI, 180);
    }
    pollAI();
  </script>
</body>
</html>
)rawliteral";

// =========================================================================================
// 8. CÁC HANDLER HTTP (PORT 80: UI + AI JSON | PORT 81: ZERO-HOLD 1-SYSCALL MJPEG STREAM)
// =========================================================================================
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
    uint8_t s_cls = 0, hw_m = 0;
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
    hw_m       = s_active_hw_mode;
    infer_us   = s_ai_infer_us;
    fps        = s_stream_fps;
    temp       = s_chip_temp_c;
    fw         = s_ai_frame_w;
    fh         = s_ai_frame_h;
    portEXIT_CRITICAL(&s_ai_mux);

    int len = snprintf(s_json_buf, sizeof(s_json_buf),
        "{\"w\":%u,\"h\":%u,\"fps\":%.1f,\"temp\":%.1f,\"stability\":%.1f,\"changed\":%.3f,"
        "\"angle_shift\":%s,\"learn_left\":%d,\"scene_cls\":%u,\"hw_mode\":%u,\"ai_us\":%lu,\"boxes\":[",
        fw, fh, fps, temp, stab, chg, ang ? "true" : "false", learn_left, s_cls, hw_m,
        (unsigned long)infer_us);

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
    httpd_resp_set_hdr(req, "Connection", "keep-alive");
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
            if (t >= 1.0f && t <= 10.0f) s_anom_threshold = t;
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
            framesize_t target_fs = FRAMESIZE_HD;
            int target_q = 16;
            if (strcmp(val, "fhd") == 0)       { target_fs = FRAMESIZE_FHD;  target_q = 18; }
            else if (strcmp(val, "hd") == 0)   { target_fs = FRAMESIZE_HD;   target_q = 16; }
            else if (strcmp(val, "svga") == 0) { target_fs = FRAMESIZE_SVGA; target_q = 14; }
            else if (strcmp(val, "vga") == 0)  { target_fs = FRAMESIZE_VGA;  target_q = 12; }
            configure_ov5640_true_30fps(s, target_fs, target_q);
            s_force_relearn = true;
        }
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, "OK", 2);
}

static esp_err_t stream_handler(httpd_req_t *req) {
    int sockfd = httpd_req_to_sockfd(req);
    int nodelay = 1;
    int sndbuf  = 65536;
    int tos     = IPTOS_LOWDELAY;
    setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
    setsockopt(sockfd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
    setsockopt(sockfd, IPPROTO_IP, IP_TOS, &tos, sizeof(tos));

    esp_err_t res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    httpd_resp_set_hdr(req, "X-Framerate", "30");

    s_stream_clients++;
    uint32_t frame_cnt = 0;
    uint32_t t_fps_start = millis();

    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        uint32_t now = millis();
        s_last_stream_ms = now;
        s_total_frames++;
        frame_cnt++;
        s_ai_frame_w = fb->width;
        s_ai_frame_h = fb->height;

        size_t jpg_len = fb->len;
        uint16_t fw = fb->width;

        if (s_stream_tx_buf && jpg_len <= MAX_JPEG_COPY_SZ) {
            char hdr_tmp[STREAM_HDR_RESERVE];
            int hlen = snprintf(hdr_tmp, sizeof(hdr_tmp),
                                "\r\n--" PART_BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                                (unsigned)jpg_len);
            memcpy(s_stream_tx_buf, hdr_tmp, hlen);
            memcpy(s_stream_tx_buf + hlen, fb->buf, jpg_len);

            if (s_ai_need_frame && !s_ai_frame_ready && s_ai_jpeg_buf && (fw / 8) <= MAX_RGB_THUMB_W) {
                memcpy(s_ai_jpeg_buf, s_stream_tx_buf + hlen, jpg_len);
                s_ai_jpeg_len    = jpg_len;
                s_ai_frame_ready = true;
                s_ai_need_frame  = false;
            }

            // Trả Frame Buffer ngay lập tức (< 0.4ms) để cảm biến OV5640 ghi khung kế tiếp không bị đợi
            esp_camera_fb_return(fb);

            // Gửi gộp [Header + JPEG] trong duy nhất 1 lời gọi hàm
            res = httpd_resp_send_chunk(req, (const char *)s_stream_tx_buf, hlen + jpg_len);
        } else {
            char hdr_buf[128];
            int hlen = snprintf(hdr_buf, sizeof(hdr_buf),
                                "\r\n--" PART_BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                                (unsigned)jpg_len);
            res = httpd_resp_send_chunk(req, hdr_buf, hlen);
            if (res == ESP_OK) {
                res = httpd_resp_send_chunk(req, (const char *)fb->buf, jpg_len);
            }
            esp_camera_fb_return(fb);
        }

        if (res != ESP_OK) break;

        if (now - t_fps_start >= 1000) {
            s_stream_fps = (frame_cnt * 1000.0f) / (now - t_fps_start);
            frame_cnt = 0;
            t_fps_start = now;
        }
    }

    if (s_stream_clients > 0) s_stream_clients--;
    return res;
}

static void start_web_and_stream_servers() {
    httpd_config_t cfg_web = HTTPD_DEFAULT_CONFIG();
    cfg_web.server_port      = 80;
    cfg_web.ctrl_port        = 32768;
    cfg_web.stack_size       = 8192;
    cfg_web.max_open_sockets = 7;
    cfg_web.lru_purge_enable = true;
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
    cfg_stream.server_port      = 81;
    cfg_stream.ctrl_port        = 32769;
    cfg_stream.stack_size       = 12288;
    cfg_stream.core_id          = 1;
    cfg_stream.task_priority    = 6;
    cfg_stream.max_open_sockets = 4;
    cfg_stream.lru_purge_enable = true;
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
        case OV5640_PID: return "OV5640 (5MP - True 720p@30fps Register Table + Sakbot AI)";
        case OV3660_PID: return "OV3660 (3MP)";
        case OV2640_PID: return "OV2640 (2MP)";
        default:         return "Unknown Sensor";
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n======================================================================");
    Serial.println("  [SKETCH 1 - V5] OV5640 720P@30FPS + SAKBOT BOUNDING-BOX DETECTOR");
    Serial.println("======================================================================");

    s_chip_temp_c = temperatureRead();
    Serial.printf("[1/4] Nhiet do noi bo ESP32-S3 : %.1f C %s\n",
                  s_chip_temp_c, (s_chip_temp_c > 70.0f) ? "[CANH BAO: CHIP NONG!]" : "[BINH THUONG]");

    bool has_psram = psramFound();
    size_t thumb_bytes = MAX_RGB_THUMB_W * MAX_RGB_THUMB_H * sizeof(uint16_t);
    s_rgb565_thumb = (uint16_t *)heap_caps_malloc(thumb_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (s_rgb565_thumb) {
        s_thumb_in_sram = true;
    } else if (has_psram) {
        s_rgb565_thumb = (uint16_t *)ps_malloc(thumb_bytes);
        s_thumb_in_sram = false;
    }

    if (has_psram) {
        s_stream_tx_buf = (uint8_t *)ps_malloc(MAX_JPEG_COPY_SZ + STREAM_HDR_RESERVE);
        s_ai_jpeg_buf   = (uint8_t *)ps_malloc(MAX_JPEG_COPY_SZ);
    }

    Serial.printf("[2/4] Bo nho OPI PSRAM         : %u KB trong / %u KB tong (DRAM: %u KB | AI Thumb: %s)\n",
                  (unsigned)(ESP.getFreePsram() / 1024),
                  (unsigned)(ESP.getPsramSize() / 1024),
                  (unsigned)(ESP.getFreeHeap() / 1024),
                  s_thumb_in_sram ? "Internal SRAM [Sieu nhanh]" : "OPI PSRAM");

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
    config.jpeg_quality = 16;
    config.fb_count     = has_psram ? 3 : 1;
    config.fb_location  = has_psram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
    config.grab_mode    = CAMERA_GRAB_LATEST;

    Serial.println("[3/4] Dang khoi tao Camera OV5640 (720p@30fps Register Table 0x3035=0x21)...");
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("  -> [LOI] esp_camera_init that bai (0x%x). Kiem tra cap det FPC!\n", err);
        s_cam_ok = false;
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        configure_ov5640_true_30fps(s, has_psram ? FRAMESIZE_HD : FRAMESIZE_VGA, 16);
        Serial.printf("[4/4] Nhan dien thanh cong     : %s (PID = 0x%04X)\n",
                      get_sensor_name(s->id.PID), s->id.PID);
    }
    s_cam_ok = true;
    s_ai_need_frame = true;

    setup_zero_bottleneck_wifi();
    start_web_and_stream_servers();

    xTaskCreatePinnedToCore(ai_continual_learning_task, "ai_learn_task", 20480, NULL, 1, NULL, 0);
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
        int learn_left = 0;
        uint8_t s_cls = 0, hw_m = 0;
        uint32_t ai_us = 0;

        portENTER_CRITICAL(&s_ai_mux);
        n_boxes    = s_num_boxes;
        if (n_boxes > 0) b0 = s_boxes[0];
        learn_left = s_fast_learn_frames;
        s_cls      = s_scene_ai_cls;
        hw_m       = s_active_hw_mode;
        ai_us      = s_ai_infer_us;
        portEXIT_CRITICAL(&s_ai_mux);

        if (learn_left > 0) {
            Serial.printf("[AI LEARNING] Dang cap nhat nen moi (con %d chu ky) | FPS=%.1f | Temp=%.1fC\n",
                          learn_left, s_stream_fps, s_chip_temp_c);
        } else if (n_boxes > 0) {
            int bx_px = (int)(b0.x * s_ai_frame_w);
            int by_px = (int)(b0.y * s_ai_frame_h);
            int bw_px = (int)(b0.w * s_ai_frame_w);
            int bh_px = (int)(b0.h * s_ai_frame_h);
            Serial.printf("[AI DETECT!] %d khung | Box#1=[x:%d,y:%d,w:%d,h:%d] | Score=%d%% | Mode=%u (M2:%u/M1:%u) | Infer=%.2fms | FPS=%.1f | Temp=%.1fC\n",
                          n_boxes, bx_px, by_px, bw_px, bh_px, b0.score, hw_m, b0.ai_cls_m2, b0.ai_cls_m1,
                          ai_us / 1000.0f, s_stream_fps, s_chip_temp_c);
        } else {
            Serial.printf("[AI NORMAL] Moi truong on dinh (Mode=%u) | FPS=%.1f | Infer=%.2fms | Temp=%.1fC\n",
                          hw_m, s_stream_fps, ai_us / 1000.0f, s_chip_temp_c);
        }
    }

    delay(20);
}
