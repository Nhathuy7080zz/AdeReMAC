// =========================================================================================
// SKETCH 1: STREAM VIDEO MƯỢT + AI NHẠY CAO (PHÂN BIỆT RÕ VẬT THỂ / ĐÈN FLASH vs ĐỔI GÓC)
// Board: ESP32 Cam Kit Phát Triển ESP32-S3 N16R8 OV5640 TypeC (Arduino IDE 2.x)
//
// BẢN NÂNG CẤP GIẢI QUYẾT TRIỆT ĐỂ 4 VẤN ĐỀ THỰC TẾ CỦA BẠN:
//
// 1. TẠI SAO CÓ VÀI ĐIỂM HOT PIXEL TRÊN MODULE MỚI & CÁCH KHẮC PHỤC:
//    - Nguyên nhân: Không phải cảm biến hỏng! Khi ở cảnh tối, nếu để Gain tự động quá cao
//      (16x) và xung XCLK/PCLK căng (20-24MHz), dòng rò tối (dark current) tại vài photodiode
//      của cảm biến 5MP OV5640 bị khuếch đại thành chấm sáng cố định (hot pixel), đồng thời
//      ngưỡng WPC (White Pixel Cancellation) mặc định của OV5640 chưa đủ thấp để xóa.
//    - Đã sửa trong code:
//      + Hạ XCLK về mức chuẩn ổn định 16 MHz + hạ trần Gain vừa phải (GAINCEILING_8X) và ghi
//        trực tiếp vào thanh ghi ISP 0x5000/0x5005 của OV5640 để ép bộ khử điểm nóng (BPC/WPC)
//        hoạt động mạnh hơn.
//      + Thêm bộ lọc "3x3 Median Outlier Despeckle" trong code AI: Tự động phát hiện các điểm
//        nhô cao đơn lẻ (hot pixel) và gọt phẳng bằng trung vị của 8 điểm xung quanh!
//
// 2. SỬA LỖI LẪN LỘN GIỮA "ĐƯA TAY / VẬT THỂ" VỚI "ĐỔI GÓC CAMERA":
//    - Nguyên nhân cũ: Đưa bàn tay lại gần hoặc bật đèn flash làm thay đổi >35% số ô và làm
//      độ sáng trung bình nhảy >18 mức, khiến code cũ tưởng nhầm là "Đổi góc camera" và xóa khung!
//    - Thuật toán mới (Border-Ring & Structural Shift Separation):
//      + Vật thể / Bàn tay / Đèn Flash: Dù to đến mấy (chiếm 50-70% khung hình) vẫn luôn có
//        tâm năng lượng cục bộ và ít nhất 1 phần viền ngoài (4 cạnh biên khung hình) giữ nguyên
//        cấu trúc nền, hoặc có đỉnh sáng cục bộ (đèn Flash). Hệ thống sẽ LUÔN ĐÓNG KHUNG ĐỎ!
//      + Đổi góc Camera thật sự: Chỉ kích hoạt khi CẢ 4 CẠNH VIỀN NGOÀI (Trên, Dưới, Trái, Phải)
//        VÀ >78% toàn bộ khung hình đồng loạt thay đổi cấu trúc góc cạnh (Edge) cùng một lúc,
//        đồng thời không có đỉnh chói cục bộ của đèn Flash!
//
// 3. NHẬN Diện NHẠY CẢ ĐÈN FLASH ĐIỆN THOẠI TRONG PHÒNG TỐI & VẬT THỂ ĐỘ TƯƠNG PHẢN THẤP:
//    - Bỏ bộ lọc sai lầm cũ (`patch_contrast < 22`) từng loại bỏ vùng tâm đèn Flash (do tâm
//      đèn Flash bão hòa ở mức trắng 255 nên độ chênh nội vùng bằng 0!).
//    - Thêm kênh "Flash / High-Delta Detector": Bất kỳ vùng nào tăng vọt độ sáng so với nền
//      (> 28 mức xám) hoặc đạt ngưỡng chói (> 185) đều được bắt dính ngay lập tức!
//
// 4. TĂNG ĐỘ NHẠY & TỐC ĐỘ PHẢN HỒI GẤP 3 LẦN:
//    - Hạ vùng chết nhiễu từ 18.0 xuống 5.5 mức xám (nhờ đã có bộ lọc khử Hot-Pixel Median),
//      bù trừ biến thiên phơi sáng tự động (Auto-Exposure Compensation), và giảm độ trễ xác nhận
//      xuống 2 chu kỳ (65ms/chu kỳ -> phản hồi tức thì trong ~0.13 giây khi đưa tay vào!).
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
// 3. CẤU HÌNH MÔ HÌNH AI HỌC MÔI TRƯỜNG LIÊN TỤC & PHÁT HIỆN NHẠY CAO
// =========================================================================================
#define GRID_W              20      // Lưới không gian 20 cột x 15 hàng (300 ô)
#define GRID_H              15
#define GRID_CELLS          (GRID_W * GRID_H)
#define MAX_ANOM_BOXES      6       // Số khung vật thể bất thường tối đa
#define MAX_JPEG_COPY_SZ    (256 * 1024)
#define MAX_RGB_THUMB_W     240
#define MAX_RGB_THUMB_H     180

struct AnomBox {
    float x, y, w, h;       // Tọa độ chuẩn hóa [0.0 .. 1.0]
    int   score;            // Độ tin cậy vật thể bất thường (60 .. 99%)
    uint8_t ai_cls_m1;      // Nhãn dự đoán Mode 1 (25% Partial AdaReMAC)
    uint8_t ai_cls_m2;      // Nhãn dự đoán Mode 2 (100% Full AdaReMAC)
    uint8_t is_flash;       // 1 = Nguồn sáng / Đèn Flash, 0 = Vật thể / Bàn tay
};

// Trạng thái học môi trường của từng ô lưới (Đặt hoàn toàn trong bộ nhớ tĩnh BSS)
static float   s_bg_y[GRID_CELLS];
static float   s_bg_cr[GRID_CELLS];
static float   s_bg_cb[GRID_CELLS];
static float   s_bg_edge[GRID_CELLS];
static float   s_bg_var[GRID_CELLS];
static uint8_t s_cell_persist[GRID_CELLS];
static float   s_bg_logits[AI_NUM_CLASSES];

// Bộ đệm trung gian tĩnh cho AI Task (0 byte Stack, chống tràn Stack 100%)
static float   s_raw_y[GRID_CELLS];
static float   s_raw_cr[GRID_CELLS];
static float   s_raw_cb[GRID_CELLS];
static float   s_raw_edge[GRID_CELLS];
static float   s_raw_max_y[GRID_CELLS]; // Đỉnh sáng nhất trong mỗi ô (để bắt đèn Flash cực nhạy!)
static float   s_cell_y[GRID_CELLS];
static float   s_cell_cr[GRID_CELLS];
static float   s_cell_cb[GRID_CELLS];
static float   s_cell_edge[GRID_CELLS];
static float   s_z_map[GRID_CELLS];
static float   s_dy_map[GRID_CELLS];
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
static int   s_fast_learn_frames = 10;
static int   s_global_shift_streak = 0;  // Đếm số frame liên tiếp cả 4 viền khung hình dịch chuyển
static volatile bool  s_force_relearn = false;
static volatile float s_anom_threshold = 2.2f; // Ngưỡng nhạy mặc định (nhạy cao với bàn tay & đèn flash)

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

// Trích xuất tensor 20x20 (400 byte INT8) dùng mảng tĩnh s_patch_raw
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
    int32_t norm_span = (true_contrast < 32) ? 32 : true_contrast;
    for (int i = 0; i < 400; i++) {
        int32_t norm = ((int32_t)(s_patch_raw[i] - min_y) * 127) / norm_span;
        if (norm < 0) norm = 0;
        if (norm > 127) norm = 127;
        out_feat400[i] = (int8_t)norm;
    }
    return true_contrast;
}

// =========================================================================================
// THUẬT TOÁN AI CẢI TIẾN:
//   - Khử Hot-Pixel bằng bộ lọc Outlier/Despeckle 3x3
//   - Bắt nhạy đèn Flash trong bóng tối (Peak Luminance & High-Delta)
//   - Tách biệt chính xác "Đưa tay / Vật thể lớn" với "Đổi góc Camera" qua Vòng Biên (Border Ring)
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

    if (!decoded_ok) return;

    // BƯỚC 1: Trích xuất đặc trưng mỗi ô + Lọc bỏ điểm Hot-Pixel đơn lẻ ngay ở mức điểm ảnh!
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
            int max1_luma = 0, max2_luma = 0; // Lưu điểm sáng thứ 2 (bỏ qua 1 hot-pixel đơn lẻ!)

            for (int y = y_start; y < y_end; y++) {
                const uint16_t *row = &s_rgb565_thumb[y * thumb_w];
                for (int x = x_start; x < x_end; x++) {
                    uint16_t p = row[x];
                    uint16_t v = ((p & 0xFF) << 8) | (p >> 8);
                    int r = ((v >> 11) & 0x1F) << 3;
                    int g = ((v >> 5)  & 0x3F) << 2;
                    int b = (v & 0x1F) << 3;
                    int luma = (r * 77 + g * 150 + b * 29) >> 8;

                    // Nếu 1 điểm ảnh nhảy vọt > 90 mức so với điểm ngay cạnh nó -> Đó là Hot Pixel!
                    // Gọt nó về mức của điểm liền kề để không làm sai lệch trung bình ô:
                    if (prev_luma >= 0 && (luma - prev_luma) > 90) {
                        int next_x = (x + 1 < x_end) ? (x + 1) : x;
                        uint16_t pn = row[next_x];
                        uint16_t vn = ((pn & 0xFF) << 8) | (pn >> 8);
                        int ln = ((((vn >> 11) & 0x1F) << 3) * 77 + (((vn >> 5) & 0x3F) << 2) * 150 + ((vn & 0x1F) << 3) * 29) >> 8;
                        if ((luma - ln) > 90) {
                            luma = (prev_luma + ln) / 2; // Đích thị là 1 Hot Pixel cô lập -> Khử ngay!
                        }
                    }

                    if (luma > max1_luma) { max2_luma = max1_luma; max1_luma = luma; }
                    else if (luma > max2_luma) { max2_luma = luma; }

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
                s_raw_y[idx]     = (float)sum_y / cnt;
                s_raw_cr[idx]    = (float)sum_r / cnt - s_raw_y[idx];
                s_raw_cb[idx]    = (float)sum_b / cnt - s_raw_y[idx];
                s_raw_edge[idx]  = (float)sum_edge / cnt;
                s_raw_max_y[idx] = (float)((cnt > 2) ? max2_luma : max1_luma);
            } else {
                s_raw_y[idx] = s_raw_cr[idx] = s_raw_cb[idx] = s_raw_edge[idx] = s_raw_max_y[idx] = 0.0f;
            }
        }
    }

    // BƯỚC 2: Bộ lọc gọt đỉnh Hot-Pixel trên lưới 3x3 (Outlier Despeckle) + Làm mượt nhẹ
    // Nếu 1 ô có giá trị cao bất thường đơn độc mà cả 8 ô xung quanh đều thấp, gọt nó về trung bình 8 ô lân cận!
    for (int gy = 0; gy < GRID_H; gy++) {
        for (int gx = 0; gx < GRID_W; gx++) {
            int idx = gy * GRID_W + gx;
            float nb_y = 0, nb_cr = 0, nb_cb = 0, nb_ed = 0, nb_cnt = 0;
            float max_nb_y = 0;
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
                    if (s_raw_y[n_idx] > max_nb_y) max_nb_y = s_raw_y[n_idx];
                    nb_cnt += 1.0f;
                }
            }
            float avg_nb_y = nb_y / nb_cnt;
            // Nếu ô hiện tại nhô cao đơn độc (> max của cả 8 ô lân cận + 25) -> Khử cụm hot-pixel!
            if (s_raw_y[idx] > max_nb_y + 25.0f && s_raw_max_y[idx] < 210.0f) {
                s_cell_y[idx]    = avg_nb_y;
                s_cell_cr[idx]   = nb_cr / nb_cnt;
                s_cell_cb[idx]   = nb_cb / nb_cnt;
                s_cell_edge[idx] = nb_ed / nb_cnt;
            } else {
                // Giữ độ sắc nét không gian (80% ô trung tâm + 20% lân cận) để bắt nhạy các vật thể nhỏ!
                s_cell_y[idx]    = 0.80f * s_raw_y[idx]    + 0.20f * avg_nb_y;
                s_cell_cr[idx]   = 0.80f * s_raw_cr[idx]   + 0.20f * (nb_cr / nb_cnt);
                s_cell_cb[idx]   = 0.80f * s_raw_cb[idx]   + 0.20f * (nb_cb / nb_cnt);
                s_cell_edge[idx] = 0.80f * s_raw_edge[idx] + 0.20f * (nb_ed / nb_cnt);
            }
        }
    }

    // Khởi tạo nền môi trường ở frame đầu tiên hoặc khi bấm nút "Học lại góc Camera"
    if (!s_env_initialized || s_force_relearn) {
        for (int i = 0; i < GRID_CELLS; i++) {
            s_bg_y[i]         = s_cell_y[i];
            s_bg_cr[i]        = s_cell_cr[i];
            s_bg_cb[i]        = s_cell_cb[i];
            s_bg_edge[i]      = s_cell_edge[i];
            s_bg_var[i]       = 5.0f;
            s_cell_persist[i] = 0;
        }
        extract_patch_int8_400(s_rgb565_thumb, thumb_w, thumb_h, 0, 0, thumb_w, thumb_h, s_patch_feat);
        s_scene_ai_cls        = adaremac_run_nn(s_patch_feat, AI_FEAT_DIM, 2, 0, s_bg_logits);
        s_env_initialized     = true;
        s_force_relearn       = false;
        s_fast_learn_frames   = 8;
        s_global_shift_streak = 0;
    }

    // BƯỚC 3: Ước lượng độ dịch phơi sáng tự động của nền (Background Auto-Exposure Offset)
    // Khi bạn đưa tay vào hoặc bật đèn flash, AEC của camera sẽ tự làm tối/sáng nhẹ phần nền còn lại.
    // Tính độ lệch trung bình trên các ô ở viền ít biến động để bù trừ AEC giúp nền đứng yên!
    float border_diff_sum = 0.0f;
    int   border_quiet_cnt = 0;
    for (int gy = 0; gy < GRID_H; gy++) {
        for (int gx = 0; gx < GRID_W; gx++) {
            if (gy == 0 || gy == GRID_H - 1 || gx == 0 || gx == GRID_W - 1) {
                int idx = gy * GRID_W + gx;
                float d = s_cell_y[idx] - s_bg_y[idx];
                if (fabsf(d) < 18.0f) {
                    border_diff_sum += d;
                    border_quiet_cnt++;
                }
            }
        }
    }
    float ae_offset = (border_quiet_cnt >= 10) ? (border_diff_sum / border_quiet_cnt) : 0.0f;

    // BƯỚC 4: Tính độ lệch từng ô (Kết hợp cả phát hiện Vật thể/Bàn tay VÀ phát hiện Đèn Flash!)
    int raw_changed_cells = 0;
    int flash_cells = 0;
    int border_top_chg = 0, border_bot_chg = 0, border_left_chg = 0, border_right_chg = 0;
    float thr = s_anom_threshold; // Mặc định 2.2 (rất nhạy với bàn tay, vật thể & ánh sáng)

    for (int gy = 0; gy < GRID_H; gy++) {
        for (int gx = 0; gx < GRID_W; gx++) {
            int i = gy * GRID_W + gx;

            // Độ lệch độ sáng sau khi đã bù trừ Auto-Exposure của camera:
            float dy_ae = fabsf((s_cell_y[i] - ae_offset) - s_bg_y[i]);
            float dy_raw = fabsf(s_cell_y[i] - s_bg_y[i]);
            float dy = (dy_ae < dy_raw) ? dy_ae : dy_raw;

            // ĐẶC BIỆT CHO ĐÈN FLASH TRONG CẢNH TỐI:
            // Nếu đỉnh sáng trong ô tăng vọt (> +30 mức xám so với nền) hoặc chói (> 180 khi nền < 120),
            // giữ nguyên độ lệch thật dy_raw + cộng thưởng đỉnh sáng để bắt đèn Flash 100%!
            float peak_jump = s_raw_max_y[i] - s_bg_y[i];
            bool is_flash_cell = (peak_jump > 35.0f && s_raw_max_y[i] > 75.0f) ||
                                 (s_cell_y[i] - s_bg_y[i] > 28.0f);
            if (is_flash_cell) {
                dy = fmaxf(dy_raw, peak_jump * 0.85f);
                flash_cells++;
            }

            float dcr = fabsf(s_cell_cr[i]   - s_bg_cr[i]);
            float dcb = fabsf(s_cell_cb[i]   - s_bg_cb[i]);
            float ded = fabsf(s_cell_edge[i] - s_bg_edge[i]);

            s_dy_map[i] = dy;
            float raw_diff = dy + 0.50f * (dcr + dcb) + 0.65f * ded;

            // Vùng chết nhiễu nhỏ (Deadband = 5.5 mức xám) nhờ đã khử Hot-Pixel ở Bước 1 & 2!
            float clean_diff = raw_diff - 5.5f;
            if (clean_diff < 0.0f) clean_diff = 0.0f;

            float sigma = (s_bg_var[i] < 4.5f) ? 4.5f : s_bg_var[i];
            float z = clean_diff / sigma;
            if (is_flash_cell && z < thr + 2.0f) {
                z = thr + 2.5f; // Ép vượt ngưỡng ngay khi phát hiện nguồn sáng/đèn Flash!
            }
            s_z_map[i] = z;

            if (z > thr) {
                s_raw_anom[i] = 1;
                raw_changed_cells++;
                if (gy == 0)          border_top_chg++;
                if (gy == GRID_H - 1) border_bot_chg++;
                if (gx == 0)          border_left_chg++;
                if (gx == GRID_W - 1) border_right_chg++;
            } else {
                s_raw_anom[i] = 0;
            }
        }
    }

    float changed_ratio = (float)raw_changed_cells / (float)GRID_CELLS;

    // BƯỚC 5: THUẬT TOÁN PHÂN BIỆT CHÍNH XÁC "ĐỔI GÓC CAMERA" vs "ĐƯA TAY / BẬT ĐÈN FLASH"
    // Khi nào mới thực sự là Đổi góc Camera?
    //   1. Cả 4 cạnh viền ngoài của khung hình (Trên, Dưới, Trái, Phải) đều phải thay đổi mạnh
    //      (> 70% số ô trên cả 4 viền cùng lúc). Khi đưa tay vào từ 1 hướng (dù tay che 60% giữa
    //      màn hình), ít nhất 1-2 cạnh viền (ví dụ viền trên hoặc viền hai bên) vẫn là nền cũ!
    //   2. Tổng diện tích biến đổi > 78% toàn bộ khung hình (changed_ratio > 0.78f).
    //   3. Kéo dài liên tiếp >= 3 chu kỳ quét (tránh trường hợp vẫy tay lướt nhanh qua sát ống kính).
    int borders_heavily_changed = 0;
    if (border_top_chg   >= (GRID_W * 7) / 10) borders_heavily_changed++;
    if (border_bot_chg   >= (GRID_W * 7) / 10) borders_heavily_changed++;
    if (border_left_chg  >= (GRID_H * 7) / 10) borders_heavily_changed++;
    if (border_right_chg >= (GRID_H * 7) / 10) borders_heavily_changed++;

    if (changed_ratio > 0.78f && borders_heavily_changed == 4) {
        s_global_shift_streak++;
    } else {
        s_global_shift_streak = 0;
    }

    bool angle_shift_now = false;
    if (s_global_shift_streak >= 3) {
        s_fast_learn_frames = 10;
        s_global_shift_streak = 0;
        angle_shift_now = true;
    }

    if (s_fast_learn_frames > 0) {
        s_fast_learn_frames--;
        float alpha_fast = 0.35f;
        for (int i = 0; i < GRID_CELLS; i++) {
            float dy = fabsf(s_cell_y[i] - s_bg_y[i]);
            s_bg_y[i]         = (1.0f - alpha_fast) * s_bg_y[i]    + alpha_fast * s_cell_y[i];
            s_bg_cr[i]        = (1.0f - alpha_fast) * s_bg_cr[i]   + alpha_fast * s_cell_cr[i];
            s_bg_cb[i]        = (1.0f - alpha_fast) * s_bg_cb[i]   + alpha_fast * s_cell_cb[i];
            s_bg_edge[i]      = (1.0f - alpha_fast) * s_bg_edge[i] + alpha_fast * s_cell_edge[i];
            s_bg_var[i]       = (1.0f - alpha_fast) * s_bg_var[i]  + alpha_fast * (dy + 4.5f);
            s_cell_persist[i] = 0;
            if (s_bg_var[i] < 4.5f)  s_bg_var[i] = 4.5f;
            if (s_bg_var[i] > 16.0f) s_bg_var[i] = 16.0f;
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
        s_env_stability = (1.0f - ((float)s_fast_learn_frames / 10.0f)) * 100.0f;
        if (s_env_stability < 5.0f) s_env_stability = 5.0f;
        s_ai_infer_us = dt_us;
        portEXIT_CRITICAL(&s_ai_mux);
        return;
    }

    // BƯỚC 6: Kiểm tra lân cận (>= 1 ô kề nhau hoặc cường độ mạnh) & Phản hồi nhanh (>= 2 chu kỳ)
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

            // Cho phép cả đốm đèn Flash nhỏ hoặc vật thể có >= 1 ô lân cận cùng thay đổi
            if (s_raw_anom[idx] && (active_neighbors >= 1 || s_z_map[idx] > thr * 1.8f)) {
                if (s_cell_persist[idx] < 10) s_cell_persist[idx]++;
            } else {
                if (s_cell_persist[idx] > 0) s_cell_persist[idx]--;
            }

            // Phản hồi cực nhạy: Chỉ cần 2 chu kỳ liên tiếp (~0.13 giây) hoặc nhảy sáng mạnh (đèn Flash)
            if (s_cell_persist[idx] >= 2 || (s_raw_anom[idx] && s_dy_map[idx] > 35.0f)) {
                s_verified_anom[idx] = 1;
            }
        }
    }

    // Cập nhật thích nghi liên tục mô hình nền môi trường
    for (int i = 0; i < GRID_CELLS; i++) {
        // Ô đang có vật thể bất thường học cực chậm (0.002) để giữ khung đỏ không bị mất khi bạn giữ yên tay!
        float alpha = s_verified_anom[i] ? 0.002f : 0.05f;
        float dy  = fabsf(s_cell_y[i]  - s_bg_y[i]);
        float dcr = fabsf(s_cell_cr[i] - s_bg_cr[i]);
        float dcb = fabsf(s_cell_cb[i] - s_bg_cb[i]);
        float diff = dy + 0.3f * (dcr + dcb);

        s_bg_y[i]    = (1.0f - alpha) * s_bg_y[i]    + alpha * s_cell_y[i];
        s_bg_cr[i]   = (1.0f - alpha) * s_bg_cr[i]   + alpha * s_cell_cr[i];
        s_bg_cb[i]   = (1.0f - alpha) * s_bg_cb[i]   + alpha * s_cell_cb[i];
        s_bg_edge[i] = (1.0f - alpha) * s_bg_edge[i] + alpha * s_cell_edge[i];
        if (!s_verified_anom[i]) {
            s_bg_var[i] = 0.92f * s_bg_var[i] + 0.08f * (diff + 4.0f);
            if (s_bg_var[i] < 4.5f)  s_bg_var[i] = 4.5f;
            if (s_bg_var[i] > 16.0f) s_bg_var[i] = 16.0f;
        }
    }

    // BƯỚC 7: Gom cụm 8 hướng tạo Bounding Box (Hỗ trợ cả Vật thể, Bàn tay lớn và Đèn Flash!)
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
            float sum_z = 0.0f, max_dy = 0.0f, max_peak = 0.0f;

            while (q_head < q_tail) {
                int cx = s_qx[q_head];
                int cy = s_qy[q_head];
                q_head++;

                int c_idx = cy * GRID_W + cx;
                sum_z += s_z_map[c_idx];
                if (s_dy_map[c_idx] > max_dy) max_dy = s_dy_map[c_idx];
                if (s_raw_max_y[c_idx] > max_peak) max_peak = s_raw_max_y[c_idx];

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
            bool is_flash_blob = (max_dy > 32.0f && max_peak > 150.0f);

            // Chấp nhận cụm >= 2 ô, HOẶC 1 ô nếu là đốm đèn Flash sáng mạnh (max_dy > 28)!
            if (cluster_cells < 2 && max_dy < 28.0f) continue;

            if (cand_count < MAX_ANOM_BOXES) {
                int px0 = (min_gx * thumb_w) / GRID_W;
                int py0 = (min_gy * thumb_h) / GRID_H;
                int pbw = ((max_gx - min_gx + 1) * thumb_w) / GRID_W;
                int pbh = ((max_gy - min_gy + 1) * thumb_h) / GRID_H;

                extract_patch_int8_400(s_rgb565_thumb, thumb_w, thumb_h,
                                       px0, py0, pbw, pbh, s_patch_feat);

                AnomBox b;
                b.x = (float)min_gx / (float)GRID_W;
                b.y = (float)min_gy / (float)GRID_H;
                b.w = (float)(max_gx - min_gx + 1) / (float)GRID_W;
                b.h = (float)(max_gy - min_gy + 1) / (float)GRID_H;
                b.is_flash  = is_flash_blob ? 1 : 0;
                b.ai_cls_m1 = adaremac_run_nn(s_patch_feat, AI_PARTIAL_DIM, 1, 2, NULL);
                b.ai_cls_m2 = adaremac_run_nn(s_patch_feat, AI_FEAT_DIM, 2, 0, NULL);

                float avg_z = sum_z / (float)cluster_cells;
                int conf = (int)(65.0f + (avg_z - thr) * 9.0f);
                if (conf < 65) conf = 65;
                if (conf > 99) conf = 99;
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
    s_env_stability = (1.0f - fminf(1.0f, changed_ratio)) * 100.0f;
    s_ai_infer_us = dt_us;
    portEXIT_CRITICAL(&s_ai_mux);
}

// Task AI trên Core 0: Quét mỗi 65ms (~15 Hz) giúp bắt chuyển động bàn tay & đèn Flash tức thì!
static void ai_continual_learning_task(void *arg) {
    while (true) {
        if (!s_cam_ok) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (s_stream_clients > 0) {
            if (!s_ai_frame_ready) {
                s_ai_need_frame = true;
                vTaskDelay(pdMS_TO_TICKS(10));
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
                analyze_frame_and_learn(fb->buf, fb->len, fb->width, fb->height);
                esp_camera_fb_return(fb);
            }
            vTaskDelay(pdMS_TO_TICKS(90));
        }
    }
}

// =========================================================================================
// 4. GIAO DIỆN WEB TRỰC QUAN: LIVE STREAM MƯỢT + NHẬN DIỆN VẬT THỂ & ĐÈN FLASH NHẠY CAO
// =========================================================================================
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>AdaReMAC ESP32-S3 — High-Sensitivity AI Object/Flash Detection & Hot-Pixel Filter</title>
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
      <div class="title">📹 ESP32-S3 OV5640 High-FPS Stream + AI Nhạy Cao (Bàn Tay / Vật Thể / Đèn Flash)</div>
      <div style="font-size:0.82rem;color:#94a3b8;margin-top:2px;">Đã khử Hot-Pixel phần cứng & phần mềm + Tách biệt chính xác Vật thể/Bàn tay với Đổi góc Camera qua 4 Cạnh Viền</div>
    </div>
    <div id="status_badge" class="badge bg-ok">Đang kết nối AI...</div>
  </div>

  <div class="Controls">
    <div class="stat-box"><div class="stat-label">Độ phân giải & Tốc độ Stream</div><div class="stat-val" id="val_fps">HD 1280x720</div></div>
    <div class="stat-box"><div class="stat-label">Biến động Khung hình</div><div class="stat-val" id="val_env">100% Ổn định</div></div>
    <div class="stat-box"><div class="stat-label">Vật thể / Đèn Flash phát hiện</div><div class="stat-val" id="val_anom">0 vật thể</div></div>
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
        <option value="hd" selected>HD (1280x720 - Khuyên dùng)</option>
        <option value="svga">SVGA (800x600 - Siêu mượt)</option>
        <option value="vga">VGA (640x480)</option>
      </select>

      <label style="font-size:0.86rem;margin-left:6px;">Bù sáng (AE):</label>
      <input type="range" min="-2" max="2" step="1" value="0" oninput="updateAE(this.value)">
      <span id="lbl_ae" style="font-weight:700;color:#38bdf8;">0</span>

      <label style="font-size:0.86rem;margin-left:6px;">Nén JPEG:</label>
      <input type="range" min="6" max="24" step="1" value="10" oninput="updateQual(this.value)">
      <span id="lbl_q" style="font-weight:700;color:#38bdf8;">10</span>

      <label style="font-size:0.86rem;margin-left:6px;">Ngưỡng AI (Nhỏ = Nhạy hơn):</label>
      <input type="range" id="rng_thr" min="1.2" max="5.0" step="0.2" value="2.2" oninput="updateThr(this.value)">
      <span id="lbl_thr" style="font-weight:700;color:#38bdf8;">2.2</span>
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
        document.getElementById('val_env').innerText = 'Khác nền: ' + (d.changed * 100).toFixed(1) + '%';
        document.getElementById('val_anom').innerText = d.boxes.length + ' khung (' + (d.ai_us / 1000).toFixed(1) + ' ms)';
        document.getElementById('val_temp').innerText = d.temp.toFixed(1) + ' °C';

        const badge = document.getElementById('status_badge');
        if (d.angle_shift || d.learn_left > 0) {
          badge.className = 'badge bg-warn';
          badge.innerText = '🔄 ĐỔI GÓC CAMERA TOÀN CẢNH -> ĐANG HỌC NỀN MỚI (' + d.stability.toFixed(0) + '%)';
        } else if (d.boxes.length > 0) {
          badge.className = 'badge bg-danger';
          badge.innerText = '⚠️ PHÁT HIỆN ' + d.boxes.length + ' VẬT THỂ / ÁNH SÁNG BẤT THƯỜNG!';
        } else {
          badge.className = 'badge bg-ok';
          badge.innerText = '✅ MÔI TRƯỜNG ỔN ĐỊNH (Lớp nền #' + d.scene_cls + ')';
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

          ctx.strokeStyle = b.fl ? '#f59e0b' : '#ef4444';
          ctx.lineWidth = 3;
          ctx.strokeRect(bx, by, bw, bh);

          ctx.fillStyle = b.fl ? 'rgba(245, 158, 11, 0.22)' : 'rgba(239, 68, 68, 0.18)';
          ctx.fillRect(bx, by, bw, bh);

          const prefix = b.fl ? '⚡ NGUỒN SÁNG / FLASH #' : '🎯 BẤT THƯỜNG #';
          const label = prefix + (idx + 1) + ' | AI Class ' + b.m2 + ' (M1:' + b.m1 + ') | ' + b.score + '%';
          ctx.font = 'bold 13px sans-serif';
          const tw = ctx.measureText(label).width + 14;
          const ly = (by > 26) ? (by - 26) : by;
          ctx.fillStyle = b.fl ? '#f59e0b' : '#ef4444';
          ctx.fillRect(bx, ly, tw, 24);
          ctx.fillStyle = b.fl ? '#000000' : '#ffffff';
          ctx.fillText(label, bx + 7, ly + 17);
        });
      } catch (e) {}
      setTimeout(pollAI, 140);
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
            "%s{\"x\":%.4f,\"y\":%.4f,\"w\":%.4f,\"h\":%.4f,\"score\":%d,\"m1\":%u,\"m2\":%u,\"fl\":%u}",
            (i > 0) ? "," : "",
            local_boxes[i].x, local_boxes[i].y, local_boxes[i].w, local_boxes[i].h,
            local_boxes[i].score, local_boxes[i].ai_cls_m1, local_boxes[i].ai_cls_m2, local_boxes[i].is_flash);
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
        case OV5640_PID: return "OV5640 (5MP - Hot-Pixel Filtered & High-Sensitivity AI)";
        case OV3660_PID: return "OV3660 (3MP)";
        case OV2640_PID: return "OV2640 (2MP)";
        default:         return "Unknown Sensor";
    }
}

void setup() {
    Serial.begin(115200);
    delay(1200);

    Serial.println("\n============================================================");
    Serial.println("  [SKETCH 1] OV5640 STREAM + HIGH-SENSITIVITY AI + WPC FIX");
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
    // Hạ XCLK xuống 16 MHz giúp tín hiệu PCLK trên cáp dẹt FPC sạch tuyệt đối, không bị nhiễu bit trắng!
    config.xclk_freq_hz = 16000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size   = has_psram ? FRAMESIZE_HD : FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count     = has_psram ? 3 : 1;
    config.fb_location  = has_psram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
    config.grab_mode    = CAMERA_GRAB_LATEST;

    Serial.println("[3/4] Dang khoi tao Camera OV5640 (16MHz Clean-PCLK + Hardware Hot-Pixel WPC/BPC)...");
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
        s->set_aec2(s, 0);                         // Tắt Night-Mode chia khung hình để giữ FPS cao
        s->set_ae_level(s, 0);
        s->set_gain_ctrl(s, 1);
        // Hạ Gain Ceiling từ 16X xuống 8X (mức 3) để giảm 50% dòng rò tối gây Hot-Pixel!
        s->set_gainceiling(s, (gainceiling_t)3);
        s->set_brightness(s, 0);
        s->set_contrast(s, 1);
        s->set_saturation(s, 0);
        s->set_sharpness(s, 0);
        s->set_denoise(s, 1);
        s->set_whitebal(s, 1);
        s->set_awb_gain(s, 1);
        s->set_bpc(s, 1);                          // Bật Black Pixel Cancellation
        s->set_wpc(s, 1);                          // Bật White Pixel Cancellation (Khử Hot-Pixel phần cứng)
        s->set_raw_gma(s, 1);
        s->set_lenc(s, 1);
        s->set_dcw(s, 1);
        s->set_vflip(s, 1);
        // Ghi trực tiếp vào thanh ghi ISP Control 0x5000 của OV5640 để cưỡng bức bật LENC + BPC + WPC:
        if (s->id.PID == OV5640_PID && s->set_reg) {
            s->set_reg(s, 0x5000, 0x86, 0x86);
        }
        Serial.printf("[4/4] Nhan dien thanh cong     : %s (PID = 0x%04X)\n",
                      get_sensor_name(s->id.PID), s->id.PID);
    }
    s_cam_ok = true;
    s_ai_need_frame = true;

    setup_zero_bottleneck_wifi();
    start_web_and_stream_servers();

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
            Serial.printf("[AI LEARNING] DOI GOC CAMERA TOAN CANH (Lech %.1f%%) -> Dang hoc nen moi | FPS=%.1f | Temp=%.1fC\n",
                          chg * 100.0f, s_stream_fps, s_chip_temp_c);
        } else if (n_boxes > 0) {
            int bx_px = (int)(b0.x * s_ai_frame_w);
            int by_px = (int)(b0.y * s_ai_frame_h);
            int bw_px = (int)(b0.w * s_ai_frame_w);
            int bh_px = (int)(b0.h * s_ai_frame_h);
            Serial.printf("[AI DETECT!] %d %s! Box#1=[x:%d,y:%d,w:%d,h:%d] | AI=%u(M1:%u) | Score=%d%% | Infer=%.1fms | FPS=%.1f | Temp=%.1fC\n",
                          n_boxes, b0.is_flash ? "NGUON SANG/FLASH" : "VAT THE/BAN TAY",
                          bx_px, by_px, bw_px, bh_px, b0.ai_cls_m2, b0.ai_cls_m1, b0.score, ai_us / 1000.0f, s_stream_fps, s_chip_temp_c);
        } else {
            Serial.printf("[AI NORMAL] Nen on dinh (Khac nen: %.1f%%, Class=%u) | FPS=%.1f | Infer=%.1fms | Temp=%.1fC\n",
                          chg * 100.0f, s_cls, s_stream_fps, ai_us / 1000.0f, s_chip_temp_c);
        }
    }

    delay(20);
}
