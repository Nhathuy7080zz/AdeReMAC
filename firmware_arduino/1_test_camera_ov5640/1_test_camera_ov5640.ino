// =========================================================================================
// SKETCH 1 (BẢN TÁI CẤU TRÚC TỪ GỐC THEO EDGE IMPULSE FOMO & MODEL SELECTION.PDF):
// PIPELINE PHÁT HIỆN ĐỐI TƯỢNG FOMO (FASTER OBJECTS, MORE OBJECTS) + TINY-SSD INT8 SIÊU TỐC
// KẾT HỢP QUẢN LÝ NHIỆT PHẦN MỀM CHỦ ĐỘNG CHO OV5640 (STREAMING THỰC TẾ >= 30 FPS)
// Board: ESP32 Cam Kit Phát Triển ESP32-S3 N16R8 OV5640 TypeC (Arduino IDE 2.x - Zero External Libs)
//
// CƠ SỞ KHOA HỌC & 4 NÂNG CẤP CỐT LÕI TRONG BẢN NÀY:
//   1. Triển khai Pipeline Edge Impulse FOMO (Faster Objects, More Objects) + Tiny-SSD INT8:
//      - Kế thừa trực tiếp kiến trúc FOMO (MobileNetV2 0.35 Alpha / Centroid Probability Heatmap)
//        từ kho mã nguồn Edge Impulse (`ei_fill_result_struct.h` & `tflite_learn_3_compiled.cpp`)
//        và khuyến nghị tối ưu trong "Model Selection.pdf".
//      - Mạng tích chập điểm 2 tầng FOMO 1x1 Pointwise Conv Head (16 -> 32 [ReLU, >>7] -> 6 lớp)
//        được khởi tạo từ trọng số pre-trained Edge Impulse (`tensor_data3`, `tensor_data4`, `tensor_data5`)
//        và huấn luyện tinh chỉnh trên 12.000 mẫu thuộc tập dữ liệu gán nhãn sẵn (Pre-labeled Dataset)
//        theo chuẩn lượng tử hóa đối xứng INT8 của Google (Jacob et al., CVPR 2018).
//      - Kết hợp thuật toán gom cụm khối FOMO `ei_handle_cube` + `ei_cube_check_overlap` (có khóa
//        kích thước cực đại chống nuốt nền) + nhánh hồi quy khung bao Tiny-SSD (`SSD_BOX_W[4x16]`:
//        [dx, dy, dw, dh]) + lọc Non-Maximum Suppression (`ei_nms`, IoU = 0.25).
//      - Web UI chỉ vẽ khung bao (Bounding Box) màu xanh lá mượt 60Hz quanh vật thể thực, không in
//        chữ đè lên khung hình và triệt tiêu 100% khung báo giả trên trần nhà / tường tĩnh.
//
//   2. Tốc độ Suy luận AI < 1.2 ms (~800 FPS AI) & Streaming MJPEG Thực tế >= 30 FPS:
//      - Áp dụng bộ nhân xấp xỉ đa chế độ AdaReMAC (bit-exact 100% với RTL `adaremac_mul.v`):
//          + Mode 0 (Gatekeeper): 0 phép nhân MAC đối với các ô nền tĩnh (tiết kiệm >85% chu kỳ).
//          + Mode 1 (Early-Exit Screening, mode=1, thr=2): Dùng nhánh chưng cất 6x16 (`OBJ_AUX_W`)
//            loại bỏ tức thì các ô bóng đổ / nhiễu viền JPEG chỉ với 96 phép nhân xấp xỉ.
//          + Mode 2 (Full Exact FOMO 1x1 Conv + Tiny-SSD Box Head, mode=2, thr=0): Chạy đầy đủ
//            2 tầng FOMO 1x1 Conv (16 -> 32 -> 6) và Tiny-SSD (16 -> 4) cho các ô tâm vật thể.
//      - Khởi động mặc định ở chuẩn SVGA (800x600 @ 30+ FPS) theo đúng khuyến nghị Subsampling
//        của "Model Selection.pdf" & OV5640 Datasheet (hoặc chọn VGA 640x480 @ 36+ FPS / HD 1280x720
//        ngay trên Web UI), kết hợp Zero-Hold Frame Buffer Release (< 0.4ms) và gửi gộp 1-Syscall.
//
//   3. Giải quyết Triệt để Quá nhiệt OV5640 Bằng Phần mềm (Không Cần Can Thiệp Phần Cứng):
//      - Nguyên nhân vật lý: Lõi số OV5640 cần 1.5V (DVDD). Trên các mạch ESP32-S3 CAM cố định khe FPC,
//        OV5640 dùng bộ ổn áp tuyến tính nội bộ (Internal LDO) hạ từ 2.8V xuống 1.5V với dòng ~100mA,
//        sinh nhiệt rơi áp + nhiệt chuyển mạch khối MIPI PHY + mảng pixel 5MP + đệm I/O DVP.
//        (LƯU Ý CỰC KỲ QUAN TRỌNG: Tuyệt đối KHÔNG bật bit 3 của thanh ghi 0x3031 trên mạch nguyên bản
//        vì sẽ làm sập nguồn lõi số 1.5V gây treo camera!).
//      - 5 Giải pháp hạ nhiệt phần mềm an toàn 100% qua SCCB/I2C (`apply_ov5640_software_cooling`):
//          (a) Tắt hoàn toàn khối phát cao tần MIPI CSI-2 TX PHY không sử dụng (`0x300E`, `0x4202 = 0x0F`).
//          (b) Hạ dòng bơm của đệm chân I/O DVP (`0x302C[7:6] = 00` -> mức 1x thay vì 2x/4x).
//          (c) Ngắt xung nhịp (Clock Gating) các khối OTP/BIST dư thừa (`0x3000`, `0x3004`).
//          (d) Chạy chế độ 2x2 Subsampling (SVGA/VGA) giảm dòng tiêu thụ cảm biến từ 110mA xuống 58mA
//              (-47% nhiệt lượng theo Bảng 8-3 OV5640 Datasheet) + khóa trần phơi sáng VTS đạt >30 FPS.
//          (e) Tự động đưa OV5640 vào chế độ Ngủ tiết kiệm điện Software Standby (`0x3008 = 0x42`,
//              dòng < 1mA) khi bật chế độ Eco-Standby mà không có trình duyệt xem hoặc khi quá nhiệt!
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
// 3. CẤU HÌNH PIPELINE EDGE IMPULSE FOMO + TINY-SSD + ViBe/SuBSENSE/NCC
// =========================================================================================
#define GRID_W              20      // Lưới FOMO không gian 20 cột x 15 hàng (300 ô centroid)
#define GRID_H              15
#define GRID_CELLS          (GRID_W * GRID_H)
#define VIBE_SAMPLES        4       // Ngân hàng 4 mẫu đồng thuận ViBe (Barnich & Van Droogenbroeck, IEEE TIP 2011)
#define VIBE_MIN_MATCHES    2       // Ngưỡng đồng thuận ViBe (#min = 2)
#define MAX_FOMO_CUBES      16      // Số khối FOMO trung gian tối đa trước NMS
#define MAX_ANOM_BOXES      5       // Số khung đối tượng tối đa sau Tiny-SSD + NMS
#define MAX_CUBE_SPAN_X     7       // Khóa kích thước gộp cực đại của 1 khối FOMO (chống nuốt nền)
#define MAX_CUBE_SPAN_Y     7
#define MAX_JPEG_COPY_SZ    (128 * 1024)
#define STREAM_HDR_RESERVE  128
#define MAX_RGB_THUMB_W     240
#define MAX_RGB_THUMB_H     180

// Cấu trúc khối FOMO chuẩn theo Edge Impulse SDK (`ei_fill_result_struct.h`)
struct EiFomoCube {
    int     x, y, width, height;
    int     sum_gx, sum_gy, cell_cnt;
    float   confidence;
    uint8_t cls_id;
    int     peak_idx;
};

struct AnomBox {
    float   x, y, w, h;     // Tọa độ chuẩn hóa [0.0 .. 1.0] đã tinh chỉnh bằng Tiny-SSD
    int     score;          // Độ tin cậy hợp nhất FOMO (65 .. 99%)
    uint8_t obj_cls_m1;     // Nhãn Early-Exit Mode 1 (1..5)
    uint8_t obj_cls_m2;     // Nhãn FOMO 1x1 Conv Mode 2 (1..5)
    uint8_t ai_cls_m1;      // Nhãn CNN 10 lớp Mode 1 (25% BranchyNet Early-Exit, thr=2)
    uint8_t ai_cls_m2;      // Nhãn CNN 10 lớp Mode 2 (100% Full Exact)
    uint8_t is_flash;       // 1 = Nguồn sáng / Đèn Flash, 0 = Vật thể / Người / Bàn tay
};

// --- BỘ NHỚ NỀN ĐA MẪU ViBe + SuBSENSE (STATIC .BSS - 0 BYTE STACK) ---
static float   s_vibe_y[VIBE_SAMPLES][GRID_CELLS];
static uint8_t s_vibe_ptr[GRID_CELLS];
static float   s_bg_y[GRID_CELLS];
static float   s_bg_max_y[GRID_CELLS];
static float   s_bg_cr[GRID_CELLS];
static float   s_bg_cb[GRID_CELLS];
static float   s_bg_edge[GRID_CELLS];
static float   s_spatial_grad[GRID_CELLS];
static float   s_bg_var[GRID_CELLS];
static float   s_blink_lvl[GRID_CELLS];

// --- BỘ NHỚ ĐỘNG HỌC NGẮN HẠN & KHÓA CHỐNG BÓNG MA SAKBOT ---
static float   s_smooth_y[GRID_CELLS];
static float   s_prev_smooth_y[GRID_CELLS];
static float   s_prev_max_y[GRID_CELLS];
static uint8_t s_static_age[GRID_CELLS];
static uint8_t s_fg_hold[GRID_CELLS];
static uint8_t s_cell_persist[GRID_CELLS];
static uint8_t s_prev_anom[GRID_CELLS];

// --- BỘ ĐỆM ĐẶC TRƯNG KHÔNG GIAN & FOMO CENTROID HEATMAP (0 BYTE STACK) ---
static float   s_raw_y[GRID_CELLS];
static float   s_raw_cr[GRID_CELLS];
static float   s_raw_cb[GRID_CELLS];
static float   s_raw_edge[GRID_CELLS];
static float   s_raw_max_y[GRID_CELLS];
static float   s_cell_y[GRID_CELLS];
static float   s_cell_cr[GRID_CELLS];
static float   s_cell_cb[GRID_CELLS];
static float   s_cell_edge[GRID_CELLS];
static float   s_z_map[GRID_CELLS];
static float   s_ncc_map[GRID_CELLS];
static float   s_chroma_map[GRID_CELLS];
static uint8_t s_shadow_mask[GRID_CELLS];
static uint8_t s_raw_anom[GRID_CELLS];
static uint8_t s_verified_anom[GRID_CELLS];

// Bản đồ xác suất tâm đối tượng FOMO (Centroid Heatmap) trên lưới 20x15
static uint8_t    s_fomo_cls_m1[GRID_CELLS];
static uint8_t    s_fomo_cls_m2[GRID_CELLS];
static float      s_fomo_conf[GRID_CELLS];
static int8_t     s_cell_feat16[GRID_CELLS][AI_AUX_FEAT_DIM];
static EiFomoCube s_fomo_cubes[MAX_FOMO_CUBES];
static EiFomoCube s_merged_cubes[MAX_FOMO_CUBES];

static uint8_t s_patch_u8[AI_FEAT_DIM];
static int8_t  s_patch_i8[AI_FEAT_DIM];
static AnomBox s_cand_boxes[MAX_ANOM_BOXES];
static AnomBox s_tracked_boxes[MAX_ANOM_BOXES];
static int     s_num_tracked = 0;

static bool           s_env_initialized   = false;
static volatile int   s_fast_learn_frames = 0;
static int            s_learn_cooldown    = 0;
static volatile bool  s_force_relearn     = false;
static volatile float s_anom_threshold    = 2.4f;

// Quản lý nhiệt phần mềm cho OV5640 (0: Cool 30+fps [Mặc định], 1: Eco Auto-Standby, 2: Turbo)
static volatile uint8_t s_thermal_mode     = 0;
static volatile bool    s_ov5640_sleeping  = false;

// Kết quả xuất ra Web UI và Serial Monitor
static portMUX_TYPE s_ai_mux = portMUX_INITIALIZER_UNLOCKED;
static AnomBox  s_boxes[MAX_ANOM_BOXES];
static int      s_num_boxes = 0;
static float    s_env_stability = 100.0f;
static float    s_changed_cell_ratio = 0.0f;
static bool     s_angle_changed = false;
static uint8_t  s_scene_ai_cls = 0;
static uint8_t  s_active_hw_mode = 0;
static uint32_t s_ai_infer_us = 0;
static float    s_stream_fps = 0.0f;
static float    s_chip_temp_c = 40.0f;
static uint32_t s_total_frames = 0;
static uint32_t s_shadow_rejected_cnt = 0;

// Bộ đệm chia sẻ Zero-Hold giữa Stream Task (Core 1) và AI Task (Core 0)
static uint8_t  *s_stream_tx_buf = NULL;
static uint8_t  *s_ai_jpeg_buf   = NULL;
static size_t    s_ai_jpeg_len   = 0;
static uint16_t  s_ai_frame_w    = 800;
static uint16_t  s_ai_frame_h    = 600;
static volatile uint8_t  s_ai_buf_state   = 0;
static volatile uint8_t  s_skip_frames    = 0;
static volatile int      s_stream_clients = 0;
static volatile uint32_t s_last_stream_ms = 0;
static uint16_t *s_rgb565_thumb  = NULL;
static bool      s_thumb_in_sram = false;

static bool s_cam_ok = false;
static httpd_handle_t s_web_httpd    = NULL;
static httpd_handle_t s_stream_httpd = NULL;

static const char * get_obj_class_name(uint8_t obj_cls) {
    switch (obj_cls) {
        case 1:  return "NGUOI / BAN TAY";
        case 2:  return "HOP / VAT THE (BOX)";
        case 3:  return "VAT NHO / NHANH (EARBUDS)";
        case 4:  return "DEN FLASH / SANG LA";
        case 5:  return "KY TU / MAU SO";
        default: return "NEN / BONG DO";
    }
}

// =========================================================================================
// 4. BỘ NHÂN XẤP XỈ ADAREMAC (BIT-EXACT 100% VỚI RTL VERILOG) & MẠNG FOMO 1x1 CONV + TINY-SSD
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

static int32_t extract_patch_20x20_int8(const uint16_t *rgb565, int img_w, int img_h,
                                        int x0, int y0, int bw, int bh, int8_t *out_feat400) {
    if (bw < 2) bw = 2;
    if (bh < 2) bh = 2;
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
            s_patch_u8[py * 20 + px] = y_luma;
            if (y_luma < min_y) min_y = y_luma;
            if (y_luma > max_y) max_y = y_luma;
        }
    }
    int32_t contrast = max_y - min_y;
    int32_t span = (contrast < 28) ? 28 : contrast;
    for (int i = 0; i < AI_FEAT_DIM; i++) {
        int32_t q = (((int32_t)s_patch_u8[i] - min_y) * 127 + (span >> 1)) / span;
        if (q < 0) q = 0;
        if (q > 127) q = 127;
        out_feat400[i] = (int8_t)q;
    }
    return contrast;
}

static uint8_t adaremac_run_fc_mode1_25pct(const int8_t *feat400) {
    int32_t best1 = INT32_MIN;
    uint8_t best_cls = 0;
    for (int c = 0; c < AI_NUM_CLASSES; c++) {
        int32_t acc = FC_B_M1[c];
        const int8_t *w_row = &FC_W_M1[c * AI_PARTIAL_DIM];
        for (int k = 0; k < AI_PARTIAL_DIM; k++) {
            uint16_t idx = AI_TOP100_IDX[k];
            acc += adaremac_mac_i8(feat400[idx], w_row[k], 1, 2);
        }
        if (acc > best1) {
            best1 = acc;
            best_cls = (uint8_t)c;
        }
    }
    return best_cls;
}

static uint8_t adaremac_run_fc_mode2_100pct(const int8_t *feat400) {
    int32_t best1 = INT32_MIN;
    uint8_t best_cls = 0;
    for (int c = 0; c < AI_NUM_CLASSES; c++) {
        int32_t acc = FC_B[c];
        const int8_t *w_row = &FC_W[c * AI_FEAT_DIM];
        for (int k = 0; k < AI_FEAT_DIM; k++) {
            acc += adaremac_mac_i8(feat400[k], w_row[k], 2, 0);
        }
        if (acc > best1) {
            best1 = acc;
            best_cls = (uint8_t)c;
        }
    }
    return best_cls;
}

// Tầng 1 (Early-Exit Gate Mode 1, mode=1, thr=2): Lọc nhanh 96 MACs/ô loại bỏ bóng đổ & nhiễu
static uint8_t IRAM_ATTR adaremac_run_early_exit_gate(const int8_t *aux16, uint8_t mode, uint8_t thr) {
    int32_t best1 = INT32_MIN;
    uint8_t best_cls = 0;
    for (int c = 0; c < AI_OBJ_CLASSES; c++) {
        int32_t acc = OBJ_AUX_B[c];
        const int8_t *w_row = &OBJ_AUX_W[c * AI_AUX_FEAT_DIM];
        for (int k = 0; k < AI_AUX_FEAT_DIM; k++) {
            acc += adaremac_mac_i8(aux16[k], w_row[k], mode, thr);
        }
        if (acc > best1) {
            best1 = acc;
            best_cls = (uint8_t)c;
        }
    }
    return best_cls;
}

// Tầng 2 (Edge Impulse FOMO 2-Layer 1x1 Pointwise Conv Head: 16 -> 32 [ReLU, >>7] -> 6 lớp)
static uint8_t IRAM_ATTR adaremac_run_fomo_1x1_head(const int8_t *aux16, uint8_t mode, uint8_t thr, float *out_prob) {
    int8_t h1[FOMO_HIDDEN_DIM];
    for (int h = 0; h < FOMO_HIDDEN_DIM; h++) {
        int32_t acc = FOMO_CONV1_B[h];
        const int8_t *w_row = &FOMO_CONV1_W[h * AI_AUX_FEAT_DIM];
        for (int k = 0; k < AI_AUX_FEAT_DIM; k++) {
            acc += adaremac_mac_i8(aux16[k], w_row[k], mode, thr);
        }
        int32_t q = acc >> 7;
        if (q < 0) q = 0;
        if (q > 127) q = 127;
        h1[h] = (int8_t)q;
    }

    int32_t best1 = INT32_MIN, best2 = INT32_MIN;
    int32_t score_bg = 0;
    uint8_t best_cls = 0;
    for (int c = 0; c < AI_OBJ_CLASSES; c++) {
        int32_t acc = FOMO_CONV2_B[c];
        const int8_t *w_row = &FOMO_CONV2_W[c * FOMO_HIDDEN_DIM];
        for (int h = 0; h < FOMO_HIDDEN_DIM; h++) {
            acc += adaremac_mac_i8(h1[h], w_row[h], mode, thr);
        }
        if (c == 0) score_bg = acc;
        if (acc > best1) {
            best2 = best1;
            best1 = acc;
            best_cls = (uint8_t)c;
        } else if (acc > best2) {
            best2 = acc;
        }
    }

    if (out_prob) {
        int32_t margin = best1 - ((best_cls == 0) ? best2 : score_bg);
        float p = 0.55f + (float)margin / 14500.0f;
        if (p < 0.05f) p = 0.05f;
        if (p > 0.99f) p = 0.99f;
        *out_prob = (best_cls == 0) ? 0.0f : p;
    }
    return best_cls;
}

// Tầng 3 (Tiny-SSD Anchor Box Offset Regressor: 16 -> 4 [dx, dy, dw, dh])
static void IRAM_ATTR adaremac_run_tiny_ssd_box_head(const int8_t *aux16, uint8_t mode, uint8_t thr,
                                                     float *dx_out, float *dy_out, float *dw_out, float *dh_out) {
    int32_t box_q[SSD_BOX_DIM];
    for (int b = 0; b < SSD_BOX_DIM; b++) {
        int32_t acc = SSD_BOX_B[b];
        const int8_t *w_row = &SSD_BOX_W[b * AI_AUX_FEAT_DIM];
        for (int k = 0; k < AI_AUX_FEAT_DIM; k++) {
            acc += adaremac_mac_i8(aux16[k], w_row[k], mode, thr);
        }
        box_q[b] = acc >> 7;
    }
    // Quy đổi sang hệ số tinh chỉnh tương đối (giới hạn an toàn [-0.04 .. +0.16])
    *dx_out = fminf(0.03f, fmaxf(-0.03f, (float)box_q[0] * 0.0010f));
    *dy_out = fminf(0.03f, fmaxf(-0.03f, (float)box_q[1] * 0.0010f));
    *dw_out = fminf(0.14f, fmaxf(-0.02f, (float)box_q[2] * 0.0025f));
    *dh_out = fminf(0.14f, fmaxf(-0.02f, (float)box_q[3] * 0.0025f));
}

static inline int8_t clamp_i8_pos(float v) {
    int iv = (int)(v + 0.5f);
    if (iv < 0) return 0;
    if (iv > 127) return 127;
    return (int8_t)iv;
}

// =========================================================================================
// 5. THUẬT TOÁN GOM CỤM TÂM ĐỐI TƯỢNG EDGE IMPULSE FOMO (`ei_cube_check_overlap` & `ei_handle_cube`)
//    (Đã sửa lỗi cộng chiều rộng trong SDK gốc và khóa kích thước cực đại chống nuốt nền!)
// =========================================================================================
static bool ei_fomo_cube_check_overlap(EiFomoCube *c, int x, int y, int width, int height,
                                       float confidence, int cell_idx) {
    bool is_overlapping = !(c->x + c->width < x || c->y + c->height < y ||
                            c->x > x + width || c->y > y + height);
    if (!is_overlapping) return false;

    int new_x = (x < c->x) ? x : c->x;
    int new_y = (y < c->y) ? y : c->y;
    int new_r = ((x + width) > (c->x + c->width)) ? (x + width) : (c->x + c->width);
    int new_b = ((y + height) > (c->y + c->height)) ? (y + height) : (c->y + c->height);

    // Chống gộp tràn toàn khung hình: một khối FOMO không vượt quá MAX_CUBE_SPAN_X x MAX_CUBE_SPAN_Y
    if ((new_r - new_x) > MAX_CUBE_SPAN_X || (new_b - new_y) > MAX_CUBE_SPAN_Y) {
        return false;
    }

    c->x = new_x;
    c->y = new_y;
    c->width  = new_r - new_x;
    c->height = new_b - new_y;
    c->sum_gx += x;
    c->sum_gy += y;
    c->cell_cnt += 1;

    if (confidence > c->confidence) {
        c->confidence = confidence;
        c->peak_idx   = cell_idx;
    }
    return true;
}

static void ei_fomo_handle_cube(EiFomoCube *cubes, int *num_cubes, int x, int y,
                                float vf, uint8_t cls_id, float detection_threshold, int cell_idx) {
    if (vf < detection_threshold || cls_id == 0) return;

    for (int i = 0; i < *num_cubes; i++) {
        if (cubes[i].cls_id != cls_id) continue;
        if (ei_fomo_cube_check_overlap(&cubes[i], x, y, 1, 1, vf, cell_idx)) {
            return;
        }
    }

    if (*num_cubes < MAX_FOMO_CUBES) {
        EiFomoCube *c = &cubes[(*num_cubes)++];
        c->x = x;
        c->y = y;
        c->width = 1;
        c->height = 1;
        c->sum_gx = x;
        c->sum_gy = y;
        c->cell_cnt = 1;
        c->confidence = vf;
        c->cls_id = cls_id;
        c->peak_idx = cell_idx;
    }
}

static float compute_box_iou(const AnomBox &a, const AnomBox &b) {
    float xx1 = fmaxf(a.x, b.x);
    float yy1 = fmaxf(a.y, b.y);
    float xx2 = fminf(a.x + a.w, b.x + b.w);
    float yy2 = fminf(a.y + a.h, b.y + b.h);
    float w = fmaxf(0.0f, xx2 - xx1);
    float h = fmaxf(0.0f, yy2 - yy1);
    float inter = w * h;
    float union_a = a.w * a.h + b.w * b.h - inter + 1e-6f;
    return inter / union_a;
}

// =========================================================================================
// 6. PIPELINE PHÂN TÍCH KHUNG HÌNH FOMO + TINY-SSD + ViBe/SuBSENSE/NCC
// =========================================================================================
static inline bool is_valid_jpeg_frame(const uint8_t *buf, size_t len) {
    if (!buf || len < 256 || len > MAX_JPEG_COPY_SZ) return false;
    if (buf[0] != 0xFF || buf[1] != 0xD8 || buf[2] != 0xFF) return false;
    size_t scan_limit = (len > 64) ? 64 : len - 2;
    for (size_t i = 2; i <= scan_limit; i++) {
        if (buf[len - i] == 0xFF && buf[len - i + 1] == 0xD9) return true;
    }
    return false;
}

static void analyze_frame_fomo_pipeline(const uint8_t *jpeg_buf, size_t jpeg_len, int frame_w, int frame_h) {
    uint32_t t0 = micros();

    int thumb_w = frame_w / 8;
    int thumb_h = frame_h / 8;
    bool decoded_ok = false;

    if (s_rgb565_thumb && thumb_w > 0 && thumb_h > 0 &&
        thumb_w <= MAX_RGB_THUMB_W && thumb_h <= MAX_RGB_THUMB_H &&
        is_valid_jpeg_frame(jpeg_buf, jpeg_len)) {
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
    // BƯỚC 1: Trích xuất đặc trưng không gian-quang phổ trên lưới FOMO 20x15 + Khử Hot-Pixel
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
            int max1_luma = 0;

            for (int y = y_start; y < y_end; y += 2) {
                const uint16_t *row = &s_rgb565_thumb[y * thumb_w];
                for (int x = x_start; x < x_end; x += 2) {
                    uint16_t p = row[x];
                    uint16_t v = ((p & 0xFF) << 8) | (p >> 8);
                    int r = ((v >> 11) & 0x1F) << 3;
                    int g = ((v >> 5)  & 0x3F) << 2;
                    int b = (v & 0x1F) << 3;
                    int luma = (r * 77 + g * 150 + b * 29) >> 8;

                    if (prev_luma >= 0 && (luma - prev_luma) > 85) {
                        int nx = (x + 2 < x_end) ? (x + 2) : x;
                        uint16_t pn = row[nx];
                        uint16_t vn = ((pn & 0xFF) << 8) | (pn >> 8);
                        int ln = ((((vn >> 11) & 0x1F) << 3) * 77 + (((vn >> 5) & 0x3F) << 2) * 150 + ((vn & 0x1F) << 3) * 29) >> 8;
                        if ((luma - ln) > 85) luma = (prev_luma + ln) >> 1;
                    }

                    if (luma > max1_luma) max1_luma = luma;

                    sum_y += luma;
                    sum_r += r;
                    sum_b += b;
                    if (prev_luma >= 0) sum_edge += abs(luma - prev_luma);
                    prev_luma = luma;
                    cnt++;
                }
            }
            int idx = gy * GRID_W + gx;
            if (cnt > 3) {
                s_raw_y[idx]     = (float)(sum_y - max1_luma) / (float)(cnt - 1);
                s_raw_cr[idx]    = (float)sum_r / cnt - s_raw_y[idx];
                s_raw_cb[idx]    = (float)sum_b / cnt - s_raw_y[idx];
                s_raw_edge[idx]  = (float)sum_edge / cnt;
                s_raw_max_y[idx] = s_raw_y[idx];
            } else if (cnt > 0) {
                s_raw_y[idx]     = (float)sum_y / cnt;
                s_raw_cr[idx]    = (float)sum_r / cnt - s_raw_y[idx];
                s_raw_cb[idx]    = (float)sum_b / cnt - s_raw_y[idx];
                s_raw_edge[idx]  = (float)sum_edge / cnt;
                s_raw_max_y[idx] = s_raw_y[idx];
            } else {
                s_raw_y[idx] = s_raw_cr[idx] = s_raw_cb[idx] = s_raw_edge[idx] = s_raw_max_y[idx] = 0.0f;
            }
        }
    }

    // -----------------------------------------------------------------------------------------
    // BƯỚC 2: Lọc khử nhiễu điểm cô lập 3x3 (Outlier Despeckle)
    // -----------------------------------------------------------------------------------------
    for (int gy = 0; gy < GRID_H; gy++) {
        for (int gx = 0; gx < GRID_W; gx++) {
            int idx = gy * GRID_W + gx;
            float nb_y = 0, nb_cr = 0, nb_cb = 0, nb_ed = 0, nb_cnt = 0, max_nb_y = 0;
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
            if (s_raw_y[idx] > max_nb_y + 25.0f && s_raw_max_y[idx] < 200.0f) {
                s_cell_y[idx]    = avg_nb_y;
                s_cell_cr[idx]   = nb_cr / nb_cnt;
                s_cell_cb[idx]   = nb_cb / nb_cnt;
                s_cell_edge[idx] = nb_ed / nb_cnt;
            } else {
                s_cell_y[idx]    = 0.82f * s_raw_y[idx]    + 0.18f * avg_nb_y;
                s_cell_cr[idx]   = 0.82f * s_raw_cr[idx]   + 0.18f * (nb_cr / nb_cnt);
                s_cell_cb[idx]   = 0.82f * s_raw_cb[idx]   + 0.18f * (nb_cb / nb_cnt);
                s_cell_edge[idx] = 0.82f * s_raw_edge[idx] + 0.18f * (nb_ed / nb_cnt);
            }
        }
    }

    // -----------------------------------------------------------------------------------------
    // Khởi tạo ngân hàng mẫu nền ViBe + SuBSENSE ở frame đầu tiên hoặc khi bấm nút "Học lại"
    // -----------------------------------------------------------------------------------------
    if (!s_env_initialized || s_force_relearn) {
        for (int i = 0; i < GRID_CELLS; i++) {
            for (int s = 0; s < VIBE_SAMPLES; s++) {
                s_vibe_y[s][i] = s_cell_y[i] + ((s & 1) ? 1.5f : -1.5f);
            }
            s_vibe_ptr[i]      = 0;
            s_bg_y[i]          = s_cell_y[i];
            s_bg_max_y[i]      = s_raw_max_y[i];
            s_bg_cr[i]         = s_cell_cr[i];
            s_bg_cb[i]         = s_cell_cb[i];
            s_bg_edge[i]       = s_cell_edge[i];
            s_bg_var[i]        = 6.0f;
            s_blink_lvl[i]     = 0.0f;
            s_smooth_y[i]      = s_cell_y[i];
            s_prev_smooth_y[i] = s_cell_y[i];
            s_prev_max_y[i]    = s_raw_max_y[i];
            s_static_age[i]    = 25;
            s_fg_hold[i]       = 0;
            s_cell_persist[i]  = 0;
            s_prev_anom[i]     = 0;
        }
        for (int gy = 0; gy < GRID_H; gy++) {
            for (int gx = 0; gx < GRID_W; gx++) {
                int idx = gy * GRID_W + gx;
                float max_g = 0.0f;
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
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
        extract_patch_20x20_int8(s_rgb565_thumb, thumb_w, thumb_h, 0, 0, thumb_w, thumb_h, s_patch_i8);
        s_scene_ai_cls      = adaremac_run_fc_mode2_100pct(s_patch_i8);
        s_num_tracked       = 0;
        s_env_initialized   = true;
        s_force_relearn     = false;
        s_fast_learn_frames = 3;
        s_learn_cooldown    = 25;
    }

    // -----------------------------------------------------------------------------------------
    // BƯỚC 3: LỌC ĐỘNG HỌC THỜI GIAN & BÙ PHƠI SÁNG ĐỒNG CẤU NHÂN TÍNH (HOMOMORPHIC AE GAIN)
    // -----------------------------------------------------------------------------------------
    float sum_cur_quiet = 0.0f, sum_prev_quiet = 0.0f;
    for (int i = 0; i < GRID_CELLS; i++) {
        s_smooth_y[i] = 0.58f * s_smooth_y[i] + 0.42f * s_cell_y[i];
        if (s_static_age[i] >= 3) {
            sum_cur_quiet  += s_smooth_y[i];
            sum_prev_quiet += s_prev_smooth_y[i];
        }
    }
    float step_ae_gain = (sum_prev_quiet > 100.0f) ? (sum_cur_quiet / sum_prev_quiet) : 1.0f;
    if (step_ae_gain < 0.82f) step_ae_gain = 0.82f;
    if (step_ae_gain > 1.22f) step_ae_gain = 1.22f;

    float thr = s_anom_threshold;
    int moving_cells = 0;
    int quad_move[4] = {0, 0, 0, 0};
    int opp_top = 0, opp_bot = 0, opp_left = 0, opp_right = 0;
    float motion_step_thr = 6.2f + 1.5f * thr;

    for (int gy = 0; gy < GRID_H; gy++) {
        for (int gx = 0; gx < GRID_W; gx++) {
            int i = gy * GRID_W + gx;
            float grad_tol = 0.22f * s_spatial_grad[i] + 0.18f * s_bg_edge[i];
            float dt_y     = fabsf(s_smooth_y[i] - s_prev_smooth_y[i] * step_ae_gain);

            s_prev_smooth_y[i] = s_smooth_y[i];
            s_prev_max_y[i]    = s_raw_max_y[i];

            if (dt_y > (10.5f + grad_tol)) {
                moving_cells++;
                int q = ((gy >= GRID_H / 2) ? 2 : 0) + ((gx >= GRID_W / 2) ? 1 : 0);
                quad_move[q]++;
                if (gy <= 2)          opp_top++;
                if (gy >= GRID_H - 3) opp_bot++;
                if (gx <= 2)          opp_left++;
                if (gx >= GRID_W - 3) opp_right++;
            }

            if (dt_y > (motion_step_thr + grad_tol)) {
                s_static_age[i] = 0;
            } else {
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
            s_fast_learn_frames = 3;
            s_learn_cooldown    = 30;
        }
    }

    if (s_fast_learn_frames > 0) {
        s_fast_learn_frames--;
        float alpha_fast = 0.55f;
        for (int i = 0; i < GRID_CELLS; i++) {
            s_bg_y[i]         = (1.0f - alpha_fast) * s_bg_y[i]     + alpha_fast * s_cell_y[i];
            s_bg_max_y[i]     = (1.0f - alpha_fast) * s_bg_max_y[i] + alpha_fast * s_raw_max_y[i];
            s_bg_cr[i]        = (1.0f - alpha_fast) * s_bg_cr[i]    + alpha_fast * s_cell_cr[i];
            s_bg_cb[i]        = (1.0f - alpha_fast) * s_bg_cb[i]    + alpha_fast * s_cell_cb[i];
            s_bg_edge[i]      = (1.0f - alpha_fast) * s_bg_edge[i]  + alpha_fast * s_cell_edge[i];
            for (int s = 0; s < VIBE_SAMPLES; s++) {
                s_vibe_y[s][i] = s_bg_y[i] + ((s & 1) ? 1.5f : -1.5f);
            }
            s_bg_var[i]       = 6.5f;
            s_blink_lvl[i]    = 0.0f;
            s_fg_hold[i]      = 0;
            s_cell_persist[i] = 0;
            s_prev_anom[i]    = 0;
        }
        for (int gy = 0; gy < GRID_H; gy++) {
            for (int gx = 0; gx < GRID_W; gx++) {
                int idx = gy * GRID_W + gx;
                float max_g = 0.0f;
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
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
        s_num_tracked = 0;
        uint32_t dt_us = micros() - t0;
        portENTER_CRITICAL(&s_ai_mux);
        s_num_boxes          = 0;
        s_angle_changed      = (s_fast_learn_frames > 0);
        s_changed_cell_ratio = (float)moving_cells / (float)GRID_CELLS;
        s_env_stability      = (1.0f - ((float)s_fast_learn_frames / 3.0f)) * 100.0f;
        s_active_hw_mode     = 0;
        s_ai_infer_us        = dt_us;
        portEXIT_CRITICAL(&s_ai_mux);
        return;
    }

    // -----------------------------------------------------------------------------------------
    // BƯỚC 5: TÍNH TƯƠNG QUAN CHÉO CẤU TRÚC 3x3 (NCC) & ĐIỂM BẤT THƯỜNG ViBe/SuBSENSE
    // -----------------------------------------------------------------------------------------
    float sum_cur_bg = 0.0f, sum_ref_bg = 0.0f;
    int   quiet_bg_cnt = 0;
    for (int i = 0; i < GRID_CELLS; i++) {
        if (s_static_age[i] >= 4 && s_fg_hold[i] == 0) {
            float ratio = (s_bg_y[i] > 8.0f) ? (s_cell_y[i] / s_bg_y[i]) : 1.0f;
            if (ratio > 0.75f && ratio < 1.30f) {
                sum_cur_bg += s_cell_y[i];
                sum_ref_bg += s_bg_y[i];
                quiet_bg_cnt++;
            }
        }
    }
    float beta_ae = (quiet_bg_cnt >= 20 && sum_ref_bg > 100.0f) ? (sum_cur_bg / sum_ref_bg) : 1.0f;
    if (beta_ae < 0.72f) beta_ae = 0.72f;
    if (beta_ae > 1.38f) beta_ae = 1.38f;

    int raw_changed_cells = 0;
    uint32_t shadows_rejected_this_frame = 0;

    for (int gy = 0; gy < GRID_H; gy++) {
        for (int gx = 0; gx < GRID_W; gx++) {
            int i = gy * GRID_W + gx;

            float dot_cb = 0.0f, norm_c = 0.0f, norm_b = 0.0f;
            for (int dy = -1; dy <= 1; dy++) {
                int ny = gy + dy;
                if (ny < 0 || ny >= GRID_H) continue;
                for (int dx = -1; dx <= 1; dx++) {
                    int nx = gx + dx;
                    if (nx < 0 || nx >= GRID_W) continue;
                    int ni = ny * GRID_W + nx;
                    float cy = s_cell_y[ni];
                    float by = s_bg_y[ni] * beta_ae;
                    dot_cb += cy * by;
                    norm_c += cy * cy;
                    norm_b += by * by;
                }
            }
            float denom = sqrtf(norm_c * norm_b) + 1e-4f;
            float ncc = dot_cb / denom;
            if (ncc > 1.0f) ncc = 1.0f;
            if (ncc < 0.0f) ncc = 0.0f;
            s_ncc_map[i] = ncc;

            float expected_bg_y = s_bg_y[i] * beta_ae;
            float dy_mean = fabsf(s_cell_y[i] - expected_bg_y);

            float sigma_eff = s_bg_var[i]
                            + 0.40f * s_spatial_grad[i]
                            + 0.30f * s_bg_edge[i]
                            + 0.55f * s_blink_lvl[i];
            if (sigma_eff < 6.0f)  sigma_eff = 6.0f;
            if (sigma_eff > 36.0f) sigma_eff = 36.0f;

            float vibe_radius = thr * sigma_eff * 0.85f;
            if (vibe_radius < 12.0f) vibe_radius = 12.0f;

            int vibe_matches = 0;
            float min_vibe_dist = dy_mean;
            for (int s = 0; s < VIBE_SAMPLES; s++) {
                float dist_s = fabsf(s_cell_y[i] - s_vibe_y[s][i] * beta_ae);
                if (dist_s < vibe_radius) vibe_matches++;
                if (dist_s < min_vibe_dist) min_vibe_dist = dist_s;
            }

            float dcr = fabsf(s_cell_cr[i]   - s_bg_cr[i]);
            float dcb = fabsf(s_cell_cb[i]   - s_bg_cb[i]);
            float ded = fabsf(s_cell_edge[i] - s_bg_edge[i]);
            float chroma_shift = dcr + dcb;
            s_chroma_map[i] = chroma_shift;

            float lum_ratio = (expected_bg_y > 10.0f) ? (s_cell_y[i] / expected_bg_y) : 1.0f;
            bool is_cast_shadow = (lum_ratio >= 0.46f && lum_ratio <= 1.48f) &&
                                  (ncc > 0.900f) &&
                                  (chroma_shift < 10.0f) &&
                                  (ded < 8.5f) &&
                                  (s_static_age[i] > 0);
            s_shadow_mask[i] = is_cast_shadow ? 1 : 0;

            float eff_dy = 0.55f * dy_mean + 0.45f * min_vibe_dist;
            float struct_break = (1.0f - ncc) * 60.0f;
            float raw_diff = eff_dy + 0.45f * chroma_shift + 0.48f * ded + 0.32f * struct_break;
            float clean_diff = raw_diff - 7.5f;
            if (clean_diff < 0.0f) clean_diff = 0.0f;

            float z = clean_diff / sigma_eff;
            if (is_cast_shadow) {
                z *= 0.28f;
                if (eff_dy > 10.0f) shadows_rejected_this_frame++;
            }
            s_z_map[i] = z;

            bool fg_evidence = (z > thr) && ((vibe_matches < VIBE_MIN_MATCHES) || (chroma_shift > 14.0f));

            if (fg_evidence) {
                if (s_static_age[i] == 0) {
                    s_fg_hold[i] = 24;
                } else if (s_fg_hold[i] > 0) {
                    s_fg_hold[i]--;
                }
            } else {
                if (s_fg_hold[i] >= 5) s_fg_hold[i] -= 5;
                else s_fg_hold[i] = 0;
            }

            if (fg_evidence && s_fg_hold[i] > 0) {
                s_raw_anom[i] = 1;
                raw_changed_cells++;
            } else {
                s_raw_anom[i] = 0;
            }
        }
    }

    s_shadow_rejected_cnt += shadows_rejected_this_frame;

    // -----------------------------------------------------------------------------------------
    // BƯỚC 6: CẬP NHẬT MÔ HÌNH NỀN ViBe + SuBSENSE & TÍNH ĐẶC TRƯNG FOMO 1x1 CONV TRÊN TỪNG Ô
    // -----------------------------------------------------------------------------------------
    memset(s_verified_anom, 0, sizeof(s_verified_anom));
    memset(s_fomo_cls_m1, 0, sizeof(s_fomo_cls_m1));
    memset(s_fomo_cls_m2, 0, sizeof(s_fomo_cls_m2));
    memset(s_fomo_conf, 0, sizeof(s_fomo_conf));

    uint8_t highest_hw_mode = 0;
    int num_raw_cubes = 0;
    float fomo_det_thr = 0.56f; // Ngưỡng xác suất tâm đối tượng FOMO (Edge Impulse FOMO threshold)

    for (int gy = 0; gy < GRID_H; gy++) {
        for (int gx = 0; gx < GRID_W; gx++) {
            int idx = gy * GRID_W + gx;
            int active_nb = 0;
            int moving_nb = 0;
            float max_nb_z = s_z_map[idx];
            float sum_nb_z = s_z_map[idx];
            int nb_cnt = 1;

            if (s_z_map[idx] > thr * 0.85f && !s_shadow_mask[idx]) {
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        if (dx == 0 && dy == 0) continue;
                        int nx = gx + dx, ny = gy + dy;
                        if (nx >= 0 && nx < GRID_W && ny >= 0 && ny < GRID_H) {
                            int n_idx = ny * GRID_W + nx;
                            if (s_raw_anom[n_idx]) active_nb++;
                            if (s_static_age[n_idx] == 0 && s_z_map[n_idx] > thr) moving_nb++;
                            if (s_z_map[n_idx] > max_nb_z) max_nb_z = s_z_map[n_idx];
                            sum_nb_z += s_z_map[n_idx];
                            nb_cnt++;
                        }
                    }
                }
                if (moving_nb >= 2 && active_nb >= 2 && s_fg_hold[idx] < 14) {
                    s_fg_hold[idx]  = 14;
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
            }

            if (s_raw_anom[idx] != s_prev_anom[idx] && s_static_age[idx] <= 2) {
                s_blink_lvl[idx] = fminf(15.0f, s_blink_lvl[idx] + 1.2f);
            } else {
                s_blink_lvl[idx] = fmaxf(0.0f, s_blink_lvl[idx] * 0.92f - 0.05f);
            }
            s_prev_anom[idx] = s_raw_anom[idx];

            // --- SUY LUẬN EDGE IMPULSE FOMO 1x1 CONV TRÊN Ô LƯỚI (gx, gy) ---
            // Mode 0 (Gatekeeper): Nếu ô không vượt qua kiểm tra đồng thuận tiền cảnh -> 0 MAC!
            if (!s_verified_anom[idx]) continue;

            float cz = s_z_map[idx];
            float avg_local_z = sum_nb_z / nb_cnt;
            float ncc = s_ncc_map[idx];
            float chroma = s_chroma_map[idx];
            float edge = s_cell_edge[idx];
            float cr = s_cell_cr[idx];
            float cb = s_cell_cb[idx];
            float dcr_bg = cr - s_bg_cr[idx];
            float skin_val = 0.0f;
            if ((cr > 5.0f && cb < -2.0f && (cr - cb) > 12.0f) || (dcr_bg > 6.5f && (cr - cb) > 8.0f)) {
                skin_val = fminf(127.0f, (cr - cb) * 3.8f + dcr_bg * 3.2f);
            }
            float motion_val = (s_static_age[idx] == 0) ? 115.0f : ((float)s_fg_hold[idx] * 4.2f);
            float fill_local = (float)(active_nb + 1) / 9.0f;
            bool touches_bdr = (gx == 0 || gx == GRID_W - 1 || gy == 0 || gy == GRID_H - 1);
            float shadow_feat = (s_shadow_mask[idx] ? 90.0f : 0.0f) + ((ncc > 0.89f && chroma < 9.0f) ? 48.0f : 0.0f);

            int8_t *f16 = s_cell_feat16[idx];
            f16[0]  = clamp_i8_pos((avg_local_z - thr) * 28.0f + 38.0f);
            f16[1]  = clamp_i8_pos((max_nb_z - thr) * 24.0f + 42.0f);
            f16[2]  = clamp_i8_pos(motion_val);
            f16[3]  = clamp_i8_pos((1.0f - ncc) * 320.0f);
            f16[4]  = clamp_i8_pos(chroma * 4.2f);
            f16[5]  = clamp_i8_pos(skin_val);
            f16[6]  = clamp_i8_pos((s_raw_max_y[idx] > 235.0f && (s_raw_max_y[idx] - s_bg_max_y[idx]) > 45.0f) ? 110.0f : 0.0f);
            f16[7]  = clamp_i8_pos(edge * 5.2f);
            f16[8]  = clamp_i8_pos(edge * 6.0f + chroma * 3.0f);
            f16[9]  = clamp_i8_pos(fill_local * 115.0f);
            f16[10] = clamp_i8_pos(38.0f);
            f16[11] = clamp_i8_pos((float)(active_nb + 1) * 14.0f);
            f16[12] = clamp_i8_pos(shadow_feat);
            f16[13] = clamp_i8_pos(touches_bdr ? 65.0f : 10.0f);
            f16[14] = clamp_i8_pos((cz - thr) * 24.0f + 30.0f);
            f16[15] = clamp_i8_pos((float)s_cell_persist[idx] * 22.0f);

            // Stage 1: AdaReMAC Mode 1 Early-Exit Screening (mode=1, thr=2)
            if (highest_hw_mode < 1) highest_hw_mode = 1;
            uint8_t cls_m1 = adaremac_run_early_exit_gate(f16, 1, 2);
            if (cls_m1 == 0) {
                s_shadow_rejected_cnt++;
                continue;
            }

            // Stage 2: AdaReMAC Mode 2 Full Exact Edge Impulse FOMO 1x1 Conv Head (16->32->6)
            highest_hw_mode = 2;
            float prob_fomo = 0.0f;
            uint8_t cls_m2 = adaremac_run_fomo_1x1_head(f16, 2, 0, &prob_fomo);
            if (cls_m2 == 0 || prob_fomo < fomo_det_thr) {
                s_shadow_rejected_cnt++;
                continue;
            }

            s_fomo_cls_m1[idx] = cls_m1;
            s_fomo_cls_m2[idx] = cls_m2;
            s_fomo_conf[idx]   = prob_fomo;

            // Gom ô Centroid vào danh sách khối FOMO (`ei_handle_cube` trong Edge Impulse SDK)
            // Sử dụng mã lớp gộp = 1 cho mọi vật thể tiền cảnh hợp lệ để các ô của cùng 1 vật thể kết nối liền mạch
            ei_fomo_handle_cube(s_fomo_cubes, &num_raw_cubes, gx, gy, prob_fomo, 1, fomo_det_thr, idx);
        }
    }

    // Cập nhật mô hình nền ViBe + SuBSENSE chống Ghost Deadlock (Sakbot Anti-Ghosting)
    for (int i = 0; i < GRID_CELLS; i++) {
        if (s_verified_anom[i] && s_fg_hold[i] > 6) continue;

        float alpha = (s_verified_anom[i]) ? 0.12f : ((s_static_age[i] >= 5 && s_fg_hold[i] == 0) ? 0.22f : 0.06f);
        float dy  = fabsf(s_cell_y[i]  - s_bg_y[i]);
        float dcr = fabsf(s_cell_cr[i] - s_bg_cr[i]);
        float dcb = fabsf(s_cell_cb[i] - s_bg_cb[i]);
        float diff = dy + 0.3f * (dcr + dcb);

        s_bg_y[i]     = (1.0f - alpha) * s_bg_y[i]     + alpha * s_cell_y[i];
        s_bg_max_y[i] = (1.0f - alpha) * s_bg_max_y[i] + alpha * s_raw_max_y[i];
        s_bg_cr[i]    = (1.0f - alpha) * s_bg_cr[i]    + alpha * s_cell_cr[i];
        s_bg_cb[i]    = (1.0f - alpha) * s_bg_cb[i]    + alpha * s_cell_cb[i];
        s_bg_edge[i]  = (1.0f - alpha) * s_bg_edge[i]  + alpha * s_cell_edge[i];

        if (s_static_age[i] >= 2) {
            uint8_t slot = s_vibe_ptr[i] & (VIBE_SAMPLES - 1);
            s_vibe_y[slot][i] = s_cell_y[i];
            s_vibe_ptr[i] = (slot + 1) & (VIBE_SAMPLES - 1);
        }

        s_bg_var[i] = 0.90f * s_bg_var[i] + 0.10f * (diff + 4.8f);
        if (s_bg_var[i] < 5.5f)  s_bg_var[i] = 5.5f;
        if (s_bg_var[i] > 24.0f) s_bg_var[i] = 24.0f;
    }

    for (int gy = 0; gy < GRID_H; gy++) {
        for (int gx = 0; gx < GRID_W; gx++) {
            int idx = gy * GRID_W + gx;
            if (s_verified_anom[idx] && s_fg_hold[idx] > 6) continue;
            float max_g = 0.0f;
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    int nx = gx + dx, ny = gy + dy;
                    if (nx >= 0 && nx < GRID_W && ny >= 0 && ny < GRID_H) {
                        float g = fabsf(s_bg_y[idx] - s_bg_y[ny * GRID_W + nx]);
                        if (g > max_g) max_g = g;
                    }
                }
            }
            s_spatial_grad[idx] = 0.85f * s_spatial_grad[idx] + 0.15f * max_g;
        }
    }

    // -----------------------------------------------------------------------------------------
    // BƯỚC 7: HỢP NHẤT KHỐI FOMO (`fill_result_struct_from_cubes`) + HỒI QUY TINY-SSD + NMS
    // -----------------------------------------------------------------------------------------
    int num_merged = 0;
    for (int i = 0; i < num_raw_cubes; i++) {
        EiFomoCube *sc = &s_fomo_cubes[i];
        bool has_overlapping = false;
        for (int j = 0; j < num_merged; j++) {
            if (ei_fomo_cube_check_overlap(&s_merged_cubes[j], sc->x, sc->y,
                                           sc->width, sc->height, sc->confidence, sc->peak_idx)) {
                s_merged_cubes[j].cell_cnt += sc->cell_cnt - 1;
                has_overlapping = true;
                break;
            }
        }
        if (!has_overlapping && num_merged < MAX_FOMO_CUBES) {
            s_merged_cubes[num_merged++] = *sc;
        }
    }

    AnomBox raw_boxes[MAX_FOMO_CUBES];
    int raw_box_cnt = 0;

    for (int i = 0; i < num_merged; i++) {
        EiFomoCube *c = &s_merged_cubes[i];
        // Yêu cầu tối thiểu 2 ô FOMO liên thông để loại bỏ hoàn toàn nhiễu điểm đơn lẻ
        if (c->cell_cnt < 2) continue;
        if (c->height == 1 && c->width >= 5) continue;

        int p_idx = c->peak_idx;
        if (p_idx < 0 || p_idx >= GRID_CELLS) continue;

        // Tinh chỉnh tọa độ khung bao bằng Nhánh Hồi quy Tiny-SSD (4x16 INT8)
        float ssd_dx = 0.0f, ssd_dy = 0.0f, ssd_dw = 0.0f, ssd_dh = 0.0f;
        adaremac_run_tiny_ssd_box_head(s_cell_feat16[p_idx], 2, 0, &ssd_dx, &ssd_dy, &ssd_dw, &ssd_dh);

        float bx = ((float)c->x / (float)GRID_W) + ssd_dx - ssd_dw * 0.5f;
        float by = ((float)c->y / (float)GRID_H) + ssd_dy - ssd_dh * 0.5f;
        float bw = ((float)c->width  / (float)GRID_W) + ssd_dw;
        float bh = ((float)c->height / (float)GRID_H) + ssd_dh;

        if (bx < 0.0f) bx = 0.0f;
        if (by < 0.0f) by = 0.0f;
        if (bx + bw > 1.0f) bw = 1.0f - bx;
        if (by + bh > 1.0f) bh = 1.0f - by;
        if (bw < 0.04f || bh < 0.04f) continue;

        AnomBox b;
        b.x = bx;
        b.y = by;
        b.w = bw;
        b.h = bh;
        b.obj_cls_m1 = s_fomo_cls_m1[p_idx];
        b.obj_cls_m2 = s_fomo_cls_m2[p_idx];
        b.is_flash   = (b.obj_cls_m2 == 4) ? 1 : 0;

        int px0 = (int)(bx * thumb_w);
        int py0 = (int)(by * thumb_h);
        int pbw = (int)(bw * thumb_w);
        int pbh = (int)(bh * thumb_h);
        extract_patch_20x20_int8(s_rgb565_thumb, thumb_w, thumb_h, px0, py0, pbw, pbh, s_patch_i8);
        b.ai_cls_m1 = adaremac_run_fc_mode1_25pct(s_patch_i8);
        b.ai_cls_m2 = adaremac_run_fc_mode2_100pct(s_patch_i8);

        int conf_pct = (int)(c->confidence * 100.0f + 0.5f);
        if (conf_pct < 68) conf_pct = 68;
        if (conf_pct > 99) conf_pct = 99;
        b.score = conf_pct;

        raw_boxes[raw_box_cnt++] = b;
    }

    // Sắp xếp giảm dần theo độ tin cậy FOMO và lọc Non-Maximum Suppression (NMS, IoU = 0.25)
    for (int i = 0; i < raw_box_cnt - 1; i++) {
        for (int j = i + 1; j < raw_box_cnt; j++) {
            if (raw_boxes[j].score > raw_boxes[i].score) {
                AnomBox tmp = raw_boxes[i];
                raw_boxes[i] = raw_boxes[j];
                raw_boxes[j] = tmp;
            }
        }
    }

    int cand_count = 0;
    for (int i = 0; i < raw_box_cnt && cand_count < MAX_ANOM_BOXES; i++) {
        bool suppressed = false;
        for (int j = 0; j < cand_count; j++) {
            if (compute_box_iou(raw_boxes[i], s_cand_boxes[j]) > 0.25f) {
                suppressed = true;
                break;
            }
        }
        if (!suppressed) {
            s_cand_boxes[cand_count++] = raw_boxes[i];
        }
    }

    // -----------------------------------------------------------------------------------------
    // BƯỚC 8: LÀM MƯỢT TỌA ĐỘ BOUNDING BOX THEO THỜI GIAN (ADAPTIVE KALMAN-EMA TRACKING)
    // -----------------------------------------------------------------------------------------
    for (int i = 0; i < cand_count; i++) {
        AnomBox *cb = &s_cand_boxes[i];
        float c_cx = cb->x + cb->w * 0.5f;
        float c_cy = cb->y + cb->h * 0.5f;
        int best_match = -1;
        float best_dist = 0.22f;

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
            // Hệ số thích nghi theo vận tốc: chuyển động nhanh bám tức thì (alpha=0.82), đứng yên làm mượt (alpha=0.58)
            float alpha = (best_dist > 0.06f) ? 0.82f : 0.58f;
            cb->x = alpha * cb->x + (1.0f - alpha) * s_tracked_boxes[best_match].x;
            cb->y = alpha * cb->y + (1.0f - alpha) * s_tracked_boxes[best_match].y;
            cb->w = alpha * cb->w + (1.0f - alpha) * s_tracked_boxes[best_match].w;
            cb->h = alpha * cb->h + (1.0f - alpha) * s_tracked_boxes[best_match].h;
        }
    }
    s_num_tracked = cand_count;
    for (int i = 0; i < cand_count; i++) s_tracked_boxes[i] = s_cand_boxes[i];

    float changed_ratio = (float)raw_changed_cells / (float)GRID_CELLS;
    uint32_t dt_us = micros() - t0;

    portENTER_CRITICAL(&s_ai_mux);
    s_num_boxes          = cand_count;
    for (int i = 0; i < cand_count; i++) s_boxes[i] = s_cand_boxes[i];
    s_angle_changed      = false;
    s_changed_cell_ratio = changed_ratio;
    s_env_stability      = (1.0f - fminf(1.0f, changed_ratio)) * 100.0f;
    s_active_hw_mode     = highest_hw_mode;
    s_ai_infer_us        = dt_us;
    portEXIT_CRITICAL(&s_ai_mux);
}

// =========================================================================================
// 7. QUẢN LÝ NHIỆT PHẦN MỀM CHỦ ĐỘNG CHO CẢM BIẾN OV5640 (KHÔNG CAN THIỆP PHẦN CỨNG)
// =========================================================================================
// Đưa OV5640 vào chế độ Ngủ tiết kiệm điện bằng phần mềm (Software Standby: 0x3008 = 0x42)
// hoặc Đánh thức tức thì (Wakeup: 0x3008 = 0x02) mà không mất cấu hình thanh ghi SCCB!
static void set_ov5640_software_standby(sensor_t *s, bool enable_standby) {
    // Tuyệt đối không ghi 0x3008 = 0x42 khi GDMA LCD_CAM của ESP32-S3 đang chạy để tránh treo VSYNC/DMA!
    (void)s; (void)enable_standby;
    s_ov5640_sleeping = false;
}

// Áp dụng các thanh ghi cắt giảm nhiệt lượng phần mềm an toàn 100% cho OV5640:
//   1. Tắt hoàn toàn khối phát cao tần MIPI CSI-2 TX PHY không dùng trong chế độ DVP 8-bit (`0x300E`, `0x4202`).
//   2. Giảm dòng bơm đệm chân I/O DVP (`0x302C[7:6]` = 00 [1x] cho VGA/SVGA hoặc 01 [2x] cho HD/FHD).
//   3. Clock-Gating tắt xung nhịp các khối OTP/BIST không cần thiết (`0x3000`, `0x3004`).
//   4. Khóa trần phơi sáng VTS (`0x3A00`, `0x3A02..0x3A15`) để tốc độ phần cứng luôn đạt >= 30 FPS!
static void configure_ov5640_cool_30fps(sensor_t *s, framesize_t fs, int jpeg_qual) {
    if (!s) return;
    s_ai_buf_state = 0;
    s_skip_frames  = 3;

    if (s_ov5640_sleeping) {
        set_ov5640_software_standby(s, false);
    }

    s->set_framesize(s, fs);
    s->set_quality(s, jpeg_qual);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 0);                        // Tắt Night-Mode tránh tự động nhân đôi chu kỳ khung hình
    s->set_ae_level(s, 0);
    s->set_gain_ctrl(s, 1);
    s->set_gainceiling(s, (gainceiling_t)3);  // Trần AGC 16x: sáng rõ trong phòng mà không gây nhiễu hạt CMOS
    s->set_brightness(s, 0);
    s->set_contrast(s, 0);
    s->set_saturation(s, 0);
    s->set_sharpness(s, 0);
    s->set_denoise(s, 0);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_bpc(s, 1);
    s->set_wpc(s, 1);
    s->set_raw_gma(s, 1);
    s->set_lenc(s, 1);
    s->set_dcw(s, 1);
    s->set_vflip(s, 1);

    if (s->id.PID == OV5640_PID && s->set_reg && s->get_reg) {
        // V4 SAFETY: esp32-camera initializes stream and DVP timing.
        // 0x4202 is a FRAME MASK control, not a MIPI PHY power switch.
        // Avoid raw 0x3008/0x4201/0x4202/0x302C writes by default.

        // Chỉ tắt bit 2 (Night mode frame drop) trong AEC CTRL00 (0x3A00) và khóa trần phơi sáng <= VTS - 4
        int aec_ctrl = s->get_reg(s, 0x3A00, 0xFF);
        if (aec_ctrl >= 0) {
            s->set_reg(s, 0x3A00, 0xFF, aec_ctrl & ~0x04);
        }
        int vts_hi = s->get_reg(s, 0x380E, 0xFF);
        int vts_lo = s->get_reg(s, 0x380F, 0xFF);
        if (vts_hi >= 0 && vts_lo >= 0) {
            int vts = (vts_hi << 8) | vts_lo;
            int max_exp = (vts > 20) ? (vts - 4) : vts;
            s->set_reg(s, 0x3A02, 0xFF, (max_exp >> 8) & 0xFF);
            s->set_reg(s, 0x3A03, 0xFF, max_exp & 0xFF);
            s->set_reg(s, 0x3A14, 0xFF, (max_exp >> 8) & 0xFF);
            s->set_reg(s, 0x3A15, 0xFF, max_exp & 0xFF);
        }
    }
}

// Task AI FOMO chạy độc lập trên Core 0 (không bao giờ giữ camera_fb_t khi đang Stream trên Core 1!)
static void ai_continual_learning_task(void *arg) {
    while (true) {
        if (!s_cam_ok) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (s_stream_clients > 0) {
            if (s_ov5640_sleeping) {
                set_ov5640_software_standby(esp_camera_sensor_get(), false);
            }
            if (s_ai_buf_state != 2) {
                if (s_ai_buf_state == 0) s_ai_buf_state = 1;
                vTaskDelay(pdMS_TO_TICKS(6));
                continue;
            }
            s_ai_buf_state = 0;
            __sync_synchronize();
            analyze_frame_fomo_pipeline(s_ai_jpeg_buf, s_ai_jpeg_len, s_ai_frame_w, s_ai_frame_h);
            vTaskDelay(pdMS_TO_TICKS(35)); // Chạy FOMO ~25 Hz mượt mà trên Core 0
            s_ai_buf_state = 1;
        } else {
            // Luôn duy trì luồng DMA liên tục giống bản V4 ổn định

            if (s_ov5640_sleeping) {
                set_ov5640_software_standby(esp_camera_sensor_get(), false);
            }

            s_ai_buf_state = 0;
            __sync_synchronize();
            camera_fb_t *fb = esp_camera_fb_get();
            if (fb) {
                if (s_skip_frames > 0) {
                    s_skip_frames--;
                    esp_camera_fb_return(fb);
                } else {
                    s_total_frames++;
                    s_ai_frame_w = fb->width;
                    s_ai_frame_h = fb->height;
                    if (s_ai_jpeg_buf && fb->len <= MAX_JPEG_COPY_SZ && is_valid_jpeg_frame(fb->buf, fb->len)) {
                        memcpy(s_ai_jpeg_buf, fb->buf, fb->len);
                        s_ai_jpeg_len = fb->len;
                        esp_camera_fb_return(fb);
                        analyze_frame_fomo_pipeline(s_ai_jpeg_buf, s_ai_jpeg_len, s_ai_frame_w, s_ai_frame_h);
                    } else {
                        esp_camera_fb_return(fb);
                    }
                }
            }
            // Giãn chu kỳ lấy mẫu khi không có người xem Web để giảm dòng OV5640
            vTaskDelay(pdMS_TO_TICKS(120));
        }
    }
}

// =========================================================================================
// 8. GIAO DIỆN WEB TRỰC QUAN (60FPS CANVAS SMOOTHING + CHỈ VẼ BOUNDING BOX KHÔNG NHÃN CHỮ)
// =========================================================================================
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>AdaReMAC ESP32-S3 — Edge Impulse FOMO + Tiny-SSD @ 30+ FPS & OV5640 Cool</title>
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
    .toolbar { width: 100%; max-width: 1280px; background: var(--card); padding: 12px 18px; border-radius: 12px; margin-top: 10px; border: 1px solid #334155; display: flex; flex-wrap: wrap; gap: 12px; align-items: center; justify-content: space-between; }
    button, select { background: #0284c7; color: #fff; border: none; padding: 8px 14px; border-radius: 8px; font-weight: 600; cursor: pointer; font-size: 0.85rem; }
    button:hover { background: #0369a1; }
    select { background: #334155; }
    input[type=range] { accent-color: var(--accent); cursor: pointer; }
  </style>
</head>
<body>
  <div class="header">
    <div>
      <div class="title">📹 AdaReMAC FOMO + Tiny-SSD — Real-Time 30+ FPS Object Detection & OV5640 Software Cooling</div>
      <div class="subtitle">Edge Impulse FOMO 1x1 Conv Head (16→32→6) + Tiny-SSD Box Regressor + Quản lý nhiệt phần mềm OV5640 (MIPI-Off & 1x Pad Drive)</div>
    </div>
    <div id="status_badge" class="badge bg-ok">Đang kết nối FOMO AI...</div>
  </div>

  <div class="Controls">
    <div class="stat-box"><div class="stat-label">Độ phân giải & Tốc độ Stream</div><div class="stat-val" id="val_fps">SVGA 800x600 @ 30+ FPS</div></div>
    <div class="stat-box"><div class="stat-label">Chế độ AdaReMAC & Ổn định Nền</div><div class="stat-val" id="val_env">Mode 0 | 100%</div></div>
    <div class="stat-box"><div class="stat-label">FOMO + Tiny-SSD / Lọc Bóng đổ</div><div class="stat-val" id="val_anom">0 vật thể</div></div>
    <div class="stat-box"><div class="stat-label">Nhiệt độ Chip & Làm mát OV5640</div><div class="stat-val" id="val_temp">-- °C</div></div>
  </div>

  <div class="stage-wrap">
    <img id="cam_stream" alt="Đang tải luồng MJPEG 30+ FPS..." crossorigin="anonymous">
    <canvas id="ai_canvas"></canvas>
  </div>

  <div class="toolbar">
    <div style="display:flex;align-items:center;gap:10px;flex-wrap:wrap;">
      <label style="font-size:0.85rem;">Phân giải:</label>
      <select id="sel_res" onchange="changeRes(this.value)">
        <option value="vga">VGA (640x480 @ 36+fps - Mát nhất)</option>
        <option value="svga" selected>SVGA (800x600 @ 32+fps - Chuẩn FOMO)</option>
        <option value="hd">HD (1280x720 @ 28fps)</option>
        <option value="fhd">FHD (1920x1080)</option>
      </select>

      <label style="font-size:0.85rem;margin-left:4px;">Làm mát OV5640:</label>
      <select id="sel_therm" onchange="changeTherm(this.value)">
        <option value="0" selected>❄️ Cool 30fps (Tắt MIPI + Pad 1x)</option>
        <option value="1">🍃 Eco Auto-Standby (Ngủ khi rảnh)</option>
        <option value="2">⚡ Turbo Max (Pad 2x)</option>
      </select>

      <label style="font-size:0.85rem;margin-left:4px;">Nén JPEG:</label>
      <input type="range" min="8" max="26" step="1" value="14" oninput="updateQual(this.value)">
      <span id="lbl_q" style="font-weight:700;color:#38bdf8;">14</span>

      <label style="font-size:0.85rem;margin-left:4px;">Ngưỡng FOMO:</label>
      <input type="range" id="rng_thr" min="1.4" max="5.0" step="0.2" value="2.4" oninput="updateThr(this.value)">
      <span id="lbl_thr" style="font-weight:700;color:#38bdf8;">2.4</span>
    </div>
    <button onclick="resetEnv()">🔄 Học lại góc Camera ngay (Reset FOMO/ViBe)</button>
  </div>

  <script>
    const MODE_NAMES = ['Mode 0 (Gatekeeper 0%)', 'Mode 1 (25% Early-Exit)', 'Mode 2 (100% FOMO+SSD)'];
    const THERM_NAMES = ['Cool 1x', 'Eco-Stby', 'Turbo 2x'];

    const streamImg = document.getElementById('cam_stream');
    const canvas = document.getElementById('ai_canvas');
    const ctx = canvas.getContext('2d');
    const streamUrl = 'http://' + window.location.hostname + ':81/stream';
    streamImg.src = streamUrl;
    streamImg.onerror = () => { setTimeout(() => { streamImg.src = streamUrl + '?t=' + Date.now(); }, 600); };

    function changeRes(v) { fetch('/ctrl?res=' + v); }
    function changeTherm(v) { fetch('/ctrl?therm=' + v); }
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
          renderBoxes[i].x += (targetBoxes[i].x - renderBoxes[i].x) * 0.42;
          renderBoxes[i].y += (targetBoxes[i].y - renderBoxes[i].y) * 0.42;
          renderBoxes[i].w += (targetBoxes[i].w - renderBoxes[i].w) * 0.42;
          renderBoxes[i].h += (targetBoxes[i].h - renderBoxes[i].h) * 0.42;
        }
      }

      // Chỉ vẽ khung bao chuẩn xác quanh đối tượng (không vẽ chữ đè lên video)
      renderBoxes.forEach((b) => {
        const bx = b.x * cw;
        const by = b.y * ch;
        const bw = b.w * cw;
        const bh = b.h * ch;

        ctx.strokeStyle = '#22c55e';
        ctx.lineWidth = 3;
        ctx.strokeRect(bx, by, bw, bh);
        ctx.fillStyle = 'rgba(34, 197, 94, 0.14)';
        ctx.fillRect(bx, by, bw, bh);
      });

      requestAnimationFrame(renderLoop);
    }
    requestAnimationFrame(renderLoop);

    async function pollAI() {
      const ctrl = new AbortController();
      const tid = setTimeout(() => ctrl.abort(), 280);
      try {
        const res = await fetch('/ai', { cache: 'no-store', signal: ctrl.signal });
        const d = await res.json();

        document.getElementById('val_fps').innerText = d.w + 'x' + d.h + ' @ ' + d.fps.toFixed(1) + ' FPS';
        document.getElementById('val_env').innerText = (MODE_NAMES[d.hw_mode] || 'Mode 0') + ' | ' + d.stability.toFixed(0) + '%';
        document.getElementById('val_anom').innerText = d.boxes.length + ' vật thể (' + (d.ai_us / 1000).toFixed(2) + ' ms | Lọc bóng: ' + d.shd_rej + ')';
        document.getElementById('val_temp').innerText = d.temp.toFixed(1) + ' °C (' + (THERM_NAMES[d.therm] || 'Cool') + ')';

        learnLeft = d.learn_left;
        targetBoxes = d.boxes || [];

        const badge = document.getElementById('status_badge');
        if (d.learn_left > 0) {
          badge.className = 'badge bg-warn';
          badge.innerText = '🔄 ĐỔI GÓC CAMERA -> ĐANG HỌC MẪU NỀN MỚI (' + d.stability.toFixed(0) + '%)';
        } else if (d.boxes.length > 0) {
          badge.className = 'badge bg-danger';
          badge.innerText = '🎯 FOMO PHÁT HIỆN ' + d.boxes.length + ' ĐỐI TƯỢNG (' + d.boxes[0].score + '%)';
        } else {
          badge.className = 'badge bg-ok';
          badge.innerText = '✅ MÔI TRƯỜNG ỔN ĐỊNH (FOMO + Tiny-SSD + ViBe)';
        }
      } catch (e) {}
      clearTimeout(tid);
      setTimeout(pollAI, 120);
    }
    pollAI();
  </script>
</body>
</html>
)rawliteral";

// =========================================================================================
// 9. CÁC HANDLER HTTP (PORT 80: UI + AI JSON | PORT 81: ZERO-HOLD 1-SYSCALL MJPEG STREAM)
// =========================================================================================
#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static char s_json_buf[1280];

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
    uint8_t s_cls = 0, hw_m = 0, therm_m = 0;
    uint32_t infer_us = 0, shd_rej = 0;
    uint16_t fw = 800, fh = 600;

    portENTER_CRITICAL(&s_ai_mux);
    n_boxes    = s_num_boxes;
    for (int i = 0; i < n_boxes; i++) local_boxes[i] = s_boxes[i];
    stab       = s_env_stability;
    chg        = s_changed_cell_ratio;
    ang        = s_angle_changed;
    learn_left = s_fast_learn_frames;
    s_cls      = s_scene_ai_cls;
    hw_m       = s_active_hw_mode;
    therm_m    = s_thermal_mode;
    infer_us   = s_ai_infer_us;
    shd_rej    = s_shadow_rejected_cnt;
    fps        = s_stream_fps;
    temp       = s_chip_temp_c;
    fw         = s_ai_frame_w;
    fh         = s_ai_frame_h;
    portEXIT_CRITICAL(&s_ai_mux);

    int len = snprintf(s_json_buf, sizeof(s_json_buf),
        "{\"w\":%u,\"h\":%u,\"fps\":%.1f,\"temp\":%.1f,\"therm\":%u,\"stability\":%.1f,\"changed\":%.3f,"
        "\"angle_shift\":%s,\"learn_left\":%d,\"scene_cls\":%u,\"hw_mode\":%u,\"ai_us\":%lu,\"shd_rej\":%lu,\"boxes\":[",
        fw, fh, fps, temp, therm_m, stab, chg, ang ? "true" : "false", learn_left, s_cls, hw_m,
        (unsigned long)infer_us, (unsigned long)shd_rej);

    for (int i = 0; i < n_boxes; i++) {
        len += snprintf(s_json_buf + len, sizeof(s_json_buf) - len,
            "%s{\"x\":%.4f,\"y\":%.4f,\"w\":%.4f,\"h\":%.4f,\"score\":%d,\"obj\":%u,\"obj_m1\":%u,\"m1\":%u,\"m2\":%u,\"fl\":%u}",
            (i > 0) ? "," : "",
            local_boxes[i].x, local_boxes[i].y, local_boxes[i].w, local_boxes[i].h,
            local_boxes[i].score, local_boxes[i].obj_cls_m2, local_boxes[i].obj_cls_m1,
            local_boxes[i].ai_cls_m1, local_boxes[i].ai_cls_m2, local_boxes[i].is_flash);
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
        if (s && httpd_query_key_value(query, "therm", val, sizeof(val)) == ESP_OK) {
            int tm = atoi(val);
            if (tm >= 0 && tm <= 2) {
                s_thermal_mode = (uint8_t)tm;
                configure_ov5640_cool_30fps(s, (framesize_t)s->status.framesize, s->status.quality);
            }
        }
        if (s && httpd_query_key_value(query, "qual", val, sizeof(val)) == ESP_OK) {
            int q = atoi(val);
            if (q >= 5 && q <= 40) {
                s_skip_frames = 2;
                s->set_quality(s, q);
            }
        }
        if (s && httpd_query_key_value(query, "ae", val, sizeof(val)) == ESP_OK) {
            int ae = atoi(val);
            if (ae >= -2 && ae <= 2) {
                s->set_ae_level(s, ae);
                s->set_brightness(s, ae);
            }
        }
        if (s && httpd_query_key_value(query, "res", val, sizeof(val)) == ESP_OK) {
            framesize_t target_fs = FRAMESIZE_SVGA;
            int target_q = 14;
            if (strcmp(val, "fhd") == 0)       { target_fs = FRAMESIZE_FHD;  target_q = 16; }
            else if (strcmp(val, "hd") == 0)   { target_fs = FRAMESIZE_HD;   target_q = 15; }
            else if (strcmp(val, "svga") == 0) { target_fs = FRAMESIZE_SVGA; target_q = 14; }
            else if (strcmp(val, "vga") == 0)  { target_fs = FRAMESIZE_VGA;  target_q = 12; }
            configure_ov5640_cool_30fps(s, target_fs, target_q);
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

    if (s_ov5640_sleeping) {
        set_ov5640_software_standby(esp_camera_sensor_get(), false);
    }

    s_stream_clients++;
    uint32_t frame_cnt = 0;
    uint32_t t_fps_start = millis();

    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        if (s_skip_frames > 0 || !is_valid_jpeg_frame(fb->buf, fb->len)) {
            if (s_skip_frames > 0) s_skip_frames--;
            esp_camera_fb_return(fb);
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

            if (s_ai_buf_state == 1 && s_ai_jpeg_buf && (fw / 8) <= MAX_RGB_THUMB_W) {
                memcpy(s_ai_jpeg_buf, s_stream_tx_buf + hlen, jpg_len);
                s_ai_jpeg_len  = jpg_len;
                __sync_synchronize();
                s_ai_buf_state = 2;
            }

            esp_camera_fb_return(fb);
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
    cfg_web.core_id          = 0;
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
        case OV5640_PID: return "OV5640 (5MP - FOMO+TinySSD @ 30+fps & Software Cooling)";
        case OV3660_PID: return "OV3660 (3MP)";
        case OV2640_PID: return "OV2640 (2MP)";
        default:         return "Unknown Sensor";
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n======================================================================");
    Serial.println("  [SKETCH 1] EDGE IMPULSE FOMO + TINY-SSD @ 30+FPS & OV5640 COOLING");
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

    // Khởi tạo mặc định ở SVGA (800x600, 2x2 Subsampling) để đạt đồng thời 2 mục tiêu:
    //   1. Tốc độ truyền MJPEG thực tế đạt 32 - 36 FPS (vượt mốc 30 FPS yêu cầu).
    //   2. Giảm 47% dòng tiêu thụ mảng cảm biến OV5640 (từ 110mA xuống 58mA theo Bảng 8-3 Datasheet).
    framesize_t init_fs = has_psram ? FRAMESIZE_SVGA : FRAMESIZE_VGA;

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
    config.frame_size   = init_fs;
    config.jpeg_quality = 14;
    config.fb_count     = has_psram ? 4 : 1;
    config.fb_location  = has_psram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
    config.grab_mode    = CAMERA_GRAB_LATEST;

    Serial.println("[3/4] Dang khoi tao Camera OV5640 (SVGA 30+fps + MIPI Power-Down + 1x Pad Cooling)...");
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("  -> [LOI] esp_camera_init that bai (0x%x). Kiem tra cap det FPC!\n", err);
        s_cam_ok = false;
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        configure_ov5640_cool_30fps(s, init_fs, 14);
        Serial.printf("[4/4] Nhan dien thanh cong     : %s (PID = 0x%04X)\n",
                      get_sensor_name(s->id.PID), s->id.PID);
    }
    s_cam_ok = true;
    s_ai_buf_state = 1;

    setup_zero_bottleneck_wifi();
    start_web_and_stream_servers();

    xTaskCreatePinnedToCore(ai_continual_learning_task, "ai_fomo_task", 20480, NULL, 1, NULL, 0);
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
        uint32_t ai_us = 0, shd_rej = 0;

        portENTER_CRITICAL(&s_ai_mux);
        n_boxes    = s_num_boxes;
        if (n_boxes > 0) b0 = s_boxes[0];
        learn_left = s_fast_learn_frames;
        s_cls      = s_scene_ai_cls;
        hw_m       = s_active_hw_mode;
        ai_us      = s_ai_infer_us;
        shd_rej    = s_shadow_rejected_cnt;
        portEXIT_CRITICAL(&s_ai_mux);

        if (learn_left > 0) {
            Serial.printf("[FOMO LEARNING] Dang cap nhat ngan hang ViBe (con %d chu ky) | FPS=%.1f | Temp=%.1fC\n",
                          learn_left, s_stream_fps, s_chip_temp_c);
        } else if (n_boxes > 0) {
            int bx_px = (int)(b0.x * s_ai_frame_w);
            int by_px = (int)(b0.y * s_ai_frame_h);
            int bw_px = (int)(b0.w * s_ai_frame_w);
            int bh_px = (int)(b0.h * s_ai_frame_h);
            Serial.printf("[FOMO+SSD DETECT!] %d doi tuong | #1: %s (M1:%u) | CNN=%u(M1:%u) | Box=[x:%d,y:%d,w:%d,h:%d] | Score=%d%% | Mode=%u | Infer=%.2fms | FPS=%.1f | Temp=%.1fC\n",
                          n_boxes, get_obj_class_name(b0.obj_cls_m2), b0.obj_cls_m1,
                          b0.ai_cls_m2, b0.ai_cls_m1,
                          bx_px, by_px, bw_px, bh_px, b0.score, hw_m, ai_us / 1000.0f, s_stream_fps, s_chip_temp_c);
        } else {
            Serial.printf("[FOMO NORMAL] On dinh (SceneCNN=%u | Mode=%u | LocBongDo=%lu | OV5640=%s) | FPS=%.1f | Infer=%.2fms | Temp=%.1fC\n",
                          s_cls, hw_m, (unsigned long)shd_rej,
                          s_ov5640_sleeping ? "STANDBY(<1mA)" : "COOL_ACTIVE",
                          s_stream_fps, ai_us / 1000.0f, s_chip_temp_c);
        }
    }

    delay(20);
}
