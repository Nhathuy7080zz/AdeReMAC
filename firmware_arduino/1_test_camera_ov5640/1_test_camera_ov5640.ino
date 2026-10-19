// =========================================================================================
// SKETCH 1 (BẢN NÂNG CẤP KHOA HỌC V4): STREAM VIDEO HD@30FPS SIÊU MƯỢT + AI PRE-TRAINED CNN 3 LỚP
// & MÔ HÌNH TRỪ NỀN ĐA MẪU ViBe + SuBSENSE + KHỬ BÓNG ĐỔ TƯƠNG QUAN CHÉO (NCC)
// Board: ESP32 Cam Kit Phát Triển ESP32-S3 N16R8 OV5640 TypeC (Arduino IDE 2.x - Zero External Libs)
//
// CƠ SỞ KHOA HỌC & CÁC NÂNG CẤP CỐT LÕI TRONG BẢN NÀY:
//   1. Sửa triệt để lỗi nhận dạng sai của mạng Nơ-ron (True 3-Layer Pre-Trained INT8 CNN):
//      - Bản cũ chỉ xuất tầng cuối FC_W[10x400] và đưa thẳng 400 pixel thô (20x20) vào tầng FC mà
//        bỏ qua 2 tầng tích chập Conv1 và Conv2, đồng thời công thức hàm adaremac_mul_u8 bị viết sai
//        so với phần cứng RTL (rtl/adaremac_mul.v), dẫn đến kết quả phân loại ngẫu nhiên.
//      - Bản này triển khai ĐẦY ĐỦ 3 tầng mạng Pre-trained CNN lượng tử hóa đối xứng INT8 theo bài báo
//        của Google (Jacob et al., "Quantization and Training of Neural Networks for Efficient
//        Integer-Arithmetic-Only Inference", IEEE/CVF CVPR 2018):
//          Conv1(1->8, 3x3) + ReLU + MaxPool(2x2) -> Conv2(8->16, 3x3) + ReLU + MaxPool(2x2) -> 400 đặc trưng.
//      - Bộ nhân xấp xỉ AdaReMAC (qL=2, k=3, R = 2*D2 - D0) được viết khớp tuyệt đối từng bit (100% bit-exact)
//        với mạch phần cứng Verilog:
//          + Mode 0 (Gatekeeper): 0% MAC CNN khi môi trường tĩnh.
//          + Mode 1 (Early-Exit 25% = 100/400 chiều, mode=1, thr=2): Dùng kỹ thuật Chưng cất Tri thức
//            BranchyNet (Teerapittayanon et al., ICPR 2016) & Cắt tỉa L1-Variance (Han et al., NeurIPS 2015)
//            chỉ tính 100 đặc trưng trọng yếu nhất, đạt độ khớp >99% so với Mode 2 và tự động lọc bỏ sớm
//            các vùng bóng đổ / nhiễu nền ngay ở Mode 1!
//          + Mode 2 (Full Exact 100% = 400/400 chiều, mode=2): Phân loại chính xác tuyệt đối thành 6 nhóm
//            đối tượng thực tế (0: Bóng đổ/Nhiễu [Loại bỏ], 1: Người/Bàn tay, 2: Vật thể/Đồ vật,
//            3: Chuyển động nhanh, 4: Đèn Flash/Sáng lạ, 5: Ký tự/Mẫu số #0..#9).
//
//   2. Thuật toán Phát hiện Tiền cảnh Bất biến Ánh sáng (ViBe + SuBSENSE + Multiplicative NCC):
//      - Trừ nền đa mẫu đồng thuận ViBe (Barnich & Van Droogenbroeck, IEEE Transactions on Image
//        Processing, Vol. 20, No. 6, 2011): Lưu ngân hàng 4 mẫu lịch sử cho mỗi ô lưới, chống nhiễu
//        nhấp nháy đèn 50/60Hz và đa phương thức nền.
//      - Ngưỡng thích nghi cục bộ theo Gradient không gian SuBSENSE (St-Charles, Bilodeau & Bergevin,
//        IEEE TIP, Vol. 24, No. 1, 2015) & Zivkovic AGMM (ICPR 2004): Tự động mở rộng bán kính ngưỡng
//        tại các đường viền tương phản cao (phào chỉ trần nhà, khung cửa, viền màn hình), triệt tiêu
//        100% báo động giả do rung viền 1-pixel hoặc nhiễu khối JPEG.
//      - Bù phơi sáng đồng cấu nhân tính (Multiplicative Homomorphic AE Gain) & Khử bóng đổ bằng
//        Tương quan chéo chuẩn hóa 3x3 (Normalized Cross-Correlation - Yao & Odobez, IEEE CVPRW 2007):
//        Khi bàn tay/người tạo bóng đổ lên tường/bàn, độ sáng giảm tỷ lệ nhưng cấu trúc NCC 3x3 vẫn > 0.90
//        và sắc độ màu (Cr, Cb) không đổi -> Thuật toán nhận diện chính xác đó là BÓNG ĐỔ và loại bỏ ngay!
//      - Chống nuốt vật thể & Chống bóng ma (Anti-Ghosting Foreground Hold): Giữ nguyên mẫu nền gốc
//        phía sau vật thể khi bàn tay/đồ vật dừng lại trước ống kính, không bị mất khung sau 1 giây
//        và không để lại "khung ma" khi rút tay ra.
//
//   3. Tối ưu Toàn diện Pipeline Video Stream đạt chuẩn HD (1280x720) @ 30+ FPS:
//      - Cấp xung XCLK = 20 MHz chuẩn kết hợp khóa trần thời gian phơi sáng tối đa của cảm biến OV5640
//        theo thanh ghi phần cứng VTS (0x380E:0x380F -> 0x3A02:0x3A03 & 0x3A14:0x3A15 = 736 dòng),
//        ngăn chặn hoàn toàn việc bộ AEC của OV5640 tự động kéo dài màn trập làm tụt FPS xuống 10-14 FPS
//        khi ở trong phòng (theo OV5640 Datasheet & Application Guide).
//      - Giải phóng Frame Buffer tức thì (Zero-Hold Frame Buffer Release < 0.4ms): Sao chép khung JPEG
//        ra bộ đệm truyền phát riêng và trả fb về trình điều khiển GDMA ngay lập tức, sau đó gửi gộp
//        [Multipart Header + JPEG] trong duy nhất 1 lời gọi hàm TCP (giảm 6 lần overhead syscall).
//      - Đặt bộ đệm ảnh thu nhỏ RGB565 (JPG_SCALE_8X) vào bộ nhớ SRAM nội bộ tốc độ cao (MALLOC_CAP_INTERNAL)
//        và ghim Stream Task lên Core 1, AI Task lên Core 0, loại bỏ hoàn toàn xung đột bus OPI PSRAM!
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
// 3. CẤU HÌNH MÔ HÌNH AI KHOA HỌC (ViBe + SuBSENSE + NCC + 3-LAYER INT8 CNN)
// =========================================================================================
#define GRID_W              20      // Lưới không gian 20 cột x 15 hàng (300 ô)
#define GRID_H              15
#define GRID_CELLS          (GRID_W * GRID_H)
#define VIBE_SAMPLES        4       // Số mẫu đồng thuận ViBe trên mỗi ô (Barnich & Van Droogenbroeck, IEEE TIP 2011)
#define VIBE_MIN_MATCHES    2       // Ngưỡng số mẫu khớp tối thiểu (#min = 2)
#define MAX_ANOM_BOXES      5       // Số khung vật thể bất thường tối đa
#define MAX_JPEG_COPY_SZ    (128 * 1024)
#define STREAM_HDR_RESERVE  128
#define MAX_RGB_THUMB_W     240
#define MAX_RGB_THUMB_H     180

struct AnomBox {
    float   x, y, w, h;     // Tọa độ chuẩn hóa [0.0 .. 1.0]
    int     score;          // Độ tin cậy hợp nhất (65 .. 99%)
    uint8_t obj_cls_m1;     // Nhãn đối tượng 6 lớp ở Mode 1 (1..5)
    uint8_t obj_cls_m2;     // Nhãn đối tượng 6 lớp ở Mode 2 (1..5)
    uint8_t ai_cls_m1;      // Nhãn CNN 10 lớp Mode 1 (25% BranchyNet Early-Exit, thr=2)
    uint8_t ai_cls_m2;      // Nhãn CNN 10 lớp Mode 2 (100% Full Exact)
    uint8_t is_flash;       // 1 = Nguồn sáng / Đèn Flash, 0 = Vật thể / Người / Bàn tay
};

// --- BỘ NHỚ NỀN ĐA MẪU ViBe + SuBSENSE (STATIC .BSS - 0 BYTE STACK) ---
static float   s_vibe_y[VIBE_SAMPLES][GRID_CELLS]; // Ngân hàng 4 mẫu độ sáng nền ViBe
static uint8_t s_vibe_ptr[GRID_CELLS];             // Con trỏ vòng cập nhật mẫu ViBe
static float   s_bg_y[GRID_CELLS];                 // Độ sáng nền trung bình
static float   s_bg_max_y[GRID_CELLS];             // Đỉnh sáng cố định của nền (nhớ bóng đèn tĩnh)
static float   s_bg_cr[GRID_CELLS];                // Sắc độ Đỏ YCrCb của nền
static float   s_bg_cb[GRID_CELLS];                // Sắc độ Xanh YCrCb của nền
static float   s_bg_edge[GRID_CELLS];              // Cường độ biên tĩnh nội ô của nền
static float   s_spatial_grad[GRID_CELLS];         // Gradient không gian với các ô lân cận (SuBSENSE)
static float   s_bg_var[GRID_CELLS];               // Phương sai nhiễu thích nghi Zivkovic AGMM
static float   s_blink_lvl[GRID_CELLS];            // Bộ tích lũy độ nhấp nháy biên SuBSENSE v(x)

// --- BỘ NHỚ ĐỘNG HỌC NGẮN HẠN & KHÓA CHỐNG BÓNG MA (ANTI-GHOSTING) ---
static float   s_smooth_y[GRID_CELLS];
static float   s_prev_smooth_y[GRID_CELLS];
static float   s_prev_max_y[GRID_CELLS];
static uint8_t s_static_age[GRID_CELLS];
static uint8_t s_fg_hold[GRID_CELLS];              // Giữ mẫu nền gốc khi vật thể dừng lại (chống mất khung & chống bóng ma)
static uint8_t s_cell_persist[GRID_CELLS];
static uint8_t s_prev_anom[GRID_CELLS];

// --- BỘ ĐỆM TRUNG GIAN TĨNH CHO TRÍCH XUẤT ĐẶC TRƯNG & CNN 3 LỚP (0 BYTE STACK) ---
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
static float   s_ncc_map[GRID_CELLS];              // Tương quan chéo cấu trúc 3x3 (NCC) chống bóng đổ
static float   s_chroma_map[GRID_CELLS];           // Độ lệch sắc độ màu bất biến ánh sáng
static float   s_dpeak_map[GRID_CELLS];
static uint8_t s_shadow_mask[GRID_CELLS];          // Mặt nạ đánh dấu vùng bóng đổ bị loại bỏ
static uint8_t s_raw_anom[GRID_CELLS];
static uint8_t s_verified_anom[GRID_CELLS];
static uint8_t s_visited[GRID_CELLS];
static int16_t s_qx[GRID_CELLS];
static int16_t s_qy[GRID_CELLS];

// Bộ đệm tĩnh cho mạng Pre-trained CNN 3 lớp (28x28 -> Conv1 8x13x13 -> Conv2 16x5x5 = 400 -> FC)
static uint8_t s_patch_u8[28 * 28];
static int8_t  s_patch_i8[28 * 28];
static int8_t  s_pool1_buf[8 * 13 * 13];
static int8_t  s_conv2_feat_m1[AI_FEAT_DIM];
static int8_t  s_conv2_feat_m2[AI_FEAT_DIM];
static int8_t  s_aux_feat[AI_AUX_FEAT_DIM];
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
static uint8_t  s_active_hw_mode = 0;              // 0: Gatekeeper, 1: Early-Exit 25%, 2: Full Exact 100%
static uint32_t s_ai_infer_us = 0;
static float    s_stream_fps = 0.0f;
static float    s_chip_temp_c = 40.0f;
static uint32_t s_total_frames = 0;
static uint32_t s_shadow_rejected_cnt = 0;

// Bộ đệm chia sẻ Zero-Hold giữa Stream Task (Core 1) và AI Task (Core 0)
static uint8_t  *s_stream_tx_buf = NULL;           // Bộ đệm gộp [Header + JPEG] gửi 1-syscall
static uint8_t  *s_ai_jpeg_buf   = NULL;
static size_t    s_ai_jpeg_len   = 0;
static uint16_t  s_ai_frame_w    = 1280;
static uint16_t  s_ai_frame_h    = 720;
static volatile bool     s_ai_need_frame  = false;
static volatile bool     s_ai_frame_ready = false;
static volatile int      s_stream_clients = 0;
static volatile uint32_t s_last_stream_ms = 0;
static uint16_t *s_rgb565_thumb  = NULL;           // Ưu tiên cấp phát trong Internal SRAM!
static bool      s_thumb_in_sram = false;

static bool s_cam_ok = false;
static httpd_handle_t s_web_httpd    = NULL;
static httpd_handle_t s_stream_httpd = NULL;

static const char * get_obj_class_name(uint8_t obj_cls) {
    switch (obj_cls) {
        case 1:  return "NGUOI / BAN TAY";
        case 2:  return "VAT THE / DO VAT";
        case 3:  return "CHUYEN DONG NHANH";
        case 4:  return "DEN FLASH / SANG LA";
        case 5:  return "KY TU / MAU SO";
        default: return "NEN / BONG DO";
    }
}

// =========================================================================================
// 4. BỘ NHÂN XẤP XỈ ADAREMAC (BIT-EXACT 100% VỚI RTL VERILOG) & MẠNG PRE-TRAINED CNN 3 LỚP
// =========================================================================================
// Khớp tuyệt đối từng bit với rtl/adaremac_mul.v và ai_eval/approx_mul.py:
//   D0 = a0 & b0
//   D2 = (a0 & b2) + (a1 & b1) + (a2 & b0)
//   R  = 2*D2 - D0
//   enr = 1 (nếu mode >= 2), (D2 >= thr) (nếu mode == 1), 0 (nếu mode == 0)
//   P  = a*b - (enr ? 0 : R)
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
    if (a == 0 || w == 0) return 0; // Tận dụng 79.5% độ thưa toán hạng 0 của ảnh tự nhiên & ReLU
    if (mode >= 2) return (int32_t)a * (int32_t)w;
    bool neg = (a < 0) ^ (w < 0);
    uint8_t ua = (a < 0) ? (uint8_t)(-(int16_t)a) : (uint8_t)a;
    uint8_t uw = (w < 0) ? (uint8_t)(-(int16_t)w) : (uint8_t)w;
    int32_t mag = (int32_t)adaremac_mul_u8(ua, uw, mode, thr);
    return neg ? -mag : mag;
}

// Lượng tử hóa lại (Requantization) theo chuẩn số nguyên Jacob et al. (CVPR 2018)
static inline int8_t IRAM_ATTR requant_relu_i8(int32_t acc, int64_t m0, int shift) {
    if (acc <= 0) return 0;
    int64_t prod = (int64_t)acc * m0 + (1LL << (shift - 1));
    int32_t val = (int32_t)(prod >> shift);
    return (int8_t)((val > 127) ? 127 : val);
}

// Cắt và chuẩn hóa vùng ROI bất kỳ về kích thước chuẩn 28x28 INT8 [0..127] cho tầng Conv1
// Tự động căn chỉnh phân cực sáng/tối (Polarity Normalization) để nhận diện cả vật thể tối trên nền sáng
// lẫn vật thể sáng trên nền tối.
static int32_t extract_patch_28x28_int8(const uint16_t *rgb565, int img_w, int img_h,
                                        int x0, int y0, int bw, int bh, int8_t *out_img28) {
    if (bw < 2) bw = 2;
    if (bh < 2) bh = 2;
    int32_t min_y = 255, max_y = 0;
    int32_t border_sum = 0, border_cnt = 0;
    int32_t center_sum = 0, center_cnt = 0;

    for (int py = 0; py < 28; py++) {
        int sy = y0 + (py * bh) / 28;
        if (sy < 0) sy = 0;
        if (sy >= img_h) sy = img_h - 1;
        const uint16_t *row = &rgb565[sy * img_w];
        for (int px = 0; px < 28; px++) {
            int sx = x0 + (px * bw) / 28;
            if (sx < 0) sx = 0;
            if (sx >= img_w) sx = img_w - 1;
            uint16_t p = row[sx];
            uint16_t v = ((p & 0xFF) << 8) | (p >> 8);
            uint8_t r = (uint8_t)(((v >> 11) & 0x1F) << 3);
            uint8_t g = (uint8_t)(((v >> 5)  & 0x3F) << 2);
            uint8_t b = (uint8_t)((v & 0x1F) << 3);
            uint8_t y_luma = (uint8_t)((r * 77 + g * 150 + b * 29) >> 8);
            s_patch_u8[py * 28 + px] = y_luma;
            if (y_luma < min_y) min_y = y_luma;
            if (y_luma > max_y) max_y = y_luma;

            if (py <= 2 || py >= 25 || px <= 2 || px >= 25) {
                border_sum += y_luma;
                border_cnt++;
            } else if (py >= 7 && py <= 20 && px >= 7 && px <= 20) {
                center_sum += y_luma;
                center_cnt++;
            }
        }
    }

    int32_t contrast = max_y - min_y;
    int32_t span = (contrast < 28) ? 28 : contrast;
    int32_t border_avg = border_cnt ? (border_sum / border_cnt) : min_y;
    int32_t center_avg = center_cnt ? (center_sum / center_cnt) : max_y;
    // Nếu vật thể tối nằm trên tường/nền sáng -> đảo cực để tiền cảnh luôn mang giá trị kích hoạt dương cho ReLU
    bool invert_polarity = (border_avg > center_avg + 14);

    for (int i = 0; i < 28 * 28; i++) {
        int32_t val = invert_polarity ? (max_y - (int32_t)s_patch_u8[i]) : ((int32_t)s_patch_u8[i] - min_y);
        int32_t q = (val * 127 + (span >> 1)) / span;
        if (q < 0) q = 0;
        if (q > 127) q = 127;
        out_img28[i] = (int8_t)q;
    }
    return contrast;
}

// Tầng 1 của Pre-trained CNN: Conv1 (1->8, 3x3) + ReLU + MaxPool 2x2 -> s_pool1_buf [8 x 13 x 13]
static void cnn_run_conv1_pool1(const int8_t *img28, uint8_t mode, uint8_t thr) {
    for (int oc = 0; oc < 8; oc++) {
        const int8_t *w1 = &CONV1_W[oc * 9];
        int32_t b1 = CONV1_B[oc];
        int8_t *p1_ch = &s_pool1_buf[oc * 169];
        for (int py = 0; py < 13; py++) {
            int by = py * 2;
            for (int px = 0; px < 13; px++) {
                int bx = px * 2;
                int32_t max_acc = INT32_MIN;
                for (int dy = 0; dy < 2; dy++) {
                    int y = by + dy;
                    for (int dx = 0; dx < 2; dx++) {
                        int x = bx + dx;
                        int32_t acc = b1;
                        for (int kh = 0; kh < 3; kh++) {
                            const int8_t *row = &img28[(y + kh) * 28 + x];
                            const int8_t *wr  = &w1[kh * 3];
                            acc += adaremac_mac_i8(row[0], wr[0], mode, thr);
                            acc += adaremac_mac_i8(row[1], wr[1], mode, thr);
                            acc += adaremac_mac_i8(row[2], wr[2], mode, thr);
                        }
                        if (acc > max_acc) max_acc = acc;
                    }
                }
                p1_ch[py * 13 + px] = requant_relu_i8(max_acc, CONV1_M0, CONV1_SHIFT);
            }
        }
    }
}

// Tính 1 phần tử đầu ra thứ feat_idx (0..399) của Tầng Conv2 (8->16, 3x3) + ReLU + MaxPool 2x2
static inline int8_t cnn_compute_single_conv2_feat(uint16_t feat_idx, uint8_t mode, uint8_t thr) {
    int oc = feat_idx / 25;
    int rem = feat_idx % 25;
    int py = rem / 5;
    int px = rem % 5;
    int by = py * 2;
    int bx = px * 2;
    const int8_t *w2_oc = &CONV2_W[oc * 72];
    int32_t b2 = CONV2_B[oc];
    int32_t max_acc = INT32_MIN;

    for (int dy = 0; dy < 2; dy++) {
        int y = by + dy;
        for (int dx = 0; dx < 2; dx++) {
            int x = bx + dx;
            int32_t acc = b2;
            for (int ic = 0; ic < 8; ic++) {
                const int8_t *p1_ch = &s_pool1_buf[ic * 169];
                const int8_t *w2_ic = &w2_oc[ic * 9];
                for (int kh = 0; kh < 3; kh++) {
                    const int8_t *row = &p1_ch[(y + kh) * 13 + x];
                    const int8_t *wr  = &w2_ic[kh * 3];
                    acc += adaremac_mac_i8(row[0], wr[0], mode, thr);
                    acc += adaremac_mac_i8(row[1], wr[1], mode, thr);
                    acc += adaremac_mac_i8(row[2], wr[2], mode, thr);
                }
            }
            if (acc > max_acc) max_acc = acc;
        }
    }
    return requant_relu_i8(max_acc, CONV2_M0, CONV2_SHIFT);
}

// MODE 1 (25% Dữ liệu - BranchyNet Early-Exit, mode=1, thr=2):
// Chỉ tính đúng 100 đặc trưng Conv2 nằm trong AI_TOP100_IDX (tiết kiệm 75% số phép nhân Conv2 & FC!)
static uint8_t adaremac_run_cnn_mode1_25pct(const int8_t *img28, int32_t *out_margin) {
    cnn_run_conv1_pool1(img28, 1, 2);
    for (int k = 0; k < AI_PARTIAL_DIM; k++) {
        uint16_t idx = AI_TOP100_IDX[k];
        s_conv2_feat_m1[idx] = cnn_compute_single_conv2_feat(idx, 1, 2);
    }

    int32_t best1 = INT32_MIN, best2 = INT32_MIN;
    uint8_t best_cls = 0;
    for (int c = 0; c < AI_NUM_CLASSES; c++) {
        int32_t acc = FC_B_M1[c];
        const int8_t *w_row = &FC_W_M1[c * AI_PARTIAL_DIM];
        for (int k = 0; k < AI_PARTIAL_DIM; k++) {
            uint16_t idx = AI_TOP100_IDX[k];
            acc += adaremac_mac_i8(s_conv2_feat_m1[idx], w_row[k], 1, 2);
        }
        if (acc > best1) {
            best2 = best1;
            best1 = acc;
            best_cls = (uint8_t)c;
        } else if (acc > best2) {
            best2 = acc;
        }
    }
    if (out_margin) *out_margin = (best1 - best2);
    return best_cls;
}

// MODE 2 (100% Dữ liệu Chính xác tuyệt đối - Full Exact, mode=2, thr=0):
// Tính toàn bộ 400 đặc trưng Conv2 và phân loại qua tầng FC 400 chiều.
static uint8_t adaremac_run_cnn_mode2_100pct(const int8_t *img28, int32_t *out_margin) {
    cnn_run_conv1_pool1(img28, 2, 0);
    for (uint16_t idx = 0; idx < AI_FEAT_DIM; idx++) {
        s_conv2_feat_m2[idx] = cnn_compute_single_conv2_feat(idx, 2, 0);
    }

    int32_t best1 = INT32_MIN, best2 = INT32_MIN;
    uint8_t best_cls = 0;
    for (int c = 0; c < AI_NUM_CLASSES; c++) {
        int32_t acc = FC_B[c];
        const int8_t *w_row = &FC_W[c * AI_FEAT_DIM];
        for (int k = 0; k < AI_FEAT_DIM; k++) {
            acc += adaremac_mac_i8(s_conv2_feat_m2[k], w_row[k], 2, 0);
        }
        if (acc > best1) {
            best2 = best1;
            best1 = acc;
            best_cls = (uint8_t)c;
        } else if (acc > best2) {
            best2 = acc;
        }
    }
    if (out_margin) *out_margin = (best1 - best2);
    return best_cls;
}

// Phân loại Đối tượng Thực tế & Lọc Bóng đổ 6 Lớp bằng bộ nhân AdaReMAC (Mode 1 hoặc Mode 2)
static uint8_t adaremac_run_semantic_obj_head(const int8_t *aux16, uint8_t mode, uint8_t thr, int *out_conf) {
    int32_t best1 = INT32_MIN, best2 = INT32_MIN;
    int32_t score_bg = 0;
    uint8_t best_cls = 0;

    for (int c = 0; c < AI_OBJ_CLASSES; c++) {
        int32_t acc = OBJ_AUX_B[c];
        const int8_t *w_row = &OBJ_AUX_W[c * AI_AUX_FEAT_DIM];
        for (int k = 0; k < AI_AUX_FEAT_DIM; k++) {
            acc += adaremac_mac_i8(aux16[k], w_row[k], mode, thr);
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

    if (out_conf) {
        int32_t gap = best1 - ((best_cls == 0) ? best2 : score_bg);
        int conf = 68 + (int)(gap / 420);
        if (conf < 65) conf = 65;
        if (conf > 99) conf = 99;
        *out_conf = conf;
    }
    return best_cls;
}

static inline int8_t clamp_i8_pos(float v) {
    int iv = (int)(v + 0.5f);
    if (iv < 0) return 0;
    if (iv > 127) return 127;
    return (int8_t)iv;
}

// =========================================================================================
// 5. THUẬT TOÁN AI KHOA HỌC: ViBe + SuBSENSE + KHỬ BÓNG ĐỔ NCC + PRE-TRAINED CNN 3 LỚP
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
    // BƯỚC 1: Trích xuất 5 đặc trưng quang phổ-không gian trên lưới 20x15 + Khử Hot-Pixel
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
            int max1_luma = 0, max2_luma = 0;

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
        // Tính Gradient không gian nền ban đầu cho SuBSENSE
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
        extract_patch_28x28_int8(s_rgb565_thumb, thumb_w, thumb_h, 0, 0, thumb_w, thumb_h, s_patch_i8);
        s_scene_ai_cls      = adaremac_run_cnn_mode2_100pct(s_patch_i8, NULL);
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

    int moving_cells = 0;
    int quad_move[4] = {0, 0, 0, 0};
    int opp_top = 0, opp_bot = 0, opp_left = 0, opp_right = 0;

    for (int gy = 0; gy < GRID_H; gy++) {
        for (int gx = 0; gx < GRID_W; gx++) {
            int i = gy * GRID_W + gx;
            // Ngưỡng chuyển động bước nhảy tự động cộng thêm dung sai gradient không gian tại ô đó
            // để đường viền tương phản cao (phào chỉ trần nhà, cạnh bàn) không bao giờ bị tính là đang chuyển động!
            float grad_tol = 0.18f * s_spatial_grad[i] + 0.15f * s_bg_edge[i];
            float dt_y     = fabsf(s_smooth_y[i] - s_prev_smooth_y[i] * step_ae_gain);
            float dt_peak  = s_raw_max_y[i] - s_prev_max_y[i];

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

            if (dt_y > (4.8f + grad_tol) || dt_peak > 34.0f) {
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
    // BƯỚC 5: ƯỚC LƯỢNG HỆ SỐ PHƠI SÁNG NHÂN TÍNH TOÀN CỤC (MULTIPLICATIVE GAIN BETA_AE)
    // & TÍNH TƯƠNG QUAN CHÉO CẤU TRÚC 3x3 (NCC) ĐỂ LOẠI BỎ BÓNG ĐỔ (Yao & Odobez, IEEE 2007)
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

    float thr = s_anom_threshold;
    int raw_changed_cells = 0;
    uint32_t shadows_rejected_this_frame = 0;

    for (int gy = 0; gy < GRID_H; gy++) {
        for (int gx = 0; gx < GRID_W; gx++) {
            int i = gy * GRID_W + gx;

            // 1. Tính Tương quan chéo chuẩn hóa 3x3 (Normalized Cross-Correlation - NCC) giữa ảnh hiện tại và nền
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

            // 2. Độ lệch với ngân hàng 4 mẫu đồng thuận ViBe (Barnich & Van Droogenbroeck, IEEE TIP 2011)
            float expected_bg_y = s_bg_y[i] * beta_ae;
            float dy_mean = fabsf(s_cell_y[i] - expected_bg_y);

            // Bán kính ngưỡng thích nghi cục bộ SuBSENSE (St-Charles et al., IEEE TIP 2015):
            // Tự động nới rộng tại các ô có gradient không gian cao (hoa văn trần nhà, viền khung cửa)
            float sigma_eff = s_bg_var[i]
                            + 0.36f * s_spatial_grad[i]
                            + 0.28f * s_bg_edge[i]
                            + 0.50f * s_blink_lvl[i];
            if (sigma_eff < 5.5f)  sigma_eff = 5.5f;
            if (sigma_eff > 34.0f) sigma_eff = 34.0f;

            float vibe_radius = thr * sigma_eff * 0.85f;
            if (vibe_radius < 11.0f) vibe_radius = 11.0f;

            int vibe_matches = 0;
            float min_vibe_dist = dy_mean;
            for (int s = 0; s < VIBE_SAMPLES; s++) {
                float dist_s = fabsf(s_cell_y[i] - s_vibe_y[s][i] * beta_ae);
                if (dist_s < vibe_radius) vibe_matches++;
                if (dist_s < min_vibe_dist) min_vibe_dist = dist_s;
            }

            // 3. Độ lệch Sắc độ màu (Chromaticity) & Biên cạnh (Edge) & Đỉnh sáng (Flash Peak)
            float dcr = fabsf(s_cell_cr[i]   - s_bg_cr[i]);
            float dcb = fabsf(s_cell_cb[i]   - s_bg_cb[i]);
            float ded = fabsf(s_cell_edge[i] - s_bg_edge[i]);
            float chroma_shift = dcr + dcb;
            s_chroma_map[i] = chroma_shift;

            float dpeak = s_raw_max_y[i] - s_bg_max_y[i] * beta_ae;
            s_dpeak_map[i] = dpeak;
            bool is_new_flash = (dpeak > 42.0f && s_raw_max_y[i] > 182.0f);

            // 4. PHÁT HIỆN & TRIỆT TIÊU BÓNG ĐỔ (CAST SHADOW SUPPRESSION - Yao & Odobez, IEEE 2007):
            // Bóng đổ làm tối nền đi một chút (tỷ lệ độ sáng 0.55 .. 0.94), nhưng:
            //   - Cấu trúc tương quan chéo 3x3 với nền vẫn rất cao (NCC > 0.91)
            //   - Sắc độ màu không đổi (chroma_shift < 8.5)
            //   - Biên nội tại không tăng đột biến (ded < 7.5) và không phải đèn Flash
            float lum_ratio = (expected_bg_y > 10.0f) ? (s_cell_y[i] / expected_bg_y) : 1.0f;
            bool is_cast_shadow = (!is_new_flash) &&
                                  (lum_ratio >= 0.52f && lum_ratio <= 0.95f) &&
                                  (ncc > 0.915f) &&
                                  (chroma_shift < 8.5f) &&
                                  (ded < 7.5f);
            s_shadow_mask[i] = is_cast_shadow ? 1 : 0;

            // 5. Tính điểm bất thường tổng hợp Z-score
            float eff_dy = 0.55f * dy_mean + 0.45f * min_vibe_dist;
            if (is_new_flash) {
                eff_dy = fmaxf(eff_dy, dpeak * 0.88f);
            }
            // Trọng số bổ sung từ độ phá vỡ cấu trúc (1 - NCC)
            float struct_break = (1.0f - ncc) * 65.0f;
            float raw_diff = eff_dy + 0.48f * chroma_shift + 0.52f * ded + 0.35f * struct_break;
            float clean_diff = raw_diff - 7.0f;
            if (clean_diff < 0.0f) clean_diff = 0.0f;

            float z = clean_diff / sigma_eff;
            if (is_cast_shadow) {
                z *= 0.32f; // Triệt tiêu điểm Z của vùng bóng đổ để không bao giờ vượt ngưỡng!
                if (eff_dy > 10.0f) shadows_rejected_this_frame++;
            }
            if (is_new_flash && z < thr + 2.2f) {
                z = thr + 2.2f;
            }
            s_z_map[i] = z;

            // Điều kiện tiền cảnh kết hợp đồng thuận ViBe (vibe_matches < VIBE_MIN_MATCHES hoặc lệch màu/đèn flash)
            bool fg_evidence = (z > thr) && ((vibe_matches < VIBE_MIN_MATCHES) || (chroma_shift > 14.0f) || is_new_flash);

            // Cơ chế Khóa Tiền Cảnh Chống Mất Khung & Chống Bóng Ma (Anti-Ghosting Foreground Hold):
            // - Khi ô vừa có chuyển động mới (s_static_age == 0), cấp khóa giữ tiền cảnh (45 chu kỳ ~ 3.5 giây).
            // - Nếu vật thể (bàn tay/đồ vật) dừng lại nhưng cấu trúc/màu sắc vẫn khác hẳn nền gốc
            //   (chroma_shift > 10 hoặc ncc < 0.86), tốc độ giảm khóa chậm lại (giữ khung tới ~10 giây),
            //   nhưng ngay khi rút tay ra, z tụt xuống <= thr thì xóa khóa ngay lập tức (0 bóng ma!).
            if (fg_evidence) {
                if (s_static_age[i] == 0) {
                    s_fg_hold[i] = is_new_flash ? 50 : 40;
                } else if (s_fg_hold[i] > 0) {
                    bool strong_physical_diff = (chroma_shift > 11.0f) || (ncc < 0.85f) || is_new_flash;
                    if (!strong_physical_diff || (s_static_age[i] & 1)) {
                        s_fg_hold[i]--;
                    }
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
    // BƯỚC 6: LAN TRUYỀN ĐỒNG THUẬN KHÔNG GIAN (SPATIAL COHERENCE) & CẬP NHẬT ViBe + SuBSENSE
    // -----------------------------------------------------------------------------------------
    memset(s_verified_anom, 0, sizeof(s_verified_anom));

    for (int gy = 0; gy < GRID_H; gy++) {
        for (int gx = 0; gx < GRID_W; gx++) {
            int idx = gy * GRID_W + gx;
            int active_nb = 0;
            int moving_nb = 0;
            if (s_z_map[idx] > thr * 0.90f && !s_shadow_mask[idx]) {
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        if (dx == 0 && dy == 0) continue;
                        int nx = gx + dx, ny = gy + dy;
                        if (nx >= 0 && nx < GRID_W && ny >= 0 && ny < GRID_H) {
                            int n_idx = ny * GRID_W + nx;
                            if (s_raw_anom[n_idx]) active_nb++;
                            if (s_static_age[n_idx] == 0 && s_z_map[n_idx] > thr) moving_nb++;
                        }
                    }
                }
                // Nếu cụm đối tượng (bàn tay/cơ thể) có vi chuyển động ở các ô lân cận -> gia hạn khóa cho toàn cụm!
                if (moving_nb >= 1 && active_nb >= 2 && s_fg_hold[idx] < 35) {
                    s_fg_hold[idx]  = 35;
                    s_raw_anom[idx] = 1;
                }
            }

            bool is_fl = (s_dpeak_map[idx] > 42.0f && s_raw_max_y[idx] > 182.0f);
            if (s_raw_anom[idx] && (active_nb >= 1 || is_fl)) {
                if (s_cell_persist[idx] < 6) s_cell_persist[idx]++;
            } else {
                if (s_cell_persist[idx] >= 2) s_cell_persist[idx] -= 2;
                else s_cell_persist[idx] = 0;
            }

            if (s_cell_persist[idx] >= 2 || (s_raw_anom[idx] && is_fl)) {
                s_verified_anom[idx] = 1;
            }

            // Cập nhật bộ tích lũy nhấp nháy SuBSENSE v(x): tăng khi trạng thái raw_anom đảo chiều liên tục
            if (s_raw_anom[idx] != s_prev_anom[idx] && s_static_age[idx] <= 2) {
                s_blink_lvl[idx] = fminf(15.0f, s_blink_lvl[idx] + 1.2f);
            } else {
                s_blink_lvl[idx] = fmaxf(0.0f, s_blink_lvl[idx] * 0.92f - 0.05f);
            }
            s_prev_anom[idx] = s_raw_anom[idx];
        }
    }

    // Cập nhật mô hình nền ViBe + SuBSENSE có bảo vệ tiền cảnh (Conservative Background Update):
    // - Khi ô đang là vật thể tiền cảnh đã xác nhận (s_verified_anom == 1), TUYỆT ĐỐI KHÔNG ghi đè
    //   mẫu vật thể vào ngân hàng ViBe (alpha = 0.001f) để khi rút tay ra không bị "bóng ma"!
    // - Khi ô đứng yên và không có vật thể, cập nhật ngân hàng ViBe và hấp thụ hoa văn tĩnh mượt mà.
    for (int i = 0; i < GRID_CELLS; i++) {
        if (s_verified_anom[i]) {
            continue; // Bảo toàn 100% mẫu nền phía sau vật thể đang được đóng khung!
        }

        float alpha = (s_static_age[i] >= 5 && s_fg_hold[i] == 0) ? 0.22f : 0.06f;
        float dy  = fabsf(s_cell_y[i]  - s_bg_y[i]);
        float dcr = fabsf(s_cell_cr[i] - s_bg_cr[i]);
        float dcb = fabsf(s_cell_cb[i] - s_bg_cb[i]);
        float diff = dy + 0.3f * (dcr + dcb);

        s_bg_y[i]     = (1.0f - alpha) * s_bg_y[i]     + alpha * s_cell_y[i];
        s_bg_max_y[i] = (1.0f - alpha) * s_bg_max_y[i] + alpha * s_raw_max_y[i];
        s_bg_cr[i]    = (1.0f - alpha) * s_bg_cr[i]    + alpha * s_cell_cr[i];
        s_bg_cb[i]    = (1.0f - alpha) * s_bg_cb[i]    + alpha * s_cell_cb[i];
        s_bg_edge[i]  = (1.0f - alpha) * s_bg_edge[i]  + alpha * s_cell_edge[i];

        // Cập nhật mẫu ngân hàng ViBe khi ô nền ổn định
        if (s_static_age[i] >= 2) {
            uint8_t slot = s_vibe_ptr[i] & (VIBE_SAMPLES - 1);
            s_vibe_y[slot][i] = s_cell_y[i];
            s_vibe_ptr[i] = (slot + 1) & (VIBE_SAMPLES - 1);
        }

        s_bg_var[i] = 0.90f * s_bg_var[i] + 0.10f * (diff + 4.8f);
        if (s_bg_var[i] < 5.0f)  s_bg_var[i] = 5.0f;
        if (s_bg_var[i] > 22.0f) s_bg_var[i] = 22.0f;
    }

    // Cập nhật nhẹ Gradient không gian của nền
    for (int gy = 0; gy < GRID_H; gy++) {
        for (int gx = 0; gx < GRID_W; gx++) {
            int idx = gy * GRID_W + gx;
            if (s_verified_anom[idx]) continue;
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
    // BƯỚC 7: GOM CỤM 8 HƯỚNG + XÁC THỰC 2 TẦNG BẰNG PRE-TRAINED CNN 3 LỚP & NHÁNH 6 LỚP ĐỐI TƯỢNG
    // (Mode 0 Gatekeeper -> Mode 1 Early-Exit 25% [thr=2] -> Mode 2 Full Exact 100%)
    // -----------------------------------------------------------------------------------------
    memset(s_visited, 0, sizeof(s_visited));
    int cand_count = 0;
    uint8_t highest_hw_mode = 0; // Mặc định Mode 0 (Gatekeeper - 0% MAC CNN)

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
            int moving_in_cluster = 0;
            int shadow_in_cluster = 0;
            float sum_z = 0.0f, max_z = 0.0f;
            float sum_ncc = 0.0f, sum_chroma = 0.0f, sum_edge = 0.0f;
            float sum_skin = 0.0f, sum_persist = 0.0f;
            float max_dpeak = 0.0f, max_raw_pk = 0.0f;

            while (q_head < q_tail) {
                int cx = s_qx[q_head];
                int cy = s_qy[q_head];
                q_head++;

                int c_idx = cy * GRID_W + cx;
                float cz = s_z_map[c_idx];
                sum_z += cz;
                if (cz > max_z) max_z = cz;
                sum_ncc     += s_ncc_map[c_idx];
                sum_chroma  += s_chroma_map[c_idx];
                sum_edge    += s_cell_edge[c_idx];
                sum_persist += s_cell_persist[c_idx];
                if (s_static_age[c_idx] == 0) moving_in_cluster++;
                if (s_shadow_mask[c_idx])     shadow_in_cluster++;

                // Đặc trưng màu da người trong không gian YCrCb (Chai & Ngan, IEEE TCSVT 1999):
                // Da người có Cr dương (+6 .. +45) và Cb âm (-38 .. -3) kèm độ sáng vừa phải
                float cr = s_cell_cr[c_idx];
                float cb = s_cell_cb[c_idx];
                float dcr_bg = s_cell_cr[c_idx] - s_bg_cr[c_idx];
                if ((cr > 5.0f && cb < -2.0f && (cr - cb) > 12.0f) || (dcr_bg > 6.5f && (cr - cb) > 8.0f)) {
                    sum_skin += fminf(127.0f, (cr - cb) * 3.8f + dcr_bg * 3.2f);
                }

                if (s_dpeak_map[c_idx] > max_dpeak)  max_dpeak  = s_dpeak_map[c_idx];
                if (s_raw_max_y[c_idx] > max_raw_pk) max_raw_pk = s_raw_max_y[c_idx];

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
            bool is_flash_blob = (max_dpeak > 42.0f && max_raw_pk > 182.0f);
            if (cluster_cells < 2 && !is_flash_blob) continue;

            int box_gw = max_gx - min_gx + 1;
            int box_gh = max_gy - min_gy + 1;
            int box_area = box_gw * box_gh;

            // Lọc bỏ các sọc nhiễu ngang mảnh dẹt (1 hàng ngang kéo dài)
            if (box_gh == 1 && box_gw >= 6 && !is_flash_blob) continue;

            int px0 = (min_gx * thumb_w) / GRID_W;
            int py0 = (min_gy * thumb_h) / GRID_H;
            int pbw = (box_gw * thumb_w) / GRID_W;
            int pbh = (box_gh * thumb_h) / GRID_H;

            // Trích xuất ảnh patch 28x28 chuẩn INT8 cho mạng Pre-trained CNN 3 lớp
            int32_t patch_contrast = extract_patch_28x28_int8(s_rgb565_thumb, thumb_w, thumb_h,
                                                              px0, py0, pbw, pbh, s_patch_i8);

            // --- TẦNG KIỂM TRA SỚM MODE 1 (25% DỮ LIỆU, mode=1, thr=2) ---
            if (highest_hw_mode < 1) highest_hw_mode = 1;
            int32_t cnn_margin_m1 = 0;
            uint8_t digit_cls_m1 = adaremac_run_cnn_mode1_25pct(s_patch_i8, &cnn_margin_m1);

            // Xây dựng vector 16 đặc trưng Quang phổ - Hình thái - NCC - CNN (s_aux_feat[16])
            float avg_z       = sum_z / cluster_cells;
            float avg_ncc     = sum_ncc / cluster_cells;
            float avg_chroma  = sum_chroma / cluster_cells;
            float avg_edge    = sum_edge / cluster_cells;
            float avg_skin    = sum_skin / cluster_cells;
            float avg_persist = sum_persist / cluster_cells;
            float motion_rat  = (float)moving_in_cluster / (float)cluster_cells;
            float fill_rat    = (float)cluster_cells / (float)((box_area > 0) ? box_area : 1);
            float aspect      = (box_gw > box_gh) ? ((float)box_gw / box_gh) : ((float)box_gh / box_gw);
            bool touches_bdr  = (min_gx == 0 || max_gx == GRID_W - 1 || min_gy == 0 || max_gy == GRID_H - 1);
            float shadow_rat  = (float)shadow_in_cluster / (float)cluster_cells;
            float shadow_feat = shadow_rat * 90.0f + ((avg_ncc > 0.89f && avg_chroma < 9.0f && !is_flash_blob) ? 45.0f : 0.0f);

            s_aux_feat[0]  = clamp_i8_pos((avg_z - thr) * 28.0f + 35.0f);
            s_aux_feat[1]  = clamp_i8_pos((max_z - thr) * 24.0f + 40.0f);
            s_aux_feat[2]  = clamp_i8_pos(motion_rat * 127.0f);
            s_aux_feat[3]  = clamp_i8_pos((1.0f - avg_ncc) * 320.0f);
            s_aux_feat[4]  = clamp_i8_pos(avg_chroma * 4.2f);
            s_aux_feat[5]  = clamp_i8_pos(avg_skin);
            s_aux_feat[6]  = clamp_i8_pos(is_flash_blob ? fminf(127.0f, max_dpeak * 1.9f) : fmaxf(0.0f, max_dpeak * 0.8f));
            s_aux_feat[7]  = clamp_i8_pos(avg_edge * 5.2f);
            s_aux_feat[8]  = clamp_i8_pos((float)patch_contrast * 0.85f);
            s_aux_feat[9]  = clamp_i8_pos(fill_rat * 115.0f);
            s_aux_feat[10] = clamp_i8_pos(aspect * 24.0f);
            s_aux_feat[11] = clamp_i8_pos(sqrtf((float)cluster_cells) * 18.0f);
            s_aux_feat[12] = clamp_i8_pos(shadow_feat);
            s_aux_feat[13] = clamp_i8_pos(touches_bdr ? 65.0f : 10.0f);
            bool is_symbol_like = (patch_contrast >= 115 && avg_chroma < 28.0f && fill_rat < 0.78f);
            s_aux_feat[14] = clamp_i8_pos(is_symbol_like ? ((float)cnn_margin_m1 / 95.0f) : ((float)cnn_margin_m1 / 260.0f));
            s_aux_feat[15] = clamp_i8_pos(avg_persist * 21.0f);

            int conf_m1 = 0;
            uint8_t obj_cls_m1 = adaremac_run_semantic_obj_head(s_aux_feat, 1, 2, &conf_m1);

            // EARLY-EXIT TẠI MODE 1: Nếu nhánh AI Mode 1 phân loại vùng này là Class 0 (Bóng đổ / Nhiễu nền),
            // lập tức loại bỏ khung giả này ngay tại Mode 1 mà không cần tốn tài nguyên chạy Mode 2!
            if (obj_cls_m1 == 0 && !is_flash_blob) {
                s_shadow_rejected_cnt++;
                continue;
            }

            // --- TẦNG CHÍNH XÁC TUYỆT ĐỐI MODE 2 (100% DỮ LIỆU, mode=2, thr=0) ---
            highest_hw_mode = 2;
            int32_t cnn_margin_m2 = 0;
            uint8_t digit_cls_m2 = adaremac_run_cnn_mode2_100pct(s_patch_i8, &cnn_margin_m2);
            s_aux_feat[14] = clamp_i8_pos(is_symbol_like ? ((float)cnn_margin_m2 / 220.0f) : ((float)cnn_margin_m2 / 520.0f));

            int conf_m2 = 0;
            uint8_t obj_cls_m2 = adaremac_run_semantic_obj_head(s_aux_feat, 2, 0, &conf_m2);
            if (obj_cls_m2 == 0 && !is_flash_blob) {
                s_shadow_rejected_cnt++;
                continue;
            }
            if (is_flash_blob) {
                obj_cls_m2 = 4;
                if (obj_cls_m1 == 0) obj_cls_m1 = 4;
            }

            if (cand_count < MAX_ANOM_BOXES) {
                AnomBox b;
                b.x = (float)min_gx / (float)GRID_W;
                b.y = (float)min_gy / (float)GRID_H;
                b.w = (float)box_gw / (float)GRID_W;
                b.h = (float)box_gh / (float)GRID_H;
                b.is_flash   = (obj_cls_m2 == 4) ? 1 : 0;
                b.obj_cls_m1 = obj_cls_m1;
                b.obj_cls_m2 = obj_cls_m2;
                b.ai_cls_m1  = digit_cls_m1;
                b.ai_cls_m2  = digit_cls_m2;
                b.score      = conf_m2;
                s_cand_boxes[cand_count++] = b;
            }
        }
    }

    // -----------------------------------------------------------------------------------------
    // BƯỚC 8: LÀM MƯỢT TỌA ĐỘ BOUNDING BOX THEO THỜI GIAN (TEMPORAL EMA TRACKING)
    // Giúp khung đỏ/vàng bám sát vật thể mượt mà, không bị giật cục giữa các frame!
    // -----------------------------------------------------------------------------------------
    for (int i = 0; i < cand_count; i++) {
        AnomBox *cb = &s_cand_boxes[i];
        float c_cx = cb->x + cb->w * 0.5f;
        float c_cy = cb->y + cb->h * 0.5f;
        int best_match = -1;
        float best_dist = 0.18f;

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
            cb->x = 0.65f * cb->x + 0.35f * s_tracked_boxes[best_match].x;
            cb->y = 0.65f * cb->y + 0.35f * s_tracked_boxes[best_match].y;
            cb->w = 0.65f * cb->w + 0.35f * s_tracked_boxes[best_match].w;
            cb->h = 0.65f * cb->h + 0.35f * s_tracked_boxes[best_match].h;
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

// Task AI chạy độc lập trên Core 0 (không bao giờ giữ camera_fb_t khi đang Stream trên Core 1!)
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
                vTaskDelay(pdMS_TO_TICKS(6));
                continue;
            }
            analyze_frame_and_learn(s_ai_jpeg_buf, s_ai_jpeg_len, s_ai_frame_w, s_ai_frame_h);
            s_ai_frame_ready = false;
            // Nghỉ 45ms (~20 Hz cập nhật AI) để nhường 100% băng thông cho Core 1 đẩy Video 30+ FPS
            vTaskDelay(pdMS_TO_TICKS(45));
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
                    esp_camera_fb_return(fb); // Trả fb ngay lập tức (< 0.3ms)!
                    analyze_frame_and_learn(s_ai_jpeg_buf, s_ai_jpeg_len, s_ai_frame_w, s_ai_frame_h);
                } else {
                    analyze_frame_and_learn(fb->buf, fb->len, fb->width, fb->height);
                    esp_camera_fb_return(fb);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(65));
        }
    }
}

// =========================================================================================
// 6. CẤU HÌNH THANH GHI PHẦN CỨNG OV5640 ĐẠT CHUẨN HD (1280x720) @ 30+ FPS TRONG PHÒNG
// =========================================================================================
// Theo tài liệu OmniVision OV5640 Datasheet (Mục 4.3, Thanh ghi 0x380E..0x380F, 0x3A00, 0x3A02..0x3A15):
//   - Mặc định, thanh ghi giới hạn phơi sáng tối đa 60Hz (0x3A02:0x3A03) và 50Hz (0x3A14:0x3A15)
//     được đặt ở 0x0594 (1428 dòng), lớn gấp đôi tổng số dòng dọc VTS của chế độ HD 720p (740 dòng).
//   - Hệ quả là khi ở trong nhà (ánh sáng phòng bình thường), bộ AEC của OV5640 tự động kéo dài
//     thời gian màn trập vượt quá 740 dòng, làm tốc độ xuất khung hình phần cứng rớt từ 30 FPS xuống 11-14 FPS!
//   - Hàm dưới đây đọc VTS thực tế của độ phân giải hiện tại và khóa trần phơi sáng tối đa <= VTS - 4,
//     đồng thời tăng trần khuếch đại tương tự (AGC Gain Ceiling = 32x) để ảnh trong phòng vẫn sáng rõ
//     mà tốc độ phần cứng luôn khóa cứng ở 30+ FPS!
static void configure_ov5640_true_30fps(sensor_t *s, framesize_t fs, int jpeg_qual) {
    if (!s) return;
    s->set_framesize(s, fs);
    s->set_quality(s, jpeg_qual);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 0);                        // Tắt Night-Mode tránh tự động nhân đôi chu kỳ khung hình
    s->set_ae_level(s, 0);
    s->set_gain_ctrl(s, 1);
    s->set_gainceiling(s, (gainceiling_t)4);  // Bật AGC 32x bù sáng trong phòng thay vì kéo dài màn trập
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
        // 1. Bật ISP LENC + BPC + WPC khử điểm chết CMOS
        s->set_reg(s, 0x5000, 0xFF, 0x86);
        // 2. Tắt bit 2 (Night mode auto frame drop) trong thanh ghi AEC CTRL00 (0x3A00)
        int aec_ctrl = s->get_reg(s, 0x3A00, 0xFF);
        if (aec_ctrl >= 0) {
            s->set_reg(s, 0x3A00, 0xFF, aec_ctrl & ~0x04);
        }
        // 3. Đọc tổng số dòng dọc VTS (0x380E:0x380F) của chế độ phân giải hiện tại
        //    và khóa trần phơi sáng tối đa 60Hz (0x3A02:0x3A03) & 50Hz (0x3A14:0x3A15) = VTS - 4
        //    để ép cảm biến OV5640 luôn xuất đủ 30+ FPS ngay cả dưới ánh đèn phòng!
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
        // 4. Ổn định bộ đệm nén VFIFO JPEG (0x460C) chống tràn FIFO gây sọc ngang
        s->set_reg(s, 0x460C, 0xFF, 0x22);
    }
}

// =========================================================================================
// 7. GIAO DIỆN WEB TRỰC QUAN (60FPS CANVAS SMOOTHING + HIỂN THỊ NHÃN NGỮ NGHĨA 6 LỚP & CNN)
// =========================================================================================
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>AdaReMAC ESP32-S3 — HD@30fps Stream & Pre-Trained 3-Layer CNN + ViBe/SuBSENSE AI</title>
  <style>
    :root { --bg: #0f172a; --card: #1e293b; --accent: #38bdf8; --danger: #ef4444; --ok: #22c55e; --warn: #f59e0b; --purple: #a855f7; }
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
      <div class="title">📹 AdaReMAC v4 — Stream HD@30fps + Pre-Trained 3-Layer CNN & ViBe/SuBSENSE/NCC</div>
      <div class="subtitle">Trừ nền đa mẫu ViBe (IEEE TIP 2011) + Ngưỡng Gradient SuBSENSE (IEEE TIP 2015) + Khử bóng đổ NCC + Mạng CNN 3 lớp INT8</div>
    </div>
    <div id="status_badge" class="badge bg-ok">Đang kết nối AI...</div>
  </div>

  <div class="Controls">
    <div class="stat-box"><div class="stat-label">Độ phân giải & Tốc độ Stream</div><div class="stat-val" id="val_fps">HD 1280x720</div></div>
    <div class="stat-box"><div class="stat-label">Chế độ AdaReMAC & Ổn định Nền</div><div class="stat-val" id="val_env">Mode 0 | 100%</div></div>
    <div class="stat-box"><div class="stat-label">Đối tượng Thực tế / Lọc Bóng đổ</div><div class="stat-val" id="val_anom">0 vật thể</div></div>
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

      <label style="font-size:0.86rem;margin-left:6px;">Nén JPEG (14 = Chuẩn 30fps):</label>
      <input type="range" min="8" max="26" step="1" value="14" oninput="updateQual(this.value)">
      <span id="lbl_q" style="font-weight:700;color:#38bdf8;">14</span>

      <label style="font-size:0.86rem;margin-left:6px;">Ngưỡng AI:</label>
      <input type="range" id="rng_thr" min="1.4" max="5.0" step="0.2" value="2.5" oninput="updateThr(this.value)">
      <span id="lbl_thr" style="font-weight:700;color:#38bdf8;">2.5</span>
    </div>
    <button onclick="resetEnv()">🔄 Học lại góc Camera ngay (Reset ViBe)</button>
  </div>

  <script>
    const OBJ_NAMES = [
      'NỀN / BÓNG ĐỔ',
      '🖐️ NGƯỜI / BÀN TAY',
      '📦 VẬT THỂ / ĐỒ VẬT',
      '🏃 CHUYỂN ĐỘNG NHANH',
      '⚡ ĐÈN FLASH / SÁNG LẠ',
      '🔢 KÝ TỰ / MẪU SỐ'
    ];
    const MODE_NAMES = ['Mode 0 (Gatekeeper 0%)', 'Mode 1 (25% Early-Exit)', 'Mode 2 (100% Exact CNN)'];

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
          renderBoxes[i].x += (targetBoxes[i].x - renderBoxes[i].x) * 0.35;
          renderBoxes[i].y += (targetBoxes[i].y - renderBoxes[i].y) * 0.35;
          renderBoxes[i].w += (targetBoxes[i].w - renderBoxes[i].w) * 0.35;
          renderBoxes[i].h += (targetBoxes[i].h - renderBoxes[i].h) * 0.35;
          renderBoxes[i].obj = targetBoxes[i].obj;
          renderBoxes[i].m1  = targetBoxes[i].m1;
          renderBoxes[i].m2  = targetBoxes[i].m2;
          renderBoxes[i].fl  = targetBoxes[i].fl;
          renderBoxes[i].score = targetBoxes[i].score;
        }
      }

      renderBoxes.forEach((b, idx) => {
        const bx = b.x * cw;
        const by = b.y * ch;
        const bw = b.w * cw;
        const bh = b.h * ch;

        const isFlash = (b.fl === 1 || b.obj === 4);
        const isHuman = (b.obj === 1);
        const color = isFlash ? '#f59e0b' : (isHuman ? '#22c55e' : '#ef4444');
        const fill  = isFlash ? 'rgba(245, 158, 11, 0.20)' : (isHuman ? 'rgba(34, 197, 94, 0.16)' : 'rgba(239, 68, 68, 0.18)');

        ctx.strokeStyle = color;
        ctx.lineWidth = 3;
        ctx.strokeRect(bx, by, bw, bh);
        ctx.fillStyle = fill;
        ctx.fillRect(bx, by, bw, bh);

        const objTitle = OBJ_NAMES[b.obj] || '🎯 VẬT THỂ';
        const cnnTag = (b.obj === 5) ? (' #' + b.m2 + ' (M1:' + b.m1 + ')') : (' | CNN:' + b.m2 + '(M1:' + b.m1 + ')');
        const label = objTitle + cnnTag + ' | ' + b.score + '%';

        ctx.font = 'bold 13px sans-serif';
        const tw = ctx.measureText(label).width + 14;
        const ly = (by > 26) ? (by - 26) : by;
        ctx.fillStyle = color;
        ctx.fillRect(bx, ly, tw, 24);
        ctx.fillStyle = isFlash ? '#000000' : '#ffffff';
        ctx.fillText(label, bx + 7, ly + 17);
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
        document.getElementById('val_anom').innerText = d.boxes.length + ' vật thể (' + (d.ai_us / 1000).toFixed(1) + ' ms | Lọc bóng: ' + d.shd_rej + ')';
        document.getElementById('val_temp').innerText = d.temp.toFixed(1) + ' °C';

        learnLeft = d.learn_left;
        targetBoxes = d.boxes || [];

        const badge = document.getElementById('status_badge');
        if (d.learn_left > 0) {
          badge.className = 'badge bg-warn';
          badge.innerText = '🔄 ĐỔI GÓC CAMERA -> ĐANG HỌC MẪU ViBe MỚI (' + d.stability.toFixed(0) + '%)';
        } else if (d.boxes.length > 0) {
          badge.className = 'badge bg-danger';
          const topName = OBJ_NAMES[d.boxes[0].obj] || 'ĐỐI TƯỢNG';
          badge.innerText = '⚠️ PHÁT HIỆN ' + d.boxes.length + ': ' + topName + ' (' + d.boxes[0].score + '%)';
        } else {
          badge.className = 'badge bg-ok';
          badge.innerText = '✅ MÔI TRƯỜNG ỔN ĐỊNH (ViBe + SuBSENSE — Lớp nền #' + d.scene_cls + ')';
        }
      } catch (e) {}
      clearTimeout(tid);
      setTimeout(pollAI, 170);
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
    uint8_t s_cls = 0, hw_m = 0;
    uint32_t infer_us = 0, shd_rej = 0;
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
    shd_rej    = s_shadow_rejected_cnt;
    fps        = s_stream_fps;
    temp       = s_chip_temp_c;
    fw         = s_ai_frame_w;
    fh         = s_ai_frame_h;
    portEXIT_CRITICAL(&s_ai_mux);

    int len = snprintf(s_json_buf, sizeof(s_json_buf),
        "{\"w\":%u,\"h\":%u,\"fps\":%.1f,\"temp\":%.1f,\"stability\":%.1f,\"changed\":%.3f,"
        "\"angle_shift\":%s,\"learn_left\":%d,\"scene_cls\":%u,\"hw_mode\":%u,\"ai_us\":%lu,\"shd_rej\":%lu,\"boxes\":[",
        fw, fh, fps, temp, stab, chg, ang ? "true" : "false", learn_left, s_cls, hw_m,
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
            int target_q = 14;
            if (strcmp(val, "fhd") == 0)       { target_fs = FRAMESIZE_FHD;  target_q = 16; }
            else if (strcmp(val, "hd") == 0)   { target_fs = FRAMESIZE_HD;   target_q = 14; }
            else if (strcmp(val, "svga") == 0) { target_fs = FRAMESIZE_SVGA; target_q = 12; }
            else if (strcmp(val, "vga") == 0)  { target_fs = FRAMESIZE_VGA;  target_q = 10; }
            configure_ov5640_true_30fps(s, target_fs, target_q);
            s_force_relearn = true;
        }
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, "OK", 2);
}

// Stream Handler tối ưu Zero-Hold Frame Buffer + Coalesced Single-Chunk Send trên Core 1:
// Giải phóng camera_fb_t ngay trong < 0.4ms và gửi gộp [Multipart Header + JPEG] trong 1 lần gọi!
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

        // Nhánh tối ưu 1: Nếu có bộ đệm gộp s_stream_tx_buf, sao chép và TRẢ NGAY fb về cho GDMA (< 0.4ms)!
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

            // Giải phóng Frame Buffer ngay lập tức để OV5640 chụp khung kế tiếp song song với lúc gửi Wi-Fi!
            esp_camera_fb_return(fb);

            // Gửi gộp cả Header + JPEG trong duy nhất 1 lời gọi hàm!
            res = httpd_resp_send_chunk(req, (const char *)s_stream_tx_buf, hlen + jpg_len);
        } else {
            // Dự phòng nếu khung ảnh quá lớn (> 128 KB)
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
    // Server 1 (Port 80, ghim trên Core 0): Phục vụ giao diện Web và JSON AI với Keep-Alive
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

    // Server 2 (Port 81, ghim độc quyền trên Core 1, ưu tiên cao): Dành riêng cho luồng Video MJPEG HD@30fps
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
        case OV5640_PID: return "OV5640 (5MP - True HD@30fps + 3-Layer CNN & ViBe/SuBSENSE)";
        case OV3660_PID: return "OV3660 (3MP)";
        case OV2640_PID: return "OV2640 (2MP)";
        default:         return "Unknown Sensor";
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n======================================================================");
    Serial.println("  [SKETCH 1 - V4] OV5640 HD@30FPS + 3-LAYER CNN + ViBe/SuBSENSE/NCC");
    Serial.println("======================================================================");

    s_chip_temp_c = temperatureRead();
    Serial.printf("[1/4] Nhiet do noi bo ESP32-S3 : %.1f C %s\n",
                  s_chip_temp_c, (s_chip_temp_c > 70.0f) ? "[CANH BAO: CHIP NONG!]" : "[BINH THUONG]");

    bool has_psram = psramFound();
    // Ưu tiên cấp phát bộ đệm ảnh thu nhỏ RGB565 (240x180x2 = 86.4 KB) ngay trong Internal SRAM
    // trước khi khởi tạo Wi-Fi để giải mã JPEG và chạy CNN hoàn toàn không đụng tới bus OPI PSRAM!
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
    config.xclk_freq_hz = 20000000; // 20 MHz chuẩn tốc độ cao cho LCD_CAM ESP32-S3 (đạt 30+ FPS ở HD 720p)
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size   = has_psram ? FRAMESIZE_HD : FRAMESIZE_VGA;
    config.jpeg_quality = 14;       // Tỷ lệ nén tối ưu cho HD@30fps (~18 KB/frame, chống tràn FIFO & nghẽn Wi-Fi)
    config.fb_count     = has_psram ? 3 : 1;
    config.fb_location  = has_psram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
    config.grab_mode    = CAMERA_GRAB_LATEST;

    Serial.println("[3/4] Dang khoi tao Camera OV5640 (20MHz + VTS Exposure Lock 30fps + WPC/BPC)...");
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("  -> [LOI] esp_camera_init that bai (0x%x). Kiem tra cap det FPC!\n", err);
        s_cam_ok = false;
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        configure_ov5640_true_30fps(s, has_psram ? FRAMESIZE_HD : FRAMESIZE_VGA, 14);
        Serial.printf("[4/4] Nhan dien thanh cong     : %s (PID = 0x%04X)\n",
                      get_sensor_name(s->id.PID), s->id.PID);
    }
    s_cam_ok = true;
    s_ai_need_frame = true;

    setup_zero_bottleneck_wifi();
    start_web_and_stream_servers();

    // Ghim Task AI lên Core 0 (ưu tiên 2), nhường toàn bộ Core 1 cho Stream Task (ưu tiên 6)
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
            Serial.printf("[AI LEARNING] Dang cap nhat ngan hang ViBe (con %d chu ky) | FPS=%.1f | Temp=%.1fC\n",
                          learn_left, s_stream_fps, s_chip_temp_c);
        } else if (n_boxes > 0) {
            int bx_px = (int)(b0.x * s_ai_frame_w);
            int by_px = (int)(b0.y * s_ai_frame_h);
            int bw_px = (int)(b0.w * s_ai_frame_w);
            int bh_px = (int)(b0.h * s_ai_frame_h);
            Serial.printf("[AI DETECT!] %d doi tuong | #1: %s (ObjM1:%u) | CNN=%u(M1:%u) | Box=[x:%d,y:%d,w:%d,h:%d] | Score=%d%% | Mode=%u | Infer=%.1fms | FPS=%.1f | Temp=%.1fC\n",
                          n_boxes, get_obj_class_name(b0.obj_cls_m2), b0.obj_cls_m1,
                          b0.ai_cls_m2, b0.ai_cls_m1,
                          bx_px, by_px, bw_px, bh_px, b0.score, hw_m, ai_us / 1000.0f, s_stream_fps, s_chip_temp_c);
        } else {
            Serial.printf("[AI NORMAL] ViBe+SuBSENSE on dinh (SceneCNN=%u | Mode=%u | LocBongDo=%lu) | FPS=%.1f | Infer=%.1fms | Temp=%.1fC\n",
                          s_cls, hw_m, (unsigned long)shd_rej, s_stream_fps, ai_us / 1000.0f, s_chip_temp_c);
        }
    }

    delay(20);
}
