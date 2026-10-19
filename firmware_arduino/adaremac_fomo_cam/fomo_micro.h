// FOMO-Micro INT8 — suy luận C thuần, khớp từng bit với ai_fomo/fomo_int8.py và RTL fomo_accel_core.v
// Phiên bản v4.3: Bổ sung giải mã Lưới Siêu Mịn Đa Tỷ Lệ (Fine-Grid 24x24 = 576 ô & 48x48 = 2.304 ô,
// gấp 4x - 16x mật độ ô lưới trên cùng diện tích khung hình 96x96) giúp đóng khung sát biên vật thể.
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FOMO_MAX_OBJECTS 16
#ifndef FOMO_GRID
#define FOMO_GRID 12
#endif
#define FOMO_FINE_MAX    48   // Lưới siêu mịn tối đa: 48x48 = 2.304 ô (mỗi ô = 2x2 pixel trên ảnh 96x96)
#define FOMO_VIS_GRID    24   // Lưới truyền lên Web UI để vẽ ô lưới mịn: 24x24 = 576 ô

typedef struct {
    float cx, cy;         // tâm đối tượng, đơn vị ô lưới chuẩn (0..12)
    float x0, y0, x1, y1; // khung bao chuẩn hóa theo đơn vị ô lưới (0..12, có độ phân giải phân số 1/48)
    float score;          // xác suất lớn nhất trong cụm (0..1)
    uint16_t cells;       // số ô lưới mịn thuộc cụm vật thể
} fomo_obj_t;

// img: ảnh xám 96x96 (hàng trước). logits: 144 byte int8 (lưới 12x12) đầu ra lớp cuối.
void fomo_micro_run(const uint8_t *img, int8_t *logits);

// Bản nhanh dùng esp-nn (SIMD ESP32-S3), xem fomo_espnn.c. Lệch bản chuẩn tối đa 1 LSB ở vài kích hoạt.
int  fomo_fast_init(void);
int  fomo_fast_available(void);
void fomo_micro_run_fast(const uint8_t *img, int8_t *logits);

// Hậu xử lý FOMO gốc trên lưới 12x12 (giữ nguyên bit-exact cho testbench C/RTL):
int fomo_decode(const int8_t *logits, int thr_logit, fomo_obj_t *out, int max_out);

// Hậu xử lý FOMO Lưới Siêu Mịn (Fine-Grid 12x12 / 24x24 / 48x48 trên cùng diện tích 96x96):
// Kết hợp nội suy song tuyến bản đồ nhiệt CNN 12x12 + Lan truyền vùng song phương (Bilateral Silhouette
// Region Growing) + Bám biên Gradient Sobel trên lưới mịn GxG (G = 12, 24 hoặc 48) giúp đóng khung
// sát biên thực tế của vật thể với độ phân giải không gian tới 1/48 khung hình (2,08%).
// vis_24x24 (tùy chọn, 576 byte): xuất mức kích hoạt (0..100%) của 576 ô lưới mịn 24x24 để hiển thị Web UI.
int fomo_decode_fine(const uint8_t *img96, const int8_t *logits12, int thr_logit,
                     int fine_grid_res, fomo_obj_t *out, int max_out, uint8_t *vis_24x24);

// Ngưỡng logit mặc định và hệ số logit -> xác suất.
int fomo_default_thr_logit(void);
float fomo_logit_scale(void);

#ifdef __cplusplus
}
#endif
