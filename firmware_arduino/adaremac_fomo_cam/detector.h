// Pipeline nhận dạng FOMO Lưới Siêu Mịn (Fine-Grid 48x48 / 24x24 / 12x12):
// JPEG -> ảnh xám 96x96 -> (FPGA | CPU | Edge Impulse) -> Giải mã lưới mịn bám sát biên -> Bám vết
#pragma once
#include <Arduino.h>
#include "fomo_micro.h"

enum Backend : uint8_t { BK_AUTO = 0, BK_CPU = 1, BK_FPGA = 2, BK_EI = 3 };

#define DET_MAX 8

struct DetObj {
    float cx, cy;          // tâm đối tượng, chuẩn hóa 0..1 theo khung hình
    float x0, y0;          // góc trên-trái của khung bao sát biên, chuẩn hóa 0..1
    float w, h;            // chiều rộng & chiều cao khung bao sát biên, chuẩn hóa 0..1
    float score;           // độ tin cậy (0..1)
    uint16_t id;           // mã bám vết
    uint16_t cells;        // số ô lưới mịn thuộc đối tượng
    uint8_t age;           // số khung liên tiếp được thấy
};

struct DetResult {
    uint32_t frame_seq;
    uint32_t t_ms;
    uint8_t  n;
    DetObj   obj[DET_MAX];
    Backend  backend_used;
    float    decode_ms, infer_ms, ai_fps;
    uint32_t fpga_cycles;
    uint8_t  fine_grid_res;                // 12, 24 hoặc 48
    int8_t   logits[144];                  // bản đồ nhiệt thô 12x12 từ CNN/FPGA
    uint8_t  fgrid[FOMO_VIS_GRID * FOMO_VIS_GRID]; // bản đồ lưới mịn 24x24 (576 ô, giá trị 0..100%)
};

void det_init();
void det_start();                          // tạo task giải mã (core 1) + task suy luận (core 0)
const char *det_cpu_engine();              // "esp-nn" hoặc "C"
const char *det_decoder_name();            // "esp_new_jpeg" / "tjpgd"
void det_get(DetResult *out);              // bản sao kết quả mới nhất
void det_set_backend(Backend b);
void det_set_threshold(int thr_logit);
int  det_get_threshold();
void det_set_fine_grid(int res);           // 12, 24 hoặc 48 (mặc định 48 -> 2.304 ô lưới)
int  det_get_fine_grid();
Backend det_get_backend();
void det_set_fpga_mode(uint8_t m);         // 0/1/2 cho cả 6 lớp
uint8_t det_get_fpga_mode();
uint32_t det_last_person_ms();
