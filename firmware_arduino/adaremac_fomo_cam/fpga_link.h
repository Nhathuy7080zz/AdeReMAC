// Giao tiếp SPI với bộ tăng tốc FOMO-Micro trên Tang Nano 20K (rtl/fomo_top_tn20k.v)
#pragma once
#include <Arduino.h>

struct FpgaResult {
    uint8_t status;      // bit0 kết quả mới, bit1 đang bận, bit2 lỗi ghi khi bận
    uint8_t frame_cnt;
    uint32_t cycles;     // số chu kỳ 27 MHz của lần suy luận gần nhất
    int8_t logits[144];
};

bool fpga_begin();                               // true nếu đọc được ID 0xF0
bool fpga_present();
void fpga_send_frame(const uint8_t *gray96);     // 9216 byte; FPGA tự chạy khi nhận đủ
bool fpga_read_result(FpgaResult *r);
bool fpga_wait_result(FpgaResult *r, uint8_t after_cnt, uint32_t timeout_ms);
void fpga_write_reg(uint8_t addr, uint8_t val);
uint8_t fpga_read_reg(uint8_t addr);
void fpga_set_modes(uint8_t mode_all);           // 0 xấp xỉ, 1 chọn lọc, 2 chính xác cho cả 6 lớp
