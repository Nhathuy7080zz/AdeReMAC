// Khởi tạo OV5640 + Tối ưu độ nét quang học ISP / VCM Focus + Quản lý nhiệt phần mềm
#pragma once
#include <Arduino.h>
#include "esp_camera.h"

struct CamStatus {
    bool     ok;
    bool     standby;
    uint16_t pid;
    int      reg_300e;      // MIPI control (mặc định DVP: 0x58 = PHY MIPI đã tắt)
    int      reg_302c;      // pad drive (0x42 = 2x chuẩn, 0x02 = 1x tiết kiệm)
    int      reg_3036;      // PLL multiplier
    int      reg_3035;      // PLL sys div
    int      reg_3037;      // PLL pre div
    uint32_t vco_mhz;       // ước tính đầu ra PLL1
    framesize_t framesize;
    int      sharpness;     // mức làm nét biên ISP (0..6)
    int      vcm_step;      // bước lấy nét cuộn dây VCM (0..255)
};

bool cam_init(framesize_t fs, int jpeg_quality);
bool cam_set_framesize(framesize_t fs);                // đổi độ phân giải rồi áp lại cấu hình ISP + PLL
void cam_apply_cool_profile();                          // mở khóa khung DVP + pad drive + PLL trong dải
void cam_apply_isp_clarity(int sharpness, int contrast);// bật LENC, RAW_GMA, DCW, BPC/WPC và làm sắc nét cạnh CIP
void cam_set_sharpness(int sharpness);                  // chỉnh độ sắc nét cạnh ISP (0..6)
int  cam_get_sharpness();
void cam_set_vcm_focus(int step_0_255);                 // điều khiển DAC cuộn dây lấy nét VCM (0..255)
int  cam_get_vcm_focus();
int  cam_auto_focusSweep();                             // quét tự động tìm điểm lấy nét VCM có độ chi tiết JPEG cao nhất
void cam_standby(bool enable);                          // 0x3008 = 0x42 / 0x02
CamStatus cam_status();
