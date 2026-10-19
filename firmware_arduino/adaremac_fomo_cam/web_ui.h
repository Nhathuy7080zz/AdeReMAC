#pragma once
#include <Arduino.h>

struct RuntimeStats {
    volatile float stream_fps;
    volatile float cam_fps;
    volatile int   stream_clients;
    volatile bool  eco_enabled;
    volatile bool  idle_mode;      // OV5640 đang ở chế độ nghỉ xen kẽ
    volatile bool  hot;            // vượt ngưỡng nhiệt
    volatile bool  af_request;     // yêu cầu quét lấy nét tự động VCM AF trong cam_task
    volatile float chip_temp_c;
    volatile float ina_ma, ina_v;  // nếu có INA219
    volatile int   jpeg_quality;
};
extern RuntimeStats g_stats;

void web_start();
