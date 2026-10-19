// Kho khung JPEG dùng chung giữa task camera (ghi), luồng MJPEG và task AI (đọc) — không khóa lâu.
// Task camera trả framebuffer cho driver ngay sau khi chép (~0,3 ms), nên luồng video không chờ AI.
#pragma once
#include <Arduino.h>

#define HUB_SLOTS 4

struct HubFrame {
    const uint8_t *data;
    size_t len;
    uint16_t w, h;
    uint32_t seq;
    int slot;
};

bool hub_init(size_t slot_bytes);
bool hub_publish(const uint8_t *jpeg, size_t len, uint16_t w, uint16_t h);   // gọi từ task camera
bool hub_acquire_latest(HubFrame *f, uint32_t newer_than);                   // false nếu chưa có khung mới hơn
void hub_release(const HubFrame *f);
uint32_t hub_seq();
