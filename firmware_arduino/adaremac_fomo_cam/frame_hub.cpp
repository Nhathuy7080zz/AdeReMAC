#include "frame_hub.h"
#include "esp_heap_caps.h"

struct Slot {
    uint8_t *buf;
    size_t len;
    uint16_t w, h;
    uint32_t seq;
    volatile int readers;
};

static Slot s_slot[HUB_SLOTS];
static size_t s_cap = 0;
static volatile int s_latest = -1;
static volatile uint32_t s_seq = 0;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

bool hub_init(size_t slot_bytes) {
    s_cap = slot_bytes;
    for (int i = 0; i < HUB_SLOTS; i++) {
        s_slot[i].buf = (uint8_t *)heap_caps_malloc(slot_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_slot[i].buf) return false;
        s_slot[i].readers = 0;
        s_slot[i].len = 0;
    }
    return true;
}

bool hub_publish(const uint8_t *jpeg, size_t len, uint16_t w, uint16_t h) {
    if (len > s_cap) return false;
    int target = -1;
    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < HUB_SLOTS; i++) {
        if (i != s_latest && s_slot[i].readers == 0) { target = i; s_slot[i].readers = -1; break; }  // -1 = đang ghi
    }
    portEXIT_CRITICAL(&s_mux);
    if (target < 0) return false;            // mọi ô đều đang được đọc: bỏ khung này
    memcpy(s_slot[target].buf, jpeg, len);
    portENTER_CRITICAL(&s_mux);
    s_slot[target].len = len;
    s_slot[target].w = w;
    s_slot[target].h = h;
    s_seq = s_seq + 1;
    s_slot[target].seq = s_seq;
    s_slot[target].readers = 0;
    s_latest = target;
    portEXIT_CRITICAL(&s_mux);
    return true;
}

bool hub_acquire_latest(HubFrame *f, uint32_t newer_than) {
    bool ok = false;
    portENTER_CRITICAL(&s_mux);
    int i = s_latest;
    if (i >= 0 && s_slot[i].seq > newer_than && s_slot[i].readers >= 0) {
        s_slot[i].readers = s_slot[i].readers + 1;
        f->data = s_slot[i].buf; f->len = s_slot[i].len; f->w = s_slot[i].w; f->h = s_slot[i].h;
        f->seq = s_slot[i].seq; f->slot = i;
        ok = true;
    }
    portEXIT_CRITICAL(&s_mux);
    return ok;
}

void hub_release(const HubFrame *f) {
    portENTER_CRITICAL(&s_mux);
    if (s_slot[f->slot].readers > 0) s_slot[f->slot].readers = s_slot[f->slot].readers - 1;
    portEXIT_CRITICAL(&s_mux);
}

uint32_t hub_seq() { return s_seq; }
