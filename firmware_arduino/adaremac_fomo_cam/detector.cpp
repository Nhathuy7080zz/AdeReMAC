#include "detector.h"
#include "app_config.h"
#include "frame_hub.h"
#include "fpga_link.h"
#include "img_converters.h"
#include "esp_heap_caps.h"
#include "fomo_micro.h"
#include "esp_jpeg_dec.h"
#include "freertos/queue.h"
#if defined(USE_EDGE_IMPULSE)
#include EI_HEADER
#endif

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static DetResult s_res;
static volatile Backend s_backend = BK_AUTO;
static volatile int s_thr = 0;            // đặt trong det_init()
static volatile int s_fine_grid = FOMO_DEFAULT_FINE_GRID; // 48 (2.304 ô), 24 (576 ô) hoặc 12 (144 ô)
static volatile uint8_t s_fpga_mode = 2;
static volatile bool s_fpga_mode_dirty = false;
static volatile uint32_t s_last_person_ms = 0;

static uint8_t *s_rgb = nullptr;          // RGB565 sau giải mã thu nhỏ (tối đa 320x240) — bộ giải mã dự phòng
static uint8_t *s_yuv = nullptr;          // CbYCrY sau giải mã thu nhỏ — bộ giải mã chính (esp_new_jpeg)
#define YUV_MAX_W 320
#define YUV_MAX_H 240
static int8_t s_byte_order = -1;          // -1 chưa biết, 0 = little-endian, 1 = big-endian

// Đường ống 2 lõi: task "prep" (core 1) giải mã JPEG -> ảnh xám 96x96; task "fomo" (core 0) suy luận.
// 3 bộ đệm luân chuyển qua 2 hàng đợi: free_q (bộ đệm trống) và ready_q (1 ảnh mới nhất chờ suy luận).
#define GRAY_BUFS 3
static uint8_t *s_gray[GRAY_BUFS];        // trong PSRAM — SRAM trong chip để dành cho Wi-Fi/web
static uint8_t *s_pend_gray = nullptr;    // lưu ảnh xám của khung đang chờ FPGA xử lý để lọc lưới mịn 48x48
static struct { uint32_t seq; float dec_ms; } s_meta[GRAY_BUFS];
static QueueHandle_t s_free_q = nullptr, s_ready_q = nullptr;
static volatile uint8_t s_decoder = 0;    // 0 chưa rõ, 1 esp_new_jpeg, 2 tjpgd (ROM, chậm)
static bool s_cpu_fast = false;

void det_set_backend(Backend b) { s_backend = b; }
Backend det_get_backend() { return s_backend; }
void det_set_threshold(int t) { s_thr = t; }
int det_get_threshold() { return s_thr; }
void det_set_fine_grid(int res) {
    if (res != 12 && res != 24 && res != 48) res = 48;
    s_fine_grid = res;
}
int det_get_fine_grid() { return s_fine_grid; }
void det_set_fpga_mode(uint8_t m) { s_fpga_mode = m > 2 ? 2 : m; s_fpga_mode_dirty = true; }
uint8_t det_get_fpga_mode() { return s_fpga_mode; }
uint32_t det_last_person_ms() { return s_last_person_ms; }

void det_get(DetResult *out) {
    portENTER_CRITICAL(&s_mux);
    *out = s_res;
    portEXIT_CRITICAL(&s_mux);
}

// ------------------------------------------------------------------ tiền xử lý
static inline uint8_t px_gray(const uint8_t *p, int be) {
    uint16_t v = be ? (uint16_t)((p[0] << 8) | p[1]) : (uint16_t)((p[1] << 8) | p[0]);
    int r = (v >> 11) << 3, g = ((v >> 5) & 0x3F) << 2, b = (v & 0x1F) << 3;
    return (uint8_t)((r * 77 + g * 150 + b * 29) >> 8);
}

static void detect_byte_order(const uint8_t *rgb, int w, int h) {
    uint64_t e[2] = {0, 0};
    for (int be = 0; be < 2; be++)
        for (int y = 0; y < h; y += 4)
            for (int x = 1; x < w; x++)
                e[be] += abs((int)px_gray(rgb + 2 * (y * w + x), be) - (int)px_gray(rgb + 2 * (y * w + x - 1), be));
    if (e[0] + e[1] > (uint64_t)w * h / 4 && (e[0] > e[1] * 13 / 10 || e[1] > e[0] * 13 / 10))
        s_byte_order = e[1] < e[0] ? 1 : 0;
}

static void to_gray96(const uint8_t *rgb, int w, int h, uint8_t *dst) {
    const int be = s_byte_order < 0 ? 1 : s_byte_order;
    for (int y = 0; y < 96; y++) {
        float fy = (y + 0.5f) * h / 96.0f - 0.5f;
        int y0 = fy < 0 ? 0 : (int)fy; int y1 = y0 + 1 < h ? y0 + 1 : y0; float ay = fy - y0; if (ay < 0) ay = 0;
        for (int x = 0; x < 96; x++) {
            float fx = (x + 0.5f) * w / 96.0f - 0.5f;
            int x0 = fx < 0 ? 0 : (int)fx; int x1 = x0 + 1 < w ? x0 + 1 : x0; float ax = fx - x0; if (ax < 0) ax = 0;
            float a = px_gray(rgb + 2 * (y0 * w + x0), be), b = px_gray(rgb + 2 * (y0 * w + x1), be);
            float c = px_gray(rgb + 2 * (y1 * w + x0), be), d = px_gray(rgb + 2 * (y1 * w + x1), be);
            float v = (a * (1 - ax) + b * ax) * (1 - ay) + (c * (1 - ax) + d * ax) * ay;
            dst[y * 96 + x] = (uint8_t)(v + 0.5f);
        }
    }
}

// Bộ giải mã dự phòng: tjpgd trong ROM qua esp32-camera
static bool decode_tjpgd(const HubFrame &f, uint8_t *dst) {
    if (!s_rgb) s_rgb = (uint8_t *)heap_caps_malloc(YUV_MAX_W * YUV_MAX_H * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_rgb) return false;
    int sc = 8;
    while (sc > 1 && (f.w / sc < 96 || f.h / sc < 72)) sc >>= 1;
    int w = f.w / sc, h = f.h / sc;
    if ((size_t)w * h * 2 > (size_t)YUV_MAX_W * YUV_MAX_H * 2) return false;
    bool ok = jpg2rgb565(f.data, f.len, s_rgb, sc == 8 ? JPG_SCALE_8X : sc == 4 ? JPG_SCALE_4X : sc == 2 ? JPG_SCALE_2X : JPG_SCALE_NONE);
    if (!ok) return false;
    if (s_byte_order < 0) detect_byte_order(s_rgb, w, h);
    to_gray96(s_rgb, w, h, dst);
    return true;
}

// Chọn tỷ lệ thu nhỏ IDCT hợp lệ (1/8, 1/4, 1/2 hoặc 1/1) cho esp_new_jpeg:
// Ở v4.2, hàm scaled_dim(800, 160) trả về 160x120 (tỷ lệ 1/5 đối với SVGA 800x600),
// làm esp_new_jpeg báo lỗi vì phần cứng giải mã JPEG chỉ hỗ trợ chia 1, 2, 4, 8 và kích thước đầu ra chẵn!
static void choose_idct_scale(int fw, int fh, int *tw, int *th) {
    static const int divs[4] = {8, 4, 2, 1};
    for (int i = 0; i < 4; i++) {
        int d = divs[i];
        int w = fw / d, h = fh / d;
        // esp_new_jpeg yêu cầu kích thước sau chia khớp chính xác (fw % d == 0, fh % d == 0),
        // chiều rộng chia hết cho 8 (hoặc >= 96) và nằm trong bộ đệm YUV_MAX_W x YUV_MAX_H
        if ((fw % d == 0) && (fh % d == 0) && (w % 8 == 0) && (h % 2 == 0) &&
            w >= 96 && h >= 72 && w <= YUV_MAX_W && h <= YUV_MAX_H) {
            *tw = w;
            *th = h;
            return;
        }
    }
    // Dự phòng cho SVGA 800x600: chia 4 -> 200x150 (200 chia hết cho 8, 150 chia hết cho 2)
    *tw = fw / 4;
    *th = fh / 4;
}

// Lấy kênh Y (độ sáng chuẩn BT.601) từ CbYCrY và co về 96x96 bằng nội suy song tuyến.
static void y_to_gray96(const uint8_t *yuv, int w, int h, uint8_t *dst) {
    for (int y = 0; y < 96; y++) {
        int fy = ((2 * y + 1) * h * 128) / 96 - 128;
        if (fy < 0) fy = 0;
        int y0 = fy >> 8, ay = fy & 255, y1 = y0 + 1 < h ? y0 + 1 : y0;
        const uint8_t *r0 = yuv + 2 * y0 * w + 1, *r1 = yuv + 2 * y1 * w + 1;
        for (int x = 0; x < 96; x++) {
            int fx = ((2 * x + 1) * w * 128) / 96 - 128;
            if (fx < 0) fx = 0;
            int x0 = fx >> 8, ax = fx & 255, x1 = x0 + 1 < w ? x0 + 1 : x0;
            int top = r0[2 * x0] * (256 - ax) + r0[2 * x1] * ax;
            int bot = r1[2 * x0] * (256 - ax) + r1[2 * x1] * ax;
            dst[y * 96 + x] = (uint8_t)((top * (256 - ay) + bot * ay + (1 << 15)) >> 16);
        }
    }
}

// Bộ giải mã chính: esp_new_jpeg (tối ưu SIMD cho ESP32-S3), thu nhỏ ngay khi giải mã IDCT.
static bool decode_new(const HubFrame &f, uint8_t *dst) {
    if (!s_yuv) return false;
    int tw = 0, th = 0;
    choose_idct_scale(f.w, f.h, &tw, &th);
    if (tw <= 0 || th <= 0 || tw > YUV_MAX_W || th > YUV_MAX_H) return false;
    jpeg_dec_config_t cfg = DEFAULT_JPEG_DEC_CONFIG();
    cfg.output_type = JPEG_PIXEL_FORMAT_CbYCrY;
    if (tw != f.w || th != f.h) { cfg.scale.width = tw; cfg.scale.height = th; }
    jpeg_dec_handle_t hd = nullptr;
    if (jpeg_dec_open(&cfg, &hd) != JPEG_ERR_OK || !hd) return false;
    jpeg_dec_io_t io = {};
    io.inbuf = (uint8_t *)f.data;
    io.inbuf_len = (int)f.len;
    jpeg_dec_header_info_t info = {};
    int need = 0;
    bool ok = jpeg_dec_parse_header(hd, &io, &info) == JPEG_ERR_OK
              && info.width == tw && info.height == th
              && jpeg_dec_get_outbuf_len(hd, &need) == JPEG_ERR_OK
              && need <= YUV_MAX_W * YUV_MAX_H * 2;
    if (ok) {
        io.outbuf = s_yuv;
        ok = jpeg_dec_process(hd, &io) == JPEG_ERR_OK;
    }
    jpeg_dec_close(hd);
    if (ok) y_to_gray96(s_yuv, tw, th, dst);
    return ok;
}

static bool decode_to_gray96(const HubFrame &f, uint8_t *dst) {
    static uint8_t fails = 0;
    if (s_decoder != 2) {
        if (decode_new(f, dst)) { s_decoder = 1; fails = 0; return true; }
        if (s_decoder == 0 && ++fails >= 5) { s_decoder = 2; Serial.println("esp_new_jpeg loi -> dung tjpgd"); }
        if (s_decoder == 1) return decode_tjpgd(f, dst); // nếu đổi độ phân giải lẻ: tự động dùng tjpgd cho khung đó
    }
    return decode_tjpgd(f, dst);
}

// ------------------------------------------------------------------ bám vết mượt & sát biên (Fine-Grid Tracker)
struct Track { DetObj o; uint8_t miss; bool used; };
static Track s_tr[DET_MAX];
static uint16_t s_next_id = 1;

static int track_update(const fomo_obj_t *raw, int n, DetObj *out) {
    bool taken[FOMO_MAX_OBJECTS] = {false};
    for (int t = 0; t < DET_MAX; t++) {
        if (!s_tr[t].used) continue;
        int best = -1; float bd = 0.22f;
        for (int i = 0; i < n; i++) {
            if (taken[i]) continue;
            float dx = raw[i].cx / FOMO_GRID - s_tr[t].o.cx;
            float dy = raw[i].cy / FOMO_GRID - s_tr[t].o.cy;
            float d = sqrtf(dx * dx + dy * dy);
            if (d < bd) { bd = d; best = i; }
        }
        if (best >= 0) {
            taken[best] = true;
            DetObj &o = s_tr[t].o;
            float ncx = raw[best].cx / FOMO_GRID;
            float ncy = raw[best].cy / FOMO_GRID;
            float nx0 = raw[best].x0 / FOMO_GRID;
            float ny0 = raw[best].y0 / FOMO_GRID;
            float nw  = (raw[best].x1 - raw[best].x0) / FOMO_GRID;
            float nh  = (raw[best].y1 - raw[best].y0) / FOMO_GRID;
            // Làm mượt EMA (alpha = 0.60) cho cả tâm và 4 biên hộp để khung bao bám sát vật thể mượt mà
            o.cx = 0.40f * o.cx + 0.60f * ncx;
            o.cy = 0.40f * o.cy + 0.60f * ncy;
            o.x0 = 0.40f * o.x0 + 0.60f * nx0;
            o.y0 = 0.40f * o.y0 + 0.60f * ny0;
            o.w  = 0.40f * o.w  + 0.60f * nw;
            o.h  = 0.40f * o.h  + 0.60f * nh;
            o.score = raw[best].score;
            o.cells = raw[best].cells;
            if (o.age < 255) o.age++;
            s_tr[t].miss = 0;
        } else if (++s_tr[t].miss > 3) {
            s_tr[t].used = false;
        }
    }
    for (int i = 0; i < n; i++) {
        if (taken[i]) continue;
        for (int t = 0; t < DET_MAX; t++) if (!s_tr[t].used) {
            s_tr[t].used = true; s_tr[t].miss = 0;
            DetObj &o = s_tr[t].o;
            o.cx = raw[i].cx / FOMO_GRID;
            o.cy = raw[i].cy / FOMO_GRID;
            o.x0 = raw[i].x0 / FOMO_GRID;
            o.y0 = raw[i].y0 / FOMO_GRID;
            o.w  = (raw[i].x1 - raw[i].x0) / FOMO_GRID;
            o.h  = (raw[i].y1 - raw[i].y0) / FOMO_GRID;
            o.score = raw[i].score;
            o.cells = raw[i].cells;
            o.id = s_next_id++;
            o.age = 1;
            break;
        }
    }
    int k = 0;
    for (int t = 0; t < DET_MAX; t++) {
        // Hiển thị ngay khi phát hiện (miss <= 1) để người dùng thấy khung bao tức thì
        if (s_tr[t].used && s_tr[t].miss <= 1 && (s_tr[t].o.age >= 1 || s_tr[t].o.score >= 0.55f)) {
            out[k++] = s_tr[t].o;
        }
    }
    return k;
}

// ------------------------------------------------------------------ Edge Impulse (tùy chọn)
#if defined(USE_EDGE_IMPULSE)
static uint8_t *s_ei_img = nullptr;
static int ei_get_data(size_t offset, size_t length, float *out) {
    for (size_t i = 0; i < length; i++) {
        uint32_t v = s_ei_img[offset + i];
        out[i] = (float)((v << 16) | (v << 8) | v);
    }
    return 0;
}
static int run_edge_impulse(const uint8_t *gray96, fomo_obj_t *raw) {
    const int W = EI_CLASSIFIER_INPUT_WIDTH, H = EI_CLASSIFIER_INPUT_HEIGHT;
    if (!s_ei_img) s_ei_img = (uint8_t *)heap_caps_malloc(W * H, MALLOC_CAP_8BIT);
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) s_ei_img[y * W + x] = gray96[(y * 96 / H) * 96 + x * 96 / W];
    signal_t sig;
    sig.total_length = W * H;
    sig.get_data = &ei_get_data;
    ei_impulse_result_t r = {0};
    if (run_classifier(&sig, &r, false) != EI_IMPULSE_OK) return 0;
    int n = 0;
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    for (size_t i = 0; i < r.bounding_boxes_count && n < FOMO_MAX_OBJECTS; i++) {
        const auto &bb = r.bounding_boxes[i];
        if (bb.value == 0) continue;
        raw[n].cx = (bb.x + bb.width * 0.5f) * FOMO_GRID / W;
        raw[n].cy = (bb.y + bb.height * 0.5f) * FOMO_GRID / H;
        raw[n].x0 = (float)bb.x * FOMO_GRID / W; raw[n].x1 = (float)(bb.x + bb.width) * FOMO_GRID / W;
        raw[n].y0 = (float)bb.y * FOMO_GRID / H; raw[n].y1 = (float)(bb.y + bb.height) * FOMO_GRID / H;
        raw[n].score = bb.value;
        raw[n].cells = 4;
        n++;
    }
#endif
    return n;
}
#endif

// ------------------------------------------------------------------ công bố kết quả
static void publish(uint32_t seq, Backend used, float dec_ms, float inf_ms, uint32_t cycles,
                    const int8_t *logits, const uint8_t *fgrid24, const fomo_obj_t *raw, int n) {
    static uint32_t t_prev = 0;
    static float fps = 0;
    DetObj tracked[DET_MAX];
    int k = track_update(raw, n, tracked);
    uint32_t now = millis();
    if (t_prev) {
        float inst = 1000.0f / (float)(now - t_prev + 1);
        fps = fps == 0 ? inst : 0.85f * fps + 0.15f * inst;
    }
    t_prev = now;
    if (k > 0) s_last_person_ms = now;
    portENTER_CRITICAL(&s_mux);
    s_res.frame_seq = seq; s_res.t_ms = now; s_res.n = k;
    for (int i = 0; i < k; i++) s_res.obj[i] = tracked[i];
    s_res.backend_used = used; s_res.decode_ms = dec_ms; s_res.infer_ms = inf_ms; s_res.ai_fps = fps;
    s_res.fpga_cycles = cycles;
    s_res.fine_grid_res = (uint8_t)s_fine_grid;
    if (logits) memcpy(s_res.logits, logits, 144);
    if (fgrid24) memcpy(s_res.fgrid, fgrid24, FOMO_VIS_GRID * FOMO_VIS_GRID);
    portEXIT_CRITICAL(&s_mux);
}

void det_init() {
#if CPU_USE_ESPNN
    s_cpu_fast = fomo_fast_init() == 0;          // trọng số + scratch esp-nn trong Internal SRAM
#endif
    s_yuv = (uint8_t *)heap_caps_aligned_alloc(16, YUV_MAX_W * YUV_MAX_H * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    for (int i = 0; i < GRAY_BUFS; i++) s_gray[i] = (uint8_t *)heap_caps_malloc(96 * 96, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_pend_gray = (uint8_t *)heap_caps_malloc(96 * 96, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    memset(&s_res, 0, sizeof(s_res));
    s_thr = fomo_default_thr_logit();
    s_fine_grid = FOMO_DEFAULT_FINE_GRID;
    s_free_q = xQueueCreate(GRAY_BUFS, sizeof(int));
    s_ready_q = xQueueCreate(1, sizeof(int));
    for (int i = 0; i < GRAY_BUFS; i++) xQueueSend(s_free_q, &i, 0);
}

const char *det_cpu_engine() { return s_cpu_fast ? "esp-nn" : "C"; }
const char *det_decoder_name() { return s_decoder == 1 ? "esp_new_jpeg" : s_decoder == 2 ? "tjpgd" : "?"; }

// ------------------------------------------------------------------ task giải mã (core 1)
static void prep_task(void *arg) {
    uint32_t last_seq = 0;
    for (;;) {
        int idx;
        if (xQueueReceive(s_free_q, &idx, portMAX_DELAY) != pdTRUE) continue;
        HubFrame f;
        while (!hub_acquire_latest(&f, last_seq)) vTaskDelay(2);
        last_seq = f.seq;
        uint32_t t0 = micros();
        bool ok = decode_to_gray96(f, s_gray[idx]);
        hub_release(&f);
        if (!ok) { xQueueSend(s_free_q, &idx, 0); vTaskDelay(1); continue; }
        s_meta[idx].seq = f.seq;
        s_meta[idx].dec_ms = (micros() - t0) / 1000.0f;
        int old;
        if (xQueueReceive(s_ready_q, &old, 0) == pdTRUE) xQueueSend(s_free_q, &old, 0);
        xQueueSend(s_ready_q, &idx, 0);
        vTaskDelay(1);
    }
}

// ------------------------------------------------------------------ task suy luận (core 0)
void det_task(void *arg) {
    bool pending = false;            // một khung đang được FPGA xử lý
    uint32_t pend_seq = 0, pend_t0 = 0;
    float pend_dec = 0;
    uint8_t pend_cnt = 0;
    int8_t logits[144];
    uint8_t fgrid24[FOMO_VIS_GRID * FOMO_VIS_GRID];
    fomo_obj_t raw[FOMO_MAX_OBJECTS];
    for (;;) {
        vTaskDelay(1);
        if (s_fpga_mode_dirty && fpga_present()) { fpga_set_modes(s_fpga_mode); s_fpga_mode_dirty = false; }
        int idx;
        if (xQueueReceive(s_ready_q, &idx, pdMS_TO_TICKS(100)) != pdTRUE) continue;
        const uint8_t *gray = s_gray[idx];
        const uint32_t seq = s_meta[idx].seq;
        const float dec_ms = s_meta[idx].dec_ms;

        Backend b = s_backend;
        if (b == BK_AUTO) b = fpga_present() ? BK_FPGA : BK_CPU;
        if (b == BK_FPGA && !fpga_present()) b = BK_CPU;
#if !defined(USE_EDGE_IMPULSE)
        if (b == BK_EI) b = BK_CPU;
#endif
        if (b != BK_FPGA && pending) pending = false;

        if (b == BK_FPGA) {
            if (pending) {
                FpgaResult r;
                if (fpga_wait_result(&r, pend_cnt, 100)) {
                    const uint8_t *ref_img = s_pend_gray ? s_pend_gray : gray;
                    int n = fomo_decode_fine(ref_img, r.logits, s_thr, s_fine_grid, raw, FOMO_MAX_OBJECTS, fgrid24);
                    publish(pend_seq, BK_FPGA, pend_dec, (micros() - pend_t0) / 1000.0f, r.cycles, r.logits, fgrid24, raw, n);
                }
            }
            FpgaResult st;
            fpga_read_result(&st);
            pend_cnt = st.frame_cnt;
            pend_t0 = micros();
            fpga_send_frame(gray);
            if (s_pend_gray) memcpy(s_pend_gray, gray, 96 * 96);
            pending = true; pend_seq = seq; pend_dec = dec_ms;
        } else if (b == BK_EI) {
#if defined(USE_EDGE_IMPULSE)
            uint32_t t1 = micros();
            int n = run_edge_impulse(gray, raw);
            memset(fgrid24, 0, sizeof(fgrid24));
            publish(seq, BK_EI, dec_ms, (micros() - t1) / 1000.0f, 0, nullptr, fgrid24, raw, n);
#endif
        } else {
            uint32_t t1 = micros();
            if (s_cpu_fast) fomo_micro_run_fast(gray, logits);
            else            fomo_micro_run(gray, logits);
            int n = fomo_decode_fine(gray, logits, s_thr, s_fine_grid, raw, FOMO_MAX_OBJECTS, fgrid24);
            publish(seq, BK_CPU, dec_ms, (micros() - t1) / 1000.0f, 0, logits, fgrid24, raw, n);
        }
        xQueueSend(s_free_q, &idx, 0);
    }
}

void det_start() {
    for (int i = 0; i < GRAY_BUFS; i++)
        if (!s_gray[i]) { Serial.println("LOI: khong cap phat duoc bo dem anh xam (PSRAM?)"); return; }
    if (xTaskCreatePinnedToCore(prep_task, "prep", 8192, nullptr, 3, nullptr, 1) != pdPASS)
        Serial.println("LOI: khong tao duoc task giai ma (thieu SRAM)");
    if (xTaskCreatePinnedToCore(det_task, "fomo", 12288, nullptr, 4, nullptr, 0) != pdPASS)
        Serial.println("LOI: khong tao duoc task AI (thieu SRAM)");
}
