// FOMO-Micro INT8 chạy bằng thư viện esp-nn (có sẵn trong gói esp32 cho Arduino, dùng lệnh SIMD của ESP32-S3).
//
// Cùng trọng số, cùng bias, cùng phép nhân với bản C chuẩn (fomo_micro.c). Chỉ khác ở bước lượng tử lại:
// esp-nn dùng kiểu làm tròn của TFLite (nhân Q31, rồi dịch phải có làm tròn), còn bản chuẩn làm tròn một lần.
// Hai cách lệch nhau tối đa 1 LSB ở vài kích hoạt; sim/test_fomo_c.c đo mức lệch này trên tập val COCO.
// Bản C chuẩn vẫn là tham chiếu bit-exact cho FPGA (Mode 2).
//
// Bố cục esp-nn: kích hoạt NHWC int8 (giá trị uint8 của mô hình trừ 128), trọng số OHWI.
#include <string.h>
#include <stdlib.h>
#include "fomo_micro_model.h"
#include "fomo_micro.h"

#if defined(ESP_PLATFORM)
#include "esp_nn.h"
#include "esp_heap_caps.h"
#define FOMO_HAVE_ESPNN 1
#define ALLOC_ALIGNED(sz) heap_caps_aligned_alloc(16, (sz), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
#elif defined(FOMO_ESPNN_HOST)
// Kiểm tra trên máy tính: dùng bản C tham chiếu (ANSI) của esp-nn, cùng công thức với bản SIMD.
#include "esp_nn_defs.h"
#include "esp_nn_ansi_headers.h"
#define esp_nn_conv_s8               esp_nn_conv_s8_ansi
#define esp_nn_get_conv_scratch_size esp_nn_get_conv_scratch_size_ansi
#define esp_nn_set_conv_scratch_buf  esp_nn_set_conv_scratch_buf_ansi
#define FOMO_HAVE_ESPNN 1
#define ALLOC_ALIGNED(sz) aligned_alloc(16, (((sz) + 15) / 16) * 16)
#endif

#if defined(FOMO_HAVE_ESPNN)

#define ACT_A_SZ (48 * 48 * 8)
#define ACT_B_SZ (24 * 24 * 16)
static int8_t s_in[96 * 96] __attribute__((aligned(16)));
static int8_t s_act_a[ACT_A_SZ] __attribute__((aligned(16)));
static int8_t s_act_b[ACT_B_SZ] __attribute__((aligned(16)));
static int8_t s_out[FOMO_GRID * FOMO_GRID] __attribute__((aligned(16)));   // lớp cuối ghi vào bộ đệm căn 16 byte
static int8_t *s_w[FOMO_NUM_LAYERS];
static int32_t s_mult[FOMO_NUM_LAYERS][32], s_shift[FOMO_NUM_LAYERS][32];
static void *s_scratch = 0;
static int s_ready = 0;

static void dims_of(const fomo_layer_t *L, data_dims_t *in, data_dims_t *fl, data_dims_t *out, conv_params_t *cp) {
    in->width = L->wi; in->height = L->hi; in->channels = L->cin; in->extra = 1;
    fl->width = L->k; fl->height = L->k; fl->channels = L->cin; fl->extra = L->cout;
    out->width = L->wo; out->height = L->ho; out->channels = L->cout; out->extra = 1;
    cp->in_offset = 128;                          // kích hoạt int8 + 128 = giá trị uint8 của mô hình
    cp->out_offset = L->relu ? -128 : 0;          // ReLU: [0,255] -> [-128,127]; lớp cuối: logit int8
    cp->stride.width = L->s; cp->stride.height = L->s;
    cp->padding.width = L->k / 2; cp->padding.height = L->k / 2;
    cp->dilation.width = 1; cp->dilation.height = 1;
    cp->activation.min = -128; cp->activation.max = 127;
}

int fomo_fast_init(void) {
    if (s_ready) return 0;
    int scratch = 0;
    for (int i = 0; i < FOMO_NUM_LAYERS; i++) {
        const fomo_layer_t *L = &FOMO_LAYERS[i];
        const int k = L->k, cin = L->cin, n = L->cout * cin * k * k;
        s_w[i] = (int8_t *)ALLOC_ALIGNED(n);
        if (!s_w[i]) return -1;
        // [co][ci][ky][kx] -> [co][ky][kx][ci]
        for (int co = 0; co < L->cout; co++)
            for (int ci = 0; ci < cin; ci++)
                for (int ky = 0; ky < k; ky++)
                    for (int kx = 0; kx < k; kx++)
                        s_w[i][((co * k + ky) * k + kx) * cin + ci] = L->w[((co * cin + ci) * k + ky) * k + kx];
        // y = round(acc * m / 2^sh)  ==  TFLite: high_mul(acc, M) rồi dịch phải r, với M = m * 2^(15+j)
        for (int co = 0; co < L->cout; co++) {
            int64_t M = (int64_t)L->m[co] << 15;
            int r = L->sh - 16;
            while (M < (1LL << 30) && M > 0) { M <<= 1; r++; }
            s_mult[i][co] = (int32_t)M;
            s_shift[i][co] = -r;
        }
        data_dims_t din, dfl, dout; conv_params_t cp;
        dims_of(L, &din, &dfl, &dout, &cp);
        int sz = esp_nn_get_conv_scratch_size(&din, &dfl, &dout, &cp);
        if (sz > scratch) scratch = sz;
    }
    if (scratch > 0) {
        s_scratch = ALLOC_ALIGNED(scratch + 64);
        if (!s_scratch) return -2;
    }
    s_ready = 1;
    return 0;
}

int fomo_fast_available(void) { return s_ready; }

void fomo_micro_run_fast(const uint8_t *img, int8_t *logits) {
    if (!s_ready) { fomo_micro_run(img, logits); return; }
    for (int i = 0; i < 96 * 96; i++) s_in[i] = (int8_t)(img[i] ^ 0x80);
    const int8_t *cur = s_in;
    for (int i = 0; i < FOMO_NUM_LAYERS; i++) {
        const fomo_layer_t *L = &FOMO_LAYERS[i];
        int8_t *dst = (i == FOMO_NUM_LAYERS - 1) ? s_out : ((cur == s_act_a) ? s_act_b : s_act_a);
        data_dims_t din, dfl, dout; conv_params_t cp;
        dims_of(L, &din, &dfl, &dout, &cp);
        quant_data_t q = { s_shift[i], s_mult[i] };
        esp_nn_set_conv_scratch_buf(s_scratch);
        esp_nn_conv_s8(&din, cur, &dfl, s_w[i], L->b, &dout, dst, &cp, &q);
        cur = dst;
    }
    memcpy(logits, s_out, sizeof(s_out));
}

#else   // không có esp-nn: dùng bản C chuẩn

int fomo_fast_init(void) { return -1; }
int fomo_fast_available(void) { return 0; }
void fomo_micro_run_fast(const uint8_t *img, int8_t *logits) { fomo_micro_run(img, logits); }

#endif
