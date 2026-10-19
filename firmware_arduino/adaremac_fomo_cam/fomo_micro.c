// FOMO-Micro INT8 — suy luận C thuần (ESP32-S3 hoặc máy tính) + Giải mã Lưới Siêu Mịn (Fine-Grid 48x48).
// Đặc tả số học: xem ai_fomo/fomo_int8.py. Mọi phép nhân ở đây là CHÍNH XÁC (tương đương Mode 2 của FPGA).
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "fomo_micro_model.h"
#include "fomo_micro.h"

#if defined(ESP_PLATFORM)
#include "esp_attr.h"
#define FOMO_FAST IRAM_ATTR
#else
#define FOMO_FAST
#endif

// Bộ đệm có viền 0 (padding 1) cho từng lớp. Kích thước lớn nhất: 8 kênh x 50 x 50 = 20000 B.
#define PAD_BUF_SZ 20000
#if defined(ESP_PLATFORM)
// Trên ESP32 bản chuẩn chỉ dùng khi tắt esp-nn -> cấp phát khi cần, để dành SRAM cho esp-nn/Wi-Fi.
static uint8_t *s_buf_a = 0, *s_buf_b = 0;
static int ensure_bufs(void) {
    if (!s_buf_a) s_buf_a = (uint8_t *)malloc(PAD_BUF_SZ);
    if (!s_buf_b) s_buf_b = (uint8_t *)malloc(PAD_BUF_SZ);
    return s_buf_a && s_buf_b;
}
#else
static uint8_t s_buf_a[PAD_BUF_SZ];
static uint8_t s_buf_b[PAD_BUF_SZ];
static int ensure_bufs(void) { return 1; }
#endif

static inline int32_t requant(int32_t acc, uint16_t m, int sh, int relu) {
    int64_t v = ((int64_t)acc * m + ((int64_t)1 << (sh - 1))) >> sh;
    if (relu) return v < 0 ? 0 : (v > 255 ? 255 : (int32_t)v);
    return v < -128 ? -128 : (v > 127 ? 127 : (int32_t)v);
}

// in: [cin][hi+2p][wi+2p] (viền 0). out: nếu out_pad = 1 thì [cout][ho+2][wo+2] có viền 0, ngược lại [cout][ho][wo].
static void FOMO_FAST conv_layer(const fomo_layer_t *L, const uint8_t *in, uint8_t *out, int out_pad, int8_t *out_i8) {
    const int k = L->k, s = L->s, p = k / 2;
    const int hp = L->hi + 2 * p, wp = L->wi + 2 * p;
    const int op = out_pad ? 1 : 0;
    const int ohp = L->ho + 2 * op, owp = L->wo + 2 * op;
    const int plane = hp * wp;
    if (out) memset(out, 0, (size_t)L->cout * ohp * owp);
    for (int co = 0; co < L->cout; co++) {
        const int8_t *wc = L->w + co * L->cin * k * k;
        const int32_t b = L->b[co];
        const uint16_t m = L->m[co];
        for (int oy = 0; oy < L->ho; oy++) {
            for (int ox = 0; ox < L->wo; ox++) {
                int32_t acc = 0;
                const uint8_t *pin = in + (oy * s) * wp + ox * s;
                const int8_t *wk = wc;
                if (k == 3) {
                    for (int ci = 0; ci < L->cin; ci++, pin += plane, wk += 9) {
                        const uint8_t *r0 = pin, *r1 = pin + wp, *r2 = pin + 2 * wp;
                        acc += r0[0] * wk[0] + r0[1] * wk[1] + r0[2] * wk[2]
                             + r1[0] * wk[3] + r1[1] * wk[4] + r1[2] * wk[5]
                             + r2[0] * wk[6] + r2[1] * wk[7] + r2[2] * wk[8];
                    }
                } else {
                    for (int ci = 0; ci < L->cin; ci++, pin += plane)
                        acc += pin[0] * wk[ci];
                }
                int32_t y = requant(acc + b, m, L->sh, L->relu);
                if (out_i8) out_i8[(co * L->ho + oy) * L->wo + ox] = (int8_t)y;
                else out[(co * ohp + oy + op) * owp + ox + op] = (uint8_t)y;
            }
        }
    }
}

void fomo_micro_run(const uint8_t *img, int8_t *logits) {
    if (!ensure_bufs()) { memset(logits, -128, 144); return; }
    const fomo_layer_t *L0 = &FOMO_LAYERS[0];
    const int wp0 = L0->wi + 2;
    memset(s_buf_a, 0, (size_t)(L0->hi + 2) * wp0);
    for (int y = 0; y < L0->hi; y++) memcpy(&s_buf_a[(y + 1) * wp0 + 1], &img[y * L0->wi], L0->wi);
    uint8_t *cur = s_buf_a, *nxt = s_buf_b;
    for (int i = 0; i < FOMO_NUM_LAYERS; i++) {
        const fomo_layer_t *L = &FOMO_LAYERS[i];
        int last = (i == FOMO_NUM_LAYERS - 1);
        int next_pad = last ? 0 : (FOMO_LAYERS[i + 1].k / 2);
        if (last) {
            conv_layer(L, cur, 0, 0, logits);
        } else {
            conv_layer(L, cur, nxt, next_pad, 0);
            uint8_t *t = cur; cur = nxt; nxt = t;
        }
    }
}

int fomo_decode(const int8_t *logits, int thr_logit, fomo_obj_t *out, int max_out) {
    uint8_t lab[FOMO_GRID * FOMO_GRID];
    int stack[FOMO_GRID * FOMO_GRID];
    memset(lab, 0, sizeof(lab));
    int n = 0;
    for (int start = 0; start < FOMO_GRID * FOMO_GRID; start++) {
        if (lab[start] || logits[start] < thr_logit) continue;
        if (n >= max_out) break;
        float sw = 0, sx = 0, sy = 0, best = -1;
        int x0 = FOMO_GRID, y0 = FOMO_GRID, x1 = -1, y1 = -1, cnt = 0, sp = 0;
        lab[start] = 1;
        stack[sp++] = start;
        while (sp) {
            int c = stack[--sp], cy = c / FOMO_GRID, cx = c % FOMO_GRID;
            float pr = 1.0f / (1.0f + expf(-(float)logits[c] * FOMO_LOGIT_SCALE));
            sw += pr; sx += (cx + 0.5f) * pr; sy += (cy + 0.5f) * pr;
            if (pr > best) best = pr;
            if (cx < x0) x0 = cx;
            if (cx > x1) x1 = cx;
            if (cy < y0) y0 = cy;
            if (cy > y1) y1 = cy;
            cnt++;
            for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++) {
                    int nx = cx + dx, ny = cy + dy;
                    if (nx < 0 || ny < 0 || nx >= FOMO_GRID || ny >= FOMO_GRID) continue;
                    int nc = ny * FOMO_GRID + nx;
                    if (!lab[nc] && logits[nc] >= thr_logit) { lab[nc] = 1; stack[sp++] = nc; }
                }
        }
        out[n].cx = sx / sw; out[n].cy = sy / sw; out[n].score = best; out[n].cells = (uint16_t)cnt;
        out[n].x0 = (float)x0; out[n].y0 = (float)y0; out[n].x1 = (float)(x1 + 1); out[n].y1 = (float)(y1 + 1);
        n++;
    }
    return n;
}

// =============================================================================
// Hậu xử lý Lưới Siêu Mịn (Fine-Grid 48x48 = 2.304 ô / 24x24 = 576 ô):
// Giải quyết triệt để hạn chế của FOMO chuẩn (lưới 12x12 thô chỉ kích hoạt ở tâm đối tượng):
//   1. Chia ảnh 96x96 thành lưới mịn GxG (mặc định G = 48 -> 2.304 ô, mỗi ô chỉ 2x2 điểm ảnh,
//      mật độ ô cao gấp 16 lần lưới 12x12 trên cùng diện tích).
//   2. Nội suy song tuyến (Bilinear Interpolation) bản đồ logit 12x12 lên lưới mịn GxG.
//   3. Tính đặc trưng cường độ sáng cục bộ I_cell[G][G], năng lượng biên Sobel Grad[G][G]
//      và độ lệch nền thích nghi BG_cell[G][G] ngay trên lưới mịn GxG.
//   4. Từ mỗi hạt giống (seed) đối tượng của FOMO, thực hiện lan truyền vùng song phương
//      (Bilateral Silhouette Region Growing) trên lưới GxG để gom toàn bộ các ô lưới mịn
//      thuộc phần thân vật thể và khóa viền khung bao [x0, y0, x1, y1] sát đường biên thực tế.
// =============================================================================
static uint8_t s_bg48[FOMO_FINE_MAX * FOMO_FINE_MAX];
static uint8_t s_bg_inited = 0;

static inline float interp_logit12(const int8_t *lg12, float gx, float gy, int G) {
    // Đưa tọa độ tâm ô lưới mịn (gx + 0.5)/G về hệ tọa độ ô 12x12
    float u = ((gx + 0.5f) * (float)FOMO_GRID / (float)G) - 0.5f;
    float v = ((gy + 0.5f) * (float)FOMO_GRID / (float)G) - 0.5f;
    if (u < 0.0f) u = 0.0f;
    if (v < 0.0f) v = 0.0f;
    if (u > (float)(FOMO_GRID - 1)) u = (float)(FOMO_GRID - 1);
    if (v > (float)(FOMO_GRID - 1)) v = (float)(FOMO_GRID - 1);
    int x0 = (int)u, y0 = (int)v;
    int x1 = (x0 + 1 < FOMO_GRID) ? (x0 + 1) : x0;
    int y1 = (y0 + 1 < FOMO_GRID) ? (y0 + 1) : y0;
    float ax = u - (float)x0, ay = v - (float)y0;
    float v00 = (float)lg12[y0 * FOMO_GRID + x0];
    float v01 = (float)lg12[y0 * FOMO_GRID + x1];
    float v10 = (float)lg12[y1 * FOMO_GRID + x0];
    float v11 = (float)lg12[y1 * FOMO_GRID + x1];
    return (v00 * (1.0f - ax) + v01 * ax) * (1.0f - ay) + (v10 * (1.0f - ax) + v11 * ax) * ay;
}

int fomo_decode_fine(const uint8_t *img96, const int8_t *logits12, int thr_logit,
                     int fine_grid_res, fomo_obj_t *out, int max_out, uint8_t *vis_24x24) {
    if (!img96 || !logits12) return fomo_decode(logits12, thr_logit, out, max_out);

    int G = fine_grid_res;
    if (G != 12 && G != 24 && G != 48) G = 48;
    const float scale_to_12 = (float)FOMO_GRID / (float)G;
    const int cell_px = 96 / G; // G=48 -> 2px, G=24 -> 4px, G=12 -> 8px

    // Bước 1: Giải mã các cụm hạt giống (seeds) trên lưới 12x12
    fomo_obj_t seeds[FOMO_MAX_OBJECTS];
    int n_seeds = fomo_decode(logits12, thr_logit, seeds, max_out);

    // Bước 2: Tính bản đồ độ sáng I_cell[G*G], biên cạnh Grad[G*G] và xác suất mịn P_fine[G*G]
    uint8_t icell[FOMO_FINE_MAX * FOMO_FINE_MAX];
    uint8_t grad[FOMO_FINE_MAX * FOMO_FINE_MAX];
    int8_t  lfine[FOMO_FINE_MAX * FOMO_FINE_MAX];
    uint8_t obj_mask[FOMO_FINE_MAX * FOMO_FINE_MAX];
    memset(obj_mask, 0, (size_t)(G * G));

    for (int gy = 0; gy < G; gy++) {
        int py0 = gy * cell_px;
        for (int gx = 0; gx < G; gx++) {
            int px0 = gx * cell_px;
            int sum = 0;
            for (int dy = 0; dy < cell_px; dy++) {
                const uint8_t *row = &img96[(py0 + dy) * 96 + px0];
                for (int dx = 0; dx < cell_px; dx++) sum += row[dx];
            }
            uint8_t mean_v = (uint8_t)(sum / (cell_px * cell_px));
            int idx = gy * G + gx;
            icell[idx] = mean_v;
            float lf = interp_logit12(logits12, (float)gx, (float)gy, G);
            int li = (int)lrintf(lf);
            if (li < -128) li = -128;
            if (li > 127)  li = 127;
            lfine[idx] = (int8_t)li;
        }
    }

    if (!s_bg_inited) {
        memcpy(s_bg48, icell, (size_t)(G * G));
        s_bg_inited = 1;
    }

    // Tính năng lượng biên cục bộ Grad[gy][gx] trên lưới mịn GxG
    for (int gy = 0; gy < G; gy++) {
        int ym = (gy > 0) ? (gy - 1) : 0;
        int yp = (gy + 1 < G) ? (gy + 1) : (G - 1);
        for (int gx = 0; gx < G; gx++) {
            int xm = (gx > 0) ? (gx - 1) : 0;
            int xp = (gx + 1 < G) ? (gx + 1) : (G - 1);
            int dx = abs((int)icell[gy * G + xp] - (int)icell[gy * G + xm]);
            int dy = abs((int)icell[yp * G + gx] - (int)icell[ym * G + gx]);
            int gval = dx + dy;
            grad[gy * G + gx] = (uint8_t)(gval > 255 ? 255 : gval);
        }
    }

    // Nếu G == 12 thì chỉ cần xuất lưới 24x24 (nếu có) và trả về kết quả 12x12
    if (G == 12) {
        if (vis_24x24) {
            for (int vy = 0; vy < FOMO_VIS_GRID; vy++) {
                for (int vx = 0; vx < FOMO_VIS_GRID; vx++) {
                    int lg = logits12[(vy / 2) * 12 + (vx / 2)];
                    float p = 1.0f / (1.0f + expf(-(float)lg * FOMO_LOGIT_SCALE));
                    vis_24x24[vy * FOMO_VIS_GRID + vx] = (uint8_t)(p * 100.0f + 0.5f);
                }
            }
        }
        for (int i = 0; i < n_seeds; i++) out[i] = seeds[i];
        return n_seeds;
    }

    // Bước 3: Với mỗi hạt giống đối tượng (seed), thực hiện Lan truyền vùng song phương
    // (Bilateral Silhouette Region Growing) trên lưới siêu mịn GxG để đóng khung sát biên vật thể
    uint16_t bfs_q[FOMO_FINE_MAX * FOMO_FINE_MAX];
    uint8_t  visited[FOMO_FINE_MAX * FOMO_FINE_MAX];

    int n_out = 0;
    for (int s = 0; s < n_seeds && n_out < max_out; s++) {
        // Quy đổi tâm và hộp thô từ hệ 12x12 sang hệ lưới mịn GxG
        int scx = (int)(seeds[s].cx / scale_to_12);
        int scy = (int)(seeds[s].cy / scale_to_12);
        if (scx < 0) scx = 0; if (scx >= G) scx = G - 1;
        if (scy < 0) scy = 0; if (scy >= G) scy = G - 1;

        // Tính thống kê cường độ sáng vùng lõi vật thể (cửa sổ 3x3 quanh tâm hạt giống) và vùng nền xung quanh
        int core_sum = 0, core_sq = 0, core_n = 0;
        int r_core = (G == 48) ? 3 : 2;
        for (int dy = -r_core; dy <= r_core; dy++) {
            for (int dx = -r_core; dx <= r_core; dx++) {
                int nx = scx + dx, ny = scy + dy;
                if (nx >= 0 && nx < G && ny >= 0 && ny < G) {
                    int v = icell[ny * G + nx];
                    core_sum += v;
                    core_sq  += v * v;
                    core_n++;
                }
            }
        }
        int mu_core = core_sum / (core_n > 0 ? core_n : 1);
        int var_core = (core_sq / (core_n > 0 ? core_n : 1)) - (mu_core * mu_core);
        int std_core = (var_core > 0) ? (int)sqrtf((float)var_core) : 12;
        int tol_i = std_core + 22;
        if (tol_i < 20) tol_i = 20;
        if (tol_i > 48) tol_i = 48;

        // Giới hạn bán kính mở rộng hợp lý từ hộp FOMO ban đầu để đóng khung trọn vẹn thân vật thể
        // mà không lan tràn ra nền tĩnh
        int seed_w = (int)((seeds[s].x1 - seeds[s].x0) / scale_to_12);
        int seed_h = (int)((seeds[s].y1 - seeds[s].y0) / scale_to_12);
        int max_rx = seed_w + (G / 5);   // mở rộng tối đa để ôm sát biên ngang của người/vật thể
        int max_ry = seed_h + (G / 3);   // mở rộng tối đa theo chiều dọc (tỷ lệ cơ thể người/vật)
        if (max_rx < G / 6) max_rx = G / 6;
        if (max_ry < G / 4) max_ry = G / 4;
        if (max_rx > G / 2) max_rx = G / 2;
        if (max_ry > (G * 3) / 4) max_ry = (G * 3) / 4;

        memset(visited, 0, (size_t)(G * G));
        int qh = 0, qt = 0;

        // Khởi tạo hàng đợi BFS từ mọi ô lưới mịn có logit nội suy >= thr_logit trong vùng hạt giống
        int sx0 = (int)(seeds[s].x0 / scale_to_12) - 1; if (sx0 < 0) sx0 = 0;
        int sy0 = (int)(seeds[s].y0 / scale_to_12) - 1; if (sy0 < 0) sy0 = 0;
        int sx1 = (int)(seeds[s].x1 / scale_to_12) + 1; if (sx1 >= G) sx1 = G - 1;
        int sy1 = (int)(seeds[s].y1 / scale_to_12) + 1; if (sy1 >= G) sy1 = G - 1;

        for (int gy = sy0; gy <= sy1; gy++) {
            for (int gx = sx0; gx <= sx1; gx++) {
                int idx = gy * G + gx;
                if (lfine[idx] >= thr_logit - 1) {
                    visited[idx] = 1;
                    bfs_q[qt++] = (uint16_t)idx;
                }
            }
        }
        if (qt == 0) {
            int idx = scy * G + scx;
            visited[idx] = 1;
            bfs_q[qt++] = (uint16_t)idx;
        }

        // Tính mức nền trung bình của bản đồ logit 12x12 để đặt sàn lan truyền vùng thân vật thể thích nghi:
        // Nền xa thường ở mức -70..-100, trong khi vùng thân/vai/chân quanh tâm FOMO ở mức -40..+30.
        int l_min_sum = 0;
        for (int i = 0; i < FOMO_GRID * FOMO_GRID; i++) l_min_sum += logits12[i];
        int l_mean = l_min_sum / (FOMO_GRID * FOMO_GRID);
        int soft_logit_floor = thr_logit - 38;
        if (l_mean + 20 > soft_logit_floor) soft_logit_floor = l_mean + 20;
        if (soft_logit_floor > thr_logit - 12) soft_logit_floor = thr_logit - 12;
        while (qh < qt) {
            int cur = bfs_q[qh++];
            int cy = cur / G, cx = cur % G;
            int cur_i = icell[cur];

            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    if (dx == 0 && dy == 0) continue;
                    int nx = cx + dx, ny = cy + dy;
                    if (nx < 0 || nx >= G || ny < 0 || ny >= G) continue;
                    if (abs(nx - scx) > max_rx || abs(ny - scy) > max_ry) continue;
                    int nidx = ny * G + nx;
                    if (visited[nidx]) continue;

                    int nl = lfine[nidx];
                    int ni = icell[nidx];
                    int ng = grad[nidx];
                    int bg_diff = abs(ni - (int)s_bg48[nidx]);

                    // Điều kiện 1: Ô lưới mịn nằm trong vùng xác suất ngữ nghĩa FOMO cao
                    int cond_sem = (nl >= thr_logit - 2);
                    // Điều kiện 2: Ô lưới mịn liên thông với thân vật thể (đồng nhất quang phổ với lõi +
                    //              có độ tương phản so với nền hoặc biên cạnh rõ, và logit >= sàn rìa)
                    int cond_body = (nl >= soft_logit_floor) &&
                                     (abs(ni - mu_core) <= tol_i) &&
                                     (abs(ni - cur_i) <= 18) &&
                                     (bg_diff >= 10 || ng >= 14 || nl >= thr_logit - 6);

                    if (cond_sem || cond_body) {
                        visited[nidx] = 1;
                        bfs_q[qt++] = (uint16_t)nidx;
                    }
                }
            }
        }

        // Bước 4: Lọc hình thái học (loại bỏ gai nhiễu đơn lẻ 1 ô ở rìa) & Khóa biên Sobel (Edge Snapping)
        int col_cnt[FOMO_FINE_MAX] = {0};
        int row_cnt[FOMO_FINE_MAX] = {0};
        float sw = 0.0f, sum_x = 0.0f, sum_y = 0.0f;
        int active_cells = 0;

        for (int i = 0; i < qt; i++) {
            int idx = bfs_q[i];
            int cy = idx / G, cx = idx % G;
            // Đếm số láng giềng 3x3 cùng thuộc đối tượng để loại gai nhiễu
            int nb = 0;
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    int nx = cx + dx, ny = cy + dy;
                    if (nx >= 0 && nx < G && ny >= 0 && ny < G && visited[ny * G + nx]) nb++;
                }
            }
            if (nb >= 2 || qt <= 2) {
                obj_mask[idx] = (uint8_t)(s + 1);
                col_cnt[cx]++;
                row_cnt[cy]++;
                float pr = 1.0f / (1.0f + expf(-(float)lfine[idx] * FOMO_LOGIT_SCALE));
                float w = pr + 0.05f * (float)grad[idx] / 255.0f;
                sw += w;
                sum_x += ((float)cx + 0.5f) * w;
                sum_y += ((float)cy + 0.5f) * w;
                active_cells++;
            }
        }

        // Tìm biên trái/phải/trên/dưới sát nhất theo mật độ ô lưới mịn & đỉnh Gradient biên
        int fx0 = 0, fx1 = G - 1, fy0 = 0, fy1 = G - 1;
        while (fx0 < G && col_cnt[fx0] == 0) fx0++;
        while (fx1 > fx0 && col_cnt[fx1] == 0) fx1--;
        while (fy0 < G && row_cnt[fy0] == 0) fy0++;
        while (fy1 > fy0 && row_cnt[fy1] == 0) fy1--;

        // Cắt bỏ rìa thưa (chỉ có 1 ô đơn độc khi cụm đủ lớn) để hộp ôm sát biên thật của vật thể
        if (fx1 - fx0 >= 4 && col_cnt[fx0] <= 1 && grad[scy * G + fx0] < 18) fx0++;
        if (fx1 - fx0 >= 4 && col_cnt[fx1] <= 1 && grad[scy * G + fx1] < 18) fx1--;
        if (fy1 - fy0 >= 4 && row_cnt[fy0] <= 1 && grad[fy0 * G + scx] < 18) fy0++;
        if (fy1 - fy0 >= 4 && row_cnt[fy1] <= 1 && grad[fy1 * G + scx] < 18) fy1--;

        if (fx0 > fx1) { fx0 = scx; fx1 = scx; }
        if (fy0 > fy1) { fy0 = scy; fy1 = scy; }

        out[n_out].cx = (sw > 0.0f ? (sum_x / sw) : ((float)scx + 0.5f)) * scale_to_12;
        out[n_out].cy = (sw > 0.0f ? (sum_y / sw) : ((float)scy + 0.5f)) * scale_to_12;
        out[n_out].x0 = (float)fx0 * scale_to_12;
        out[n_out].y0 = (float)fy0 * scale_to_12;
        out[n_out].x1 = (float)(fx1 + 1) * scale_to_12;
        out[n_out].y1 = (float)(fy1 + 1) * scale_to_12;
        out[n_out].score = seeds[s].score;
        out[n_out].cells = (uint16_t)active_cells;
        n_out++;
    }

    // Bước 4: Cập nhật mô hình nền thích nghi (EMA chậm) cho các ô không thuộc đối tượng
    for (int i = 0; i < G * G; i++) {
        if (!obj_mask[i]) {
            s_bg48[i] = (uint8_t)(((int)s_bg48[i] * 15 + (int)icell[i] + 8) >> 4);
        }
    }

    // Bước 5: Xuất bản đồ lưới mịn 24x24 (576 ô) cho Web UI hiển thị trực quan
    if (vis_24x24) {
        for (int vy = 0; vy < FOMO_VIS_GRID; vy++) {
            for (int vx = 0; vx < FOMO_VIS_GRID; vx++) {
                if (G == 48) {
                    // Gộp 2x2 ô 48x48 thành 1 ô hiển thị 24x24
                    int y0 = vy * 2, x0 = vx * 2;
                    int in_obj = (obj_mask[y0 * 48 + x0] != 0) + (obj_mask[y0 * 48 + x0 + 1] != 0) +
                                 (obj_mask[(y0 + 1) * 48 + x0] != 0) + (obj_mask[(y0 + 1) * 48 + x0 + 1] != 0);
                    int lmax = lfine[y0 * 48 + x0];
                    if (lfine[y0 * 48 + x0 + 1] > lmax) lmax = lfine[y0 * 48 + x0 + 1];
                    if (lfine[(y0 + 1) * 48 + x0] > lmax) lmax = lfine[(y0 + 1) * 48 + x0];
                    if (lfine[(y0 + 1) * 48 + x0 + 1] > lmax) lmax = lfine[(y0 + 1) * 48 + x0 + 1];
                    float p = 1.0f / (1.0f + expf(-(float)lmax * FOMO_LOGIT_SCALE));
                    int pct = (int)(p * 100.0f + 0.5f);
                    if (in_obj > 0 && pct < 45) pct = 40 + in_obj * 12;
                    if (pct > 100) pct = 100;
                    vis_24x24[vy * FOMO_VIS_GRID + vx] = (uint8_t)pct;
                } else {
                    int idx = vy * 24 + vx;
                    float p = 1.0f / (1.0f + expf(-(float)lfine[idx] * FOMO_LOGIT_SCALE));
                    int pct = (int)(p * 100.0f + 0.5f);
                    if (obj_mask[idx] && pct < 55) pct = 65;
                    vis_24x24[idx] = (uint8_t)(pct > 100 ? 100 : pct);
                }
            }
        }
    }

    return n_out;
}

int fomo_default_thr_logit(void) { return FOMO_DEFAULT_THR_LOGIT; }
float fomo_logit_scale(void) { return FOMO_LOGIT_SCALE; }
