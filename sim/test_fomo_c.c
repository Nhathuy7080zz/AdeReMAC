// Kiểm chứng bit-exact bản C (firmware ESP32-S3) với mô hình tham chiếu Python + Kiểm tra lưới siêu mịn 48x48.
//   gcc -O2 -I../firmware_arduino/adaremac_fomo_cam test_fomo_c.c ../firmware_arduino/adaremac_fomo_cam/fomo_micro.c -lm -o test_fomo_c
//   ./test_fomo_c vectors
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "fomo_micro.h"
#include "fomo_micro_model.h"

static int read_hex(const char *path, unsigned *buf, int n) {
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    int k = 0;
    while (k < n && fscanf(f, "%x", &buf[k]) == 1) k++;
    fclose(f);
    return k;
}

int main(int argc, char **argv) {
    const char *dir = argc > 1 ? argv[1] : "vectors";
    int fails = 0, tests = 0;
    for (int t = 0; t < 16; t++) {
        char p[512];
        unsigned fr[9216], ex[144];
        snprintf(p, sizeof p, "%s/frame%d.hex", dir, t);
        if (read_hex(p, fr, 9216) != 9216) break;
        snprintf(p, sizeof p, "%s/expect%d_m2.hex", dir, t);
        if (read_hex(p, ex, 144) != 144) break;
        uint8_t img[9216], vis24[576];
        int8_t lg[144];
        for (int i = 0; i < 9216; i++) img[i] = (uint8_t)fr[i];
        clock_t c0 = clock();
        fomo_micro_run(img, lg);
        double ms = 1000.0 * (clock() - c0) / CLOCKS_PER_SEC;
        int bad = 0;
        for (int i = 0; i < 144; i++) bad += ((uint8_t)lg[i] != ex[i]);
        fomo_obj_t obj12[FOMO_MAX_OBJECTS], obj48[FOMO_MAX_OBJECTS];
        int n12 = fomo_decode(lg, FOMO_DEFAULT_THR_LOGIT, obj12, FOMO_MAX_OBJECTS);
        int n48 = fomo_decode_fine(img, lg, FOMO_DEFAULT_THR_LOGIT, 48, obj48, FOMO_MAX_OBJECTS, vis24);
        printf("vector %d: %s (%d/144 lech), %.2f ms, %d doi tuong", t, bad ? "SAI" : "KHOP", bad, ms, n48);
        for (int i = 0; i < n48; i++)
            printf(" | 12x12:[%.2f..%.2f,%.2f..%.2f] -> 48x48:[%.3f..%.3f,%.3f..%.3f (%u o, p=%.2f)]",
                   obj12[i].x0 / 12.0f, obj12[i].x1 / 12.0f, obj12[i].y0 / 12.0f, obj12[i].y1 / 12.0f,
                   obj48[i].x0 / 12.0f, obj48[i].x1 / 12.0f, obj48[i].y0 / 12.0f, obj48[i].y1 / 12.0f,
                   obj48[i].cells, obj48[i].score);
        printf("\n");
        fails += bad != 0;
        tests++;
    }
    printf("%s: %d/%d vector khop bit-exact\n", fails ? "FAIL" : "PASS", tests - fails, tests);
    return fails || !tests;
}
