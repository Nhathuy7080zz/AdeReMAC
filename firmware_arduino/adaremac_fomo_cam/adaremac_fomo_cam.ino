// =============================================================================
// AdaReMAC-EdgeVision v4.3 — Phát hiện người FOMO Lưới Siêu Mịn (48x48 = 2.304 ô)
//                            + Stream MJPEG Sắc Nét (SVGA/HD + ISP CIP + VCM Focus)
// ESP32-S3 N16R8 CAM (OV5640) [+ Sipeed Tang Nano 20K làm bộ tăng tốc FOMO qua SPI]
//
//  Core 1 : task camera (lấy khung, trả framebuffer ngay) + HTTP stream/UI + task giải mã JPEG -> xám 96x96
//  Core 0 : task AI (FOMO trên FPGA / ESP32 (esp-nn) + Fine-Grid 48x48 Silhouette Growing -> bám vết)
//  Luồng video KHÔNG chờ AI: AI chạy ở tốc độ riêng, web vẽ khung bao sát biên + ô lưới mịn 60Hz.
//
//  Mở trình duyệt: http://192.168.4.1  (Wi-Fi "AdaReMAC-FOMO", mật khẩu 12345678)
// =============================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "app_config.h"
#include "cam_ov5640.h"
#include "frame_hub.h"
#include "fpga_link.h"
#include "detector.h"
#include "web_ui.h"

// ------------------------------------------------------------------ task camera + lấy nét VCM + quản lý nhiệt
// ACTIVE : OV5640 chạy liên tục (luôn giữ ACTIVE khi đang có trình duyệt xem stream để ảnh không bị mờ/giật).
// IDLE   : Chỉ kích hoạt khi KHÔNG có người xem (stream_clients == 0), bật "tiết kiệm nhiệt" (hoặc quá nhiệt),
//          và không phát hiện người trong THERMAL_IDLE_AFTER_MS -> OV5640 ngủ (0x3008 = 0x42),
//          mỗi THERMAL_IDLE_PERIOD_MS thức dậy bỏ THERMAL_WAKE_DISCARD khung cho AE/AWB hội tụ rồi lấy 1 khung.
static void cam_task(void *arg) {
    uint32_t cnt = 0, t0 = millis(), next_wake = 0;
    for (;;) {
        if (g_stats.af_request) {
            g_stats.af_request = false;
            cam_standby(false);
            int best_vcm = cam_auto_focusSweep();
            Serial.printf("VCM Auto-Focus hoan tat: vcm_step = %d\n", best_vcm);
        }

        uint32_t now = millis();
        // Không bao giờ ngắt xung DVP khi đang có trình duyệt xem luồng stream (tránh mất hội tụ AE/AWB gây mờ ảnh)
        bool want_idle = (g_stats.stream_clients == 0) &&
            (g_stats.hot || (g_stats.eco_enabled && (now - det_last_person_ms() > THERMAL_IDLE_AFTER_MS)));
        g_stats.idle_mode = want_idle;

        if (want_idle) {
            if ((int32_t)(now - next_wake) < 0) {
                cam_standby(true);
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
            next_wake = now + THERMAL_IDLE_PERIOD_MS;
            cam_standby(false);
            for (int i = 0; i < THERMAL_WAKE_DISCARD; i++) {          // bỏ 6 khung đầu khi AE/AWB chưa hội tụ
                camera_fb_t *d = esp_camera_fb_get();
                if (d) esp_camera_fb_return(d);
            }
        } else {
            cam_standby(false);
        }

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) { vTaskDelay(1); continue; }
        if (fb->format == PIXFORMAT_JPEG && fb->len > 4 && fb->buf[0] == 0xFF && fb->buf[1] == 0xD8)
            hub_publish(fb->buf, fb->len, fb->width, fb->height);
        esp_camera_fb_return(fb);
        cnt++;
        if (now - t0 >= 1000) { g_stats.cam_fps = cnt * 1000.0f / (now - t0); cnt = 0; t0 = now; }
        if (want_idle) cam_standby(true);
    }
}

// ------------------------------------------------------------------ INA219 (tùy chọn)
#if INA219_ENABLE
static bool ina_read(uint8_t reg, int16_t *v) {
    Wire1.beginTransmission(INA219_ADDR); Wire1.write(reg);
    if (Wire1.endTransmission(false) != 0) return false;
    if (Wire1.requestFrom((uint8_t)INA219_ADDR, (uint8_t)2) != 2) return false;
    *v = (int16_t)((Wire1.read() << 8) | Wire1.read());
    return true;
}
#endif

static void wifi_start() {
#if WIFI_AP_MODE
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS, 6, 0, 4);
    Serial.printf("Wi-Fi AP \"%s\" -> http://%s\n", WIFI_AP_SSID, WiFi.softAPIP().toString().c_str());
#else
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);
    while (WiFi.status() != WL_CONNECTED) delay(200);
    Serial.printf("Wi-Fi -> http://%s\n", WiFi.localIP().toString().c_str());
#endif
    WiFi.setSleep(false);              // tránh trễ gói cho luồng MJPEG
    WiFi.setTxPower(WIFI_TX_POWER);    // cân bằng tốc độ truyền stream SVGA/HD và nhiệt độ bo
}

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("\n== AdaReMAC-EdgeVision v4.3 (Fine-Grid 48x48 FOMO + High-Clarity OV5640 Stream) ==");
    if (!psramFound()) Serial.println("CANH BAO: khong thay PSRAM - chon Tools > PSRAM > OPI PSRAM");
    if (!hub_init(FRAME_SLOT_BYTES)) Serial.println("LOI: khong cap phat duoc bo dem khung");
    if (!cam_init(STREAM_FRAMESIZE, STREAM_JPEG_QUALITY)) {
        Serial.println("LOI: khoi tao camera that bai (kiem tra cap FPC / chan)");
    } else {
        CamStatus cs = cam_status();
        Serial.printf("Camera PID 0x%04X | SVGA Q=%d Sharp=%d VCM=%d | 0x300E=0x%02X 0x302C=0x%02X | PLL1 ~%u MHz\n",
                      cs.pid, STREAM_JPEG_QUALITY, cs.sharpness, cs.vcm_step,
                      cs.reg_300e, cs.reg_302c, (unsigned)cs.vco_mhz);
    }
    g_stats.eco_enabled = (THERMAL_ECO_DEFAULT != 0);
    g_stats.jpeg_quality = STREAM_JPEG_QUALITY;
    wifi_start();
    det_init();
    Serial.printf("AI tren CPU: %s | Fine-Grid: %dx%d (%d o) | SRAM trong con %u B, PSRAM con %u B\n",
                  det_cpu_engine(), det_get_fine_grid(), det_get_fine_grid(),
                  det_get_fine_grid() * det_get_fine_grid(),
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL), (unsigned)ESP.getFreePsram());
    if (fpga_begin()) Serial.println("Tang Nano 20K: da ket noi (ID 0xF0) -> AI chay tren FPGA + Fine-Grid 48x48");
    else              Serial.println("Tang Nano 20K: khong thay -> AI chay tren ESP32-S3 + Fine-Grid 48x48");
#if INA219_ENABLE
    Wire1.begin(PIN_INA_SDA, PIN_INA_SCL, 400000);
#endif
    web_start();
    xTaskCreatePinnedToCore(cam_task, "cam", 6144, nullptr, 5, nullptr, 1);
    det_start();
    delay(100);
    Serial.printf("Sau khi khoi dong: SRAM trong con %u B (can > ~30 KB cho Wi-Fi/web)\n",
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
}

void loop() {
    static uint32_t last = 0;
    if (millis() - last < 2000) { delay(50); return; }
    last = millis();
    g_stats.chip_temp_c = temperatureRead();
    // trễ (hysteresis) 5 °C để không bật/tắt liên tục
    if (g_stats.chip_temp_c > THERMAL_HOT_C) g_stats.hot = true;
    else if (g_stats.chip_temp_c < THERMAL_HOT_C - 5.0f) g_stats.hot = false;
#if INA219_ENABLE
    int16_t sh, bus;
    if (ina_read(0x01, &sh) && ina_read(0x02, &bus)) {
        g_stats.ina_ma = sh * 0.01f / INA_SHUNT_OHMS;          // 10 uV/LSB
        g_stats.ina_v  = ((bus >> 3) * 4) / 1000.0f;           // 4 mV/LSB
    }
#endif
    DetResult r;
    det_get(&r);
    Serial.printf("cam %.1f fps | stream %.1f fps | AI %.1f fps [%s, luoi %ux%u] giai ma %.1f ms (%s) suy luan %.1f ms | %u doi tuong | %.1f C%s%s\n",
                  g_stats.cam_fps, g_stats.stream_fps, r.ai_fps,
                  r.backend_used == BK_FPGA ? "FPGA" : r.backend_used == BK_EI ? "EI" : det_cpu_engine(),
                  (unsigned)r.fine_grid_res, (unsigned)r.fine_grid_res,
                  r.decode_ms, det_decoder_name(), r.infer_ms, r.n, g_stats.chip_temp_c,
                  g_stats.idle_mode ? " | OV5640 nghi xen ke" : "", g_stats.hot ? " | CANH BAO NHIET" : "");
}
