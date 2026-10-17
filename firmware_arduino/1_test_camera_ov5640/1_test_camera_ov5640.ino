// =========================================================================================
// SKETCH 1: KIỂM TRA ĐỘC LẬP CAMERA OV5640 + PSRAM + NHIỆT ĐỘ CHIP TRÊN ARDUINO IDE
// Board: ESP32 Cam Kit Phát Triển ESP32-S3 N16R8 OV5640 TypeC
//
// CÁCH CHẠY TRÊN ARDUINO IDE 2.x (Không cần cài thêm bất kỳ thư viện ngoài nào):
//   1. Mở file 1_test_camera_ov5640.ino này bằng Arduino IDE.
//   2. Vào menu Tools, chọn đúng:
//        - Board            : "ESP32S3 Dev Module"
//        - Flash Size       : "16MB (128Mb)"
//        - Partition Scheme : "Huge APP (3MB No OTA/1MB SPIFFS)"
//        - PSRAM            : "OPI PSRAM"   <-- BẮT BUỘC CHỌN OPI PSRAM!
//        - USB CDC On Boot  : "Enabled" (nếu cắm cổng USB) / "Disabled" (nếu cắm cổng UART)
//   3. Bấm nút Upload (->), mở Serial Monitor ở tốc độ 115200 baud.
//   4. (Tùy chọn) Nếu muốn xem video trực tiếp trên trình duyệt Web, điền tên Wi-Fi vào
//      WIFI_SSID và WIFI_PASS ở dòng 30-31 bên dưới! Để trống "" nếu chỉ muốn test qua Serial.
// =========================================================================================

#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"

// Điền tên Wi-Fi và mật khẩu nếu muốn xem Live Stream trên trình duyệt (để trống "" nếu không cần):
const char *WIFI_SSID = "";
const char *WIFI_PASS = "";

// ---------------- SƠ ĐỒ 14 CHÂN CỐ ĐỊNH KHE FPC 24-PIN TRÊN MẠCH ESP32-S3 N16R8 CAM ------
// LƯU Ý: Toàn bộ 14 chân này nằm ở Hàng Trái (Left Row). Tuyệt đối không cắm dây ngoài vào đây!
#define CAM_PIN_PWDN    -1
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK    15
#define CAM_PIN_SIOD     4
#define CAM_PIN_SIOC     5
#define CAM_PIN_D7      16
#define CAM_PIN_D6      17
#define CAM_PIN_D5      18
#define CAM_PIN_D4      12
#define CAM_PIN_D3      10
#define CAM_PIN_D2       8
#define CAM_PIN_D1       9
#define CAM_PIN_D0      11
#define CAM_PIN_VSYNC    6
#define CAM_PIN_HREF     7
#define CAM_PIN_PCLK    13

static bool s_cam_ok = false;
static bool s_wifi_ok = false;
static uint32_t s_frame_idx = 0;

static const char *get_sensor_name(int pid) {
    switch (pid) {
        case OV5640_PID: return "OV5640 (5MP - max 2592x1944 QSXGA)";
        case OV3660_PID: return "OV3660 (3MP - max 2048x1536 QXGA)";
        case OV2640_PID: return "OV2640 (2MP - max 1600x1200 UXGA)";
        default:         return "Unknown Sensor";
    }
}

// Handler chụp 1 ảnh JPEG khi người dùng mở http://<IP>/ trên trình duyệt
static esp_err_t capture_handler(httpd_req_t *req) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=ov5640.jpg");
    esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return res;
}

static void start_web_server() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t uri_cap = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = capture_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &uri_cap);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1500);

    Serial.println("\n============================================================");
    Serial.println("  [ARDUINO IDE] KIEM TRA CAMERA OV5640 & NHIET DO ESP32-S3");
    Serial.println("============================================================");

    // 1. Kiểm tra nhiệt độ nội bộ của chip ESP32-S3
    float t_boot = temperatureRead();
    Serial.printf("[1/4] Nhiet do noi bo ESP32-S3 : %.1f C %s\n",
                  t_boot, (t_boot > 70.0f) ? "[CANH BAO: CHIP NONG BAT THUONG!]" : "[BINH THUONG]");

    // 2. Kiểm tra bộ nhớ 8MB OPI PSRAM
    bool has_psram = psramFound();
    Serial.printf("[2/4] Bo nho OPI PSRAM         : %u KB trong / %u KB tong (DRAM trong: %u KB)\n",
                  (unsigned)(ESP.getFreePsram() / 1024),
                  (unsigned)(ESP.getPsramSize() / 1024),
                  (unsigned)(ESP.getFreeHeap() / 1024));

    if (!has_psram) {
        Serial.println("  -> [CANH BAO] Chua nhan OPI PSRAM! Vao menu Tools -> PSRAM -> Chon 'OPI PSRAM'.");
    }

    // 3. Khởi tạo Camera OV5640
    camera_config_t config = {};
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0       = CAM_PIN_D0;
    config.pin_d1       = CAM_PIN_D1;
    config.pin_d2       = CAM_PIN_D2;
    config.pin_d3       = CAM_PIN_D3;
    config.pin_d4       = CAM_PIN_D4;
    config.pin_d5       = CAM_PIN_D5;
    config.pin_d6       = CAM_PIN_D6;
    config.pin_d7       = CAM_PIN_D7;
    config.pin_xclk     = CAM_PIN_XCLK;
    config.pin_pclk     = CAM_PIN_PCLK;
    config.pin_vsync    = CAM_PIN_VSYNC;
    config.pin_href     = CAM_PIN_HREF;
    config.pin_sccb_sda = CAM_PIN_SIOD;
    config.pin_sccb_scl = CAM_PIN_SIOC;
    config.pin_pwdn     = CAM_PIN_PWDN;
    config.pin_reset    = CAM_PIN_RESET;
    config.xclk_freq_hz = 20000000;

    // Nếu dùng Wi-Fi Web Server thì chọn JPEG để xem trên trình duyệt; ngược lại dùng RGB565 để đo độ sáng
    bool want_wifi = (strlen(WIFI_SSID) > 0);
    config.pixel_format = want_wifi ? PIXFORMAT_JPEG : PIXFORMAT_RGB565;
    config.frame_size   = has_psram ? (want_wifi ? FRAMESIZE_VGA : FRAMESIZE_QVGA) : FRAMESIZE_QQVGA;
    config.jpeg_quality = 12;
    config.fb_count     = has_psram ? 2 : 1;
    config.fb_location  = has_psram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
    config.grab_mode    = CAMERA_GRAB_LATEST;

    Serial.println("[3/4] Dang khoi tao Camera OV5640 (XCLK=GPIO15, SIOD=GPIO4, SIOC=GPIO5)...");
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("  -> [LOI] esp_camera_init that bai (ma loi = 0x%x)\n", err);
        Serial.println("  -> Kiem tra: (1) Rut dien truoc khi cam cap det FPC chua?");
        Serial.println("               (2) Mat la dong vang cua cap det da up xuong mat bo mach PCB chua?");
        s_cam_ok = false;
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    Serial.printf("[4/4] Nhan dien thanh cong     : %s (PID = 0x%04X)\n",
                  s ? get_sensor_name(s->id.PID) : "Unknown", s ? s->id.PID : 0);
    Serial.println("============================================================\n");
    s_cam_ok = true;

    // 4. (Tùy chọn) Kết nối Wi-Fi để xem ảnh trên trình duyệt nếu người dùng đã điền WIFI_SSID
    if (want_wifi) {
        Serial.printf("Dang ket noi Wi-Fi '%s'", WIFI_SSID);
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        int retry = 0;
        while (WiFi.status() != WL_CONNECTED && retry < 20) {
            delay(500);
            Serial.print(".");
            retry++;
        }
        Serial.println();
        if (WiFi.status() == WL_CONNECTED) {
            s_wifi_ok = true;
            start_web_server();
            Serial.print("-> Mo trinh duyet Web truy cap dia chi: http://");
            Serial.println(WiFi.localIP());
        }
    }
}

void loop() {
    float temp_c = temperatureRead();
    if (!s_cam_ok) {
        Serial.printf("[Chua nhan Camera] Nhiet do chip ESP32-S3: %.1f C\n", temp_c);
        delay(1000);
        return;
    }

    uint32_t t0 = micros();
    camera_fb_t *fb = esp_camera_fb_get();
    uint32_t dt_us = micros() - t0;

    if (!fb) {
        Serial.println("[LOI] Khong lay duoc Framebuffer!");
        delay(1000);
        return;
    }

    // Tính giá trị trung bình các byte ảnh (thử che tay trước ống kính sẽ thấy giá trị thay đổi ngay)
    uint64_t sum = 0;
    size_t step = (fb->len > 1024) ? (fb->len / 1024) : 1;
    size_t cnt = 0;
    for (size_t i = 0; i < fb->len; i += step) {
        sum += fb->buf[i];
        cnt++;
    }
    uint32_t avg_val = cnt ? (uint32_t)(sum / cnt) : 0;

    Serial.printf("[Frame #%04lu] OK | %ux%u | %u bytes | Capture: %.2f ms | Do sang TB: %3lu/255 | Temp: %.1f C\n",
                  (unsigned long)(++s_frame_idx),
                  (unsigned)fb->width,
                  (unsigned)fb->height,
                  (unsigned)fb->len,
                  dt_us / 1000.0f,
                  (unsigned long)avg_val,
                  temp_c);

    if (temp_c > 75.0f) {
        Serial.println("  [CANH BAO NHIET DO] Chip ESP32-S3 vuot 75 C! Kiem tra lai day cam ngoai vi.");
    }

    esp_camera_fb_return(fb);
    delay(1000);
}
