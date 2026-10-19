// =============================================================================
// OV5640 trên ESP32-S3: Khởi tạo + Làm sắc nét luồng Stream (ISP CIP + VCM Focus)
//                       + Giảm nhiệt phần mềm an toàn.
//
// Phân tích nguyên nhân luồng stream bị mờ ở v4.2 và cách khắc phục ở v4.3:
// 1. Quang học (VCM Focus): OV5640 có thấu kính gắn cuộn dây VCM điều khiển qua DAC 10-bit
//    nội bộ (thanh ghi 0x3602, 0x3603). Mặc định khi khởi động DAC = 0 (lấy nét vô cực),
//    khiến vật thể trong phòng (0,3 m - 2,5 m) bị mất nét (out-of-focus). Hàm cam_set_vcm_focus()
//    và cam_auto_focusSweep() nạp trực tiếp dòng DAC VCM để khóa nét đúng cự ly thực tế.
// 2. Xử lý tín hiệu ảnh (ISP Clarity): Bật đầy đủ Lens Correction (LENC), Raw Gamma (RAW_GMA),
//    Khử điểm ảnh lỗi (BPC/WPC), Downsize Crop Window (DCW) và tăng cường biên cạnh phần cứng
//    CIP (thanh ghi 0x5308, 0x5302, 0x5303) giúp ảnh JPEG trong và sắc cạnh.
// 3. Nén & Tín hiệu DVP: Nâng mặc định lên SVGA (800x600) với jpeg_quality = 8 (giảm nhiễu
//    khối DCT 8x8), giữ dòng lái chân DVP ở mức 2x (0x302C = 0x42) và mở khóa hoàn toàn
//    bộ đếm khung DVP (0x4201 = 0x00, 0x4202 = 0x00).
// =============================================================================
#include "cam_ov5640.h"
#include "app_config.h"

static framesize_t s_fs = STREAM_FRAMESIZE;
static bool s_standby = false;
static int  s_sharpness = CAM_DEFAULT_SHARPNESS;
static int  s_contrast  = CAM_DEFAULT_CONTRAST;
static int  s_vcm_step  = CAM_DEFAULT_VCM_STEP;

static int rd(sensor_t *s, int r) { return s->get_reg(s, r, 0xFF); }

void cam_apply_cool_profile() {
    sensor_t *s = esp_camera_sensor_get();
    if (!s || s->id.PID != OV5640_PID) return;

    // Đảm bảo cảm biến ở chế độ hoạt động bình thường và mở khóa 100% khung hình DVP
    s->set_reg(s, 0x3008, 0xFF, 0x02);
    s->set_reg(s, 0x4201, 0xFF, 0x00);
    s->set_reg(s, 0x4202, 0xFF, 0x00);

#if THERMAL_PLL_IN_SPEC
    // Đưa VCO của PLL1 về dải chuẩn 500-1000 MHz (x90, sys_div 2 -> PLL1 = 900 MHz), giữ nguyên tốc độ PCLK/SYSCLK
    if (s_fs >= FRAMESIZE_QVGA && s_fs < FRAMESIZE_XGA) {
#if CAM_FPS_BOOST
        s->set_pll(s, 0, 61, 1, 0, 2, 2, 1, 4);
#else
        s->set_pll(s, 0, 90, 2, 0, 2, 2, 1, 4);
#endif
    } else if (s_fs >= FRAMESIZE_XGA) {
        s->set_pll(s, 0, 100, 2, 0, 2, 2, 1, 4);
    } else {
        s->set_pll(s, 0, 80, 2, 0, 2, 2, 1, 4);
    }
#endif

    // Điều chỉnh dòng lái chân I/O DVP (0x302C[7:6]):
    // 00 = 1x, 01 = 2x (0x42 - mặc định v4.3 giúp xung PCLK ở SVGA/HD vuông vắn, không vỡ nét), 11 = 4x (0xC3)
    int v = rd(s, 0x302C);
    if (v >= 0) {
#if THERMAL_PAD_DRIVE_1X
        s->set_reg(s, 0x302C, 0xFF, (v & 0x3F) | 0x00);
#else
        s->set_reg(s, 0x302C, 0xFF, (v & 0x3F) | 0x40);
#endif
    }
}

void cam_apply_isp_clarity(int sharpness, int contrast) {
    sensor_t *s = esp_camera_sensor_get();
    if (!s || s->id.PID != OV5640_PID) return;
    if (sharpness < 0) sharpness = 0;
    if (sharpness > 6) sharpness = 6;
    if (contrast < -2) contrast = -2;
    if (contrast > 2)  contrast = 2;
    s_sharpness = sharpness;
    s_contrast  = contrast;

    // 1. Kích hoạt đầy đủ các khối hiệu chỉnh quang học trên ISP của OV5640
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_wb_mode(s, 0);        // Auto White Balance
    s->set_exposure_ctrl(s, 1);  // Auto Exposure
    s->set_gain_ctrl(s, 1);      // Auto Gain
    s->set_gainceiling(s, (gainceiling_t)2); // Giới hạn khuếch đại vừa phải để giảm nhiễu hạt (grain noise)
    s->set_bpc(s, 1);            // Khử điểm ảnh chết (Black Pixel Cancellation)
    s->set_wpc(s, 1);            // Khử điểm ảnh cháy trắng (White Pixel Cancellation)
    s->set_raw_gma(s, 1);        // Hiệu chỉnh Gamma gốc giúp dải tương phản trung thực
    s->set_lenc(s, 1);           // Hiệu chỉnh tối 4 góc thấu kính (Lens Shading Correction)
    s->set_dcw(s, 1);            // Downsize Crop Window chuẩn xác
    s->set_contrast(s, contrast);
    s->set_brightness(s, 0);
    s->set_saturation(s, 0);

    // 2. Cấu hình khối CIP (Color Interpolation & Edge Sharpening: 0x5300..0x5308)
    s->set_sharpness(s, sharpness);
    if (sharpness == 0) {
        // Chế độ làm nét tự động nhẹ
        int r5308 = rd(s, 0x5308);
        if (r5308 >= 0) s->set_reg(s, 0x5308, 0xFF, r5308 & ~0x40);
    } else {
        // Chế độ làm nét cạnh tường minh (Manual Edge Enhancement):
        // 0x5308[6] = 1 (Manual sharpness), 0x5302 = cường độ làm nét biên, 0x5303 = ngưỡng lọc nhiễu nền (coring)
        int r5308 = rd(s, 0x5308);
        if (r5308 >= 0) s->set_reg(s, 0x5308, 0xFF, (r5308 | 0x40));
        uint8_t mt2 = (uint8_t)(sharpness * 7 + 4);   // 11..46: tăng độ sắc cạnh viền vật thể
        s->set_reg(s, 0x5302, 0xFF, mt2);
        s->set_reg(s, 0x5303, 0xFF, 0x06);            // chặn nhiễu hạt vùng phẳng
    }
}

void cam_set_sharpness(int sharpness) {
    cam_apply_isp_clarity(sharpness, s_contrast);
}

int cam_get_sharpness() { return s_sharpness; }

void cam_set_vcm_focus(int step_0_255) {
    if (step_0_255 < 0) step_0_255 = 0;
    if (step_0_255 > 255) step_0_255 = 255;
    s_vcm_step = step_0_255;
    sensor_t *s = esp_camera_sensor_get();
    if (!s || s->id.PID != OV5640_PID || s_standby) return;

    // 1. Ghi trực tiếp vào bộ điều khiển dòng DAC 10-bit của cuộn dây VCM nội bộ OV5640 (0x3602, 0x3603):
    //    dac10 = 0..1020. Theo OV5640 register map:
    //    0x3603[7:0] chứa 8 bit cao/trung, 0x3602[7:4] chứa các bit cấu hình/LSB.
    uint16_t dac10 = (uint16_t)(step_0_255 * 4);
    int r3602 = rd(s, 0x3602);
    if (r3602 < 0) r3602 = 0x00;
    s->set_reg(s, 0x3602, 0xFF, (uint8_t)((r3602 & 0x0F) | ((dac10 & 0x03) << 6) | 0x10));
    s->set_reg(s, 0x3603, 0xFF, (uint8_t)((dac10 >> 2) & 0xFF));

    // 2. Nếu vi điều khiển AF nội bộ (0x3022..0x3024) đang hoạt động, gửi thêm tọa độ lấy nét
    s->set_reg(s, 0x3024, 0xFF, (uint8_t)step_0_255);
}

int cam_get_vcm_focus() { return s_vcm_step; }

// Quét tự động tìm điểm lấy nét VCM có năng lượng tần số cao (kích thước tệp JPEG ở cùng khung cảnh) lớn nhất:
// Trong chuẩn nén JPEG, khung hình càng đúng nét (nhiều chi tiết tần số cao trên khối DCT 8x8)
// thì số lượng hệ số AC khác 0 càng lớn -> độ dài byte của khung JPEG (fb->len) đạt cực đại!
int cam_auto_focusSweep() {
    sensor_t *s = esp_camera_sensor_get();
    if (!s || s->id.PID != OV5640_PID || s_standby) return s_vcm_step;

    int best_step = s_vcm_step;
    size_t best_len = 0;

    // Bước 1: Quét thô 8 mốc từ 20 đến 230
    for (int st = 20; st <= 230; st += 30) {
        cam_set_vcm_focus(st);
        vTaskDelay(pdMS_TO_TICKS(35));
        // Bỏ 1 khung cũ trong bộ đệm
        camera_fb_t *d = esp_camera_fb_get();
        if (d) esp_camera_fb_return(d);
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            size_t len = fb->len;
            esp_camera_fb_return(fb);
            if (len > best_len) { best_len = len; best_step = st; }
        }
    }
    // Bước 2: Quét tinh quanh đỉnh tốt nhất (+/- 24, bước 8)
    int lo = best_step - 24 < 0 ? 0 : best_step - 24;
    int hi = best_step + 24 > 255 ? 255 : best_step + 24;
    for (int st = lo; st <= hi; st += 8) {
        cam_set_vcm_focus(st);
        vTaskDelay(pdMS_TO_TICKS(30));
        camera_fb_t *d = esp_camera_fb_get();
        if (d) esp_camera_fb_return(d);
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            size_t len = fb->len;
            esp_camera_fb_return(fb);
            if (len > best_len) { best_len = len; best_step = st; }
        }
    }
    cam_set_vcm_focus(best_step);
    return best_step;
}

bool cam_init(framesize_t fs, int jpeg_quality) {
    camera_config_t c = {};
    c.pin_pwdn = CAM_PIN_PWDN;   c.pin_reset = CAM_PIN_RESET;  c.pin_xclk = CAM_PIN_XCLK;
    c.pin_sccb_sda = CAM_PIN_SIOD; c.pin_sccb_scl = CAM_PIN_SIOC;
    c.pin_d7 = CAM_PIN_D7; c.pin_d6 = CAM_PIN_D6; c.pin_d5 = CAM_PIN_D5; c.pin_d4 = CAM_PIN_D4;
    c.pin_d3 = CAM_PIN_D3; c.pin_d2 = CAM_PIN_D2; c.pin_d1 = CAM_PIN_D1; c.pin_d0 = CAM_PIN_D0;
    c.pin_vsync = CAM_PIN_VSYNC; c.pin_href = CAM_PIN_HREF; c.pin_pclk = CAM_PIN_PCLK;
    c.xclk_freq_hz = CAM_XCLK_HZ;
    c.ledc_timer = LEDC_TIMER_0;  c.ledc_channel = LEDC_CHANNEL_0;
    c.pixel_format = PIXFORMAT_JPEG;
    c.frame_size = fs;
    c.jpeg_quality = jpeg_quality;
    c.fb_count = 3;
    c.fb_location = CAMERA_FB_IN_PSRAM;
    c.grab_mode = CAMERA_GRAB_LATEST;
    if (esp_camera_init(&c) != ESP_OK) return false;
    s_fs = fs;
    s_standby = false;
    sensor_t *s = esp_camera_sensor_get();
    if (s && s->id.PID == OV5640_PID) {
        s->set_vflip(s, 0);
        s->set_hmirror(s, 0);
    }
    cam_apply_cool_profile();
    cam_apply_isp_clarity(s_sharpness, s_contrast);
    cam_set_vcm_focus(s_vcm_step);
    return true;
}

bool cam_set_framesize(framesize_t fs) {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) return false;
    if (s->set_framesize(s, fs) != 0) return false;   // hàm này đặt lại PLL mặc định
    s_fs = fs;
    cam_apply_cool_profile();
    cam_apply_isp_clarity(s_sharpness, s_contrast);
    cam_set_vcm_focus(s_vcm_step);
    return true;
}

void cam_standby(bool enable) {
    sensor_t *s = esp_camera_sensor_get();
    if (!s || s->id.PID != OV5640_PID || enable == s_standby) return;
    // 0x3008 SYSTEM CTROL0: bit6 = software power down (driver Linux: 0x42 = ngủ, 0x02 = chạy)
    s->set_reg(s, 0x3008, 0xFF, enable ? 0x42 : 0x02);
    s_standby = enable;
    if (!enable) {
        // Khi thức dậy, khôi phục độ nét ISP và vị trí cuộn dây VCM
        cam_apply_isp_clarity(s_sharpness, s_contrast);
        cam_set_vcm_focus(s_vcm_step);
    }
}

CamStatus cam_status() {
    CamStatus st = {};
    sensor_t *s = esp_camera_sensor_get();
    st.ok = s != nullptr;
    st.standby = s_standby;
    st.framesize = s_fs;
    st.sharpness = s_sharpness;
    st.vcm_step  = s_vcm_step;
    if (!s) return st;
    st.pid = s->id.PID;
    if (s->id.PID == OV5640_PID && !s_standby) {
        st.reg_300e = rd(s, 0x300E);
        st.reg_302c = rd(s, 0x302C);
        st.reg_3036 = rd(s, 0x3036);
        st.reg_3035 = rd(s, 0x3035);
        st.reg_3037 = rd(s, 0x3037);
        static const float pre_map[9] = {1, 1, 2, 3, 4, 1.5f, 6, 2.5f, 8};
        int pre = st.reg_3037 & 0x0F;
        float refin = CAM_XCLK_HZ / 1e6f / pre_map[pre > 8 ? 8 : pre];
        st.vco_mhz = (uint32_t)(refin * st.reg_3036 / ((st.reg_3037 & 0x10) ? 2 : 1));
    }
    return st;
}
