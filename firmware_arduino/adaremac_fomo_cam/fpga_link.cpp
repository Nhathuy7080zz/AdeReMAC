#include "fpga_link.h"
#include "app_config.h"
#include <SPI.h>

// Giao thức (xem rtl/fomo_top_tn20k.v): byte thứ n trên MISO mang dữ liệu chuẩn bị sau byte n-2.
static SPIClass s_spi(FSPI);
static bool s_present = false;
static uint8_t s_zero[160];

static inline void cs_low()  { digitalWrite(PIN_FPGA_CS, LOW); }
static inline void cs_high() { digitalWrite(PIN_FPGA_CS, HIGH); delayMicroseconds(2); }

void fpga_write_reg(uint8_t addr, uint8_t val) {
    uint8_t tx[3] = {0x03, addr, val};
    s_spi.beginTransaction(SPISettings(FPGA_SPI_HZ, MSBFIRST, SPI_MODE0));
    cs_low(); s_spi.writeBytes(tx, 3); cs_high();
    s_spi.endTransaction();
}

uint8_t fpga_read_reg(uint8_t addr) {
    uint8_t tx[4] = {0x04, addr, 0, 0}, rx[4] = {0};
    s_spi.beginTransaction(SPISettings(FPGA_SPI_HZ, MSBFIRST, SPI_MODE0));
    cs_low(); s_spi.transferBytes(tx, rx, 4); cs_high();
    s_spi.endTransaction();
    return rx[3];
}

bool fpga_begin() {
#if FPGA_ENABLE
    pinMode(PIN_FPGA_CS, OUTPUT);
    digitalWrite(PIN_FPGA_CS, HIGH);
    s_spi.begin(PIN_FPGA_SCLK, PIN_FPGA_MISO, PIN_FPGA_MOSI, -1);
    delay(5);
    s_present = fpga_read_reg(0x10) == 0xF0;
#endif
    return s_present;
}

bool fpga_present() { return s_present; }

void fpga_send_frame(const uint8_t *gray96) {
    uint8_t cmd = 0x01;
    s_spi.beginTransaction(SPISettings(FPGA_SPI_HZ, MSBFIRST, SPI_MODE0));
    cs_low();
    s_spi.writeBytes(&cmd, 1);
    s_spi.writeBytes(gray96, 9216);
    cs_high();
    s_spi.endTransaction();
}

bool fpga_read_result(FpgaResult *r) {
    uint8_t buf[152];
    uint8_t tx0[2] = {0x02, 0x00};
    s_spi.beginTransaction(SPISettings(FPGA_SPI_HZ, MSBFIRST, SPI_MODE0));
    cs_low();
    s_spi.transferBytes(tx0, buf, 2);
    s_spi.transferBytes(s_zero, buf + 2, 150);
    cs_high();
    s_spi.endTransaction();
    const uint8_t *p = buf + 2;
    r->status = p[0];
    r->frame_cnt = p[1];
    r->cycles = p[2] | (p[3] << 8) | (p[4] << 16) | ((uint32_t)p[5] << 24);
    memcpy(r->logits, p + 6, 144);
    return true;
}

bool fpga_wait_result(FpgaResult *r, uint8_t after_cnt, uint32_t timeout_ms) {
    uint32_t t0 = millis();
    do {
        fpga_read_result(r);
        if (r->frame_cnt != after_cnt && !(r->status & 0x02)) return true;
        delayMicroseconds(500);
    } while (millis() - t0 < timeout_ms);
    return false;
}

void fpga_set_modes(uint8_t m) {
    for (uint8_t i = 0; i < 6; i++) fpga_write_reg(i, m & 3);
}
