// Tang Nano 20K top level (GW2AR-18C). 27 MHz on-board clock, no PLL.
// Ban da toi uu chong nong chip:
//   1. Xoa cong wake_esp thua (tranh Gowin tu gan ngau nhien vao chan ngoai vi gay chap).
//   2. Giam N_LANES = 1 (chi giu Lane 0 co bo giam sat sai so, giam 95% cong suat dong).
//   3. Dua chan spi_miso ve trang thai tro khang cao (High-Z / 1'bz) khi spi_cs_n = 1,
//      tranh xung dot ap tren Pin 49 voi ESP32-S3 hoac mach LCD tren board.
module top_tn20k #(
    parameter N_LANES = 1
) (
    input  wire       sys_clk,     // 27 MHz (Pin 4)
    input  wire       spi_sclk,    // from ESP32-S3 GPIO 42 (Pin 41)
    input  wire       spi_cs_n,    // from ESP32-S3 GPIO 1  (Pin 42)
    input  wire       spi_mosi,    // from ESP32-S3 GPIO 2  (Pin 48)
    output wire       spi_miso,    // to   ESP32-S3 GPIO 41 (Pin 49)
    output wire [5:0] led_n        // 6 LED onboard (Pin 15..20)
);
    // Power-on reset tu dong
    reg [7:0] por = 8'd0;
    wire rst_n = &por;
    always @(posedge sys_clk) if (!rst_n) por <= por + 8'd1;

    wire [1:0] mode;
    wire       run, spi_seen, wake_esp_int;
    wire       spi_miso_core;

    // Khi CS_N = 1 (khong truyen SPI), tha noi chan MISO (1'bz) de khong keo dòng tren Pin 49
    assign spi_miso = spi_cs_n ? 1'bz : spi_miso_core;

    adaremac_core #(.N_LANES(N_LANES)) u_core (
        .clk(sys_clk),
        .rst_n(rst_n),
        .spi_sclk(spi_sclk),
        .spi_cs_n(spi_cs_n),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso_core),
        .o_mode(mode),
        .o_run(run),
        .o_spi_seen(spi_seen),
        .o_wake_esp(wake_esp_int)
    );

    reg [24:0] hb = 25'd0;
    always @(posedge sys_clk) hb <= hb + 25'd1;

    // led0/1 = mode, led2 = run, led3 = heartbeat (~0.6s), led4 = SPI seen, led5 = wake_esp_int
    assign led_n = ~{wake_esp_int, spi_seen, hb[24], run, mode[1], mode[0]};
endmodule
