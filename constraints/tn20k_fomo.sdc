// Xung he thong 27 MHz (Pin 4)
create_clock -name sys_clk -period 37.037 -waveform {0 18.518} [get_ports {sys_clk}]
// SCLK tu ESP32-S3 (20 MHz). Hai mien xung khong dong bo: du lieu qua mien bang co toggle 3 tang FF.
create_clock -name spi_sclk -period 50 -waveform {0 25} [get_ports {spi_sclk}]
set_clock_groups -asynchronous -group [get_clocks {sys_clk}] -group [get_clocks {spi_sclk}]
