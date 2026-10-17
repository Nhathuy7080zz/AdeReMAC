// Tang Nano 20K, standalone (no ESP32, no SPI): nhan S2 de doi cau hinh, ket qua
// (so phep tinh, so lan residual bat, tong sai so) in ra UART moi ~1 giay qua
// chinh cap USB-C dang cam (BL616 lam cau USB<->serial), 115200 8N1.
//
// Da cap nhat chong nong chip & chong ket Reset:
//   1. N_LANES = 1 (chi chay 1 lane co bo giam sat sai so, giam 95% nhiet do).
//   2. rst_n = por_done (khong phu thuoc chan 88 MODE0 de tranh ket Reset).
module top_standalone #(
    parameter N_LANES             = 1,
    parameter SNAPSHOT_CYCLES     = 27_000_000, // ~1s @ 27 MHz; testbench overrides this
    parameter BTN_DEBOUNCE_CYCLES = 270000      // ~10ms @ 27 MHz; testbench overrides this
) (
    input  wire        sys_clk,     // 27 MHz onboard (Pin 4)
    input  wire        btn_rst,     // S1, pin 88 tren board
    input  wire        btn_mode,    // S2, pin 87 tren board (doi cau hinh)
    output wire        uart_tx_pin, // Pin 69 -> BL616 USB-UART
    output wire [5:0]  led_n        // Pin 15..20
);
    // Power-on reset tu dong (khong phu thuoc btn_rst de tranh treo khi chua bat Dual-Purpose Pin)
    reg [7:0] por = 8'd0;
    wire      por_done = &por;
    always @(posedge sys_clk) if (!por_done) por <= por + 8'd1;
    wire rst_n = por_done;

    // ---- nut S2: doi cau hinh 0..4 ----
    wire mode_pulse;
    btn_sync #(.DEBOUNCE_CYCLES(BTN_DEBOUNCE_CYCLES)) u_btn (
        .clk(sys_clk), .rst_n(rst_n), .btn_raw(btn_mode), .pulse(mode_pulse));

    reg [2:0] cfg_idx;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) cfg_idx <= 3'd0;
        else if (mode_pulse) cfg_idx <= (cfg_idx == 3'd4) ? 3'd0 : cfg_idx + 3'd1;
    end

    reg [1:0] mode, thr;
    always @* begin
        case (cfg_idx)
            3'd0: begin mode = 2'd0; thr = 2'd0; end
            3'd1: begin mode = 2'd1; thr = 2'd3; end
            3'd2: begin mode = 2'd1; thr = 2'd2; end
            3'd3: begin mode = 2'd1; thr = 2'd1; end
            default: begin mode = 2'd2; thr = 2'd0; end // 3'd4
        endcase
    end

    // ---- mang MAC (lane 0 mang monitor: nhan cung, so sanh voi exact) ----
    wire        mon_valid, mon_enr;
    wire [15:0] mon_err;
    wire [7:0]  chk;
    mac_array #(.N_LANES(N_LANES)) u_arr (
        .clk(sys_clk), .rst_n(rst_n), .run(1'b1), .mode(mode), .thr(thr),
        .mon_valid(mon_valid), .mon_enr(mon_enr), .mon_err(mon_err), .chk(chk));

    // ---- bo dem, snapshot moi ~1s hoac ngay khi doi cau hinh ----
    reg [31:0] ops, enr_cnt;
    reg [39:0] err_sum;
    reg [31:0] ops_win, enr_win;
    reg [39:0] err_win;
    reg [2:0]  win_cfg;
    reg        report_go;

    reg [24:0] tick;
    wire       tick_done = (tick == SNAPSHOT_CYCLES - 1);
    wire       window_end = tick_done | mode_pulse;

    reg [2:0] settle;
    wire      counting = (settle == 3'd0);

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            tick <= 25'd0; ops <= 32'd0; enr_cnt <= 32'd0; err_sum <= 40'd0;
            ops_win <= 32'd0; enr_win <= 32'd0; err_win <= 40'd0; win_cfg <= 3'd0;
            report_go <= 1'b0; settle <= 3'd4;
        end else begin
            report_go <= 1'b0;
            if (settle != 3'd0) settle <= settle - 3'd1;
            if (mon_valid && counting) begin
                ops     <= ops + 32'd1;
                enr_cnt <= enr_cnt + {31'd0, mon_enr};
                err_sum <= err_sum + {24'd0, mon_err};
            end
            if (window_end) begin
                tick      <= 25'd0;
                ops_win   <= ops;      enr_win <= enr_cnt;   err_win <= err_sum;
                win_cfg   <= cfg_idx;
                ops       <= 32'd0;    enr_cnt <= 32'd0;     err_sum <= 40'd0;
                report_go <= 1'b1;
                settle    <= 3'd4;
            end else begin
                tick <= tick + 25'd1;
            end
        end
    end

    wire uart_busy;
    report_tx u_rpt (
        .clk(sys_clk), .rst_n(rst_n), .send(report_go),
        .state_in(win_cfg), .ops_in(ops_win), .enr_in(enr_win), .err_in(err_win),
        .busy(uart_busy), .uart_pin(uart_tx_pin));

    reg [24:0] hb = 25'd0;
    always @(posedge sys_clk) hb <= hb + 25'd1;
    assign led_n = ~{^chk, uart_busy, hb[24], cfg_idx};
endmodule
