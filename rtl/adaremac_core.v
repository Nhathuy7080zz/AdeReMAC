// AdaReMAC core: SPI register file + MAC array + accuracy/activity counters.
// Da dat mac dinh N_LANES = 1 de ha nhiet do chip FPGA tren Tang Nano 20K.
module adaremac_core #(
    parameter N_LANES = 1
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       spi_sclk,
    input  wire       spi_cs_n,
    input  wire       spi_mosi,
    output wire       spi_miso,
    output wire [1:0] o_mode,
    output wire       o_run,
    output reg        o_spi_seen,
    output wire       o_wake_esp
);
    localparam [7:0] LANES_B = N_LANES;

    reg [1:0] mode, thr;
    reg       run, snap, clr_stats;
    reg [7:0] wake_thr;
    assign o_mode = mode;
    assign o_run  = run;

    // ---------------- SPI ----------------
    wire       wr_en;
    wire [6:0] wr_addr, rd_addr;
    wire [7:0] wr_data;
    reg  [7:0] rd_data;

    spi_slave u_spi (
        .clk(clk), .rst_n(rst_n),
        .sclk(spi_sclk), .cs_n(spi_cs_n), .mosi(spi_mosi), .miso(spi_miso),
        .wr_en(wr_en), .wr_addr(wr_addr), .wr_data(wr_data),
        .rd_addr(rd_addr), .rd_data(rd_data));

    always @(posedge clk) begin
        snap      <= 1'b0;
        clr_stats <= 1'b0;
        if (!rst_n) begin
            mode <= 2'd0; thr <= 2'd2; run <= 1'b0; o_spi_seen <= 1'b0;
            wake_thr <= 8'h80;
        end else if (wr_en) begin
            o_spi_seen <= 1'b1;
            case (wr_addr)
                7'h00: mode <= wr_data[1:0];
                7'h01: thr  <= wr_data[1:0];
                7'h02: begin run <= wr_data[0]; snap <= wr_data[1]; clr_stats <= wr_data[2]; end
                7'h05: wake_thr <= wr_data;
                default: ;
            endcase
        end
    end

    // ---------------- MAC array + monitor ----------------
    wire        mon_valid, mon_enr;
    wire [15:0] mon_err;
    wire [7:0]  chk;

    mac_array #(.N_LANES(N_LANES)) u_arr (
        .clk(clk), .rst_n(rst_n), .run(run), .mode(mode), .thr(thr),
        .mon_valid(mon_valid), .mon_enr(mon_enr), .mon_err(mon_err), .chk(chk));

    reg [31:0] ops, enr_cnt;
    reg [39:0] err_sum;
    reg [31:0] ops_s, enr_s;
    reg [39:0] err_s;

    always @(posedge clk) begin
        if (!rst_n || clr_stats) begin
            ops <= 32'd0; enr_cnt <= 32'd0; err_sum <= 40'd0;
        end else if (mon_valid) begin
            ops     <= ops + 32'd1;
            enr_cnt <= enr_cnt + {31'd0, mon_enr};
            err_sum <= err_sum + {24'd0, mon_err};
        end
        if (snap) begin
            ops_s <= ops; enr_s <= enr_cnt; err_s <= err_sum;
        end
    end

    assign o_wake_esp = run & (chk >= wake_thr);

    // ---------------- read mux ----------------
    always @* begin
        case (rd_addr)
            7'h00: rd_data = {6'b0, mode};
            7'h01: rd_data = {6'b0, thr};
            7'h02: rd_data = {7'b0, run};
            7'h03: rd_data = 8'hA5;
            7'h04: rd_data = LANES_B;
            7'h05: rd_data = wake_thr;
            7'h10: rd_data = ops_s[7:0];
            7'h11: rd_data = ops_s[15:8];
            7'h12: rd_data = ops_s[23:16];
            7'h13: rd_data = ops_s[31:24];
            7'h14: rd_data = enr_s[7:0];
            7'h15: rd_data = enr_s[15:8];
            7'h16: rd_data = enr_s[23:16];
            7'h17: rd_data = enr_s[31:24];
            7'h18: rd_data = err_s[7:0];
            7'h19: rd_data = err_s[15:8];
            7'h1A: rd_data = err_s[23:16];
            7'h1B: rd_data = err_s[31:24];
            7'h1C: rd_data = err_s[39:32];
            7'h20: rd_data = chk;
            default: rd_data = 8'h00;
        endcase
    end
endmodule
