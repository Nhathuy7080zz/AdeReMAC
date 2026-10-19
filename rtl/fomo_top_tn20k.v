// =============================================================================
// fomo_top_tn20k.v  -  Top cho Sipeed Tang Nano 20K: bo tang toc FOMO-Micro qua SPI
//
// Ket noi voi ESP32-S3 (giu nguyen chan nhu ban cu):
//   spi_sclk Pin 41 <- GPIO42 | spi_cs_n Pin 42 <- GPIO1 | spi_mosi Pin 48 <- GPIO2 | spi_miso Pin 49 -> GPIO41
// Xung 27 MHz tren board (Pin 4), khong dung PLL.
//
// Giao thuc SPI (mode 0, MSB truoc, SCLK den ~20 MHz). Byte n tren MISO = T[n-2]:
//   0x01 WRITE_FRAME : 0x01 + 9216 byte anh xam 96x96 (hang truoc). CS len -> tu dong chay.
//   0x02 READ_RESULT : 0x02 + 1 byte dem + 150 byte:
//                      [status, frame_cnt, cycles[7:0..31:24], logit[0..143] (int8, 12x12)]
//                      status bit0 = ket qua moi chua doc, bit1 = dang ban, bit2 = loi ghi khi ban
//   0x03 WRITE_REG   : 0x03, addr, data
//   0x04 READ_REG    : 0x04, addr, dem, data
// Thanh ghi: 0x00..0x05 mode AdaReMAC lop 1..6 (0 xap xi / 1 chon loc / 2 chinh xac, mac dinh 2)
//            0x06 thr (mac dinh 2) | 0x07 bit0 = dung bias bu Mode 0 (mac dinh 1)
//            0x10 ID = 0xF0 | 0x11 phien ban = 0x04
// LED (tich cuc muc thap): 0 ban | 1 co ket qua | 2 da thay SPI | 3 nhip tim | 4 loi | 5 mode lop 1 != 2
// =============================================================================
module fomo_top_tn20k (
    input  wire       sys_clk,
    input  wire       spi_sclk,
    input  wire       spi_cs_n,
    input  wire       spi_mosi,
    output wire       spi_miso,
    output wire [5:0] led_n
);
    localparam FRAME_BASE  = 15'd18432;
    localparam FRAME_BYTES = 16'd9216;
    localparam CMD_WRITE_FRAME = 8'h01, CMD_READ_RESULT = 8'h02, CMD_WRITE_REG = 8'h03, CMD_READ_REG = 8'h04;

    reg [7:0] por = 8'd0;
    wire rst_n = &por;
    always @(posedge sys_clk) if (!rst_n) por <= por + 8'd1;

    // ---------------------------------------------------------------- SPI
    wire        byte_ev, cs_end, cs_active;
    wire [7:0]  byte_data;
    wire [15:0] byte_idx;
    reg  [7:0]  tx_next;
    wire        miso_core;
    assign spi_miso = spi_cs_n ? 1'bz : miso_core;

    fomo_spi_if u_spi (
        .clk(sys_clk), .rst_n(rst_n),
        .sclk(spi_sclk), .cs_n(spi_cs_n), .mosi(spi_mosi), .miso(miso_core),
        .byte_ev(byte_ev), .byte_data(byte_data), .byte_idx(byte_idx),
        .cs_end(cs_end), .cs_active(cs_active), .tx_next(tx_next));

    // ---------------------------------------------------------------- thanh ghi
    reg [11:0] layer_mode;
    reg [1:0]  thr;
    reg        comp_en;
    reg        spi_seen, err_wr_busy, res_new;
    reg [7:0]  frame_cnt;
    reg [7:0]  cmd, reg_addr;
    reg [7:0]  rd_ptr;

    function [7:0] reg_rd(input [7:0] a);
        case (a)
            8'h00: reg_rd = {6'd0, layer_mode[1:0]};
            8'h01: reg_rd = {6'd0, layer_mode[3:2]};
            8'h02: reg_rd = {6'd0, layer_mode[5:4]};
            8'h03: reg_rd = {6'd0, layer_mode[7:6]};
            8'h04: reg_rd = {6'd0, layer_mode[9:8]};
            8'h05: reg_rd = {6'd0, layer_mode[11:10]};
            8'h06: reg_rd = {6'd0, thr};
            8'h07: reg_rd = {7'd0, comp_en};
            8'h10: reg_rd = 8'hF0;
            8'h11: reg_rd = 8'h04;
            default: reg_rd = 8'h00;
        endcase
    endfunction

    // ---------------------------------------------------------------- loi tang toc
    reg         acc_start;
    wire        acc_busy, acc_done;
    wire [31:0] acc_cycles;
    reg         ext_we;
    reg  [14:0] ext_addr;
    reg  [7:0]  ext_wdata;
    wire [7:0]  res_rdata;

    fomo_accel_core u_core (
        .clk(sys_clk), .rst_n(rst_n),
        .start(acc_start), .busy(acc_busy), .done(acc_done), .cycles(acc_cycles),
        .layer_mode(layer_mode), .thr(thr), .comp_en(comp_en),
        .ext_we(ext_we), .ext_addr(ext_addr), .ext_wdata(ext_wdata),
        .res_raddr(rd_ptr - 8'd6), .res_rdata(res_rdata));

    // byte thu k cua dong ket qua
    function [7:0] res_byte(input [7:0] k);
        case (k)
            8'd0: res_byte = {5'd0, err_wr_busy, acc_busy, res_new};
            8'd1: res_byte = frame_cnt;
            8'd2: res_byte = acc_cycles[7:0];
            8'd3: res_byte = acc_cycles[15:8];
            8'd4: res_byte = acc_cycles[23:16];
            8'd5: res_byte = acc_cycles[31:24];
            default: res_byte = (k < 8'd150) ? res_rdata : 8'h00;
        endcase
    endfunction

    reg [15:0] wr_cnt;
    always @(posedge sys_clk) begin
        acc_start <= 1'b0;
        ext_we    <= 1'b0;
        if (!rst_n) begin
            layer_mode <= 12'hAAA; thr <= 2'd2; comp_en <= 1'b1;
            spi_seen <= 1'b0; err_wr_busy <= 1'b0; res_new <= 1'b0; frame_cnt <= 8'd0;
            cmd <= 8'd0; tx_next <= 8'd0; rd_ptr <= 8'd0; wr_cnt <= 16'd0;
        end else begin
            if (acc_done) begin
                res_new   <= 1'b1;
                frame_cnt <= frame_cnt + 8'd1;
            end
            if (byte_ev) begin
                spi_seen <= 1'b1;
                if (byte_idx == 16'd0) begin
                    cmd     <= byte_data;
                    wr_cnt  <= 16'd0;
                    tx_next <= 8'h00;
                    if (byte_data == CMD_READ_RESULT) begin
                        rd_ptr  <= 8'd1;
                        tx_next <= res_byte(8'd0);
                    end
                end else begin
                    case (cmd)
                    CMD_WRITE_FRAME: begin
                        if (acc_busy) err_wr_busy <= 1'b1;
                        else if (wr_cnt < FRAME_BYTES) begin
                            ext_we    <= 1'b1;
                            ext_addr  <= FRAME_BASE + wr_cnt[14:0];
                            ext_wdata <= byte_data;
                        end
                        wr_cnt <= wr_cnt + 16'd1;
                    end
                    CMD_READ_RESULT: begin
                        tx_next <= res_byte(rd_ptr);
                        rd_ptr  <= rd_ptr + 8'd1;
                        if (rd_ptr == 8'd2) res_new <= 1'b0;   // da doc xong byte status
                    end
                    CMD_WRITE_REG: begin
                        if (byte_idx == 16'd1) reg_addr <= byte_data;
                        if (byte_idx == 16'd2) case (reg_addr)
                            8'h00: layer_mode[1:0]   <= byte_data[1:0];
                            8'h01: layer_mode[3:2]   <= byte_data[1:0];
                            8'h02: layer_mode[5:4]   <= byte_data[1:0];
                            8'h03: layer_mode[7:6]   <= byte_data[1:0];
                            8'h04: layer_mode[9:8]   <= byte_data[1:0];
                            8'h05: layer_mode[11:10] <= byte_data[1:0];
                            8'h06: thr               <= byte_data[1:0];
                            8'h07: comp_en           <= byte_data[0];
                            8'h08: err_wr_busy       <= 1'b0;          // xoa co loi
                            default: ;
                        endcase
                    end
                    CMD_READ_REG: if (byte_idx == 16'd1) tx_next <= reg_rd(byte_data);
                    default: ;
                    endcase
                end
            end
            if (cs_end && cmd == CMD_WRITE_FRAME && wr_cnt == FRAME_BYTES && !acc_busy)
                acc_start <= 1'b1;
            if (cs_end) cmd <= 8'd0;
        end
    end

    reg [24:0] hb = 25'd0;
    always @(posedge sys_clk) hb <= hb + 25'd1;
    assign led_n = ~{layer_mode[1:0] != 2'd2, err_wr_busy, hb[24], spi_seen, res_new, acc_busy};
endmodule
