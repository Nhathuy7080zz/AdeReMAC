// =============================================================================
// fomo_accel_core.v  -  Bo tang toc FOMO-Micro INT8 tren Tang Nano 20K (GW2AR-18)
//
// Chay tuan tu cac lop conv k x k (stride 1/2, padding k/2) cua FOMO-Micro.
// Moi chu ky: doc 1 kich hoat (phat cho ca 16 lan) + 1 tu trong so 16 x int8,
// 16 lan MAC AdaReMAC tinh 16 kenh ra song song. Het 1 diem anh -> requant tuan tu
// tung kenh:  y = clamp(((acc + B) * M + 2^(SH-1)) >>> SH)  -> ghi lai RAM kich hoat.
// Dac ta so hoc khop tung bit voi ai_fomo/fomo_int8.py va fomo_micro.c (ESP32-S3).
//
// Bo nho (BSRAM):
//   act_mem : 32K x 8   anh vao (9216 B) + 2 vung kich hoat ping-pong
//   wmem    : 2K x 128  trong so, moi tu = 16 lan x int8
//   bias/comp/M : bang theo kenh (<= 128 muc)
//   res_mem : 144 x int8 logit "person" 12x12 (lop cuoi)
// =============================================================================
`ifndef FOMO_MEM_DIR
`define FOMO_MEM_DIR ""
`endif

module fomo_accel_core #(
    parameter LANES  = 16,
    parameter ACT_AW = 15
) (
    input  wire              clk,
    input  wire              rst_n,
    input  wire              start,
    output reg               busy,
    output reg               done,
    output reg  [31:0]       cycles,
    input  wire [11:0]       layer_mode,   // 2 bit / lop, lop 0 o bit [1:0]
    input  wire [1:0]        thr,
    input  wire              comp_en,      // dung bias da bu khi lop chay Mode 0
    // cong ghi anh tu SPI (chi dung khi !busy)
    input  wire              ext_we,
    input  wire [ACT_AW-1:0] ext_addr,
    input  wire [7:0]        ext_wdata,
    // doc ket qua
    input  wire [7:0]        res_raddr,
    output wire [7:0]        res_rdata
);
    // ------------------------------------------------------------------ cau hinh lop
    reg  [2:0]  li;
    reg  [5:0]  L_CIN, L_COUT;
    reg  [1:0]  L_K, L_S;
    reg  [6:0]  L_HI, L_WI, L_HO, L_WO;
    reg  [14:0] L_HIWI;
    reg  [12:0] L_HOWO;
    reg         L_RELU, L_LAST;
    reg  [4:0]  L_SH;
    reg  [10:0] L_WBASE;
    reg  [6:0]  L_CBASE;
    reg  [14:0] L_INB, L_OUTB;
    reg  [1:0]  L_NG;
    reg  [8:0]  L_KT;
`include "fomo_model.vh"

    // ------------------------------------------------------------------ bo nho
    reg [7:0]           act_mem [0:(1<<ACT_AW)-1];
    reg [LANES*8-1:0]   wmem    [0:WMEM_DEPTH-1];
    reg [31:0]          bias_mem[0:CMEM_DEPTH-1];
    reg [31:0]          comp_mem[0:CMEM_DEPTH-1];
    reg [15:0]          m_mem   [0:CMEM_DEPTH-1];
    reg [7:0]           res_mem [0:143];
    initial begin
        $readmemh({`FOMO_MEM_DIR, "fomo_w.hex"},    wmem);
        $readmemh({`FOMO_MEM_DIR, "fomo_bias.hex"}, bias_mem);
        $readmemh({`FOMO_MEM_DIR, "fomo_comp.hex"}, comp_mem);
        $readmemh({`FOMO_MEM_DIR, "fomo_m.hex"},    m_mem);
    end
    assign res_rdata = res_mem[res_raddr];

    // ------------------------------------------------------------------ FSM
    localparam S_IDLE = 3'd0, S_PIX = 3'd1, S_MAC = 3'd2, S_DRAIN = 3'd3, S_RQ = 3'd4, S_NEXT = 3'd5;
    reg [2:0]  st;
    reg [1:0]  cog;
    reg [6:0]  oy, ox;
    reg [5:0]  ci;
    reg [1:0]  ky, kx;
    reg [8:0]  kidx;
    reg [14:0] ci_off;          // ci * HI * WI
    reg [2:0]  dcnt;
    reg [5:0]  rq_cnt;
    reg [31:0] cyc_cnt;

    wire [1:0] lmode = layer_mode[{li, 1'b0} +: 2];
    wire [5:0] ncout = (L_COUT > {cog, 4'd0} + 6'd16) ? 6'd16 : (L_COUT - {cog, 4'd0});

    // ---- sinh dia chi tap (to hop tu bo dem) ----
    wire [1:0]        pad = L_K >> 1;
    wire signed [9:0] iy  = $signed({3'b0, oy}) * $signed({8'b0, L_S}) + $signed({8'b0, ky}) - $signed({8'b0, pad});
    wire signed [9:0] ix  = $signed({3'b0, ox}) * $signed({8'b0, L_S}) + $signed({8'b0, kx}) - $signed({8'b0, pad});
    wire              oob = iy < 0 || ix < 0 || iy >= $signed({3'b0, L_HI}) || ix >= $signed({3'b0, L_WI});
    wire [14:0]       tap_addr = L_INB + ci_off + iy[6:0] * L_WI + ix[6:0];
    wire [10:0]       w_addr   = L_WBASE + (cog[0] ? L_KT : 9'd0) + kidx;
    wire              last_tap = (ci == L_CIN - 6'd1) && (ky == L_K - 2'd1) && (kx == L_K - 2'd1);
    wire              tap_v    = (st == S_MAC);
    // tang 0: chot dia chi (cat duong to hop dai: nhan dia chi -> BSRAM)
    reg  [14:0]       a_addr_r;
    reg  [10:0]       w_addr_r;
    reg               v0, oob0;
    always @(posedge clk) begin
        a_addr_r <= tap_addr;
        w_addr_r <= w_addr;
        v0       <= tap_v;
        oob0     <= oob;
    end

    // ---- requant: 3 tang ----
    reg               r1_v, r2_v;
    reg  [5:0]        r1_co, r2_co;
    reg signed [32:0] r1_sum;
    reg  [15:0]       r1_m;
    reg signed [49:0] r2_prod;
    wire signed [31:0] acc_lane [0:LANES-1];
    wire [5:0]  rq_j   = rq_cnt;
    wire [5:0]  rq_co  = {cog, 4'd0} + rq_j;
    wire [6:0]  cidx   = L_CBASE + rq_co;
    wire signed [49:0] r3_rnd = r2_prod + ($signed(50'd1) <<< (L_SH - 5'd1));
    wire signed [49:0] r3_sh  = r3_rnd >>> L_SH;
    wire [7:0]  r3_y_c = L_RELU ? ((r3_sh < 0) ? 8'd0 : (r3_sh > 255) ? 8'd255 : r3_sh[7:0])
                                : ((r3_sh < -128) ? 8'h80 : (r3_sh > 127) ? 8'h7F : r3_sh[7:0]);
    // tang 4: chot gia tri + dia chi ghi
    reg  [7:0]  r3_y;
    reg  [14:0] r3_addr;
    reg         r3_v;
    always @(posedge clk) begin
        r3_y    <= r3_y_c;
        r3_addr <= L_OUTB + r2_co * L_HOWO + oy * L_WO + ox;
        r3_v    <= r2_v;
    end
    wire        r3_we   = r3_v && !L_LAST;

    // ---- cong RAM kich hoat (1 cong) ----
    wire [ACT_AW-1:0] act_addr  = !busy ? ext_addr : (r3_we ? r3_addr : a_addr_r);
    wire              act_we    = !busy ? ext_we   : r3_we;
    wire [7:0]        act_wdata = !busy ? ext_wdata : r3_y;
    reg  [7:0]        act_rdata;
    always @(posedge clk) begin
        if (act_we) act_mem[act_addr] <= act_wdata;
        act_rdata <= act_mem[act_addr];
    end

    reg [LANES*8-1:0] w_rdata;
    always @(posedge clk) w_rdata <= wmem[w_addr_r];

    always @(posedge clk) if (r3_v && L_LAST) res_mem[oy * L_WO + ox] <= r3_y;

    // ---- duong ong MAC ----
    reg v1, v2, oob1, clr_acc;
    always @(posedge clk) begin
        v1   <= v0;            // du lieu BSRAM hop le
        oob1 <= oob0;
        v2   <= v1;            // tich trong pm_r hop le
    end
    wire [7:0] x_in = oob1 ? 8'd0 : act_rdata;

    genvar g;
    generate
        for (g = 0; g < LANES; g = g + 1) begin : lane
            fomo_mac_lane u_lane (
                .clk(clk), .clr(clr_acc), .mul_en(v1), .acc_en(v2),
                .x(x_in), .w_sm(w_rdata[g*8 +: 8]), .mode(lmode), .thr(thr),
                .acc(acc_lane[g]));
        end
    endgenerate

    // ---- FSM chinh ----
    always @(posedge clk) begin
        done    <= 1'b0;
        clr_acc <= 1'b0;
        r1_v    <= 1'b0;
        if (!rst_n) begin
            st <= S_IDLE; busy <= 1'b0; cycles <= 32'd0; r2_v <= 1'b0;
            li <= 3'd0; cog <= 2'd0; oy <= 7'd0; ox <= 7'd0;
        end else begin
            // tang requant 2
            r2_v    <= r1_v;
            r2_co   <= r1_co;
            r2_prod <= r1_sum * $signed({1'b0, r1_m});
            if (busy) cyc_cnt <= cyc_cnt + 32'd1;

            case (st)
            S_IDLE: if (start) begin
                busy <= 1'b1; cyc_cnt <= 32'd1;
                li <= 3'd0; cog <= 2'd0; oy <= 7'd0; ox <= 7'd0;
                st <= S_PIX;
            end
            S_PIX: begin                       // bat dau 1 diem anh ra
                clr_acc <= 1'b1;
                ci <= 6'd0; ky <= 2'd0; kx <= 2'd0; kidx <= 9'd0; ci_off <= 15'd0;
                st <= S_MAC;
            end
            S_MAC: begin                       // moi chu ky phat 1 tap
                kidx <= kidx + 9'd1;
                if (kx == L_K - 2'd1) begin
                    kx <= 2'd0;
                    if (ky == L_K - 2'd1) begin
                        ky <= 2'd0;
                        ci <= ci + 6'd1;
                        ci_off <= ci_off + L_HIWI;
                    end else ky <= ky + 2'd1;
                end else kx <= kx + 2'd1;
                if (last_tap) begin dcnt <= 3'd0; st <= S_DRAIN; end
            end
            S_DRAIN: begin                     // cho RAM + nhan + cong don
                dcnt <= dcnt + 3'd1;
                if (dcnt == 3'd3) begin rq_cnt <= 6'd0; st <= S_RQ; end
            end
            S_RQ: begin
                if (rq_cnt < ncout) begin
                    r1_v   <= 1'b1;
                    r1_co  <= rq_co;
                    r1_sum <= $signed({acc_lane[rq_j[3:0]][31], acc_lane[rq_j[3:0]]})
                            + $signed({bias_mem[cidx][31], bias_mem[cidx]})
                            + ((comp_en && lmode == 2'd0) ? $signed({comp_mem[cidx][31], comp_mem[cidx]}) : 33'sd0);
                    r1_m   <= m_mem[cidx];
                end
                rq_cnt <= rq_cnt + 6'd1;
                if (rq_cnt == ncout + 6'd2) st <= S_NEXT;
            end
            S_NEXT: begin
                st <= S_PIX;
                if (ox == L_WO - 7'd1) begin
                    ox <= 7'd0;
                    if (oy == L_HO - 7'd1) begin
                        oy <= 7'd0;
                        if (cog == L_NG - 2'd1) begin
                            cog <= 2'd0;
                            if (li == NUM_LAYERS - 1) begin
                                st <= S_IDLE; busy <= 1'b0; done <= 1'b1; cycles <= cyc_cnt;
                            end else li <= li + 3'd1;
                        end else cog <= cog + 2'd1;
                    end else oy <= oy + 7'd1;
                end else ox <= ox + 7'd1;
            end
            default: st <= S_IDLE;
            endcase
        end
    end
endmodule
