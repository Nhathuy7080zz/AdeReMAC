// =============================================================================
// fomo_mac_lane.v  -  1 lan MAC: kich hoat uint8 x trong so (dau-do lon) qua bo nhan
//                     xap xi AdaReMAC, cong don 32 bit co dau.
//   w_sm[7]   : dau cua trong so (1 = am)
//   w_sm[6:0] : do lon |w| <= 127  (luu san dang dau-do lon -> khong can mach dao dau truoc bo nhan)
//   p = (-1)^sign * adaremac_mul(x, |w|, mode, thr)
// Duong ong 2 tang: [x, w] -> (nhan AdaReMAC) -> pm_r -> (cong don co dau) -> acc
// =============================================================================
module fomo_mac_lane (
    input  wire        clk,
    input  wire        clr,        // xoa acc (uu tien hon acc_en)
    input  wire        mul_en,     // x/w hop le o chu ky nay
    input  wire        acc_en,     // cong pm_r vao acc
    input  wire [7:0]  x,          // kich hoat (0 neu ngoai bien)
    input  wire [7:0]  w_sm,       // trong so dang dau-do lon
    input  wire [1:0]  mode,
    input  wire [1:0]  thr,
    output reg  signed [31:0] acc
);
    wire [15:0] pmag;
    wire        enr_unused;
    adaremac_mul u_mul (
        .a(x), .b({1'b0, w_sm[6:0]}), .mode(mode), .thr(thr), .p(pmag), .enr(enr_unused));

    reg [15:0] pm_r;
    reg        neg_r;
    always @(posedge clk) begin
        pm_r  <= mul_en ? pmag : 16'd0;
        neg_r <= w_sm[7];
    end

    always @(posedge clk) begin
        if (clr)         acc <= 32'sd0;
        else if (acc_en) acc <= neg_r ? acc - {16'd0, pm_r} : acc + {16'd0, pm_r};
    end
endmodule
