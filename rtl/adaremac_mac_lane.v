// One AdaReMAC MAC lane: signed INT8 x signed INT8, 32-bit accumulator.
// Sign-magnitude wrapper around the unsigned approximate multiplier.
// 3 stages: [abs/sign] -> [approx multiply] -> [accumulate]
// Every stage has a clock enable, so with en=0 the lane does not toggle.
module adaremac_mac_lane #(
    parameter ACC_W   = 32,
    parameter MONITOR = 0     // 1: also compute |exact - approx| (uses one hard multiplier)
) (
    input  wire                     clk,
    input  wire                     rst_n,
    input  wire                     en,      // operands valid this cycle
    input  wire                     first,   // first product of a dot product (acc <= product)
    input  wire signed [7:0]        a,
    input  wire signed [7:0]        b,
    input  wire [1:0]               mode,
    input  wire [1:0]               thr,
    output reg  signed [ACC_W-1:0]  acc,
    output reg                      v_out,   // stage-3 valid (aligned with enr_out / err_mag)
    output reg                      enr_out,
    output wire [15:0]              err_mag
);
    // ---- stage 1: sign-magnitude ------------------------------------------------
    reg [7:0] ma, mb;
    reg       neg1, first1, v1;
    always @(posedge clk) begin
        if (!rst_n) v1 <= 1'b0;
        else begin
            v1 <= en;
            if (en) begin
                ma     <= a[7] ? (~a + 8'd1) : a;   // -128 -> 128 fits in 8 bits
                mb     <= b[7] ? (~b + 8'd1) : b;
                neg1   <= a[7] ^ b[7];
                first1 <= first;
            end
        end
    end

    // ---- stage 2: approximate multiply ------------------------------------------
    wire [15:0] p_mag;
    wire        enr_w;
    adaremac_mul u_mul (.a(ma), .b(mb), .mode(mode), .thr(thr), .p(p_mag), .enr(enr_w));

    reg [15:0] p2;
    reg        neg2, first2, v2, enr2;
    reg [15:0] ex2;                                   // exact magnitude (monitor only)
    always @(posedge clk) begin
        if (!rst_n) v2 <= 1'b0;
        else begin
            v2 <= v1;
            if (v1) begin
                p2     <= p_mag;
                neg2   <= neg1;
                first2 <= first1;
                enr2   <= enr_w;
                if (MONITOR) ex2 <= ma * mb;
            end
        end
    end

    // ---- stage 3: accumulate -----------------------------------------------------
    wire signed [ACC_W-1:0] pe = {{(ACC_W-16){1'b0}}, p2};
    always @(posedge clk) begin
        if (!rst_n) begin
            acc <= {ACC_W{1'b0}}; v_out <= 1'b0; enr_out <= 1'b0;
        end else begin
            v_out <= v2;
            if (v2) begin
                enr_out <= enr2;
                if (first2) acc <= neg2 ? -pe : pe;
                else        acc <= neg2 ? (acc - pe) : (acc + pe);
            end
        end
    end

    // ---- monitor: |exact - approx| (sign does not change the error magnitude) ------
    // Registered so it lines up with v_out / enr_out (all three come from stage 3).
    wire signed [16:0] diff  = $signed({1'b0, ex2}) - $signed({1'b0, p2});
    wire        [15:0] err_c = diff[16] ? (~diff[15:0] + 16'd1) : diff[15:0];
    reg         [15:0] err_r;
    always @(posedge clk) begin
        if (!rst_n)   err_r <= 16'd0;
        else if (v2)  err_r <= MONITOR ? err_c : 16'd0;
    end
    assign err_mag = err_r;
endmodule
