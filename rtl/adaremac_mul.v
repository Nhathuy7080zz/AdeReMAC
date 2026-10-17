// AdaReMAC approximate multiplier, unsigned 8x8, qL = 2
//
//   P   = PH + PL,   PL = D0 + 2*D1 + 4*D2   (diagonals 0..2, weights 1/2/4)
//   Main path : P^L = 2*(D0 + D1 + D2)       (one adder, one shift)
//   Residual  : R   = PL - P^L = 2*D2 - D0   (D1 disappears; shift + subtract only)
//
//   mode 0 : full approximate  -> p = PH + P^L            (residual isolated, ENR = 0)
//   mode 1 : selective         -> residual on only when D2 >= thr
//   mode 2/3: full exact       -> p = PH + P^L + R = a*b  (ENR = 1)
//
// Operand isolation (AND with enr) keeps the residual logic from toggling when
// it is off. On FPGA this is the practical equivalent of clock gating.
module adaremac_mul (
    input  wire [7:0]  a,
    input  wire [7:0]  b,
    input  wire [1:0]  mode,
    input  wire [1:0]  thr,     // mode 1 threshold on D2 (0..3)
    output wire [15:0] p,
    output wire        enr      // residual path active for this operation
);
    // ---- PPG: high part (diagonals >= 3), computed exactly -------------------
    wire [15:0] row_hi [0:7];
    genvar j;
    generate
        for (j = 0; j < 8; j = j + 1) begin : g_row
            wire [15:0] full = {8'b0, (a & {8{b[j]}})} << j;
            assign row_hi[j] = full & 16'hFFF8;      // drop bit positions 0..2
        end
    endgenerate

    wire [15:0] ph = ((row_hi[0] + row_hi[1]) + (row_hi[2] + row_hi[3]))
                   + ((row_hi[4] + row_hi[5]) + (row_hi[6] + row_hi[7]));

    // ---- Low diagonals D0, D1, D2 (counts of partial-product ones) -----------
    wire       d0 = a[0] & b[0];
    wire [1:0] d1 = (a[0] & b[1]) + (a[1] & b[0]);
    wire [1:0] d2 = (a[0] & b[2]) + (a[1] & b[1]) + (a[2] & b[0]);

    // ---- Main approximate path: P^L = qL * (D0 + D1 + D2), qL = 2 -------------
    wire [2:0] ml  = d0 + d1 + d2;                  // 0..6
    wire [3:0] plh = {ml, 1'b0};                    // 0..12

    // ---- Adaptive controller: decide whether the residual runs ---------------
    assign enr = mode[1] ? 1'b1 : (mode[0] ? (d2 >= thr) : 1'b0);

    // ---- Sparse residual: R = (D2 << 1) - D0 ----------------------------------
    wire [1:0] d2_g = d2 & {2{enr}};
    wire       d0_g = d0 & enr;
    wire signed [4:0] r = $signed({2'b00, d2_g, 1'b0}) - $signed({4'b0000, d0_g});

    // ---- Combine (low part is always >= 0, max 17) ----------------------------
    wire signed [5:0] low = $signed({2'b00, plh}) + r;
    assign p = ph + {11'b0, low[4:0]};
endmodule
