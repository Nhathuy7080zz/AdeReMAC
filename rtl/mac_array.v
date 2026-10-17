// N parallel AdaReMAC lanes fed by per-lane xorshift32 generators.
// Purpose on hardware: make the switching activity large enough that the power
// difference between modes is measurable with an INA219. Lane 0 carries the
// accuracy monitor (hard multiplier as golden reference).
module mac_array #(
    parameter N_LANES = 16
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        run,
    input  wire [1:0]  mode,
    input  wire [1:0]  thr,
    output wire        mon_valid,   // lane 0 result valid
    output wire        mon_enr,     // lane 0 residual was active
    output wire [15:0] mon_err,     // lane 0 |exact - approx|
    output reg  [7:0]  chk          // XOR of all accumulator LSBs (keeps every lane alive)
);
    wire [7:0] lsb [0:N_LANES-1];

    genvar k;
    generate
        for (k = 0; k < N_LANES; k = k + 1) begin : g_lane
            localparam [31:0] SEED = (32'h2545F491 + k * 32'h9E3779B9) | 32'h1;
            reg [31:0] s;
            wire [31:0] x1 = s ^ (s << 13);
            wire [31:0] x2 = x1 ^ (x1 >> 17);
            wire [31:0] x3 = x2 ^ (x2 << 5);
            always @(posedge clk) begin
                if (!rst_n)   s <= SEED;
                else if (run) s <= x3;
            end

            wire signed [31:0] acc;
            wire v, e;
            wire [15:0] er;
            adaremac_mac_lane #(.ACC_W(32), .MONITOR(k == 0)) u_lane (
                .clk(clk), .rst_n(rst_n), .en(run), .first(1'b0),
                .a(s[15:8]), .b(s[31:24]), .mode(mode), .thr(thr),
                .acc(acc), .v_out(v), .enr_out(e), .err_mag(er));
            assign lsb[k] = acc[7:0];

            if (k == 0) begin : g_m
                assign mon_valid = v;
                assign mon_enr   = e;
                assign mon_err   = er;
            end
        end
    endgenerate

    integer i;
    reg [7:0] x;
    always @* begin
        x = 8'd0;
        for (i = 0; i < N_LANES; i = i + 1) x = x ^ lsb[i];
    end
    always @(posedge clk) chk <= x;
endmodule
