// Formats one status line as ASCII and streams it out over uart_tx.
//   "S<n> OPS=<8 hex> ENR=<8 hex> ERR=<10 hex>\r\n"     (45 bytes, index 0..44)
// <n> is the 0..4 config index (see top_standalone.v for what each one means).
// Latches its inputs on `send` (ignored while busy) so the caller is free to
// keep counting immediately after triggering a report.
module report_tx (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        send,
    input  wire [2:0]  state_in,
    input  wire [31:0] ops_in,
    input  wire [31:0] enr_in,
    input  wire [39:0] err_in,
    output wire        busy,
    output wire        uart_pin
);
    localparam MSGLEN = 45;

    reg [2:0]  s_state;
    reg [31:0] s_ops;
    reg [31:0] s_enr;
    reg [39:0] s_err;
    reg [5:0]  idx;
    reg        sending, armed, seen_low;
    reg [7:0]  tx_byte;
    reg        tx_valid;
    wire       tx_ready;

    assign busy = sending;

    function [7:0] hexc(input [3:0] n);
        hexc = (n < 4'ha) ? (8'h30 + n) : (8'h41 + n - 8'ha);
    endfunction

    // Combinational: byte for the current idx, given the latched snapshot.
    function [7:0] byte_at(input [5:0] i, input [2:0] st, input [31:0] ops,
                            input [31:0] enr, input [39:0] err);
        integer d;
        begin
            case (i)
                0:  byte_at = "S";
                1:  byte_at = 8'h30 + {5'b0, st};   // state is 0..4, one decimal digit
                2:  byte_at = " ";
                3:  byte_at = "O";
                4:  byte_at = "P";
                5:  byte_at = "S";
                6:  byte_at = "=";
                15: byte_at = " ";
                16: byte_at = "E";
                17: byte_at = "N";
                18: byte_at = "R";
                19: byte_at = "=";
                28: byte_at = " ";
                29: byte_at = "E";
                30: byte_at = "R";
                31: byte_at = "R";
                32: byte_at = "=";
                43: byte_at = 8'h0D;
                44: byte_at = 8'h0A;
                default: begin
                    if (i >= 7 && i <= 14) begin
                        d = i - 7;                      // ops nibble 0 (MSB) .. 7 (LSB)
                        byte_at = hexc(ops[(31 - 4*d) -: 4]);
                    end else if (i >= 20 && i <= 27) begin
                        d = i - 20;
                        byte_at = hexc(enr[(31 - 4*d) -: 4]);
                    end else if (i >= 33 && i <= 42) begin
                        d = i - 33;                      // err nibble 0 (MSB) .. 9 (LSB)
                        byte_at = hexc(err[(39 - 4*d) -: 4]);
                    end else begin
                        byte_at = " ";
                    end
                end
            endcase
        end
    endfunction

    uart_tx #(.CLK_FRE(27), .BAUD_RATE(115200)) u_tx (
        .clk(clk), .rst_n(rst_n),
        .tx_data(tx_byte), .tx_data_valid(tx_valid), .tx_data_ready(tx_ready), .tx_pin(uart_pin));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sending <= 1'b0; armed <= 1'b0; seen_low <= 1'b0; idx <= 6'd0; tx_valid <= 1'b0;
        end else begin
            tx_valid <= 1'b0;
            if (!sending) begin
                if (send) begin
                    s_state <= state_in; s_ops <= ops_in; s_enr <= enr_in; s_err <= err_in;
                    idx <= 6'd0; sending <= 1'b1; armed <= 1'b0; seen_low <= 1'b0;
                end
            end else if (idx == MSGLEN) begin
                sending <= 1'b0;
            end else if (!armed) begin
                if (tx_ready) begin
                    tx_byte  <= byte_at(idx, s_state, s_ops, s_enr, s_err);
                    tx_valid <= 1'b1;
                    armed    <= 1'b1;
                    seen_low <= 1'b0;
                end
            end else begin
                if (!tx_ready) seen_low <= 1'b1;
                if (tx_ready && seen_low) begin
                    idx   <= idx + 6'd1;
                    armed <= 1'b0;
                end
            end
        end
    end
endmodule
