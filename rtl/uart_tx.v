// Simple UART transmitter, 8N1. Adapted from Sipeed's Tang Nano 20K uart example.
// Protocol: when tx_data_ready=1, present tx_data and pulse tx_data_valid for
// exactly one cycle to send a byte. tx_data_ready drops until the stop bit ends.
module uart_tx #(
    parameter CLK_FRE   = 27,      // MHz
    parameter BAUD_RATE = 115200
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] tx_data,
    input  wire       tx_data_valid,
    output reg        tx_data_ready,
    output wire       tx_pin
);
    localparam CYCLE = CLK_FRE * 1000000 / BAUD_RATE;
    localparam S_IDLE = 0, S_START = 1, S_SEND = 2, S_STOP = 3;

    reg [1:0]  state;
    reg [15:0] cyc;
    reg [2:0]  bit_cnt;
    reg [7:0]  shreg;
    reg        txreg;
    assign tx_pin = txreg;

    wire cyc_done = (cyc == CYCLE - 1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; cyc <= 16'd0; bit_cnt <= 3'd0; txreg <= 1'b1; tx_data_ready <= 1'b1;
        end else begin
            case (state)
                S_IDLE: begin
                    txreg <= 1'b1;
                    if (tx_data_valid) begin
                        shreg <= tx_data; state <= S_START; cyc <= 16'd0; tx_data_ready <= 1'b0;
                    end else tx_data_ready <= 1'b1;
                end
                S_START: begin
                    txreg <= 1'b0;
                    if (cyc_done) begin state <= S_SEND; cyc <= 16'd0; bit_cnt <= 3'd0; end
                    else cyc <= cyc + 16'd1;
                end
                S_SEND: begin
                    txreg <= shreg[0];
                    if (cyc_done) begin
                        cyc <= 16'd0;
                        if (bit_cnt == 3'd7) state <= S_STOP;
                        else begin shreg <= shreg >> 1; bit_cnt <= bit_cnt + 3'd1; end
                    end else cyc <= cyc + 16'd1;
                end
                S_STOP: begin
                    txreg <= 1'b1;
                    if (cyc_done) begin state <= S_IDLE; tx_data_ready <= 1'b1; end
                    else cyc <= cyc + 16'd1;
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
