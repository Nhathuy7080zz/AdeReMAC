// Button -> synchronized, debounced, single-cycle pulse on press (rising edge
// of the debounced level). Board buttons are active-high with a pull-down.
module btn_sync #(
    parameter DEBOUNCE_CYCLES = 270000   // ~10 ms at 27 MHz
) (
    input  wire clk,
    input  wire rst_n,
    input  wire btn_raw,
    output reg  pulse
);
    reg [1:0] sync_r;
    always @(posedge clk) sync_r <= {sync_r[0], btn_raw};

    reg [19:0] cnt;
    reg        level, level_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt <= 20'd0; level <= 1'b0; level_d <= 1'b0; pulse <= 1'b0;
        end else begin
            if (sync_r[1] == level) begin
                cnt <= 20'd0;
            end else begin
                cnt <= cnt + 20'd1;
                if (cnt >= DEBOUNCE_CYCLES) begin
                    level <= sync_r[1];
                    cnt   <= 20'd0;
                end
            end
            level_d <= level;
            pulse   <= level & ~level_d;     // one cycle on 0 -> 1 transition
        end
    end
endmodule
