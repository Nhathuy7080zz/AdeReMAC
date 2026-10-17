// SPI slave (mode 0: CPOL=0, CPHA=0), MSB first, oversampled by the fabric clock.
// Keep SCLK <= clk/6 (27 MHz fabric -> use <= 4 MHz; the host firmware uses 1 MHz).
//
// Frame (CS low for exactly 16 SCLK):
//   byte 0 : {W, addr[6:0]}     W=1 write, W=0 read
//   byte 1 : write -> data from master
//            read  -> data from slave on MISO (master sends dummy)
module spi_slave (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       sclk,
    input  wire       cs_n,
    input  wire       mosi,
    output wire       miso,
    output reg        wr_en,       // 1-cycle pulse
    output reg  [6:0] wr_addr,
    output reg  [7:0] wr_data,
    output reg  [6:0] rd_addr,     // register file must present rd_data combinationally
    input  wire [7:0] rd_data
);
    reg [2:0] sclk_s, cs_s, mosi_s;
    always @(posedge clk) begin
        sclk_s <= {sclk_s[1:0], sclk};
        cs_s   <= {cs_s[1:0],   cs_n};
        mosi_s <= {mosi_s[1:0], mosi};
    end
    wire sclk_rise = (sclk_s[2:1] == 2'b01);
    wire sclk_fall = (sclk_s[2:1] == 2'b10);
    wire cs_act    = ~cs_s[1];
    wire mosi_v    = mosi_s[1];

    reg [7:0] rx_sh, tx_sh;
    reg [2:0] bit_cnt;
    reg       byte_idx;      // 0 = command byte, 1 = data byte
    reg       is_write;
    reg       load_tx;       // load read data at the next falling edge

    assign miso = tx_sh[7];

    always @(posedge clk) begin
        wr_en <= 1'b0;
        if (!rst_n || !cs_act) begin
            bit_cnt  <= 3'd0;
            byte_idx <= 1'b0;
            load_tx  <= 1'b0;
            tx_sh    <= 8'h00;
        end else begin
            if (sclk_rise) begin
                rx_sh   <= {rx_sh[6:0], mosi_v};
                bit_cnt <= bit_cnt + 3'd1;
                if (bit_cnt == 3'd7) begin
                    if (!byte_idx) begin                 // command byte complete
                        is_write <= rx_sh[6];            // rx_sh[6] is bit 7 of the byte
                        rd_addr  <= {rx_sh[5:0], mosi_v};
                        wr_addr  <= {rx_sh[5:0], mosi_v};
                        load_tx  <= 1'b1;
                        byte_idx <= 1'b1;
                    end else if (is_write) begin         // data byte complete
                        wr_data  <= {rx_sh[6:0], mosi_v};
                        wr_en    <= 1'b1;
                        is_write <= 1'b0;                // ignore any extra clocks in this frame
                    end
                end
            end
            if (sclk_fall) begin
                if (load_tx) begin
                    tx_sh   <= rd_data;                  // first data bit is on MISO before next rising edge
                    load_tx <= 1'b0;
                end else begin
                    tx_sh   <= {tx_sh[6:0], 1'b0};
                end
            end
        end
    end
endmodule
