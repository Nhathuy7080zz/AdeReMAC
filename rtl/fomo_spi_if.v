// =============================================================================
// fomo_spi_if.v  -  SPI slave mode 0 (CPOL=0, CPHA=0), MSB truoc, cho ESP32-S3.
//
// Khac voi spi_slave.v cu (lay mau SCLK bang xung 27 MHz -> SCLK toi da ~4 MHz),
// module nay dich bit NGAY TRONG MIEN SCLK, chi chuyen tung BYTE da hoan chinh sang
// mien clk bang co toggle -> chiu duoc SCLK ~20 MHz (1 byte = 400 ns >> 3 chu ky clk).
//
// Mien clk nhan:
//   byte_ev   : xung 1 chu ky khi co 1 byte moi
//   byte_data : gia tri byte
//   byte_idx  : so thu tu byte trong giao dich hien tai (0 = byte lenh)
//   cs_end    : xung 1 chu ky khi CS len muc cao (sau khi da xu ly byte cuoi)
// Mien clk cung cap tx_next: byte se duoc phat o byte SAU byte dang chay
// (quy uoc: byte thu n tren MISO mang gia tri tx_next da dat khi byte n-2 ket thuc).
// =============================================================================
module fomo_spi_if (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       sclk,
    input  wire       cs_n,
    input  wire       mosi,
    output wire       miso,
    output reg        byte_ev,
    output reg  [7:0] byte_data,
    output reg [15:0] byte_idx,
    output reg        cs_end,
    output wire       cs_active,
    input  wire [7:0] tx_next
);
    // ------------------------------- mien SCLK -------------------------------
    reg [2:0] bcnt = 3'd0;          // gia tri dau (FF GW2A khoi dong = 0; can cho mo phong)
    reg [6:0] sh;
    always @(posedge sclk or posedge cs_n) begin
        if (cs_n) begin
            bcnt <= 3'd0;
        end else begin
            sh   <= {sh[5:0], mosi};
            bcnt <= bcnt + 3'd1;
        end
    end

    reg [7:0] rx_byte = 8'h00;
    reg       rx_tog  = 1'b0;
    always @(posedge sclk) begin
        if (!cs_n && bcnt == 3'd7) begin
            rx_byte <= {sh, mosi};
            rx_tog  <= ~rx_tog;
        end
    end

    reg [7:0] tx_sh = 8'h00;
    always @(negedge sclk or posedge cs_n) begin
        if (cs_n)               tx_sh <= 8'h00;
        else if (bcnt == 3'd0)  tx_sh <= tx_next;          // ranh gioi byte: nap byte ke tiep
        else                    tx_sh <= {tx_sh[6:0], 1'b0};
    end
    assign miso = tx_sh[7];

    // ------------------------------- mien clk --------------------------------
    reg [2:0] tog_s = 3'd0;
    reg [4:0] cs_s  = 5'b11111;      // CS tre hon 2 tang so voi toggle -> byte cuoi luon xu ly truoc
    always @(posedge clk) begin
        tog_s <= {tog_s[1:0], rx_tog};
        cs_s  <= {cs_s[3:0], cs_n};
    end
    wire ev = tog_s[2] ^ tog_s[1];
    assign cs_active = ~cs_s[4];

    always @(posedge clk) begin
        byte_ev <= 1'b0;
        cs_end  <= 1'b0;
        if (!rst_n) begin
            byte_idx <= 16'd0;
        end else begin
            if (ev) begin
                byte_ev   <= 1'b1;
                byte_data <= rx_byte;
            end
            if (byte_ev)
                byte_idx <= byte_idx + 16'd1;
            if (cs_s[4] && !cs_s[3]) begin       // bat dau giao dich moi
                byte_idx <= 16'd0;
            end
            if (!cs_s[4] && cs_s[3])             // CS vua len (sau 2 tang tre)
                cs_end <= 1'b1;
        end
    end
endmodule
