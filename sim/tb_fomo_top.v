// =============================================================================
// tb_fomo_top.v - Kiem chung fomo_top_tn20k qua dung giao thuc SPI ma ESP32-S3 dung.
//   1) READ_REG ID, WRITE_REG mode
//   2) WRITE_FRAME 9216 byte (SCLK 20 MHz, clk 27 MHz)
//   3) cho xong, READ_RESULT, so 144 logit voi vector vang tu Python (bit-exact)
// Chay: xem sim/run_fomo_sim.sh
// =============================================================================
`timescale 1ns/1ps
module tb_fomo_top;
    reg clk = 0;
    always #18.518 clk = ~clk;                 // 27 MHz

    reg  sclk = 0, cs_n = 1, mosi = 0;
    wire miso;
    wire [5:0] led_n;

    fomo_top_tn20k dut (.sys_clk(clk), .spi_sclk(sclk), .spi_cs_n(cs_n),
                        .spi_mosi(mosi), .spi_miso(miso), .led_n(led_n));

    localparam real HALF = 25.0;               // SCLK 20 MHz
    reg [7:0] frame [0:9215];
    reg [7:0] expect_m2 [0:143];
    reg [7:0] expect_m0 [0:143];
    reg [7:0] rx;
    reg [7:0] res [0:149];
    integer i, t, errs, total_fail;
    reg [8*128-1:0] path;

    task xfer(input [7:0] tx, output [7:0] r);
        integer b;
        begin
            for (b = 7; b >= 0; b = b - 1) begin
                mosi = tx[b];
                #(HALF);
                sclk = 1; r[b] = miso;
                #(HALF);
                sclk = 0;
            end
        end
    endtask

    task cs_lo; begin cs_n = 0; #(HALF); end endtask
    task cs_hi; begin #(HALF); cs_n = 1; #(400); end endtask

    task write_reg(input [7:0] a, input [7:0] d);
        begin cs_lo; xfer(8'h03, rx); xfer(a, rx); xfer(d, rx); cs_hi; end
    endtask

    task read_reg(input [7:0] a, output [7:0] d);
        begin cs_lo; xfer(8'h04, rx); xfer(a, rx); xfer(8'h00, rx); xfer(8'h00, d); cs_hi; end
    endtask

    task send_frame;
        begin
            cs_lo; xfer(8'h01, rx);
            for (i = 0; i < 9216; i = i + 1) xfer(frame[i], rx);
            cs_hi;
        end
    endtask

    task read_result;
        begin
            cs_lo; xfer(8'h02, rx); xfer(8'h00, rx);
            for (i = 0; i < 150; i = i + 1) xfer(8'h00, res[i]);
            cs_hi;
        end
    endtask

    task wait_done;
        integer guard;
        begin
            guard = 0;
            @(posedge clk);
            while ((dut.u_core.busy || !dut.res_new) && guard < 2000000) begin @(posedge clk); guard = guard + 1; end
            if (guard >= 2000000) begin $display("TIMEOUT"); $finish; end
        end
    endtask

    task check(input integer tt, input integer use_m0);
        begin
            errs = 0;
            for (i = 0; i < 144; i = i + 1)
                if (res[6 + i] !== (use_m0 ? expect_m0[i] : expect_m2[i])) begin
                    if (errs < 5) $display("  lech o %0d: rtl=%02x expect=%02x", i, res[6 + i], use_m0 ? expect_m0[i] : expect_m2[i]);
                    errs = errs + 1;
                end
            $display("vector %0d %s: status=%02x frame_cnt=%0d cycles=%0d (%.2f ms @27MHz) -> %s (%0d lech)",
                     tt, use_m0 ? "Mode0+bu" : "Mode2   ", res[0], res[1],
                     {res[5], res[4], res[3], res[2]}, {res[5], res[4], res[3], res[2]} / 27000.0,
                     errs ? "SAI" : "KHOP", errs);
            if (errs) total_fail = total_fail + 1;
        end
    endtask

    integer m, nvec;
    initial begin
        total_fail = 0;
        if (!$value$plusargs("nvec=%d", nvec)) nvec = 1;
        #20000;                                 // cho POR (256 chu ky) xong
        read_reg(8'h10, rx);
        $display("ID = %02x (can F0)", rx);
        if (rx !== 8'hF0) total_fail = total_fail + 1;
        for (t = 0; t < nvec; t = t + 1) begin
            $sformat(path, "vectors/frame%0d.hex", t);      $readmemh(path, frame);
            $sformat(path, "vectors/expect%0d_m2.hex", t);  $readmemh(path, expect_m2);
            $sformat(path, "vectors/expect%0d_m0c.hex", t); $readmemh(path, expect_m0);
            for (m = 0; m < 2; m = m + 1) begin
                write_reg(8'h00, m ? 8'd0 : 8'd2); write_reg(8'h01, m ? 8'd0 : 8'd2);
                write_reg(8'h02, m ? 8'd0 : 8'd2); write_reg(8'h03, m ? 8'd0 : 8'd2);
                write_reg(8'h04, m ? 8'd0 : 8'd2); write_reg(8'h05, m ? 8'd0 : 8'd2);
                send_frame;
                wait_done;
                read_result;
                check(t, m);
            end
        end
        $display("%s", total_fail ? "FAIL" : "PASS");
        $finish;
    end
endmodule
