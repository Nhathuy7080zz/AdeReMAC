`timescale 1ns/1ps
// End-to-end test: bit-banged SPI master -> register file -> MAC array -> counters
// + Hardware FOMO / Tiny-SSD Co-Processor MAC registers (0x30..0x37).
// The hardware error sum is checked against an independent software model
// of lane 0 (xorshift32 operands + AdaReMAC equations).
module tb_core;
    reg clk = 0, rst_n = 0;
    reg sclk = 0, cs_n = 1, mosi = 0;
    wire miso;
    wire [1:0] o_mode; wire o_run, o_seen;

    localparam N = 4;
    adaremac_core #(.N_LANES(N)) dut (
        .clk(clk), .rst_n(rst_n),
        .spi_sclk(sclk), .spi_cs_n(cs_n), .spi_mosi(mosi), .spi_miso(miso),
        .o_mode(o_mode), .o_run(o_run), .o_spi_seen(o_seen));

    always #18.5 clk = ~clk;   // ~27 MHz

    // ---------- SPI master (1 MHz, mode 0) ----------
    reg [7:0] rx;
    task xfer(input [7:0] b0, input [7:0] b1);
        integer i; reg [15:0] tx; reg [15:0] r;
        begin
            tx = {b0, b1}; r = 0;
            cs_n = 0; #1000;
            for (i = 15; i >= 0; i = i - 1) begin
                mosi = tx[i]; #500;
                sclk = 1; r = {r[14:0], miso}; #500;
                sclk = 0;
            end
            #1000; cs_n = 1; #2000;
            rx = r[7:0];
        end
    endtask
    task wr(input [6:0] a, input [7:0] d); begin xfer({1'b1, a}, d); end endtask
    task rd(input [6:0] a);                begin xfer({1'b0, a}, 8'h00); end endtask

    // ---------- software model of lane 0 ----------
    reg [31:0] ms;                       // model xorshift state
    task model_step(input [1:0] mode, input [1:0] thr,
                    output integer err, output integer enr);
        reg [31:0] x1, x2, x3;
        reg signed [7:0] sa, sb;
        integer ma, mb, d0, d1, d2, r, en;
        begin
            sa = ms[15:8]; sb = ms[31:24];
            ma = sa[7] ? -sa : sa;  mb = sb[7] ? -sb : sb;
            d0 = ma[0] & mb[0];
            d1 = (ma[0] & mb[1]) + (ma[1] & mb[0]);
            d2 = (ma[0] & mb[2]) + (ma[1] & mb[1]) + (ma[2] & mb[0]);
            r  = 2*d2 - d0;
            if (mode[1]) en = 1; else if (mode[0]) en = (d2 >= thr); else en = 0;
            enr = en; err = en ? 0 : ((r < 0) ? -r : r);
            x1 = ms ^ (ms << 13); x2 = x1 ^ (x1 >> 17); x3 = x2 ^ (x2 << 5);
            ms = x3;
        end
    endtask

    integer fails = 0;
    reg [31:0] ops, enr_hw; reg [39:0] err_hw;
    reg signed [31:0] fomo_acc_hw;
    integer i, e_i, n_i, exp_err, exp_enr;

    task run_case(input [1:0] mode, input [1:0] thr);
        begin
            wr(7'h00, {6'b0, mode});
            wr(7'h01, {6'b0, thr});
            wr(7'h02, 8'h05);             // RUN + CLR_STATS
            #60000;
            wr(7'h02, 8'h00);             // stop
            #2000;
            wr(7'h02, 8'h02);             // SNAP (run stays 0)
            #1000;
            rd(7'h10); ops[7:0]=rx;   rd(7'h11); ops[15:8]=rx;
            rd(7'h12); ops[23:16]=rx; rd(7'h13); ops[31:24]=rx;
            rd(7'h14); enr_hw[7:0]=rx;   rd(7'h15); enr_hw[15:8]=rx;
            rd(7'h16); enr_hw[23:16]=rx; rd(7'h17); enr_hw[31:24]=rx;
            rd(7'h18); err_hw[7:0]=rx;   rd(7'h19); err_hw[15:8]=rx;
            rd(7'h1A); err_hw[23:16]=rx; rd(7'h1B); err_hw[31:24]=rx;
            rd(7'h1C); err_hw[39:32]=rx;
            exp_err = 0; exp_enr = 0;
            for (i = 0; i < ops; i = i + 1) begin
                model_step(mode, thr, e_i, n_i);
                exp_err = exp_err + e_i; exp_enr = exp_enr + n_i;
            end
            $display("mode=%0d thr=%0d | ops=%0d enr=%0d (exp %0d) err_sum=%0d (exp %0d) mean_err=%f",
                     mode, thr, ops, enr_hw, exp_enr, err_hw, exp_err, (ops ? (1.0*err_hw)/ops : 0.0));
            if (ops == 0 || err_hw !== exp_err || enr_hw !== exp_enr) begin
                fails = fails + 1; $display("  -> MISMATCH");
            end
            if (mode[1] && err_hw !== 0) begin fails = fails + 1; $display("  -> exact mode must have zero error"); end
        end
    endtask

    initial begin
        ms = (32'h2545F491 + 0) | 32'h1;   // lane-0 seed
        #200 rst_n = 1;
        #2000;
        rd(7'h03); if (rx !== 8'hA5) begin fails = fails + 1; $display("ID read wrong: %h", rx); end
        else $display("ID = %h OK", rx);
        rd(7'h04); $display("LANES = %0d", rx);
        wr(7'h00, 8'h01); rd(7'h00); if (rx !== 8'h01) begin fails = fails + 1; $display("MODE readback wrong: %h", rx); end
        run_case(2'd0, 2'd0);
        run_case(2'd1, 2'd2);
        run_case(2'd1, 2'd1);
        run_case(2'd2, 2'd0);

        // Verify FOMO / Tiny-SSD Hardware Co-Processor MAC registers (0x30..0x37) in Exact Mode 2
        wr(7'h00, 8'h02); // Mode 2 (Exact)
        wr(7'h32, 8'h00); // Clear fomo_acc = 0
        wr(7'h30, 8'd40); wr(7'h31, 8'd20);  // +800
        wr(7'h30, 8'd50); wr(7'h31, -8'sd10); // -500 -> total = +300
        rd(7'h30); fomo_acc_hw[7:0]=rx;   rd(7'h31); fomo_acc_hw[15:8]=rx;
        rd(7'h32); fomo_acc_hw[23:16]=rx; rd(7'h33); fomo_acc_hw[31:24]=rx;
        if (fomo_acc_hw !== 32'sd300) begin
            fails = fails + 1; $display("FOMO HW MAC mismatch: got %0d, exp 300", fomo_acc_hw);
        end else begin
            $display("FOMO HW MAC OK: acc=%0d", fomo_acc_hw);
        end

        if (fails == 0) $display("TB_CORE PASS"); else $display("TB_CORE FAIL fails=%0d", fails);
        $finish;
    end
endmodule
