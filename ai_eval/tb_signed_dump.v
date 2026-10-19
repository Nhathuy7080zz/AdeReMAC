`timescale 1ns/1ps
// Dump ket qua that cua adaremac_mac_lane cho moi cap (a,b) co dau -128..127.
// Chay: vvp tb.vvp +mode=0 +thr=0 +out=dump.txt
// Moi dong: <tich co dau> <residual-bat 0/1>, theo thu tu a (ngoai) -> b (trong).
module tb_signed_dump;
    reg clk = 0, rst_n = 0, en = 0;
    reg signed [7:0] a = 0, b = 0;
    reg [1:0] mode, thr;
    wire signed [31:0] acc;
    wire v_out, enr_out;
    wire [15:0] err_mag;

    adaremac_mac_lane #(.ACC_W(32), .MONITOR(0)) dut (
        .clk(clk), .rst_n(rst_n), .en(en), .first(1'b1), .a(a), .b(b),
        .mode(mode), .thr(thr), .acc(acc), .v_out(v_out), .enr_out(enr_out), .err_mag(err_mag));

    always #5 clk = ~clk;

    integer fd, ia, ib, mi, ti, n_out;
    reg [8*256-1:0] fname;

    always @(posedge clk) if (v_out === 1'b1) begin
        $fwrite(fd, "%0d %0d\n", acc, enr_out);
        n_out = n_out + 1;
    end

    initial begin
        if (!$value$plusargs("mode=%d", mi)) mi = 2;
        if (!$value$plusargs("thr=%d", ti)) ti = 0;
        if (!$value$plusargs("out=%s", fname)) fname = "dump.txt";
        mode = mi[1:0]; thr = ti[1:0];
        fd = $fopen(fname, "w");
        n_out = 0;
        repeat (4) @(negedge clk);
        rst_n = 1;
        @(negedge clk);
        for (ia = -128; ia < 128; ia = ia + 1)
            for (ib = -128; ib < 128; ib = ib + 1) begin
                a = ia; b = ib; en = 1;
                @(negedge clk);
            end
        en = 0;
        repeat (8) @(negedge clk);
        $fclose(fd);
        $display("ops_written=%0d", n_out);
        $finish;
    end
endmodule
