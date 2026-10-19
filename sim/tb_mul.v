`timescale 1ns/1ps
// Exhaustive check of adaremac_mul over all 65536 operand pairs, every mode.
module tb_mul;
    reg  [7:0] a, b;
    reg  [1:0] mode, thr;
    wire [15:0] p;
    wire enr;
    adaremac_mul dut (.a(a), .b(b), .mode(mode), .thr(thr), .p(p), .enr(enr));

    integer ia, ib, t, errors;
    integer exact, d0, d1, d2, r, expect_p, expect_enr;
    // stats per configuration
    integer sum_abs, err_cnt, enr_cnt;
    real    nmed, er, act;

    task run_cfg(input [1:0] m, input [1:0] th, input integer label);
    begin
        mode = m; thr = th; sum_abs = 0; err_cnt = 0; enr_cnt = 0;
        for (ia = 0; ia < 256; ia = ia + 1)
            for (ib = 0; ib < 256; ib = ib + 1) begin
                a = ia; b = ib; #1;
                exact = ia * ib;
                d0 = a[0] & b[0];
                d1 = (a[0] & b[1]) + (a[1] & b[0]);
                d2 = (a[0] & b[2]) + (a[1] & b[1]) + (a[2] & b[0]);
                r  = 2*d2 - d0;
                if (m[1])      expect_enr = 1;
                else if (m[0]) expect_enr = (d2 >= th);
                else           expect_enr = 0;
                expect_p = expect_enr ? exact : (exact - r);
                if (p !== expect_p[15:0] || enr !== expect_enr[0]) begin
                    errors = errors + 1;
                    if (errors < 10)
                        $display("FAIL a=%0d b=%0d mode=%0d thr=%0d p=%0d exp=%0d enr=%0d exp_enr=%0d",
                                 ia, ib, m, th, p, expect_p, enr, expect_enr);
                end
                sum_abs = sum_abs + ((exact > p) ? exact - p : p - exact);
                if (p != exact) err_cnt = err_cnt + 1;
                if (enr) enr_cnt = enr_cnt + 1;
            end
        nmed = (sum_abs / 65536.0) / (255.0 * 255.0);
        er   = err_cnt / 65536.0;
        act  = enr_cnt / 65536.0;
        $display("cfg mode=%0d thr=%0d | NMED=%e  ErrRate=%6.2f%%  ResidualActive=%6.2f%%",
                 m, th, nmed, er*100.0, act*100.0);
    end
    endtask

    initial begin
        errors = 0;
        run_cfg(2'd0, 2'd0, 0);
        run_cfg(2'd1, 2'd1, 1);
        run_cfg(2'd1, 2'd2, 2);
        run_cfg(2'd1, 2'd3, 3);
        run_cfg(2'd2, 2'd0, 4);
        run_cfg(2'd3, 2'd0, 5);
        if (errors == 0) $display("TB_MUL PASS"); else $display("TB_MUL FAIL errors=%0d", errors);
        $finish;
    end
endmodule
