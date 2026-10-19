// TU DONG SINH boi ai_fomo/export_fomo_micro.py - khong sua tay
localparam NUM_LAYERS = 6;
localparam WMEM_DEPTH = 2048;
localparam CMEM_DEPTH = 128;
always @* begin
    L_CIN = 0; L_COUT = 0; L_K = 0; L_S = 0; L_HI = 0; L_WI = 0; L_HIWI = 0;
    L_HO = 0; L_WO = 0; L_HOWO = 0; L_RELU = 0; L_LAST = 0; L_SH = 0; L_WBASE = 0;
    L_CBASE = 0; L_INB = 0; L_OUTB = 0; L_NG = 0; L_KT = 0;
    case (li)
    3'd0: begin L_CIN = 1; L_COUT = 8; L_K = 3; L_S = 2; L_HI = 96; L_WI = 96; L_HIWI = 9216; L_HO = 48; L_WO = 48; L_HOWO = 2304; L_RELU = 1; L_LAST = 0; L_SH = 23; L_WBASE = 0; L_CBASE = 0; L_INB = 18432; L_OUTB = 0; L_NG = 1; L_KT = 9; end
    3'd1: begin L_CIN = 8; L_COUT = 16; L_K = 3; L_S = 2; L_HI = 48; L_WI = 48; L_HIWI = 2304; L_HO = 24; L_WO = 24; L_HOWO = 576; L_RELU = 1; L_LAST = 0; L_SH = 23; L_WBASE = 9; L_CBASE = 8; L_INB = 0; L_OUTB = 18432; L_NG = 1; L_KT = 72; end
    3'd2: begin L_CIN = 16; L_COUT = 32; L_K = 3; L_S = 2; L_HI = 24; L_WI = 24; L_HIWI = 576; L_HO = 12; L_WO = 12; L_HOWO = 144; L_RELU = 1; L_LAST = 0; L_SH = 23; L_WBASE = 81; L_CBASE = 24; L_INB = 18432; L_OUTB = 0; L_NG = 2; L_KT = 144; end
    3'd3: begin L_CIN = 32; L_COUT = 32; L_K = 3; L_S = 1; L_HI = 12; L_WI = 12; L_HIWI = 144; L_HO = 12; L_WO = 12; L_HOWO = 144; L_RELU = 1; L_LAST = 0; L_SH = 23; L_WBASE = 369; L_CBASE = 56; L_INB = 0; L_OUTB = 18432; L_NG = 2; L_KT = 288; end
    3'd4: begin L_CIN = 32; L_COUT = 32; L_K = 3; L_S = 1; L_HI = 12; L_WI = 12; L_HIWI = 144; L_HO = 12; L_WO = 12; L_HOWO = 144; L_RELU = 1; L_LAST = 0; L_SH = 24; L_WBASE = 945; L_CBASE = 88; L_INB = 18432; L_OUTB = 0; L_NG = 2; L_KT = 288; end
    3'd5: begin L_CIN = 32; L_COUT = 1; L_K = 1; L_S = 1; L_HI = 12; L_WI = 12; L_HIWI = 144; L_HO = 12; L_WO = 12; L_HOWO = 144; L_RELU = 0; L_LAST = 1; L_SH = 26; L_WBASE = 1521; L_CBASE = 120; L_INB = 0; L_OUTB = 18432; L_NG = 1; L_KT = 32; end
    default: ;
    endcase
end
