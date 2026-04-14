// alu.sv — pipelined alu + fpu
// int ops : 1-cycle latency (alu_int)
// fp ops  : 3-cycle latency (fpu_addsub / fpu_mul / fpu_div)
// results broadcast on valid_out pulse with rob_tag passthrough
// key fix: all intermediate pipeline values stored in top-level module regs,
//          not as local vars inside named blocks — ensures correct pipelining

// ---------------------------------------------------------------------------
// alu_int — single-cycle integer / branch / compare unit
// ---------------------------------------------------------------------------
module alu_int (
    input         clk,
    input         reset,
    input         valid_in,
    input  [63:0] a,
    input  [63:0] b,
    input  [4:0]  op,
    input  [5:0]  rob_tag_in,
    output reg        valid_out,
    output reg [63:0] result,
    output reg [5:0]  rob_tag_out
);
    // op codes
    localparam ADD   = 5'd0,  SUB   = 5'd1,  MUL  = 5'd2,  DIV  = 5'd3;
    localparam AND   = 5'd4,  OR    = 5'd5,  XOR  = 5'd6,  NOT  = 5'd7;
    localparam SHR   = 5'd8,  SHL   = 5'd9;
    localparam CMPNZ = 5'd14, CMPGT = 5'd15;

    reg [63:0] comb;
    always @(*) begin
        case (op)
            ADD:   comb = a + b;
            SUB:   comb = a - b;
            MUL:   comb = a * b;
            DIV:   comb = (b == 0) ? 64'd0 : $signed(a) / $signed(b);
            AND:   comb = a & b;
            OR:    comb = a | b;
            XOR:   comb = a ^ b;
            NOT:   comb = ~a;
            SHR:   comb = a >> b[5:0];
            SHL:   comb = a << b[5:0];
            CMPNZ: comb = (a != 0) ? 64'd1 : 64'd0;
            CMPGT: comb = ($signed(a) > $signed(b)) ? 64'd1 : 64'd0;
            default: comb = 64'd0;
        endcase
    end

    // register output — 1 cycle latency
    always @(posedge clk) begin
        if (reset) begin
            valid_out <= 0; result <= 0; rob_tag_out <= 0;
        end else begin
            valid_out   <= valid_in;
            result      <= comb;
            rob_tag_out <= rob_tag_in;
        end
    end
endmodule


// ---------------------------------------------------------------------------
// fpu_addsub — 3-stage pipelined fp add/sub
// stage 0: unpack + align mantissas
// stage 1: add/sub aligned mantissas
// stage 2: normalize + round → output
// All intermediate values are top-level module regs, not local block vars.
// ---------------------------------------------------------------------------
module fpu_addsub (
    input         clk,
    input         reset,
    input         valid_in,
    input  [63:0] a,
    input  [63:0] b,
    input         do_sub,
    input  [5:0]  rob_tag_in,
    output reg        valid_out,
    output reg [63:0] result,
    output reg [5:0]  rob_tag_out
);

    // ---- stage 0 pipeline regs ----
    reg        s0_valid;
    reg [5:0]  s0_tag;
    reg        s0_special;
    reg [63:0] s0_special_val;
    reg        s0_sx, s0_sy;
    reg [10:0] s0_er;
    reg [55:0] s0_ax, s0_ay;

    // ---- stage 1 pipeline regs ----
    reg        s1_valid;
    reg [5:0]  s1_tag;
    reg        s1_special;
    reg [63:0] s1_special_val;
    reg        s1_sr;
    reg [10:0] s1_er;
    reg [56:0] s1_sum;

    // ================================================================
    // stage 0: unpack, detect specials, align mantissas
    // ================================================================
    always @(posedge clk) begin
        if (reset) begin
            s0_valid <= 0;
        end else begin
            s0_valid       <= valid_in;
            s0_tag         <= rob_tag_in;
            s0_special     <= 0;
            s0_special_val <= 0;
            s0_ax <= 0; s0_ay <= 0; s0_er <= 0;
            s0_sx <= a[63]; s0_sy <= b[63] ^ do_sub;

            if (valid_in) begin : s0_logic
                reg        sx, sy;
                reg [10:0] ex, ey, ediff, er;
                reg [52:0] mx, my;
                reg [55:0] ax, ay;
                reg        xnan, ynan, xinf, yinf, xz, yz;

                sx = a[63];          ex = a[62:52]; mx = {1'b1, a[51:0]};
                sy = b[63] ^ do_sub; ey = b[62:52]; my = {1'b1, b[51:0]};

                xnan = (ex==11'h7FF) && (a[51:0]!=0);
                ynan = (ey==11'h7FF) && (b[51:0]!=0);
                xinf = (ex==11'h7FF) && (a[51:0]==0);
                yinf = (ey==11'h7FF) && (b[51:0]==0);
                xz   = (ex==0)       && (a[51:0]==0);
                yz   = (ey==0)       && (b[51:0]==0);

                ax = 0; ay = 0; er = 0;
                ediff = 0;

                if (xnan) begin
                    s0_special <= 1; s0_special_val <= a;
                end else if (ynan) begin
                    s0_special <= 1; s0_special_val <= {sy, ey, b[51:0]};
                end else if (xinf && yinf) begin
                    s0_special <= 1;
                    s0_special_val <= (sx==sy) ? {sx,11'h7FF,52'd0} : 64'h7FF8000000000000;
                end else if (xinf) begin
                    s0_special <= 1; s0_special_val <= {sx,11'h7FF,52'd0};
                end else if (yinf) begin
                    s0_special <= 1; s0_special_val <= {sy,11'h7FF,52'd0};
                end else if (xz && yz) begin
                    s0_special <= 1;
                    s0_special_val <= ((sx==1)&&(sy==1)) ? 64'h8000000000000000 : 64'd0;
                end else if (xz) begin
                    s0_special <= 1; s0_special_val <= {sy,ey,b[51:0]};
                end else if (yz) begin
                    s0_special <= 1; s0_special_val <= {sx,ex,a[51:0]};
                end else begin
                    if (ex >= ey) begin
                        ediff = ex - ey; er = ex;
                        ax = {1'b0, mx, 2'b0};
                        ay = (ediff >= 56) ? 56'd0 : ({1'b0, my, 2'b0} >> ediff);
                    end else begin
                        ediff = ey - ex; er = ey;
                        ay = {1'b0, my, 2'b0};
                        ax = (ediff >= 56) ? 56'd0 : ({1'b0, mx, 2'b0} >> ediff);
                    end
                    s0_er <= er;
                    s0_ax <= ax;
                    s0_ay <= ay;
                end

                s0_sx <= sx;
                s0_sy <= sy;
            end
        end
    end

    // ================================================================
    // stage 1: add or subtract aligned mantissas
    // ================================================================
    always @(posedge clk) begin
        if (reset) begin
            s1_valid <= 0;
        end else begin
            s1_valid       <= s0_valid;
            s1_tag         <= s0_tag;
            s1_er          <= s0_er;
            s1_special     <= s0_special;
            s1_special_val <= s0_special_val;

            if (s0_sx == s0_sy) begin
                s1_sum <= {1'b0, s0_ax} + {1'b0, s0_ay};
                s1_sr  <= s0_sx;
            end else if (s0_ax >= s0_ay) begin
                s1_sum <= {1'b0, s0_ax} - {1'b0, s0_ay};
                s1_sr  <= s0_sx;
            end else begin
                s1_sum <= {1'b0, s0_ay} - {1'b0, s0_ax};
                s1_sr  <= s0_sy;
            end
        end
    end

    // ================================================================
    // stage 2: normalize + round → output
    // ================================================================
    always @(posedge clk) begin
        if (reset) begin
            valid_out <= 0;
        end else begin
            valid_out   <= s1_valid;
            rob_tag_out <= s1_tag;

            if (s1_special) begin
                result <= s1_special_val;
            end else if (s1_sum == 0) begin
                result <= 64'd0;
            end else begin : s2_norm
                reg [56:0] s;
                reg [10:0] er;
                reg [53:0] mr;
                reg        guard, rb, sticky;
                integer    shift_cnt;

                s  = s1_sum;
                er = s1_er;

                if (s[55]) begin
                    // carry out — shift right 1
                    mr    = {1'b0, s[55:3]};
                    guard = s[2]; rb = s[1]; sticky = s[0];
                    er    = er + 1;
                end else begin
                    // shift left until leading 1 at bit 54
                    shift_cnt = 0;
                    while (s[54] == 0 && s != 0 && shift_cnt < 56) begin
                        s = s << 1;
                        shift_cnt = shift_cnt + 1;
                    end
                    er    = er - shift_cnt[10:0];
                    mr    = {1'b0, s[54:2]};
                    guard = s[1]; rb = s[0]; sticky = 1'b0;
                end

                if (guard && (rb || sticky || mr[0])) mr = mr + 54'd1;
                if (mr[53]) begin mr = mr >> 1; er = er + 1; end

                result <= {s1_sr, er, mr[51:0]};
            end
        end
    end

endmodule


// ---------------------------------------------------------------------------
// fpu_mul — 3-stage pipelined fp multiply
// stage 0: unpack, detect specials, compute biased result exponent
// stage 1: 53×53 → 106-bit integer mantissa product
// stage 2: normalize + round → output
// ---------------------------------------------------------------------------
module fpu_mul (
    input         clk,
    input         reset,
    input         valid_in,
    input  [63:0] a,
    input  [63:0] b,
    input  [5:0]  rob_tag_in,
    output reg        valid_out,
    output reg [63:0] result,
    output reg [5:0]  rob_tag_out
);

    // stage 0 regs
    reg        s0_valid;
    reg [5:0]  s0_tag;
    reg        s0_special;
    reg [63:0] s0_special_val;
    reg        s0_sr;
    reg [10:0] s0_er;
    reg [52:0] s0_mx, s0_my;

    // stage 1 regs
    reg        s1_valid;
    reg [5:0]  s1_tag;
    reg        s1_special;
    reg [63:0] s1_special_val;
    reg        s1_sr;
    reg [10:0] s1_er;
    reg [105:0] s1_prod;

    // ---- stage 0 ----
    always @(posedge clk) begin
        if (reset) begin
            s0_valid <= 0;
        end else begin
            s0_valid   <= valid_in;
            s0_tag     <= rob_tag_in;
            s0_special <= 0; s0_special_val <= 0;
            s0_sr<=0; s0_er<=0; s0_mx<=0; s0_my<=0;

            if (valid_in) begin : s0l
                reg        sx, sy, sr;
                reg [10:0] ex, ey, er;
                reg [52:0] mx, my;
                reg        xnan, ynan, xinf, yinf, xz, yz;

                sx=a[63]; ex=a[62:52]; mx={1'b1,a[51:0]};
                sy=b[63]; ey=b[62:52]; my={1'b1,b[51:0]};
                sr=sx^sy;
                er=ex+ey-11'd1023;

                xnan=(ex==11'h7FF)&&(a[51:0]!=0);
                ynan=(ey==11'h7FF)&&(b[51:0]!=0);
                xinf=(ex==11'h7FF)&&(a[51:0]==0);
                yinf=(ey==11'h7FF)&&(b[51:0]==0);
                xz  =(ex==0)      &&(a[51:0]==0);
                yz  =(ey==0)      &&(b[51:0]==0);

                if      (xnan)          begin s0_special<=1; s0_special_val<=a; end
                else if (ynan)          begin s0_special<=1; s0_special_val<=b; end
                else if ((xinf&&yz)||(xz&&yinf))
                                        begin s0_special<=1; s0_special_val<=64'h7FF8000000000000; end
                else if (xinf||yinf)    begin s0_special<=1; s0_special_val<={sr,11'h7FF,52'd0}; end
                else if (xz||yz)        begin s0_special<=1; s0_special_val<={sr,63'd0}; end
                else begin
                    s0_sr<=sr; s0_er<=er; s0_mx<=mx; s0_my<=my;
                end
            end
        end
    end

    // ---- stage 1: product ----
    always @(posedge clk) begin
        if (reset) begin s1_valid<=0; end
        else begin
            s1_valid       <= s0_valid;
            s1_tag         <= s0_tag;
            s1_special     <= s0_special;
            s1_special_val <= s0_special_val;
            s1_sr          <= s0_sr;
            s1_er          <= s0_er;
            s1_prod        <= s0_mx * s0_my;
        end
    end

    // ---- stage 2: normalize + round ----
    always @(posedge clk) begin
        if (reset) begin valid_out<=0; end
        else begin
            valid_out   <= s1_valid;
            rob_tag_out <= s1_tag;

            if (s1_special) begin
                result <= s1_special_val;
            end else begin : s2l
                reg [53:0] mr;
                reg [10:0] er;
                reg        guard, rb, sticky;

                er = s1_er;
                if (s1_prod[105]) begin
                    mr    = {1'b0, s1_prod[105:53]};
                    guard = s1_prod[52]; rb = s1_prod[51];
                    sticky= |s1_prod[50:0]; er=er+1;
                end else begin
                    mr    = {1'b0, s1_prod[104:52]};
                    guard = s1_prod[51]; rb = s1_prod[50];
                    sticky= |s1_prod[49:0];
                end

                if (guard&&(rb||sticky||mr[0])) mr=mr+54'd1;
                if (mr[53]) begin mr=mr>>1; er=er+1; end
                result <= {s1_sr, er, mr[51:0]};
            end
        end
    end

endmodule


// ---------------------------------------------------------------------------
// fpu_div — 3-stage pipelined fp divide
// stage 0: unpack
// stage 1: integer division for 54-bit quotient
// stage 2: normalize + round → output
// ---------------------------------------------------------------------------
module fpu_div (
    input         clk,
    input         reset,
    input         valid_in,
    input  [63:0] a,
    input  [63:0] b,
    input  [5:0]  rob_tag_in,
    output reg        valid_out,
    output reg [63:0] result,
    output reg [5:0]  rob_tag_out
);

    // stage 0 regs
    reg        s0_valid;
    reg [5:0]  s0_tag;
    reg        s0_special;
    reg [63:0] s0_special_val;
    reg        s0_sr;
    reg [12:0] s0_er;
    reg [52:0] s0_mx, s0_my;

    // stage 1 regs
    reg        s1_valid;
    reg [5:0]  s1_tag;
    reg        s1_special;
    reg [63:0] s1_special_val;
    reg        s1_sr;
    reg [12:0] s1_er;
    reg [105:0] s1_qr, s1_rem;

    // ---- stage 0 ----
    always @(posedge clk) begin
        if (reset) begin s0_valid<=0; end
        else begin
            s0_valid   <= valid_in;
            s0_tag     <= rob_tag_in;
            s0_special <= 0; s0_special_val <= 0;
            s0_sr<=0; s0_er<=0; s0_mx<=0; s0_my<=0;

            if (valid_in) begin : s0l
                reg        sx, sy, sr;
                reg signed [12:0] ex, ey, er;
                reg [52:0] mx, my;
                reg        xnan, ynan, xinf, yinf, xz, yz;

                sx=a[63]; sy=b[63]; sr=sx^sy;
                mx=(a[62:52]==0)?{1'b0,a[51:0]}:{1'b1,a[51:0]};
                my=(b[62:52]==0)?{1'b0,b[51:0]}:{1'b1,b[51:0]};

                xnan=(a[62:52]==11'h7FF)&&(a[51:0]!=0);
                ynan=(b[62:52]==11'h7FF)&&(b[51:0]!=0);
                xinf=(a[62:52]==11'h7FF)&&(a[51:0]==0);
                yinf=(b[62:52]==11'h7FF)&&(b[51:0]==0);
                xz  =(a[62:52]==0)      &&(a[51:0]==0);
                yz  =(b[62:52]==0)      &&(b[51:0]==0);

                if (a[62:52]==0) ex=13'd1; else ex={2'b0,a[62:52]};
                if (b[62:52]==0) ey=13'd1; else ey={2'b0,b[62:52]};
                er=ex-ey+13'd1022;

                if      (xnan)         begin s0_special<=1; s0_special_val<=a; end
                else if (ynan)         begin s0_special<=1; s0_special_val<=b; end
                else if (xinf&&yinf)   begin s0_special<=1; s0_special_val<=64'h7FF8000000000000; end
                else if (xz&&yz)       begin s0_special<=1; s0_special_val<=64'h7FF8000000000000; end
                else if (xinf)         begin s0_special<=1; s0_special_val<={sr,11'h7FF,52'd0}; end
                else if (yinf)         begin s0_special<=1; s0_special_val<={sr,63'd0}; end
                else if (yz)           begin s0_special<=1; s0_special_val<={sr,11'h7FF,52'd0}; end
                else if (xz)           begin s0_special<=1; s0_special_val<={sr,63'd0}; end
                else begin
                    s0_sr<=sr; s0_er<=er; s0_mx<=mx; s0_my<=my;
                end
            end
        end
    end

    // ---- stage 1: integer division ----
    always @(posedge clk) begin
        if (reset) begin s1_valid<=0; end
        else begin
            s1_valid       <= s0_valid;
            s1_tag         <= s0_tag;
            s1_special     <= s0_special;
            s1_special_val <= s0_special_val;
            s1_sr          <= s0_sr;
            s1_er          <= s0_er;
            begin : div_op
                reg [105:0] num;
                num    = {s0_mx, 53'd0};
                s1_qr  <= (s0_my!=0) ? (num / {53'd0, s0_my}) : 106'd0;
                s1_rem <= (s0_my!=0) ? (num % {53'd0, s0_my}) : 106'd0;
            end
        end
    end

    // ---- stage 2: normalize + round ----
    always @(posedge clk) begin
        if (reset) begin valid_out<=0; end
        else begin
            valid_out   <= s1_valid;
            rob_tag_out <= s1_tag;

            if (s1_special) begin
                result <= s1_special_val;
            end else begin : s2l
                reg [53:0] mr;
                reg [12:0] er;
                reg        guard, rb, sticky;

                er=s1_er;
                if (s1_qr[53]) begin
                    mr    ={1'b0,s1_qr[53:1]};
                    guard =s1_qr[0]; rb=1'b0;
                    sticky=(s1_rem!=0); er=er+1;
                end else begin
                    mr    ={1'b0,s1_qr[52:0]};
                    guard =1'b0; rb=1'b0;
                    sticky=(s1_rem!=0);
                end

                if (guard&&(rb||sticky||mr[0])) mr=mr+54'd1;
                if (mr[53]) begin mr=mr>>1; er=er+1; end
                if (er<=0) begin mr=mr>>(1-er[10:0]); er=0; end
                result <= {s1_sr, er[10:0], mr[51:0]};
            end
        end
    end

endmodule