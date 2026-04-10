module alu (
    input [63:0] a,
    input [63:0] b,
    input [4:0] op,
    output reg [63:0] result
);

  // fp_add: handles fadd and fsub (do_sub flips sign of y)
  function automatic [63:0] fp_add;
    input [63:0] x, y;
    input do_sub;
    reg sx, sy, sr;
    reg [10:0] ex, ey, er;
    reg [52:0] mx, my;
    reg [10:0] ediff;
    reg [55:0] ax, ay;
    reg [56:0] sum;
    reg [53:0] mr;
    reg guard, round_bit, sticky;
    reg x_nan, y_nan, x_inf, y_inf, x_zero, y_zero;
    begin
      sx     = x[63];
      ex     = x[62:52];
      mx     = {1'b1, x[51:0]};  // implicit leading 1
      sy     = y[63];
      ey     = y[62:52];
      my     = {1'b1, y[51:0]};
      sy     = sy ^ do_sub;  // flip y sign for sub

      x_nan  = (ex == 11'h7FF) && (x[51:0] != 0);
      y_nan  = (ey == 11'h7FF) && (y[51:0] != 0);
      x_inf  = (ex == 11'h7FF) && (x[51:0] == 0);
      y_inf  = (ey == 11'h7FF) && (y[51:0] == 0);
      x_zero = (ex == 0) && (x[51:0] == 0);
      y_zero = (ey == 0) && (y[51:0] == 0);

      if (x_nan) begin
        fp_add = x;
      end else if (y_nan) begin
        fp_add = {sy, ey, y[51:0]};  // propagate y nan w/ possibly flipped sign
      end else if (x_inf && y_inf) begin
        if (sx == sy) fp_add = {sx, 11'h7FF, 52'd0};  // same sign => inf
        else fp_add = 64'h7FF8000000000000;  // opposite => nan
      end else if (x_inf) begin
        fp_add = {sx, 11'h7FF, 52'd0};
      end else if (y_inf) begin
        fp_add = {sy, 11'h7FF, 52'd0};
      end else if (x_zero && y_zero) begin
        fp_add = ((sx == 1) && (sy == 1)) ? 64'h8000000000000000 : 64'd0;  // -0+-0=-0
      end else if (x_zero) begin
        fp_add = {sy, ey, y[51:0]};
      end else if (y_zero) begin
        fp_add = {sx, ex, x[51:0]};
      end else begin
        // align mantissas to larger exp
        if (ex >= ey) begin
          ediff = ex - ey;
          ax    = {1'b0, mx, 2'b0};
          ay    = (ediff >= 56) ? 56'd0 : ({1'b0, my, 2'b0} >> ediff);
          er    = ex;
        end else begin
          ediff = ey - ex;
          ay    = {1'b0, my, 2'b0};
          ax    = (ediff >= 56) ? 56'd0 : ({1'b0, mx, 2'b0} >> ediff);
          er    = ey;
        end

        // add or sub aligned mantissas
        if (sx == sy) begin
          sum = ax + ay;
          sr  = sx;
        end else if (ax >= ay) begin
          sum = ax - ay;
          sr  = sx;
        end else begin
          sum = ay - ax;
          sr  = sy;
        end

        if (sum == 0) begin
          fp_add = 64'd0;
        end else begin
          if (sum[55]) begin
            // carry out: shift right 1, bump exp
            mr        = {1'b0, sum[55:3]};
            guard     = sum[2];
            round_bit = sum[1];
            sticky    = sum[0];
            er        = er + 1;
          end else begin
            // normalize: shift left until leading 1 at bit 54
            begin : norm_loop
              reg [56:0] s;
              s = sum;
              while (s[54] == 0 && s != 0) begin
                s  = s << 1;
                er = er - 1;
              end
              mr        = {1'b0, s[54:2]};
              guard     = s[1];
              round_bit = s[0];
              sticky    = 1'b0;
            end
          end

          // round to nearest even
          if (guard && (round_bit || sticky || mr[0])) mr = mr + 54'd1;

          // rounding carry: renormalize
          if (mr[53]) begin
            mr = mr >> 1;
            er = er + 1;
          end

          fp_add = {sr, er, mr[51:0]};
        end
      end
    end
  endfunction


  function automatic [63:0] fp_mul;
    input [63:0] x, y;
    reg sx, sy, sr;
    reg [10:0] ex, ey, er;
    reg [52:0] mx, my;
    reg [105:0] prod;
    reg [ 53:0] mr;
    reg guard, round_bit, sticky;
    reg x_nan, y_nan, x_inf, y_inf, x_zero, y_zero;
    begin
      sx     = x[63];
      ex     = x[62:52];
      mx     = {1'b1, x[51:0]};
      sy     = y[63];
      ey     = y[62:52];
      my     = {1'b1, y[51:0]};
      sr     = sx ^ sy;  // result sign

      x_nan  = (ex == 11'h7FF) && (x[51:0] != 0);
      y_nan  = (ey == 11'h7FF) && (y[51:0] != 0);
      x_inf  = (ex == 11'h7FF) && (x[51:0] == 0);
      y_inf  = (ey == 11'h7FF) && (y[51:0] == 0);
      x_zero = (ex == 0) && (x[51:0] == 0);
      y_zero = (ey == 0) && (y[51:0] == 0);

      if (x_nan) begin
        fp_mul = x;
      end else if (y_nan) begin
        fp_mul = y;
      end else if ((x_inf && y_zero) || (x_zero && y_inf)) begin
        fp_mul = 64'h7FF8000000000000;  // inf*0 => nan
      end else if (x_inf || y_inf) begin
        fp_mul = {sr, 11'h7FF, 52'd0};  // inf*finite => inf
      end else if (x_zero || y_zero) begin
        fp_mul = {sr, 63'd0};  // 0*anything => signed zero
      end else begin
        // unbias: result exp = ex + ey - 1023
        er   = ex + ey - 11'd1023;
        prod = mx * my;  // 106-bit product, leading 1 at bit 104 or 105

        if (prod[105]) begin
          // leading 1 at 105: shift right 1, bump exp
          mr        = {1'b0, prod[105:53]};
          guard     = prod[52];
          round_bit = prod[51];
          sticky    = |prod[50:0];
          er        = er + 1;
        end else begin
          // leading 1 at 104
          mr        = {1'b0, prod[104:52]};
          guard     = prod[51];
          round_bit = prod[50];
          sticky    = |prod[49:0];
        end

        if (guard && (round_bit || sticky || mr[0])) mr = mr + 54'd1;

        if (mr[53]) begin
          mr = mr >> 1;
          er = er + 1;
        end

        fp_mul = {sr, er, mr[51:0]};
      end
    end
  endfunction


  function automatic [63:0] fp_div;
    input [63:0] x, y;
    reg sx, sy, sr;
    reg signed [12:0] ex, ey, er;
    reg [52:0] mx, my;
    reg [105:0] num;
    reg [105:0] qr;
    reg [105:0] rem;
    reg [ 53:0] mr;
    reg guard, round_bit, sticky;
    reg x_nan, y_nan, x_inf, y_inf, x_zero, y_zero;
    begin
      sx     = x[63];
      ex     = {2'b0, x[62:52]};
      sy     = y[63];
      ey     = {2'b0, y[62:52]};
      sr     = sx ^ sy;

      // no implicit leading 1 for subnormals
      mx     = (x[62:52] == 0) ? {1'b0, x[51:0]} : {1'b1, x[51:0]};
      my     = (y[62:52] == 0) ? {1'b0, y[51:0]} : {1'b1, y[51:0]};

      x_nan  = (x[62:52] == 11'h7FF) && (x[51:0] != 0);
      y_nan  = (y[62:52] == 11'h7FF) && (y[51:0] != 0);
      x_inf  = (x[62:52] == 11'h7FF) && (x[51:0] == 0);
      y_inf  = (y[62:52] == 11'h7FF) && (y[51:0] == 0);
      x_zero = (x[62:52] == 0) && (x[51:0] == 0);
      y_zero = (y[62:52] == 0) && (y[51:0] == 0);

      if (x_nan) begin
        fp_div = x;
      end else if (y_nan) begin
        fp_div = y;
      end else if (x_inf && y_inf) begin
        fp_div = 64'h7FF8000000000000;  // inf/inf => nan
      end else if (x_zero && y_zero) begin
        fp_div = 64'h7FF8000000000000;  // 0/0 => nan
      end else if (x_inf) begin
        fp_div = {sr, 11'h7FF, 52'd0};  // inf/finite => inf
      end else if (y_inf) begin
        fp_div = {sr, 63'd0};  // finite/inf => 0
      end else if (y_zero) begin
        fp_div = {sr, 11'h7FF, 52'd0};  // finite/0 => inf
      end else if (x_zero) begin
        fp_div = {sr, 63'd0};  // 0/finite => 0
      end else begin
        // subnormals have true exp=1, not 0
        if (x[62:52] == 0) ex = 13'd1;
        else ex = {2'b0, x[62:52]};
        if (y[62:52] == 0) ey = 13'd1;
        else ey = {2'b0, y[62:52]};

        // base er=ex-ey+1022: shift of 53 means qr leading 1 at bit 52 or 53
        // qr[53] case adds 1 => final er=ex-ey+1023 (for mx>=my)
        // qr[52] case no add => er=ex-ey+1022 (for mx<my)
        er  = ex - ey + 13'd1022;

        num = {mx, 53'd0};  // shift left 53 for integer div precision
        qr  = num / {53'd0, my};
        rem = num % {53'd0, my};

        if (qr[53]) begin
          // mx>=my: leading 1 at qr[53], shift right into mr[52]
          mr        = {1'b0, qr[53:1]};
          guard     = qr[0];
          round_bit = 1'b0;
          sticky    = (rem != 0);
          er        = er + 1;
        end else begin
          // mx<my: leading 1 at qr[52], already in place
          mr        = {1'b0, qr[52:0]};
          guard     = 1'b0;
          round_bit = 1'b0;
          sticky    = (rem != 0);
        end

        if (guard && (round_bit || sticky || mr[0])) mr = mr + 54'd1;

        // rounding carry: renormalize
        if (mr[53]) begin
          mr = mr >> 1;
          er = er + 1;
        end

        // underflow: flush to subnormal
        if (er <= 0) begin
          mr = mr >> (1 - er);
          er = 0;
        end

        fp_div = {sr, er[10:0], mr[51:0]};
      end
    end
  endfunction

  always @(*) begin
    case (op)
      5'd0: result = a + b;
      5'd1: result = a - b;
      5'd2: result = a * b;
      5'd3: result = (b == 0) ? 64'd0 : $signed(a) / $signed(b);  // div by zero => 0

      5'd4: result = a & b;
      5'd5: result = a | b;
      5'd6: result = a ^ b;
      5'd7: result = ~a;
      5'd8: result = a >> b[5:0];  // shift right
      5'd9: result = a << b[5:0];  // shift left

      5'd10: result = fp_add(a, b, 1'b0);  // fadd
      5'd11: result = fp_add(a, b, 1'b1);  // fsub
      5'd12: result = fp_mul(a, b);
      5'd13: result = fp_div(a, b);

      5'd14: result = (a != 64'd0) ? 64'd1 : 64'd0;  // brnz cond
      5'd15: result = ($signed(a) > $signed(b)) ? 64'd1 : 64'd0;  // brgt cond

      default: result = 64'd0;
    endcase
  end

endmodule
