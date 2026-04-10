// tb_alu.v — unit tests for the alu module
module tb_alu;

  reg [63:0] a, b;
  reg  [ 4:0] op;
  wire [63:0] result;

  alu dut (
      .a(a),
      .b(b),
      .op(op),
      .result(result)
  );

  integer pass_count, fail_count;

  task check;
    input [63:0] expected;
    input [255:0] name;
    begin
      if (result === expected) begin
        $display("  pass [%s]: got 0x%016h", name, result);
        pass_count = pass_count + 1;
      end else begin
        $display("  FAIL [%s]: got 0x%016h  expected 0x%016h", name, result, expected);
        fail_count = fail_count + 1;
      end
    end
  endtask

  initial begin
    $dumpfile("sim/tb_alu.vcd");
    $dumpvars(0, tb_alu);
    pass_count = 0;
    fail_count = 0;

    $display("\n--- int ops ---");
    op = 5'd0;
    a  = 64'd20;
    b  = 64'd30;
    #1;
    check(64'd50, "add");
    op = 5'd1;
    a  = 64'd100;
    b  = 64'd40;
    #1;
    check(64'd60, "sub");
    op = 5'd1;
    a  = 64'd5;
    b  = 64'd10;
    #1;
    check(64'hFFFFFFFFFFFFFFFB, "sub_wrap");
    op = 5'd2;
    a  = 64'd6;
    b  = 64'd7;
    #1;
    check(64'd42, "mul");
    op = 5'd2;
    a  = 64'd99;
    b  = 64'd0;
    #1;
    check(64'd0, "mul_zero");
    op = 5'd3;
    a  = 64'd100;
    b  = 64'd4;
    #1;
    check(64'd25, "div");
    op = 5'd3;
    a  = 64'd7;
    b  = 64'd0;
    #1;
    check(64'd0, "div_zero");

    $display("\n--- bitwise ---");
    op = 5'd4;
    a  = 64'hF0;
    b  = 64'hFF;
    #1;
    check(64'hF0, "and");
    op = 5'd4;
    a  = 64'hFF;
    b  = 64'd0;
    #1;
    check(64'd0, "and_zero");
    op = 5'd5;
    a  = 64'hF0;
    b  = 64'h0F;
    #1;
    check(64'hFF, "or");
    op = 5'd6;
    a  = 64'hFF;
    b  = 64'h0F;
    #1;
    check(64'hF0, "xor");
    op = 5'd7;
    a  = 64'd0;
    b  = 64'd0;
    #1;
    check(64'hFFFFFFFFFFFFFFFF, "not");

    $display("\n--- shifts ---");
    op = 5'd8;
    a  = 64'h80;
    b  = 64'd3;
    #1;
    check(64'h10, "shftr");
    op = 5'd9;
    a  = 64'd1;
    b  = 64'd8;
    #1;
    check(64'd256, "shftl");
    op = 5'd8;
    a  = 64'd42;
    b  = 64'd0;
    #1;
    check(64'd42, "shftr_0");

    $display("\n--- fadd ---");
    op = 5'd10;
    a  = 64'h3FF0000000000000;
    b  = 64'h4000000000000000;
    #1;
    check(64'h4008000000000000, "1+2=3");
    a = 64'h3FF0000000000000;
    b = 64'hBFF0000000000000;
    #1;
    check(64'h0000000000000000, "1+(-1)=0");
    a = 64'h7FF0000000000000;
    b = 64'h7FF0000000000000;
    #1;
    check(64'h7FF0000000000000, "inf+inf");
    a = 64'h7FF0000000000000;
    b = 64'hFFF0000000000000;
    #1;
    check(64'h7FF8000000000000, "inf-inf=nan");
    a = 64'h7FF8000000000000;
    b = 64'h3FF0000000000000;
    #1;
    check(64'h7FF8000000000000, "nan+1=nan");
    a = 64'h8000000000000000;
    b = 64'h8000000000000000;
    #1;
    check(64'h8000000000000000, "-0+-0=-0");

    $display("\n--- fsub ---");
    op = 5'd11;
    a  = 64'h4008000000000000;
    b  = 64'h3FF0000000000000;
    #1;
    check(64'h4000000000000000, "3-1=2");
    a = 64'h3FF0000000000000;
    b = 64'h4000000000000000;
    #1;
    check(64'hBFF0000000000000, "1-2=-1");
    a = 64'h4000000000000000;
    b = 64'h0000000000000000;
    #1;
    check(64'h4000000000000000, "x-0=x");
    a = 64'h7FF0000000000000;
    b = 64'h7FF0000000000000;
    #1;
    check(64'h7FF8000000000000, "inf-inf=nan");
    a = 64'h7FF8000000000001;
    b = 64'h3FF0000000000000;
    #1;
    check(64'h7FF8000000000001, "nan-1=nan");

    $display("\n--- fmul ---");
    op = 5'd12;
    a  = 64'h4000000000000000;
    b  = 64'h4008000000000000;
    #1;
    check(64'h4018000000000000, "2*3=6");
    a = 64'hBFF0000000000000;
    b = 64'h4000000000000000;
    #1;
    check(64'hC000000000000000, "-1*2=-2");
    a = 64'h7FF0000000000000;
    b = 64'h0000000000000000;
    #1;
    check(64'h7FF8000000000000, "inf*0=nan");
    a = 64'h7FF0000000000000;
    b = 64'h4000000000000000;
    #1;
    check(64'h7FF0000000000000, "inf*2=inf");
    a = 64'hFFF0000000000000;
    b = 64'h4000000000000000;
    #1;
    check(64'hFFF0000000000000, "-inf*2=-inf");
    a = 64'h8000000000000000;
    b = 64'h4000000000000000;
    #1;
    check(64'h8000000000000000, "-0*2=-0");
    a = 64'h0010000000000000;
    b = 64'h4000000000000000;
    #1;
    check(64'h0020000000000000, "subn*2");
    a = 64'h7FF8000000000000;
    b = 64'h4000000000000000;
    #1;
    check(64'h7FF8000000000000, "nan*2=nan");
    a = 64'h3FD5555555555555;
    b = 64'h4008000000000000;
    #1;
    check(64'h3FF0000000000000, "fmul_round");

    $display("\n--- fdiv ---");
    op = 5'd13;
    a  = 64'h3FF0000000000000;
    b  = 64'h4000000000000000;
    #1;
    check(64'h3FE0000000000000, "1/2=0.5");
    a = 64'h4000000000000000;
    b = 64'h3FF0000000000000;
    #1;
    check(64'h4000000000000000, "2/1=2");
    a = 64'h4018000000000000;
    b = 64'h4000000000000000;
    #1;
    check(64'h4008000000000000, "6/2=3");
    a = 64'h3FF0000000000000;
    b = 64'h4008000000000000;
    #1;
    check(64'h3FD5555555555555, "1/3_round");
    a = 64'h7FF0000000000000;
    b = 64'h7FF0000000000000;
    #1;
    check(64'h7FF8000000000000, "inf/inf=nan");
    a = 64'h0000000000000000;
    b = 64'h0000000000000000;
    #1;
    check(64'h7FF8000000000000, "0/0=nan");
    a = 64'h3FF0000000000000;
    b = 64'h0000000000000000;
    #1;
    check(64'h7FF0000000000000, "1/0=inf");
    a = 64'h0000000000000000;
    b = 64'h3FF0000000000000;
    #1;
    check(64'h0000000000000000, "0/1=0");
    a = 64'h7FF0000000000000;
    b = 64'h4000000000000000;
    #1;
    check(64'h7FF0000000000000, "inf/2=inf");
    a = 64'h4000000000000000;
    b = 64'h7FF0000000000000;
    #1;
    check(64'h0000000000000000, "2/inf=0");
    a = 64'h7FF8000000000000;
    b = 64'h3FF0000000000000;
    #1;
    check(64'h7FF8000000000000, "nan/1=nan");
    a = 64'h3FF0000000000000;
    b = 64'h7FF8000000000000;
    #1;
    check(64'h7FF8000000000000, "1/nan=nan");
    a = 64'h0010000000000000;
    b = 64'h4000000000000000;
    #1;
    check(64'h0008000000000000, "norm/2=subn");
    a = 64'h0000000000000001;
    b = 64'h3FF0000000000000;
    #1;
    check(64'h0000000000000001, "tiny/1=tiny");
    a = 64'hBFF0000000000000;
    b = 64'h4000000000000000;
    #1;
    check(64'hBFE0000000000000, "-1/2=-0.5");

    $display("\n--- results: %0d passed, %0d failed ---", pass_count, fail_count);
    $finish;
  end
endmodule
