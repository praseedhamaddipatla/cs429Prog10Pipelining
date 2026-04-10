// tb_regfile.sv — unit tests for reg_file module
module tb_regfile;

  reg clk, reset, write;
  reg [4:0] raddr1, raddr2, raddr3, waddr;
  reg [63:0] data;
  wire [63:0] r1, r2, r3;

  reg_file dut (
      .clk   (clk),
      .reset (reset),
      .raddr1(raddr1),
      .raddr2(raddr2),
      .raddr3(raddr3),
      .waddr (waddr),
      .data  (data),
      .write (write),
      .r1    (r1),
      .r2    (r2),
      .r3    (r3)
  );

  always #5 clk = ~clk;

  integer pass_count, fail_count, i;

  task check64;
    input [63:0] expected, got;
    input [255:0] name;
    begin
      if (got === expected) begin
        $display("  pass [%s]: 0x%016h", name, got);
        pass_count = pass_count + 1;
      end else begin
        $display("  FAIL [%s]: got 0x%016h  exp 0x%016h", name, got, expected);
        fail_count = fail_count + 1;
      end
    end
  endtask

  task do_reset;
    begin
      reset = 1;
      write = 0;
      @(posedge clk);
      @(posedge clk);
      #1;
      reset = 0;
    end
  endtask

  task write_reg;
    input [4:0] reg_num;
    input [63:0] val;
    begin
      write = 1;
      waddr = reg_num;
      data  = val;
      @(posedge clk);
      #1;
      write = 0;
    end
  endtask

  initial begin
    $dumpfile("sim/tb_regfile.vcd");
    $dumpvars(0, tb_regfile);
    clk = 0;
    pass_count = 0;
    fail_count = 0;
    raddr1 = 0;
    raddr2 = 0;
    raddr3 = 0;
    do_reset();

    // reset: r0-r30=0, r31=stack ptr (512*1024=524288)
    $display("\n--- reset state ---");
    begin
      integer all_zero;
      all_zero = 1;
      for (i = 0; i < 31; i = i + 1) begin
        raddr1 = i;
        #1;
        if (r1 !== 64'd0) all_zero = 0;
      end
      if (all_zero) begin
        $display("  pass [r0-r30_zero]");
        pass_count = pass_count + 1;
      end else begin
        $display("  FAIL [r0-r30_zero]");
        fail_count = fail_count + 1;
      end
    end
    raddr1 = 5'd31;
    #1;
    check64(64'd524288, r1, "reset_r31");

    // basic write/read via r1
    $display("\n--- read/write ---");
    write_reg(5'd1, 64'hDEADBEEFCAFEBABE);
    raddr1 = 5'd1;
    #1;
    check64(64'hDEADBEEFCAFEBABE, r1, "wr_r1");

    write_reg(5'd15, 64'd12345);
    raddr1 = 5'd15;
    #1;
    check64(64'd12345, r1, "wr_r15");

    // simultaneous two-port read r1+r2
    write_reg(5'd2, 64'd100);
    write_reg(5'd3, 64'd200);
    raddr1 = 5'd2;
    raddr2 = 5'd3;
    #1;
    check64(64'd100, r1, "2port_r2");
    check64(64'd200, r2, "2port_r3");

    // third port r3 via raddr3 (brgt rt)
    write_reg(5'd7, 64'd777);
    raddr3 = 5'd7;
    #1;
    check64(64'd777, r3, "3port_r7");

    // overwrite
    write_reg(5'd1, 64'd999);
    raddr1 = 5'd1;
    #1;
    check64(64'd999, r1, "overwrite_r1");

    // r31 writable (call/return update stack ptr)
    write_reg(5'd31, 64'd0);
    raddr1 = 5'd31;
    #1;
    check64(64'd0, r1, "wr_r31");

    // write=0 gates write
    $display("\n--- write gating ---");
    write_reg(5'd5, 64'd42);
    write = 0;
    waddr = 5'd5;
    data  = 64'd999;
    @(posedge clk);
    #1;
    raddr1 = 5'd5;
    #1;
    check64(64'd42, r1, "write_gated");

    // re-reset
    $display("\n--- re-reset ---");
    write_reg(5'd8, 64'hFFFFFFFFFFFFFFFF);
    do_reset();
    raddr1 = 5'd8;
    #1;
    check64(64'd0, r1, "rereset_r8");
    raddr1 = 5'd31;
    #1;
    check64(64'd524288, r1, "rereset_r31");

    $display("\n--- results: %0d passed, %0d failed ---", pass_count, fail_count);
    $finish;
  end
endmodule