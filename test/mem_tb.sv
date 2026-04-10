// tb_mem.v — unit tests for mem_module
module tb_mem;

  reg clk;
  reg [63:0] fetch_addr, data_addr, write_data;
  reg         we;
  wire [31:0] instr_out;
  wire [63:0] read_data;

  mem_module dut (
      .clk       (clk),
      .fetch_addr(fetch_addr),
      .instr_out (instr_out),
      .data_addr (data_addr),
      .write_data(write_data),
      .we        (we),
      .read_data (read_data)
  );

  always #5 clk = ~clk;

  integer pass_count, fail_count;

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

  task check32;
    input [31:0] expected, got;
    input [255:0] name;
    begin
      if (got === expected) begin
        $display("  pass [%s]: 0x%08h", name, got);
        pass_count = pass_count + 1;
      end else begin
        $display("  FAIL [%s]: got 0x%08h  exp 0x%08h", name, got, expected);
        fail_count = fail_count + 1;
      end
    end
  endtask

  task mem_write64;
    input [63:0] addr, val;
    begin
      we = 1;
      data_addr = addr;
      write_data = val;
      @(posedge clk);
      #1;
      we = 0;
    end
  endtask

  task mem_read64;
    input [63:0] addr;
    begin
      we = 0;
      data_addr = addr;
      @(posedge clk);
      #1;
    end
  endtask

  initial begin
    $dumpfile("sim/tb_mem.vcd");
    $dumpvars(0, tb_mem);
    clk = 0;
    pass_count = 0;
    fail_count = 0;
    we = 0;
    fetch_addr = 0;
    data_addr = 0;
    write_data = 0;
    @(posedge clk);
    @(posedge clk);

    // basic store/load via data port
    $display("\n--- basic r/w ---");
    mem_write64(64'h100, 64'hDEADBEEFCAFEBABE);
    mem_read64(64'h100);
    check64(64'hDEADBEEFCAFEBABE, read_data, "rw_0x100");

    mem_write64(64'h108, 64'd42);
    mem_read64(64'h108);
    check64(64'd42, read_data, "rw_0x108");

    // adjacent addrs don't alias
    mem_write64(64'h200, 64'd11);
    mem_write64(64'h208, 64'd99);
    mem_read64(64'h200);
    check64(64'd11, read_data, "no_alias_lo");
    mem_read64(64'h208);
    check64(64'd99, read_data, "no_alias_hi");

    // overwrite
    $display("\n--- overwrite ---");
    mem_write64(64'h300, 64'd1);
    mem_write64(64'h300, 64'd2);
    mem_read64(64'h300);
    check64(64'd2, read_data, "overwrite");

    // edge values
    $display("\n--- edge values ---");
    mem_write64(64'h400, 64'd0);
    mem_read64(64'h400);
    check64(64'd0, read_data, "zero_val");

    mem_write64(64'h500, 64'hFFFFFFFFFFFFFFFF);
    mem_read64(64'h500);
    check64(64'hFFFFFFFFFFFFFFFF, read_data, "all_ones");

    // fp bit pattern round-trips
    $display("\n--- fp bit patterns ---");
    mem_write64(64'h600, 64'h3FF0000000000000);
    mem_read64(64'h600);
    check64(64'h3FF0000000000000, read_data, "fp_1.0");

    mem_write64(64'h608, 64'h7FF0000000000000);
    mem_read64(64'h608);
    check64(64'h7FF0000000000000, read_data, "fp_inf");

    mem_write64(64'h610, 64'h7FF8000000000000);
    mem_read64(64'h610);
    check64(64'h7FF8000000000000, read_data, "fp_nan");

    // fetch port: little-endian 32-bit instr read
    $display("\n--- fetch port ---");
    dut.bytes[64'h2000] = 8'hAB;
    dut.bytes[64'h2001] = 8'hCD;
    dut.bytes[64'h2002] = 8'hEF;
    dut.bytes[64'h2003] = 8'h01;
    fetch_addr = 64'h2000;
    @(posedge clk);
    #1;
    check32(32'h01EFCDAB, instr_out, "instr_le");

    $display("\n--- results: %0d passed, %0d failed ---", pass_count, fail_count);
    $finish;
  end
endmodule
