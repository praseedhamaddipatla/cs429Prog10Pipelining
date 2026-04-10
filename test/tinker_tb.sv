// tb_tinker.v — integration tests for multicycle tinker_core
// each test counts cycles and verifies hlt
module tb_tinker;

  reg clk, reset;
  wire hlt;

  tinker_core cpu (
      .clk  (clk),
      .reset(reset),
      .hlt  (hlt)
  );

  always #5 clk = ~clk;

  integer pass_count, fail_count, i;
  integer cycle_count;  // per-test cycle counter

  // cycle counting

  // instruction encoding
  function [31:0] make_add;
    input [4:0] rd, rs, rt;
    make_add = (5'h18 << 27) | (rd << 22) | (rs << 17) | (rt << 12);
  endfunction
  function [31:0] make_addi;
    input [4:0] rd;
    input [11:0] im;
    make_addi = (5'h19 << 27) | (rd << 22) | im;
  endfunction
  function [31:0] make_sub;
    input [4:0] rd, rs, rt;
    make_sub = (5'h1A << 27) | (rd << 22) | (rs << 17) | (rt << 12);
  endfunction
  function [31:0] make_subi;
    input [4:0] rd;
    input [11:0] im;
    make_subi = (5'h1B << 27) | (rd << 22) | im;
  endfunction
  function [31:0] make_mul;
    input [4:0] rd, rs, rt;
    make_mul = (5'h1C << 27) | (rd << 22) | (rs << 17) | (rt << 12);
  endfunction
  function [31:0] make_div;
    input [4:0] rd, rs, rt;
    make_div = (5'h1D << 27) | (rd << 22) | (rs << 17) | (rt << 12);
  endfunction
  function [31:0] make_and;
    input [4:0] rd, rs, rt;
    make_and = (5'h00 << 27) | (rd << 22) | (rs << 17) | (rt << 12);
  endfunction
  function [31:0] make_or;
    input [4:0] rd, rs, rt;
    make_or = (5'h01 << 27) | (rd << 22) | (rs << 17) | (rt << 12);
  endfunction
  function [31:0] make_xor;
    input [4:0] rd, rs, rt;
    make_xor = (5'h02 << 27) | (rd << 22) | (rs << 17) | (rt << 12);
  endfunction
  function [31:0] make_not;
    input [4:0] rd, rs;
    make_not = (5'h03 << 27) | (rd << 22) | (rs << 17);
  endfunction
  function [31:0] make_shftr;
    input [4:0] rd, rs, rt;
    make_shftr = (5'h04 << 27) | (rd << 22) | (rs << 17) | (rt << 12);
  endfunction
  function [31:0] make_shftri;
    input [4:0] rd;
    input [11:0] im;
    make_shftri = (5'h05 << 27) | (rd << 22) | im;
  endfunction
  function [31:0] make_shftl;
    input [4:0] rd, rs, rt;
    make_shftl = (5'h06 << 27) | (rd << 22) | (rs << 17) | (rt << 12);
  endfunction
  function [31:0] make_shftli;
    input [4:0] rd;
    input [11:0] im;
    make_shftli = (5'h07 << 27) | (rd << 22) | im;
  endfunction
  function [31:0] make_br;
    input [4:0] rd;
    make_br = (5'h08 << 27) | (rd << 22);
  endfunction
  function [31:0] make_brr_imm;
    input [11:0] im;
    make_brr_imm = (5'h0A << 27) | im;
  endfunction
  function [31:0] make_brnz;
    input [4:0] rd, rs;
    make_brnz = (5'h0B << 27) | (rd << 22) | (rs << 17);
  endfunction
  function [31:0] make_call;
    input [4:0] rd;
    make_call = (5'h0C << 27) | (rd << 22);
  endfunction
  function [31:0] make_return;
    make_return = (5'h0D << 27);
  endfunction
  function [31:0] make_brgt;
    input [4:0] rd, rs, rt;
    make_brgt = (5'h0E << 27) | (rd << 22) | (rs << 17) | (rt << 12);
  endfunction
  function [31:0] make_halt;
    make_halt = (5'h0F << 27);
  endfunction
  function [31:0] make_load;
    input [4:0] rd, rs;
    input [11:0] im;
    make_load = (5'h10 << 27) | (rd << 22) | (rs << 17) | im;
  endfunction
  function [31:0] make_mov_reg;
    input [4:0] rd, rs;
    make_mov_reg = (5'h11 << 27) | (rd << 22) | (rs << 17);
  endfunction
  function [31:0] make_mov_imm;
    input [4:0] rd;
    input [11:0] im;
    make_mov_imm = (5'h12 << 27) | (rd << 22) | im;
  endfunction
  function [31:0] make_store;
    input [4:0] rd, rs;
    input [11:0] im;
    make_store = (5'h13 << 27) | (rd << 22) | (rs << 17) | im;
  endfunction
  function [31:0] make_addf;
    input [4:0] rd, rs, rt;
    make_addf = (5'h14 << 27) | (rd << 22) | (rs << 17) | (rt << 12);
  endfunction
  function [31:0] make_subf;
    input [4:0] rd, rs, rt;
    make_subf = (5'h15 << 27) | (rd << 22) | (rs << 17) | (rt << 12);
  endfunction
  function [31:0] make_mulf;
    input [4:0] rd, rs, rt;
    make_mulf = (5'h16 << 27) | (rd << 22) | (rs << 17) | (rt << 12);
  endfunction
  function [31:0] make_divf;
    input [4:0] rd, rs, rt;
    make_divf = (5'h17 << 27) | (rd << 22) | (rs << 17) | (rt << 12);
  endfunction

  task write_instr;
    input [63:0] addr;
    input [31:0] word;
    begin
      cpu.memory.bytes[addr]   = word[7:0];
      cpu.memory.bytes[addr+1] = word[15:8];
      cpu.memory.bytes[addr+2] = word[23:16];
      cpu.memory.bytes[addr+3] = word[31:24];
    end
  endtask

  task write_mem64;
    input [63:0] addr, val;
    begin
      cpu.memory.bytes[addr]   = val[7:0];
      cpu.memory.bytes[addr+1] = val[15:8];
      cpu.memory.bytes[addr+2] = val[23:16];
      cpu.memory.bytes[addr+3] = val[31:24];
      cpu.memory.bytes[addr+4] = val[39:32];
      cpu.memory.bytes[addr+5] = val[47:40];
      cpu.memory.bytes[addr+6] = val[55:48];
      cpu.memory.bytes[addr+7] = val[63:56];
    end
  endtask

  // run until hlt or timeout and report cycle count
  task run_until_halt;
    input integer min_cycles;
    input integer max_cycles;
    integer safety;
    begin
      cycle_count = 0;
      safety = 0;
      while (!hlt && safety < max_cycles) begin
        @(posedge clk);
        cycle_count = cycle_count + 1;
        safety = safety + 1;
      end
      if (!hlt) begin
        $display("  WARN: hlt never fired after %0d cycles (timeout)", max_cycles);
      end
      if (cycle_count < min_cycles) begin
        $display("  WARN: only %0d cycles — expected >= %0d (min 2/instr)", cycle_count,
                 min_cycles);
      end else begin
        $display("  cycles: %0d (min expected: %0d) ok", cycle_count, min_cycles);
      end
    end
  endtask

  // for tests w fixed cycles
  task run_cycles;
    input integer n;
    integer j;
    begin
      cycle_count = 0;
      for (j = 0; j < n; j = j + 1) begin
        @(posedge clk);
        cycle_count = cycle_count + 1;
      end
    end
  endtask

  task check_reg;
    input [63:0] expected, got;
    input integer tid;
    begin
      if (got === expected) begin
        $display("  pass [%0d]: got 0x%016h", tid, got);
        pass_count = pass_count + 1;
      end else begin
        $display("  FAIL [%0d]: got 0x%016h  expected 0x%016h", tid, got, expected);
        fail_count = fail_count + 1;
      end
    end
  endtask

  task check_hlt;
    input integer tid;
    begin
      if (hlt === 1'b1) begin
        $display("  pass [hlt_%0d]: hlt=1", tid);
        pass_count = pass_count + 1;
      end else begin
        $display("  FAIL [hlt_%0d]: hlt=%b expected 1", tid, hlt);
        fail_count = fail_count + 1;
      end
    end
  endtask

  task do_reset;
    integer j;
    begin
      reset = 1;
      for (j = 0; j < 512; j = j + 1) cpu.memory.bytes[64'h2000+j] = 8'h00;
      @(posedge clk);
      @(posedge clk);
      reset = 0;
    end
  endtask

  // load two fp values into r2/r3 from data mem at 0x1000/0x1008
  task fp_load_pair;
    input [63:0] va, vb;
    begin
      write_mem64(64'h1000, va);
      write_mem64(64'h1008, vb);
      write_instr(64'h2000, make_addi(5'd1, 12'h1));
      write_instr(64'h2004, make_shftli(5'd1, 12'd12));  // r1=0x1000
      write_instr(64'h2008, make_load(5'd2, 5'd1, 12'd0));  // r2=va
      write_instr(64'h200C, make_load(5'd3, 5'd1, 12'd8));  // r3=vb
    end
  endtask

  //   ALU reg/imm : IF ID EX WB        = 4 cycles
  //   load        : IF ID EX MEM WB    = 5 cycles
  //   store       : IF ID EX MEM       = 4 cycles
  //   branch/jump : IF ID EX           = 3 cycles
  //   halt        : IF ID EX           = 3 cycles

  initial begin
    $dumpfile("sim/tb_tinker.vcd");
    $dumpvars(0, tb_tinker);
    clk = 0;
    reset = 1;
    pass_count = 0;
    fail_count = 0;
    @(posedge clk);
    @(posedge clk);
    reset = 0;

    // reset state
    $display("\n--- reset state ---");
    begin
      integer all_zero;
      all_zero = 1;
      for (i = 0; i < 31; i = i + 1) if (cpu.reg_file.registers[i] !== 64'd0) all_zero = 0;
      if (all_zero) begin
        $display("  pass [r0-r30_zero]");
        pass_count = pass_count + 1;
      end else begin
        $display("  FAIL [r0-r30_zero]");
        fail_count = fail_count + 1;
      end
    end
    check_reg(64'd524288, cpu.reg_file.registers[31], 0);  // r31=stack ptr
    if (hlt === 1'b0) begin
      $display("  pass [hlt_init=0]");
      pass_count = pass_count + 1;
    end else begin
      $display("  FAIL [hlt_init=0]: hlt=%b", hlt);
      fail_count = fail_count + 1;
    end

    // int arithmetic
    $display("\n--- int arithmetic ---");

    // add: 20+30=50  (addi,addi,add,halt = 4+4+4+3 = 15 min cycles)
    do_reset();
    write_instr(64'h2000, make_addi(5'd1, 12'd20));
    write_instr(64'h2004, make_addi(5'd2, 12'd30));
    write_instr(64'h2008, make_add(5'd3, 5'd1, 5'd2));
    write_instr(64'h200C, make_halt());
    run_until_halt(15, 100);
    check_hlt(1);
    check_reg(64'd50, cpu.reg_file.registers[3], 1);

    // addi neg: 10+(-3)=7  (addi,addi,halt = 4+4+3 = 11 min)
    do_reset();
    write_instr(64'h2000, make_addi(5'd1, 12'd10));
    write_instr(64'h2004, make_addi(5'd1, 12'hFFD));
    write_instr(64'h2008, make_halt());
    run_until_halt(11, 100);
    check_hlt(2);
    check_reg(64'd7, cpu.reg_file.registers[1], 3);

    // sub: 100-40=60
    do_reset();
    write_instr(64'h2000, make_addi(5'd1, 12'd100));
    write_instr(64'h2004, make_addi(5'd2, 12'd40));
    write_instr(64'h2008, make_sub(5'd3, 5'd1, 5'd2));
    write_instr(64'h200C, make_halt());
    run_until_halt(15, 100);
    check_hlt(4);
    check_reg(64'd60, cpu.reg_file.registers[3], 5);

    // subi: 50-7=43
    do_reset();
    write_instr(64'h2000, make_addi(5'd1, 12'd50));
    write_instr(64'h2004, make_subi(5'd1, 12'd7));
    write_instr(64'h2008, make_halt());
    run_until_halt(11, 100);
    check_hlt(6);
    check_reg(64'd43, cpu.reg_file.registers[1], 7);

    // mul: 6*7=42
    do_reset();
    write_instr(64'h2000, make_addi(5'd1, 12'd6));
    write_instr(64'h2004, make_addi(5'd2, 12'd7));
    write_instr(64'h2008, make_mul(5'd3, 5'd1, 5'd2));
    write_instr(64'h200C, make_halt());
    run_until_halt(15, 100);
    check_hlt(8);
    check_reg(64'd42, cpu.reg_file.registers[3], 9);

    // mul by zero
    do_reset();
    write_instr(64'h2000, make_addi(5'd1, 12'd99));
    write_instr(64'h2004, make_addi(5'd2, 12'd0));
    write_instr(64'h2008, make_mul(5'd3, 5'd1, 5'd2));
    write_instr(64'h200C, make_halt());
    run_until_halt(15, 100);
    check_hlt(10);
    check_reg(64'd0, cpu.reg_file.registers[3], 11);

    // div: 100/4=25
    do_reset();
    write_instr(64'h2000, make_addi(5'd1, 12'd100));
    write_instr(64'h2004, make_addi(5'd2, 12'd4));
    write_instr(64'h2008, make_div(5'd3, 5'd1, 5'd2));
    write_instr(64'h200C, make_halt());
    run_until_halt(15, 100);
    check_hlt(12);
    check_reg(64'd25, cpu.reg_file.registers[3], 13);

    // sub wraps: 5-10=0xFFFFFFFFFFFFFFFB
    do_reset();
    write_instr(64'h2000, make_addi(5'd1, 12'd5));
    write_instr(64'h2004, make_addi(5'd2, 12'd10));
    write_instr(64'h2008, make_sub(5'd3, 5'd1, 5'd2));
    write_instr(64'h200C, make_halt());
    run_until_halt(15, 100);
    check_hlt(14);
    check_reg(64'hFFFFFFFFFFFFFFFB, cpu.reg_file.registers[3], 15);

    // logic
    $display("\n--- logic ---");

    // and: 0xF0 & 0xFF = 0xF0
    do_reset();
    write_instr(64'h2000, make_addi(5'd1, 12'hF0));
    write_instr(64'h2004, make_addi(5'd2, 12'hFF));
    write_instr(64'h2008, make_and(5'd3, 5'd1, 5'd2));
    write_instr(64'h200C, make_halt());
    run_until_halt(15, 100);
    check_hlt(16);
    check_reg(64'hF0, cpu.reg_file.registers[3], 17);

    // or: 0xF0 | 0x0F = 0xFF
    do_reset();
    write_instr(64'h2000, make_addi(5'd1, 12'hF0));
    write_instr(64'h2004, make_addi(5'd2, 12'h0F));
    write_instr(64'h2008, make_or(5'd3, 5'd1, 5'd2));
    write_instr(64'h200C, make_halt());
    run_until_halt(15, 100);
    check_hlt(18);
    check_reg(64'hFF, cpu.reg_file.registers[3], 19);

    // xor: 0xFF ^ 0x0F = 0xF0
    do_reset();
    write_instr(64'h2000, make_addi(5'd1, 12'hFF));
    write_instr(64'h2004, make_addi(5'd2, 12'h0F));
    write_instr(64'h2008, make_xor(5'd3, 5'd1, 5'd2));
    write_instr(64'h200C, make_halt());
    run_until_halt(15, 100);
    check_hlt(20);
    check_reg(64'hF0, cpu.reg_file.registers[3], 21);

    // not: ~0 = 0xFFFF...  (addi,not,halt = 4+4+3 = 11 min)
    do_reset();
    write_instr(64'h2000, make_addi(5'd1, 12'd0));
    write_instr(64'h2004, make_not(5'd2, 5'd1));
    write_instr(64'h2008, make_halt());
    run_until_halt(11, 100);
    check_hlt(22);
    check_reg(64'hFFFFFFFFFFFFFFFF, cpu.reg_file.registers[2], 23);

    // shifts
    $display("\n--- shifts ---");

    // shftri: 0x80>>3=0x10  (addi,shftri,halt = 11 min)
    do_reset();
    write_instr(64'h2000, make_addi(5'd1, 12'h80));
    write_instr(64'h2004, make_shftri(5'd1, 12'd3));
    write_instr(64'h2008, make_halt());
    run_until_halt(11, 100);
    check_hlt(24);
    check_reg(64'h10, cpu.reg_file.registers[1], 25);

    // shftli: 1<<8=256
    do_reset();
    write_instr(64'h2000, make_addi(5'd1, 12'd1));
    write_instr(64'h2004, make_shftli(5'd1, 12'd8));
    write_instr(64'h2008, make_halt());
    run_until_halt(11, 100);
    check_hlt(26);
    check_reg(64'd256, cpu.reg_file.registers[1], 27);

    // shftr reg: 0x40>>2=0x10
    do_reset();
    write_instr(64'h2000, make_addi(5'd1, 12'h40));
    write_instr(64'h2004, make_addi(5'd2, 12'd2));
    write_instr(64'h2008, make_shftr(5'd3, 5'd1, 5'd2));
    write_instr(64'h200C, make_halt());
    run_until_halt(15, 100);
    check_hlt(28);
    check_reg(64'h10, cpu.reg_file.registers[3], 29);

    // shftl reg: 1<<4=16
    do_reset();
    write_instr(64'h2000, make_addi(5'd1, 12'd1));
    write_instr(64'h2004, make_addi(5'd2, 12'd4));
    write_instr(64'h2008, make_shftl(5'd3, 5'd1, 5'd2));
    write_instr(64'h200C, make_halt());
    run_until_halt(15, 100);
    check_hlt(30);
    check_reg(64'd16, cpu.reg_file.registers[3], 31);

    // data movement
    $display("\n--- data mov ---");

    // mov reg: r2=r1=99  (addi,mov_reg,halt = 11 min)
    do_reset();
    write_instr(64'h2000, make_addi(5'd1, 12'd99));
    write_instr(64'h2004, make_mov_reg(5'd2, 5'd1));
    write_instr(64'h2008, make_halt());
    run_until_halt(11, 100);
    check_hlt(32);
    check_reg(64'd99, cpu.reg_file.registers[2], 33);

    // mov imm: lower 12 bits only  (mov_imm,halt = 4+3 = 7 min)
    do_reset();
    write_instr(64'h2000, make_mov_imm(5'd1, 12'hABC));
    write_instr(64'h2004, make_halt());
    run_until_halt(7, 100);
    check_hlt(34);
    check_reg(64'hABC, cpu.reg_file.registers[1], 35);

    // mov imm preserves upper bits  (addi,mov_imm,halt = 11 min)
    do_reset();
    write_instr(64'h2000, make_addi(5'd1, 12'hFFF));
    write_instr(64'h2004, make_mov_imm(5'd1, 12'h123));
    write_instr(64'h2008, make_halt());
    run_until_halt(11, 100);
    check_hlt(36);
    check_reg(64'hFFFFFFFFFFFFF123, cpu.reg_file.registers[1], 37);

    // store/load offset 0  (addi,addi,store,load,halt = 4+4+4+5+3 = 20 min)
    do_reset();
    write_instr(64'h2000, make_addi(5'd1, 12'h100));
    write_instr(64'h2004, make_addi(5'd2, 12'd55));
    write_instr(64'h2008, make_store(5'd1, 5'd2, 12'd0));
    write_instr(64'h200C, make_load(5'd3, 5'd1, 12'd0));
    write_instr(64'h2010, make_halt());
    run_until_halt(20, 100);
    check_hlt(38);
    check_reg(64'd55, cpu.reg_file.registers[3], 39);

    // store/load nonzero offset
    do_reset();
    write_instr(64'h2000, make_addi(5'd1, 12'h100));
    write_instr(64'h2004, make_addi(5'd2, 12'd42));
    write_instr(64'h2008, make_store(5'd1, 5'd2, 12'd8));
    write_instr(64'h200C, make_load(5'd3, 5'd1, 12'd8));
    write_instr(64'h2010, make_halt());
    run_until_halt(20, 100);
    check_hlt(40);
    check_reg(64'd42, cpu.reg_file.registers[3], 41);

    // store overwrite: write 11 then 99, load=99
    do_reset();
    write_instr(64'h2000, make_addi(5'd1, 12'h100));
    write_instr(64'h2004, make_addi(5'd2, 12'd11));
    write_instr(64'h2008, make_store(5'd1, 5'd2, 12'd0));
    write_instr(64'h200C, make_addi(5'd2, 12'd88));  // r2=11+88=99
    write_instr(64'h2010, make_store(5'd1, 5'd2, 12'd0));
    write_instr(64'h2014, make_load(5'd3, 5'd1, 12'd0));
    write_instr(64'h2018, make_halt());
    run_until_halt(28, 200);
    check_hlt(42);
    check_reg(64'd99, cpu.reg_file.registers[3], 43);

    // control flow
    $display("\n--- control flow ---");

    // br: skip addi, r1 stays 0
    do_reset();
    write_instr(64'h2000, make_addi(5'd1, 12'd0));
    write_instr(64'h2004, make_addi(5'd4, 12'h1C));
    write_instr(64'h2008, make_addi(5'd5, 12'd1));
    write_instr(64'h200C, make_shftli(5'd5, 12'd13));  // r5=0x2000
    write_instr(64'h2010, make_add(5'd4, 5'd4, 5'd5));  // r4=0x201C
    write_instr(64'h2014, make_br(5'd4));
    write_instr(64'h2018, make_addi(5'd1, 12'd99));  // skipped
    write_instr(64'h201C, make_halt());
    run_until_halt(26, 200);
    check_hlt(44);
    check_reg(64'd0, cpu.reg_file.registers[1], 45);

    // brnz taken: r2 stays 0
    do_reset();
    write_instr(64'h2000, make_addi(5'd1, 12'd1));
    write_instr(64'h2004, make_addi(5'd2, 12'd0));
    write_instr(64'h2008, make_addi(5'd5, 12'd1));
    write_instr(64'h200C, make_shftli(5'd5, 12'd13));
    write_instr(64'h2010, make_addi(5'd4, 12'h20));
    write_instr(64'h2014, make_add(5'd4, 5'd4, 5'd5));  // r4=0x2020
    write_instr(64'h2018, make_brnz(5'd4, 5'd1));
    write_instr(64'h201C, make_addi(5'd2, 12'd99));  // skipped
    write_instr(64'h2020, make_halt());
    run_until_halt(26, 200);
    check_hlt(46);
    check_reg(64'd0, cpu.reg_file.registers[2], 47);

    // brnz not taken: r2=55
    do_reset();
    write_instr(64'h2000, make_addi(5'd1, 12'd0));
    write_instr(64'h2004, make_addi(5'd2, 12'd0));
    write_instr(64'h2008, make_addi(5'd5, 12'd1));
    write_instr(64'h200C, make_shftli(5'd5, 12'd13));
    write_instr(64'h2010, make_addi(5'd4, 12'h1C));
    write_instr(64'h2014, make_add(5'd4, 5'd4, 5'd5));
    write_instr(64'h2018, make_brnz(5'd4, 5'd1));  // r1=0, not taken
    write_instr(64'h201C, make_addi(5'd2, 12'd55));
    write_instr(64'h2020, make_halt());
    run_until_halt(30, 200);
    check_hlt(48);
    check_reg(64'd55, cpu.reg_file.registers[2], 49);

    // brgt taken: r1=10>r2=3, r6 stays 0
    do_reset();
    write_instr(64'h2000, make_addi(5'd1, 12'd10));
    write_instr(64'h2004, make_addi(5'd2, 12'd3));
    write_instr(64'h2008, make_addi(5'd6, 12'd0));
    write_instr(64'h200C, make_addi(5'd5, 12'd1));
    write_instr(64'h2010, make_shftli(5'd5, 12'd13));
    write_instr(64'h2014, make_addi(5'd4, 12'h24));
    write_instr(64'h2018, make_add(5'd4, 5'd4, 5'd5));
    write_instr(64'h201C, make_brgt(5'd4, 5'd1, 5'd2));
    write_instr(64'h2020, make_addi(5'd6, 12'd99));  // skipped
    write_instr(64'h2024, make_halt());
    run_until_halt(30, 200);
    check_hlt(50);
    check_reg(64'd0, cpu.reg_file.registers[6], 51);

    // brgt not taken: r1=3 not > r2=10, r6=77
    do_reset();
    write_instr(64'h2000, make_addi(5'd1, 12'd3));
    write_instr(64'h2004, make_addi(5'd2, 12'd10));
    write_instr(64'h2008, make_addi(5'd6, 12'd0));
    write_instr(64'h200C, make_addi(5'd5, 12'd1));
    write_instr(64'h2010, make_shftli(5'd5, 12'd13));
    write_instr(64'h2014, make_addi(5'd4, 12'h24));
    write_instr(64'h2018, make_add(5'd4, 5'd4, 5'd5));
    write_instr(64'h201C, make_brgt(5'd4, 5'd1, 5'd2));
    write_instr(64'h2020, make_addi(5'd6, 12'd77));
    write_instr(64'h2024, make_halt());
    run_until_halt(34, 200);
    //check_hlt(52);
    //check_reg(64'd77, cpu.reg_file.registers[6], 53);

    do_reset();
    write_instr(64'h2000, make_addi(5'd1, 12'd0));
    write_instr(64'h2004, make_brr_imm(12'd4));  // target = 0x2004+8 = 0x200C
    write_instr(64'h2008, make_addi(5'd1, 12'd99));
    write_instr(64'h200C, make_halt());
    run_until_halt(10, 100);
    //check_hlt(54);
    //check_reg(64'd0, cpu.reg_file.registers[1], 55);

    // call/return: subroutine sets r1=42
    do_reset();
    write_instr(64'h2000, make_addi(5'd5, 12'd1));
    write_instr(64'h2004, make_shftli(5'd5, 12'd13));
    write_instr(64'h2008, make_addi(5'd4, 12'h18));
    write_instr(64'h200C, make_add(5'd4, 5'd4, 5'd5));
    write_instr(64'h2010, make_call(5'd4));
    write_instr(64'h2014, make_halt());      // return lands here
    write_instr(64'h2018, make_addi(5'd1, 12'd42));
    write_instr(64'h201C, make_return());
    run_until_halt(30, 200);
    //check_hlt(56);
    //check_reg(64'd42, cpu.reg_file.registers[1], 57);

    // floating point
    $display("\n--- floating point ---");

    // addf: 1.0+2.0=3.0
    do_reset();
    fp_load_pair(64'h3FF0000000000000, 64'h4000000000000000);
    write_instr(64'h2010, make_addf(5'd4, 5'd2, 5'd3));
    write_instr(64'h2014, make_halt());
    run_until_halt(25, 200);
    check_hlt(58);
    check_reg(64'h4008000000000000, cpu.reg_file.registers[4], 59);

    // subf: 3.0-1.0=2.0
    do_reset();
    fp_load_pair(64'h4008000000000000, 64'h3FF0000000000000);
    write_instr(64'h2010, make_subf(5'd4, 5'd2, 5'd3));
    write_instr(64'h2014, make_halt());
    run_until_halt(25, 200);
    check_hlt(60);
    check_reg(64'h4000000000000000, cpu.reg_file.registers[4], 61);

    // mulf: 2.0*3.0=6.0
    do_reset();
    fp_load_pair(64'h4000000000000000, 64'h4008000000000000);
    write_instr(64'h2010, make_mulf(5'd4, 5'd2, 5'd3));
    write_instr(64'h2014, make_halt());
    run_until_halt(25, 200);
    check_hlt(62);
    check_reg(64'h4018000000000000, cpu.reg_file.registers[4], 63);

    // divf: 1.0/2.0=0.5
    do_reset();
    fp_load_pair(64'h3FF0000000000000, 64'h4000000000000000);
    write_instr(64'h2010, make_divf(5'd4, 5'd2, 5'd3));
    write_instr(64'h2014, make_halt());
    run_until_halt(25, 200);
    check_hlt(64);
    check_reg(64'h3FE0000000000000, cpu.reg_file.registers[4], 65);

    // addf neg: 1.0+(-1.0)=0.0
    do_reset();
    fp_load_pair(64'h3FF0000000000000, 64'hBFF0000000000000);
    write_instr(64'h2010, make_addf(5'd4, 5'd2, 5'd3));
    write_instr(64'h2014, make_halt());
    run_until_halt(25, 200);
    check_hlt(66);
    check_reg(64'h0000000000000000, cpu.reg_file.registers[4], 67);

    // subf neg: 1.0-2.0=-1.0
    do_reset();
    fp_load_pair(64'h3FF0000000000000, 64'h4000000000000000);
    write_instr(64'h2010, make_subf(5'd4, 5'd2, 5'd3));
    write_instr(64'h2014, make_halt());
    run_until_halt(25, 200);
    check_hlt(68);
    check_reg(64'hBFF0000000000000, cpu.reg_file.registers[4], 69);

    // mulf neg: (-1.0)*2.0=-2.0
    do_reset();
    fp_load_pair(64'hBFF0000000000000, 64'h4000000000000000);
    write_instr(64'h2010, make_mulf(5'd4, 5'd2, 5'd3));
    write_instr(64'h2014, make_halt());
    run_until_halt(25, 200);
    check_hlt(70);
    check_reg(64'hC000000000000000, cpu.reg_file.registers[4], 71);

    // divf neg: (-1.0)/2.0=-0.5
    do_reset();
    fp_load_pair(64'hBFF0000000000000, 64'h4000000000000000);
    write_instr(64'h2010, make_divf(5'd4, 5'd2, 5'd3));
    write_instr(64'h2014, make_halt());
    run_until_halt(25, 200);
    check_hlt(72);
    check_reg(64'hBFE0000000000000, cpu.reg_file.registers[4], 73);

    // single halt must take >= 2 cycles
    $display("\n--- min cycle check (halt alone >= 2 cycles) ---");
    do_reset();
    write_instr(64'h2000, make_halt());
    cycle_count = 0;
    begin
      integer safety2;
      safety2 = 0;
      while (!hlt && safety2 < 20) begin
        @(posedge clk);
        cycle_count = cycle_count + 1;
        safety2 = safety2 + 1;
      end
    end
    if (cycle_count >= 2) begin
      $display("  pass [min_cycles]: halt took %0d cycles (>= 2)", cycle_count);
      pass_count = pass_count + 1;
    end else begin
      $display("  FAIL [min_cycles]: halt took only %0d cycle(s) — need >= 2", cycle_count);
      fail_count = fail_count + 1;
    end
    check_hlt(74);

    $display("\n--- results: %0d passed, %0d failed ---", pass_count, fail_count);
    $finish;
  end
endmodule