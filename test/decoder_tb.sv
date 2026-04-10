// tb_decoder.v — unit tests for the decoder module
module tb_decoder;

  reg [31:0] instr;
  wire [4:0] raddr1, raddr2, waddr, op, rt_addr;
  wire [11:0] immediate;
  wire use_imm, write;
  wire is_load, is_store, is_branch, is_brgt, is_jump;
  wire is_brr_reg, is_brr_imm, is_return, is_call, is_halt;
  wire is_mov_reg, is_mov_imm;

  decoder dut (
      .instr     (instr),
      .raddr1    (raddr1),
      .raddr2    (raddr2),
      .waddr     (waddr),
      .immediate (immediate),
      .op        (op),
      .use_imm   (use_imm),
      .write     (write),
      .is_load   (is_load),
      .is_store  (is_store),
      .is_branch (is_branch),
      .is_brgt   (is_brgt),
      .is_jump   (is_jump),
      .is_brr_reg(is_brr_reg),
      .is_brr_imm(is_brr_imm),
      .is_return (is_return),
      .is_call   (is_call),
      .is_halt   (is_halt),
      .is_mov_reg(is_mov_reg),
      .is_mov_imm(is_mov_imm),
      .rt_addr   (rt_addr)
  );

  integer pass_count, fail_count;

  // opcode[31:27] rd[26:22] rs[21:17] rt[16:12] imm[11:0]
  function [31:0] enc;
    input [4:0] opc, rd, rs, rt;
    input [11:0] im;
    enc = (opc << 27) | (rd << 22) | (rs << 17) | (rt << 12) | im;
  endfunction

  task chk5;
    input [4:0] exp, got;
    input [255:0] name;
    begin
      if (got === exp) begin
        $display("  pass [%s]: %0d", name, got);
        pass_count = pass_count + 1;
      end else begin
        $display("  FAIL [%s]: got %0d  exp %0d", name, got, exp);
        fail_count = fail_count + 1;
      end
    end
  endtask

  task chk12;
    input [11:0] exp, got;
    input [255:0] name;
    begin
      if (got === exp) begin
        $display("  pass [%s]: 0x%03h", name, got);
        pass_count = pass_count + 1;
      end else begin
        $display("  FAIL [%s]: got 0x%03h  exp 0x%03h", name, got, exp);
        fail_count = fail_count + 1;
      end
    end
  endtask

  task chk1;
    input exp, got;
    input [255:0] name;
    begin
      if (got === exp) begin
        $display("  pass [%s]: %b", name, got);
        pass_count = pass_count + 1;
      end else begin
        $display("  FAIL [%s]: got %b  exp %b", name, got, exp);
        fail_count = fail_count + 1;
      end
    end
  endtask

  initial begin
    $dumpfile("sim/tb_decoder.vcd");
    $dumpvars(0, tb_decoder);
    pass_count = 0;
    fail_count = 0;

    $display("\n--- and (0x00) ---");
    instr = enc(5'h00, 5'd3, 5'd1, 5'd2, 12'd0);
    #1;
    chk5(5'd3, waddr, "waddr");
    chk5(5'd1, raddr1, "raddr1");
    chk5(5'd2, raddr2, "raddr2");
    chk1(1'b1, write, "write");
    chk1(1'b0, is_branch, "!branch");

    $display("\n--- or (0x01) ---");
    instr = enc(5'h01, 5'd4, 5'd5, 5'd6, 12'd0);
    #1;
    chk5(5'd4, waddr, "waddr");
    chk5(5'd5, raddr1, "raddr1");
    chk5(5'd6, raddr2, "raddr2");

    $display("\n--- xor (0x02) ---");
    instr = enc(5'h02, 5'd1, 5'd2, 5'd3, 12'd0);
    #1;
    chk5(5'd1, waddr, "waddr");

    $display("\n--- not (0x03) ---");
    instr = enc(5'h03, 5'd2, 5'd1, 5'd0, 12'd0);
    #1;
    chk5(5'd2, waddr, "waddr");
    chk5(5'd1, raddr1, "raddr1");

    $display("\n--- shftr (0x04) ---");
    instr = enc(5'h04, 5'd3, 5'd1, 5'd2, 12'd0);
    #1;

    $display("\n--- shftri (0x05) ---");
    instr = enc(5'h05, 5'd1, 5'd0, 5'd0, 12'd5);
    #1;
    chk12(12'd5, immediate, "imm");
    chk1(1'b1, use_imm, "use_imm");
    chk5(5'd1, waddr, "waddr");

    $display("\n--- shftl/shftli (0x06/0x07) ---");
    instr = enc(5'h06, 5'd3, 5'd1, 5'd2, 12'd0);
    #1;
    instr = enc(5'h07, 5'd1, 5'd0, 5'd0, 12'd8);
    #1;
    chk12(12'd8, immediate, "shftli_imm");

    $display("\n--- br (0x08) ---");
    instr = enc(5'h08, 5'd4, 5'd0, 5'd0, 12'd0);
    #1;
    chk1(1'b1, is_jump, "is_jump");
    chk1(1'b0, is_brr_imm, "!brr_imm");

    $display("\n--- brr_reg (0x09) ---");
    instr = enc(5'h09, 5'd4, 5'd0, 5'd0, 12'd0);
    #1;
    chk1(1'b1, is_brr_reg, "is_brr_reg");

    $display("\n--- brr_imm (0x0A) ---");
    instr = enc(5'h0A, 5'd0, 5'd0, 5'd0, 12'd8);
    #1;
    chk12(12'd8, immediate, "imm");
    chk1(1'b1, is_brr_imm, "is_brr_imm");

    $display("\n--- brnz (0x0B) ---");
    instr = enc(5'h0B, 5'd4, 5'd1, 5'd0, 12'd0);
    #1;
    chk5(5'd1, raddr1, "raddr1");
    chk1(1'b1, is_branch, "is_branch");

    $display("\n--- call (0x0C) ---");
    instr = enc(5'h0C, 5'd4, 5'd0, 5'd0, 12'd0);
    #1;
    chk1(1'b1, is_call, "is_call");

    $display("\n--- return (0x0D) ---");
    instr = enc(5'h0D, 5'd0, 5'd0, 5'd0, 12'd0);
    #1;
    chk1(1'b1, is_return, "is_return");

    $display("\n--- brgt (0x0E) ---");
    instr = enc(5'h0E, 5'd4, 5'd1, 5'd2, 12'd0);
    #1;
    chk1(1'b1, is_brgt, "is_brgt");
    chk5(5'd2, rt_addr, "rt_addr");

    $display("\n--- halt (0x0F) ---");
    instr = enc(5'h0F, 5'd0, 5'd0, 5'd0, 12'd0);
    #1;
    chk1(1'b1, is_halt, "is_halt");

    $display("\n--- load (0x10) ---");
    instr = enc(5'h10, 5'd3, 5'd1, 5'd0, 12'h10);
    #1;
    chk5(5'd3, waddr, "waddr");
    chk5(5'd1, raddr1, "raddr1");
    chk12(12'h10, immediate, "imm");
    chk1(1'b1, is_load, "is_load");
    chk1(1'b1, write, "write");

    $display("\n--- mov_reg (0x11) ---");
    instr = enc(5'h11, 5'd2, 5'd1, 5'd0, 12'd0);
    #1;
    chk5(5'd2, waddr, "waddr");
    chk5(5'd1, raddr1, "raddr1");
    chk1(1'b1, is_mov_reg, "is_mov_reg");
    chk1(1'b1, write, "write");

    $display("\n--- mov_imm (0x12) ---");
    instr = enc(5'h12, 5'd1, 5'd0, 5'd0, 12'hABC);
    #1;
    chk12(12'hABC, immediate, "imm");
    chk1(1'b1, is_mov_imm, "is_mov_imm");
    chk1(1'b1, write, "write");

    $display("\n--- store (0x13) ---");
    instr = enc(5'h13, 5'd1, 5'd2, 5'd0, 12'd8);
    #1;
    chk1(1'b1, is_store, "is_store");
    chk1(1'b0, write, "!write");

    $display("\n--- addf (0x14) ---");
    instr = enc(5'h14, 5'd4, 5'd2, 5'd3, 12'd0);
    #1;
    chk1(1'b1, write, "write");

    $display("\n--- sub/subi (0x1A/0x1B) ---");
    instr = enc(5'h1A, 5'd3, 5'd1, 5'd2, 12'd0);
    #1;
    instr = enc(5'h1B, 5'd1, 5'd0, 5'd0, 12'd7);
    #1;
    chk12(12'd7, immediate, "subi_imm");

    $display("\n--- results: %0d passed, %0d failed ---", pass_count, fail_count);
    $finish;
  end
endmodule
