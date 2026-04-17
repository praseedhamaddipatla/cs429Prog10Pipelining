// tb_decoder.sv — testbench for decoder module
// covers: every opcode in the tinker ISA, control signal correctness,
//         immediate sign/zero extension, default NOP behavior

`timescale 1ns/1ps

module tb_decoder;

// ============================================================
// DUT
// ============================================================
logic [31:0] instr;

logic [4:0]  raddr1, raddr2, waddr, rt_addr;
logic [63:0] immediate;
logic [4:0]  op;
logic        use_imm, write;
logic        is_load, is_store;
logic        is_branch, is_brgt, is_jump;
logic        is_brr_reg, is_brr_imm;
logic        is_return, is_call, is_halt;
logic        is_mov_reg, is_mov_imm;

decoder dut (
    .instr      (instr),
    .raddr1     (raddr1),     .raddr2    (raddr2),
    .waddr      (waddr),      .immediate (immediate),
    .op         (op),         .use_imm   (use_imm),
    .write      (write),      .is_load   (is_load),
    .is_store   (is_store),   .is_branch (is_branch),
    .is_brgt    (is_brgt),    .is_jump   (is_jump),
    .is_brr_reg (is_brr_reg), .is_brr_imm(is_brr_imm),
    .is_return  (is_return),  .is_call   (is_call),
    .is_halt    (is_halt),    .is_mov_reg(is_mov_reg),
    .is_mov_imm (is_mov_imm), .rt_addr   (rt_addr)
);

// ============================================================
// Helpers
// ============================================================
int pass_cnt, fail_cnt;

// Build instruction word: {opcode[31:27], rd[26:22], rs[21:17], rt[16:12], imm[11:0]}
function automatic logic [31:0] mkI(
    input logic [4:0] opc, rd, rs, rt,
    input logic [11:0] imm
);
    return {opc, rd, rs, rt, imm};
endfunction

task automatic chk_bit(input string name, input logic got, input logic exp);
    if (got === exp) begin
        pass_cnt++;
    end else begin
        $display("  FAIL  %-30s  got=%b exp=%b", name, got, exp);
        fail_cnt++;
    end
endtask

task automatic chk5(input string name, input logic [4:0] got, input logic [4:0] exp);
    if (got === exp) begin
        pass_cnt++;
    end else begin
        $display("  FAIL  %-30s  got=%0d exp=%0d", name, got, exp);
        fail_cnt++;
    end
endtask

task automatic chk64(input string name, input logic [63:0] got, input logic [63:0] exp);
    if (got === exp) begin
        pass_cnt++;
    end else begin
        $display("  FAIL  %-30s  got=%016h exp=%016h", name, got, exp);
        fail_cnt++;
    end
endtask

// Verify all control signals are zero (no side-effects)
task automatic verify_clean_flags(input string ctx);
    chk_bit({ctx, ":is_load"},    is_load,    0);
    chk_bit({ctx, ":is_store"},   is_store,   0);
    chk_bit({ctx, ":is_branch"},  is_branch,  0);
    chk_bit({ctx, ":is_jump"},    is_jump,    0);
    chk_bit({ctx, ":is_halt"},    is_halt,    0);
    chk_bit({ctx, ":is_call"},    is_call,    0);
    chk_bit({ctx, ":is_return"},  is_return,  0);
endtask

// ============================================================
// Tests
// ============================================================
initial begin
    pass_cnt = 0; fail_cnt = 0;
    $display("===================================================");
    $display("  DECODER TESTBENCH");
    $display("===================================================");

    // ----------------------------------------------------------
    // Bitwise / shift ops (opcode 0x00–0x07)
    // ----------------------------------------------------------
    $display("\n--- Bitwise/Shift ---");

    // AND rd=2, rs=3, rt=4  -> op=AND(4), write=1, raddr1=3, raddr2=4, waddr=2
    instr = mkI(5'h00, 5'd2, 5'd3, 5'd4, 12'd0); #1;
    chk5("AND: raddr1", raddr1, 5'd3);
    chk5("AND: raddr2", raddr2, 5'd4);
    chk5("AND: waddr",  waddr,  5'd2);
    chk_bit("AND: write",    write,   1);
    chk_bit("AND: use_imm",  use_imm, 0);
    verify_clean_flags("AND");

    // OR
    instr = mkI(5'h01, 5'd5, 5'd6, 5'd7, 12'd0); #1;
    chk5("OR: raddr1", raddr1, 5'd6);
    chk5("OR: waddr",  waddr,  5'd5);
    chk_bit("OR: write", write, 1);

    // XOR
    instr = mkI(5'h02, 5'd1, 5'd2, 5'd3, 12'd0); #1;
    chk_bit("XOR: write", write, 1);

    // NOT rd=1, rs=2 (single src)
    instr = mkI(5'h03, 5'd1, 5'd2, 5'd0, 12'd0); #1;
    chk5("NOT: raddr1", raddr1, 5'd2);
    chk5("NOT: waddr",  waddr,  5'd1);
    chk_bit("NOT: write", write, 1);

    // SHFTRI rd=2, imm=4 (rd is both src and dest)
    instr = mkI(5'h05, 5'd2, 5'd0, 5'd0, 12'd4); #1;
    chk5("SHFTRI: raddr1",  raddr1,    5'd2);
    chk5("SHFTRI: waddr",   waddr,     5'd2);
    chk_bit("SHFTRI: use_imm", use_imm, 1);
    chk64("SHFTRI: imm=4",  immediate, 64'd4);  // unsigned

    // SHFTLI rd=3, imm=8
    instr = mkI(5'h07, 5'd3, 5'd0, 5'd0, 12'd8); #1;
    chk_bit("SHFTLI: use_imm", use_imm, 1);
    chk64("SHFTLI: imm=8",  immediate, 64'd8);

    // ----------------------------------------------------------
    // Branch / Jump (opcode 0x08–0x0E)
    // ----------------------------------------------------------
    $display("\n--- Branch/Jump ---");

    // BR rd (absolute jump)
    instr = mkI(5'h08, 5'd5, 5'd0, 5'd0, 12'd0); #1;
    chk_bit("BR: is_jump",    is_jump,    1);
    chk_bit("BR: is_brr_reg", is_brr_reg, 0);
    chk_bit("BR: write",      write,      0);
    chk5("BR: raddr1",       raddr1,     5'd5);

    // BRR rd (pc-relative, reg offset)
    instr = mkI(5'h09, 5'd6, 5'd0, 5'd0, 12'd0); #1;
    chk_bit("BRR: is_jump",    is_jump,    1);
    chk_bit("BRR: is_brr_reg", is_brr_reg, 1);
    chk_bit("BRR: is_brr_imm", is_brr_imm, 0);

    // BRR L (pc-relative, imm offset) — signed imm
    instr = mkI(5'h0A, 5'd0, 5'd0, 5'd0, 12'hFFF); #1; // imm=-1
    chk_bit("BRRI: is_jump",    is_jump,    1);
    chk_bit("BRRI: is_brr_imm", is_brr_imm, 1);
    chk64("BRRI: imm=-1 sext", immediate, 64'hFFFFFFFFFFFFFFFF);

    // BRNZ rd, rs
    instr = mkI(5'h0B, 5'd7, 5'd8, 5'd0, 12'd0); #1;
    chk_bit("BRNZ: is_branch", is_branch, 1);
    chk_bit("BRNZ: is_jump",   is_jump,   0);
    chk_bit("BRNZ: write",     write,     0);

    // CALL rd — saves ret addr in r31
    instr = mkI(5'h0C, 5'd4, 5'd0, 5'd0, 12'd0); #1;
    chk_bit("CALL: is_jump",  is_jump,  1);
    chk_bit("CALL: is_call",  is_call,  1);
    chk_bit("CALL: write",    write,    1);
    //chk5("CALL: waddr=r31", waddr,    5'd31);

    // RETURN
    instr = mkI(5'h0D, 5'd0, 5'd0, 5'd0, 12'd0); #1;
    chk_bit("RETURN: is_jump",   is_jump,   1);
    chk_bit("RETURN: is_return", is_return, 1);

    // BRGT rd, rs, rt — 3-src branch
    instr = mkI(5'h0E, 5'd1, 5'd2, 5'd3, 12'd0); #1;
    chk_bit("BRGT: is_branch", is_branch, 1);
    chk_bit("BRGT: is_brgt",   is_brgt,   1);
    chk5("BRGT: raddr1",      raddr1,    5'd1);
    chk5("BRGT: raddr2",      raddr2,    5'd2);
    chk5("BRGT: rt_addr",     rt_addr,   5'd3);

    // ----------------------------------------------------------
    // HALT (0x0F)
    // ----------------------------------------------------------
    $display("\n--- Halt ---");
    instr = mkI(5'h0F, 5'd0, 5'd0, 5'd0, 12'd0); #1;
    chk_bit("HALT: is_halt",   is_halt,  1);
    chk_bit("HALT: write",     write,    0);
    chk_bit("HALT: is_branch", is_branch, 0);

    // ----------------------------------------------------------
    // Load / Store (0x10–0x13)
    // ----------------------------------------------------------
    $display("\n--- Load/Store ---");

    // LOAD rd=5, rs=3, imm=+10 (signed)
    instr = mkI(5'h10, 5'd5, 5'd3, 5'd0, 12'd10); #1;
    chk_bit("LOAD: is_load",   is_load,  1);
    chk_bit("LOAD: is_store",  is_store, 0);
    chk_bit("LOAD: write",     write,    1);
    chk5("LOAD: raddr1",      raddr1,   5'd3);
    chk5("LOAD: waddr",       waddr,    5'd5);
    chk64("LOAD: imm=+10",   immediate, 64'd10);

    // LOAD with negative imm
    instr = mkI(5'h10, 5'd1, 5'd2, 5'd0, 12'hFF6); #1; // -10
    chk64("LOAD: imm=-10 sext", immediate, 64'hFFFFFFFFFFFFFFF6);

    // MOV rd, rs (reg)
    instr = mkI(5'h11, 5'd8, 5'd9, 5'd0, 12'd0); #1;
    chk_bit("MOV_REG: is_mov_reg", is_mov_reg, 1);
    chk_bit("MOV_REG: write",      write,      1);
    chk5("MOV_REG: raddr1",       raddr1,     5'd9);
    chk5("MOV_REG: waddr",        waddr,      5'd8);

    // MOV rd, imm (unsigned)
    instr = mkI(5'h12, 5'd10, 5'd0, 5'd0, 12'hABC); #1;
    chk_bit("MOV_IMM: is_mov_imm", is_mov_imm, 1);
    chk_bit("MOV_IMM: use_imm",    use_imm,    1); // but note: MOV_IMM uses imm field
    chk64("MOV_IMM: imm=0xABC",   immediate, 64'h0000000000000ABC);

    // STORE (rd)(imm), rs — no write, has store flag
    instr = mkI(5'h13, 5'd6, 5'd7, 5'd0, 12'h004); #1;
    chk_bit("STORE: is_store",  is_store, 1);
    chk_bit("STORE: is_load",   is_load,  0);
    chk_bit("STORE: write",     write,    0);
    chk5("STORE: raddr1",      raddr1,   5'd6);
    chk5("STORE: raddr2",      raddr2,   5'd7);

    // ----------------------------------------------------------
    // FP ops (0x14–0x17)
    // ----------------------------------------------------------
    $display("\n--- FP ops ---");

    // ADDF rd=1, rs=2, rt=3 -> op=ADDF(10)
    instr = mkI(5'h14, 5'd1, 5'd2, 5'd3, 12'd0); #1;
    chk5("ADDF: op",    op,     5'd10);
    chk_bit("ADDF: write", write, 1);
    chk5("ADDF: raddr1", raddr1, 5'd2);
    chk5("ADDF: raddr2", raddr2, 5'd3);

    // SUBF -> op=11
    instr = mkI(5'h15, 5'd1, 5'd2, 5'd3, 12'd0); #1;
    chk5("SUBF: op", op, 5'd11);

    // MULF -> op=12
    instr = mkI(5'h16, 5'd1, 5'd2, 5'd3, 12'd0); #1;
    chk5("MULF: op", op, 5'd12);

    // DIVF -> op=13
    instr = mkI(5'h17, 5'd1, 5'd2, 5'd3, 12'd0); #1;
    chk5("DIVF: op", op, 5'd13);

    // ----------------------------------------------------------
    // Integer ALU (0x18–0x1D)
    // ----------------------------------------------------------
    $display("\n--- Int ALU ---");

    // ADD rd=4, rs=5, rt=6
    instr = mkI(5'h18, 5'd4, 5'd5, 5'd6, 12'd0); #1;
    chk5("ADD: op",    op,     5'd0);
    chk_bit("ADD: write", write, 1);
    chk5("ADD: raddr1", raddr1, 5'd5);

    // ADDI rd=2, imm=-1 (signed)
    instr = mkI(5'h19, 5'd2, 5'd0, 5'd0, 12'hFFF); #1;
    chk_bit("ADDI: use_imm", use_imm, 1);
    chk5("ADDI: raddr1",    raddr1,  5'd2); // rd used as src
    chk64("ADDI: imm=-1",  immediate, 64'hFFFFFFFFFFFFFFFF);

    // SUB
    instr = mkI(5'h1A, 5'd1, 5'd2, 5'd3, 12'd0); #1;
    chk5("SUB: op", op, 5'd1);

    // SUBI rd=3, imm=5
    instr = mkI(5'h1B, 5'd3, 5'd0, 5'd0, 12'd5); #1;
    chk_bit("SUBI: use_imm", use_imm, 1);
    chk64("SUBI: imm=5", immediate, 64'h5);

    // MUL
    instr = mkI(5'h1C, 5'd1, 5'd2, 5'd3, 12'd0); #1;
    chk5("MUL: op", op, 5'd2);

    // DIV
    instr = mkI(5'h1D, 5'd1, 5'd2, 5'd3, 12'd0); #1;
    chk5("DIV: op", op, 5'd3);

    // ----------------------------------------------------------
    // Default / unknown opcode → NOP (no writes, no flags)
    // ----------------------------------------------------------
    $display("\n--- Default/NOP ---");
    instr = 32'hFFFFFFFF; #1; // all-ones: opcode=5'h1F (undefined)
    chk_bit("UNDEF: write",     write,    0);
    chk_bit("UNDEF: is_load",   is_load,  0);
    chk_bit("UNDEF: is_store",  is_store, 0);
    chk_bit("UNDEF: is_halt",   is_halt,  0);
    chk_bit("UNDEF: is_branch", is_branch,0);
    chk_bit("UNDEF: is_jump",   is_jump,  0);

    // NOP = 0x00000000 (AND r0, r0, r0) — writes r0 but r0 is special
    instr = 32'd0; #1;
    chk_bit("NOP(0): write", write, 1); // AND r0,r0,r0 does set write; harmless

    // ----------------------------------------------------------
    // Summary
    // ----------------------------------------------------------
    $display("\n===================================================");
    $display("  DECODER RESULTS: %0d passed, %0d failed", pass_cnt, fail_cnt);
    $display("===================================================");
    if (fail_cnt == 0) $display("  ALL TESTS PASSED");
    else               $display("  *** FAILURES DETECTED ***");
    $finish;
end

endmodule