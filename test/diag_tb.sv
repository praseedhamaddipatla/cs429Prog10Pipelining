// diag_tb.sv — Enhanced diagnostic testbench: branch + memory failures
//
// Compile (from project root - tinker.sv already `include`s all hdl/ files):
//   iverilog -g2005-sv -o diag_tb.vvp diag_tb.sv tinker.sv
// Run:  vvp diag_tb.vvp
// NOTE: Do NOT pass hdl/*.sv files separately - tinker.sv `include`s them.

`timescale 1ns/1ps
`define MEM_SIZE  (512 * 1024)
`define TIMEOUT   50

module diag_tb;

  reg  clk, reset;
  wire hlt;
  tinker_core dut(.clk(clk), .reset(reset), .hlt(hlt));
  initial clk = 0;
  always  #5 clk = ~clk;

  // ---- Instruction encoders ----
  function [31:0] f_addi;   input [4:0] rd; input [11:0] imm; f_addi  ={5'h19,rd,10'd0,imm}; endfunction
  function [31:0] f_subi;   input [4:0] rd; input [11:0] imm; f_subi  ={5'h1B,rd,10'd0,imm}; endfunction
  function [31:0] f_xor3;   input [4:0] rd,rs,rt;             f_xor3  ={5'h02,rd,rs,rt,12'd0}; endfunction
  function [31:0] f_add3;   input [4:0] rd,rs,rt;             f_add3  ={5'h18,rd,rs,rt,12'd0}; endfunction
  function [31:0] f_shftli; input [4:0] rd; input [11:0] imm; f_shftli={5'h07,rd,10'd0,imm}; endfunction
  function [31:0] f_br;     input [4:0] rd;                   f_br    ={5'h08,rd,22'd0}; endfunction
  function [31:0] f_brnz;   input [4:0] rd,rs;                f_brnz  ={5'h0B,rd,rs,17'd0}; endfunction
  function [31:0] f_brgt;   input [4:0] rd,rs,rt;             f_brgt  ={5'h0E,rd,rs,rt,12'd0}; endfunction
  function [31:0] f_sub3;   input [4:0] rd,rs,rt;             f_sub3  ={5'h1A,rd,rs,rt,12'd0}; endfunction
  function [31:0] f_load;   input [4:0] rd,rs; input [11:0] imm; f_load={5'h10,rd,rs,5'd0,imm}; endfunction
  function [31:0] f_store;  input [4:0] rd,rs; input [11:0] imm; f_store={5'h13,rd,rs,5'd0,imm}; endfunction
  function [31:0] f_halt;   f_halt=32'h7800_0000; endfunction

  // ---- Helpers ----
  task load_w; input [63:0] a; input [31:0] w;
    dut.memory.bytes[a+0]=w[7:0]; dut.memory.bytes[a+1]=w[15:8];
    dut.memory.bytes[a+2]=w[23:16]; dut.memory.bytes[a+3]=w[31:24];
  endtask
  task load_d; input [63:0] a; input [63:0] v; integer j;
    for(j=0;j<8;j=j+1) dut.memory.bytes[a+j]=v[j*8+:8];
  endtask
  task clear_mem; integer m; for(m=0;m<`MEM_SIZE;m=m+1) dut.memory.bytes[m]=8'h00; endtask

  integer g_cyc; reg g_to;
  task run; input integer mx;
    reset=1; @(posedge clk);#1; @(posedge clk);#1; reset=0; @(posedge clk);#1;
    g_cyc=0; g_to=0;
    while(!hlt && g_cyc<mx) begin @(posedge clk);#1; g_cyc=g_cyc+1; end
    if(!hlt) g_to=1;
  endtask

  integer pass_n,fail_n;
  task chk; input [255:0] nm; input [63:0] got,exp;
    if(got===exp) begin $display("    PASS  %0s  got=%0d",nm,got); pass_n=pass_n+1; end
    else          begin $display("    FAIL  %0s  got=%0d  exp=%0d",nm,got,exp); fail_n=fail_n+1; end
  endtask

  // Enhanced dump: shows ROB head contents and RS states
  task dump_basic;
    $display("    pc=0x%X hlt=%0b cyc=%0d",dut.pc_reg,hlt,g_cyc);
    $display("    rob: cnt=%0d head=%0d tail=%0d",dut.rob_cnt,dut.rob_head,dut.rob_tail);
    $display("    rs:  cnt=%0d  fp_cnt=%0d  lsq_cnt=%0d",dut.rs_cnt,dut.fp_cnt,dut.lsq_cnt);
    $display("    fl:  cnt=%0d head=%0d tail=%0d",dut.fl_cnt,dut.fl_head,dut.fl_tail);
    $display("    redirect_en=%0b redirect_pc=0x%X",dut.redirect_en,dut.redirect_pc);
    $display("    stall=%0b  dq_v0=%0b dq_v1=%0b",dut.stall,dut.dq_v0,dut.dq_v1);
  endtask

  task dump_rob_head;
    reg [4:0] h;
    h = dut.rob_head;
    $display("    ROB[head=%0d]: valid=%0b done=%0b arch=r%0d is_br=%0b is_jmp=%0b is_hlt=%0b",
             h, dut.rob_valid[h], dut.rob_done[h], dut.rob_arch[h],
             dut.rob_is_branch[h], dut.rob_is_jump[h], dut.rob_is_halt[h]);
    $display("      pred_taken=%0b pred_tgt=0x%X act_taken=%0b act_tgt=0x%X pc=0x%X",
             dut.rob_pred_taken[h],dut.rob_pred_tgt[h],
             dut.rob_act_taken[h],dut.rob_act_tgt[h],dut.rob_pc[h]);
    $display("      result=0x%X has_dest=%0b",dut.rob_result[h],dut.rob_has_dest[h]);
  endtask

  task dump_rs_all;
    integer ii;
    $display("    RS entries (v=valid p=psrdy t=ptrdy u=uimm br=ibr jmp=ijmp):");
    for(ii=0;ii<8;ii=ii+1)
      if(dut.rs_v[ii])
        $display("      rs[%0d]: op=%0d rob=%0d psrdy=%0b ptrdy=%0b uimm=%0b ibr=%0b ijmp=%0b vs=0x%X vt=0x%X imm=0x%X",
                 ii,dut.rs_op[ii],dut.rs_rob[ii],dut.rs_psrdy[ii],dut.rs_ptrdy[ii],
                 dut.rs_uimm[ii],dut.rs_ibr[ii],dut.rs_ijmp[ii],
                 dut.rs_vs[ii],dut.rs_vt[ii],dut.rs_imm[ii]);
  endtask

  task dump_lsq_all;
    integer ii;
    $display("    LSQ entries (head=%0d tail=%0d cnt=%0d):",dut.lsq_head,dut.lsq_tail,dut.lsq_cnt);
    for(ii=0;ii<8;ii=ii+1)
      if(dut.lsq_v[ii])
        $display("      lsq[%0d]: ld=%0b st=%0b ardy=%0b drdy=%0b cmt=%0b base=0x%X imm=%0d rob=%0d",
                 ii,dut.lsq_ld[ii],dut.lsq_st[ii],dut.lsq_ardy[ii],dut.lsq_drdy[ii],
                 dut.lsq_cmt[ii],dut.lsq_base[ii],dut.lsq_imm[ii],dut.lsq_rob[ii]);
  endtask

  task dump_prf_used;
    integer ii;
    $display("    PRF (non-identity ready entries):");
    for(ii=0;ii<32;ii=ii+1)
      if(dut.rat_map[ii] != ii[5:0])
        $display("      arch r%0d -> phys p%0d val=0x%X rdy=%0b",
                 ii,dut.rat_map[ii],dut.prf[dut.rat_map[ii]],dut.prf_rdy[dut.rat_map[ii]]);
  endtask

  task dump_regs_used;
    integer ii;
    $display("    arch_rf / reg_file (non-zero):");
    for(ii=0;ii<32;ii=ii+1)
      if(dut.arch_rf[ii] !== 64'd0 && ii != 31)
        $display("      r%0d: arch=%0d reg=%0d",ii,dut.arch_rf[ii],dut.reg_file.registers[ii]);
  endtask

  // Run with periodic snapshots every N cycles
  task run_verbose; input integer mx; input integer snap_interval;
    integer snap;
    reset=1; @(posedge clk);#1; @(posedge clk);#1; reset=0; @(posedge clk);#1;
    g_cyc=0; g_to=0; snap=snap_interval;
    while(!hlt && g_cyc<mx) begin
      @(posedge clk);#1; g_cyc=g_cyc+1;
      if(g_cyc==snap) begin
        $display("  [snap @%0d cyc]",g_cyc);
        dump_basic; dump_rob_head; dump_rs_all;
        snap=snap+snap_interval;
      end
    end
    if(!hlt) g_to=1;
  endtask

  // ===========================================================
  // SIMPLE: addi + halt (sanity check)
  // ===========================================================
  task t_sanity; $display("SANITY: addi r1,42 + halt");  clear_mem;
    load_w(64'h2000,f_addi(5'd1,12'd42));
    load_w(64'h2004,f_halt());
    run(200);
    if(g_to) begin $display("    TIMEOUT"); fail_n=fail_n+1; dump_basic; dump_rob_head; dump_rs_all; end
    else begin $display("    %0d cycles",g_cyc); chk("r1(=42)",dut.reg_file.registers[1],64'd42); end
    $display(""); endtask

  // ===========================================================
  // BR-1: single unconditional br
  // ===========================================================
  task t_br1; $display("BR-1: single br (br r1 -> 0x2020)");  clear_mem;
    load_w(64'h2000,f_addi(5'd1,12'd514));  load_w(64'h2004,f_shftli(5'd1,12'd4)); // r1=8224=0x2020
    load_w(64'h2008,f_addi(5'd5,12'd99));
    load_w(64'h200C,f_br(5'd1));
    load_w(64'h2010,f_addi(5'd5,12'd11));   // MUST NOT run
    load_w(64'h2014,f_halt());
    load_w(64'h2020,f_addi(5'd6,12'd77));
    load_w(64'h2024,f_halt());
    run_verbose(`TIMEOUT, 50);
    if(g_to) begin
      $display("    TIMEOUT after %0d cycles",g_cyc);
      fail_n=fail_n+1;
      dump_basic; dump_rob_head; dump_rs_all; dump_prf_used; dump_regs_used;
    end else begin
      $display("    halted in %0d cycles",g_cyc);
      chk("r1(=8224)",dut.reg_file.registers[1],64'd8224);
      chk("r5(=99)",dut.reg_file.registers[5],64'd99);
      chk("r6(=77)",dut.reg_file.registers[6],64'd77);
    end
    $display(""); endtask

  // ===========================================================
  // BR-2: chain of 3 br instructions
  // ===========================================================
  task t_br2; $display("BR-2: 3-br chain");  clear_mem;
    load_w(64'h2000,f_addi(5'd1,12'd516));  load_w(64'h2004,f_shftli(5'd1,12'd4)); // r1=8256=0x2040
    load_w(64'h2008,f_addi(5'd2,12'd516));  load_w(64'h200C,f_shftli(5'd2,12'd4));
    load_w(64'h2010,f_addi(5'd2,12'd8));    // r2=8264=0x2048
    load_w(64'h2014,f_addi(5'd3,12'd516));  load_w(64'h2018,f_shftli(5'd3,12'd4));
    load_w(64'h201C,f_addi(5'd3,12'd16));   // r3=8272=0x2050
    load_w(64'h2020,f_br(5'd1));
    load_w(64'h2024,f_halt());
    load_w(64'h2040,f_br(5'd2)); load_w(64'h2044,f_halt());
    load_w(64'h2048,f_br(5'd3)); load_w(64'h204C,f_halt());
    load_w(64'h2050,f_addi(5'd7,12'd42));
    load_w(64'h2054,f_halt());
    run_verbose(`TIMEOUT, 100);
    if(g_to) begin
      $display("    TIMEOUT after %0d cycles",g_cyc);
      fail_n=fail_n+1;
      dump_basic; dump_rob_head; dump_rs_all; dump_prf_used; dump_regs_used;
      $display("    r1=%0d r2=%0d r3=%0d r7=%0d",
               dut.reg_file.registers[1],dut.reg_file.registers[2],
               dut.reg_file.registers[3],dut.reg_file.registers[7]);
    end else begin
      $display("    halted in %0d cycles",g_cyc);
      chk("r7(=42)",dut.reg_file.registers[7],64'd42);
    end
    $display(""); endtask

  // ===========================================================
  // BRNZ-1: brnz countdown loop, 5 iters
  // ===========================================================
  task t_brnz1; $display("BRNZ-1: brnz loop 5 iters");  clear_mem;
    load_w(64'h2000,f_addi(5'd1,12'd513));  load_w(64'h2004,f_shftli(5'd1,12'd4)); // r1=8208=0x2010
    load_w(64'h2008,f_addi(5'd2,12'd5));
    load_w(64'h200C,f_br(5'd1));
    load_w(64'h2010,f_subi(5'd2,12'd1));
    load_w(64'h2014,f_brnz(5'd1,5'd2));
    load_w(64'h2018,f_addi(5'd3,12'd99));
    load_w(64'h201C,f_halt());
    run_verbose(5000, 500);
    if(g_to) begin
      $display("    TIMEOUT after %0d cycles",g_cyc);
      fail_n=fail_n+1;
      dump_basic; dump_rob_head; dump_rs_all; dump_prf_used; dump_regs_used;
      $display("    r1=%0d r2=%0d r3=%0d",
               dut.reg_file.registers[1],dut.reg_file.registers[2],dut.reg_file.registers[3]);
    end else begin
      $display("    halted in %0d cycles",g_cyc);
      chk("r2(=0)",dut.reg_file.registers[2],64'd0);
      chk("r3(=99)",dut.reg_file.registers[3],64'd99);
    end
    $display(""); endtask

  // ===========================================================
  // BRNZ-BR: brnz + 2-br chain, 3 iters
  // ===========================================================
  task t_brnz_br; $display("BRNZ-BR: brnz+2-br-chain 3 iters");  clear_mem;
    load_w(64'h2000,f_addi(5'd1,12'd516));  load_w(64'h2004,f_shftli(5'd1,12'd4)); // r1=0x2040
    load_w(64'h2008,f_addi(5'd2,12'd516));  load_w(64'h200C,f_shftli(5'd2,12'd4));
    load_w(64'h2010,f_addi(5'd2,12'd8));    // r2=0x2048
    load_w(64'h2014,f_addi(5'd3,12'd514));  load_w(64'h2018,f_shftli(5'd3,12'd4)); // r3=0x2020
    load_w(64'h201C,f_addi(5'd10,12'd3));
    load_w(64'h2020,f_subi(5'd10,12'd1));
    load_w(64'h2024,f_brnz(5'd1,5'd10));
    load_w(64'h2028,f_addi(5'd11,12'd55));
    load_w(64'h202C,f_halt());
    load_w(64'h2040,f_br(5'd2)); load_w(64'h2044,f_halt());
    load_w(64'h2048,f_br(5'd3)); load_w(64'h204C,f_halt());
    run_verbose(5000, 500);
    if(g_to) begin
      $display("    TIMEOUT after %0d cycles",g_cyc);
      fail_n=fail_n+1;
      dump_basic; dump_rob_head; dump_rs_all; dump_prf_used; dump_regs_used;
      $display("    r1=%0d r2=%0d r3=%0d r10=%0d r11=%0d",
               dut.reg_file.registers[1],dut.reg_file.registers[2],
               dut.reg_file.registers[3],dut.reg_file.registers[10],
               dut.reg_file.registers[11]);
    end else begin
      $display("    halted in %0d cycles",g_cyc);
      chk("r10(=0)",dut.reg_file.registers[10],64'd0);
      chk("r11(=55)",dut.reg_file.registers[11],64'd55);
    end
    $display(""); endtask

  // ===========================================================
  // MEM-1: simple store + load round-trip
  // ===========================================================
  task t_mem1; $display("MEM-1: store+load round-trip");  clear_mem;
    load_w(64'h2000,f_addi(5'd1,12'd2748));
    load_w(64'h2004,f_addi(5'd2,12'd256));   load_w(64'h2008,f_shftli(5'd2,12'd8));
    load_w(64'h200C,f_store(5'd2,5'd1,12'd0));
    load_w(64'h2010,f_load(5'd3,5'd2,12'd0));
    load_w(64'h2014,f_halt());
    run_verbose(`TIMEOUT, 50);
    if(g_to) begin
      $display("    TIMEOUT after %0d cycles",g_cyc);
      fail_n=fail_n+1;
      dump_basic; dump_rob_head; dump_rs_all; dump_lsq_all; dump_prf_used;
      $display("    r1=%0d r2=%0d r3=%0d",
               dut.reg_file.registers[1],dut.reg_file.registers[2],dut.reg_file.registers[3]);
    end else begin
      $display("    halted in %0d cycles",g_cyc);
      chk("r3(=2748)",dut.reg_file.registers[3],64'd2748);
    end
    $display(""); endtask

  // ===========================================================
  // MEM-2: load loop - sum 4 values
  // ===========================================================
  task t_mem2; $display("MEM-2: load loop sum 4 values");  clear_mem;
    load_d(64'h10000,64'd1); load_d(64'h10008,64'd2);
    load_d(64'h10010,64'd3); load_d(64'h10018,64'd4);
    load_w(64'h2000,f_addi(5'd1,12'd256));   load_w(64'h2004,f_shftli(5'd1,12'd8));
    load_w(64'h2008,f_xor3(5'd2,5'd2,5'd2));
    load_w(64'h200C,f_addi(5'd3,12'd4));
    load_w(64'h2010,f_addi(5'd5,12'd8));
    load_w(64'h2014,f_addi(5'd4,12'd514));    load_w(64'h2018,f_shftli(5'd4,12'd4));
    load_w(64'h201C,f_addi(5'd4,12'd4));      // r4=0x2024
    load_w(64'h2020,f_br(5'd4));
    load_w(64'h2024,f_load(5'd6,5'd1,12'd0));
    load_w(64'h2028,f_add3(5'd2,5'd2,5'd6));
    load_w(64'h202C,f_add3(5'd1,5'd1,5'd5));
    load_w(64'h2030,f_subi(5'd3,12'd1));
    load_w(64'h2034,f_brnz(5'd4,5'd3));
    load_w(64'h2038,f_halt());
    run_verbose(5000, 500);
    if(g_to) begin
      $display("    TIMEOUT after %0d cycles",g_cyc);
      fail_n=fail_n+1;
      dump_basic; dump_rob_head; dump_rs_all; dump_lsq_all; dump_prf_used; dump_regs_used;
    end else begin
      $display("    halted in %0d cycles",g_cyc);
      chk("r2(sum=10)",dut.reg_file.registers[2],64'd10);
      chk("r3(=0)",dut.reg_file.registers[3],64'd0);
    end
    $display(""); endtask

  // ===========================================================
  // MEM-3: store then load-verify
  // ===========================================================
  task t_mem3; $display("MEM-3: store then load-verify (no branches)");  clear_mem;
    load_w(64'h2000,f_addi(5'd7,12'd256));   load_w(64'h2004,f_shftli(5'd7,12'd8));
    load_w(64'h2008,f_addi(5'd1,12'd10));
    load_w(64'h200C,f_addi(5'd2,12'd20));
    load_w(64'h2010,f_addi(5'd3,12'd30));
    load_w(64'h2014,f_store(5'd7,5'd1,12'd0));
    load_w(64'h2018,f_store(5'd7,5'd2,12'd8));
    load_w(64'h201C,f_store(5'd7,5'd3,12'd16));
    load_w(64'h2020,f_load(5'd4,5'd7,12'd0));
    load_w(64'h2024,f_load(5'd5,5'd7,12'd8));
    load_w(64'h2028,f_load(5'd6,5'd7,12'd16));
    load_w(64'h202C,f_add3(5'd9,5'd4,5'd5));
    load_w(64'h2030,f_add3(5'd9,5'd9,5'd6));
    load_w(64'h2034,f_halt());
    run_verbose(`TIMEOUT, 100);
    if(g_to) begin
      $display("    TIMEOUT after %0d cycles",g_cyc);
      fail_n=fail_n+1;
      dump_basic; dump_rob_head; dump_rs_all; dump_lsq_all; dump_prf_used; dump_regs_used;
    end else begin
      $display("    halted in %0d cycles",g_cyc);
      chk("r9(=60)",dut.reg_file.registers[9],64'd60);
    end
    $display(""); endtask

  // ===========================================================
  // WIN-1: many independent addi into r20-r27 (OOO window test)
  // ===========================================================
  function [31:0] f_mov_reg; input [4:0] rd,rs; f_mov_reg={5'h11,rd,rs,17'd0}; endfunction
  task t_win1; $display("WIN-1: independent addi r20-r27"); clear_mem;
    // addi r20,1 .. addi r27,8 then halt
    load_w(64'h2000,f_addi(5'd20,12'd1));
    load_w(64'h2004,f_addi(5'd21,12'd2));
    load_w(64'h2008,f_addi(5'd22,12'd3));
    load_w(64'h200C,f_addi(5'd23,12'd4));
    load_w(64'h2010,f_addi(5'd24,12'd5));
    load_w(64'h2014,f_addi(5'd25,12'd6));
    load_w(64'h2018,f_addi(5'd26,12'd7));
    load_w(64'h201C,f_addi(5'd27,12'd8));
    load_w(64'h2020,f_halt());
    run(200);
    if(g_to) begin $display("    TIMEOUT"); fail_n=fail_n+1; dump_basic; dump_rob_head; dump_rs_all; end
    else begin
      $display("    halted in %0d cycles",g_cyc);
      chk("r20(=1)",dut.reg_file.registers[20],64'd1);
      chk("r21(=2)",dut.reg_file.registers[21],64'd2);
      chk("r22(=3)",dut.reg_file.registers[22],64'd3);
      chk("r23(=4)",dut.reg_file.registers[23],64'd4);
      chk("r24(=5)",dut.reg_file.registers[24],64'd5);
      chk("r25(=6)",dut.reg_file.registers[25],64'd6);
      chk("r26(=7)",dut.reg_file.registers[26],64'd7);
      chk("r27(=8)",dut.reg_file.registers[27],64'd8);
    end
    $display(""); endtask

  // ===========================================================
  // WIN-2: deep dependency chain filling ROB (>8 instructions)
  // ===========================================================
  task t_win2; $display("WIN-2: 16-instr chain with dep + loop"); clear_mem;
    // r1=1, r2=2,...,r8=8 then add r20=r1+r2, r21=r3+r4, r22=r5+r6, r23=r7+r8
    // r24=r20+r21, r25=r22+r23, r26=r24+r25
    load_w(64'h2000,f_addi(5'd1,12'd1));
    load_w(64'h2004,f_addi(5'd2,12'd2));
    load_w(64'h2008,f_addi(5'd3,12'd3));
    load_w(64'h200C,f_addi(5'd4,12'd4));
    load_w(64'h2010,f_addi(5'd5,12'd5));
    load_w(64'h2014,f_addi(5'd6,12'd6));
    load_w(64'h2018,f_addi(5'd7,12'd7));
    load_w(64'h201C,f_addi(5'd8,12'd8));
    load_w(64'h2020,f_add3(5'd20,5'd1,5'd2));  // r20 = 3
    load_w(64'h2024,f_add3(5'd21,5'd3,5'd4));  // r21 = 7
    load_w(64'h2028,f_add3(5'd22,5'd5,5'd6));  // r22 = 11
    load_w(64'h202C,f_add3(5'd23,5'd7,5'd8));  // r23 = 15
    load_w(64'h2030,f_add3(5'd24,5'd20,5'd21)); // r24 = 10
    load_w(64'h2034,f_add3(5'd25,5'd22,5'd23)); // r25 = 26
    load_w(64'h2038,f_add3(5'd26,5'd24,5'd25)); // r26 = 36
    load_w(64'h203C,f_halt());
    run(500);
    if(g_to) begin $display("    TIMEOUT"); fail_n=fail_n+1; dump_basic; dump_rob_head; dump_rs_all; end
    else begin
      $display("    halted in %0d cycles",g_cyc);
      chk("r20(=3)",dut.reg_file.registers[20],64'd3);
      chk("r21(=7)",dut.reg_file.registers[21],64'd7);
      chk("r24(=10)",dut.reg_file.registers[24],64'd10);
      chk("r26(=36)",dut.reg_file.registers[26],64'd36);
    end
    $display(""); endtask

  // ===========================================================
  // BRGT-1: brgt taken (r1=5 > r2=3, jump to 0x2020)
  // ===========================================================
  task t_brgt1; $display("BRGT-1: brgt taken (r1=5>r2=3, jump to halt)"); clear_mem;
    // r1=5
    load_w(64'h2000,f_addi(5'd1,12'd5));
    // r2=3
    load_w(64'h2004,f_addi(5'd2,12'd3));
    // r3 = 0x2020 (target): addi r3,514; shftli r3,4
    load_w(64'h2008,f_addi(5'd3,12'd514));
    load_w(64'h200C,f_shftli(5'd3,12'd4));
    // brgt r3, r1, r2 (branch to r3 if r1>r2)
    load_w(64'h2010,f_brgt(5'd3,5'd1,5'd2));
    // not-taken path: addi r5,99
    load_w(64'h2014,f_addi(5'd5,12'd99));
    load_w(64'h2018,f_halt()); // dead end (not reached if taken)
    // taken path at 0x2020: halt
    load_w(64'h2020,f_halt());
    run(200);
    if(g_to) begin $display("    TIMEOUT"); fail_n=fail_n+1; dump_basic; dump_rob_head; dump_rs_all; end
    else begin
      $display("    halted in %0d cycles",g_cyc);
      chk("r5(=0,not-taken-path-skipped)",dut.reg_file.registers[5],64'd0);
    end
    $display(""); endtask

  // BRGT-2: brgt not-taken (r1=3 < r2=5, fall through)
  task t_brgt2; $display("BRGT-2: brgt not-taken (r1=3<r2=5, fallthrough)"); clear_mem;
    load_w(64'h2000,f_addi(5'd1,12'd3));
    load_w(64'h2004,f_addi(5'd2,12'd5));
    load_w(64'h2008,f_addi(5'd3,12'd514));
    load_w(64'h200C,f_shftli(5'd3,12'd4)); // r3=0x2020
    load_w(64'h2010,f_brgt(5'd3,5'd1,5'd2));
    // not-taken: fall through
    load_w(64'h2014,f_addi(5'd5,12'd77));
    load_w(64'h2018,f_halt());
    load_w(64'h2020,f_halt()); // in case taken (wrong)
    run(200);
    if(g_to) begin $display("    TIMEOUT"); fail_n=fail_n+1; dump_basic; dump_rob_head; dump_rs_all; end
    else begin
      $display("    halted in %0d cycles",g_cyc);
      chk("r5(=77,fallthrough)",dut.reg_file.registers[5],64'd77);
    end
    $display(""); endtask

  // ===========================================================
  // MEM-4: 16 sequential loads from base address (r0=65536)
  // ===========================================================
  task t_mem4; integer k; $display("MEM-4: 16 sequential loads into r8-r23"); clear_mem;
    // preload data: m[65536+8k] = k+1 for k=0..15
    for (k=0;k<16;k=k+1) load_d(64'd65536 + k*8, k+1);
    // set r1=65536 via addi r1,1; shftli r1,16
    load_w(64'h2000,f_addi(5'd1,12'd1));
    load_w(64'h2004,f_shftli(5'd1,12'd16));
    // loads r8-r23 from (r1)(0..120)
    load_w(64'h2008,f_load(5'd8, 5'd1, 12'd0));
    load_w(64'h200C,f_load(5'd9, 5'd1, 12'd8));
    load_w(64'h2010,f_load(5'd10,5'd1, 12'd16));
    load_w(64'h2014,f_load(5'd11,5'd1, 12'd24));
    load_w(64'h2018,f_load(5'd12,5'd1, 12'd32));
    load_w(64'h201C,f_load(5'd13,5'd1, 12'd40));
    load_w(64'h2020,f_load(5'd14,5'd1, 12'd48));
    load_w(64'h2024,f_load(5'd15,5'd1, 12'd56));
    load_w(64'h2028,f_load(5'd16,5'd1, 12'd64));
    load_w(64'h202C,f_load(5'd17,5'd1, 12'd72));
    load_w(64'h2030,f_load(5'd18,5'd1, 12'd80));
    load_w(64'h2034,f_load(5'd19,5'd1, 12'd88));
    load_w(64'h2038,f_load(5'd20,5'd1, 12'd96));
    load_w(64'h203C,f_load(5'd21,5'd1, 12'd104));
    load_w(64'h2040,f_load(5'd22,5'd1, 12'd112));
    load_w(64'h2044,f_load(5'd23,5'd1, 12'd120));
    load_w(64'h2048,f_halt());
    run(500);
    if(g_to) begin $display("    TIMEOUT"); fail_n=fail_n+1; dump_basic; dump_rob_head; dump_rs_all; dump_lsq_all; end
    else begin
      $display("    halted in %0d cycles",g_cyc);
      chk("r8(=1)",  dut.reg_file.registers[8],  64'd1);
      chk("r9(=2)",  dut.reg_file.registers[9],  64'd2);
      chk("r12(=5)", dut.reg_file.registers[12], 64'd5);
      chk("r15(=8)", dut.reg_file.registers[15], 64'd8);
      chk("r16(=9)", dut.reg_file.registers[16], 64'd9);
      chk("r19(=12)",dut.reg_file.registers[19], 64'd12);
      chk("r20(=13)",dut.reg_file.registers[20], 64'd13);
      chk("r23(=16)",dut.reg_file.registers[23], 64'd16);
    end
    $display(""); endtask

  // ===========================================================
  // MAIN
  // ===========================================================
  initial begin
    pass_n=0; fail_n=0;
    $display("================================================================");
    $display("   Enhanced Diagnostic Testbench  —  tinker_core");
    $display("================================================================");
    $display("");
    $display("--- SANITY ---");
    t_sanity;
    $display("--- BRANCH TESTS ---");
    t_br1; t_br2; t_brnz1; t_brnz_br;
    $display("--- BRGT TESTS ---");
    t_brgt1; t_brgt2;
    $display("--- MEMORY TESTS ---");
    t_mem1; t_mem2; t_mem3; t_mem4;
    $display("--- WINDOW TESTS ---");
    t_win1; t_win2;
    $display("================================================================");
    $display("  PASS: %0d    FAIL: %0d    TOTAL: %0d",pass_n,fail_n,pass_n+fail_n);
    $display("================================================================");
    $finish;
  end

endmodule