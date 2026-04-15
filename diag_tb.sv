// diag_tb.sv — Targeted diagnostic testbench: branch + memory failures
// Tests progressively harder cases; first failure pinpoints the exact bug.
//
// Compile (from project root):
//   iverilog -g2005-sv -o diag_tb.vvp diag_tb.sv tinker.sv \
//            hdl/alu.sv hdl/fpu.sv hdl/reg_file.sv hdl/decoder.sv \
//            hdl/fetch.sv hdl/memory.sv
// Run:  vvp diag_tb.vvp

`timescale 1ns/1ps
`define MEM_SIZE  (512 * 1024)
`define TIMEOUT   20000

module diag_tb;

  reg  clk, reset;
  wire hlt;
  tinker_core dut(.clk(clk), .reset(reset), .hlt(hlt));
  initial clk = 0;
  always  #5 clk = ~clk;

  // ---- Instruction encoders ----
  function [31:0] f_addi;   input [4:0] rd; input [11:0] imm; f_addi  ={5'h19,rd,17'd0,imm}; endfunction
  function [31:0] f_subi;   input [4:0] rd; input [11:0] imm; f_subi  ={5'h1B,rd,17'd0,imm}; endfunction
  function [31:0] f_xor3;   input [4:0] rd,rs,rt;             f_xor3  ={5'h02,rd,rs,rt,12'd0}; endfunction
  function [31:0] f_add3;   input [4:0] rd,rs,rt;             f_add3  ={5'h18,rd,rs,rt,12'd0}; endfunction
  function [31:0] f_shftli; input [4:0] rd; input [11:0] imm; f_shftli={5'h07,rd,17'd0,imm}; endfunction
  function [31:0] f_br;     input [4:0] rd;                   f_br    ={5'h08,rd,22'd0}; endfunction
  function [31:0] f_brnz;   input [4:0] rd,rs;                f_brnz  ={5'h0B,rd,rs,17'd0}; endfunction
  function [31:0] f_load;   input [4:0] rd,rs; input [11:0] imm; f_load={5'h10,rd,rs,12'd0,imm}; endfunction
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
  task dump;
    $display("    pc=0x%X rob_cnt=%0d fl_cnt=%0d rs_cnt=%0d redirect=%0b",
             dut.pc_reg,dut.rob_cnt,dut.fl_cnt,dut.rs_cnt,dut.redirect_en);
    $display("    arch: r1=%0d r2=%0d r3=%0d r4=%0d r5=%0d r6=%0d r10=%0d r11=%0d",
             dut.arch_rf[1],dut.arch_rf[2],dut.arch_rf[3],dut.arch_rf[4],
             dut.arch_rf[5],dut.arch_rf[6],dut.arch_rf[10],dut.arch_rf[11]);
    $display("    regs: r1=%0d r2=%0d r3=%0d r4=%0d r5=%0d r6=%0d r10=%0d r11=%0d",
             dut.reg_file.registers[1],dut.reg_file.registers[2],dut.reg_file.registers[3],
             dut.reg_file.registers[4],dut.reg_file.registers[5],dut.reg_file.registers[6],
             dut.reg_file.registers[10],dut.reg_file.registers[11]);
  endtask

  // ===========================================================
  // BR-1: single unconditional br
  // preamble builds r1=0x2020, br r1 skips addi r5,11, lands at addi r6,77
  // ===========================================================
  task t_br1; $display("BR-1: single br");  clear_mem;
    load_w(64'h2000,f_addi(5'd1,12'd514));  load_w(64'h2004,f_shftli(5'd1,12'd4)); // r1=8224=0x2020
    load_w(64'h2008,f_addi(5'd5,12'd99));   // r5=99
    load_w(64'h200C,f_br(5'd1));             // -> 0x2020
    load_w(64'h2010,f_addi(5'd5,12'd11));   // MUST NOT run
    load_w(64'h2014,f_halt());
    load_w(64'h2020,f_addi(5'd6,12'd77));   // r6=77 <- must reach
    load_w(64'h2024,f_halt());
    run(`TIMEOUT);
    if(g_to) begin $display("    TIMEOUT"); fail_n=fail_n+1; dump; end
    else     begin $display("    %0d cycles",g_cyc); chk("r5(=99)",dut.reg_file.registers[5],64'd99);
                   chk("r6(=77)",dut.reg_file.registers[6],64'd77); end
    $display(""); endtask

  // ===========================================================
  // BR-2: chain of 3 br instructions
  // r1=0x2040, r2=0x2048, r3=0x2050; br r1->br r2->br r3->addi r7,42
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
    load_w(64'h2050,f_addi(5'd7,12'd42));   // <- must reach
    load_w(64'h2054,f_halt());
    run(`TIMEOUT);
    if(g_to) begin $display("    TIMEOUT"); fail_n=fail_n+1; dump;
      $display("    r1=%0d r2=%0d r3=%0d",dut.reg_file.registers[1],
               dut.reg_file.registers[2],dut.reg_file.registers[3]); end
    else     begin $display("    %0d cycles",g_cyc); chk("r7(=42)",dut.reg_file.registers[7],64'd42); end
    $display(""); endtask

  // ===========================================================
  // BR-3: brnz countdown loop, 5 iters, no br chain
  // r1=loop_addr=0x2010, r2=5; loop: subi r2,1; brnz r2,r1
  // ===========================================================
  task t_brnz3; $display("BR-3: brnz loop 5 iters");  clear_mem;
    load_w(64'h2000,f_addi(5'd1,12'd513));  load_w(64'h2004,f_shftli(5'd1,12'd4)); // r1=8208=0x2010
    load_w(64'h2008,f_addi(5'd2,12'd5));    // r2=5
    load_w(64'h200C,f_br(5'd1));             // jump to loop
    load_w(64'h2010,f_subi(5'd2,12'd1));    // r2--
    load_w(64'h2014,f_brnz(5'd1,5'd2));     // if r2!=0: jump to r1
    load_w(64'h2018,f_addi(5'd3,12'd99));   // r3=99
    load_w(64'h201C,f_halt());
    run(`TIMEOUT);
    if(g_to) begin $display("    TIMEOUT"); fail_n=fail_n+1; dump; end
    else     begin $display("    %0d cycles",g_cyc);
                   chk("r2(=0)",dut.reg_file.registers[2],64'd0);
                   chk("r3(=99)",dut.reg_file.registers[3],64'd99); end
    $display(""); endtask

  // ===========================================================
  // BR-4: brnz + 2-br chain, 3 iters  (mirrors branch_loop structure)
  // r1=0x2040 (first br), r2=0x2048 (second br), r3=0x2020 (subi)
  // r10=3; loop: subi->brnz->br r2->br r3->subi
  // ===========================================================
  task t_brnz_br4; $display("BR-4: brnz+2-br-chain 3 iters");  clear_mem;
    load_w(64'h2000,f_addi(5'd1,12'd516));  load_w(64'h2004,f_shftli(5'd1,12'd4)); // r1=8256=0x2040
    load_w(64'h2008,f_addi(5'd2,12'd516));  load_w(64'h200C,f_shftli(5'd2,12'd4));
    load_w(64'h2010,f_addi(5'd2,12'd8));    // r2=8264=0x2048
    load_w(64'h2014,f_addi(5'd3,12'd514));  load_w(64'h2018,f_shftli(5'd3,12'd4)); // r3=8224=0x2020
    load_w(64'h201C,f_addi(5'd10,12'd3));   // r10=3
    load_w(64'h2020,f_subi(5'd10,12'd1));   // r10--  (loop body)
    load_w(64'h2024,f_brnz(5'd1,5'd10));    // if r10!=0 -> r1=0x2040
    load_w(64'h2028,f_addi(5'd11,12'd55));  // r11=55
    load_w(64'h202C,f_halt());
    load_w(64'h2040,f_br(5'd2)); load_w(64'h2044,f_halt());  // br r2->0x2048
    load_w(64'h2048,f_br(5'd3)); load_w(64'h204C,f_halt());  // br r3->0x2020
    run(5000);
    if(g_to) begin $display("    TIMEOUT"); fail_n=fail_n+1; dump;
      $display("    r1=%0d r2=%0d r3=%0d r10=%0d r11=%0d",
               dut.reg_file.registers[1],dut.reg_file.registers[2],
               dut.reg_file.registers[3],dut.reg_file.registers[10],
               dut.reg_file.registers[11]); end
    else     begin $display("    %0d cycles",g_cyc);
                   chk("r10(=0)",dut.reg_file.registers[10],64'd0);
                   chk("r11(=55)",dut.reg_file.registers[11],64'd55); end
    $display(""); endtask

  // ===========================================================
  // BR-5: brnz + 5-br chain, 10 iters (exact branch_loop topology)
  // ===========================================================
  task t_brnz_br5; $display("BR-5: brnz+5-br-chain 10 iters (branch_loop topology)");  clear_mem;
    // r1=0x2060(br-chain start), r2..r5=chain links, r6=subi(=0x2040), r10=10
    load_w(64'h2000,f_addi(5'd1,12'd518));  load_w(64'h2004,f_shftli(5'd1,12'd4)); // r1=8288=0x2060
    load_w(64'h2008,f_addi(5'd2,12'd518));  load_w(64'h200C,f_shftli(5'd2,12'd4));
    load_w(64'h2010,f_addi(5'd2,12'd8));    // r2=8296=0x2068
    load_w(64'h2014,f_addi(5'd3,12'd519));  load_w(64'h2018,f_shftli(5'd3,12'd4)); // r3=8304=0x2070
    load_w(64'h201C,f_addi(5'd4,12'd519));  load_w(64'h2020,f_shftli(5'd4,12'd4));
    load_w(64'h2024,f_addi(5'd4,12'd8));    // r4=8312=0x2078
    load_w(64'h2028,f_addi(5'd5,12'd520));  load_w(64'h202C,f_shftli(5'd5,12'd4)); // r5=8320=0x2080
    load_w(64'h2030,f_addi(5'd6,12'd516));  load_w(64'h2034,f_shftli(5'd6,12'd4)); // r6=8256=0x2040
    load_w(64'h2038,f_addi(5'd10,12'd10));  // r10=10
    load_w(64'h203C,f_br(5'd6));             // jump to 0x2040 (loop body)
    // loop body:
    load_w(64'h2040,f_subi(5'd10,12'd1));
    load_w(64'h2044,f_brnz(5'd1,5'd10));    // if r10!=0 -> r1=0x2060
    load_w(64'h2048,f_addi(5'd11,12'd77));
    load_w(64'h204C,f_halt());
    // br chain:
    load_w(64'h2060,f_br(5'd2)); load_w(64'h2064,f_halt());  // -> r2=0x2068
    load_w(64'h2068,f_br(5'd3)); load_w(64'h206C,f_halt());  // -> r3=0x2070
    load_w(64'h2070,f_br(5'd4)); load_w(64'h2074,f_halt());  // -> r4=0x2078
    load_w(64'h2078,f_br(5'd5)); load_w(64'h207C,f_halt());  // -> r5=0x2080
    load_w(64'h2080,f_br(5'd6)); load_w(64'h2084,f_halt());  // -> r6=0x2040
    run(20000);
    if(g_to) begin $display("    TIMEOUT"); fail_n=fail_n+1; dump;
      $display("    r1=%0d r2=%0d r3=%0d r4=%0d r5=%0d r6=%0d r10=%0d r11=%0d",
               dut.reg_file.registers[1],dut.reg_file.registers[2],dut.reg_file.registers[3],
               dut.reg_file.registers[4],dut.reg_file.registers[5],dut.reg_file.registers[6],
               dut.reg_file.registers[10],dut.reg_file.registers[11]); end
    else     begin $display("    %0d cycles",g_cyc);
                   chk("r10(=0)",dut.reg_file.registers[10],64'd0);
                   chk("r11(=77)",dut.reg_file.registers[11],64'd77); end
    $display(""); endtask

  // ===========================================================
  // MEM-1: simple store + load round-trip
  // ===========================================================
  task t_mem1; $display("MEM-1: store+load round-trip");  clear_mem;
    load_w(64'h2000,f_addi(5'd1,12'd2748));
    load_w(64'h2004,f_addi(5'd2,12'd256));   load_w(64'h2008,f_shftli(5'd2,12'd8)); // r2=65536
    load_w(64'h200C,f_store(5'd2,5'd1,12'd0));
    load_w(64'h2010,f_load(5'd3,5'd2,12'd0));
    load_w(64'h2014,f_halt());
    run(`TIMEOUT);
    if(g_to) begin $display("    TIMEOUT"); fail_n=fail_n+1; dump; end
    else     begin $display("    %0d cycles",g_cyc); chk("r3(=2748)",dut.reg_file.registers[3],64'd2748); end
    $display(""); endtask

  // ===========================================================
  // MEM-2: load loop - sum 4 values from memory
  // ===========================================================
  task t_mem2; $display("MEM-2: load loop sum 4 values");  clear_mem;
    load_d(64'h10000,64'd1); load_d(64'h10008,64'd2);
    load_d(64'h10010,64'd3); load_d(64'h10018,64'd4);
    load_w(64'h2000,f_addi(5'd1,12'd256));   load_w(64'h2004,f_shftli(5'd1,12'd8)); // r1=65536
    load_w(64'h2008,f_xor3(5'd2,5'd2,5'd2)); // r2=0
    load_w(64'h200C,f_addi(5'd3,12'd4));      // r3=4 (count)
    load_w(64'h2010,f_addi(5'd5,12'd8));      // r5=8 (stride)
    load_w(64'h2014,f_addi(5'd4,12'd514));    load_w(64'h2018,f_shftli(5'd4,12'd4));
    load_w(64'h201C,f_addi(5'd4,12'd4));      // r4=8228=0x2024 (loop addr)
    load_w(64'h2020,f_br(5'd4));               // jump to loop
    load_w(64'h2024,f_load(5'd6,5'd1,12'd0));
    load_w(64'h2028,f_add3(5'd2,5'd2,5'd6));
    load_w(64'h202C,f_add3(5'd1,5'd1,5'd5));
    load_w(64'h2030,f_subi(5'd3,12'd1));
    load_w(64'h2034,f_brnz(5'd4,5'd3));       // if r3!=0: loop
    load_w(64'h2038,f_halt());
    run(5000);
    if(g_to) begin $display("    TIMEOUT"); fail_n=fail_n+1; dump;
      $display("    r2=%0d(exp 10) r3=%0d(exp 0)",
               dut.reg_file.registers[2],dut.reg_file.registers[3]); end
    else     begin $display("    %0d cycles",g_cyc);
                   chk("r2(sum=10)",dut.reg_file.registers[2],64'd10);
                   chk("r3(=0)",dut.reg_file.registers[3],64'd0); end
    $display(""); endtask

  // ===========================================================
  // MEM-3: store loop then load-verify (write 10,20,30 then sum)
  // ===========================================================
  task t_mem3; $display("MEM-3: store then load-verify");  clear_mem;
    load_w(64'h2000,f_addi(5'd7,12'd256));   load_w(64'h2004,f_shftli(5'd7,12'd8)); // r7=65536
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
    run(`TIMEOUT);
    if(g_to) begin $display("    TIMEOUT"); fail_n=fail_n+1; dump; end
    else     begin $display("    %0d cycles",g_cyc); chk("r9(=60)",dut.reg_file.registers[9],64'd60); end
    $display(""); endtask

  // ===========================================================
  // MEM-4: load value, use as brnz condition (3-iter loop via memory counter)
  // ===========================================================
  task t_mem4; $display("MEM-4: load-then-brnz (memory-driven loop)");  clear_mem;
    load_d(64'h10000,64'd3);  // initial counter
    load_w(64'h2000,f_addi(5'd7,12'd256));   load_w(64'h2004,f_shftli(5'd7,12'd8)); // r7=65536
    load_w(64'h2008,f_addi(5'd1,12'd514));   load_w(64'h200C,f_shftli(5'd1,12'd4));
    load_w(64'h2010,f_addi(5'd1,12'd8));     // r1=8216=0x2018 (loop addr)
    load_w(64'h2014,f_br(5'd1));
    load_w(64'h2018,f_load(5'd2,5'd7,12'd0));     // r2=mem[r7]
    load_w(64'h201C,f_subi(5'd2,12'd1));           // r2--
    load_w(64'h2020,f_store(5'd7,5'd2,12'd0));    // mem[r7]=r2
    load_w(64'h2024,f_brnz(5'd1,5'd2));            // if r2!=0: loop
    load_w(64'h2028,f_addi(5'd8,12'd55));
    load_w(64'h202C,f_halt());
    run(5000);
    if(g_to) begin $display("    TIMEOUT"); fail_n=fail_n+1; dump; end
    else     begin $display("    %0d cycles",g_cyc); chk("r8(=55)",dut.reg_file.registers[8],64'd55); end
    $display(""); endtask

  // ===========================================================
  // MAIN
  // ===========================================================
  initial begin
    pass_n=0; fail_n=0;
    $display("================================================================");
    $display("   Branch & Memory Diagnostic Testbench  —  tinker_core");
    $display("================================================================");
    $display("");
    $display("--- BRANCH TESTS ---");
    t_br1; t_br2; t_brnz3; t_brnz_br4; t_brnz_br5;
    $display("--- MEMORY TESTS ---");
    t_mem1; t_mem2; t_mem3; t_mem4;
    $display("================================================================");
    $display("  PASS: %0d    FAIL: %0d    TOTAL: %0d",pass_n,fail_n,pass_n+fail_n);
    $display("================================================================");
    $finish;
  end

endmodule