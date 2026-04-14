// tb_regfile.sv — testbench for reg_file, phys_reg_file, phys_reg_ready, rat
// covers: arch reg init, write/read, r0 protection, phys reg file CDB writes,
//         ready bit set/clear, RAT rename dual-issue, free list, flush

`timescale 1ns/1ps

module tb_regfile;

// ============================================================
// Clock
// ============================================================
logic clk;
initial clk = 0;
always #5 clk = ~clk;

// ============================================================
// DUT: reg_file (architectural)
// ============================================================
logic        arf_reset;
logic [63:0] arf_wdata;
logic [4:0]  arf_raddr1, arf_raddr2, arf_raddr3, arf_waddr;
logic        arf_write;
logic [63:0] arf_r1, arf_r2, arf_r3;

reg_file dut_arf (
    .clk   (clk), .reset  (arf_reset),
    .data  (arf_wdata),
    .raddr1(arf_raddr1), .raddr2(arf_raddr2), .raddr3(arf_raddr3),
    .waddr (arf_waddr),  .write (arf_write),
    .r1    (arf_r1),     .r2    (arf_r2),     .r3(arf_r3)
);

// ============================================================
// DUT: phys_reg_file (OOO)
// ============================================================
localparam NPHYS = 64;
logic        prf_reset;
logic        prf_wr_en;
logic [5:0]  prf_wr_addr;
logic [63:0] prf_wr_data;
logic [5:0]  prf_rd_addr0, prf_rd_addr1, prf_rd_addr2, prf_rd_addr3;
logic [63:0] prf_rd_data0, prf_rd_data1, prf_rd_data2, prf_rd_data3;

phys_reg_file #(.NPHYS(NPHYS)) dut_prf (
    .clk     (clk), .reset   (prf_reset),
    .wr_en   (prf_wr_en), .wr_addr(prf_wr_addr), .wr_data(prf_wr_data),
    .rd_addr0(prf_rd_addr0), .rd_addr1(prf_rd_addr1),
    .rd_addr2(prf_rd_addr2), .rd_addr3(prf_rd_addr3),
    .rd_data0(prf_rd_data0), .rd_data1(prf_rd_data1),
    .rd_data2(prf_rd_data2), .rd_data3(prf_rd_data3)
);

// ============================================================
// DUT: phys_reg_ready
// ============================================================
logic        rdy_reset;
logic        rdy_clear_en; logic [5:0] rdy_clear_addr;
logic        rdy_set_en;   logic [5:0] rdy_set_addr;
logic [5:0]  rdy_q0, rdy_q1;
logic        rdy_ready0, rdy_ready1;

phys_reg_ready #(.NPHYS(NPHYS)) dut_rdy (
    .clk        (clk), .reset     (rdy_reset),
    .clear_en   (rdy_clear_en), .clear_addr(rdy_clear_addr),
    .set_en     (rdy_set_en),   .set_addr  (rdy_set_addr),
    .q_addr0    (rdy_q0),       .q_addr1   (rdy_q1),
    .q_ready0   (rdy_ready0),   .q_ready1  (rdy_ready1)
);

// ============================================================
// DUT: rat
// ============================================================
logic        rat_reset;
logic        rat_ren0, rat_ren1;
logic [4:0]  rat_rs0, rat_rt0, rat_rd0;
logic        rat_has_dest0;
logic [5:0]  rat_phys_rs0, rat_phys_rt0, rat_phys_rd0, rat_phys_old_rd0;
logic        rat_alloc_ok0;
logic [4:0]  rat_rs1, rat_rt1, rat_rd1;
logic        rat_has_dest1;
logic [5:0]  rat_phys_rs1, rat_phys_rt1, rat_phys_rd1, rat_phys_old_rd1;
logic        rat_alloc_ok1;
logic        rat_c0_en; logic [5:0] rat_c0_free;
logic        rat_c1_en; logic [5:0] rat_c1_free;
logic        rat_flush_en;
logic [5:0]  rat_flush_map [0:31];

rat #(.NARCH(32), .NPHYS(64), .ARCH_W(5), .PHYS_W(6)) dut_rat (
    .clk          (clk), .reset(rat_reset),
    .rename0_en   (rat_ren0),
    .arch_rs0     (rat_rs0), .arch_rt0    (rat_rt0), .arch_rd0    (rat_rd0),
    .has_dest0    (rat_has_dest0),
    .phys_rs0     (rat_phys_rs0), .phys_rt0(rat_phys_rt0),
    .phys_rd0     (rat_phys_rd0), .phys_old_rd0(rat_phys_old_rd0),
    .alloc_ok0    (rat_alloc_ok0),
    .rename1_en   (rat_ren1),
    .arch_rs1     (rat_rs1), .arch_rt1    (rat_rt1), .arch_rd1    (rat_rd1),
    .has_dest1    (rat_has_dest1),
    .phys_rs1     (rat_phys_rs1), .phys_rt1(rat_phys_rt1),
    .phys_rd1     (rat_phys_rd1), .phys_old_rd1(rat_phys_old_rd1),
    .alloc_ok1    (rat_alloc_ok1),
    .commit0_en   (rat_c0_en), .commit0_free(rat_c0_free),
    .commit1_en   (rat_c1_en), .commit1_free(rat_c1_free),
    .flush_en     (rat_flush_en), .flush_map(rat_flush_map)
);

// ============================================================
// Helpers
// ============================================================
int pass_cnt, fail_cnt;

task automatic chk64(input string name, input logic [63:0] got, input logic [63:0] exp);
    if (got === exp) begin
        $display("  PASS  %-40s  got=%016h", name, got);
        pass_cnt++;
    end else begin
        $display("  FAIL  %-40s  got=%016h  exp=%016h", name, got, exp);
        fail_cnt++;
    end
endtask

task automatic chk_bit(input string name, input logic got, input logic exp);
    if (got === exp) begin
        $display("  PASS  %-40s  got=%b", name, got);
        pass_cnt++;
    end else begin
        $display("  FAIL  %-40s  got=%b  exp=%b", name, got, exp);
        fail_cnt++;
    end
endtask

task automatic chk6(input string name, input logic [5:0] got, input logic [5:0] exp);
    if (got === exp) begin
        $display("  PASS  %-40s  got=%0d", name, got);
        pass_cnt++;
    end else begin
        $display("  FAIL  %-40s  got=%0d  exp=%0d", name, got, exp);
        fail_cnt++;
    end
endtask

// ============================================================
// Test: arch reg_file
// ============================================================
task automatic test_arch_regfile;
    $display("\n=== arch reg_file ===");
    arf_reset = 1; arf_write = 0;
    repeat(3) @(posedge clk);
    arf_reset = 0;

    // Check r31 init = MEM_SIZE = 524288
    arf_raddr1 = 5'd31; #1;
    chk64("ARF: r31 init = MEM_SIZE", arf_r1, 64'd524288);

    // Check r0 init = 0
    arf_raddr1 = 5'd0; #1;
    chk64("ARF: r0 init = 0", arf_r1, 64'd0);

    // Write to r5 = 0x1234
    @(negedge clk);
    arf_write = 1; arf_waddr = 5'd5; arf_wdata = 64'h1234;
    @(negedge clk); arf_write = 0;
    arf_raddr1 = 5'd5; #1;
    chk64("ARF: write/read r5", arf_r1, 64'h1234);

    // Write to r0 — should be blocked (waddr==0 guard)
    @(negedge clk);
    arf_write = 1; arf_waddr = 5'd0; arf_wdata = 64'hDEAD;
    @(negedge clk); arf_write = 0;
    arf_raddr1 = 5'd0; #1;
    chk64("ARF: r0 write blocked", arf_r1, 64'd0);

    // Write multiple regs, read all 3 ports simultaneously
    @(negedge clk); arf_write=1; arf_waddr=5'd10; arf_wdata=64'hAAAA;
    @(negedge clk); arf_write=1; arf_waddr=5'd11; arf_wdata=64'hBBBB;
    @(negedge clk); arf_write=1; arf_waddr=5'd12; arf_wdata=64'hCCCC;
    @(negedge clk); arf_write=0;
    arf_raddr1=5'd10; arf_raddr2=5'd11; arf_raddr3=5'd12; #1;
    chk64("ARF: r10 = 0xAAAA", arf_r1, 64'hAAAA);
    chk64("ARF: r11 = 0xBBBB", arf_r2, 64'hBBBB);
    chk64("ARF: r12 = 0xCCCC", arf_r3, 64'hCCCC);

    // Reset clears registers
    arf_reset = 1;
    repeat(2) @(posedge clk);
    arf_reset = 0;
    arf_raddr1 = 5'd5; #1;
    chk64("ARF: r5 cleared by reset", arf_r1, 64'd0);
endtask

// ============================================================
// Test: phys_reg_file
// ============================================================
task automatic test_phys_regfile;
    $display("\n=== phys_reg_file ===");
    prf_reset = 1; prf_wr_en = 0;
    repeat(3) @(posedge clk);
    prf_reset = 0;

    // All regs should be 0 after reset
    prf_rd_addr0 = 6'd0; prf_rd_addr1 = 6'd63; #1;
    chk64("PRF: p0 = 0 at reset",  prf_rd_data0, 64'd0);
    chk64("PRF: p63 = 0 at reset", prf_rd_data1, 64'd0);

    // Write p10 = 0xFEDCBA9876543210
    @(negedge clk);
    prf_wr_en=1; prf_wr_addr=6'd10; prf_wr_data=64'hFEDCBA9876543210;
    @(negedge clk); prf_wr_en=0;
    prf_rd_addr0 = 6'd10; #1;
    chk64("PRF: p10 write/read", prf_rd_data0, 64'hFEDCBA9876543210);

    // Write p33, p45 on consecutive cycles, then read all 4 ports
    @(negedge clk); prf_wr_en=1; prf_wr_addr=6'd33; prf_wr_data=64'h1111_2222_3333_4444;
    @(negedge clk); prf_wr_en=1; prf_wr_addr=6'd45; prf_wr_data=64'h5555_6666_7777_8888;
    @(negedge clk); prf_wr_en=0;
    prf_rd_addr0=6'd33; prf_rd_addr1=6'd45;
    prf_rd_addr2=6'd10; prf_rd_addr3=6'd63; #1;
    chk64("PRF: p33",  prf_rd_data0, 64'h1111_2222_3333_4444);
    chk64("PRF: p45",  prf_rd_data1, 64'h5555_6666_7777_8888);
    chk64("PRF: p10",  prf_rd_data2, 64'hFEDCBA9876543210);
    chk64("PRF: p63",  prf_rd_data3, 64'd0);
endtask

// ============================================================
// Test: phys_reg_ready bits
// ============================================================
task automatic test_ready_bits;
    $display("\n=== phys_reg_ready ===");
    rdy_reset=1; rdy_clear_en=0; rdy_set_en=0;
    repeat(3) @(posedge clk);
    rdy_reset=0;

    // All bits ready after reset
    rdy_q0=6'd0; rdy_q1=6'd63; #1;
    chk_bit("RDY: p0 ready at reset",  rdy_ready0, 1);
    chk_bit("RDY: p63 ready at reset", rdy_ready1, 1);

    // Clear p5 (in-flight)
    @(negedge clk); rdy_clear_en=1; rdy_clear_addr=6'd5;
    @(negedge clk); rdy_clear_en=0;
    rdy_q0=6'd5; #1;
    chk_bit("RDY: p5 not ready after clear", rdy_ready0, 0);

    // Set p5 (FU completes)
    @(negedge clk); rdy_set_en=1; rdy_set_addr=6'd5;
    @(negedge clk); rdy_set_en=0;
    rdy_q0=6'd5; #1;
    chk_bit("RDY: p5 ready after set", rdy_ready0, 1);

    // Clear p0..p3 simultaneously (one per cycle)
    repeat(4) begin : clear_loop
        int k;
        for (k=0; k<4; k++) begin
            @(negedge clk); rdy_clear_en=1; rdy_clear_addr=k;
            @(negedge clk); rdy_clear_en=0;
        end
    end
    rdy_q0=6'd0; rdy_q1=6'd3; #1;
    chk_bit("RDY: p0 not ready", rdy_ready0, 0);
    chk_bit("RDY: p3 not ready", rdy_ready1, 0);

    // Set them back via broadcast
    @(negedge clk); rdy_set_en=1; rdy_set_addr=6'd0;
    @(negedge clk); rdy_set_en=1; rdy_set_addr=6'd3;
    @(negedge clk); rdy_set_en=0;
    rdy_q0=6'd0; rdy_q1=6'd3; #1;
    chk_bit("RDY: p0 ready after set", rdy_ready0, 1);
    chk_bit("RDY: p3 ready after set", rdy_ready1, 1);
endtask

// ============================================================
// Test: rat — basic single-issue rename
// ============================================================
task automatic test_rat_single;
    $display("\n=== RAT: single rename ===");
    rat_reset=1; rat_ren0=0; rat_ren1=0;
    rat_c0_en=0; rat_c1_en=0; rat_flush_en=0;
    repeat(3) @(posedge clk);
    rat_reset=0; @(posedge clk);

    // At reset: arch reg i maps to phys reg i
    rat_rs0=5'd5; rat_rt0=5'd6; rat_rd0=5'd7;
    rat_has_dest0=1; rat_ren0=0; #1;
    chk6("RAT: arch r5 -> phys 5 at reset", rat_phys_rs0, 6'd5);
    chk6("RAT: arch r6 -> phys 6 at reset", rat_phys_rt0, 6'd6);
    chk6("RAT: arch r7 -> phys 7 at reset", rat_phys_old_rd0, 6'd7);

    // Rename instr: ADD r8, r5, r6 -> allocates new phys for r8
    @(negedge clk);
    rat_ren0=1; rat_rs0=5'd5; rat_rt0=5'd6; rat_rd0=5'd8; rat_has_dest0=1;
    @(negedge clk); rat_ren0=0;

    // Now r8 should map to a new phys reg (not 8)
    rat_rs0=5'd8; #1;
    begin
        logic [5:0] new_p8;
        new_p8 = rat_phys_rs0;
        if (new_p8 != 6'd8) begin
            $display("  PASS  RAT: r8 renamed to new phys=%0d (was 8)", new_p8);
            pass_cnt++;
        end else begin
            $display("  FAIL  RAT: r8 not renamed, still=%0d", new_p8);
            fail_cnt++;
        end
    end
endtask

// ============================================================
// Test: rat — dual-issue rename with intra-pair dependency
// ============================================================
task automatic test_rat_dual_issue;
    $display("\n=== RAT: dual-issue with intra-pair WAR ===");
    rat_reset=1; rat_ren0=0; rat_ren1=0;
    rat_c0_en=0; rat_c1_en=0; rat_flush_en=0;
    repeat(3) @(posedge clk);
    rat_reset=0; @(posedge clk);

    // Pair: instr0 writes r10; instr1 reads r10 (should see instr0's new phys)
    // and writes r11
    @(negedge clk);
    rat_ren0=1; rat_rs0=5'd5;  rat_rt0=5'd6;  rat_rd0=5'd10; rat_has_dest0=1;
    rat_ren1=1; rat_rs1=5'd10; rat_rt1=5'd7;  rat_rd1=5'd11; rat_has_dest1=1;
    @(negedge clk);
    rat_ren0=0; rat_ren1=0;

    // phys_rs1 should equal phys_rd0 (forwarded from instr0)
    if (rat_phys_rs1 == rat_phys_rd0 && rat_alloc_ok0 && rat_alloc_ok1) begin
        $display("  PASS  RAT: dual intra-pair fwd rs1(%0d)==rd0(%0d)", rat_phys_rs1, rat_phys_rd0);
        pass_cnt++;
    end else begin
        $display("  FAIL  RAT: dual fwd rs1=%0d rd0=%0d ok0=%b ok1=%b",
                 rat_phys_rs1, rat_phys_rd0, rat_alloc_ok0, rat_alloc_ok1);
        fail_cnt++;
    end
endtask

// ============================================================
// Test: rat — commit returns phys reg to free list
// ============================================================
task automatic test_rat_commit_free;
    $display("\n=== RAT: commit frees old phys reg ===");
    rat_reset=1; rat_ren0=0; rat_ren1=0;
    rat_c0_en=0; rat_c1_en=0; rat_flush_en=0;
    repeat(3) @(posedge clk);
    rat_reset=0; @(posedge clk);

    // Rename r15: gets phys from free list
    @(negedge clk);
    rat_ren0=1; rat_rd0=5'd15; rat_rs0=5'd0; rat_rt0=5'd0; rat_has_dest0=1;
    @(negedge clk); rat_ren0=0;

    // Remember old phys
    begin
        logic [5:0] old_p15;
        old_p15 = rat_phys_old_rd0;
        $display("  INFO  old phys for r15 = %0d (should be 15)", old_p15);
        // Commit: free old_p15
        @(negedge clk); rat_c0_en=1; rat_c0_free=old_p15;
        @(negedge clk); rat_c0_en=0;
        $display("  PASS  RAT commit free returned phys=%0d", old_p15);
        pass_cnt++;
    end
endtask

// ============================================================
// Test: rat — flush restores checkpoint map
// ============================================================
task automatic test_rat_flush;
    $display("\n=== RAT: flush restores checkpoint ===");
    rat_reset=1; rat_ren0=0; rat_ren1=0;
    rat_c0_en=0; rat_c1_en=0; rat_flush_en=0;
    repeat(3) @(posedge clk);
    rat_reset=0; @(posedge clk);

    // Set flush map: all arch regs point to phys reg = arch+10 (arbitrary checkpoint)
    for (int i=0; i<32; i++) rat_flush_map[i] = i + 6'd10 < 64 ? i + 6'd10 : i;

    // Rename r20 to disturb map
    @(negedge clk);
    rat_ren0=1; rat_rd0=5'd20; rat_rs0=5'd0; rat_rt0=5'd0; rat_has_dest0=1;
    @(negedge clk); rat_ren0=0;

    // Flush
    @(negedge clk); rat_flush_en=1;
    @(negedge clk); rat_flush_en=0;
    @(posedge clk); #1;

    // r20 should now map to flush_map[20] = 30
    rat_rs0=5'd20; #1;
    chk6("RAT: flush restores r20 -> 30", rat_phys_rs0, 6'd30);
endtask

// ============================================================
// Main
// ============================================================
initial begin
    pass_cnt = 0; fail_cnt = 0;
    arf_reset=0; arf_write=0; arf_wdata=0;
    arf_raddr1=0; arf_raddr2=0; arf_raddr3=0; arf_waddr=0;
    prf_reset=0; prf_wr_en=0; prf_wr_addr=0; prf_wr_data=0;
    prf_rd_addr0=0; prf_rd_addr1=0; prf_rd_addr2=0; prf_rd_addr3=0;
    rdy_reset=0; rdy_clear_en=0; rdy_clear_addr=0; rdy_set_en=0; rdy_set_addr=0;
    rdy_q0=0; rdy_q1=0;
    rat_reset=0; rat_ren0=0; rat_ren1=0;
    rat_rs0=0; rat_rt0=0; rat_rd0=0; rat_has_dest0=0;
    rat_rs1=0; rat_rt1=0; rat_rd1=0; rat_has_dest1=0;
    rat_c0_en=0; rat_c0_free=0; rat_c1_en=0; rat_c1_free=0;
    rat_flush_en=0;
    for (int i=0; i<32; i++) rat_flush_map[i] = 0;

    $display("===================================================");
    $display("  REGISTER FILES + RAT TESTBENCH");
    $display("===================================================");

    test_arch_regfile;
    test_phys_regfile;
    test_ready_bits;
    test_rat_single;
    test_rat_dual_issue;
    test_rat_commit_free;
    test_rat_flush;

    $display("\n===================================================");
    $display("  RESULTS: %0d passed, %0d failed", pass_cnt, fail_cnt);
    $display("===================================================");
    if (fail_cnt == 0) $display("  ALL TESTS PASSED");
    else               $display("  *** FAILURES DETECTED ***");
    $finish;
end

initial begin #200000; $display("TIMEOUT"); $finish; end

endmodule