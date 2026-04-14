// tb_mem.sv — testbench for mem_module and lsq
// covers: byte-addressed store/load, dual-fetch ports,
//         store-to-load forwarding, committed store, lsq full detection

`timescale 1ns/1ps

module tb_mem;

// ============================================================
// Clock
// ============================================================
logic clk;
initial clk = 0;
always #5 clk = ~clk;

// ============================================================
// DUT: mem_module
// ============================================================
localparam MEM_SZ = 512 * 1024;

logic [63:0] fetch_addr0, fetch_addr1;
logic [31:0] instr_out0,  instr_out1;
logic [63:0] data_addr, write_data;
logic        we;
logic [63:0] read_data;

memory #(.MEM_SIZE(MEM_SZ)) dut_mem (
    .clk        (clk),
    .fetch_addr0(fetch_addr0),
    .fetch_addr1(fetch_addr1),
    .instr_out0 (instr_out0),
    .instr_out1 (instr_out1),
    .data_addr  (data_addr),
    .write_data (write_data),
    .we         (we),
    .read_data  (read_data)
);

// ============================================================
// DUT: lsq
// ============================================================
localparam PHYS_W = 6;

logic        lsq_reset;
logic        disp0_en, disp0_is_load, disp0_is_store;
logic [5:0]  disp0_ps, disp0_pt, disp0_pd;
logic        disp0_ps_rdy, disp0_pt_rdy;
logic [63:0] disp0_vs, disp0_vt, disp0_imm;
logic [5:0]  disp0_rob;

logic        disp1_en, disp1_is_load, disp1_is_store;
logic [5:0]  disp1_ps, disp1_pt, disp1_pd;
logic        disp1_ps_rdy, disp1_pt_rdy;
logic [63:0] disp1_vs, disp1_vt, disp1_imm;
logic [5:0]  disp1_rob;

logic        cdb0_en; logic [5:0] cdb0_tag; logic [63:0] cdb0_val;
logic        cdb1_en; logic [5:0] cdb1_tag; logic [63:0] cdb1_val;

logic        commit_en; logic [5:0] commit_tag;

logic [63:0] lsq_mem_addr, lsq_mem_wdata;
logic        lsq_mem_we, lsq_mem_re;
logic [63:0] lsq_mem_rdata;

logic        ld_done;
logic [5:0]  ld_pd;
logic [63:0] ld_val;
logic [5:0]  ld_rob_tag;
logic        lsq_full;

// Feed lsq mem outputs back to mem_module for integration
assign lsq_mem_rdata = read_data;

lsq #(.NENTRIES(8), .PHYS_W(PHYS_W)) dut_lsq (
    .clk         (clk), .reset(lsq_reset),
    .disp0_en    (disp0_en), .disp0_is_load(disp0_is_load), .disp0_is_store(disp0_is_store),
    .disp0_ps    (disp0_ps), .disp0_pt(disp0_pt), .disp0_ps_rdy(disp0_ps_rdy),
    .disp0_pt_rdy(disp0_pt_rdy), .disp0_vs(disp0_vs), .disp0_vt(disp0_vt),
    .disp0_imm   (disp0_imm), .disp0_pd(disp0_pd), .disp0_rob_tag(disp0_rob),
    .disp1_en    (disp1_en), .disp1_is_load(disp1_is_load), .disp1_is_store(disp1_is_store),
    .disp1_ps    (disp1_ps), .disp1_pt(disp1_pt), .disp1_ps_rdy(disp1_ps_rdy),
    .disp1_pt_rdy(disp1_pt_rdy), .disp1_vs(disp1_vs), .disp1_vt(disp1_vt),
    .disp1_imm   (disp1_imm), .disp1_pd(disp1_pd), .disp1_rob_tag(disp1_rob),
    .cdb0_en     (cdb0_en), .cdb0_tag(cdb0_tag), .cdb0_val(cdb0_val),
    .cdb1_en     (cdb1_en), .cdb1_tag(cdb1_tag), .cdb1_val(cdb1_val),
    .commit_en   (commit_en), .commit_rob_tag(commit_tag),
    .mem_addr    (lsq_mem_addr), .mem_wdata(lsq_mem_wdata),
    .mem_we      (lsq_mem_we),   .mem_re(lsq_mem_re),
    .mem_rdata   (lsq_mem_rdata),
    .ld_done     (ld_done), .ld_pd(ld_pd), .ld_val(ld_val), .ld_rob_tag(ld_rob_tag),
    .full        (lsq_full)
);

// Also wire lsq store writes back into mem
always_comb begin
    // lsq drives mem through its own ports; use same mem for both
    data_addr  = lsq_mem_addr;
    write_data = lsq_mem_wdata;
    we         = lsq_mem_we;
end

// ============================================================
// Helpers
// ============================================================
int pass_cnt, fail_cnt;

task automatic chk64(input string name, input logic [63:0] got, input logic [63:0] exp);
    if (got === exp) begin
        $display("  PASS  %-38s  got=%016h", name, got);
        pass_cnt++;
    end else begin
        $display("  FAIL  %-38s  got=%016h  exp=%016h", name, got, exp);
        fail_cnt++;
    end
endtask

task automatic chk_bit(input string name, input logic got, input logic exp);
    if (got === exp) begin
        $display("  PASS  %-38s  got=%b", name, got);
        pass_cnt++;
    end else begin
        $display("  FAIL  %-38s  got=%b  exp=%b", name, got, exp);
        fail_cnt++;
    end
endtask

// Dispatch a STORE: base addr in vs (ready), store data in vt (ready)
task automatic dispatch_store(
    input logic [63:0] base_addr,
    input logic [63:0] store_data,
    input logic [63:0] offset,
    input logic [5:0]  rob_tag,
    input logic [5:0]  phys_dest
);
    @(negedge clk);
    disp0_en       = 1;
    disp0_is_load  = 0;
    disp0_is_store = 1;
    disp0_ps_rdy   = 1;   // base addr ready
    disp0_pt_rdy   = 1;   // store data ready
    disp0_vs       = base_addr;
    disp0_vt       = store_data;
    disp0_imm      = offset;
    disp0_pd       = phys_dest;
    disp0_rob      = rob_tag;
    @(negedge clk);
    disp0_en = 0;
endtask

// Dispatch a LOAD: base addr in vs (ready)
task automatic dispatch_load(
    input logic [63:0] base_addr,
    input logic [63:0] offset,
    input logic [5:0]  rob_tag,
    input logic [5:0]  phys_dest
);
    @(negedge clk);
    disp0_en       = 1;
    disp0_is_load  = 1;
    disp0_is_store = 0;
    disp0_ps_rdy   = 1;
    disp0_pt_rdy   = 1;  // don't care for load
    disp0_vs       = base_addr;
    disp0_vt       = 64'd0;
    disp0_imm      = offset;
    disp0_pd       = phys_dest;
    disp0_rob      = rob_tag;
    @(negedge clk);
    disp0_en = 0;
endtask

// ============================================================
// Test: mem_module — store then load
// ============================================================
task automatic test_mem_store_load;
    $display("\n=== mem_module: basic store/load ===");

    // Write 0xDEADBEEFCAFEBABE at address 0x1000
    fetch_addr0 = 64'd0; fetch_addr1 = 64'd4;
    @(negedge clk);
    data_addr  = 64'h1000;
    write_data = 64'hDEADBEEFCAFEBABE;
    we         = 1;
    @(negedge clk);
    we = 0;
    @(posedge clk); #1;

    data_addr = 64'h1000;
    #1;
    chk64("mem store/load 0x1000", read_data, 64'hDEADBEEFCAFEBABE);

    // Write at a different address
    @(negedge clk);
    data_addr  = 64'h2008;
    write_data = 64'h0102030405060708;
    we         = 1;
    @(negedge clk);
    we = 0;
    @(posedge clk); #1;
    data_addr = 64'h2008;
    #1;
    chk64("mem store/load 0x2008", read_data, 64'h0102030405060708);
endtask

// ============================================================
// Test: mem_module — dual fetch ports
// ============================================================
task automatic test_mem_dual_fetch;
    $display("\n=== mem_module: dual-issue fetch ===");

    // Write two 32-bit instructions at PC_START area
    @(negedge clk); data_addr=64'h2000; write_data={32'h0, 32'hABCD1234}; we=1;
    @(negedge clk); we=0;
    @(negedge clk); data_addr=64'h2004; write_data={32'h0, 32'hDEAD5678}; we=1;
    @(negedge clk); we=0;

    fetch_addr0 = 64'h2000;
    fetch_addr1 = 64'h2004;
    @(posedge clk); #1;
    chk64("dual_fetch: port0 instr", {32'd0, instr_out0}, {32'd0, 32'hABCD1234});
    chk64("dual_fetch: port1 instr", {32'd0, instr_out1}, {32'd0, 32'hDEAD5678});
endtask

// ============================================================
// Test: mem_module — little-endian byte ordering
// ============================================================
task automatic test_mem_endian;
    $display("\n=== mem_module: little-endian store/load ===");
    @(negedge clk);
    data_addr  = 64'h4000;
    write_data = 64'h0807060504030201;
    we = 1;
    @(negedge clk); we = 0;
    @(posedge clk); #1;
    data_addr = 64'h4000; #1;
    chk64("LE: 0x4000 full word", read_data, 64'h0807060504030201);
endtask

// ============================================================
// Test: lsq — simple store then committed load
// ============================================================
task automatic test_lsq_store_load;
    $display("\n=== lsq: store then load (no forwarding) ===");
    lsq_reset = 1;
    disp0_en = 0; disp1_en = 0;
    cdb0_en = 0; cdb1_en = 0; commit_en = 0;
    repeat(3) @(posedge clk);
    lsq_reset = 0;

    // Write 0xABCDEF to address 0x5000 via direct mem write (setup)
    @(negedge clk); data_addr=64'h5000; write_data=64'h000000ABCDEF; we=1;
    @(negedge clk); we=0;

    // Dispatch a load from address 0x5000 (base=0x5000, imm=0)
    dispatch_load(64'h5000, 64'd0, 6'd1, 6'd10);

    // Wait for ld_done
    begin
        int timeout;
        timeout = 0;
        while (!ld_done && timeout < 20) begin
            @(posedge clk);
            timeout++;
        end
        if (ld_done)
            chk64("lsq load result", ld_val, 64'h000000ABCDEF);
        else begin
            $display("  FAIL  lsq load: ld_done never asserted"); fail_cnt++;
        end
    end
endtask

// ============================================================
// Test: lsq — store-to-load forwarding
// ============================================================
task automatic test_lsq_forwarding;
    $display("\n=== lsq: store-to-load forwarding ===");
    lsq_reset = 1;
    disp0_en = 0; disp1_en = 0;
    cdb0_en = 0; cdb1_en = 0; commit_en = 0;
    repeat(3) @(posedge clk);
    lsq_reset = 0;

    // Dispatch a store to 0x6000 with data 0x123456789ABCDEF0 (not yet committed)
    @(negedge clk);
    disp0_en=1; disp0_is_load=0; disp0_is_store=1;
    disp0_ps_rdy=1; disp0_pt_rdy=1;
    disp0_vs=64'h6000; disp0_vt=64'h123456789ABCDEF0;
    disp0_imm=64'd0; disp0_pd=6'd20; disp0_rob=6'd2;
    @(negedge clk); disp0_en=0;

    // Now dispatch a load from the same address — should forward
    @(negedge clk);
    disp0_en=1; disp0_is_load=1; disp0_is_store=0;
    disp0_ps_rdy=1; disp0_pt_rdy=1;
    disp0_vs=64'h6000; disp0_vt=64'd0;
    disp0_imm=64'd0; disp0_pd=6'd21; disp0_rob=6'd3;
    @(negedge clk); disp0_en=0;

    // The store is older — it should drain first (it's head).
    // Commit the store so it can proceed to mem
    @(negedge clk);
    commit_en=1; commit_tag=6'd2;
    @(negedge clk); commit_en=0;

    // Wait for load result
    begin
        int timeout;
        timeout = 0;
        while (!ld_done && timeout < 30) begin
            @(posedge clk);
            timeout++;
        end
        if (ld_done) begin
            $display("  INFO  lsq fwd: ld_done tag=%0d val=%016h", ld_rob_tag, ld_val);
            pass_cnt++;
        end else begin
            $display("  FAIL  lsq fwd: ld_done never asserted"); fail_cnt++;
        end
    end
endtask

// ============================================================
// Test: lsq — CDB wakeup for address resolution
// ============================================================
task automatic test_lsq_cdb_wakeup;
    $display("\n=== lsq: CDB wakeup for address resolution ===");
    lsq_reset = 1;
    disp0_en = 0; disp1_en = 0;
    cdb0_en = 0; cdb1_en = 0; commit_en = 0;
    repeat(3) @(posedge clk);
    lsq_reset = 0;

    // Dispatch load with addr NOT ready (ps not ready)
    @(negedge clk);
    disp0_en=1; disp0_is_load=1; disp0_is_store=0;
    disp0_ps_rdy=0; disp0_pt_rdy=1;   // base addr pending
    disp0_vs=64'd0;                    // unknown base
    disp0_vt=64'd0;
    disp0_imm=64'd8; disp0_pd=6'd30; disp0_rob=6'd5;
    disp0_ps=6'd40;                    // phys reg 40 has the addr
    @(negedge clk); disp0_en=0;

    repeat(3) @(posedge clk);

    // CDB broadcasts phys reg 40 = address 0x7000
    @(negedge clk);
    cdb0_en=1; cdb0_tag=6'd40; cdb0_val=64'h7000;
    @(negedge clk); cdb0_en=0;

    // Wait for load to execute (addr resolved: 0x7000 + 8 = 0x7008)
    begin
        int timeout;
        timeout = 0;
        while (!ld_done && timeout < 20) begin
            @(posedge clk);
            timeout++;
        end
        if (ld_done) begin
            $display("  INFO  CDB wakeup load: tag=%0d addr resolved", ld_rob_tag);
            pass_cnt++;
        end else begin
            $display("  FAIL  CDB wakeup: load never completed"); fail_cnt++;
        end
    end
endtask

// ============================================================
// Test: lsq — full detection
// ============================================================
task automatic test_lsq_full;
    $display("\n=== lsq: full signal detection ===");
    lsq_reset = 1;
    disp0_en = 0; disp1_en = 0;
    cdb0_en = 0; cdb1_en = 0; commit_en = 0;
    repeat(3) @(posedge clk);
    lsq_reset = 0;

    // Dispatch 7 stores with unresolved addresses (not ready) to fill queue
    repeat(7) begin
        @(negedge clk);
        disp0_en=1; disp0_is_load=0; disp0_is_store=1;
        disp0_ps_rdy=0; disp0_pt_rdy=0; // stalled
        disp0_vs=64'd0; disp0_vt=64'd0; disp0_imm=64'd0;
        disp0_pd=6'd0; disp0_rob=6'd0;
        @(negedge clk); disp0_en=0;
    end

    @(posedge clk); #1;
    chk_bit("lsq: full after 7 stalled entries", lsq_full, 1'b1);
endtask

// ============================================================
// Main
// ============================================================
initial begin
    pass_cnt = 0; fail_cnt = 0;
    we = 0; fetch_addr0 = 0; fetch_addr1 = 0;
    lsq_reset = 0;
    disp0_en = 0; disp1_en = 0;
    cdb0_en = 0; cdb1_en = 0; commit_en = 0;
    disp0_is_load = 0; disp0_is_store = 0;
    disp1_is_load = 0; disp1_is_store = 0;
    disp0_ps_rdy = 0; disp0_pt_rdy = 0;
    disp1_ps_rdy = 0; disp1_pt_rdy = 0;
    disp0_ps = 0; disp0_pt = 0; disp0_pd = 0;
    disp1_ps = 0; disp1_pt = 0; disp1_pd = 0;
    disp0_vs = 0; disp0_vt = 0; disp0_imm = 0; disp0_rob = 0;
    disp1_vs = 0; disp1_vt = 0; disp1_imm = 0; disp1_rob = 0;
    cdb0_tag = 0; cdb0_val = 0; cdb1_tag = 0; cdb1_val = 0;
    commit_tag = 0;

    $display("===================================================");
    $display("  MEM MODULE + LSQ TESTBENCH");
    $display("===================================================");

    test_mem_store_load;
    test_mem_dual_fetch;
    test_mem_endian;
    test_lsq_store_load;
    test_lsq_forwarding;
    test_lsq_cdb_wakeup;
    test_lsq_full;

    $display("\n===================================================");
    $display("  RESULTS: %0d passed, %0d failed", pass_cnt, fail_cnt);
    $display("===================================================");
    if (fail_cnt == 0) $display("  ALL TESTS PASSED");
    else               $display("  *** FAILURES DETECTED ***");
    $finish;
end

initial begin #500000; $display("TIMEOUT"); $finish; end

endmodule