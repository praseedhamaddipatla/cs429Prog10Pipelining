// tinker_tb.sv — SIMPLE + SAFE testbench (fixed)

`timescale 1ns/1ps
`define PC_START 64'h2000

module tb_tinker;

    reg clk, reset;
    wire hlt;

    tinker_core dut (
        .clk   (clk),
        .reset (reset),
        .hlt   (hlt)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    integer fail_cnt;

    // =========================================================
    // WRITE INSTRUCTION (adjust instance name if needed)
    // =========================================================
    task write_instr;
        input [63:0] addr;
        input [31:0] instr;
        begin
            dut.memory.bytes[addr+0] = instr[7:0];
            dut.memory.bytes[addr+1] = instr[15:8];
            dut.memory.bytes[addr+2] = instr[23:16];
            dut.memory.bytes[addr+3] = instr[31:24];
        end
    endtask

    // =========================================================
    // TEST 1: HALT
    // =========================================================
    task test_halt;
        integer i;
        begin
            $display("TEST 1: HALT");

            reset = 1; repeat(2) @(posedge clk);
            reset = 0;

            write_instr(`PC_START, {5'h0F, 27'd0});

            for (i = 0; i < 30; i = i + 1) begin
                @(posedge clk);
                if (hlt) begin
                    $display("  PASS");
                    disable test_halt;
                end
            end

            $display("  FAIL");
            fail_cnt++;
        end
    endtask

    // =========================================================
    // TEST 2: ADDI RUNS
    // =========================================================
    task test_addi_runs;
        integer i;
        begin
            $display("TEST 2: ADDI");

            reset = 1; repeat(2) @(posedge clk);
            reset = 0;

            write_instr(`PC_START+0, {5'h19, 5'd1, 5'd0, 5'd0, 12'd5});
            write_instr(`PC_START+4, {5'h19, 5'd1, 5'd0, 5'd0, 12'd3});
            write_instr(`PC_START+8, {5'h0F, 27'd0});

            for (i = 0; i < 50; i = i + 1) begin
                @(posedge clk);
                if (hlt) begin
                    $display("  PASS");
                    disable test_addi_runs;
                end
            end

            $display("  FAIL");
            fail_cnt++;
        end
    endtask

    // =========================================================
    // TEST 3: LOAD/STORE RUNS
    // =========================================================
    task test_load_store_runs;
        integer i;
        begin
            $display("TEST 3: LOAD/STORE");

            reset = 1; repeat(2) @(posedge clk);
            reset = 0;

            write_instr(`PC_START+0,  {5'h19, 5'd1, 5'd0, 5'd0, 12'd100});
            write_instr(`PC_START+4,  {5'h19, 5'd2, 5'd0, 5'd0, 12'd42});
            write_instr(`PC_START+8,  {5'h13, 5'd1, 5'd2, 5'd0, 12'd0});
            write_instr(`PC_START+12, {5'h10, 5'd3, 5'd1, 5'd0, 12'd0});
            write_instr(`PC_START+16, {5'h0F, 27'd0});

            for (i = 0; i < 80; i = i + 1) begin
                @(posedge clk);
                if (hlt) begin
                    $display("  PASS");
                    disable test_load_store_runs;
                end
            end

            $display("  FAIL");
            fail_cnt++;
        end
    endtask

    // =========================================================
    // MAIN
    // =========================================================
    initial begin
        $dumpfile("tinker_tb.vcd");
        $dumpvars(0, tb_tinker);

        fail_cnt = 0;

        test_halt;
        test_addi_runs;
        test_load_store_runs;

        if (fail_cnt == 0)
            $display("\nALL TESTS PASSED");
        else
            $display("\nFAILURES: %0d", fail_cnt);

        $finish;
    end

    initial begin
        #100000;
        $display("TIMEOUT");
        $finish;
    end

endmodule