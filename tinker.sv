// tinker_core — 5-stage in-order pipeline
// Architecture mirrors friend's design: IF → ID → EX → MEM → WB
// Branch prediction at decode (backward-taken heuristic),
// mispredict resolved at execute (1-cycle penalty).
// Call/return: address computed in EX, resolved in MEM for return.

`define MEM_SIZE (512 * 1024)
`define PC_START 64'h2000

`include "hdl/alu.sv"
`include "hdl/fpu.sv"
`include "hdl/reg_file.sv"
`include "hdl/decoder.sv"
`include "hdl/fetch.sv"
`include "hdl/memory.sv"

module tinker_core (
    input  logic clk,
    input  logic reset,
    output logic hlt
);

localparam [63:0] RESET_PC  = `PC_START;
localparam [4:0]  STACK_REG = 5'd31;
localparam        LSQ_DEPTH = 8;

// ============================================================
// ARCHITECTURAL REGISTER FILE
// ============================================================
reg [63:0] regs [0:31];

// ============================================================
// IF/ID PIPELINE LATCH
// ============================================================
reg        if_id_valid;
reg [63:0] if_id_pc;
reg [31:0] if_id_instr;
reg [63:0] if_id_seq_next;

// ============================================================
// ID/EX PIPELINE LATCH
// ============================================================
reg        id_ex_valid;
reg [63:0] id_ex_pc;
reg [4:0]  id_ex_rd;
reg [4:0]  id_ex_alu_op;
reg [63:0] id_ex_src1;
reg [63:0] id_ex_src2;
reg [63:0] id_ex_src3;
reg [63:0] id_ex_imm;
reg        id_ex_has_lit;
reg        id_ex_wr;
reg        id_ex_rd_mem;
reg        id_ex_wr_mem;
reg        id_ex_wr_from_mem;
reg        id_ex_is_branch;
reg        id_ex_is_call;
reg        id_ex_is_ret;
reg        id_ex_is_branch_reg;
reg        id_ex_is_branch_lit;
reg        id_ex_is_branch_nz;
reg        id_ex_is_branch_gt;
reg        id_ex_rd_is_br_tgt;
reg        id_ex_is_fp;
reg        id_ex_is_halt;
reg        id_ex_has_lsq;
reg [2:0]  id_ex_lsq_idx;
reg        id_ex_pred_taken;
reg [63:0] id_ex_pred_tgt;
reg [63:0] id_ex_seq_next;

// ============================================================
// EX/MEM PIPELINE LATCH
// ============================================================
reg        ex_mem_valid;
reg [4:0]  ex_mem_rd;
reg [63:0] ex_mem_addr;
reg [63:0] ex_mem_alu;
reg [63:0] ex_mem_store;
reg        ex_mem_wr;
reg        ex_mem_rd_mem;
reg        ex_mem_wr_mem;
reg        ex_mem_wr_from_mem;
reg        ex_mem_is_ret;
reg        ex_mem_is_halt;
reg        ex_mem_has_lsq;
reg [2:0]  ex_mem_lsq_idx;

// ============================================================
// MEM/WB PIPELINE LATCH
// ============================================================
reg        mem_wb_valid;
reg [4:0]  mem_wb_rd;
reg [63:0] mem_wb_result;
reg        mem_wb_wr;
reg        mem_wb_is_halt;
reg        mem_wb_has_lsq;
reg [2:0]  mem_wb_lsq_idx;

// ============================================================
// ARCHITECTURAL PC
// ============================================================
reg [63:0] pc;
reg        ctrl_pending;

// ============================================================
// LOAD/STORE QUEUE
// ============================================================
reg        lsq_valid   [0:LSQ_DEPTH-1];
reg        lsq_is_ld   [0:LSQ_DEPTH-1];
reg        lsq_is_st   [0:LSQ_DEPTH-1];
reg        lsq_done    [0:LSQ_DEPTH-1];
reg        lsq_a_rdy   [0:LSQ_DEPTH-1];
reg [63:0] lsq_addr    [0:LSQ_DEPTH-1];
reg        lsq_d_rdy   [0:LSQ_DEPTH-1];
reg [63:0] lsq_data    [0:LSQ_DEPTH-1];
reg [63:0] lsq_ld_res  [0:LSQ_DEPTH-1];
reg [2:0]  lsq_head, lsq_tail;
reg [3:0]  lsq_cnt;

reg        cmt_st_pending;
reg [2:0]  cmt_st_idx;
reg [63:0] cmt_st_addr;
reg [63:0] cmt_st_data;

// ============================================================
// DECODER WIRES
// ============================================================
wire [4:0]  dec_op;
wire [4:0]  dec_rd, dec_rs, dec_rt;
wire [11:0] dec_lit;
wire        dec_wr_reg, dec_rd_mem, dec_wr_mem;
wire        dec_has_rs, dec_has_rt, dec_has_lit;
wire [4:0]  dec_alu_op;
wire        dec_is_branch, dec_is_call, dec_is_ret;
wire        dec_rd_is_val, dec_rd_is_adr;
wire        dec_wr_from_mem;
wire        dec_rd_is_br_tgt;
wire        dec_br_reg, dec_br_lit, dec_br_nz, dec_br_gt;

decoder decode (
    .instruct       (if_id_instr),
    .op             (dec_op),
    .rd             (dec_rd),
    .rs             (dec_rs),
    .rt             (dec_rt),
    .lit            (dec_lit),
    .write_reg      (dec_wr_reg),
    .read_mem       (dec_rd_mem),
    .write_mem      (dec_wr_mem),
    .has_rs         (dec_has_rs),
    .has_rt         (dec_has_rt),
    .has_lit        (dec_has_lit),
    .alu_op         (dec_alu_op),
    .branch_instruct(dec_is_branch),
    .call_instruct  (dec_is_call),
    .return_instruct(dec_is_ret),
    .rd_is_val      (dec_rd_is_val),
    .rd_is_adr      (dec_rd_is_adr),
    .write_from_mem (dec_wr_from_mem),
    .rd_is_branch_target(dec_rd_is_br_tgt),
    .branch_reg     (dec_br_reg),
    .branch_lit     (dec_br_lit),
    .branch_nz      (dec_br_nz),
    .branch_gt      (dec_br_gt)
);

// ============================================================
// REG_FILE (autograder compatibility)
// ============================================================
reg_file reg_file (
    .clk    (clk),
    .reset  (reset),
    .write  (mem_wb_valid && mem_wb_wr),
    .write_enable2(1'b0),
    .waddr  (mem_wb_rd),
    .waddr2 (5'd0),
    .data   (mem_wb_result),
    .data2  (64'd0),
    .raddr1 (5'd0), .raddr2(5'd0), .raddr3(5'd0), .raddr4(5'd0),
    .r1(), .r2(), .r3(), .r4()
);

// ============================================================
// ALU
// ============================================================
wire [63:0] alu_b_in = id_ex_has_lit ? id_ex_imm : id_ex_src2;
wire [63:0] alu_result;

alu alu (
    .a     (id_ex_src1),
    .b     (alu_b_in),
    .alu_op(id_ex_alu_op),
    .c     (alu_result)
);

// ============================================================
// FPU (multi-cycle)
// ============================================================
wire        fpu_vout;
wire [63:0] fpu_result;
wire        id_ex_is_fp_instr = id_ex_valid &&
                (id_ex_alu_op >= 5'h14) && (id_ex_alu_op <= 5'h17);

fpu fpu (
    .clk      (clk),
    .reset    (reset),
    .valid_in (id_ex_is_fp_instr),
    .a        (id_ex_src1),
    .b        (alu_b_in),
    .alu_op   (id_ex_alu_op),
    .valid_out(fpu_vout),
    .c        (fpu_result)
);

reg fp_s0_v, fp_s1_v, fp_s2_v, fp_s3_v, fp_s4_v;
reg [4:0] fp_s0_rd, fp_s1_rd, fp_s2_rd, fp_s3_rd, fp_s4_rd;
reg fp_s0_wr, fp_s1_wr, fp_s2_wr, fp_s3_wr, fp_s4_wr;

wire fp_wb_valid = fpu_vout && fp_s4_wr;
wire [4:0]  fp_wb_rd  = fp_s4_rd;
wire [63:0] fp_wb_val = fpu_result;

// ============================================================
// MEMORY
// ============================================================
wire [31:0] mem_instr;
wire [63:0] mem_rd_data;
wire [63:0] mem_fetch_pc;
wire        mem_re, mem_we;
wire [63:0] mem_daddr, mem_wdata;

memory memory (
    .clk        (clk),
    .pc         (mem_fetch_pc),
    .instruction(mem_instr),
    .mem_read   (mem_re),
    .mem_write  (mem_we),
    .data_addr  (mem_daddr),
    .write_data (mem_wdata),
    .read_data  (mem_rd_data)
);

// ============================================================
// EX-STAGE COMBINATIONAL
// ============================================================
reg [63:0] ex_addr;
reg [63:0] ex_store_data;
reg        ex_redir_valid;
reg [63:0] ex_redir_tgt;

always @(*) begin
    ex_addr       = alu_result;
    ex_store_data = id_ex_src2;
    ex_redir_valid = 1'b0;
    ex_redir_tgt   = 64'd0;

    if (id_ex_is_call) begin
        ex_addr        = id_ex_src2 - 64'd8;   // r31 - 8
        ex_store_data  = id_ex_pc + 64'd4;      // return address stored
        ex_redir_valid = 1'b1;
        ex_redir_tgt   = id_ex_src1;            // call target
    end else if (id_ex_is_ret) begin
        ex_addr = id_ex_src1 - 64'd8;           // load return addr
    end else if (id_ex_is_branch) begin
        if (id_ex_is_branch_reg) begin
            ex_redir_valid = 1'b1;
            ex_redir_tgt   = id_ex_pc + id_ex_src1;
        end else if (id_ex_is_branch_lit) begin
            ex_redir_valid = 1'b1;
            ex_redir_tgt   = id_ex_pc + id_ex_imm;
        end else if (id_ex_is_branch_nz) begin
            if (id_ex_src1 != 64'd0) begin
                ex_redir_valid = 1'b1;
                ex_redir_tgt   = id_ex_src2;
            end
        end else if (id_ex_is_branch_gt) begin
            if ($signed(id_ex_src2) > $signed(id_ex_src3)) begin
                ex_redir_valid = 1'b1;
                ex_redir_tgt   = id_ex_src1;
            end
        end else if (id_ex_rd_is_br_tgt) begin
            ex_redir_valid = 1'b1;
            ex_redir_tgt   = id_ex_src1;
        end
    end
end

// Mispredict: prediction wrong direction or wrong target
wire ex_mispredict = id_ex_valid && id_ex_pred_taken &&
    ((id_ex_pred_taken != ex_redir_valid) ||
     (ex_redir_valid && (id_ex_pred_tgt != ex_redir_tgt)));

// Return resolves in MEM
wire mem_ret_resolve = ex_mem_valid && ex_mem_is_ret;
wire [63:0] mem_ret_tgt = mem_rd_data;

// Memory port assignments
assign mem_fetch_pc = pc;
assign mem_re   = ex_mem_valid && ex_mem_rd_mem;
assign mem_we   = cmt_st_pending &&
                  !(ex_mem_valid && (ex_mem_rd_mem ||
                    (ex_mem_wr_mem && !ex_mem_has_lsq)));
assign mem_daddr = cmt_st_pending ? cmt_st_addr : ex_mem_addr;
assign mem_wdata = cmt_st_pending ? cmt_st_data  : ex_mem_store;

// ============================================================
// DECODE COMBINATIONAL
// ============================================================
function automatic [63:0] expand_imm;
    input [11:0] lit;
    input [4:0]  op;
    begin
        case (op)
            5'h05, 5'h07, 5'h12, 5'h19, 5'h1B:
                expand_imm = {52'd0, lit};
            default:
                expand_imm = {{52{lit[11]}}, lit};
        endcase
    end
endfunction

reg [63:0] dec_imm_val;
reg        dec_is_halt;
reg [4:0]  dec_ra1, dec_ra2, dec_ra3;
reg        dec_s1_used, dec_s2_used, dec_s3_used;
reg [63:0] dec_s1, dec_s2, dec_s3;
reg        dec_stall;
wire       dec_needs_lsq = if_id_valid && (dec_rd_mem || dec_wr_mem) &&
                           !dec_is_call && !dec_is_ret;
wire       lsq_full = (lsq_cnt >= LSQ_DEPTH - 1);

reg        pred_ctrl_xfer;
reg        pred_taken;
reg [63:0] pred_seq_next;
reg [63:0] pred_tgt;
reg        pred_fetch_redir;
reg [63:0] pred_fetch_tgt;

always @(*) begin
    dec_imm_val = expand_imm(dec_lit, dec_alu_op);
    dec_is_halt = if_id_valid && (dec_op == 5'h0F) && (dec_lit == 12'h000);

    dec_ra1 = 5'd0; dec_ra2 = 5'd0; dec_ra3 = 5'd0;
    dec_s1_used = 1'b0; dec_s2_used = 1'b0; dec_s3_used = 1'b0;

    if (dec_br_nz) begin
        dec_ra1 = dec_rs; dec_ra2 = dec_rd;
        dec_s1_used = 1; dec_s2_used = 1;
    end else if (dec_br_gt) begin
        dec_ra1 = dec_rd; dec_ra2 = dec_rs; dec_ra3 = dec_rt;
        dec_s1_used = 1; dec_s2_used = 1; dec_s3_used = 1;
    end else if (dec_is_call) begin
        dec_ra1 = dec_rd; dec_ra2 = STACK_REG;
        dec_s1_used = 1; dec_s2_used = 1;
    end else if (dec_is_ret) begin
        dec_ra1 = STACK_REG;
        dec_s1_used = 1;
    end else if (dec_rd_is_adr) begin
        dec_ra1 = dec_rd; dec_ra2 = dec_rs;
        dec_s1_used = 1; dec_s2_used = 1;
    end else if (dec_rd_is_val || dec_rd_is_br_tgt) begin
        dec_ra1 = dec_rd; dec_ra2 = dec_rt;
        dec_s1_used = 1; dec_s2_used = dec_has_rt;
    end else begin
        dec_ra1 = dec_rs; dec_ra2 = dec_rt;
        dec_s1_used = dec_has_rs; dec_s2_used = dec_has_rt;
    end

    dec_s1 = regs[dec_ra1];
    dec_s2 = regs[dec_ra2];
    dec_s3 = regs[dec_ra3];

    // Forwarding from WB
    if (mem_wb_valid && mem_wb_wr) begin
        if (dec_s1_used && dec_ra1 == mem_wb_rd) dec_s1 = mem_wb_result;
        if (dec_s2_used && dec_ra2 == mem_wb_rd) dec_s2 = mem_wb_result;
        if (dec_s3_used && dec_ra3 == mem_wb_rd) dec_s3 = mem_wb_result;
    end

    // Forwarding from EX/MEM (non-load)
    if (ex_mem_valid && ex_mem_wr && !ex_mem_wr_from_mem) begin
        if (dec_s1_used && dec_ra1 == ex_mem_rd) dec_s1 = ex_mem_alu;
        if (dec_s2_used && dec_ra2 == ex_mem_rd) dec_s2 = ex_mem_alu;
        if (dec_s3_used && dec_ra3 == ex_mem_rd) dec_s3 = ex_mem_alu;
    end

    // Forwarding from FP WB
    if (fp_wb_valid) begin
        if (dec_s1_used && dec_ra1 == fp_wb_rd) dec_s1 = fp_wb_val;
        if (dec_s2_used && dec_ra2 == fp_wb_rd) dec_s2 = fp_wb_val;
        if (dec_s3_used && dec_ra3 == fp_wb_rd) dec_s3 = fp_wb_val;
    end

    // Stall detection
    dec_stall = 1'b0;
    if (if_id_valid) begin
        if (dec_needs_lsq && lsq_full) dec_stall = 1'b1;

        if (dec_s1_used &&
            ((id_ex_valid && id_ex_wr && dec_ra1 == id_ex_rd) ||
             (ex_mem_valid && ex_mem_wr && ex_mem_wr_from_mem && dec_ra1 == ex_mem_rd)))
            dec_stall = 1'b1;
        if (dec_s2_used &&
            ((id_ex_valid && id_ex_wr && dec_ra2 == id_ex_rd) ||
             (ex_mem_valid && ex_mem_wr && ex_mem_wr_from_mem && dec_ra2 == ex_mem_rd)))
            dec_stall = 1'b1;
        if (dec_s3_used &&
            ((id_ex_valid && id_ex_wr && dec_ra3 == id_ex_rd) ||
             (ex_mem_valid && ex_mem_wr && ex_mem_wr_from_mem && dec_ra3 == ex_mem_rd)))
            dec_stall = 1'b1;

        // FP hazard
        if (dec_s1_used &&
            ((fp_s0_v && dec_ra1 == fp_s0_rd) || (fp_s1_v && dec_ra1 == fp_s1_rd) ||
             (fp_s2_v && dec_ra1 == fp_s2_rd) || (fp_s3_v && dec_ra1 == fp_s3_rd)))
            dec_stall = 1'b1;
        if (dec_s2_used &&
            ((fp_s0_v && dec_ra2 == fp_s0_rd) || (fp_s1_v && dec_ra2 == fp_s1_rd) ||
             (fp_s2_v && dec_ra2 == fp_s2_rd) || (fp_s3_v && dec_ra2 == fp_s3_rd)))
            dec_stall = 1'b1;
        if (dec_s3_used &&
            ((fp_s0_v && dec_ra3 == fp_s0_rd) || (fp_s1_v && dec_ra3 == fp_s1_rd) ||
             (fp_s2_v && dec_ra3 == fp_s2_rd) || (fp_s3_v && dec_ra3 == fp_s3_rd)))
            dec_stall = 1'b1;
    end

    // Branch prediction — mirrors friend's logic exactly
    pred_ctrl_xfer = 1'b0;
    pred_taken     = 1'b0;
    pred_seq_next  = if_id_pc + 64'd4;
    pred_tgt       = pred_seq_next;

    if (if_id_valid && !dec_is_ret) begin
        if (dec_is_call) begin
            pred_ctrl_xfer = 1'b1;
            pred_taken     = 1'b1;
            pred_tgt       = dec_s1;
        end else if (dec_is_branch) begin
            pred_ctrl_xfer = 1'b1;
            if (dec_br_reg) begin
                pred_taken = 1'b1;
                pred_tgt   = if_id_pc + dec_s1;
            end else if (dec_br_lit) begin
                pred_taken = 1'b1;
                pred_tgt   = if_id_pc + dec_imm_val;
            end else if (dec_br_nz) begin
                if (dec_s2 < if_id_pc) begin
                    pred_taken = 1'b1;
                    pred_tgt   = dec_s2;
                end else begin
                    pred_taken = 1'b0;
                    pred_tgt   = pred_seq_next;
                end
            end else if (dec_br_gt) begin
                if (dec_s1 < if_id_pc) begin
                    pred_taken = 1'b1;
                    pred_tgt   = dec_s1;
                end else begin
                    pred_taken = 1'b0;
                    pred_tgt   = pred_seq_next;
                end
            end else if (dec_rd_is_br_tgt) begin
                pred_taken = 1'b1;
                pred_tgt   = dec_s1;
            end
        end
    end

    pred_fetch_redir = if_id_valid && !dec_stall &&
                       pred_ctrl_xfer && pred_taken;
    pred_fetch_tgt   = pred_tgt;
end

// LSQ commit/retire logic
reg lsq_do_commit_st, lsq_do_retire;
reg [2:0] lsq_retire_idx;

always @(*) begin
    lsq_do_commit_st = 1'b0;
    lsq_do_retire    = 1'b0;
    lsq_retire_idx   = 3'd0;

    if (cmt_st_pending && !(ex_mem_valid &&
        (ex_mem_rd_mem || (ex_mem_wr_mem && !ex_mem_has_lsq)))) begin
        lsq_do_commit_st = 1'b1;
        lsq_do_retire    = 1'b1;
        lsq_retire_idx   = cmt_st_idx;
    end

    if (mem_wb_valid && mem_wb_has_lsq &&
        lsq_valid[mem_wb_lsq_idx] && lsq_is_ld[mem_wb_lsq_idx]) begin
        lsq_do_retire  = 1'b1;
        lsq_retire_idx = mem_wb_lsq_idx;
    end
end

// ============================================================
// SEQUENTIAL LOGIC
// ============================================================
integer qi;

always @(posedge clk or posedge reset) begin
    if (reset) begin
        pc           <= RESET_PC;
        ctrl_pending <= 1'b0;

        if_id_valid  <= 1'b0; if_id_pc <= 64'd0;
        if_id_instr  <= 32'd0; if_id_seq_next <= 64'd0;

        id_ex_valid  <= 1'b0;
        ex_mem_valid <= 1'b0;
        mem_wb_valid <= 1'b0;

        fp_s0_v <= 0; fp_s1_v <= 0; fp_s2_v <= 0; fp_s3_v <= 0; fp_s4_v <= 0;
        fp_s0_wr<= 0; fp_s1_wr<= 0; fp_s2_wr<= 0; fp_s3_wr<= 0; fp_s4_wr<= 0;

        lsq_head <= 3'd0; lsq_tail <= 3'd0; lsq_cnt <= 4'd0;
        cmt_st_pending <= 1'b0;

        for (qi = 0; qi < 32; qi = qi + 1) regs[qi] <= 64'd0;
        regs[31] <= 64'd524288;

        for (qi = 0; qi < LSQ_DEPTH; qi = qi + 1) begin
            lsq_valid[qi] <= 1'b0; lsq_is_ld[qi] <= 1'b0;
            lsq_is_st[qi] <= 1'b0; lsq_done[qi]  <= 1'b0;
            lsq_a_rdy[qi] <= 1'b0; lsq_d_rdy[qi] <= 1'b0;
        end

    end else begin

        // -------- FP pipeline shift --------
        fp_s4_v  <= fp_s3_v;  fp_s4_rd <= fp_s3_rd; fp_s4_wr <= fp_s3_wr;
        fp_s3_v  <= fp_s2_v;  fp_s3_rd <= fp_s2_rd; fp_s3_wr <= fp_s2_wr;
        fp_s2_v  <= fp_s1_v;  fp_s2_rd <= fp_s1_rd; fp_s2_wr <= fp_s1_wr;
        fp_s1_v  <= fp_s0_v;  fp_s1_rd <= fp_s0_rd; fp_s1_wr <= fp_s0_wr;
        fp_s0_v  <= id_ex_is_fp_instr;
        fp_s0_rd <= id_ex_rd;
        fp_s0_wr <= id_ex_valid && id_ex_wr;

        // -------- LSQ cleanup --------
        if (lsq_do_commit_st) cmt_st_pending <= 1'b0;

        if (lsq_do_retire && lsq_valid[lsq_retire_idx]) begin
            lsq_valid[lsq_retire_idx] <= 1'b0;
            lsq_is_ld[lsq_retire_idx] <= 1'b0;
            lsq_is_st[lsq_retire_idx] <= 1'b0;
            lsq_done[lsq_retire_idx]  <= 1'b0;
            lsq_a_rdy[lsq_retire_idx] <= 1'b0;
            lsq_d_rdy[lsq_retire_idx] <= 1'b0;
        end

        if (!(mem_wb_valid && mem_wb_is_halt)) begin

            // -------- WB --------
            mem_wb_valid  <= ex_mem_valid;
            mem_wb_rd     <= ex_mem_rd;
            mem_wb_result <= ex_mem_wr_from_mem ? mem_rd_data : ex_mem_alu;
            mem_wb_wr     <= ex_mem_wr;
            mem_wb_is_halt<= ex_mem_is_halt;
            mem_wb_has_lsq<= ex_mem_has_lsq;
            mem_wb_lsq_idx<= ex_mem_lsq_idx;

            if (mem_wb_valid && mem_wb_wr)
                regs[mem_wb_rd] <= mem_wb_result;

            if (fp_wb_valid)
                regs[fp_wb_rd] <= fp_wb_val;

            if (ex_mem_valid && ex_mem_has_lsq && ex_mem_rd_mem)
                lsq_ld_res[ex_mem_lsq_idx] <= mem_rd_data;

            if (mem_wb_valid && mem_wb_has_lsq &&
                lsq_valid[mem_wb_lsq_idx] && lsq_is_st[mem_wb_lsq_idx]) begin
                cmt_st_pending <= 1'b1;
                cmt_st_idx     <= mem_wb_lsq_idx;
                cmt_st_addr    <= lsq_addr[mem_wb_lsq_idx];
                cmt_st_data    <= lsq_data[mem_wb_lsq_idx];
            end

            // -------- EX → MEM --------
            if (id_ex_is_fp_instr) begin
                ex_mem_valid <= 1'b0; ex_mem_rd <= 5'd0; ex_mem_addr <= 64'd0;
                ex_mem_alu   <= 64'd0; ex_mem_store <= 64'd0; ex_mem_wr <= 1'b0;
                ex_mem_rd_mem <= 1'b0; ex_mem_wr_mem <= 1'b0;
                ex_mem_wr_from_mem <= 1'b0; ex_mem_is_ret <= 1'b0;
                ex_mem_is_halt <= 1'b0; ex_mem_has_lsq <= 1'b0;
                ex_mem_lsq_idx <= 3'd0;
            end else begin
                ex_mem_valid <= id_ex_valid; ex_mem_rd <= id_ex_rd;
                ex_mem_addr  <= ex_addr; ex_mem_alu <= alu_result;
                ex_mem_store <= ex_store_data; ex_mem_wr <= id_ex_wr;
                ex_mem_rd_mem <= id_ex_rd_mem; ex_mem_wr_mem <= id_ex_wr_mem;
                ex_mem_wr_from_mem <= id_ex_wr_from_mem;
                ex_mem_is_ret <= id_ex_is_ret; ex_mem_is_halt <= id_ex_is_halt;
                ex_mem_has_lsq <= id_ex_has_lsq; ex_mem_lsq_idx <= id_ex_lsq_idx;
            end

            if (id_ex_valid && id_ex_has_lsq) begin
                lsq_a_rdy[id_ex_lsq_idx] <= 1'b1;
                lsq_addr[id_ex_lsq_idx]  <= ex_addr;
                if (id_ex_wr_mem) begin
                    lsq_d_rdy[id_ex_lsq_idx] <= 1'b1;
                    lsq_data[id_ex_lsq_idx]  <= ex_store_data;
                    lsq_done[id_ex_lsq_idx]  <= 1'b1;
                end
            end

            // -------- ID → EX --------
            if (dec_stall || !if_id_valid || ex_mispredict || mem_ret_resolve) begin
                id_ex_valid <= 1'b0; id_ex_pc <= 64'd0; id_ex_rd <= 5'd0;
                id_ex_alu_op <= 5'd0; id_ex_src1 <= 64'd0; id_ex_src2 <= 64'd0;
                id_ex_src3 <= 64'd0; id_ex_imm <= 64'd0; id_ex_has_lit <= 1'b0;
                id_ex_wr <= 1'b0; id_ex_rd_mem <= 1'b0; id_ex_wr_mem <= 1'b0;
                id_ex_wr_from_mem <= 1'b0; id_ex_is_branch <= 1'b0;
                id_ex_is_call <= 1'b0; id_ex_is_ret <= 1'b0;
                id_ex_is_branch_reg <= 1'b0; id_ex_is_branch_lit <= 1'b0;
                id_ex_is_branch_nz <= 1'b0; id_ex_is_branch_gt <= 1'b0;
                id_ex_rd_is_br_tgt <= 1'b0; id_ex_is_fp <= 1'b0;
                id_ex_is_halt <= 1'b0; id_ex_has_lsq <= 1'b0;
                id_ex_lsq_idx <= 3'd0; id_ex_pred_taken <= 1'b0;
                id_ex_pred_tgt <= 64'd0; id_ex_seq_next <= 64'd0;
            end else begin
                id_ex_valid <= if_id_valid; id_ex_pc <= if_id_pc;
                id_ex_rd <= dec_rd; id_ex_alu_op <= dec_alu_op;
                id_ex_src1 <= dec_s1; id_ex_src2 <= dec_s2; id_ex_src3 <= dec_s3;
                id_ex_imm <= dec_imm_val; id_ex_has_lit <= dec_has_lit;
                id_ex_wr <= dec_wr_reg; id_ex_rd_mem <= dec_rd_mem;
                id_ex_wr_mem <= dec_wr_mem; id_ex_wr_from_mem <= dec_wr_from_mem;
                id_ex_is_branch <= dec_is_branch; id_ex_is_call <= dec_is_call;
                id_ex_is_ret <= dec_is_ret; id_ex_is_branch_reg <= dec_br_reg;
                id_ex_is_branch_lit <= dec_br_lit; id_ex_is_branch_nz <= dec_br_nz;
                id_ex_is_branch_gt <= dec_br_gt; id_ex_rd_is_br_tgt <= dec_rd_is_br_tgt;
                id_ex_is_fp <= (dec_alu_op >= 5'h14) && (dec_alu_op <= 5'h17);
                id_ex_is_halt <= dec_is_halt; id_ex_has_lsq <= dec_needs_lsq;
                id_ex_lsq_idx <= lsq_tail; id_ex_pred_taken <= pred_taken;
                id_ex_pred_tgt <= pred_tgt; id_ex_seq_next <= if_id_seq_next;
            end

            // LSQ allocation
            if (!dec_stall && !ex_mispredict && !pred_fetch_redir &&
                !mem_ret_resolve && dec_needs_lsq) begin
                lsq_valid[lsq_tail] <= 1'b1;
                lsq_is_ld[lsq_tail] <= dec_rd_mem;
                lsq_is_st[lsq_tail] <= dec_wr_mem;
                lsq_done[lsq_tail]  <= 1'b0;
                lsq_a_rdy[lsq_tail] <= 1'b0;
                lsq_d_rdy[lsq_tail] <= 1'b0;
            end

            // Control stall for return
            if (mem_ret_resolve || (id_ex_valid && (id_ex_is_branch || id_ex_is_call)) ||
                ex_mispredict)
                ctrl_pending <= 1'b0;
            else if (if_id_valid && dec_is_ret && !dec_stall)
                ctrl_pending <= 1'b1;

            // -------- PC --------
            if (mem_ret_resolve)
                pc <= mem_ret_tgt;
            else if (ex_mispredict)
                pc <= ex_redir_valid ? ex_redir_tgt : id_ex_seq_next;
            else if (pred_fetch_redir && !dec_stall)
                pc <= pred_fetch_tgt;
            else if (!ctrl_pending && !dec_stall)
                pc <= pc + 64'd4;

            // -------- IF/ID --------
            if (mem_ret_resolve || ex_mispredict) begin
                if_id_valid <= 1'b0; if_id_pc <= 64'd0; if_id_instr <= 32'd0;
                if_id_seq_next <= 64'd0;
            end else if (dec_stall) begin
                // hold
            end else if (!ctrl_pending) begin
                if_id_valid    <= 1'b1;
                if_id_pc       <= pc;
                if_id_instr    <= mem_instr;
                if_id_seq_next <= pc + 64'd4;
            end else begin
                if_id_valid <= 1'b0; if_id_pc <= 64'd0; if_id_instr <= 32'd0;
                if_id_seq_next <= 64'd0;
            end

            // -------- LSQ pointers --------
            begin : lsq_ptrs
                reg rh, at;
                rh = lsq_do_retire && lsq_valid[lsq_retire_idx] &&
                     (lsq_retire_idx == lsq_head);
                at = !dec_stall && !ex_mispredict && !pred_fetch_redir &&
                     !mem_ret_resolve && dec_needs_lsq;
                if (rh && at) begin
                    lsq_head <= (lsq_head == LSQ_DEPTH-1) ? 3'd0 : lsq_head + 3'd1;
                    lsq_tail <= (lsq_tail == LSQ_DEPTH-1) ? 3'd0 : lsq_tail + 3'd1;
                end else if (rh) begin
                    lsq_head <= (lsq_head == LSQ_DEPTH-1) ? 3'd0 : lsq_head + 3'd1;
                    lsq_cnt  <= lsq_cnt - 4'd1;
                end else if (at) begin
                    lsq_tail <= (lsq_tail == LSQ_DEPTH-1) ? 3'd0 : lsq_tail + 3'd1;
                    lsq_cnt  <= lsq_cnt + 4'd1;
                end
            end

        end // !halt
    end // !reset
end

// ============================================================
// HALT
// ============================================================
always @(*) begin
    hlt = mem_wb_valid && mem_wb_is_halt && !cmt_st_pending &&
          !fp_s0_v && !fp_s1_v && !fp_s2_v && !fp_s3_v && !fp_s4_v;
end

endmodule