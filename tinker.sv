// tinker_core.sv — 5-stage in-order pipeline
// Stages: IF → ID → EX → MEM → WB
//
// All support-file port mappings:
//   decoder  : .instruct .raddr1 .raddr2 .waddr .immediate .op .use_imm .write
//              .is_load .is_store .is_branch .is_brgt .is_jump .is_brr_reg .is_brr_imm
//              .is_return .is_call .is_halt .is_mov_reg .is_mov_imm .rt_addr
//              NOTE: 'op' is the ALU op; branch/load/store flags separate
//   alu      : .clk .reset .valid_in .a .b .op[4:0] .rob_tag_in[5:0]
//              → .valid_out .result .rob_tag_out  (1-cycle pipelined)
//   fpu      : .clk .reset .valid_in .a .b .op[2:0] .rob_tag_in[5:0]
//              → .valid_out .result .rob_tag_out  (3-cycle pipelined)
//              op encoding: 0=addf 1=subf 2=mulf 3=divf
//   reg_file : .clk .reset .data .raddr1 .raddr2 .raddr3 .waddr .write → .r1 .r2 .r3
//   memory   : .clk .fetch_addr0 .fetch_addr1 → .instr_out0 .instr_out1
//              .data_addr .write_data .we → .read_data
//
// Because alu is 1-cycle pipelined, branch resolution happens one cycle after
// instructions enter EX (i.e., aligned with alu_vout).  We track a one-cycle
// "execution side-channel" (exc_*) that mirrors the ID/EX control fields so
// they arrive at the same cycle as alu_vout.

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

  // ─────────────────────────────────────────────
  // CONSTANTS
  // ─────────────────────────────────────────────
  localparam [63:0] PC_INIT = `PC_START;
  localparam [4:0] SP = 5'd31;
  localparam LSQDEP = 8;

  // Instruction opcodes (from decoder.sv case statements)
  localparam OP_BR = 5'h08;  // br rd  — abs jump
  localparam OP_BRR = 5'h09;  // brr rd — pc-rel reg
  localparam OP_BRRI = 5'h0A;  // brr L  — pc-rel imm
  localparam OP_BRNZ = 5'h0B;  // brnz rd,rs
  localparam OP_CALL = 5'h0C;
  localparam OP_RET = 5'h0D;
  localparam OP_BRGT = 5'h0E;
  localparam OP_HALT = 5'h0F;
  localparam OP_LOAD = 5'h10;
  localparam OP_MOV = 5'h11;
  localparam OP_MOVI = 5'h12;
  localparam OP_STORE = 5'h13;
  localparam OP_ADDF = 5'h14;
  localparam OP_SUBF = 5'h15;
  localparam OP_MULF = 5'h16;
  localparam OP_DIVF = 5'h17;

  // ─────────────────────────────────────────────
  // REGISTER FILES
  // ─────────────────────────────────────────────
  reg [63:0] gprf[0:31];  // fast copy for forwarding

  // ─────────────────────────────────────────────
  // IF/ID LATCH
  // ─────────────────────────────────────────────
  reg pipe_if_v;
  reg [63:0] pipe_if_pc;
  reg [31:0] pipe_if_ir;
  reg [63:0] pipe_if_npc;  // pc + 4

  // ─────────────────────────────────────────────
  // ID/EX LATCH
  // ─────────────────────────────────────────────
  reg pipe_id_v;
  reg [63:0] pipe_id_pc;
  reg [4:0] pipe_id_dst;
  reg [4:0] pipe_id_aop;  // ALU op (decoder 'op' output)
  reg [4:0] pipe_id_iop;  // instruction opcode (for branch/mem classification)
  reg [63:0] pipe_id_rA;  // operand A
  reg [63:0] pipe_id_rB;  // operand B / store data
  reg [63:0] pipe_id_rC;  // operand C (brgt third src)
  reg [63:0] pipe_id_imm;
  reg pipe_id_use_imm;
  reg pipe_id_writes;
  reg pipe_id_is_ld;
  reg pipe_id_is_st;
  reg pipe_id_is_fp;
  reg pipe_id_is_hlt;
  reg pipe_id_is_ret;
  reg pipe_id_is_call;
  reg pipe_id_is_br;  // any branch (brnz/brgt/brr/br)
  reg pipe_id_br_abs;  // br rd (absolute)
  reg pipe_id_br_reg;  // brr rd (pc-relative via reg)
  reg pipe_id_br_imm;  // brr L  (pc-relative via imm)
  reg pipe_id_br_nz;  // brnz
  reg pipe_id_br_gt;  // brgt
  reg pipe_id_has_lsq;
  reg [2:0] pipe_id_lsq_slot;
  reg pipe_id_ptaken;  // predicted taken?
  reg [63:0] pipe_id_ptgt;  // predicted target
  reg [63:0] pipe_id_npc;

  // ─────────────────────────────────────────────
  // EXECUTION SIDE-CHANNEL (ID/EX fields shifted 1 cycle, aligned with alu_vout)
  // ─────────────────────────────────────────────
  reg exc_v;
  reg [63:0] exc_pc;
  reg [4:0] exc_dst;
  reg [4:0] exc_iop;
  reg [63:0] exc_rA, exc_rB, exc_rC, exc_imm;
  reg exc_writes;
  reg exc_is_ld, exc_is_st;
  reg exc_is_ret, exc_is_call, exc_is_br;
  reg exc_br_abs, exc_br_reg, exc_br_imm, exc_br_nz, exc_br_gt;
  reg exc_is_fp, exc_is_hlt;
  reg        exc_has_lsq;
  reg [ 2:0] exc_lsq_slot;
  reg        exc_ptaken;
  reg [63:0] exc_ptgt;
  reg [63:0] exc_npc;

  // ─────────────────────────────────────────────
  // EX/MEM LATCH
  // ─────────────────────────────────────────────
  reg        pipe_ex_v;
  reg [ 4:0] pipe_ex_dst;
  reg [63:0] pipe_ex_maddr;
  reg [63:0] pipe_ex_alu;
  reg [63:0] pipe_ex_sval;  // store value
  reg        pipe_ex_writes;
  reg        pipe_ex_is_ld;
  reg        pipe_ex_is_st;
  reg        pipe_ex_rd_from_mem;
  reg        pipe_ex_is_ret;
  reg        pipe_ex_is_hlt;
  reg        pipe_ex_has_lsq;
  reg [ 2:0] pipe_ex_lsq_slot;

  // ─────────────────────────────────────────────
  // MEM/WB LATCH
  // ─────────────────────────────────────────────
  reg        pipe_wb_v;
  reg [ 4:0] pipe_wb_dst;
  reg [63:0] pipe_wb_result;
  reg        pipe_wb_writes;
  reg        pipe_wb_is_hlt;
  reg        pipe_wb_has_lsq;
  reg [ 2:0] pipe_wb_lsq_slot;

  // ─────────────────────────────────────────────
  // PC
  // ─────────────────────────────────────────────
  reg [63:0] fetch_pc;
  reg        stall_for_ret;  // assert after dispatching return; clear when MEM resolves

  // ─────────────────────────────────────────────
  // DECODER WIRES
  // ─────────────────────────────────────────────
  wire [4:0] dw_raddr1, dw_raddr2, dw_waddr, dw_rt_addr;
  wire [63:0] dw_imm;
  wire [ 4:0] dw_op;  // This IS the ALU op for integer instr; also instruction category
  wire dw_use_imm, dw_write;
  wire dw_is_load, dw_is_store;
  wire dw_is_branch, dw_is_brgt, dw_is_jump;
  wire dw_is_brr_reg, dw_is_brr_imm;
  wire dw_is_ret, dw_is_call, dw_is_halt;
  wire dw_is_mov_reg, dw_is_mov_imm;

  // Instruction opcode (bits 31:27) — for classification not in decoder outputs
  wire [4:0] if_iop = pipe_if_ir[31:27];

  decoder u_dec (
      .instruct  (pipe_if_ir),
      .raddr1    (dw_raddr1),
      .raddr2    (dw_raddr2),
      .waddr     (dw_waddr),
      .immediate (dw_imm),
      .op        (dw_op),
      .use_imm   (dw_use_imm),
      .write     (dw_write),
      .is_load   (dw_is_load),
      .is_store  (dw_is_store),
      .is_branch (dw_is_branch),
      .is_brgt   (dw_is_brgt),
      .is_jump   (dw_is_jump),
      .is_brr_reg(dw_is_brr_reg),
      .is_brr_imm(dw_is_brr_imm),
      .is_return (dw_is_ret),
      .is_call   (dw_is_call),
      .is_halt   (dw_is_halt),
      .is_mov_reg(dw_is_mov_reg),
      .is_mov_imm(dw_is_mov_imm),
      .rt_addr   (dw_rt_addr)
  );

  // Derived decode signals (computed here so decoder stays unmodified)
  wire dw_is_fp = (if_iop >= OP_ADDF) && (if_iop <= OP_DIVF);
  wire dw_br_abs = dw_is_jump && !dw_is_brr_reg && !dw_is_brr_imm && !dw_is_call && !dw_is_ret;
  wire dw_br_nz = dw_is_branch && !dw_is_brgt;
  wire dw_is_ctrl = dw_is_jump || dw_is_branch;  // any control flow
  wire dw_needs_lsq = pipe_if_v && (dw_is_load || dw_is_store) && !dw_is_call && !dw_is_ret;

  // ─────────────────────────────────────────────
  // REG_FILE (autograder compatibility, written at WB)
  // ─────────────────────────────────────────────
  reg_file u_rf (
      .clk   (clk),
      .reset (reset),
      .data  (pipe_wb_result),
      .waddr (pipe_wb_dst),
      .write (pipe_wb_v && pipe_wb_writes),
      .raddr1(5'd0),
      .raddr2(5'd0),
      .raddr3(5'd0),
      .r1(),
      .r2(),
      .r3()
  );

  // ─────────────────────────────────────────────
  // ALU (1-cycle pipelined)
  // ─────────────────────────────────────────────
  reg alu_fire;
  reg [63:0] alu_ain, alu_bin;
  reg  [ 4:0] alu_op_r;

  wire        alu_vout;
  wire [63:0] alu_result;
  wire [ 5:0] alu_tag_out;

  alu u_alu (
      .clk        (clk),
      .reset      (reset),
      .valid_in   (alu_fire),
      .a          (alu_ain),
      .b          (alu_bin),
      .op         (alu_op_r),
      .rob_tag_in (6'd0),
      .valid_out  (alu_vout),
      .result     (alu_result),
      .rob_tag_out(alu_tag_out)
  );

  // ─────────────────────────────────────────────
  // FPU (3-cycle pipelined; op[2:0]: 0=add,1=sub,2=mul,3=div)
  // ─────────────────────────────────────────────
  reg fpu_fire;
  reg [63:0] fpu_ain, fpu_bin;
  reg  [ 2:0] fpu_op_r;

  wire        fpu_vout;
  wire [63:0] fpu_result;

  fpu u_fpu (
      .clk        (clk),
      .reset      (reset),
      .valid_in   (fpu_fire),
      .a          (fpu_ain),
      .b          (fpu_bin),
      .op         (fpu_op_r),
      .rob_tag_in (6'd0),
      .valid_out  (fpu_vout),
      .result     (fpu_result),
      .rob_tag_out()
  );

  // FP destination tracking (4 pipeline stages + writeback)
  reg         fps_v                           [0:4];
  reg  [ 4:0] fps_dst                         [0:4];
  reg         fps_wr                          [0:4];

  wire        fp_wb_v = fpu_vout && fps_wr[4];
  wire [ 4:0] fp_wb_dst = fps_dst[4];
  wire [63:0] fp_wb_val = fpu_result;

  // ─────────────────────────────────────────────
  // MEMORY
  // ─────────────────────────────────────────────
  wire [31:0] fetch_ir0, fetch_ir1;  // dual combinational instruction fetch

  reg  [63:0] mem_daddr;
  reg  [63:0] mem_wdata;
  reg         mem_we;
  wire [63:0] mem_rdata;

  memory #(
      .MEM_SIZE(`MEM_SIZE)
  ) u_mem (
      .clk        (clk),
      .fetch_addr0(fetch_pc),
      .fetch_addr1(fetch_pc + 64'd4),
      .instr_out0 (fetch_ir0),
      .instr_out1 (fetch_ir1),
      .data_addr  (mem_daddr),
      .write_data (mem_wdata),
      .we         (mem_we),
      .read_data  (mem_rdata)
  );

  // ─────────────────────────────────────────────
  // IMMEDIATE EXPAND
  // ─────────────────────────────────────────────
  // The decoder already outputs the correctly expanded 64-bit immediate.
  // We use dw_imm directly.

  // ─────────────────────────────────────────────
  // LSQ
  // ─────────────────────────────────────────────
  reg        lsq_v    [0:LSQDEP-1];
  reg        lsq_is_ld[0:LSQDEP-1];
  reg        lsq_is_st[0:LSQDEP-1];
  reg [63:0] lsq_addr [0:LSQDEP-1];
  reg [63:0] lsq_data [0:LSQDEP-1];
  reg [2:0] lsq_head, lsq_tail;
  reg  [ 3:0] lsq_cnt;
  wire        lsq_full = (lsq_cnt >= LSQDEP - 1);

  reg         cmt_st_pend;
  reg  [ 2:0] cmt_st_slot;
  reg  [63:0] cmt_st_addr;
  reg  [63:0] cmt_st_data;

  // ─────────────────────────────────────────────
  // DECODE COMBINATIONAL: operand fetch + forwarding + prediction + stall
  // ─────────────────────────────────────────────
  reg [63:0] id_opA, id_opB, id_opC;
  reg [4:0] id_ra1, id_ra2, id_ra3;
  reg id_s1u, id_s2u, id_s3u;
  reg         id_stall;
  reg         id_ptaken;
  reg  [63:0] id_ptgt;
  reg         id_pred_redir;
  reg  [63:0] id_pred_tgt;

  // EX-stage branch resolution (combinational, based on exc_* + alu_result)
  reg         ex_redir_v;
  reg  [63:0] ex_redir_tgt;
  reg         ex_mispredict;

  // Return resolved in MEM
  wire        ret_resolve = pipe_ex_v && pipe_ex_is_ret;
  wire [63:0] ret_tgt = mem_rdata;

  always @(*) begin
    // ── Operand address select (mirrors friend's logic exactly) ──
    id_ra1 = 5'd0;
    id_ra2 = 5'd0;
    id_ra3 = 5'd0;
    id_s1u = 0;
    id_s2u = 0;
    id_s3u = 0;

    if (dw_br_nz) begin
      // brnz rd, rs: compare rs != 0; target = rd value
      // decoder sets raddr1=rs, raddr2=rd
      id_ra1 = dw_raddr1;
      id_ra2 = dw_raddr2;
      id_s1u = 1;
      id_s2u = 1;
    end else if (dw_is_brgt) begin
      // brgt rd, rs, rt: compare rs>rt; target = rd value
      // decoder sets raddr1=rd, raddr2=rs, rt_addr=rt
      id_ra1 = dw_raddr1;
      id_ra2 = dw_raddr2;
      id_ra3 = dw_rt_addr;
      id_s1u = 1;
      id_s2u = 1;
      id_s3u = 1;
    end else if (dw_is_call) begin
      // decoder sets raddr1=rd (target), raddr2=r31 (sp)
      id_ra1 = dw_raddr1;
      id_ra2 = dw_raddr2;
      id_s1u = 1;
      id_s2u = 1;
    end else if (dw_is_ret) begin
      // decoder sets raddr1=r31
      id_ra1 = dw_raddr1;
      id_s1u = 1;
    end else begin
      // Normal arithmetic / load / store / br / brr
      id_ra1 = dw_raddr1;
      id_ra2 = dw_raddr2;
      id_s1u = 1;
      id_s2u = (dw_raddr2 != 5'd0 || dw_is_store);
    end

    // ── Base values from gprf ──
    id_opA = gprf[id_ra1];
    id_opB = gprf[id_ra2];
    id_opC = gprf[id_ra3];

    // ── Forwarding from WB ──
    if (pipe_wb_v && pipe_wb_writes) begin
      if (id_s1u && id_ra1 == pipe_wb_dst) id_opA = pipe_wb_result;
      if (id_s2u && id_ra2 == pipe_wb_dst) id_opB = pipe_wb_result;
      if (id_s3u && id_ra3 == pipe_wb_dst) id_opC = pipe_wb_result;
    end

    // ── Forwarding from EX/MEM (non-load) ──
    if (pipe_ex_v && pipe_ex_writes && !pipe_ex_rd_from_mem) begin
      if (id_s1u && id_ra1 == pipe_ex_dst) id_opA = pipe_ex_alu;
      if (id_s2u && id_ra2 == pipe_ex_dst) id_opB = pipe_ex_alu;
      if (id_s3u && id_ra3 == pipe_ex_dst) id_opC = pipe_ex_alu;
    end

    // ── Forwarding from FP WB ──
    if (fp_wb_v) begin
      if (id_s1u && id_ra1 == fp_wb_dst) id_opA = fp_wb_val;
      if (id_s2u && id_ra2 == fp_wb_dst) id_opB = fp_wb_val;
      if (id_s3u && id_ra3 == fp_wb_dst) id_opC = fp_wb_val;
    end

    // ── Stall detection ──
    id_stall = 1'b0;
    if (pipe_if_v) begin
      if (dw_needs_lsq && lsq_full) id_stall = 1'b1;

      // Load-use hazard
      if (id_s1u &&
            ((pipe_id_v && pipe_id_writes && id_ra1 == pipe_id_dst) ||
             (pipe_ex_v && pipe_ex_writes && pipe_ex_rd_from_mem && id_ra1 == pipe_ex_dst)))
        id_stall = 1'b1;
      if (id_s2u &&
            ((pipe_id_v && pipe_id_writes && id_ra2 == pipe_id_dst) ||
             (pipe_ex_v && pipe_ex_writes && pipe_ex_rd_from_mem && id_ra2 == pipe_ex_dst)))
        id_stall = 1'b1;
      if (id_s3u &&
            ((pipe_id_v && pipe_id_writes && id_ra3 == pipe_id_dst) ||
             (pipe_ex_v && pipe_ex_writes && pipe_ex_rd_from_mem && id_ra3 == pipe_ex_dst)))
        id_stall = 1'b1;

      // FP pipeline hazard
      if (id_s1u && (
            (fps_v[0] && id_ra1 == fps_dst[0]) || (fps_v[1] && id_ra1 == fps_dst[1]) ||
            (fps_v[2] && id_ra1 == fps_dst[2]) || (fps_v[3] && id_ra1 == fps_dst[3])))
        id_stall = 1'b1;
      if (id_s2u && (
            (fps_v[0] && id_ra2 == fps_dst[0]) || (fps_v[1] && id_ra2 == fps_dst[1]) ||
            (fps_v[2] && id_ra2 == fps_dst[2]) || (fps_v[3] && id_ra2 == fps_dst[3])))
        id_stall = 1'b1;
      if (id_s3u && (
            (fps_v[0] && id_ra3 == fps_dst[0]) || (fps_v[1] && id_ra3 == fps_dst[1]) ||
            (fps_v[2] && id_ra3 == fps_dst[2]) || (fps_v[3] && id_ra3 == fps_dst[3])))
        id_stall = 1'b1;
    end

    // ── Branch prediction (mirrors friend's design exactly) ──
    id_ptaken = 1'b0;
    id_ptgt   = pipe_if_pc + 64'd4;

    if (pipe_if_v && !dw_is_ret) begin
      if (dw_is_call) begin
        id_ptaken = 1'b1;
        id_ptgt   = id_opA;  // call target = rd value (raddr1=rd)
      end else if (dw_is_brr_reg) begin
        id_ptaken = 1'b1;
        id_ptgt   = pipe_if_pc + id_opA;
      end else if (dw_is_brr_imm) begin
        id_ptaken = 1'b1;
        id_ptgt   = pipe_if_pc + dw_imm;
      end else if (dw_br_abs) begin
        id_ptaken = 1'b1;
        id_ptgt   = id_opA;  // br rd: absolute
      end else if (dw_br_nz) begin
        // backward-branch-taken: target(=rd value=raddr2) < current_pc
        id_ptaken = (id_opB < pipe_if_pc) ? 1'b1 : 1'b0;
        id_ptgt   = (id_opB < pipe_if_pc) ? id_opB : (pipe_if_pc + 64'd4);
      end else if (dw_is_brgt) begin
        // target = rd value = raddr1
        id_ptaken = (id_opA < pipe_if_pc) ? 1'b1 : 1'b0;
        id_ptgt   = (id_opA < pipe_if_pc) ? id_opA : (pipe_if_pc + 64'd4);
      end
    end

    id_pred_redir = pipe_if_v && !id_stall && id_ptaken;
    id_pred_tgt   = id_ptgt;

    // ── EX-stage branch resolution (from exc_* + alu_result, fires when alu_vout) ──
    ex_redir_v    = 1'b0;
    ex_redir_tgt  = 64'd0;

    if (exc_v) begin
      if (exc_is_call) begin
        ex_redir_v   = 1'b1;
        ex_redir_tgt = exc_rA;  // call target = rd value
      end else if (exc_br_reg) begin
        ex_redir_v   = 1'b1;
        ex_redir_tgt = exc_pc + exc_rA;
      end else if (exc_br_imm) begin
        ex_redir_v   = 1'b1;
        ex_redir_tgt = exc_pc + exc_imm;
      end else if (exc_br_nz) begin
        // alu computed CMPNZ(rs); alu_result[0]=1 if taken
        ex_redir_v   = alu_result[0];
        ex_redir_tgt = exc_rB;  // target = rd value (raddr2=rd for brnz)
      end else if (exc_br_gt) begin
        // alu computed CMPGT(rd,rs); alu_result[0]=1 if taken
        ex_redir_v   = alu_result[0];
        ex_redir_tgt = exc_rA;  // target = rd value (raddr1=rd for brgt)
      end else if (exc_br_abs) begin
        ex_redir_v   = 1'b1;
        ex_redir_tgt = exc_rA;
      end
    end

    // Mispredict: actual outcome differs from prediction
    ex_mispredict = alu_vout && exc_v &&
                    (exc_is_br || exc_is_call || exc_br_abs) &&
                    ((exc_ptaken != ex_redir_v) ||
                     (ex_redir_v && exc_ptgt != ex_redir_tgt));
  end

  // LSQ retire/commit combinational
  reg       lsq_retire;
  reg [2:0] lsq_retire_slot;
  reg       lsq_commit_st;

  always @(*) begin
    lsq_retire      = 1'b0;
    lsq_retire_slot = 3'd0;
    lsq_commit_st   = 1'b0;

    // Store commit: when data port is free
    if (cmt_st_pend && !(pipe_ex_v && (pipe_ex_is_ld || (pipe_ex_is_st && !pipe_ex_has_lsq)))) begin
      lsq_commit_st   = 1'b1;
      lsq_retire      = 1'b1;
      lsq_retire_slot = cmt_st_slot;
    end

    // Load retirement at WB
    if (pipe_wb_v && pipe_wb_has_lsq &&
        lsq_v[pipe_wb_lsq_slot] && lsq_is_ld[pipe_wb_lsq_slot]) begin
      lsq_retire      = 1'b1;
      lsq_retire_slot = pipe_wb_lsq_slot;
    end
  end

  // ─────────────────────────────────────────────
  // MAIN CLOCKED LOGIC
  // ─────────────────────────────────────────────
  integer qi;

  always @(posedge clk) begin
    if (reset) begin
      fetch_pc      <= PC_INIT;
      stall_for_ret <= 1'b0;

      pipe_if_v     <= 1'b0;
      pipe_id_v     <= 1'b0;
      pipe_ex_v     <= 1'b0;
      pipe_wb_v     <= 1'b0;
      exc_v         <= 1'b0;

      for (qi = 0; qi < 5; qi = qi + 1) begin
        fps_v[qi]  <= 0;
        fps_wr[qi] <= 0;
      end

      lsq_head <= 0;
      lsq_tail <= 0;
      lsq_cnt <= 0;
      cmt_st_pend <= 1'b0;
      alu_fire <= 1'b0;
      fpu_fire <= 1'b0;
      mem_we <= 1'b0;

      for (qi = 0; qi < 32; qi = qi + 1) gprf[qi] <= 64'd0;
      for (qi = 0; qi < LSQDEP; qi = qi + 1) begin
        lsq_v[qi] <= 0;
        lsq_is_ld[qi] <= 0;
        lsq_is_st[qi] <= 0;
      end
      gprf[31] <= 64'd524288;

    end else begin

      // ── Defaults for pulse signals ──
      alu_fire <= 1'b0;
      fpu_fire <= 1'b0;
      mem_we <= 1'b0;

      // ─────────────────────────────────────
      // FP pipeline shift
      // ─────────────────────────────────────
      fps_v[4] <= fps_v[3];
      fps_dst[4] <= fps_dst[3];
      fps_wr[4] <= fps_wr[3];
      fps_v[3] <= fps_v[2];
      fps_dst[3] <= fps_dst[2];
      fps_wr[3] <= fps_wr[2];
      fps_v[2] <= fps_v[1];
      fps_dst[2] <= fps_dst[1];
      fps_wr[2] <= fps_wr[1];
      fps_v[1] <= fps_v[0];
      fps_dst[1] <= fps_dst[0];
      fps_wr[1] <= fps_wr[0];
      fps_v[0] <= pipe_id_v && pipe_id_is_fp;
      fps_dst[0] <= pipe_id_dst;
      fps_wr[0] <= pipe_id_v && pipe_id_writes;

      // ─────────────────────────────────────
      // LSQ cleanup
      // ─────────────────────────────────────
      if (lsq_commit_st) cmt_st_pend <= 1'b0;

      if (lsq_retire && lsq_v[lsq_retire_slot]) begin
        lsq_v[lsq_retire_slot]     <= 1'b0;
        lsq_is_ld[lsq_retire_slot] <= 1'b0;
        lsq_is_st[lsq_retire_slot] <= 1'b0;
      end

      // ─────────────────────────────────────
      // Memory port drive
      // ─────────────────────────────────────
      if (lsq_commit_st) begin
        mem_daddr <= cmt_st_addr;
        mem_wdata <= cmt_st_data;
        mem_we    <= 1'b1;
      end else begin
        mem_daddr <= pipe_ex_maddr;
        mem_wdata <= 64'd0;
      end

      // ─────────────────────────────────────
      // WB stage
      // ─────────────────────────────────────
      if (!(pipe_wb_v && pipe_wb_is_hlt)) begin

        pipe_wb_v        <= pipe_ex_v;
        pipe_wb_dst      <= pipe_ex_dst;
        pipe_wb_result   <= pipe_ex_rd_from_mem ? mem_rdata : pipe_ex_alu;
        pipe_wb_writes   <= pipe_ex_writes;
        pipe_wb_is_hlt   <= pipe_ex_is_hlt;
        pipe_wb_has_lsq  <= pipe_ex_has_lsq;
        pipe_wb_lsq_slot <= pipe_ex_lsq_slot;

        // Write back to gprf
        if (pipe_wb_v && pipe_wb_writes) gprf[pipe_wb_dst] <= pipe_wb_result;

        if (fp_wb_v) gprf[fp_wb_dst] <= fp_wb_val;

        // Stage deferred store
        if (pipe_wb_v && pipe_wb_has_lsq &&
                lsq_v[pipe_wb_lsq_slot] && lsq_is_st[pipe_wb_lsq_slot]) begin
          cmt_st_pend <= 1'b1;
          cmt_st_slot <= pipe_wb_lsq_slot;
          cmt_st_addr <= lsq_addr[pipe_wb_lsq_slot];
          cmt_st_data <= lsq_data[pipe_wb_lsq_slot];
        end

        // ────────────────────────────────
        // EX/MEM: populated from exc_* when alu_vout fires
        // ────────────────────────────────

        // Handle return separately (no ALU, goes directly to pipe_ex)
        if (pipe_id_v && pipe_id_is_ret) begin
          pipe_ex_v           <= 1'b1;
          pipe_ex_dst         <= 5'd0;
          pipe_ex_maddr       <= pipe_id_rA - 64'd8;  // r31 - 8
          pipe_ex_alu         <= 64'd0;
          pipe_ex_sval        <= 64'd0;
          pipe_ex_writes      <= 1'b0;
          pipe_ex_is_ld       <= 1'b0;
          pipe_ex_is_st       <= 1'b0;
          pipe_ex_rd_from_mem <= 1'b0;
          pipe_ex_is_ret      <= 1'b1;
          pipe_ex_is_hlt      <= 1'b0;
          pipe_ex_has_lsq     <= 1'b0;
          pipe_ex_lsq_slot    <= 3'd0;
          // Suppress alu result path for return
          exc_v               <= 1'b0;
        end else if (alu_vout && exc_v) begin
          // Normal: alu result arrived — populate EX/MEM
          begin : ex_populate
            reg [63:0] calc_addr;
            reg [63:0] calc_sval;
            calc_addr = alu_result;  // for loads/stores, alu computed base+imm
            calc_sval = exc_rB;

            if (exc_is_call) begin
              calc_addr = exc_rB - 64'd8;  // sp - 8
              calc_sval = exc_pc + 64'd4;  // return address
            end

            pipe_ex_v           <= exc_v;
            pipe_ex_dst         <= exc_dst;
            pipe_ex_maddr       <= calc_addr;
            pipe_ex_alu         <= alu_result;
            pipe_ex_sval        <= calc_sval;
            pipe_ex_writes      <= exc_writes;
            pipe_ex_is_ld       <= exc_is_ld;
            pipe_ex_is_st       <= exc_is_st;
            pipe_ex_rd_from_mem <= exc_is_ld;
            pipe_ex_is_ret      <= 1'b0;
            pipe_ex_is_hlt      <= exc_is_hlt;
            pipe_ex_has_lsq     <= exc_has_lsq;
            pipe_ex_lsq_slot    <= exc_lsq_slot;

            // Update LSQ with resolved address/data
            if (exc_has_lsq) begin
              lsq_addr[exc_lsq_slot] <= calc_addr;
              if (exc_is_st) lsq_data[exc_lsq_slot] <= calc_sval;
            end
          end

          // Mispredict flush
          if (ex_mispredict) begin
            pipe_if_v     <= 1'b0;
            pipe_id_v     <= 1'b0;
            exc_v         <= 1'b0;
            stall_for_ret <= 1'b0;
            fetch_pc      <= ex_redir_v ? ex_redir_tgt : exc_npc;
          end

        end else if (pipe_id_v && pipe_id_is_fp) begin
          // FP instruction: bubble EX/MEM, FP pipeline tracks separately
          pipe_ex_v <= 1'b0;
          pipe_ex_writes <= 1'b0;
          pipe_ex_is_ld <= 1'b0;
          pipe_ex_is_st <= 1'b0;
          pipe_ex_is_ret <= 1'b0;
          pipe_ex_is_hlt <= 1'b0;
          pipe_ex_has_lsq <= 1'b0;
        end else if (!alu_vout) begin
          // No ALU result — bubble (stall or no instruction in EX)
          pipe_ex_v <= 1'b0;
          pipe_ex_writes <= 1'b0;
          pipe_ex_is_ld <= 1'b0;
          pipe_ex_is_st <= 1'b0;
          pipe_ex_is_ret <= 1'b0;
          pipe_ex_is_hlt <= 1'b0;
          pipe_ex_has_lsq <= 1'b0;
        end

        // Return resolve (MEM reads return address from memory)
        if (ret_resolve) begin
          pipe_if_v     <= 1'b0;
          pipe_id_v     <= 1'b0;
          exc_v         <= 1'b0;
          stall_for_ret <= 1'b0;
          fetch_pc      <= ret_tgt;
        end

        // ────────────────────────────────
        // Side-channel shift: ID/EX → exc_*
        // ────────────────────────────────
        exc_v        <= pipe_id_v;
        exc_pc       <= pipe_id_pc;
        exc_dst      <= pipe_id_dst;
        exc_iop      <= pipe_id_iop;
        exc_rA       <= pipe_id_rA;
        exc_rB       <= pipe_id_rB;
        exc_rC       <= pipe_id_rC;
        exc_imm      <= pipe_id_imm;
        exc_writes   <= pipe_id_writes;
        exc_is_ld    <= pipe_id_is_ld;
        exc_is_st    <= pipe_id_is_st;
        exc_is_ret   <= pipe_id_is_ret;
        exc_is_call  <= pipe_id_is_call;
        exc_is_br    <= pipe_id_is_br;
        exc_br_abs   <= pipe_id_br_abs;
        exc_br_reg   <= pipe_id_br_reg;
        exc_br_imm   <= pipe_id_br_imm;
        exc_br_nz    <= pipe_id_br_nz;
        exc_br_gt    <= pipe_id_br_gt;
        exc_is_fp    <= pipe_id_is_fp;
        exc_is_hlt   <= pipe_id_is_hlt;
        exc_has_lsq  <= pipe_id_has_lsq;
        exc_lsq_slot <= pipe_id_lsq_slot;
        exc_ptaken   <= pipe_id_ptaken;
        exc_ptgt     <= pipe_id_ptgt;
        exc_npc      <= pipe_id_npc;

        // ────────────────────────────────
        // EX stage: fire ALU/FPU from ID/EX latch
        // ────────────────────────────────
        if (!ex_mispredict && !ret_resolve) begin

          if (pipe_id_v && !pipe_id_is_fp && !pipe_id_is_ret && !pipe_id_is_hlt) begin
            alu_fire <= 1'b1;
            alu_ain  <= pipe_id_rA;
            alu_bin  <= pipe_id_use_imm ? pipe_id_imm : pipe_id_rB;
            alu_op_r <= pipe_id_aop;
          end else if (pipe_id_v && pipe_id_is_fp) begin
            fpu_fire <= 1'b1;
            fpu_ain <= pipe_id_rA;
            fpu_bin <= pipe_id_use_imm ? pipe_id_imm : pipe_id_rB;
            // Map ALU opcode to FPU op[2:0]: 0x14→0, 0x15→1, 0x16→2, 0x17→3
            fpu_op_r <= pipe_id_aop[1:0] <= 2'b01 ? {1'b0, pipe_id_aop[0]} : {1'b1, pipe_id_aop[0]};
          end
          // halt/ret: no ALU needed — handled separately

          // ── ID/EX ← IF/ID ──
          if (id_stall || !pipe_if_v) begin
            pipe_id_v       <= 1'b0;
            pipe_id_writes  <= 1'b0;
            pipe_id_is_ld   <= 1'b0;
            pipe_id_is_st   <= 1'b0;
            pipe_id_is_ret  <= 1'b0;
            pipe_id_is_call <= 1'b0;
            pipe_id_is_br   <= 1'b0;
            pipe_id_is_fp   <= 1'b0;
            pipe_id_is_hlt  <= 1'b0;
            pipe_id_has_lsq <= 1'b0;
          end else begin
            pipe_id_v        <= pipe_if_v;
            pipe_id_pc       <= pipe_if_pc;
            pipe_id_dst      <= dw_waddr;
            pipe_id_aop      <= dw_op;  // ALU op from decoder
            pipe_id_iop      <= if_iop;  // raw instruction opcode
            pipe_id_rA       <= id_opA;
            pipe_id_rB       <= id_opB;
            pipe_id_rC       <= id_opC;
            pipe_id_imm      <= dw_imm;
            pipe_id_use_imm  <= dw_use_imm;
            pipe_id_writes   <= dw_write && !dw_is_store;
            pipe_id_is_ld    <= dw_is_load;
            pipe_id_is_st    <= dw_is_store;
            pipe_id_is_fp    <= dw_is_fp;
            pipe_id_is_hlt   <= dw_is_halt;
            pipe_id_is_ret   <= dw_is_ret;
            pipe_id_is_call  <= dw_is_call;
            pipe_id_is_br    <= dw_is_branch || dw_is_brr_reg || dw_is_brr_imm || dw_br_abs;
            pipe_id_br_abs   <= dw_br_abs;
            pipe_id_br_reg   <= dw_is_brr_reg;
            pipe_id_br_imm   <= dw_is_brr_imm;
            pipe_id_br_nz    <= dw_br_nz;
            pipe_id_br_gt    <= dw_is_brgt;
            pipe_id_has_lsq  <= dw_needs_lsq;
            pipe_id_lsq_slot <= lsq_tail;
            pipe_id_ptaken   <= id_ptaken;
            pipe_id_ptgt     <= id_ptgt;
            pipe_id_npc      <= pipe_if_npc;
          end

          // LSQ allocation
          if (pipe_if_v && !id_stall && !id_pred_redir && dw_needs_lsq) begin
            lsq_v[lsq_tail]     <= 1'b1;
            lsq_is_ld[lsq_tail] <= dw_is_load;
            lsq_is_st[lsq_tail] <= dw_is_store;
          end

          // Control stall bookkeeping (stall fetch after dispatching return)
          if (ret_resolve || (exc_v && (exc_is_br || exc_is_call || exc_br_abs)))
            stall_for_ret <= 1'b0;
          else if (pipe_if_v && dw_is_ret && !id_stall) stall_for_ret <= 1'b1;

          // ── PC and IF/ID ──
          if (id_pred_redir && !id_stall) begin
            // Predicted-taken: redirect fetch, flush IF/ID
            fetch_pc  <= id_pred_tgt;
            pipe_if_v <= 1'b0;
          end else if (!stall_for_ret && !id_stall) begin
            pipe_if_v   <= 1'b1;
            pipe_if_pc  <= fetch_pc;
            pipe_if_ir  <= fetch_ir0;
            pipe_if_npc <= fetch_pc + 64'd4;
            fetch_pc    <= fetch_pc + 64'd4;
          end else begin
            pipe_if_v <= 1'b0;
          end

        end  // !ex_mispredict && !ret_resolve

        // ── LSQ pointer maintenance ──
        begin : lsq_ptrs
          reg rh, at;
          rh = lsq_retire && lsq_v[lsq_retire_slot] && (lsq_retire_slot == lsq_head);
          at = pipe_if_v && !id_stall && !ex_mispredict &&
                     !id_pred_redir && !ret_resolve && dw_needs_lsq;

          if (rh && at) begin
            lsq_head <= (lsq_head == LSQDEP - 1) ? 3'd0 : lsq_head + 3'd1;
            lsq_tail <= (lsq_tail == LSQDEP - 1) ? 3'd0 : lsq_tail + 3'd1;
          end else if (rh) begin
            lsq_head <= (lsq_head == LSQDEP - 1) ? 3'd0 : lsq_head + 3'd1;
            lsq_cnt  <= lsq_cnt - 4'd1;
          end else if (at) begin
            lsq_tail <= (lsq_tail == LSQDEP - 1) ? 3'd0 : lsq_tail + 3'd1;
            lsq_cnt  <= lsq_cnt + 4'd1;
          end
        end

      end  // !halt
    end  // !reset
  end

  // ─────────────────────────────────────────────
  // HALT
  // ─────────────────────────────────────────────
  always @(*) begin
    hlt = pipe_wb_v && pipe_wb_is_hlt && !cmt_st_pend &&
          !fps_v[0] && !fps_v[1] && !fps_v[2] && !fps_v[3] && !fps_v[4];
  end

endmodule
