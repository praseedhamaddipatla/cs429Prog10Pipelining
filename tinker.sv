
// tinker.sv — tinker cpu core (ooo, dual-issue)
// optimizations: forwarding, multi-issue, ooo, pipelined fu, ls queue,
//   deeper pipeline, branch prediction, register renaming
// FIXES applied to the 57-point baseline:
//   FIX1: alu_ibgt_p1 registered flag replaces fragile "!= 64'd0" guard
//   FIX2: lsq_mwe gated on !redirect_en (no wrong-path store writes)
//   FIX3: LSQ execute blocked during flush (!flush_this_cycle)
// EXTENSIONS:
//   EXT1: Dual ALUs (alu0 + alu1), dual FPUs (fpu0 + fpu1), dual LS (ld0 + ld1)
//   EXT2: 4-port CDB (c0=ALU0, c1=LD0/FPU0, c2=ALU1, c3=LD1/FPU1)
//   EXT3: 64-byte fetch (4 instructions), 4-entry DQ

`define MEM_SIZE (512 * 1024)
`define PC_START 64'h2000

`include "hdl/alu.sv"
`include "hdl/fpu.sv"
`include "hdl/reg_file.sv"
`include "hdl/decoder.sv"
`include "hdl/fetch.sv"
`include "hdl/memory.sv"

module tinker_core (
    input clk,
    input reset,
    output logic hlt
);

  localparam NPHYS = 64;
  localparam PHYS_W = 6;
  localparam ROB_SIZE = 32;
  localparam ROB_BITS = 5;
  localparam RS_INT = 8;
  localparam RS_FP = 4;
  localparam LSQ_SIZE = 8;

  reg [      63:0] prf      [0:NPHYS-1];
  reg              prf_rdy  [0:NPHYS-1];
  reg [      63:0] arch_rf  [     0:31];

  reg [PHYS_W-1:0] rat_map  [     0:31];
  reg [PHYS_W-1:0] free_list[0:NPHYS-1];
  reg [5:0] fl_head, fl_tail;
  reg [       6:0] fl_cnt;

  reg              rob_valid     [0:ROB_SIZE-1];
  reg              rob_done      [0:ROB_SIZE-1];
  reg [       4:0] rob_arch      [0:ROB_SIZE-1];
  reg [PHYS_W-1:0] rob_phys      [0:ROB_SIZE-1];
  reg [PHYS_W-1:0] rob_old       [0:ROB_SIZE-1];
  reg [      63:0] rob_result    [0:ROB_SIZE-1];
  reg              rob_has_dest  [0:ROB_SIZE-1];
  reg              rob_is_store  [0:ROB_SIZE-1];
  reg              rob_is_halt   [0:ROB_SIZE-1];
  reg              rob_is_branch [0:ROB_SIZE-1];
  reg              rob_is_jump   [0:ROB_SIZE-1];
  reg [      63:0] rob_pc        [0:ROB_SIZE-1];
  reg              rob_pred_taken[0:ROB_SIZE-1];
  reg [      63:0] rob_pred_tgt  [0:ROB_SIZE-1];
  reg              rob_act_taken [0:ROB_SIZE-1];
  reg [      63:0] rob_act_tgt   [0:ROB_SIZE-1];

  reg [ROB_BITS-1:0] rob_head, rob_tail;
  reg  [  ROB_BITS:0] rob_cnt;

  wire                rob_full = (rob_cnt >= ROB_SIZE - 2);

  reg                 rs_v                                 [  0:RS_INT-1];
  reg  [         4:0] rs_op                                [  0:RS_INT-1];
  reg  [  PHYS_W-1:0] rs_ps                                [  0:RS_INT-1];
  reg  [  PHYS_W-1:0] rs_pt                                [  0:RS_INT-1];
  reg                 rs_psrdy                             [  0:RS_INT-1];
  reg                 rs_ptrdy                             [  0:RS_INT-1];
  reg  [        63:0] rs_vs                                [  0:RS_INT-1];
  reg  [        63:0] rs_vt                                [  0:RS_INT-1];
  reg  [        63:0] rs_imm                               [  0:RS_INT-1];
  reg                 rs_uimm                              [  0:RS_INT-1];
  reg  [ROB_BITS-1:0] rs_rob                               [  0:RS_INT-1];
  reg  [        63:0] rs_pc                                [  0:RS_INT-1];
  reg                 rs_ibr                               [  0:RS_INT-1];
  reg                 rs_ibgt                              [  0:RS_INT-1];
  reg                 rs_ijmp                              [  0:RS_INT-1];
  reg                 rs_ibrreg                            [  0:RS_INT-1];
  reg                 rs_ibrimm                            [  0:RS_INT-1];
  reg                 rs_imovr                             [  0:RS_INT-1];
  reg                 rs_imovi                             [  0:RS_INT-1];
  reg                 rs_ical                              [  0:RS_INT-1];
  reg                 rs_iret                              [  0:RS_INT-1];
  reg                 rs_ptaken                            [  0:RS_INT-1];
  reg  [        63:0] rs_ptgt                              [  0:RS_INT-1];
  reg  [  PHYS_W-1:0] rs_pt3                               [  0:RS_INT-1];
  reg                 rs_pt3rdy                            [  0:RS_INT-1];

  reg  [         3:0] rs_cnt;
  wire                rs_full = (rs_cnt >= RS_INT - 1);

  reg                 fp_v                                 [   0:RS_FP-1];
  reg  [         4:0] fp_op                                [   0:RS_FP-1];
  reg  [  PHYS_W-1:0] fp_ps                                [   0:RS_FP-1];
  reg  [  PHYS_W-1:0] fp_pt                                [   0:RS_FP-1];
  reg                 fp_psrdy                             [   0:RS_FP-1];
  reg                 fp_ptrdy                             [   0:RS_FP-1];
  reg  [        63:0] fp_vs                                [   0:RS_FP-1];
  reg  [        63:0] fp_vt                                [   0:RS_FP-1];
  reg  [ROB_BITS-1:0] fp_rob                               [   0:RS_FP-1];

  reg  [         2:0] fp_cnt;
  wire                fp_full = (fp_cnt >= RS_FP - 1);

  reg                 lsq_v                                [0:LSQ_SIZE-1];
  reg                 lsq_ld                               [0:LSQ_SIZE-1];
  reg                 lsq_st                               [0:LSQ_SIZE-1];
  reg                 lsq_ardy                             [0:LSQ_SIZE-1];
  reg                 lsq_drdy                             [0:LSQ_SIZE-1];
  reg                 lsq_cmt                              [0:LSQ_SIZE-1];
  reg  [        63:0] lsq_base                             [0:LSQ_SIZE-1];
  reg  [        63:0] lsq_data                             [0:LSQ_SIZE-1];
  reg  [        63:0] lsq_imm                              [0:LSQ_SIZE-1];
  reg  [  PHYS_W-1:0] lsq_ps                               [0:LSQ_SIZE-1];
  reg  [  PHYS_W-1:0] lsq_pt                               [0:LSQ_SIZE-1];
  reg  [  PHYS_W-1:0] lsq_pd                               [0:LSQ_SIZE-1];
  reg  [ROB_BITS-1:0] lsq_rob                              [0:LSQ_SIZE-1];
  reg                 lsq_isret                            [0:LSQ_SIZE-1];

  reg [2:0] lsq_head, lsq_tail;
  reg [4:0] lsq_cnt;
  wire lsq_full = (lsq_cnt >= LSQ_SIZE - 2);

  wire lsq_exec = !redirect_en && lsq_cnt > 0 && lsq_v[lsq_head] && lsq_ardy[lsq_head] &&
                  (lsq_ld[lsq_head] ||
                   (lsq_st[lsq_head] && lsq_drdy[lsq_head] && lsq_cmt[lsq_head]));

  // Second LSQ execute slot: only dual-execute two loads
  wire [2:0] lsq_head2 = lsq_head + 3'd1;
  wire lsq_exec2 = !redirect_en && lsq_exec && lsq_cnt > 1 &&
                   lsq_v[lsq_head2] && lsq_ardy[lsq_head2] && lsq_ld[lsq_head2] &&
                   lsq_ld[lsq_head]; // only dual-execute two loads

  // 4-entry decode queue
  reg        dq_v     [0:3];
  reg [31:0] dq_i     [0:3];
  reg [63:0] dq_pc    [0:3];
  reg        dq_ptaken[0:3];
  reg [63:0] dq_ptgt  [0:3];
  reg [2:0]  dq_cnt;  // number of valid entries

  // Aliases for compatibility with dispatch logic
  wire        dq_v0     = dq_v[0];
  wire [31:0] dq_i0     = dq_i[0];
  wire [63:0] dq_pc0    = dq_pc[0];
  wire        dq_ptaken0 = dq_ptaken[0];
  wire [63:0] dq_ptgt0  = dq_ptgt[0];
  wire        dq_v1     = dq_v[1];
  wire [31:0] dq_i1     = dq_i[1];
  wire [63:0] dq_pc1    = dq_pc[1];
  wire        dq_ptaken1 = dq_ptaken[1];
  wire [63:0] dq_ptgt1  = dq_ptgt[1];

  wire [4:0] d0_rs, d0_rt, d0_rd, d0_rtx;
  wire [63:0] d0_imm;
  wire [ 4:0] d0_op;
  wire d0_uimm, d0_wr, d0_ld, d0_st, d0_br, d0_brgt, d0_jmp;
  wire d0_brrr, d0_brri, d0_ret, d0_call, d0_hlt, d0_mvr, d0_mvi;
  wire d0_fp = (d0_op >= 5'd10 && d0_op <= 5'd13);
  wire d0_mem = d0_ld || d0_st;

  wire [4:0] d1_rs, d1_rt, d1_rd, d1_rtx;
  wire [63:0] d1_imm;
  wire [ 4:0] d1_op;
  wire d1_uimm, d1_wr, d1_ld, d1_st, d1_br, d1_brgt, d1_jmp;
  wire d1_brrr, d1_brri, d1_ret, d1_call, d1_hlt, d1_mvr, d1_mvi;
  wire       d1_fp = (d1_op >= 5'd10 && d1_op <= 5'd13);
  wire       d1_mem = d1_ld || d1_st;

  wire       stall = rob_full || rs_full || fp_full || lsq_full;
  wire       d0_en = dq_v0 && !stall;
  wire       d0_ctrl = d0_jmp || d0_br || d0_call || d0_ret;
  wire       d1_en = dq_v1 && !stall && !d0_hlt && !(d0_en && d0_ctrl);

  // ─────────────────── ALU0 pipeline ───────────────────
  reg        alu_en;
  reg  [4:0] alu_op;
  reg [63:0] alu_a, alu_b;
  reg [5:0] alu_rtag;
  reg [PHYS_W-1:0] alu_pd;

  reg [63:0] alu_vs_p, alu_pc_p;
  reg alu_ibr_p, alu_ibgt_p, alu_ijmp_p;
  reg alu_ibrreg_p, alu_ibrimm_p;
  reg alu_imovr_p, alu_imovi_p;
  reg alu_ical_p, alu_iret_p;
  reg        alu_ptaken_p;
  reg [63:0] alu_ptgt_p;

  // ─────────────────── ALU1 pipeline ───────────────────
  reg        alu1_en;
  reg  [4:0] alu1_op;
  reg [63:0] alu1_a, alu1_b;
  reg [5:0]  alu1_rtag;
  reg [PHYS_W-1:0] alu1_pd;
  reg [63:0] alu1_vs_p, alu1_pc_p;
  reg alu1_ibr_p, alu1_ibgt_p, alu1_ijmp_p;
  reg alu1_ibrreg_p, alu1_ibrimm_p;
  reg alu1_imovr_p, alu1_imovi_p, alu1_ical_p, alu1_iret_p;
  reg        alu1_ptaken_p;
  reg [63:0] alu1_ptgt_p, alu1_ibgt_tgt_p;
  reg [PHYS_W-1:0] alu1_pd_p1;
  reg [63:0] alu1_vs_p1, alu1_pc_p1, alu1_b_p1, alu1_ibgt_tgt_p1;
  reg alu1_imovr_p1, alu1_imovi_p1, alu1_ical_p1, alu1_iret_p1;
  reg alu1_ibr_p1, alu1_ijmp_p1, alu1_ibgt_p1, alu1_ibrreg_p1, alu1_ibrimm_p1;
  wire alu1_vout;
  wire [63:0] alu1_res;
  wire [5:0]  alu1_tout;

  // ─────────────────── FPU0 pipeline ───────────────────
  reg        fpu_en;
  reg [ 4:0] fpu_op;
  reg [63:0] fpu_a, fpu_b;
  reg  [       5:0] fpu_rtag;
  reg  [PHYS_W-1:0] fpu_pd;

  reg  [PHYS_W-1:0] fp_pd_p                        [0:2];

  // ─────────────────── FPU1 pipeline ───────────────────
  reg  fpu1_en;
  reg  [4:0] fpu1_op;
  reg [63:0] fpu1_a, fpu1_b;
  reg  [5:0] fpu1_rtag;
  reg [PHYS_W-1:0] fpu1_pd;
  reg [PHYS_W-1:0] fp1_pd_p [0:2];
  wire fpu1_vout;
  wire [63:0] fpu1_res;
  wire [5:0]  fpu1_tout;
  wire fpu1_is_add = (fpu1_op == 5'd10);
  wire fpu1_is_sub = (fpu1_op == 5'd11);
  wire fpu1_is_mul = (fpu1_op == 5'd12);
  wire fpu1_is_div = (fpu1_op == 5'd13);
  reg [2:0] fpu1_sel;
  always @(*) begin
    if (fpu1_is_add) fpu1_sel = 3'd0;
    else if (fpu1_is_sub) fpu1_sel = 3'd1;
    else if (fpu1_is_mul) fpu1_sel = 3'd2;
    else if (fpu1_is_div) fpu1_sel = 3'd3;
    else fpu1_sel = 3'd0;
  end

  // ─────────────────── ALU/FPU outputs ───────────────────
  wire              alu_vout;
  wire [      63:0] alu_res;
  wire [       5:0] alu_tout;

  wire              fpu_vout;
  wire [      63:0] fpu_res;
  wire [       5:0] fpu_tout;

  wire              fpu_is_add = (fpu_op == 5'd10);
  wire              fpu_is_sub = (fpu_op == 5'd11);
  wire              fpu_is_mul = (fpu_op == 5'd12);
  wire              fpu_is_div = (fpu_op == 5'd13);

  reg  [       2:0] fpu_sel;
  always @(*) begin
    if (fpu_is_add) fpu_sel = 3'd0;
    else if (fpu_is_sub) fpu_sel = 3'd1;
    else if (fpu_is_mul) fpu_sel = 3'd2;
    else if (fpu_is_div) fpu_sel = 3'd3;
    else fpu_sel = 3'd0;
  end

  alu alu (
      .clk(clk),
      .reset(reset),
      .valid_in(alu_en),
      .a(alu_a),
      .b(alu_b),
      .op(alu_op),
      .rob_tag_in(alu_rtag),
      .valid_out(alu_vout),
      .result(alu_res),
      .rob_tag_out(alu_tout)
  );

  alu u_alu1 (
      .clk(clk), .reset(reset),
      .valid_in(alu1_en),
      .a(alu1_a), .b(alu1_b), .op(alu1_op),
      .rob_tag_in(alu1_rtag),
      .valid_out(alu1_vout), .result(alu1_res), .rob_tag_out(alu1_tout)
  );

  fpu fpu (
      .clk(clk),
      .reset(reset),
      .valid_in(fpu_en && (fpu_is_add || fpu_is_sub || fpu_is_mul || fpu_is_div)),
      .a(fpu_a),
      .b(fpu_b),
      .op(fpu_sel),
      .rob_tag_in(fpu_rtag),
      .valid_out(fpu_vout),
      .result(fpu_res),
      .rob_tag_out(fpu_tout)
  );

  fpu u_fpu1 (
      .clk(clk), .reset(reset),
      .valid_in(fpu1_en && (fpu1_is_add || fpu1_is_sub || fpu1_is_mul || fpu1_is_div)),
      .a(fpu1_a), .b(fpu1_b), .op(fpu1_sel),
      .rob_tag_in(fpu1_rtag),
      .valid_out(fpu1_vout), .result(fpu1_res), .rob_tag_out(fpu1_tout)
  );

  reg [63:0] pc_reg;
  wire [31:0] mem_i0, mem_i1, mem_i2, mem_i3;
  reg  [63:0] lsq_maddr;
  reg  [63:0] lsq_mwdata;
  reg         lsq_mwe;
  wire [63:0] lsq_mrdata;
  wire [63:0] lsq_maddr2 = lsq_v[lsq_head2] ? (lsq_base[lsq_head2] + lsq_imm[lsq_head2]) : 64'd0;
  wire [63:0] lsq_mrdata2;

  memory #(
      .MEM_SIZE(`MEM_SIZE)
  ) memory (
      .clk(clk),
      .fetch_addr0(pc_reg),
      .fetch_addr1(pc_reg + 64'd4),
      .fetch_addr2(pc_reg + 64'd8),
      .fetch_addr3(pc_reg + 64'd12),
      .instr_out0(mem_i0),
      .instr_out1(mem_i1),
      .instr_out2(mem_i2),
      .instr_out3(mem_i3),
      .data_addr(lsq_maddr),
      .write_data(lsq_mwdata),
      .we(lsq_mwe),
      .read_data(lsq_mrdata),
      .data_addr2(lsq_maddr2),
      .read_data2(lsq_mrdata2)
  );

  wire        bp_pred_taken0, bp_pred_taken1;
  wire [63:0] bp_pred_target0, bp_pred_target1;
  wire        bp_hit0, bp_hit1;

  branch_predictor u_bp0 (
      .clk(clk), .reset(reset),
      .pc(pc_reg),
      .pred_taken(bp_pred_taken0), .pred_target(bp_pred_target0), .btb_hit(bp_hit0),
      .upd_en(bp_upd_en), .upd_pc(bp_upd_pc),
      .upd_taken(bp_upd_taken), .upd_target(bp_upd_target)
  );

  branch_predictor u_bp1 (
      .clk(clk), .reset(reset),
      .pc(pc_reg + 64'd4),
      .pred_taken(bp_pred_taken1), .pred_target(bp_pred_target1), .btb_hit(bp_hit1),
      .upd_en(bp_upd_en), .upd_pc(bp_upd_pc),
      .upd_taken(bp_upd_taken), .upd_target(bp_upd_target)
  );

  reg [63:0] rf_commit_data;
  reg [ 4:0] rf_commit_waddr;
  reg        rf_commit_wen;

  reg_file reg_file (
      .clk(clk),
      .reset(reset),
      .data(rf_commit_data),
      .raddr1(5'd0),
      .raddr2(5'd0),
      .raddr3(5'd0),
      .waddr(rf_commit_waddr),
      .write(rf_commit_wen),
      .r1(),
      .r2(),
      .r3()
  );

  decoder u_d0 (
      .instr(dq_i0),
      .raddr1(d0_rs),
      .raddr2(d0_rt),
      .waddr(d0_rd),
      .immediate(d0_imm),
      .op(d0_op),
      .use_imm(d0_uimm),
      .write(d0_wr),
      .is_load(d0_ld),
      .is_store(d0_st),
      .is_branch(d0_br),
      .is_brgt(d0_brgt),
      .is_jump(d0_jmp),
      .is_brr_reg(d0_brrr),
      .is_brr_imm(d0_brri),
      .is_return(d0_ret),
      .is_call(d0_call),
      .is_halt(d0_hlt),
      .is_mov_reg(d0_mvr),
      .is_mov_imm(d0_mvi),
      .rt_addr(d0_rtx)
  );

  decoder u_d1 (
      .instr(dq_i1),
      .raddr1(d1_rs),
      .raddr2(d1_rt),
      .waddr(d1_rd),
      .immediate(d1_imm),
      .op(d1_op),
      .use_imm(d1_uimm),
      .write(d1_wr),
      .is_load(d1_ld),
      .is_store(d1_st),
      .is_branch(d1_br),
      .is_brgt(d1_brgt),
      .is_jump(d1_jmp),
      .is_brr_reg(d1_brrr),
      .is_brr_imm(d1_brri),
      .is_return(d1_ret),
      .is_call(d1_call),
      .is_halt(d1_hlt),
      .is_mov_reg(d1_mvr),
      .is_mov_imm(d1_mvi),
      .rt_addr(d1_rtx)
  );

  // ─────────────────── CDB0 = ALU0 result ───────────────────
  wire                c0en = alu_vout;
  wire [  PHYS_W-1:0] c0pd;
  wire [        63:0] c0val;
  wire [ROB_BITS-1:0] c0rob = alu_tout[ROB_BITS-1:0];

  reg  [  PHYS_W-1:0] alu_pd_p1;
  reg  [        63:0] alu_vs_p1;
  reg alu_imovr_p1, alu_imovi_p1;
  reg alu_ical_p1, alu_iret_p1;
  reg alu_ibr_p1, alu_ijmp_p1;
  reg alu_ibgt_p1;  // FIX1: registered brgt flag
  reg alu_ibrreg_p1, alu_ibrimm_p1;
  reg [63:0] alu_pc_p1;
  reg [63:0] alu_b_p1;
  reg [63:0] alu_ibgt_tgt_p;
  reg [63:0] alu_ibgt_tgt_p1;

  always @(posedge clk) begin
    alu_pd_p1       <= alu_pd;
    alu_vs_p1       <= alu_a;
    alu_imovr_p1    <= alu_imovr_p;
    alu_imovi_p1    <= alu_imovi_p;
    alu_ical_p1     <= alu_ical_p;
    alu_iret_p1     <= alu_iret_p;
    alu_ibr_p1      <= alu_ibr_p;
    alu_ijmp_p1     <= alu_ijmp_p;
    alu_ibgt_p1     <= alu_ibgt_p;  // FIX1
    alu_ibrreg_p1   <= alu_ibrreg_p;
    alu_ibrimm_p1   <= alu_ibrimm_p;
    alu_pc_p1       <= alu_pc_p;
    alu_b_p1        <= alu_b;
    alu_ibgt_tgt_p1 <= alu_ibgt_tgt_p;
    // ALU1 stage-1 pipeline registers
    alu1_pd_p1       <= alu1_pd;
    alu1_vs_p1       <= alu1_a;
    alu1_imovr_p1    <= alu1_imovr_p;
    alu1_imovi_p1    <= alu1_imovi_p;
    alu1_ical_p1     <= alu1_ical_p;
    alu1_iret_p1     <= alu1_iret_p;
    alu1_ibr_p1      <= alu1_ibr_p;
    alu1_ijmp_p1     <= alu1_ijmp_p;
    alu1_ibgt_p1     <= alu1_ibgt_p;
    alu1_ibrreg_p1   <= alu1_ibrreg_p;
    alu1_ibrimm_p1   <= alu1_ibrimm_p;
    alu1_pc_p1       <= alu1_pc_p;
    alu1_b_p1        <= alu1_b;
    alu1_ibgt_tgt_p1 <= alu1_ibgt_tgt_p;
  end

  assign c0pd = alu_pd_p1;
  assign c0val = alu_imovr_p1 ? alu_vs_p1
               : alu_ical_p1  ? (alu_pc_p1 + 64'd4)
               : alu_imovi_p1 ? ((alu_vs_p1 & ~64'hFFF) | alu_b_p1)
               : alu_res;

  wire alu_act_taken_w = alu_ibr_p1 ? alu_res[0] : alu_ijmp_p1 ? 1'b1 : 1'b0;

  // FIX1: Use alu_ibgt_p1 flag instead of "!= 64'd0" value check.
  wire [63:0] alu_act_tgt_w =
      alu_ibrimm_p1 ? (alu_pc_p1 + alu_b_p1)
    : alu_ibrreg_p1 ? (alu_pc_p1 + alu_vs_p1)
    : alu_ibgt_p1   ? alu_ibgt_tgt_p1           // FIX1: explicit flag
  : alu_ibr_p1 ? alu_b_p1 : alu_vs_p1;

  // ─────────────────── CDB2 = ALU1 result ───────────────────
  wire [PHYS_W-1:0] c2pd;
  wire [63:0] c2val;
  wire c2en = alu1_vout;
  wire [ROB_BITS-1:0] c2rob = alu1_tout[ROB_BITS-1:0];

  assign c2pd = alu1_pd_p1;
  assign c2val = alu1_imovr_p1 ? alu1_vs_p1
               : alu1_ical_p1  ? (alu1_pc_p1 + 64'd4)
               : alu1_imovi_p1 ? ((alu1_vs_p1 & ~64'hFFF) | alu1_b_p1)
               : alu1_res;

  wire alu1_act_taken_w = alu1_ibr_p1 ? alu1_res[0] : alu1_ijmp_p1 ? 1'b1 : 1'b0;
  wire [63:0] alu1_act_tgt_w =
      alu1_ibrimm_p1 ? (alu1_pc_p1 + alu1_b_p1)
    : alu1_ibrreg_p1 ? (alu1_pc_p1 + alu1_vs_p1)
    : alu1_ibgt_p1   ? alu1_ibgt_tgt_p1
    : alu1_ibr_p1    ? alu1_b_p1 : alu1_vs_p1;

  // ─────────────────── CDB1 = LD0 or FPU0 ───────────────────
  wire [PHYS_W-1:0] fp_pd = fp_pd_p[2];

  reg ld_done;
  reg [PHYS_W-1:0] ld_pd;
  reg [63:0] ld_val;
  reg [ROB_BITS-1:0] ld_rtag;
  reg ld_isret;

  wire c1en = ld_done || fpu_vout;
  wire [PHYS_W-1:0] c1pd = ld_done ? ld_pd : fp_pd;
  wire [63:0] c1val = ld_done ? ld_val : fpu_res;
  wire [ROB_BITS-1:0] c1rob = ld_done ? ld_rtag : fpu_tout[ROB_BITS-1:0];

  // ─────────────────── CDB3 = LD1 or FPU1 ───────────────────
  wire [PHYS_W-1:0] fp1_pd = fp1_pd_p[2];

  reg        ld1_done;
  reg [PHYS_W-1:0] ld1_pd;
  reg [63:0] ld1_val;
  reg [ROB_BITS-1:0] ld1_rtag;
  reg ld1_isret;

  wire c3en = ld1_done || fpu1_vout;
  wire [PHYS_W-1:0] c3pd = ld1_done ? ld1_pd : fp1_pd;
  wire [63:0] c3val = ld1_done ? ld1_val : fpu1_res;
  wire [ROB_BITS-1:0] c3rob = ld1_done ? ld1_rtag : fpu1_tout[ROB_BITS-1:0];

  wire [63:0] lsq_h_addr = lsq_base[lsq_head] + lsq_imm[lsq_head];

  // Store-to-load forwarding for lsq_head
  integer sf;
  reg fwd_hit;
  reg [63:0] fwd_val;
  always @(*) begin
    fwd_hit = 0;
    fwd_val = 64'd0;
    for (sf = 0; sf < LSQ_SIZE; sf = sf + 1)
    if (lsq_v[sf] && lsq_st[sf] && lsq_ardy[sf] && lsq_drdy[sf] &&
          (lsq_base[sf] + lsq_imm[sf] == lsq_h_addr)) begin
      fwd_hit = 1;
      fwd_val = lsq_data[sf];
    end
  end

  // Store-to-load forwarding for lsq_head2
  integer sf2;
  reg fwd_hit2;
  reg [63:0] fwd_val2;
  always @(*) begin
    fwd_hit2 = 0;
    fwd_val2 = 64'd0;
    for (sf2 = 0; sf2 < LSQ_SIZE; sf2 = sf2 + 1)
    if (lsq_v[sf2] && lsq_st[sf2] && lsq_ardy[sf2] && lsq_drdy[sf2] &&
          (lsq_base[sf2] + lsq_imm[sf2] == lsq_maddr2)) begin
      fwd_hit2 = 1;
      fwd_val2 = lsq_data[sf2];
    end
  end

  reg        redirect_en;
  reg [63:0] redirect_pc;
  reg        flush_this_cycle;
  reg        bp_upd_en;
  reg [63:0] bp_upd_pc;
  reg        bp_upd_taken;
  reg [63:0] bp_upd_target;
  reg        commit_happened;
  reg        commit_freed_reg;

  reg        call_pending;
  reg [63:0] call_tgt;
  reg [63:0] call_addr;
  reg [63:0] call_wdata;
  reg        ret_pending;
  reg [63:0] ret_addr;

  integer i, j;

  integer rs_free_slot, fp_free_slot;
  always @(*) begin
    rs_free_slot = 0;
    for (i = RS_INT - 1; i >= 0; i = i - 1) if (!rs_v[i]) rs_free_slot = i;
    fp_free_slot = 0;
    for (i = RS_FP - 1; i >= 0; i = i - 1) if (!fp_v[i]) fp_free_slot = i;
  end

  // RS issue slot 0
  integer rs_iss_idx;
  reg     rs_iss_found;
  always @(*) begin
    rs_iss_idx   = 0;
    rs_iss_found = 0;
    for (i = 0; i < RS_INT; i = i + 1)
    if (!rs_iss_found && rs_v[i] && rs_psrdy[i] && (rs_uimm[i] || rs_ptrdy[i]) &&
        (!rs_ibgt[i] || rs_pt3rdy[i])) begin
      rs_iss_idx   = i;
      rs_iss_found = 1;
    end
  end

  // RS issue slot 1 (second ready instruction, different from slot 0)
  integer rs_iss_idx2;
  reg     rs_iss_found2;
  always @(*) begin
    rs_iss_idx2   = 0;
    rs_iss_found2 = 0;
    if (rs_iss_found) begin
      for (i = 0; i < RS_INT; i = i + 1)
      if (!rs_iss_found2 && rs_v[i] && rs_psrdy[i] && (rs_uimm[i] || rs_ptrdy[i]) &&
          (!rs_ibgt[i] || rs_pt3rdy[i]) && i != rs_iss_idx) begin
        rs_iss_idx2   = i;
        rs_iss_found2 = 1;
      end
    end
  end

  // FP issue slot 0
  integer fp_iss_idx;
  reg     fp_iss_found;
  always @(*) begin
    fp_iss_idx   = 0;
    fp_iss_found = 0;
    for (i = 0; i < RS_FP; i = i + 1)
    if (!fp_iss_found && fp_v[i] && fp_psrdy[i] && fp_ptrdy[i]) begin
      fp_iss_idx   = i;
      fp_iss_found = 1;
    end
  end

  // FP issue slot 1
  integer fp_iss_idx2;
  reg     fp_iss_found2;
  always @(*) begin
    fp_iss_idx2   = 0;
    fp_iss_found2 = 0;
    if (fp_iss_found) begin
      for (i = 0; i < RS_FP; i = i + 1)
      if (!fp_iss_found2 && fp_v[i] && fp_psrdy[i] && fp_ptrdy[i] && i != fp_iss_idx) begin
        fp_iss_idx2   = i;
        fp_iss_found2 = 1;
      end
    end
  end

  // FIX2: Gate store writes on !redirect_en to prevent wrong-path stores
  always @(*) begin
    if (call_pending) begin
      lsq_mwe    = 1;
      lsq_maddr  = call_addr;
      lsq_mwdata = call_wdata;
    end else if (ret_pending) begin
      lsq_mwe    = 0;
      lsq_maddr  = ret_addr;
      lsq_mwdata = 64'd0;
    end else begin
      lsq_mwe    = 0;
      lsq_maddr  = lsq_h_addr;
      lsq_mwdata = lsq_data[lsq_head];
      // FIX2: !redirect_en prevents committed LSQ stores from writing during flush
      if (!redirect_en && lsq_cnt > 0 && lsq_v[lsq_head] && lsq_ardy[lsq_head] &&
          lsq_st[lsq_head] && lsq_drdy[lsq_head] && lsq_cmt[lsq_head])
        lsq_mwe = 1;
    end
  end

  always @(posedge clk) begin
    if (reset) begin
      hlt    <= 0;
      pc_reg <= `PC_START;
      flush_this_cycle = 0;
      dq_v[0]      <= 0;
      dq_v[1]      <= 0;
      dq_v[2]      <= 0;
      dq_v[3]      <= 0;
      dq_cnt       <= 0;
      redirect_en  <= 0;
      alu_en       <= 0;
      fpu_en       <= 0;
      ld_done      <= 0;
      call_pending <= 0;
      ret_pending  <= 0;
      rob_head     <= 0;
      rob_tail     <= 0;
      rob_cnt      <= 0;
      lsq_head     <= 0;
      lsq_tail     <= 0;
      lsq_cnt      <= 0;
      rs_cnt       <= 0;
      fp_cnt       <= 0;
      fl_head      <= 0;
      fl_tail      <= 32;
      fl_cnt       <= 32;

      for (i = 0; i < 32; i = i + 1) begin
        arch_rf[i] = 64'd0;
        rat_map[i] = i[PHYS_W-1:0];
        prf[i]     = 64'd0;
        prf_rdy[i] = 1;
      end
      arch_rf[31] = 64'd524288;
      prf[31]     = 64'd524288;

      for (i = 32; i < NPHYS; i = i + 1) begin
        prf[i] = 0;
        prf_rdy[i] = 1;
      end
      for (i = 0; i < 32; i = i + 1) free_list[i] = 6'(32 + i);

      for (i = 0; i < ROB_SIZE; i = i + 1) begin
        rob_valid[i] = 0;
        rob_done[i]  = 0;
      end
      for (i = 0; i < RS_INT; i = i + 1) rs_v[i] = 0;
      for (i = 0; i < RS_FP; i = i + 1) fp_v[i] = 0;
      for (i = 0; i < LSQ_SIZE; i = i + 1) begin
        lsq_v[i] = 0;
        lsq_isret[i] = 0;
      end

      alu_pd_p1       <= 0;
      alu_vs_p1       <= 0;
      alu_imovr_p1    <= 0;
      alu_imovi_p1    <= 0;
      alu_ical_p1     <= 0;
      alu_iret_p1     <= 0;
      alu_ibr_p1      <= 0;
      alu_ijmp_p1     <= 0;
      alu_ibgt_p1     <= 0;
      alu_ibrreg_p1   <= 0;
      alu_ibrimm_p1   <= 0;
      alu_pc_p1       <= 0;
      alu_b_p1        <= 0;
      fp_pd_p[0]      <= 0;
      fp_pd_p[1]      <= 0;
      fp_pd_p[2]      <= 0;
      alu_vs_p        <= 0;
      alu_pc_p        <= 0;
      alu_ibr_p       <= 0;
      alu_ibgt_p      <= 0;
      alu_ijmp_p      <= 0;
      alu_ibrreg_p    <= 0;
      alu_ibrimm_p    <= 0;
      alu_imovr_p     <= 0;
      alu_imovi_p     <= 0;
      alu_ical_p      <= 0;
      alu_iret_p      <= 0;
      alu_ptaken_p    <= 0;
      alu_ptgt_p      <= 0;
      alu_ibgt_tgt_p  <= 0;
      alu_ibgt_tgt_p1 <= 0;
      rf_commit_wen   <= 0;
      rf_commit_waddr <= 0;
      rf_commit_data  <= 0;
      bp_upd_en       <= 0;
      bp_upd_pc       <= 0;
      bp_upd_taken    <= 0;
      bp_upd_target   <= 0;
      dq_ptaken[0]    <= 0;
      dq_ptaken[1]    <= 0;
      dq_ptaken[2]    <= 0;
      dq_ptaken[3]    <= 0;
      dq_ptgt[0]      <= 0;
      dq_ptgt[1]      <= 0;
      dq_ptgt[2]      <= 0;
      dq_ptgt[3]      <= 0;

      // EXT1 resets
      alu1_en <= 0; fpu1_en <= 0; ld1_done <= 0; ld1_isret <= 0;
      alu1_pd_p1 <= 0; alu1_vs_p1 <= 0;
      alu1_imovr_p1 <= 0; alu1_imovi_p1 <= 0; alu1_ical_p1 <= 0; alu1_iret_p1 <= 0;
      alu1_ibr_p1 <= 0; alu1_ijmp_p1 <= 0; alu1_ibgt_p1 <= 0;
      alu1_ibrreg_p1 <= 0; alu1_ibrimm_p1 <= 0;
      alu1_pc_p1 <= 0; alu1_b_p1 <= 0; alu1_ibgt_tgt_p1 <= 0;
      alu1_vs_p <= 0; alu1_pc_p <= 0; alu1_ibr_p <= 0; alu1_ibgt_p <= 0;
      alu1_ijmp_p <= 0; alu1_ibrreg_p <= 0; alu1_ibrimm_p <= 0;
      alu1_imovr_p <= 0; alu1_imovi_p <= 0; alu1_ical_p <= 0; alu1_iret_p <= 0;
      alu1_ptaken_p <= 0; alu1_ptgt_p <= 0; alu1_ibgt_tgt_p <= 0;
      fp1_pd_p[0] <= 0; fp1_pd_p[1] <= 0; fp1_pd_p[2] <= 0;

    end else begin

      flush_this_cycle = 0;
      commit_happened  = 0;
      commit_freed_reg = 0;
      redirect_en   <= 0;
      bp_upd_en     <= 0;
      alu_en        <= 0;
      fpu_en        <= 0;
      ld_done       <= 0;
      ld_isret      <= 0;
      rf_commit_wen <= 0;
      call_pending  <= 0;
      ret_pending   <= 0;
      // EXT1 defaults
      alu1_en <= 0; fpu1_en <= 0; ld1_done <= 0; ld1_isret <= 0;

      if (call_pending) begin
        redirect_en <= 1;
        redirect_pc <= call_tgt;
      end
      if (ret_pending) begin
        redirect_en <= 1;
        redirect_pc <= lsq_mrdata;
      end

      if (!redirect_en) begin
        for (i = 0; i < 32; i = i + 1) begin
          if (rat_map[i] == i[PHYS_W-1:0] && prf_rdy[i]) begin
            prf[i]    <= reg_file.registers[i];
            arch_rf[i] <= reg_file.registers[i];
          end
        end
      end

      // ═══════════════════════════════════════════════
      // A. CDB BROADCAST (4 ports: c0=ALU0, c1=LD0/FPU0, c2=ALU1, c3=LD1/FPU1)
      // ═══════════════════════════════════════════════

      // ─── CDB0: ALU0 ───
      if (c0en && rob_valid[c0rob]) begin
        prf[c0pd]         <= c0val;
        prf_rdy[c0pd]     <= 1;
        rob_done[c0rob]   <= 1;
        rob_result[c0rob] <= c0val;
        if (alu_ibr_p1 || alu_ijmp_p1) begin
          rob_act_taken[c0rob] <= alu_act_taken_w;
          rob_act_tgt[c0rob]   <= alu_act_tgt_w;
        end
        for (i = 0; i < RS_INT; i = i + 1)
        if (rs_v[i]) begin
          if (!rs_psrdy[i] && rs_ps[i] == c0pd) begin
            rs_psrdy[i] <= 1;
            rs_vs[i] <= c0val;
          end
          if (!rs_ptrdy[i] && rs_pt[i] == c0pd) begin
            rs_ptrdy[i] <= 1;
            rs_vt[i] <= c0val;
          end
          if (rs_ibgt[i] && !rs_pt3rdy[i] && rs_pt3[i] == c0pd) begin
            rs_pt3rdy[i] <= 1;
            rs_imm[i]    <= c0val;
          end
        end
        for (i = 0; i < RS_FP; i = i + 1)
        if (fp_v[i]) begin
          if (!fp_psrdy[i] && fp_ps[i] == c0pd) begin
            fp_psrdy[i] <= 1;
            fp_vs[i] <= c0val;
          end
          if (!fp_ptrdy[i] && fp_pt[i] == c0pd) begin
            fp_ptrdy[i] <= 1;
            fp_vt[i] <= c0val;
          end
        end
        for (i = 0; i < LSQ_SIZE; i = i + 1)
        if (lsq_v[i]) begin
          if (!lsq_ardy[i] && lsq_ps[i] == c0pd) begin
            lsq_ardy[i] <= 1;
            lsq_base[i] <= c0val;
          end
          if (!lsq_drdy[i] && lsq_pt[i] == c0pd) begin
            lsq_drdy[i] <= 1;
            lsq_data[i] <= c0val;
          end
        end
      end

      // ─── CDB1: LD0 / FPU0 ───
      if (c1en && rob_valid[c1rob]) begin
        prf[c1pd]         <= c1val;
        prf_rdy[c1pd]     <= 1;
        rob_done[c1rob]   <= 1;
        rob_result[c1rob] <= c1val;
        if (ld_done && ld_isret) begin
          rob_act_taken[c1rob] <= 1'b1;
          rob_act_tgt[c1rob]   <= ld_val;
        end
        for (i = 0; i < RS_INT; i = i + 1)
        if (rs_v[i]) begin
          if (!rs_psrdy[i] && rs_ps[i] == c1pd) begin
            rs_psrdy[i] <= 1;
            rs_vs[i] <= c1val;
          end
          if (!rs_ptrdy[i] && rs_pt[i] == c1pd) begin
            rs_ptrdy[i] <= 1;
            rs_vt[i] <= c1val;
          end
          if (rs_ibgt[i] && !rs_pt3rdy[i] && rs_pt3[i] == c1pd) begin
            rs_pt3rdy[i] <= 1;
            rs_imm[i]    <= c1val;
          end
        end
        for (i = 0; i < RS_FP; i = i + 1)
        if (fp_v[i]) begin
          if (!fp_psrdy[i] && fp_ps[i] == c1pd) begin
            fp_psrdy[i] <= 1;
            fp_vs[i] <= c1val;
          end
          if (!fp_ptrdy[i] && fp_pt[i] == c1pd) begin
            fp_ptrdy[i] <= 1;
            fp_vt[i] <= c1val;
          end
        end
        for (i = 0; i < LSQ_SIZE; i = i + 1)
        if (lsq_v[i]) begin
          if (!lsq_ardy[i] && lsq_ps[i] == c1pd) begin
            lsq_ardy[i] <= 1;
            lsq_base[i] <= c1val;
          end
          if (!lsq_drdy[i] && lsq_pt[i] == c1pd) begin
            lsq_drdy[i] <= 1;
            lsq_data[i] <= c1val;
          end
        end
      end

      // ─── CDB2: ALU1 ───
      if (c2en && rob_valid[c2rob]) begin
        prf[c2pd]         <= c2val;
        prf_rdy[c2pd]     <= 1;
        rob_done[c2rob]   <= 1;
        rob_result[c2rob] <= c2val;
        if (alu1_ibr_p1 || alu1_ijmp_p1) begin
          rob_act_taken[c2rob] <= alu1_act_taken_w;
          rob_act_tgt[c2rob]   <= alu1_act_tgt_w;
        end
        for (i = 0; i < RS_INT; i = i + 1)
        if (rs_v[i]) begin
          if (!rs_psrdy[i] && rs_ps[i] == c2pd) begin
            rs_psrdy[i] <= 1;
            rs_vs[i] <= c2val;
          end
          if (!rs_ptrdy[i] && rs_pt[i] == c2pd) begin
            rs_ptrdy[i] <= 1;
            rs_vt[i] <= c2val;
          end
          if (rs_ibgt[i] && !rs_pt3rdy[i] && rs_pt3[i] == c2pd) begin
            rs_pt3rdy[i] <= 1;
            rs_imm[i]    <= c2val;
          end
        end
        for (i = 0; i < RS_FP; i = i + 1)
        if (fp_v[i]) begin
          if (!fp_psrdy[i] && fp_ps[i] == c2pd) begin
            fp_psrdy[i] <= 1;
            fp_vs[i] <= c2val;
          end
          if (!fp_ptrdy[i] && fp_pt[i] == c2pd) begin
            fp_ptrdy[i] <= 1;
            fp_vt[i] <= c2val;
          end
        end
        for (i = 0; i < LSQ_SIZE; i = i + 1)
        if (lsq_v[i]) begin
          if (!lsq_ardy[i] && lsq_ps[i] == c2pd) begin
            lsq_ardy[i] <= 1;
            lsq_base[i] <= c2val;
          end
          if (!lsq_drdy[i] && lsq_pt[i] == c2pd) begin
            lsq_drdy[i] <= 1;
            lsq_data[i] <= c2val;
          end
        end
      end

      // ─── CDB3: LD1 / FPU1 ───
      if (c3en && rob_valid[c3rob]) begin
        prf[c3pd]         <= c3val;
        prf_rdy[c3pd]     <= 1;
        rob_done[c3rob]   <= 1;
        rob_result[c3rob] <= c3val;
        if (ld1_done && ld1_isret) begin
          rob_act_taken[c3rob] <= 1'b1;
          rob_act_tgt[c3rob]   <= ld1_val;
        end
        for (i = 0; i < RS_INT; i = i + 1)
        if (rs_v[i]) begin
          if (!rs_psrdy[i] && rs_ps[i] == c3pd) begin
            rs_psrdy[i] <= 1;
            rs_vs[i] <= c3val;
          end
          if (!rs_ptrdy[i] && rs_pt[i] == c3pd) begin
            rs_ptrdy[i] <= 1;
            rs_vt[i] <= c3val;
          end
          if (rs_ibgt[i] && !rs_pt3rdy[i] && rs_pt3[i] == c3pd) begin
            rs_pt3rdy[i] <= 1;
            rs_imm[i]    <= c3val;
          end
        end
        for (i = 0; i < RS_FP; i = i + 1)
        if (fp_v[i]) begin
          if (!fp_psrdy[i] && fp_ps[i] == c3pd) begin
            fp_psrdy[i] <= 1;
            fp_vs[i] <= c3val;
          end
          if (!fp_ptrdy[i] && fp_pt[i] == c3pd) begin
            fp_ptrdy[i] <= 1;
            fp_vt[i] <= c3val;
          end
        end
        for (i = 0; i < LSQ_SIZE; i = i + 1)
        if (lsq_v[i]) begin
          if (!lsq_ardy[i] && lsq_ps[i] == c3pd) begin
            lsq_ardy[i] <= 1;
            lsq_base[i] <= c3val;
          end
          if (!lsq_drdy[i] && lsq_pt[i] == c3pd) begin
            lsq_drdy[i] <= 1;
            lsq_data[i] <= c3val;
          end
        end
      end

      // ═══════════════════════════════════════════════
      // B. ROB COMMIT
      // ═══════════════════════════════════════════════
      begin : commit_blk
        reg                do_flush;
        reg [ROB_BITS-1:0] ch;
        do_flush = 0;
        ch = rob_head;

        if (rob_cnt > 0 && rob_valid[ch] && rob_done[ch]) begin
          if (rob_has_dest[ch]) begin
            arch_rf[rob_arch[ch]] <= rob_result[ch];
            prf[rob_phys[ch]]     <= rob_result[ch];
            rf_commit_data        <= rob_result[ch];
            rf_commit_waddr       <= rob_arch[ch];
            rf_commit_wen         <= 1;
            if (rob_arch[ch] == 5'd0) reg_file.registers[0] <= rob_result[ch];
          end

          if (rob_is_halt[ch]) hlt <= 1;

          if (rob_has_dest[ch]) begin
            free_list[fl_tail] <= rob_old[ch];
            fl_tail            <= fl_tail + 1;
            fl_cnt             <= fl_cnt + 1;
          end

          if (rob_is_store[ch])
            for (i = 0; i < LSQ_SIZE; i = i + 1)
            if (lsq_v[i] && lsq_st[i] && lsq_rob[i] == ch) lsq_cmt[i] <= 1;

          if (rob_is_branch[ch] || rob_is_jump[ch]) begin
            bp_upd_en     <= 1;
            bp_upd_pc     <= rob_pc[ch];
            bp_upd_taken  <= rob_act_taken[ch];
            bp_upd_target <= rob_act_tgt[ch];
            if (rob_pred_taken[ch] != rob_act_taken[ch] ||
                (rob_act_taken[ch] && rob_pred_tgt[ch] != rob_act_tgt[ch])) begin
              do_flush         = 1;
              flush_this_cycle = 1;
              redirect_en <= 1;
              redirect_pc <= rob_act_taken[ch] ? rob_act_tgt[ch] : (rob_pc[ch] + 64'd4);
            end
          end

          rob_valid[ch] <= 0;
          rob_done[ch]  <= 0;
          rob_head      <= ch + 1;
          rob_cnt       <= rob_cnt - 1;
          commit_happened  = 1;
          commit_freed_reg = rob_has_dest[ch];

          if (do_flush) begin
            for (i = 0; i < RS_INT; i = i + 1) rs_v[i] <= 0;
            for (i = 0; i < RS_FP; i = i + 1) fp_v[i] <= 0;
            for (i = 0; i < LSQ_SIZE; i = i + 1) begin
              lsq_v[i] <= 0;
              lsq_isret[i] <= 0;
            end
            for (i = 0; i < ROB_SIZE; i = i + 1)
            if (i[ROB_BITS-1:0] != ch) begin
              rob_valid[i] <= 0;
              rob_done[i]  <= 0;
            end
            for (i = 0; i < 32; i = i + 1) begin
              rat_map[i] <= i[PHYS_W-1:0];
              prf[i]     <= arch_rf[i];
              prf_rdy[i] <= 1;
            end
            prf[31] <= arch_rf[31];
            for (i = 32; i < NPHYS; i = i + 1) prf_rdy[i] <= 1;
            for (i = 0; i < 32; i = i + 1) free_list[i] <= 6'(32 + i);
            fl_head  <= 0;
            fl_tail  <= 32;
            fl_cnt   <= 32;
            rob_tail <= ch + 1;
            rob_cnt  <= 0;
            rs_cnt   <= 0;
            fp_cnt   <= 0;
            lsq_head <= 0;
            lsq_tail <= 0;
            lsq_cnt  <= 0;
            dq_v[0]  <= 0;
            dq_v[1]  <= 0;
            dq_v[2]  <= 0;
            dq_v[3]  <= 0;
            dq_cnt   <= 0;
          end
        end
      end  // commit_blk

      // ═══════════════════════════════════════════════
      // C. LSQ EXECUTE
      // FIX3: Block during flush to prevent stale loads/stores firing
      // ═══════════════════════════════════════════════
      if (!redirect_en && !flush_this_cycle &&
          lsq_cnt > 0 && lsq_v[lsq_head] && lsq_ardy[lsq_head]) begin
        if (lsq_ld[lsq_head]) begin
          ld_done         <= 1;
          ld_pd           <= lsq_pd[lsq_head];
          ld_rtag         <= lsq_rob[lsq_head];
          ld_val          <= fwd_hit ? fwd_val : lsq_mrdata;
          ld_isret        <= lsq_isret[lsq_head];
          lsq_v[lsq_head] <= 0;
          lsq_head        <= lsq_exec2 ? (lsq_head + 2) : (lsq_head + 1);
          // Second load (lsq_head2) — only if also a load
          if (lsq_exec2) begin
            ld1_done          <= 1;
            ld1_pd            <= lsq_pd[lsq_head2];
            ld1_rtag          <= lsq_rob[lsq_head2];
            ld1_val           <= fwd_hit2 ? fwd_val2 : lsq_mrdata2;
            ld1_isret         <= lsq_isret[lsq_head2];
            lsq_v[lsq_head2] <= 0;
          end
        end else if (lsq_st[lsq_head] && lsq_drdy[lsq_head] && lsq_cmt[lsq_head]) begin
          lsq_v[lsq_head] <= 0;
          lsq_head        <= lsq_head + 1;
        end
      end

      // ═══════════════════════════════════════════════
      // DISPATCH BLOCK
      // ═══════════════════════════════════════════════
      if (!redirect_en && !call_pending && !ret_pending && !flush_this_cycle) begin : dispatch_blk
        reg [PHYS_W-1:0] p0_new, p0_old, p0_ps, p0_pt;
        reg [63:0] p0_vs, p0_vt;
        reg p0_psrdy, p0_ptrdy;
        reg [ROB_BITS-1:0] p0_rob;
        reg [  PHYS_W-1:0] p0_r31_phys;
        reg [        63:0] p0_r31_val;
        reg                p0_r31_rdy;

        reg [PHYS_W-1:0] p1_new, p1_old, p1_ps, p1_pt;
        reg [63:0] p1_vs, p1_vt;
        reg p1_psrdy, p1_ptrdy;
        reg [ROB_BITS-1:0] p1_rob;
        reg [  PHYS_W-1:0] p1_r31_phys;
        reg [        63:0] p1_r31_val;
        reg                p1_r31_rdy;

        reg [         3:0] rs_d0_slot;
        reg [         2:0] fp_d0_slot;
        reg d0_used_rs, d0_used_fp;

        reg [         5:0] fh;
        reg [         6:0] fc;
        reg [ROB_BITS-1:0] rt;
        reg [  ROB_BITS:0] rc;
        reg [         3:0] rsc;
        reg [         2:0] fpc;
        reg [         4:0] lc;
        reg [         2:0] lt;

        fh = fl_head;
        fc = fl_cnt;
        rt = rob_tail;
        rc = rob_cnt;
        rsc = rs_cnt;
        fpc = fp_cnt;
        lc = lsq_cnt;
        lt = lsq_tail;
        rs_d0_slot = 0;
        d0_used_rs = 0;
        fp_d0_slot = 0;
        d0_used_fp = 0;

        if (d0_en) begin
          p0_ps    = rat_map[d0_rs];
          p0_pt    = rat_map[d0_rt];
          p0_vs    = prf[p0_ps];
          p0_vt    = prf[p0_pt];
          p0_psrdy = prf_rdy[p0_ps];
          p0_ptrdy = prf_rdy[p0_pt];
          // same-cycle CDB forwarding: CDB fires this cycle but prf_rdy not yet updated
          if (c0en) begin
            if (!p0_psrdy && p0_ps == c0pd) begin p0_psrdy = 1; p0_vs = c0val; end
            if (!p0_ptrdy && p0_pt == c0pd) begin p0_ptrdy = 1; p0_vt = c0val; end
          end
          if (c1en) begin
            if (!p0_psrdy && p0_ps == c1pd) begin p0_psrdy = 1; p0_vs = c1val; end
            if (!p0_ptrdy && p0_pt == c1pd) begin p0_ptrdy = 1; p0_vt = c1val; end
          end
          if (c2en) begin
            if (!p0_psrdy && p0_ps == c2pd) begin p0_psrdy = 1; p0_vs = c2val; end
            if (!p0_ptrdy && p0_pt == c2pd) begin p0_ptrdy = 1; p0_vt = c2val; end
          end
          if (c3en) begin
            if (!p0_psrdy && p0_ps == c3pd) begin p0_psrdy = 1; p0_vs = c3val; end
            if (!p0_ptrdy && p0_pt == c3pd) begin p0_ptrdy = 1; p0_vt = c3val; end
          end
          p0_r31_phys = rat_map[5'd31];
          p0_r31_val  = prf[p0_r31_phys];
          p0_r31_rdy  = prf_rdy[p0_r31_phys];

          if (d0_wr && !d0_call && fc > 0) begin
            p0_new = free_list[fh];
            p0_old = rat_map[d0_rd];
            fh     = fh + 1;
            fc     = fc - 1;
            rat_map[d0_rd]  <= p0_new;
            prf_rdy[p0_new] <= 0;
          end else begin
            p0_new = rat_map[d0_rd];
            p0_old = rat_map[d0_rd];
          end

          p0_rob = rt;
          rt = rt + 1;
          rc = rc + 1;
          rob_valid[p0_rob]      <= 1;
          rob_done[p0_rob]       <= (!d0_wr && !d0_br && !d0_jmp) ? 1 : 0;
          rob_arch[p0_rob]       <= d0_rd;
          rob_phys[p0_rob]       <= p0_new;
          rob_old[p0_rob]        <= p0_old;
          rob_has_dest[p0_rob]   <= d0_wr;
          rob_is_store[p0_rob]   <= d0_st;
          rob_is_halt[p0_rob]    <= d0_hlt;
          rob_is_branch[p0_rob]  <= d0_br;
          rob_is_jump[p0_rob]    <= d0_jmp;
          rob_pc[p0_rob]         <= dq_pc0;
          rob_pred_taken[p0_rob] <= dq_ptaken0;
          rob_pred_tgt[p0_rob]   <= dq_ptaken0 ? dq_ptgt0 : (dq_pc0 + 64'd4);
          rob_act_taken[p0_rob]  <= 0;
          rob_act_tgt[p0_rob]    <= 0;

          if (d0_mem) begin
            lsq_v[lt]     <= 1;
            lsq_ld[lt]    <= d0_ld;
            lsq_st[lt]    <= d0_st;
            lsq_ardy[lt]  <= p0_psrdy;
            lsq_drdy[lt]  <= d0_ld ? 1'b1 : p0_ptrdy;
            lsq_cmt[lt]   <= 0;
            lsq_base[lt]  <= p0_vs;
            lsq_data[lt]  <= p0_vt;
            lsq_imm[lt]   <= d0_imm;
            lsq_ps[lt]    <= p0_ps;
            lsq_pt[lt]    <= p0_pt;
            lsq_pd[lt]    <= p0_new;
            lsq_rob[lt]   <= p0_rob;
            lsq_isret[lt] <= 0;
            lt = lt + 1;
            lc = lc + 1;
          end else if (d0_call) begin
            call_pending <= 1;
            call_tgt     <= p0_vs;
            call_addr    <= p0_r31_val + 64'hFFFFFFFFFFFFFFF8;
            call_wdata   <= dq_pc0 + 64'd4;
            rc = rc - 1;
            rt = rt - 1;
            rob_valid[p0_rob] <= 0;
          end else if (d0_ret) begin
            ret_pending <= 1;
            ret_addr    <= p0_r31_val + 64'hFFFFFFFFFFFFFFF8;
            rc = rc - 1;
            rt = rt - 1;
            rob_valid[p0_rob] <= 0;
          end else if (d0_fp) begin
            fp_d0_slot = fp_free_slot[2:0];
            d0_used_fp = 1;
            fp_v[fp_free_slot]     <= 1;
            fp_op[fp_free_slot]    <= d0_op;
            fp_ps[fp_free_slot]    <= p0_ps;
            fp_pt[fp_free_slot]    <= p0_pt;
            fp_psrdy[fp_free_slot] <= p0_psrdy;
            fp_ptrdy[fp_free_slot] <= p0_ptrdy;
            fp_vs[fp_free_slot]    <= p0_vs;
            fp_vt[fp_free_slot]    <= p0_vt;
            fp_rob[fp_free_slot]   <= p0_rob;
            fpc = fpc + 1;
          end else if (!d0_hlt) begin
            rs_d0_slot = rs_free_slot[3:0];
            d0_used_rs = 1;
            rs_v[rs_free_slot]      <= 1;
            rs_op[rs_free_slot]     <= d0_op;
            // brgt rd,rs,rt: alu_a=rs(cmpA), alu_b=rt(cmpB), imm=rd(target)
            // p0_ps=rat[rd], p0_pt=rat[rs], d0_rtx=rt
            rs_ps[rs_free_slot]     <= d0_brgt ? p0_pt                    : p0_ps;
            rs_pt[rs_free_slot]     <= d0_brgt ? rat_map[d0_rtx]          : p0_pt;
            rs_psrdy[rs_free_slot]  <= d0_brgt ? p0_ptrdy                 : p0_psrdy;
            rs_ptrdy[rs_free_slot]  <= d0_brgt ? prf_rdy[rat_map[d0_rtx]] : p0_ptrdy;
            rs_vs[rs_free_slot]     <= d0_brgt ? p0_vt                    : p0_vs;
            rs_vt[rs_free_slot]     <= d0_brgt ? prf[rat_map[d0_rtx]]     : p0_vt;
            rs_imm[rs_free_slot]    <= d0_brgt ? p0_vs                    : d0_imm;
            rs_uimm[rs_free_slot]   <= d0_uimm || d0_brri;
            rs_pt3[rs_free_slot]    <= d0_brgt ? p0_ps                    : 6'd0;
            rs_pt3rdy[rs_free_slot] <= d0_brgt ? p0_psrdy                 : 1'b1;
            rs_rob[rs_free_slot]    <= p0_rob;
            rs_pc[rs_free_slot]     <= dq_pc0;
            rs_ibr[rs_free_slot]    <= d0_br;
            rs_ibgt[rs_free_slot]   <= d0_brgt;
            rs_ijmp[rs_free_slot]   <= d0_jmp;
            rs_ibrreg[rs_free_slot] <= d0_brrr;
            rs_ibrimm[rs_free_slot] <= d0_brri;
            rs_imovr[rs_free_slot]  <= d0_mvr;
            rs_imovi[rs_free_slot]  <= d0_mvi;
            rs_ical[rs_free_slot]   <= 0;
            rs_iret[rs_free_slot]   <= 0;
            rs_ptaken[rs_free_slot] <= 0;
            rs_ptgt[rs_free_slot]   <= dq_pc0 + 64'd4;
            rsc = rsc + 1;
          end
        end

        if (d1_en) begin
          p1_ps    = (d0_en && d0_wr && d0_rd == d1_rs) ? p0_new : rat_map[d1_rs];
          p1_pt    = (d0_en && d0_wr && d0_rd == d1_rt) ? p0_new : rat_map[d1_rt];
          p1_vs    = prf[p1_ps];
          p1_vt    = prf[p1_pt];
          p1_psrdy = (d0_en && d0_wr && d0_rd == d1_rs) ? 1'b0 : prf_rdy[p1_ps];
          p1_ptrdy = (d0_en && d0_wr && d0_rd == d1_rt) ? 1'b0 : prf_rdy[p1_pt];
          // same-cycle CDB forwarding for d1 (skip if d0 just renamed the reg)
          if (c0en) begin
            if (!p1_psrdy && !(d0_en && d0_wr && d0_rd == d1_rs) && p1_ps == c0pd) begin p1_psrdy = 1; p1_vs = c0val; end
            if (!p1_ptrdy && !(d0_en && d0_wr && d0_rd == d1_rt) && p1_pt == c0pd) begin p1_ptrdy = 1; p1_vt = c0val; end
          end
          if (c1en) begin
            if (!p1_psrdy && !(d0_en && d0_wr && d0_rd == d1_rs) && p1_ps == c1pd) begin p1_psrdy = 1; p1_vs = c1val; end
            if (!p1_ptrdy && !(d0_en && d0_wr && d0_rd == d1_rt) && p1_pt == c1pd) begin p1_ptrdy = 1; p1_vt = c1val; end
          end
          if (c2en) begin
            if (!p1_psrdy && !(d0_en && d0_wr && d0_rd == d1_rs) && p1_ps == c2pd) begin p1_psrdy = 1; p1_vs = c2val; end
            if (!p1_ptrdy && !(d0_en && d0_wr && d0_rd == d1_rt) && p1_pt == c2pd) begin p1_ptrdy = 1; p1_vt = c2val; end
          end
          if (c3en) begin
            if (!p1_psrdy && !(d0_en && d0_wr && d0_rd == d1_rs) && p1_ps == c3pd) begin p1_psrdy = 1; p1_vs = c3val; end
            if (!p1_ptrdy && !(d0_en && d0_wr && d0_rd == d1_rt) && p1_pt == c3pd) begin p1_ptrdy = 1; p1_vt = c3val; end
          end
          p1_r31_phys = rat_map[5'd31];
          p1_r31_val  = prf[p1_r31_phys];
          p1_r31_rdy  = prf_rdy[p1_r31_phys];

          if (d1_wr && !d1_call && fc > 0) begin
            p1_new = free_list[fh];
            p1_old = (d0_en && d0_wr && d0_rd == d1_rd) ? p0_new : rat_map[d1_rd];
            fh     = fh + 1;
            fc     = fc - 1;
            rat_map[d1_rd]  <= p1_new;
            prf_rdy[p1_new] <= 0;
          end else begin
            p1_new = rat_map[d1_rd];
            p1_old = rat_map[d1_rd];
          end

          p1_rob = rt;
          rt = rt + 1;
          rc = rc + 1;
          rob_valid[p1_rob]      <= 1;
          rob_done[p1_rob]       <= (!d1_wr && !d1_br && !d1_jmp) ? 1 : 0;
          rob_arch[p1_rob]       <= d1_rd;
          rob_phys[p1_rob]       <= p1_new;
          rob_old[p1_rob]        <= p1_old;
          rob_has_dest[p1_rob]   <= d1_wr;
          rob_is_store[p1_rob]   <= d1_st;
          rob_is_halt[p1_rob]    <= d1_hlt;
          rob_is_branch[p1_rob]  <= d1_br;
          rob_is_jump[p1_rob]    <= d1_jmp;
          rob_pc[p1_rob]         <= dq_pc1;
          rob_pred_taken[p1_rob] <= dq_ptaken1;
          rob_pred_tgt[p1_rob]   <= dq_ptaken1 ? dq_ptgt1 : (dq_pc1 + 64'd4);
          rob_act_taken[p1_rob]  <= 0;
          rob_act_tgt[p1_rob]    <= 0;

          if (d1_mem) begin
            lsq_v[lt]     <= 1;
            lsq_ld[lt]    <= d1_ld;
            lsq_st[lt]    <= d1_st;
            lsq_ardy[lt]  <= p1_psrdy;
            lsq_drdy[lt]  <= d1_ld ? 1'b1 : p1_ptrdy;
            lsq_cmt[lt]   <= 0;
            lsq_base[lt]  <= p1_vs;
            lsq_data[lt]  <= p1_vt;
            lsq_imm[lt]   <= d1_imm;
            lsq_ps[lt]    <= p1_ps;
            lsq_pt[lt]    <= p1_pt;
            lsq_pd[lt]    <= p1_new;
            lsq_rob[lt]   <= p1_rob;
            lsq_isret[lt] <= 0;
            lt = lt + 1;
            lc = lc + 1;
          end else if (d1_call) begin
            call_pending <= 1;
            call_tgt     <= p1_vs;
            call_addr    <= p1_r31_val + 64'hFFFFFFFFFFFFFFF8;
            call_wdata   <= dq_pc1 + 64'd4;
            rc = rc - 1;
            rt = rt - 1;
            rob_valid[p1_rob] <= 0;
          end else if (d1_ret) begin
            ret_pending <= 1;
            ret_addr    <= p1_r31_val + 64'hFFFFFFFFFFFFFFF8;
            rc = rc - 1;
            rt = rt - 1;
            rob_valid[p1_rob] <= 0;
          end else if (d1_fp) begin
            begin : fp_slot1
              reg [2:0] fslot;
              fslot = 0;
              for (j = RS_FP - 1; j >= 0; j = j - 1)
              if (!fp_v[j] && (!d0_used_fp || 3'(j) != fp_d0_slot)) fslot = j[2:0];
              fp_v[fslot]     <= 1;
              fp_op[fslot]    <= d1_op;
              fp_ps[fslot]    <= p1_ps;
              fp_pt[fslot]    <= p1_pt;
              fp_psrdy[fslot] <= p1_psrdy;
              fp_ptrdy[fslot] <= p1_ptrdy;
              fp_vs[fslot]    <= p1_vs;
              fp_vt[fslot]    <= p1_vt;
              fp_rob[fslot]   <= p1_rob;
              fpc = fpc + 1;
            end
          end else if (!d1_hlt) begin
            begin : rs_slot1
              reg [3:0] rslot;
              rslot = 0;
              for (j = RS_INT - 1; j >= 0; j = j - 1)
              if (!rs_v[j] && (!d0_used_rs || 4'(j) != rs_d0_slot)) rslot = j[3:0];
              rs_v[rslot]      <= 1;
              rs_op[rslot]     <= d1_op;
              // brgt rd,rs,rt: alu_a=rs(cmpA), alu_b=rt(cmpB), imm=rd(target)
              // p1_ps=rat[rd], p1_pt=rat[rs], d1_rtx=rt
              rs_ps[rslot]     <= d1_brgt ? p1_pt                    : p1_ps;
              rs_pt[rslot]     <= d1_brgt ? rat_map[d1_rtx]          : p1_pt;
              rs_psrdy[rslot]  <= d1_brgt ? p1_ptrdy                 : p1_psrdy;
              rs_ptrdy[rslot]  <= d1_brgt ? prf_rdy[rat_map[d1_rtx]] : p1_ptrdy;
              rs_vs[rslot]     <= d1_brgt ? p1_vt                    : p1_vs;
              rs_vt[rslot]     <= d1_brgt ? prf[rat_map[d1_rtx]]     : p1_vt;
              rs_imm[rslot]    <= d1_brgt ? p1_vs                    : d1_imm;
              rs_uimm[rslot]   <= d1_uimm || d1_brri;
              rs_pt3[rslot]    <= d1_brgt ? p1_ps                    : 6'd0;
              rs_pt3rdy[rslot] <= d1_brgt ? p1_psrdy                 : 1'b1;
              rs_rob[rslot]    <= p1_rob;
              rs_pc[rslot]     <= dq_pc1;
              rs_ibr[rslot]    <= d1_br;
              rs_ibgt[rslot]   <= d1_brgt;
              rs_ijmp[rslot]   <= d1_jmp;
              rs_ibrreg[rslot] <= d1_brrr;
              rs_ibrimm[rslot] <= d1_brri;
              rs_imovr[rslot]  <= d1_mvr;
              rs_imovi[rslot]  <= d1_mvi;
              rs_ical[rslot]   <= 0;
              rs_iret[rslot]   <= 0;
              rs_ptaken[rslot] <= 0;
              rs_ptgt[rslot]   <= dq_pc1 + 64'd4;
              rsc = rsc + 1;
            end
          end
        end

        fl_head  <= fh;
        fl_cnt   <= fc + (commit_freed_reg ? 6'd1 : 6'd0);
        rob_tail <= rt;
        rob_cnt  <= rc - {{ROB_BITS{1'b0}}, commit_happened};
        rs_cnt   <= rsc - (rs_iss_found ? 4'd1 : 4'd0) - (rs_iss_found2 ? 4'd1 : 4'd0);
        fp_cnt   <= fpc - (fp_iss_found ? 3'd1 : 3'd0) - (fp_iss_found2 ? 3'd1 : 3'd0);
        lsq_tail <= lt;
        lsq_cnt  <= lc - (lsq_exec ? 5'd1 : 5'd0) - (lsq_exec2 ? 5'd1 : 5'd0);
      end  // dispatch_blk

      if ((call_pending || ret_pending) && !redirect_en) begin
        if (rs_iss_found) rs_cnt <= rs_cnt - 1;
        if (fp_iss_found) fp_cnt <= fp_cnt - 1;
        if (lsq_exec) lsq_cnt <= lsq_cnt - 1;
      end

      // ═══════════════════════════════════════════════
      // D. FU ISSUE (dual ALU + dual FPU)
      // ═══════════════════════════════════════════════
      if (!redirect_en) begin
        // ALU0 issue
        if (rs_iss_found) begin
          alu_en           <= 1;
          alu_op           <= rs_op[rs_iss_idx];
          alu_a            <= rs_vs[rs_iss_idx];
          alu_b            <= rs_uimm[rs_iss_idx] ? rs_imm[rs_iss_idx] : rs_vt[rs_iss_idx];
          alu_rtag         <= {1'b0, rs_rob[rs_iss_idx]};
          alu_pd           <= rob_phys[rs_rob[rs_iss_idx]];
          alu_vs_p         <= rs_vs[rs_iss_idx];
          alu_pc_p         <= rs_pc[rs_iss_idx];
          alu_ibr_p        <= rs_ibr[rs_iss_idx];
          alu_ibgt_p       <= rs_ibgt[rs_iss_idx];
          // FIX1: Only set ibgt_tgt for actual brgt instructions
          alu_ibgt_tgt_p   <= rs_ibgt[rs_iss_idx] ? rs_imm[rs_iss_idx] : 64'd0;
          alu_ijmp_p       <= rs_ijmp[rs_iss_idx];
          alu_ibrreg_p     <= rs_ibrreg[rs_iss_idx];
          alu_ibrimm_p     <= rs_ibrimm[rs_iss_idx];
          alu_imovr_p      <= rs_imovr[rs_iss_idx];
          alu_imovi_p      <= rs_imovi[rs_iss_idx];
          alu_ical_p       <= rs_ical[rs_iss_idx];
          alu_iret_p       <= rs_iret[rs_iss_idx];
          alu_ptaken_p     <= rs_ptaken[rs_iss_idx];
          alu_ptgt_p       <= rs_ptgt[rs_iss_idx];
          rs_v[rs_iss_idx] <= 0;
        end

        // ALU1 issue
        if (rs_iss_found2) begin
          alu1_en           <= 1;
          alu1_op           <= rs_op[rs_iss_idx2];
          alu1_a            <= rs_vs[rs_iss_idx2];
          alu1_b            <= rs_uimm[rs_iss_idx2] ? rs_imm[rs_iss_idx2] : rs_vt[rs_iss_idx2];
          alu1_rtag         <= {1'b0, rs_rob[rs_iss_idx2]};
          alu1_pd           <= rob_phys[rs_rob[rs_iss_idx2]];
          alu1_vs_p         <= rs_vs[rs_iss_idx2];
          alu1_pc_p         <= rs_pc[rs_iss_idx2];
          alu1_ibr_p        <= rs_ibr[rs_iss_idx2];
          alu1_ibgt_p       <= rs_ibgt[rs_iss_idx2];
          alu1_ibgt_tgt_p   <= rs_ibgt[rs_iss_idx2] ? rs_imm[rs_iss_idx2] : 64'd0;
          alu1_ijmp_p       <= rs_ijmp[rs_iss_idx2];
          alu1_ibrreg_p     <= rs_ibrreg[rs_iss_idx2];
          alu1_ibrimm_p     <= rs_ibrimm[rs_iss_idx2];
          alu1_imovr_p      <= rs_imovr[rs_iss_idx2];
          alu1_imovi_p      <= rs_imovi[rs_iss_idx2];
          alu1_ical_p       <= rs_ical[rs_iss_idx2];
          alu1_iret_p       <= rs_iret[rs_iss_idx2];
          alu1_ptaken_p     <= rs_ptaken[rs_iss_idx2];
          alu1_ptgt_p       <= rs_ptgt[rs_iss_idx2];
          rs_v[rs_iss_idx2] <= 0;
        end

        // FPU0 issue
        if (fp_iss_found) begin
          fpu_en   <= 1;
          fpu_op   <= fp_op[fp_iss_idx];
          fpu_a    <= fp_vs[fp_iss_idx];
          fpu_b    <= fp_vt[fp_iss_idx];
          fpu_rtag <= {1'b0, fp_rob[fp_iss_idx]};
          fpu_pd   <= rob_phys[fp_rob[fp_iss_idx]];
          fp_v[fp_iss_idx] <= 0;
        end

        // FPU1 issue
        if (fp_iss_found2) begin
          fpu1_en   <= 1;
          fpu1_op   <= fp_op[fp_iss_idx2];
          fpu1_a    <= fp_vs[fp_iss_idx2];
          fpu1_b    <= fp_vt[fp_iss_idx2];
          fpu1_rtag <= {1'b0, fp_rob[fp_iss_idx2]};
          fpu1_pd   <= rob_phys[fp_rob[fp_iss_idx2]];
          fp_v[fp_iss_idx2] <= 0;
        end
      end

      // FPU pd pipeline
      fp_pd_p[0] <= fpu_pd;
      fp_pd_p[1] <= fp_pd_p[0];
      fp_pd_p[2] <= fp_pd_p[1];

      // FPU1 pd pipeline
      fp1_pd_p[0] <= fpu1_pd;
      fp1_pd_p[1] <= fp1_pd_p[0];
      fp1_pd_p[2] <= fp1_pd_p[1];

      // ═══════════════════════════════════════════════
      // F. FETCH / DECODE QUEUE (4-entry, 64-byte fetch)
      // ═══════════════════════════════════════════════
      // Implementation: DQ slots 0..1 are "dispatch slots" (same as original 2-entry)
      // Slots 2..3 are "prefetch" slots that get shifted into 0..1 on the next cycle.
      // Any branch/ctrl in slot 0 or 1 clears slots 2..3 to avoid wrong-path dispatch.
      if (redirect_en || call_pending || ret_pending) begin
        dq_v[0]    <= 0; dq_v[1]    <= 0; dq_v[2]    <= 0; dq_v[3]    <= 0;
        dq_cnt     <= 0;
        dq_ptaken[0] <= 0; dq_ptaken[1] <= 0;
        dq_ptaken[2] <= 0; dq_ptaken[3] <= 0;
        if (redirect_en) pc_reg <= redirect_pc;
      end else if (!stall && !hlt) begin
        if (d0_en && d0_ctrl) begin
          // Control in slot 0: shift slot 1 to slot 0, fetch new into slot 1,
          // discard slots 2..3 (wrong-path or stale).
          dq_v[0]      <= dq_v[1];
          dq_i[0]      <= dq_i[1];
          dq_pc[0]     <= dq_pc[1];
          dq_ptaken[0] <= dq_ptaken[1];
          dq_ptgt[0]   <= dq_ptgt[1];
          dq_v[1]      <= 1;
          dq_i[1]      <= mem_i0;
          dq_pc[1]     <= pc_reg;
          dq_ptaken[1] <= bp_pred_taken0;
          dq_ptgt[1]   <= bp_pred_target0;
          dq_v[2]      <= 0;
          dq_v[3]      <= 0;
          dq_cnt       <= (dq_v[1] ? 3'd1 : 3'd0) + 3'd1;
          pc_reg       <= bp_pred_taken0 ? bp_pred_target0 : (pc_reg + 64'd4);
        end else begin
          // Normal path (d0_ctrl=0).
          // Determine how many dispatched and handle accordingly.
          // Key rule: if d1 is a control instruction, clear slots 2..3.
          begin : fetch_norm
            reg d1_is_ctrl_r;
            d1_is_ctrl_r = d1_en && (d1_jmp || d1_br || d1_call || d1_ret);

            if (d0_en && d1_en) begin
              // Both dispatched.
              if (d1_is_ctrl_r) begin
                // d1 was ctrl: must discard dq[2..3], fetch fresh
                dq_v[2] <= 0; dq_v[3] <= 0;
                dq_v[0]      <= 1;
                dq_i[0]      <= mem_i0;
                dq_pc[0]     <= pc_reg;
                dq_ptaken[0] <= bp_pred_taken0;
                dq_ptgt[0]   <= bp_pred_target0;
                if (bp_pred_taken0) begin
                  dq_v[1] <= 0; dq_cnt <= 3'd1;
                  pc_reg  <= bp_pred_target0;
                end else begin
                  dq_v[1]      <= 1;
                  dq_i[1]      <= mem_i1;
                  dq_pc[1]     <= pc_reg + 64'd4;
                  dq_ptaken[1] <= bp_pred_taken1;
                  dq_ptgt[1]   <= bp_pred_target1;
                  dq_cnt       <= 3'd2;
                  pc_reg       <= bp_pred_taken1 ? bp_pred_target1 : (pc_reg + 64'd8);
                end
              end else begin
                // Normal dual-dispatch (no ctrl in either slot):
                // Shift dq[2..3] -> dq[0..1], fetch 2 new instructions into dq[2..3]
                dq_v[0]      <= dq_v[2];
                dq_i[0]      <= dq_i[2];
                dq_pc[0]     <= dq_pc[2];
                dq_ptaken[0] <= dq_ptaken[2];
                dq_ptgt[0]   <= dq_ptgt[2];
                dq_v[1]      <= dq_v[3];
                dq_i[1]      <= dq_i[3];
                dq_pc[1]     <= dq_pc[3];
                dq_ptaken[1] <= dq_ptaken[3];
                dq_ptgt[1]   <= dq_ptgt[3];
                // Fill 2 new instrs into slots 2 and 3
                dq_v[2]      <= 1;
                dq_i[2]      <= mem_i0;
                dq_pc[2]     <= pc_reg;
                dq_ptaken[2] <= bp_pred_taken0;
                dq_ptgt[2]   <= bp_pred_target0;
                if (bp_pred_taken0) begin
                  dq_v[3] <= 0; dq_cnt <= 3'd3;
                  pc_reg  <= bp_pred_target0;
                end else begin
                  dq_v[3]      <= 1;
                  dq_i[3]      <= mem_i1;
                  dq_pc[3]     <= pc_reg + 64'd4;
                  dq_ptaken[3] <= bp_pred_taken1;
                  dq_ptgt[3]   <= bp_pred_target1;
                  dq_cnt       <= 3'd4;
                  pc_reg       <= bp_pred_taken1 ? bp_pred_target1 : (pc_reg + 64'd8);
                end
              end
            end else if (d0_en) begin
              // Only d0 dispatched (non-ctrl).
              // Shift dq[1..3] -> dq[0..2], fetch 1 new into slot 3.
              dq_v[0]      <= dq_v[1];
              dq_i[0]      <= dq_i[1];
              dq_pc[0]     <= dq_pc[1];
              dq_ptaken[0] <= dq_ptaken[1];
              dq_ptgt[0]   <= dq_ptgt[1];
              dq_v[1]      <= dq_v[2];
              dq_i[1]      <= dq_i[2];
              dq_pc[1]     <= dq_pc[2];
              dq_ptaken[1] <= dq_ptaken[2];
              dq_ptgt[1]   <= dq_ptgt[2];
              dq_v[2]      <= dq_v[3];
              dq_i[2]      <= dq_i[3];
              dq_pc[2]     <= dq_pc[3];
              dq_ptaken[2] <= dq_ptaken[3];
              dq_ptgt[2]   <= dq_ptgt[3];
              dq_v[3]      <= 1;
              dq_i[3]      <= mem_i0;
              dq_pc[3]     <= pc_reg;
              dq_ptaken[3] <= bp_pred_taken0;
              dq_ptgt[3]   <= bp_pred_target0;
              dq_cnt       <= dq_cnt; // shifted 1 out, filled 1 in — count unchanged
              pc_reg       <= bp_pred_taken0 ? bp_pred_target0 : (pc_reg + 64'd4);
            end else begin
              // No dispatch (d0 slot empty) — fill DQ from scratch with 2 instructions.
              // Use same logic as original 2-entry DQ, then fill slots 2..3 extra.
              dq_v[0]      <= 1;
              dq_i[0]      <= mem_i0;
              dq_pc[0]     <= pc_reg;
              dq_ptaken[0] <= bp_pred_taken0;
              dq_ptgt[0]   <= bp_pred_target0;
              if (bp_pred_taken0) begin
                // predicted taken on slot 0: only fetch 1 instruction
                dq_v[1] <= 0; dq_v[2] <= 0; dq_v[3] <= 0;
                dq_cnt  <= 3'd1;
                pc_reg  <= bp_pred_target0;
              end else begin
                dq_v[1]      <= 1;
                dq_i[1]      <= mem_i1;
                dq_pc[1]     <= pc_reg + 64'd4;
                dq_ptaken[1] <= bp_pred_taken1;
                dq_ptgt[1]   <= bp_pred_target1;
                dq_v[2]      <= 1;
                dq_i[2]      <= mem_i2;
                dq_pc[2]     <= pc_reg + 64'd8;
                dq_ptaken[2] <= 0;
                dq_ptgt[2]   <= 0;
                dq_v[3]      <= 1;
                dq_i[3]      <= mem_i3;
                dq_pc[3]     <= pc_reg + 64'd12;
                dq_ptaken[3] <= 0;
                dq_ptgt[3]   <= 0;
                dq_cnt       <= bp_pred_taken1 ? 3'd2 : 3'd4;
                pc_reg       <= bp_pred_taken1 ? bp_pred_target1 : (pc_reg + 64'd16);
              end
            end
          end
        end
      end

    end  // !reset
  end  // always

endmodule
