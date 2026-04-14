// decoder.sv — instruction decode
// same encoding as original tinker isa
// outputs control signals + operand addrs for rename/dispatch

module decoder (
    input  [31:0] instr,
    output reg [4:0]  raddr1,    // arch src reg a
    output reg [4:0]  raddr2,    // arch src reg b
    output reg [4:0]  waddr,     // arch dest reg
    output reg [63:0] immediate,
    output reg [4:0]  op,        // alu/fpu op code
    output reg        use_imm,   // b operand = imm
    output reg        write,     // instr writes a reg
    output reg        is_load,
    output reg        is_store,
    output reg        is_branch,
    output reg        is_brgt,
    output reg        is_jump,
    output reg        is_brr_reg,
    output reg        is_brr_imm,
    output reg        is_return,
    output reg        is_call,
    output reg        is_halt,
    output reg        is_mov_reg,
    output reg        is_mov_imm,
    output reg [4:0]  rt_addr    // third src for brgt
);

    wire [4:0]  opcode = instr[31:27];
    wire [4:0]  rd     = instr[26:22];
    wire [4:0]  rs     = instr[21:17];
    wire [4:0]  rt     = instr[16:12];
    wire [11:0] imm12  = instr[11:0];

    wire [63:0] imm_signed   = {{52{imm12[11]}}, imm12};
    wire [63:0] imm_unsigned = {52'd0, imm12};

    // alu op codes
    localparam ADD   = 5'd0;
    localparam SUB   = 5'd1;
    localparam MUL   = 5'd2;
    localparam DIV   = 5'd3;
    localparam AND   = 5'd4;
    localparam OR    = 5'd5;
    localparam XOR   = 5'd6;
    localparam NOT   = 5'd7;
    localparam SHR   = 5'd8;
    localparam SHL   = 5'd9;
    localparam ADDF  = 5'd10;
    localparam SUBF  = 5'd11;
    localparam MULF  = 5'd12;
    localparam DIVF  = 5'd13;
    localparam CMPNZ = 5'd14;
    localparam CMPGT = 5'd15;

    always @(*) begin
        // defaults — nop
        raddr1     = 5'd0; raddr2    = 5'd0; waddr     = 5'd0;
        rt_addr    = 5'd0; immediate = 64'd0; op        = ADD;
        use_imm    = 0;    write     = 0;    is_load   = 0;
        is_store   = 0;    is_branch = 0;    is_brgt   = 0;
        is_jump    = 0;    is_brr_reg= 0;    is_brr_imm= 0;
        is_return  = 0;    is_call   = 0;    is_halt   = 0;
        is_mov_reg = 0;    is_mov_imm= 0;

        case (opcode)
            5'h00: begin // and rd, rs, rt
                raddr1=rs; raddr2=rt; waddr=rd; op=AND; write=1; end
            5'h01: begin // or
                raddr1=rs; raddr2=rt; waddr=rd; op=OR; write=1; end
            5'h02: begin // xor
                raddr1=rs; raddr2=rt; waddr=rd; op=XOR; write=1; end
            5'h03: begin // not rd, rs
                raddr1=rs; waddr=rd; op=NOT; write=1; end

            5'h04: begin // shftr rd, rs, rt
                raddr1=rs; raddr2=rt; waddr=rd; op=SHR; write=1; end
            5'h05: begin // shftri rd, L
                raddr1=rd; waddr=rd; immediate=imm_unsigned;
                op=SHR; use_imm=1; write=1; end
            5'h06: begin // shftl
                raddr1=rs; raddr2=rt; waddr=rd; op=SHL; write=1; end
            5'h07: begin // shftli rd, L
                raddr1=rd; waddr=rd; immediate=imm_unsigned;
                op=SHL; use_imm=1; write=1; end

            5'h08: begin // br rd — absolute jump
                raddr1=rd; is_jump=1; end
            5'h09: begin // brr rd — pc-relative via reg
                raddr1=rd; is_jump=1; is_brr_reg=1; end
            5'h0A: begin // brr L — pc-relative via imm
                immediate=imm_signed; is_jump=1; is_brr_imm=1; end
            5'h0B: begin // brnz rd, rs
                raddr1=rs; raddr2=rd; op=CMPNZ; is_branch=1; end
            5'h0C: begin // call rd
                raddr1=rd; waddr=5'd31; is_jump=1; is_call=1; write=1; end
            5'h0D: begin // return
                is_jump=1; is_return=1; end
            5'h0E: begin // brgt rd, rs, rt
                raddr1=rd; raddr2=rs; rt_addr=rt;
                op=CMPGT; is_branch=1; is_brgt=1; end
            5'h0F: begin // halt
                is_halt=1; end

            5'h10: begin // load rd, (rs)(L)
                raddr1=rs; waddr=rd; immediate=imm_signed;
                is_load=1; write=1; end
            5'h11: begin // mov rd, rs
                raddr1=rs; waddr=rd; is_mov_reg=1; write=1; end
            5'h12: begin // mov rd, L
                raddr1=rd; waddr=rd; immediate=imm_unsigned;
                is_mov_imm=1; use_imm=1; write=1; end
            5'h13: begin // store (rd)(L), rs
                raddr1=rd; raddr2=rs; immediate=imm_signed; is_store=1; end

            5'h14: begin // addf
                raddr1=rs; raddr2=rt; waddr=rd; op=ADDF; write=1; end
            5'h15: begin // subf
                raddr1=rs; raddr2=rt; waddr=rd; op=SUBF; write=1; end
            5'h16: begin // mulf
                raddr1=rs; raddr2=rt; waddr=rd; op=MULF; write=1; end
            5'h17: begin // divf
                raddr1=rs; raddr2=rt; waddr=rd; op=DIVF; write=1; end

            5'h18: begin // add
                raddr1=rs; raddr2=rt; waddr=rd; op=ADD; write=1; end
            5'h19: begin // addi
                raddr1=rd; waddr=rd; immediate=imm_signed;
                op=ADD; use_imm=1; write=1; end
            5'h1A: begin // sub
                raddr1=rs; raddr2=rt; waddr=rd; op=SUB; write=1; end
            5'h1B: begin // subi
                raddr1=rd; waddr=rd; immediate=imm_signed;
                op=SUB; use_imm=1; write=1; end
            5'h1C: begin // mul
                raddr1=rs; raddr2=rt; waddr=rd; op=MUL; write=1; end
            5'h1D: begin // div
                raddr1=rs; raddr2=rt; waddr=rd; op=DIV; write=1; end

            default: begin end
        endcase
    end

endmodule

// rob.sv — reorder buffer
// circular buffer; instrs enter in order (tail), commit in order (head)
// holds result until safe to commit to arch state
// detects branch mispredictions at commit
// signals halt when halt instr is at head
// dual-issue: accepts 2 entries per cycle, commits up to 2 per cycle

`ifndef PHYS_W
  `define PHYS_W 6
`endif
`ifndef PC_START
  `define PC_START 64'h2000
`endif

module rob #(
    parameter NENTRIES  = 64,
    parameter PHYS_W    = 6,
    parameter ROB_BITS  = 6   // log2(NENTRIES); must match 6-bit tags in rs/lsq/alu
) (
    input         clk,
    input         reset,

    // --- alloc ports (from dispatch, dual-issue) ---
    input         alloc0_en,
    input  [4:0]  alloc0_arch_rd,   // dest arch reg
    input  [PHYS_W-1:0] alloc0_phys_rd,  // new phys dest
    input  [PHYS_W-1:0] alloc0_old_rd,   // old phys mapping (to free on commit)
    input  [63:0] alloc0_pc,
    input         alloc0_is_branch,
    input         alloc0_is_jump,
    input         alloc0_is_call,
    input         alloc0_is_return,
    input         alloc0_is_store,
    input         alloc0_is_halt,
    input         alloc0_pred_taken,
    input  [63:0] alloc0_pred_target,
    input         alloc0_has_dest,
    output [ROB_BITS-1:0] alloc0_tag,  // rob index assigned

    input         alloc1_en,
    input  [4:0]  alloc1_arch_rd,
    input  [PHYS_W-1:0] alloc1_phys_rd,
    input  [PHYS_W-1:0] alloc1_old_rd,
    input  [63:0] alloc1_pc,
    input         alloc1_is_branch,
    input         alloc1_is_jump,
    input         alloc1_is_call,
    input         alloc1_is_return,
    input         alloc1_is_store,
    input         alloc1_is_halt,
    input         alloc1_pred_taken,
    input  [63:0] alloc1_pred_target,
    input         alloc1_has_dest,
    output [ROB_BITS-1:0] alloc1_tag,

    // --- cdb write-back (mark complete) ---
    input         wb0_en,
    input  [ROB_BITS-1:0] wb0_rob_tag,
    input  [63:0] wb0_result,
    input         wb0_actual_taken,   // branch result
    input  [63:0] wb0_actual_target,

    input         wb1_en,
    input  [ROB_BITS-1:0] wb1_rob_tag,
    input  [63:0] wb1_result,
    input         wb1_actual_taken,
    input  [63:0] wb1_actual_target,

    // load done (from lsq)
    input         ld_done,
    input  [ROB_BITS-1:0] ld_rob_tag,
    input  [63:0] ld_result,

    // fp done (from fpu pipeline)
    input         fp_done,
    input  [ROB_BITS-1:0] fp_rob_tag,
    input  [63:0] fp_result,

    // --- commit outputs ---
    // commit 0 (head)
    output reg        commit0_en,
    output reg [4:0]  commit0_arch_rd,
    output reg [PHYS_W-1:0] commit0_phys_rd,
    output reg [PHYS_W-1:0] commit0_old_rd,
    output reg [63:0] commit0_result,
    output reg        commit0_has_dest,
    output reg        commit0_is_store,
    output reg [ROB_BITS-1:0] commit0_rob_tag,

    // commit 1 (head+1, if head+1 also done)
    output reg        commit1_en,
    output reg [4:0]  commit1_arch_rd,
    output reg [PHYS_W-1:0] commit1_phys_rd,
    output reg [PHYS_W-1:0] commit1_old_rd,
    output reg [63:0] commit1_result,
    output reg        commit1_has_dest,
    output reg        commit1_is_store,
    output reg [ROB_BITS-1:0] commit1_rob_tag,

    // branch misprediction: flush + redirect
    output reg        mispredict,
    output reg [63:0] correct_pc,

    // branch predictor update (on every branch commit)
    output reg        bp_upd_en,
    output reg [63:0] bp_upd_pc,
    output reg        bp_upd_taken,
    output reg [63:0] bp_upd_target,

    // halt committed
    output reg        halt_commit,

    // store commit (to lsq)
    output reg        st_commit_en,
    output reg [ROB_BITS-1:0] st_commit_rob_tag,

    // rat map snapshot for mispredict recovery
    // (rob holds last-committed map — simplified: use arch reg file on flush)
    output reg [ROB_BITS-1:0] rob_head,

    // capacity
    output wire full,
    output wire nearly_full
);

    // rob entry
    reg        valid_r      [0:NENTRIES-1];
    reg        done_r       [0:NENTRIES-1]; // result available
    reg [4:0]  arch_rd_r    [0:NENTRIES-1];
    reg [PHYS_W-1:0] phys_rd_r [0:NENTRIES-1];
    reg [PHYS_W-1:0] old_rd_r  [0:NENTRIES-1];
    reg [63:0] result_r     [0:NENTRIES-1];
    reg [63:0] pc_r         [0:NENTRIES-1];
    reg        is_branch_r  [0:NENTRIES-1];
    reg        is_jump_r    [0:NENTRIES-1];
    reg        is_call_r    [0:NENTRIES-1];
    reg        is_return_r  [0:NENTRIES-1];
    reg        is_store_r   [0:NENTRIES-1];
    reg        is_halt_r    [0:NENTRIES-1];
    reg        pred_taken_r [0:NENTRIES-1];
    reg [63:0] pred_target_r[0:NENTRIES-1];
    reg        actual_taken_r [0:NENTRIES-1];
    reg [63:0] actual_target_r[0:NENTRIES-1];
    reg        has_dest_r   [0:NENTRIES-1];

    reg [ROB_BITS-1:0] head_r, tail_r;
    reg [ROB_BITS:0]   cnt_r;  // extra bit for full detection

    assign full        = (cnt_r >= NENTRIES - 1);
    assign nearly_full = (cnt_r >= NENTRIES - 2);

    // alloc tags = current tail positions
    assign alloc0_tag = tail_r;
    assign alloc1_tag = tail_r + 1;

    integer i;
    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < NENTRIES; i = i + 1)
                begin valid_r[i] <= 0; done_r[i] <= 0; end
            head_r      <= 0; tail_r      <= 0; cnt_r <= 0;
            commit0_en  <= 0; commit1_en  <= 0;
            mispredict  <= 0; halt_commit <= 0;
            st_commit_en <= 0; bp_upd_en <= 0;
        end else begin
            commit0_en   <= 0; commit1_en <= 0;
            mispredict   <= 0; halt_commit <= 0;
            st_commit_en <= 0; bp_upd_en  <= 0;

            // --- mark done from cdb ---
            if (wb0_en) begin
                done_r[wb0_rob_tag]          <= 1;
                result_r[wb0_rob_tag]        <= wb0_result;
                actual_taken_r[wb0_rob_tag]  <= wb0_actual_taken;
                actual_target_r[wb0_rob_tag] <= wb0_actual_target;
            end
            if (wb1_en) begin
                done_r[wb1_rob_tag]          <= 1;
                result_r[wb1_rob_tag]        <= wb1_result;
                actual_taken_r[wb1_rob_tag]  <= wb1_actual_taken;
                actual_target_r[wb1_rob_tag] <= wb1_actual_target;
            end
            if (ld_done) begin
                done_r[ld_rob_tag]   <= 1;
                result_r[ld_rob_tag] <= ld_result;
            end
            if (fp_done) begin
                done_r[fp_rob_tag]   <= 1;
                result_r[fp_rob_tag] <= fp_result;
            end

            // --- alloc new entries ---
            if (alloc0_en) begin
                valid_r[tail_r]       <= 1;
                done_r[tail_r]        <= 0;
                arch_rd_r[tail_r]     <= alloc0_arch_rd;
                phys_rd_r[tail_r]     <= alloc0_phys_rd;
                old_rd_r[tail_r]      <= alloc0_old_rd;
                pc_r[tail_r]          <= alloc0_pc;
                is_branch_r[tail_r]   <= alloc0_is_branch;
                is_jump_r[tail_r]     <= alloc0_is_jump;
                is_call_r[tail_r]     <= alloc0_is_call;
                is_return_r[tail_r]   <= alloc0_is_return;
                is_store_r[tail_r]    <= alloc0_is_store;
                is_halt_r[tail_r]     <= alloc0_is_halt;
                pred_taken_r[tail_r]  <= alloc0_pred_taken;
                pred_target_r[tail_r] <= alloc0_pred_target;
                has_dest_r[tail_r]    <= alloc0_has_dest;
            end
            if (alloc1_en) begin
                valid_r[tail_r+1]       <= 1;
                done_r[tail_r+1]        <= 0;
                arch_rd_r[tail_r+1]     <= alloc1_arch_rd;
                phys_rd_r[tail_r+1]     <= alloc1_phys_rd;
                old_rd_r[tail_r+1]      <= alloc1_old_rd;
                pc_r[tail_r+1]          <= alloc1_pc;
                is_branch_r[tail_r+1]   <= alloc1_is_branch;
                is_jump_r[tail_r+1]     <= alloc1_is_jump;
                is_call_r[tail_r+1]     <= alloc1_is_call;
                is_return_r[tail_r+1]   <= alloc1_is_return;
                is_store_r[tail_r+1]    <= alloc1_is_store;
                is_halt_r[tail_r+1]     <= alloc1_is_halt;
                pred_taken_r[tail_r+1]  <= alloc1_pred_taken;
                pred_target_r[tail_r+1] <= alloc1_pred_target;
                has_dest_r[tail_r+1]    <= alloc1_has_dest;
            end
            tail_r <= tail_r + alloc0_en + alloc1_en;
            cnt_r  <= cnt_r + alloc0_en + alloc1_en;

            // --- commit head (in order) ---
            rob_head <= head_r;

            if (valid_r[head_r] && done_r[head_r]) begin
                // commit head entry
                commit0_en      <= 1;
                commit0_arch_rd <= arch_rd_r[head_r];
                commit0_phys_rd <= phys_rd_r[head_r];
                commit0_old_rd  <= old_rd_r[head_r];
                commit0_result  <= result_r[head_r];
                commit0_has_dest <= has_dest_r[head_r];
                commit0_is_store <= is_store_r[head_r];
                commit0_rob_tag <= head_r;

                // halt at commit
                if (is_halt_r[head_r]) halt_commit <= 1;

                // store commit signal to lsq
                if (is_store_r[head_r]) begin
                    st_commit_en     <= 1;
                    st_commit_rob_tag <= head_r;
                end

                // branch/jump resolution at commit
                if (is_branch_r[head_r] || is_jump_r[head_r]) begin
                    bp_upd_en     <= 1;
                    bp_upd_pc     <= pc_r[head_r];
                    bp_upd_taken  <= actual_taken_r[head_r];
                    bp_upd_target <= actual_target_r[head_r];
                    // detect misprediction
                    if (pred_taken_r[head_r] != actual_taken_r[head_r] ||
                        (actual_taken_r[head_r] &&
                         pred_target_r[head_r] != actual_target_r[head_r])) begin
                        mispredict  <= 1;
                        correct_pc  <= actual_taken_r[head_r]
                                       ? actual_target_r[head_r]
                                       : (pc_r[head_r] + 64'd4);
                    end
                end

                valid_r[head_r] <= 0;
                head_r <= head_r + 1;
                cnt_r  <= cnt_r - 1;

                // try commit head+1 in same cycle (dual commit)
                if (!mispredict && !is_halt_r[head_r] &&
                    valid_r[head_r+1] && done_r[head_r+1]) begin
                    commit1_en      <= 1;
                    commit1_arch_rd <= arch_rd_r[head_r+1];
                    commit1_phys_rd <= phys_rd_r[head_r+1];
                    commit1_old_rd  <= old_rd_r[head_r+1];
                    commit1_result  <= result_r[head_r+1];
                    commit1_has_dest <= has_dest_r[head_r+1];
                    commit1_is_store <= is_store_r[head_r+1];
                    commit1_rob_tag <= head_r + 1;

                    if (is_halt_r[head_r+1]) halt_commit <= 1;

                    if (is_store_r[head_r+1]) begin
                        // note: simplified — only one store commit per cycle
                    end

                    valid_r[head_r+1] <= 0;
                    head_r <= head_r + 2;
                    cnt_r  <= cnt_r - 2;
                end

            end // commit

            // --- flush on mispredict: squash all in-flight ---
            if (mispredict) begin
                for (i = 0; i < NENTRIES; i = i + 1)
                    begin valid_r[i] <= 0; done_r[i] <= 0; end
                head_r <= 0; tail_r <= 0; cnt_r <= 0;
            end

        end
    end

endmodule

// rs.sv — reservation stations
// unified rs for int alu, separate rs for fp, ls queue for mem ops
// listens to cdb; wakes up when both operands ready
// dual-issue dispatch: accepts up to 2 instrs per cycle
// issues 1 instr/cycle to each functional unit

`ifndef PHYS_W
  `define PHYS_W 6
`endif

// int/branch reservation stations (8 entries)
module rs_int #(
    parameter NENTRIES = 8,
    parameter PHYS_W   = 6
) (
    input         clk,
    input         reset,

    // dispatch port (up to 2 per cycle)
    input         disp0_en,
    input  [4:0]  disp0_op,
    input  [PHYS_W-1:0] disp0_ps,   // phys src a
    input  [PHYS_W-1:0] disp0_pt,   // phys src b
    input         disp0_ps_rdy,     // src a already ready
    input         disp0_pt_rdy,     // src b already ready
    input  [63:0] disp0_vs,         // src a value (if rdy)
    input  [63:0] disp0_vt,         // src b value (if rdy)
    input  [63:0] disp0_imm,
    input         disp0_use_imm,
    input  [5:0]  disp0_rob_tag,    // rob entry index
    input  [63:0] disp0_pc,
    input         disp0_is_branch,
    input         disp0_is_brgt,
    input         disp0_is_jump,
    input         disp0_is_brr_reg,
    input         disp0_is_brr_imm,
    input         disp0_is_mov_reg,
    input         disp0_is_mov_imm,
    input         disp0_pred_taken,
    input  [63:0] disp0_pred_target,

    input         disp1_en,
    input  [4:0]  disp1_op,
    input  [PHYS_W-1:0] disp1_ps,
    input  [PHYS_W-1:0] disp1_pt,
    input         disp1_ps_rdy,
    input         disp1_pt_rdy,
    input  [63:0] disp1_vs,
    input  [63:0] disp1_vt,
    input  [63:0] disp1_imm,
    input         disp1_use_imm,
    input  [5:0]  disp1_rob_tag,
    input  [63:0] disp1_pc,
    input         disp1_is_branch,
    input         disp1_is_brgt,
    input         disp1_is_jump,
    input         disp1_is_brr_reg,
    input         disp1_is_brr_imm,
    input         disp1_is_mov_reg,
    input         disp1_is_mov_imm,
    input         disp1_pred_taken,
    input  [63:0] disp1_pred_target,

    // cdb broadcast (up to 2 per cycle from alu0/alu1)
    input         cdb0_en,
    input  [PHYS_W-1:0] cdb0_tag,   // phys dest reg
    input  [63:0] cdb0_val,
    input  [5:0]  cdb0_rob_tag,

    input         cdb1_en,
    input  [PHYS_W-1:0] cdb1_tag,
    input  [63:0] cdb1_val,
    input  [5:0]  cdb1_rob_tag,

    // issue to int alu
    output reg        issue_en,
    output reg [4:0]  issue_op,
    output reg [63:0] issue_vs,
    output reg [63:0] issue_vt,
    output reg [5:0]  issue_rob_tag,
    output reg [63:0] issue_pc,
    output reg        issue_is_branch,
    output reg        issue_is_brgt,
    output reg        issue_is_jump,
    output reg        issue_is_brr_reg,
    output reg        issue_is_brr_imm,
    output reg        issue_is_mov_reg,
    output reg        issue_is_mov_imm,
    output reg        issue_pred_taken,
    output reg [63:0] issue_pred_target,

    // back-pressure signals
    output wire full,
    output wire nearly_full  // stall fetch if 1 slot left
);

    // rs entry fields
    reg        valid     [0:NENTRIES-1];
    reg [4:0]  op_r      [0:NENTRIES-1];
    reg [PHYS_W-1:0] ps_r [0:NENTRIES-1];
    reg [PHYS_W-1:0] pt_r [0:NENTRIES-1];
    reg        ps_rdy_r  [0:NENTRIES-1];
    reg        pt_rdy_r  [0:NENTRIES-1];
    reg [63:0] vs_r      [0:NENTRIES-1];
    reg [63:0] vt_r      [0:NENTRIES-1];
    reg [63:0] imm_r     [0:NENTRIES-1];
    reg        use_imm_r [0:NENTRIES-1];
    reg [5:0]  rob_tag_r [0:NENTRIES-1];
    reg [63:0] pc_r      [0:NENTRIES-1];
    reg        is_branch_r  [0:NENTRIES-1];
    reg        is_brgt_r    [0:NENTRIES-1];
    reg        is_jump_r    [0:NENTRIES-1];
    reg        is_brr_reg_r [0:NENTRIES-1];
    reg        is_brr_imm_r [0:NENTRIES-1];
    reg        is_mov_reg_r [0:NENTRIES-1];
    reg        is_mov_imm_r [0:NENTRIES-1];
    reg        pred_taken_r  [0:NENTRIES-1];
    reg [63:0] pred_target_r [0:NENTRIES-1];

    // free slot count
    reg [3:0] free_cnt;
    assign full        = (free_cnt == 0);
    assign nearly_full = (free_cnt <= 2);

    // find free slot (priority encoder)
    integer k;
    reg [3:0] free_slot0, free_slot1;
    reg       found0, found1;

    always @(*) begin
        free_slot0 = 0; free_slot1 = 0;
        found0 = 0; found1 = 0;
        for (k = NENTRIES-1; k >= 0; k = k - 1) begin
            if (!valid[k]) begin
                if (!found0) begin free_slot0 = k; found0 = 1; end
                else if (!found1) begin free_slot1 = k; found1 = 1; end
            end
        end
    end

    // select ready entry to issue (oldest-first = lowest index)
    integer j;
    reg [3:0] iss_idx;
    reg       iss_found;

    always @(*) begin
        iss_idx = 0; iss_found = 0;
        for (j = 0; j < NENTRIES; j = j + 1) begin
            if (!iss_found && valid[j] && ps_rdy_r[j] &&
                (use_imm_r[j] || pt_rdy_r[j])) begin
                iss_idx = j; iss_found = 1;
            end
        end
    end

    // helper: apply cdb to a value/ready pair
    `define CDB_FWD(ps, ps_rdy, vs) \
        (cdb0_en && (cdb0_tag == (ps))) ? 1'b1 : \
        (cdb1_en && (cdb1_tag == (ps))) ? 1'b1 : (ps_rdy)
    `define CDB_VAL(ps, ps_rdy, vs) \
        (cdb0_en && (cdb0_tag == (ps)) && !(ps_rdy)) ? cdb0_val : \
        (cdb1_en && (cdb1_tag == (ps)) && !(ps_rdy)) ? cdb1_val : (vs)

    integer i;
    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < NENTRIES; i = i + 1) valid[i] <= 0;
            free_cnt  <= NENTRIES;
            issue_en  <= 0;
        end else begin
            issue_en <= 0;

            // --- cdb snoop: update all waiting entries ---
            for (i = 0; i < NENTRIES; i = i + 1) begin
                if (valid[i]) begin
                    if (cdb0_en && !ps_rdy_r[i] && cdb0_tag == ps_r[i]) begin
                        ps_rdy_r[i] <= 1; vs_r[i] <= cdb0_val; end
                    if (cdb1_en && !ps_rdy_r[i] && cdb1_tag == ps_r[i]) begin
                        ps_rdy_r[i] <= 1; vs_r[i] <= cdb1_val; end
                    if (cdb0_en && !pt_rdy_r[i] && cdb0_tag == pt_r[i]) begin
                        pt_rdy_r[i] <= 1; vt_r[i] <= cdb0_val; end
                    if (cdb1_en && !pt_rdy_r[i] && cdb1_tag == pt_r[i]) begin
                        pt_rdy_r[i] <= 1; vt_r[i] <= cdb1_val; end
                end
            end

            // --- issue ready entry ---
            if (iss_found) begin
                issue_en         <= 1;
                issue_op         <= op_r[iss_idx];
                issue_vs         <= vs_r[iss_idx];
                issue_vt         <= use_imm_r[iss_idx] ? imm_r[iss_idx] : vt_r[iss_idx];
                issue_rob_tag    <= rob_tag_r[iss_idx];
                issue_pc         <= pc_r[iss_idx];
                issue_is_branch  <= is_branch_r[iss_idx];
                issue_is_brgt    <= is_brgt_r[iss_idx];
                issue_is_jump    <= is_jump_r[iss_idx];
                issue_is_brr_reg <= is_brr_reg_r[iss_idx];
                issue_is_brr_imm <= is_brr_imm_r[iss_idx];
                issue_is_mov_reg <= is_mov_reg_r[iss_idx];
                issue_is_mov_imm <= is_mov_imm_r[iss_idx];
                issue_pred_taken  <= pred_taken_r[iss_idx];
                issue_pred_target <= pred_target_r[iss_idx];
                // free the slot
                valid[iss_idx] <= 0;
                free_cnt       <= free_cnt + 1 - (disp0_en ? 1 : 0) - (disp1_en ? 1 : 0);
            end else begin
                free_cnt <= free_cnt - (disp0_en ? 1 : 0) - (disp1_en ? 1 : 0);
            end

            // --- dispatch instr 0 ---
            if (disp0_en && found0) begin
                valid[free_slot0]      <= 1;
                op_r[free_slot0]       <= disp0_op;
                ps_r[free_slot0]       <= disp0_ps;
                pt_r[free_slot0]       <= disp0_pt;
                ps_rdy_r[free_slot0]   <= disp0_ps_rdy;
                pt_rdy_r[free_slot0]   <= disp0_pt_rdy;
                vs_r[free_slot0]       <= disp0_vs;
                vt_r[free_slot0]       <= disp0_vt;
                imm_r[free_slot0]      <= disp0_imm;
                use_imm_r[free_slot0]  <= disp0_use_imm;
                rob_tag_r[free_slot0]  <= disp0_rob_tag;
                pc_r[free_slot0]       <= disp0_pc;
                is_branch_r[free_slot0]  <= disp0_is_branch;
                is_brgt_r[free_slot0]    <= disp0_is_brgt;
                is_jump_r[free_slot0]    <= disp0_is_jump;
                is_brr_reg_r[free_slot0] <= disp0_is_brr_reg;
                is_brr_imm_r[free_slot0] <= disp0_is_brr_imm;
                is_mov_reg_r[free_slot0] <= disp0_is_mov_reg;
                is_mov_imm_r[free_slot0] <= disp0_is_mov_imm;
                pred_taken_r[free_slot0]  <= disp0_pred_taken;
                pred_target_r[free_slot0] <= disp0_pred_target;
            end

            // --- dispatch instr 1 ---
            if (disp1_en && found1) begin
                valid[free_slot1]      <= 1;
                op_r[free_slot1]       <= disp1_op;
                ps_r[free_slot1]       <= disp1_ps;
                pt_r[free_slot1]       <= disp1_pt;
                ps_rdy_r[free_slot1]   <= disp1_ps_rdy;
                pt_rdy_r[free_slot1]   <= disp1_pt_rdy;
                vs_r[free_slot1]       <= disp1_vs;
                vt_r[free_slot1]       <= disp1_vt;
                imm_r[free_slot1]      <= disp1_imm;
                use_imm_r[free_slot1]  <= disp1_use_imm;
                rob_tag_r[free_slot1]  <= disp1_rob_tag;
                pc_r[free_slot1]       <= disp1_pc;
                is_branch_r[free_slot1]  <= disp1_is_branch;
                is_brgt_r[free_slot1]    <= disp1_is_brgt;
                is_jump_r[free_slot1]    <= disp1_is_jump;
                is_brr_reg_r[free_slot1] <= disp1_is_brr_reg;
                is_brr_imm_r[free_slot1] <= disp1_is_brr_imm;
                is_mov_reg_r[free_slot1] <= disp1_is_mov_reg;
                is_mov_imm_r[free_slot1] <= disp1_is_mov_imm;
                pred_taken_r[free_slot1]  <= disp1_pred_taken;
                pred_target_r[free_slot1] <= disp1_pred_target;
            end
        end
    end

endmodule


// fp reservation stations (4 entries — fp less common)
module rs_fp #(
    parameter NENTRIES = 4,
    parameter PHYS_W   = 6
) (
    input         clk,
    input         reset,

    // dispatch (single port — fp less frequent)
    input         disp_en,
    input  [4:0]  disp_op,
    input  [PHYS_W-1:0] disp_ps,
    input  [PHYS_W-1:0] disp_pt,
    input         disp_ps_rdy,
    input         disp_pt_rdy,
    input  [63:0] disp_vs,
    input  [63:0] disp_vt,
    input  [5:0]  disp_rob_tag,

    // cdb
    input         cdb0_en,
    input  [PHYS_W-1:0] cdb0_tag,
    input  [63:0] cdb0_val,
    input         cdb1_en,
    input  [PHYS_W-1:0] cdb1_tag,
    input  [63:0] cdb1_val,

    // issue to fpu
    output reg        issue_en,
    output reg [4:0]  issue_op,
    output reg [63:0] issue_vs,
    output reg [63:0] issue_vt,
    output reg [5:0]  issue_rob_tag,

    output wire full
);

    reg        valid_r   [0:NENTRIES-1];
    reg [4:0]  op_r      [0:NENTRIES-1];
    reg [PHYS_W-1:0] ps_r [0:NENTRIES-1];
    reg [PHYS_W-1:0] pt_r [0:NENTRIES-1];
    reg        ps_rdy_r  [0:NENTRIES-1];
    reg        pt_rdy_r  [0:NENTRIES-1];
    reg [63:0] vs_r      [0:NENTRIES-1];
    reg [63:0] vt_r      [0:NENTRIES-1];
    reg [5:0]  rob_tag_r [0:NENTRIES-1];

    reg [2:0] free_cnt;
    assign full = (free_cnt == 0);

    integer k;
    reg [2:0] free_slot;
    reg       found_free;
    reg [2:0] iss_idx;
    reg       iss_found;

    always @(*) begin
        free_slot = 0; found_free = 0;
        iss_idx = 0; iss_found = 0;
        for (k = 0; k < NENTRIES; k = k + 1) begin
            if (!valid_r[k] && !found_free) begin free_slot = k; found_free = 1; end
            if (valid_r[k] && ps_rdy_r[k] && pt_rdy_r[k] && !iss_found)
                begin iss_idx = k; iss_found = 1; end
        end
    end

    integer i;
    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < NENTRIES; i = i + 1) valid_r[i] <= 0;
            free_cnt <= NENTRIES; issue_en <= 0;
        end else begin
            issue_en <= 0;
            // cdb snoop
            for (i = 0; i < NENTRIES; i = i + 1) begin
                if (valid_r[i]) begin
                    if (cdb0_en && !ps_rdy_r[i] && cdb0_tag == ps_r[i])
                        begin ps_rdy_r[i] <= 1; vs_r[i] <= cdb0_val; end
                    if (cdb1_en && !ps_rdy_r[i] && cdb1_tag == ps_r[i])
                        begin ps_rdy_r[i] <= 1; vs_r[i] <= cdb1_val; end
                    if (cdb0_en && !pt_rdy_r[i] && cdb0_tag == pt_r[i])
                        begin pt_rdy_r[i] <= 1; vt_r[i] <= cdb0_val; end
                    if (cdb1_en && !pt_rdy_r[i] && cdb1_tag == pt_r[i])
                        begin pt_rdy_r[i] <= 1; vt_r[i] <= cdb1_val; end
                end
            end
            // issue
            if (iss_found) begin
                issue_en      <= 1;
                issue_op      <= op_r[iss_idx];
                issue_vs      <= vs_r[iss_idx];
                issue_vt      <= vt_r[iss_idx];
                issue_rob_tag <= rob_tag_r[iss_idx];
                valid_r[iss_idx] <= 0;
                free_cnt <= free_cnt + 1 - disp_en;
            end else begin
                free_cnt <= free_cnt - disp_en;
            end
            // dispatch
            if (disp_en && found_free) begin
                valid_r[free_slot]   <= 1;
                op_r[free_slot]      <= disp_op;
                ps_r[free_slot]      <= disp_ps;
                pt_r[free_slot]      <= disp_pt;
                ps_rdy_r[free_slot]  <= disp_ps_rdy;
                pt_rdy_r[free_slot]  <= disp_pt_rdy;
                vs_r[free_slot]      <= disp_vs;
                vt_r[free_slot]      <= disp_vt;
                rob_tag_r[free_slot] <= disp_rob_tag;
            end
        end
    end

endmodule