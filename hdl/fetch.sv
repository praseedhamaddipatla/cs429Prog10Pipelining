// fetch.sv — fetch unit with branch prediction
// 2-bit saturating counter btb (branch target buffer)
// dual-issue: fetches 2 instrs per cycle when possible
// redirected by rob at commit (misprediction recovery)

`ifndef PC_START
  `define PC_START 64'h2000
`endif
`ifndef MEM_SIZE
  `define MEM_SIZE (512 * 1024)
`endif

// 2-bit saturating counter branch predictor
// 64-entry btb, direct-mapped by pc[7:2]
module branch_predictor #(
    parameter BTB_ENTRIES = 64
) (
    input         clk,
    input         reset,
    // query (fetch stage)
    input  [63:0] pc,
    output        pred_taken,
    output [63:0] pred_target,
    output        btb_hit,
    // update (commit stage — ground truth)
    input         upd_en,
    input  [63:0] upd_pc,
    input         upd_taken,
    input  [63:0] upd_target
);

    localparam IDX_BITS = $clog2(BTB_ENTRIES); // 6

    // btb entry: valid, tag, target, 2-bit counter
    reg                  btb_valid  [0:BTB_ENTRIES-1];
    reg [63-IDX_BITS:0]  btb_tag    [0:BTB_ENTRIES-1]; // upper pc bits
    reg [63:0]           btb_target [0:BTB_ENTRIES-1];
    reg [1:0]            btb_ctr    [0:BTB_ENTRIES-1]; // 2-bit sat counter

    wire [IDX_BITS-1:0] q_idx = pc[IDX_BITS+1:2]; // word-aligned index
    wire [63-IDX_BITS:0] q_tag = pc[63:IDX_BITS+2];

    // predict
    assign btb_hit     = btb_valid[q_idx] && (btb_tag[q_idx] == q_tag);
    assign pred_taken  = btb_hit && btb_ctr[q_idx][1]; // taken if counter >= 2
    assign pred_target = btb_target[q_idx];

    // update on commit
    wire [IDX_BITS-1:0] u_idx = upd_pc[IDX_BITS+1:2];
    wire [63-IDX_BITS:0] u_tag = upd_pc[63:IDX_BITS+2];

    integer i;
    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < BTB_ENTRIES; i = i + 1) begin
                btb_valid[i] <= 0;
                btb_ctr[i]   <= 2'b01; // weakly not-taken
            end
        end else if (upd_en) begin
            btb_valid[u_idx]  <= 1'b1;
            btb_tag[u_idx]    <= u_tag;
            btb_target[u_idx] <= upd_target;
            // update 2-bit saturating counter
            if (upd_taken)
                btb_ctr[u_idx] <= (btb_ctr[u_idx] == 2'b11) ? 2'b11 : btb_ctr[u_idx] + 1;
            else
                btb_ctr[u_idx] <= (btb_ctr[u_idx] == 2'b00) ? 2'b00 : btb_ctr[u_idx] - 1;
        end
    end

endmodule


// fetch.sv — fetch unit with branch prediction

`ifndef PC_START
  `define PC_START 64'h2000
`endif
`ifndef MEM_SIZE
  `define MEM_SIZE (512 * 1024)
`endif

// 2-bit saturating counter branch predictor
// 64-entry btb, direct-mapped by pc[7:2]
module branch_predictor #(
    parameter BTB_ENTRIES = 64
) (
    input         clk,
    input         reset,
    input  [63:0] pc,
    output        pred_taken,
    output [63:0] pred_target,
    output        btb_hit,
    input         upd_en,
    input  [63:0] upd_pc,
    input         upd_taken,
    input  [63:0] upd_target
);

    localparam IDX_BITS = $clog2(BTB_ENTRIES); // 6

    reg                  btb_valid  [0:BTB_ENTRIES-1];
    reg [63-IDX_BITS:0]  btb_tag    [0:BTB_ENTRIES-1];
    reg [63:0]           btb_target [0:BTB_ENTRIES-1];
    reg [1:0]            btb_ctr    [0:BTB_ENTRIES-1];

    wire [IDX_BITS-1:0] q_idx = pc[IDX_BITS+1:2];
    wire [63-IDX_BITS:0] q_tag = pc[63:IDX_BITS+2];

    assign btb_hit     = btb_valid[q_idx] && (btb_tag[q_idx] == q_tag);
    assign pred_taken  = btb_hit && btb_ctr[q_idx][1];
    assign pred_target = btb_target[q_idx];

    wire [IDX_BITS-1:0] u_idx = upd_pc[IDX_BITS+1:2];
    wire [63-IDX_BITS:0] u_tag = upd_pc[63:IDX_BITS+2];

    integer i;
    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < BTB_ENTRIES; i = i + 1) begin
                btb_valid[i] <= 0;
                btb_ctr[i]   <= 2'b01;
            end
        end else if (upd_en) begin
            btb_valid[u_idx]  <= 1'b1;
            btb_tag[u_idx]    <= u_tag;
            btb_target[u_idx] <= upd_target;
            if (upd_taken)
                btb_ctr[u_idx] <= (btb_ctr[u_idx] == 2'b11) ? 2'b11 : btb_ctr[u_idx] + 1;
            else
                btb_ctr[u_idx] <= (btb_ctr[u_idx] == 2'b00) ? 2'b00 : btb_ctr[u_idx] - 1;
        end
    end

endmodule


module fetch_unit #(
    parameter MEM_SIZE = `MEM_SIZE
) (
    input         clk,
    input         reset,
    input         halt,
    input         stall,
    input         redirect_en,
    input  [63:0] redirect_pc,
    input         bp_upd_en,
    input  [63:0] bp_upd_pc,
    input         bp_upd_taken,
    input  [63:0] bp_upd_target,
    input  [31:0] instr0_in,
    input  [31:0] instr1_in,
    output [63:0] fetch_pc0,
    output [63:0] fetch_pc1,
    output reg        out_valid0,
    output reg [31:0] out_instr0,
    output reg [63:0] out_pc0,
    output reg        out_pred_taken0,
    output reg [63:0] out_pred_target0,
    output reg        out_valid1,
    output reg [31:0] out_instr1,
    output reg [63:0] out_pc1,
    output reg        out_pred_taken1,
    output reg [63:0] out_pred_target1
);

    reg [63:0] pc_reg;

    wire        pred_taken0, pred_taken1;
    wire [63:0] pred_target0, pred_target1;
    wire        btb_hit0, btb_hit1;

    branch_predictor bp0 (
        .clk(clk), .reset(reset),
        .pc(pc_reg),
        .pred_taken(pred_taken0), .pred_target(pred_target0), .btb_hit(btb_hit0),
        .upd_en(bp_upd_en), .upd_pc(bp_upd_pc),
        .upd_taken(bp_upd_taken), .upd_target(bp_upd_target)
    );

    branch_predictor bp1 (
        .clk(clk), .reset(reset),
        .pc(pc_reg + 64'd4),
        .pred_taken(pred_taken1), .pred_target(pred_target1), .btb_hit(btb_hit1),
        .upd_en(bp_upd_en), .upd_pc(bp_upd_pc),
        .upd_taken(bp_upd_taken), .upd_target(bp_upd_target)
    );

    assign fetch_pc0 = pc_reg;
    assign fetch_pc1 = pc_reg + 64'd4;

    function [63:0] wrap_pc;
        input [63:0] p;
        wrap_pc = (p >= MEM_SIZE) ? `PC_START : p;
    endfunction

    always @(posedge clk) begin
        if (reset) begin
            pc_reg        <= `PC_START;
            out_valid0    <= 0;
            out_valid1    <= 0;
        end else if (halt) begin
            out_valid0    <= 0;
            out_valid1    <= 0;
        end else if (redirect_en) begin
            pc_reg     <= wrap_pc(redirect_pc);
            out_valid0 <= 0;
            out_valid1 <= 0;
        end else if (!stall) begin
            out_valid0       <= 1'b1;
            out_instr0       <= instr0_in;
            out_pc0          <= pc_reg;
            out_pred_taken0  <= pred_taken0;
            out_pred_target0 <= pred_target0;

            if (!pred_taken0) begin
                out_valid1       <= 1'b1;
                out_instr1       <= instr1_in;
                out_pc1          <= pc_reg + 64'd4;
                out_pred_taken1  <= pred_taken1;
                out_pred_target1 <= pred_target1;
                pc_reg <= wrap_pc(pred_taken1 ? pred_target1 : pc_reg + 64'd8);
            end else begin
                out_valid1 <= 1'b0;
                pc_reg     <= wrap_pc(pred_target0);
            end
        end
    end

endmodule