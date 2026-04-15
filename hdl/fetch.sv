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


// fetch unit — dual-issue, with branch prediction
// outputs up to 2 instrs per cycle into fetch queue
module fetch_unit (
    input         clk,
    input         reset
);
endmodule