`ifndef PC_START
  `define PC_START 64'h2000
`endif
`ifndef MEM_SIZE
  `define MEM_SIZE (512 * 1024)
`endif

module fetch (
    input clk,
    input reset,
    input halt,
    input advance,  //!stall && !hlt

    input is_jump,
    input is_branch,
    input is_brgt,
    input is_brr_reg,
    input is_brr_imm,
    input is_return,
    input is_call,

    input        branch_cond,
    input [63:0] data1,
    input [63:0] data2,
    input [63:0] immediate,
    input [63:0] mem_rdata,

    output [63:0] pc
);

  reg [63:0] pc_reg;
  assign pc = pc_reg;

  wire taken = (is_branch && branch_cond) || is_jump;

  wire [63:0] next_pc =
      is_brr_imm             ? (pc_reg + immediate) :
      is_brr_reg             ? (pc_reg + data1)     :
      (is_branch && is_brgt) ? data1                :
      is_branch              ? data2                :
                               data1;               // jump / call

  wire [63:0] nxt   = (next_pc  >= `MEM_SIZE) ? `PC_START : next_pc;
  wire [63:0] ret    = (mem_rdata >= `MEM_SIZE) ? `PC_START : mem_rdata;
  wire [63:0] seq    = (pc_reg + 64'd4 >= `MEM_SIZE) ? `PC_START : pc_reg + 64'd4;

  always @(posedge clk) begin
    if (reset) begin
      pc_reg <= `PC_START;
    end else if (!halt) begin
      // redirect > advance > hold
      if (is_return)
        pc_reg <= ret;
      else if (taken)
        pc_reg <= nxt;
      else if (advance)
        pc_reg <= seq;
      // else: stall — hold pc_reg
    end
  end

endmodule