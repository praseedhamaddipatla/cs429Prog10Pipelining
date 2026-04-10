// instr fetch & pc control
module fetch (
    input clk,
    input reset,
    input halt,
    input advance,

    // decoded instr type
    input is_jump,
    input is_branch,
    input is_brgt,
    input is_brr_reg,
    input is_brr_imm,
    input is_return,
    input is_call,

    // branch condition from ALU
    input branch_cond,

    // register values
    input [63:0] data1,
    input [63:0] data2,
    input [63:0] immediate,

    // return address from stack
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
                               data1;               // br / call

  always @(posedge clk) begin
    if (reset) begin
      pc_reg <= `PC_START;
    end else if (!halt) begin
      if (is_return) begin
        // return redirect
        if (mem_rdata >= `MEM_SIZE) pc_reg <= `PC_START;
        else pc_reg <= mem_rdata;
      end else if (taken) begin
        if (next_pc >= `MEM_SIZE) pc_reg <= `PC_START;
        else pc_reg <= next_pc;
      end else if (advance) begin
        if (pc_reg + 64'd4 >= `MEM_SIZE) pc_reg <= `PC_START;
        else pc_reg <= pc_reg + 64'd4;
      end
    end
  end
endmodule