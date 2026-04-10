`define MEM_SIZE (512 * 1024)
`define PC_START 64'h2000

`include "hdl/alu.sv"
`include "hdl/regfile.sv"
`include "hdl/decoder.sv"
`include "hdl/fetch.sv"
`include "hdl/mem_module.sv"

module tinker_core (
    input clk,
    input reset,
    output reg hlt
);

  parameter S0 = 3'd0;
  parameter S1 = 3'd1;
  parameter S2 = 3'd2;
  parameter S3 = 3'd3;
  parameter S4 = 3'd4;

  reg [2:0] state, next_state;

  wire [63:0] pc;
  wire [31:0] instr;

  wire [4:0] raddr1, raddr2, waddr;
  wire [63:0] immediate;
  wire [4:0] op;
  wire use_imm, write;
  wire is_load, is_store;
  wire is_branch, is_brgt, is_jump;
  wire is_brr_reg, is_brr_imm;
  wire is_return, is_call;
  wire is_halt;
  wire is_mov_reg, is_mov_imm;
  wire [4:0] rt_addr;

  wire [63:0] data1, data2, data3;

  reg [63:0] alu_a, alu_b;
  wire [63:0] alu_result;

  wire [63:0] mem_rdata;
  reg  [63:0] mem_out_reg;

  reg [31:0] IR;
  reg [63:0] pc_reg;
  reg [63:0] a_reg, b_reg, c_reg;
  reg [63:0] imm_reg;

  reg write_r, use_imm_r;
  reg is_load_r, is_store_r;
  reg is_branch_r, is_jump_r;
  reg is_call_r, is_return_r;
  reg is_halt_r;
  reg is_mov_reg_r, is_mov_imm_r;
  reg is_brgt_r, is_brr_reg_r, is_brr_imm_r;

  // r31 latched at S1
  reg [63:0] r31_latched;

  //  STATE
  always @(posedge clk) begin
    if (reset) state <= S0;
    else if (!hlt) state <= next_state;
  end

  always @(*) begin
    case (state)
      S0: next_state = S1;
      S1: next_state = S2;
      S2: begin
        if (is_load_r || is_store_r || is_call_r || is_return_r)
          next_state = S3;
        else if (write_r && !is_branch_r)
          next_state = S4;
        else
          next_state = S0;
      end
      S3: begin
        if (is_load_r || is_return_r)
          next_state = S4;
        else
          next_state = S0;  // store and call both exit to S0
      end
      default: next_state = S0;
    endcase
  end

  //  IR latch
  always @(posedge clk) begin
    if (state == S0) begin
      IR     <= instr;
      pc_reg <= pc;
    end
  end

  wire [31:0] dec_instr = IR;

  //  ctrl latch
  always @(posedge clk) begin
    if (reset) begin
      write_r <= 0; use_imm_r <= 0;
      is_load_r <= 0; is_store_r <= 0;
      is_branch_r <= 0; is_jump_r <= 0;
      is_call_r <= 0; is_return_r <= 0;
      is_halt_r <= 0;
      is_mov_reg_r <= 0; is_mov_imm_r <= 0;
      is_brgt_r <= 0; is_brr_reg_r <= 0; is_brr_imm_r <= 0;
      r31_latched <= 64'd524288;
    end else if (state == S1) begin
      write_r      <= write;
      use_imm_r    <= use_imm;
      is_load_r    <= is_load;
      is_store_r   <= is_store;
      is_branch_r  <= is_branch;
      is_jump_r    <= is_jump;
      is_call_r    <= is_call;
      is_return_r  <= is_return;
      is_halt_r    <= is_halt;
      is_mov_reg_r <= is_mov_reg;
      is_mov_imm_r <= is_mov_imm;
      is_brgt_r    <= is_brgt;
      is_brr_reg_r <= is_brr_reg;
      is_brr_imm_r <= is_brr_imm;
      a_reg        <= data1;
      b_reg        <= data2;
      imm_reg      <= immediate;
      r31_latched  <= reg_file.registers[31];
    end
  end

  //  halt
  always @(posedge clk) begin
    if (reset) hlt <= 0;
    else if (state == S1 && is_halt) hlt <= 1;
  end

  //  ALU
  always @(*) begin
    alu_a = a_reg;
    alu_b = use_imm_r ? imm_reg : b_reg;
  end

  alu alu_inst (
      .a(alu_a),
      .b(alu_b),
      .op(op),
      .result(alu_result)
  );

  always @(posedge clk) begin
    if (state == S2) c_reg <= alu_result;
  end

  //  memroy
  // latched r31
  wire [63:0] stack_top = r31_latched - 64'd8;

  wire [63:0] mem_data_addr =
      (is_call_r || is_return_r) ? stack_top : (a_reg + imm_reg);

  wire [63:0] mem_wdata =
      is_call_r ? (pc_reg + 64'd4) : b_reg;

  wire mem_we = (state == S3) && (is_store_r || is_call_r);

  mem_module #(.MEM_SIZE(`MEM_SIZE)) memory (
      .clk(clk),
      .fetch_addr(pc),
      .instr_out(instr),
      .data_addr(mem_data_addr),
      .write_data(mem_wdata),
      .we(mem_we),
      .read_data(mem_rdata)
  );

  always @(posedge clk) begin
    if (state == S3) mem_out_reg <= mem_rdata;
  end

  //  writeback
  wire [63:0] wb_data =
      is_load_r    ? mem_out_reg :
      is_mov_reg_r ? a_reg :
      is_mov_imm_r ? ((a_reg & ~64'hFFF) | imm_reg) :
      c_reg;

  wire reg_we =
      (state == S4) && write_r &&
      !is_call_r && !is_return_r;

  //  fetch
  wire advance =
      (state == S4) ||
      (state == S3 && !is_load_r && !is_return_r && !is_call_r) ||
      (state == S2 &&
       !is_load_r && !is_store_r && !is_call_r && !is_return_r &&
       !(write_r && !is_branch_r));

  fetch fetch_inst (
      .clk(clk),
      .reset(reset),
      .halt(hlt),
      .advance(advance),

      .is_jump   (is_jump_r && !is_return_r && (state == S2)),
      .is_branch (is_branch_r && (state == S2)),
      .is_brgt   (is_brgt_r),
      .is_brr_reg(is_brr_reg_r),
      .is_brr_imm(is_brr_imm_r),

      .is_return(is_return_r && (state == S4)),
      .is_call  (is_call_r   && (state == S2)),

      .branch_cond(alu_result[0]),
      .data1   (a_reg),
      .data2   (b_reg),
      .immediate(imm_reg),
      .mem_rdata(mem_out_reg),
      .pc(pc)
  );

  //  decoder
  decoder dec_inst (
      .instr     (dec_instr),
      .raddr1    (raddr1),
      .raddr2    (raddr2),
      .waddr     (waddr),
      .immediate (immediate),
      .op        (op),
      .use_imm   (use_imm),
      .write     (write),
      .is_load   (is_load),
      .is_store  (is_store),
      .is_branch (is_branch),
      .is_brgt   (is_brgt),
      .is_jump   (is_jump),
      .is_brr_reg(is_brr_reg),
      .is_brr_imm(is_brr_imm),
      .is_return (is_return),
      .is_call   (is_call),
      .is_halt   (is_halt),
      .is_mov_reg(is_mov_reg),
      .is_mov_imm(is_mov_imm),
      .rt_addr   (rt_addr)
  );

  //  regfile
  reg_file reg_file (
      .clk   (clk),
      .reset (reset),
      .raddr1(raddr1),
      .raddr2(raddr2),
      .raddr3(rt_addr),
      .waddr (waddr),
      .data  (wb_data),
      .write (reg_we),
      .r1    (data1),
      .r2    (data2),
      .r3    (data3)
  );

endmodule