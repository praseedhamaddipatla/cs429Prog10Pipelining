`define MEM_SIZE (512 * 1024)
`define PC_START 64'h2000
 
`include "hdl/alu.sv"
`include "hdl/regfile.sv"
`include "hdl/decoder.sv"
`include "hdl/fetch.sv"
`include "hdl/mem_module.sv"
 
module tinker_core (
    input  clk,
    input  reset,
    output reg hlt
);
 
// pc/fetch
wire [63:0] pc_if;       // current fetch address (from fetch unit)
wire [31:0] instr_if;    // raw instruction from memory
 
// if/id pipeline reg
reg [31:0]  if_id_instr;
reg [63:0]  if_id_pc;
reg         if_id_valid;  // 0 = bubble
 
// decode wires
wire [4:0]  d_raddr1, d_raddr2, d_waddr, d_rt_addr;
wire [63:0] d_immediate;
wire [4:0]  d_op;
wire        d_use_imm, d_write;
wire        d_is_load,  d_is_store;
wire        d_is_branch, d_is_brgt, d_is_jump;
wire        d_is_brr_reg, d_is_brr_imm;
wire        d_is_return, d_is_call;
wire        d_is_halt;
wire        d_is_mov_reg, d_is_mov_imm;
 
// reg file read ports
wire [63:0] d_data1, d_data2, d_data3;
 
// id/ex pipeline reg
reg [63:0]  id_ex_pc;
reg [63:0]  id_ex_data1, id_ex_data2;
reg [63:0]  id_ex_imm;
reg [4:0]   id_ex_op;
reg [4:0]   id_ex_waddr;
reg         id_ex_use_imm, id_ex_write;
reg         id_ex_is_load,  id_ex_is_store;
reg         id_ex_is_branch, id_ex_is_brgt, id_ex_is_jump;
reg         id_ex_is_brr_reg, id_ex_is_brr_imm;
reg         id_ex_is_return, id_ex_is_call;
reg         id_ex_is_halt;
reg         id_ex_is_mov_reg, id_ex_is_mov_imm;
reg         id_ex_valid;
 
// ex wires
reg  [63:0] ex_alu_a, ex_alu_b;
wire [63:0] ex_alu_result;
 
// forwarded values from later stages
wire [63:0] fwd_ex_val;
wire [63:0] fwd_mem_val;
 
// forward mux selects
reg [1:0] fwd_sel_a, fwd_sel_b;
 
// ex/mem
reg [63:0]  ex_mem_alu_result;
reg [63:0]  ex_mem_data2;      // store data
reg [63:0]  ex_mem_pc;
reg [4:0]   ex_mem_waddr;
reg         ex_mem_write;
reg         ex_mem_is_load,  ex_mem_is_store;
reg         ex_mem_is_branch, ex_mem_is_brgt, ex_mem_is_jump;
reg         ex_mem_is_brr_reg, ex_mem_is_brr_imm;
reg         ex_mem_is_return, ex_mem_is_call;
reg         ex_mem_is_halt;
reg         ex_mem_is_mov_reg, ex_mem_is_mov_imm;
reg [63:0]  ex_mem_data1;      // rs1 value (for mov/branch)
reg [63:0]  ex_mem_imm;
reg         ex_mem_branch_cond;
reg         ex_mem_valid;
 
// me wires
wire [63:0] mem_rdata; 
 
// stack pointer latched
wire [63:0] mem_stack_top  = id_ex_data1 - 64'd8;
reg  [63:0] ex_mem_r31;
 
wire [63:0] mem_data_addr  =
    (ex_mem_is_call || ex_mem_is_return) ? (ex_mem_r31 - 64'd8)
                                         : (ex_mem_alu_result); 
wire [63:0] mem_wdata      =
    ex_mem_is_call  ? (ex_mem_pc + 64'd4)  // push return address
                    : ex_mem_data2;          // store rs2
 
wire        mem_we         = ex_mem_is_store || ex_mem_is_call;
 
// mem/wb pipeline reg
reg [63:0]  mem_wb_result;     // final writeback
reg [4:0]   mem_wb_waddr;
reg         mem_wb_write;
reg         mem_wb_is_load;    // if 1, mem_wb_result was loaded from mem
reg         mem_wb_valid;
 
// hazard/stall/flush control
wire load_use_stall =
    id_ex_valid && id_ex_is_load &&
    (id_ex_waddr == d_raddr1 || id_ex_waddr == d_raddr2);
 
// ctrl hazard
wire branch_taken_ex;
wire flush_if_id;
 
// return
wire return_resolving   = ex_mem_valid && ex_mem_is_return;
wire flush_id_ex_return;
 
// stall
wire stall = load_use_stall;
 
// forwarding
assign fwd_ex_val  =
    ex_mem_is_mov_reg ? ex_mem_data1  :
    ex_mem_is_mov_imm ? ((ex_mem_data1 & ~64'hFFF) | ex_mem_imm) :
    ex_mem_alu_result;
 
assign fwd_mem_val = mem_wb_result;
 
// forwarding mux selects port A
always @(*) begin
    if (ex_mem_valid && ex_mem_write && !ex_mem_is_load &&
        ex_mem_waddr != 5'd0 && ex_mem_waddr == id_ex_waddr)
        fwd_sel_a = 2'b10;   // forward from EX/MEM
    else if (mem_wb_valid && mem_wb_write &&
             mem_wb_waddr != 5'd0 && mem_wb_waddr == id_ex_waddr)
        fwd_sel_a = 2'b01;   // forward from MEM/WB
    else
        fwd_sel_a = 2'b00;
end
 
// forwarding mux selects port B
reg [4:0] id_ex_raddr2;
 
always @(*) begin
    if (ex_mem_valid && ex_mem_write && !ex_mem_is_load &&
        ex_mem_waddr != 5'd0 && ex_mem_waddr == id_ex_raddr2)
        fwd_sel_b = 2'b10;
    else if (mem_wb_valid && mem_wb_write &&
             mem_wb_waddr != 5'd0 && mem_wb_waddr == id_ex_raddr2)
        fwd_sel_b = 2'b01;
    else
        fwd_sel_b = 2'b00;
end
 
// apply forwarding muxes
always @(*) begin
    case (fwd_sel_a)
        2'b10:   ex_alu_a = fwd_ex_val;
        2'b01:   ex_alu_a = fwd_mem_val;
        default: ex_alu_a = id_ex_data1;
    endcase
 
    // bypass port B forwarding if immediate
    if (id_ex_use_imm)
        ex_alu_b = id_ex_imm;
    else begin
        case (fwd_sel_b)
            2'b10:   ex_alu_b = fwd_ex_val;
            2'b01:   ex_alu_b = fwd_mem_val;
            default: ex_alu_b = id_ex_data2;
        endcase
    end
end
 
// sub module inst
 
// ALU 
alu alu_inst (
    .a     (ex_alu_a),
    .b     (ex_alu_b),
    .op    (id_ex_op),
    .result(ex_alu_result)
);
 
// mem 
mem_module #(.MEM_SIZE(`MEM_SIZE)) memory (
    .clk        (clk),
    .fetch_addr (pc_if),
    .instr_out  (instr_if),
    .data_addr  (mem_data_addr),
    .write_data (mem_wdata),
    .we         (mem_we),
    .read_data  (mem_rdata)
);
 
// decoder 
decoder dec_inst (
    .instr     (if_id_instr),
    .raddr1    (d_raddr1),
    .raddr2    (d_raddr2),
    .waddr     (d_waddr),
    .immediate (d_immediate),
    .op        (d_op),
    .use_imm   (d_use_imm),
    .write     (d_write),
    .is_load   (d_is_load),
    .is_store  (d_is_store),
    .is_branch (d_is_branch),
    .is_brgt   (d_is_brgt),
    .is_jump   (d_is_jump),
    .is_brr_reg(d_is_brr_reg),
    .is_brr_imm(d_is_brr_imm),
    .is_return (d_is_return),
    .is_call   (d_is_call),
    .is_halt   (d_is_halt),
    .is_mov_reg(d_is_mov_reg),
    .is_mov_imm(d_is_mov_imm),
    .rt_addr   (d_rt_addr)
);
 
// reg file 
wire [63:0] wb_data =
    mem_wb_is_load ? mem_wb_result   // already holds mem_rdata latched
                   : mem_wb_result;
 
wire reg_we = mem_wb_valid && mem_wb_write;
 
reg_file reg_file (
    .clk   (clk),
    .reset (reset),
    .raddr1(d_raddr1),
    .raddr2(d_raddr2),
    .raddr3(d_rt_addr),        // for rt (mov source)
    .waddr (mem_wb_waddr),
    .data  (wb_data),
    .write (reg_we),
    .r1    (d_data1),
    .r2    (d_data2),
    .r3    (d_data3)
);
 
// fetch
wire advance_fetch = !stall && !hlt;
 
// br condition from ex stage
assign branch_taken_ex =
    ex_mem_valid && ex_mem_is_branch &&
    ((ex_mem_is_brgt && ex_mem_branch_cond) ||
     (!ex_mem_is_brgt && ex_mem_branch_cond));
 
// flush IF/ID when branch resolves
assign flush_if_id      = ex_mem_valid && (ex_mem_is_jump || branch_taken_ex || ex_mem_is_call);
assign flush_id_ex_return = ex_mem_valid && ex_mem_is_return && !mem_wb_valid;
 
fetch fetch_inst (
    .clk       (clk),
    .reset     (reset),
    .halt      (hlt),
    .advance   (advance_fetch),
 
    .is_jump   (ex_mem_valid && ex_mem_is_jump && !ex_mem_is_return),
    .is_branch (ex_mem_valid && ex_mem_is_branch),
    .is_brgt   (ex_mem_is_brgt),
    .is_brr_reg(ex_mem_is_brr_reg),
    .is_brr_imm(ex_mem_is_brr_imm),
 
    .is_return (mem_wb_valid && mem_wb_is_load && return_resolving),
    .is_call   (ex_mem_valid && ex_mem_is_call),
 
    .branch_cond(ex_mem_branch_cond),
    .data1     (ex_mem_data1),
    .data2     (ex_mem_data2),
    .immediate (ex_mem_imm),
    .mem_rdata (mem_rdata),   // return target
    .pc        (pc_if)
);
 
// pipeline reg updates
 
// IF/ID 
always @(posedge clk) begin
    if (reset || flush_if_id || flush_id_ex_return) begin
        if_id_instr <= 32'd0;
        if_id_pc    <= 64'd0;
        if_id_valid <= 1'b0;
    end else if (!stall) begin
        if_id_instr <= instr_if;
        if_id_pc    <= pc_if;
        if_id_valid <= 1'b1;
    end
    // stall
end
 
// ID/EX 
always @(posedge clk) begin
    if (reset || stall || flush_id_ex_return) begin
        // insert bubble
        id_ex_valid      <= 1'b0;
        id_ex_write      <= 1'b0;
        id_ex_is_load    <= 1'b0;
        id_ex_is_store   <= 1'b0;
        id_ex_is_branch  <= 1'b0;
        id_ex_is_jump    <= 1'b0;
        id_ex_is_call    <= 1'b0;
        id_ex_is_return  <= 1'b0;
        id_ex_is_halt    <= 1'b0;
        id_ex_is_mov_reg <= 1'b0;
        id_ex_is_mov_imm <= 1'b0;
        id_ex_is_brgt    <= 1'b0;
        id_ex_is_brr_reg <= 1'b0;
        id_ex_is_brr_imm <= 1'b0;
        id_ex_waddr      <= 5'd0;
        id_ex_raddr2     <= 5'd0;
    end else begin
        id_ex_valid      <= if_id_valid;
        id_ex_pc         <= if_id_pc;
        id_ex_data1      <= d_data1;
        id_ex_data2      <= d_data2;
        id_ex_imm        <= d_immediate;
        id_ex_op         <= d_op;
        id_ex_waddr      <= d_waddr;
        id_ex_raddr2     <= d_raddr2;
        id_ex_use_imm    <= d_use_imm;
        id_ex_write      <= d_write;
        id_ex_is_load    <= d_is_load;
        id_ex_is_store   <= d_is_store;
        id_ex_is_branch  <= d_is_branch;
        id_ex_is_brgt    <= d_is_brgt;
        id_ex_is_jump    <= d_is_jump;
        id_ex_is_brr_reg <= d_is_brr_reg;
        id_ex_is_brr_imm <= d_is_brr_imm;
        id_ex_is_return  <= d_is_return;
        id_ex_is_call    <= d_is_call;
        id_ex_is_halt    <= d_is_halt;
        id_ex_is_mov_reg <= d_is_mov_reg;
        id_ex_is_mov_imm <= d_is_mov_imm;
    end
end
 
// EX/MEM 
always @(posedge clk) begin
    if (reset) begin
        ex_mem_valid <= 1'b0;
    end else begin
        ex_mem_valid      <= id_ex_valid;
        ex_mem_alu_result <= ex_alu_result;
        ex_mem_data1      <= ex_alu_a;       // rs1 (forwarded) for mov/branch
        ex_mem_data2      <= ex_alu_b;       // rs2 (forwarded) for store
        ex_mem_imm        <= id_ex_imm;
        ex_mem_pc         <= id_ex_pc;
        ex_mem_waddr      <= id_ex_waddr;
        ex_mem_write      <= id_ex_write;
        ex_mem_is_load    <= id_ex_is_load;
        ex_mem_is_store   <= id_ex_is_store;
        ex_mem_is_branch  <= id_ex_is_branch;
        ex_mem_is_brgt    <= id_ex_is_brgt;
        ex_mem_is_jump    <= id_ex_is_jump;
        ex_mem_is_brr_reg <= id_ex_is_brr_reg;
        ex_mem_is_brr_imm <= id_ex_is_brr_imm;
        ex_mem_is_return  <= id_ex_is_return;
        ex_mem_is_call    <= id_ex_is_call;
        ex_mem_is_halt    <= id_ex_is_halt;
        ex_mem_is_mov_reg <= id_ex_is_mov_reg;
        ex_mem_is_mov_imm <= id_ex_is_mov_imm;
        // branch condition: ALU result bit 0
        ex_mem_branch_cond <= ex_alu_result[0];
        // latch r31 for call/return memory address
        ex_mem_r31        <= d_data1;  // r31 is read when decode reads raddr1=31
    end
end
 
// MEM/WB 
// get final writeback
wire [63:0] mem_stage_result =
    ex_mem_is_load    ? mem_rdata  :
    ex_mem_is_mov_reg ? ex_mem_data1 :
    ex_mem_is_mov_imm ? ((ex_mem_data1 & ~64'hFFF) | ex_mem_imm) :
    ex_mem_alu_result;
 
always @(posedge clk) begin
    if (reset) begin
        mem_wb_valid <= 1'b0;
    end else begin
        mem_wb_valid   <= ex_mem_valid;
        mem_wb_result  <= mem_stage_result;
        mem_wb_waddr   <= ex_mem_waddr;
        mem_wb_write   <= ex_mem_write && !ex_mem_is_call && !ex_mem_is_return;
        mem_wb_is_load <= ex_mem_is_load;
    end
end
 
// halt
always @(posedge clk) begin
    if (reset)
        hlt <= 1'b0;
    else if (id_ex_valid && id_ex_is_halt)
        hlt <= 1'b1;
end
 
endmodule