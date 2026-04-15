// decoder.sv — instruction decode
// Yosys-compatible (localparams without types, integer loop vars)

module decoder (
    input      [31:0] instr,
    output reg [ 4:0] raddr1,
    output reg [ 4:0] raddr2,
    output reg [ 4:0] waddr,
    output reg [63:0] immediate,
    output reg [ 4:0] op,
    output reg        use_imm,
    output reg        write,
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
    output reg [ 4:0] rt_addr
);
  wire [ 4:0] opcode = instr[31:27];
  wire [ 4:0] rd = instr[26:22];
  wire [ 4:0] rs = instr[21:17];
  wire [ 4:0] rt = instr[16:12];
  wire [11:0] imm12 = instr[11:0];

  wire [63:0] imm_signed = {{52{imm12[11]}}, imm12};
  wire [63:0] imm_unsigned = {52'd0, imm12};

  localparam ADD = 5'd0, SUB = 5'd1, MUL = 5'd2, DIV = 5'd3;
  localparam AND = 5'd4, OR = 5'd5, XOR = 5'd6, NOT = 5'd7;
  localparam SHR = 5'd8, SHL = 5'd9;
  localparam ADDF = 5'd10, SUBF = 5'd11, MULF = 5'd12, DIVF = 5'd13;
  localparam CMPNZ = 5'd14, CMPGT = 5'd15;

  always @(*) begin
    raddr1 = 5'd0;
    raddr2 = 5'd0;
    waddr = 5'd0;
    rt_addr = 5'd0;
    immediate = 64'd0;
    op = ADD;
    use_imm = 0;
    write = 0;
    is_load = 0;
    is_store = 0;
    is_branch = 0;
    is_brgt = 0;
    is_jump = 0;
    is_brr_reg = 0;
    is_brr_imm = 0;
    is_return = 0;
    is_call = 0;
    is_halt = 0;
    is_mov_reg = 0;
    is_mov_imm = 0;
    case (opcode)
      5'h00: begin
        raddr1 = rs;
        raddr2 = rt;
        waddr = rd;
        op = AND;
        write = 1;
      end
      5'h01: begin
        raddr1 = rs;
        raddr2 = rt;
        waddr = rd;
        op = OR;
        write = 1;
      end
      5'h02: begin
        raddr1 = rs;
        raddr2 = rt;
        waddr = rd;
        op = XOR;
        write = 1;
      end
      5'h03: begin
        raddr1 = rs;
        waddr = rd;
        op = NOT;
        write = 1;
      end
      5'h04: begin
        raddr1 = rs;
        raddr2 = rt;
        waddr = rd;
        op = SHR;
        write = 1;
      end
      5'h05: begin
        raddr1 = rd;
        waddr = rd;
        immediate = imm_unsigned;
        op = SHR;
        use_imm = 1;
        write = 1;
      end
      5'h06: begin
        raddr1 = rs;
        raddr2 = rt;
        waddr = rd;
        op = SHL;
        write = 1;
      end
      5'h07: begin
        raddr1 = rd;
        waddr = rd;
        immediate = imm_unsigned;
        op = SHL;
        use_imm = 1;
        write = 1;
      end
      5'h08: begin
        raddr1  = rd;
        is_jump = 1;
      end
      5'h09: begin
        raddr1 = rd;
        is_jump = 1;
        is_brr_reg = 1;
      end
      5'h0A: begin
        immediate = imm_signed;
        is_jump = 1;
        is_brr_imm = 1;
      end
      5'h0B: begin
        raddr1 = rs;
        raddr2 = rd;
        op = CMPNZ;
        is_branch = 1;
      end
      5'h0C: begin
        raddr1  = rd;
        raddr2  = 5'd31;
        waddr   = rd;
        is_jump = 1;
        is_call = 1;
        write   = 1;
      end
      5'h0D: begin
        raddr1 = 5'd31;
        is_jump = 1;
        is_return = 1;
      end
      5'h0E: begin
        raddr1 = rd;
        raddr2 = rs;
        rt_addr = rt;
        op = CMPGT;
        is_branch = 1;
        is_brgt = 1;
      end
      5'h0F: begin
        is_halt = 1;
      end
      5'h10: begin
        raddr1 = rs;
        waddr = rd;
        immediate = imm_signed;
        is_load = 1;
        write = 1;
      end
      5'h11: begin
        raddr1 = rs;
        waddr = rd;
        is_mov_reg = 1;
        write = 1;
      end
      5'h12: begin
        raddr1 = rd;
        waddr = rd;
        immediate = imm_unsigned;
        is_mov_imm = 1;
        use_imm = 1;
        write = 1;
      end
      5'h13: begin
        raddr1 = rd;
        raddr2 = rs;
        immediate = imm_signed;
        is_store = 1;
      end
      5'h14: begin
        raddr1 = rs;
        raddr2 = rt;
        waddr = rd;
        op = ADDF;
        write = 1;
      end
      5'h15: begin
        raddr1 = rs;
        raddr2 = rt;
        waddr = rd;
        op = SUBF;
        write = 1;
      end
      5'h16: begin
        raddr1 = rs;
        raddr2 = rt;
        waddr = rd;
        op = MULF;
        write = 1;
      end
      5'h17: begin
        raddr1 = rs;
        raddr2 = rt;
        waddr = rd;
        op = DIVF;
        write = 1;
      end
      5'h18: begin
        raddr1 = rs;
        raddr2 = rt;
        waddr = rd;
        op = ADD;
        write = 1;
      end
      5'h19: begin
        raddr1 = rd;
        waddr = rd;
        immediate = imm_signed;
        op = ADD;
        use_imm = 1;
        write = 1;
      end
      5'h1A: begin
        raddr1 = rs;
        raddr2 = rt;
        waddr = rd;
        op = SUB;
        write = 1;
      end
      5'h1B: begin
        raddr1 = rd;
        waddr = rd;
        immediate = imm_signed;
        op = SUB;
        use_imm = 1;
        write = 1;
      end
      5'h1C: begin
        raddr1 = rs;
        raddr2 = rt;
        waddr = rd;
        op = MUL;
        write = 1;
      end
      5'h1D: begin
        raddr1 = rs;
        raddr2 = rt;
        waddr = rd;
        op = DIV;
        write = 1;
      end
      default: begin
      end
    endcase
  end
endmodule
