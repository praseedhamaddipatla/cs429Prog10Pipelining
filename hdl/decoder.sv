// decoder.sv — instruction decode (FIXED)
// Fixes:
//  - call: waddr=rd (not r31); raddr2=r31 so dispatch can access SP for stack store
//  - return: raddr1=r31 so dispatch has SP value for stack load

module decoder (
    input      [31:0] instr,
    output reg [ 4:0] raddr1,      // arch src reg a
    output reg [ 4:0] raddr2,      // arch src reg b  (also r31 for call)
    output reg [ 4:0] waddr,       // arch dest reg
    output reg [63:0] immediate,
    output reg [ 4:0] op,          // alu/fpu op code
    output reg        use_imm,     // b operand = imm
    output reg        write,       // instr writes a reg
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
    output reg [ 4:0] rt_addr      // third src for brgt
);

  wire [ 4:0] opcode = instr[31:27];
  wire [ 4:0] rd = instr[26:22];
  wire [ 4:0] rs = instr[21:17];
  wire [ 4:0] rt = instr[16:12];
  wire [11:0] imm12 = instr[11:0];

  wire [63:0] imm_signed = {{52{imm12[11]}}, imm12};
  wire [63:0] imm_unsigned = {52'd0, imm12};

  // alu op codes
  localparam ADD = 5'd0;
  localparam SUB = 5'd1;
  localparam MUL = 5'd2;
  localparam DIV = 5'd3;
  localparam AND = 5'd4;
  localparam OR = 5'd5;
  localparam XOR = 5'd6;
  localparam NOT = 5'd7;
  localparam SHR = 5'd8;
  localparam SHL = 5'd9;
  localparam ADDF = 5'd10;
  localparam SUBF = 5'd11;
  localparam MULF = 5'd12;
  localparam DIVF = 5'd13;
  localparam CMPNZ = 5'd14;
  localparam CMPGT = 5'd15;

  always @(*) begin
    // defaults — nop
    raddr1     = 5'd0;
    raddr2     = 5'd0;
    waddr      = 5'd0;
    rt_addr    = 5'd0;
    immediate  = 64'd0;
    op         = ADD;
    use_imm    = 0;
    write      = 0;
    is_load    = 0;
    is_store   = 0;
    is_branch  = 0;
    is_brgt    = 0;
    is_jump    = 0;
    is_brr_reg = 0;
    is_brr_imm = 0;
    is_return  = 0;
    is_call    = 0;
    is_halt    = 0;
    is_mov_reg = 0;
    is_mov_imm = 0;

    case (opcode)
      5'h00: begin  // and rd, rs, rt
        raddr1 = rs;
        raddr2 = rt;
        waddr = rd;
        op = AND;
        write = 1;
      end
      5'h01: begin  // or
        raddr1 = rs;
        raddr2 = rt;
        waddr = rd;
        op = OR;
        write = 1;
      end
      5'h02: begin  // xor
        raddr1 = rs;
        raddr2 = rt;
        waddr = rd;
        op = XOR;
        write = 1;
      end
      5'h03: begin  // not rd, rs
        raddr1 = rs;
        waddr = rd;
        op = NOT;
        write = 1;
      end

      5'h04: begin  // shftr rd, rs, rt
        raddr1 = rs;
        raddr2 = rt;
        waddr = rd;
        op = SHR;
        write = 1;
      end
      5'h05: begin  // shftri rd, L
        raddr1 = rd;
        waddr = rd;
        immediate = imm_unsigned;
        op = SHR;
        use_imm = 1;
        write = 1;
      end
      5'h06: begin  // shftl
        raddr1 = rs;
        raddr2 = rt;
        waddr = rd;
        op = SHL;
        write = 1;
      end
      5'h07: begin  // shftli rd, L
        raddr1 = rd;
        waddr = rd;
        immediate = imm_unsigned;
        op = SHL;
        use_imm = 1;
        write = 1;
      end

      5'h08: begin  // br rd — absolute jump to rd
        raddr1  = rd;
        is_jump = 1;
      end
      5'h09: begin  // brr rd — pc-relative via reg
        raddr1 = rd;
        is_jump = 1;
        is_brr_reg = 1;
      end
      5'h0A: begin  // brr L — pc-relative via imm
        immediate = imm_signed;
        is_jump = 1;
        is_brr_imm = 1;
      end
      5'h0B: begin  // brnz rd, rs  (branch if rs != 0, target=rd)
        raddr1 = rs;
        raddr2 = rd;
        op = CMPNZ;
        is_branch = 1;
      end
      5'h0C: begin  // call rd
        // rd = pc+4 (link register = destination register)
        // also pushes pc+4 to stack via mem[r31-8]
        // raddr1=rd (jump target), raddr2=r31 (stack pointer for push)
        raddr1  = rd;
        raddr2  = 5'd31;
        waddr   = rd;
        is_jump = 1;
        is_call = 1;
        write   = 1;
      end
      5'h0D: begin  // return
        // pc = mem[r31-8], no register write
        // raddr1=r31 (stack pointer for load)
        raddr1 = 5'd31;
        is_jump = 1;
        is_return = 1;
      end
      5'h0E: begin  // brgt rd, rs, rt
        raddr1 = rd;
        raddr2 = rs;
        rt_addr = rt;
        op = CMPGT;
        is_branch = 1;
        is_brgt = 1;
      end
      5'h0F: begin  // halt
        is_halt = 1;
      end

      5'h10: begin  // load rd, (rs)(L)
        raddr1 = rs;
        waddr = rd;
        immediate = imm_signed;
        is_load = 1;
        write = 1;
      end
      5'h11: begin  // mov rd, rs
        raddr1 = rs;
        waddr = rd;
        is_mov_reg = 1;
        write = 1;
      end
      5'h12: begin  // mov rd, L  (rd = rd + L, i.e. addi semantics for upper bits)
        raddr1 = rd;
        waddr = rd;
        immediate = imm_unsigned;
        is_mov_imm = 1;
        use_imm = 1;
        write = 1;
      end
      5'h13: begin  // store (rd)(L), rs
        raddr1 = rd;
        raddr2 = rs;
        immediate = imm_signed;
        is_store = 1;
      end

      5'h14: begin  // addf
        raddr1 = rs;
        raddr2 = rt;
        waddr = rd;
        op = ADDF;
        write = 1;
      end
      5'h15: begin  // subf
        raddr1 = rs;
        raddr2 = rt;
        waddr = rd;
        op = SUBF;
        write = 1;
      end
      5'h16: begin  // mulf
        raddr1 = rs;
        raddr2 = rt;
        waddr = rd;
        op = MULF;
        write = 1;
      end
      5'h17: begin  // divf
        raddr1 = rs;
        raddr2 = rt;
        waddr = rd;
        op = DIVF;
        write = 1;
      end

      5'h18: begin  // add
        raddr1 = rs;
        raddr2 = rt;
        waddr = rd;
        op = ADD;
        write = 1;
      end
      5'h19: begin  // addi
        raddr1 = rd;
        waddr = rd;
        immediate = imm_signed;
        op = ADD;
        use_imm = 1;
        write = 1;
      end
      5'h1A: begin  // sub
        raddr1 = rs;
        raddr2 = rt;
        waddr = rd;
        op = SUB;
        write = 1;
      end
      5'h1B: begin  // subi
        raddr1 = rd;
        waddr = rd;
        immediate = imm_signed;
        op = SUB;
        use_imm = 1;
        write = 1;
      end
      5'h1C: begin  // mul
        raddr1 = rs;
        raddr2 = rt;
        waddr = rd;
        op = MUL;
        write = 1;
      end
      5'h1D: begin  // div
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