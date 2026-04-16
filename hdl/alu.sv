
// alu.sv — pipelined alu + fpu
// int ops : 1-cycle latency (alu_int)
// fp ops  : 3-cycle latency (fpu_addsub / fpu_mul / fpu_div)
// results broadcast on valid_out pulse with rob_tag passthrough
// key fix: all intermediate pipeline values stored in top-level module regs,
//          not as local vars inside named blocks — ensures correct pipelining

// ---------------------------------------------------------------------------
// alu_int — single-cycle integer / branch / compare unit
// ---------------------------------------------------------------------------
module alu (
    input         clk,
    input         reset,
    input         valid_in,
    input  [63:0] a,
    input  [63:0] b,
    input  [4:0]  op,
    input  [5:0]  rob_tag_in,
    output reg        valid_out,
    output reg [63:0] result,
    output reg [5:0]  rob_tag_out
);
    // op codes
    localparam ADD   = 5'd0,  SUB   = 5'd1,  MUL  = 5'd2,  DIV  = 5'd3;
    localparam AND   = 5'd4,  OR    = 5'd5,  XOR  = 5'd6,  NOT  = 5'd7;
    localparam SHR   = 5'd8,  SHL   = 5'd9;
    localparam CMPNZ = 5'd14, CMPGT = 5'd15;

    reg [63:0] comb;
    always @(*) begin
        case (op)
            ADD:   comb = a + b;
            SUB:   comb = a - b;
            MUL:   comb = a * b;
            DIV:   comb = (b == 0) ? 64'd0 : $signed(a) / $signed(b);
            AND:   comb = a & b;
            OR:    comb = a | b;
            XOR:   comb = a ^ b;
            NOT:   comb = ~a;
            SHR:   comb = a >> b[5:0];
            SHL:   comb = a << b[5:0];
            CMPNZ: comb = (a != 0) ? 64'd1 : 64'd0;
            CMPGT: comb = ($signed(a) > $signed(b)) ? 64'd1 : 64'd0;
            default: comb = 64'd0;
        endcase
    end

    // register output — 1 cycle latency
    always @(posedge clk) begin
        if (reset) begin
            valid_out <= 0; result <= 0; rob_tag_out <= 0;
        end else begin
            valid_out   <= valid_in;
            result      <= comb;
            rob_tag_out <= rob_tag_in;
        end
    end
endmodule