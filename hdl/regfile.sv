module reg_file(
    input clk,
    input reset,
    input [63:0] data,
    input [4:0] raddr1,
    input [4:0] raddr2,
    input [4:0] raddr3,
    input [4:0] waddr,
    input write,
    output [63:0] r1,
    output [63:0] r2,
    output [63:0] r3
);

reg [63:0] registers [0:31];

// alu sees reg values in the same cycle
assign r1 = registers[raddr1];
assign r2 = registers[raddr2];
assign r3 = registers[raddr3];


integer i;
always @(posedge clk) begin
    if (reset) begin
        for (i = 0; i < 31; i = i + 1)
            registers[i] <= 64'd0;
        // overwrites reg[31] with MEM_SIZE on reset edge
        registers[31] <= 64'd524288;
    end else if (write) begin
        registers[waddr] <= data;
    end
end

endmodule