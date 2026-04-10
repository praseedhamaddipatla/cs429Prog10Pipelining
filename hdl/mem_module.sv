// memory module
// has two ports: one for instr fetch and one for data load/store
module mem_module #(
    parameter MEM_SIZE = 512 * 1024
) (
    input clk,
    input [63:0] fetch_addr,
    output [31:0] instr_out,
    input [63:0] data_addr,
    input [63:0] write_data,
    input we,
    output [63:0] read_data
);

  // actual mem
  reg [7:0] bytes[0:MEM_SIZE-1];

  // instr fetch port
  assign instr_out = {
    bytes[fetch_addr+3], bytes[fetch_addr+2], bytes[fetch_addr+1], bytes[fetch_addr]
  };
  assign read_data = {
    bytes[data_addr+7],
    bytes[data_addr+6],
    bytes[data_addr+5],
    bytes[data_addr+4],
    bytes[data_addr+3],
    bytes[data_addr+2],
    bytes[data_addr+1],
    bytes[data_addr]
  };

  // load/store port
  always @(posedge clk) begin
    if (we) begin
      bytes[data_addr]   <= write_data[7:0];
      bytes[data_addr+1] <= write_data[15:8];
      bytes[data_addr+2] <= write_data[23:16];
      bytes[data_addr+3] <= write_data[31:24];
      bytes[data_addr+4] <= write_data[39:32];
      bytes[data_addr+5] <= write_data[47:40];
      bytes[data_addr+6] <= write_data[55:48];
      bytes[data_addr+7] <= write_data[63:56];
    end
  end
endmodule