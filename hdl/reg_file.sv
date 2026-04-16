// regfile.sv — architectural + physical register files

module reg_file (
    input         clk,
    input         reset,
    input  [63:0] data,
    input  [4:0]  raddr1,
    input  [4:0]  raddr2,
    input  [4:0]  raddr3,
    input  [4:0]  waddr,
    input         write,
    output [63:0] r1,
    output [63:0] r2,
    output [63:0] r3
);

    reg [63:0] registers [0:31];

    // async read
    assign r1 = registers[raddr1];
    assign r2 = registers[raddr2];
    assign r3 = registers[raddr3];

    integer i;
    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < 31; i = i + 1)
                registers[i] <= 64'd0;
            // r31 = stack ptr init = MEM_SIZE
            registers[31] <= 64'd524288;
        end else if (write && waddr != 5'd0) begin
            registers[waddr] <= data;
        end
    end

endmodule


// physical reg file — 64 entries
// written by functional units via cdb broadcast
// read by reservation stations at issue
module phys_reg_file #(
    parameter NPHYS = 64
) (
    input         clk,
    input         reset,
    // write port (cdb broadcast)
    input                          wr_en,
    input  [$clog2(NPHYS)-1:0]    wr_addr,
    input  [63:0]                  wr_data,
    // read ports (2 per issue slot, dual-issue = 4 total)
    input  [$clog2(NPHYS)-1:0]    rd_addr0,
    input  [$clog2(NPHYS)-1:0]    rd_addr1,
    input  [$clog2(NPHYS)-1:0]    rd_addr2,
    input  [$clog2(NPHYS)-1:0]    rd_addr3,
    output [63:0]                  rd_data0,
    output [63:0]                  rd_data1,
    output [63:0]                  rd_data2,
    output [63:0]                  rd_data3
);

    reg [63:0] regs [0:NPHYS-1];

    // async read
    assign rd_data0 = regs[rd_addr0];
    assign rd_data1 = regs[rd_addr1];
    assign rd_data2 = regs[rd_addr2];
    assign rd_data3 = regs[rd_addr3];

    integer i;
    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < NPHYS; i = i + 1)
                regs[i] <= 64'd0;
        end else if (wr_en) begin
            regs[wr_addr] <= wr_data;
        end
    end

endmodule


// ready bits — tracks which phys regs have valid data
// set when fu writes, clear on rename (in-flight)
module phys_reg_ready #(
    parameter NPHYS = 64
) (
    input                       clk,
    input                       reset,
    // clear on rename (new in-flight dest)
    input                       clear_en,
    input [$clog2(NPHYS)-1:0]  clear_addr,
    // set on cdb broadcast
    input                       set_en,
    input [$clog2(NPHYS)-1:0]  set_addr,
    // query
    input [$clog2(NPHYS)-1:0]  q_addr0,
    input [$clog2(NPHYS)-1:0]  q_addr1,
    output                      q_ready0,
    output                      q_ready1
);

    reg ready [0:NPHYS-1];

    assign q_ready0 = ready[q_addr0];
    assign q_ready1 = ready[q_addr1];

    integer i;
    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < NPHYS; i = i + 1)
                ready[i] <= 1'b1; // all phys regs ready at reset
        end else begin
            if (clear_en) ready[clear_addr] <= 1'b0;
            if (set_en)   ready[set_addr]   <= 1'b1;
        end
    end

endmodule

// rat.sv — register alias table (rename stage)
// maps 32 arch regs -> 64 phys regs
// free list tracks available phys regs
// dual-issue: renames 2 instrs per cycle

module rat #(
    parameter NARCH  = 32,
    parameter NPHYS  = 64,
    parameter ARCH_W = 5,
    parameter PHYS_W = 6
) (
    input          clk,
    input          reset,

    // --- rename port 0 (instr 0 of dual-issue pair) ---
    input          rename0_en,
    input [ARCH_W-1:0] arch_rs0,   // src a arch reg
    input [ARCH_W-1:0] arch_rt0,   // src b arch reg
    input [ARCH_W-1:0] arch_rd0,   // dest arch reg
    input              has_dest0,  // instr writes a reg
    output [PHYS_W-1:0] phys_rs0,  // renamed src a
    output [PHYS_W-1:0] phys_rt0,  // renamed src b
    output [PHYS_W-1:0] phys_rd0,  // new phys dest
    output [PHYS_W-1:0] phys_old_rd0, // prev mapping (for free on commit)
    output              alloc_ok0, // free list had a reg

    // --- rename port 1 (instr 1 of dual-issue pair) ---
    // note: port 1 sees port 0's write to map table
    input          rename1_en,
    input [ARCH_W-1:0] arch_rs1,
    input [ARCH_W-1:0] arch_rt1,
    input [ARCH_W-1:0] arch_rd1,
    input              has_dest1,
    output [PHYS_W-1:0] phys_rs1,
    output [PHYS_W-1:0] phys_rt1,
    output [PHYS_W-1:0] phys_rd1,
    output [PHYS_W-1:0] phys_old_rd1,
    output              alloc_ok1,

    // --- commit port (free old phys reg, unblock arch->phys) ---
    input          commit0_en,
    input [PHYS_W-1:0] commit0_free,   // old phys reg to free

    input          commit1_en,
    input [PHYS_W-1:0] commit1_free,

    // --- mispredict/flush: restore snapshot ---
    input          flush_en,
    input [PHYS_W-1:0] flush_map [0:NARCH-1]  // checkpoint map
);

    // map table: arch reg -> current phys reg
    reg [PHYS_W-1:0] map [0:NARCH-1];

    // free list — circular fifo of available phys regs
    // init: phys 32..63 are free (0..31 mirror arch regs)
    localparam FREE_DEPTH = NPHYS - NARCH; // 32 free initially
    reg [PHYS_W-1:0] free_list [0:NPHYS-1]; // generous sizing
    reg [5:0] free_head, free_tail;
    reg [6:0] free_count; // up to 64

    // combinational lookup of current mappings
    // port 0 reads map as-is
    assign phys_rs0     = map[arch_rs0];
    assign phys_rt0     = map[arch_rt0];
    assign phys_old_rd0 = map[arch_rd0];
    // next free phys for port 0 dest
    assign phys_rd0     = (has_dest0 && alloc_ok0) ? free_list[free_head] : map[arch_rd0];
    assign alloc_ok0    = (free_count >= 1);

    // port 1 sees port 0's tentative update
    // if port 0 wrote arch_rd0 -> phys_rd0, port 1 must see that
    wire [PHYS_W-1:0] map1_rs1 = (rename0_en && has_dest0 && (arch_rd0 == arch_rs1)) 
                                  ? phys_rd0 : map[arch_rs1];
    wire [PHYS_W-1:0] map1_rt1 = (rename0_en && has_dest0 && (arch_rd0 == arch_rt1)) 
                                  ? phys_rd0 : map[arch_rt1];
    wire [PHYS_W-1:0] map1_old_rd1 = (rename0_en && has_dest0 && (arch_rd0 == arch_rd1))
                                  ? phys_rd0 : map[arch_rd1];

    assign phys_rs1     = map1_rs1;
    assign phys_rt1     = map1_rt1;
    assign phys_old_rd1 = map1_old_rd1;
    // port 1 needs second free reg (head+1 if port 0 also allocated)
    wire [5:0] free_head1 = (rename0_en && has_dest0 && alloc_ok0) 
                             ? (free_head + 1) : free_head;
    assign phys_rd1     = (has_dest1 && alloc_ok1) ? free_list[free_head1] : map1_old_rd1;
    assign alloc_ok1    = (has_dest1) ? (free_count >= (has_dest0 ? 2 : 1)) : 1'b1;

    integer i;
    always @(posedge clk) begin
        if (reset) begin
            // arch reg i maps to phys reg i (identity)
            for (i = 0; i < NARCH; i = i + 1)
                map[i] <= i; // Verilog automatically truncates 'i' to PHYS_W bits
            
            // free list: phys 32..63
            for (i = 0; i < FREE_DEPTH; i = i + 1)
                free_list[i] <= NARCH + i; // Logic fixed: no manual slicing needed
                
            free_head  <= 6'd0;
            free_tail  <= FREE_DEPTH[5:0];
            free_count <= FREE_DEPTH[6:0];

        end else if (flush_en) begin
            // restore checkpoint map on mispredict
            for (i = 0; i < NARCH; i = i + 1)
                map[i] <= flush_map[i];
            // note: free list recovery needs rob to return freed phys regs
            // simplified: trust that committed free regs are re-enqueued

        end else begin
            // commit frees — push old phys regs back
            if (commit0_en) begin
                free_list[free_tail] <= commit0_free;
                free_tail  <= free_tail + 1;
                free_count <= free_count + 1;
            end
            if (commit1_en) begin
                free_list[commit1_en ? (free_tail + commit0_en) : free_tail] <= commit1_free;
                free_tail  <= free_tail + 1 + commit0_en;
                free_count <= free_count + 1 + commit0_en;
            end

            // rename port 0
            if (rename0_en && has_dest0 && alloc_ok0) begin
                map[arch_rd0] <= free_list[free_head];
                free_head     <= free_head + 1;
                free_count    <= free_count - 1;
            end

            // rename port 1 (sees port 0's map update via combinational above)
            if (rename1_en && has_dest1 && alloc_ok1) begin
                map[arch_rd1] <= free_list[free_head1];
                free_head     <= free_head1 + 1;
                free_count    <= free_count - (has_dest0 && rename0_en ? 2 : 1);
            end
        end
    end

endmodule