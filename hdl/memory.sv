// mem_module.sv — dual-port memory
`ifndef MEM_SIZE
  `define MEM_SIZE (512 * 1024)
`endif

module memory #(
    parameter MEM_SIZE = `MEM_SIZE
) (
    input         clk,
    input  [63:0] fetch_addr0,
    input  [63:0] fetch_addr1,
    output [31:0] instr_out0,
    output [31:0] instr_out1,
    input  [63:0] data_addr,
    input  [63:0] write_data,
    input         we,
    output [63:0] read_data
);

    reg [7:0] bytes [0:MEM_SIZE-1];

    // instr fetch — little-endian word
    assign instr_out0 = {bytes[fetch_addr0+3], bytes[fetch_addr0+2],
                         bytes[fetch_addr0+1], bytes[fetch_addr0]};
    assign instr_out1 = {bytes[fetch_addr1+3], bytes[fetch_addr1+2],
                         bytes[fetch_addr1+1], bytes[fetch_addr1]};

    // data load — little-endian 64-bit
    assign read_data = {bytes[data_addr+7], bytes[data_addr+6],
                        bytes[data_addr+5], bytes[data_addr+4],
                        bytes[data_addr+3], bytes[data_addr+2],
                        bytes[data_addr+1], bytes[data_addr]};

    // store (committed by rob)
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

// lsq.sv — load/store queue
// in-order mem access: loads/stores enqueued in program order
// store-to-load forwarding: load checks store queue for matching addr
// stores committed to mem only when rob commits the entry
// dual-issue: dispatches 2 ls ops per cycle

`ifndef PHYS_W
  `define PHYS_W 6
`endif

module lsq #(
    parameter NENTRIES = 8,
    parameter PHYS_W   = 6
) (
    input         clk,
    input         reset,

    // dispatch from rename/dispatch stage
    input         disp0_en,
    input         disp0_is_load,
    input         disp0_is_store,
    input  [PHYS_W-1:0] disp0_ps,   // base addr phys reg
    input  [PHYS_W-1:0] disp0_pt,   // store data phys reg (store only)
    input         disp0_ps_rdy,
    input         disp0_pt_rdy,
    input  [63:0] disp0_vs,
    input  [63:0] disp0_vt,
    input  [63:0] disp0_imm,        // addr offset
    input  [PHYS_W-1:0] disp0_pd,   // phys dest (load only)
    input  [5:0]  disp0_rob_tag,

    input         disp1_en,
    input         disp1_is_load,
    input         disp1_is_store,
    input  [PHYS_W-1:0] disp1_ps,
    input  [PHYS_W-1:0] disp1_pt,
    input         disp1_ps_rdy,
    input         disp1_pt_rdy,
    input  [63:0] disp1_vs,
    input  [63:0] disp1_vt,
    input  [63:0] disp1_imm,
    input  [PHYS_W-1:0] disp1_pd,
    input  [5:0]  disp1_rob_tag,

    // cdb for address/data resolution
    input         cdb0_en,
    input  [PHYS_W-1:0] cdb0_tag,
    input  [63:0] cdb0_val,
    input         cdb1_en,
    input  [PHYS_W-1:0] cdb1_tag,
    input  [63:0] cdb1_val,

    // commit signal from rob (allows store to write mem)
    input         commit_en,
    input  [5:0]  commit_rob_tag,

    // mem interface
    output reg [63:0] mem_addr,
    output reg [63:0] mem_wdata,
    output reg        mem_we,       // store write enable
    output reg        mem_re,       // load read enable (combinational)
    input  [63:0]     mem_rdata,

    // cdb output (load result broadcast)
    output reg        ld_done,
    output reg [PHYS_W-1:0] ld_pd,
    output reg [63:0] ld_val,
    output reg [5:0]  ld_rob_tag,

    output wire full
);

    // lsq entry
    reg        valid_r    [0:NENTRIES-1];
    reg        is_load_r  [0:NENTRIES-1];
    reg        is_store_r [0:NENTRIES-1];
    reg        addr_rdy_r [0:NENTRIES-1]; // base reg ready
    reg        data_rdy_r [0:NENTRIES-1]; // store data ready
    reg        committed_r[0:NENTRIES-1]; // rob committed this store
    reg [63:0] base_r     [0:NENTRIES-1];
    reg [63:0] data_r     [0:NENTRIES-1];
    reg [63:0] imm_r      [0:NENTRIES-1];
    reg [PHYS_W-1:0] ps_r [0:NENTRIES-1];
    reg [PHYS_W-1:0] pt_r [0:NENTRIES-1];
    reg [PHYS_W-1:0] pd_r [0:NENTRIES-1];
    reg [5:0]  rob_tag_r  [0:NENTRIES-1];

    // head/tail fifo pointers
    reg [3:0] head, tail;
    reg [4:0] cnt;
    assign full = (cnt >= NENTRIES - 1);

    // effective addr of each entry
    wire [63:0] eff_addr [0:NENTRIES-1];
    genvar g;
    generate
        for (g = 0; g < NENTRIES; g = g + 1)
            assign eff_addr[g] = base_r[g] + imm_r[g];
    endgenerate

    // head entry (oldest)
    wire [63:0] head_addr  = eff_addr[head];
    wire        head_valid = valid_r[head];
    wire        head_ld    = is_load_r[head];
    wire        head_st    = is_store_r[head];
    wire        head_addr_rdy = addr_rdy_r[head];
    wire        head_data_rdy = data_rdy_r[head];
    wire        head_committed = committed_r[head];

    // store-to-load forwarding:
    // scan all older stores for matching addr
    integer f;
    reg        fwd_hit;
    reg [63:0] fwd_val;
    always @(*) begin
        fwd_hit = 0; fwd_val = 64'd0;
        // check all valid stores older than head load
        for (f = 0; f < NENTRIES; f = f + 1) begin
            if (valid_r[f] && is_store_r[f] &&
                addr_rdy_r[f] && data_rdy_r[f] &&
                eff_addr[f] == head_addr) begin
                fwd_hit = 1;
                fwd_val = data_r[f];
            end
        end
    end

    integer i;
    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < NENTRIES; i = i + 1) valid_r[i] <= 0;
            head <= 0; tail <= 0; cnt <= 0;
            mem_we <= 0; mem_re <= 0; ld_done <= 0;
        end else begin
            mem_we <= 0;
            ld_done <= 0;

            // --- cdb snoop: resolve base addr + store data ---
            for (i = 0; i < NENTRIES; i = i + 1) begin
                if (valid_r[i]) begin
                    if (cdb0_en && !addr_rdy_r[i] && cdb0_tag == ps_r[i])
                        begin addr_rdy_r[i] <= 1; base_r[i] <= cdb0_val; end
                    if (cdb1_en && !addr_rdy_r[i] && cdb1_tag == ps_r[i])
                        begin addr_rdy_r[i] <= 1; base_r[i] <= cdb1_val; end
                    if (cdb0_en && !data_rdy_r[i] && cdb0_tag == pt_r[i])
                        begin data_rdy_r[i] <= 1; data_r[i] <= cdb0_val; end
                    if (cdb1_en && !data_rdy_r[i] && cdb1_tag == pt_r[i])
                        begin data_rdy_r[i] <= 1; data_r[i] <= cdb1_val; end
                end
            end

            // --- mark committed stores ---
            if (commit_en) begin
                for (i = 0; i < NENTRIES; i = i + 1) begin
                    if (valid_r[i] && is_store_r[i] &&
                        rob_tag_r[i] == commit_rob_tag)
                        committed_r[i] <= 1;
                end
            end

            // --- execute head entry ---
            if (head_valid && head_addr_rdy) begin
                if (head_ld) begin
                    // load: forward from store or go to mem
                    valid_r[head] <= 0;
                    head <= head + 1;
                    cnt  <= cnt - 1 + disp0_en + disp1_en;
                    ld_done    <= 1;
                    ld_pd      <= pd_r[head];
                    ld_rob_tag <= rob_tag_r[head];
                    if (fwd_hit) begin
                        // store-to-load forward
                        ld_val <= fwd_val;
                    end else begin
                        // read from mem (combinational port)
                        ld_val   <= mem_rdata;
                        mem_re   <= 1;
                        mem_addr <= head_addr;
                    end
                end else if (head_st && head_data_rdy && head_committed) begin
                    // store: only write mem after rob commit
                    mem_we    <= 1;
                    mem_addr  <= head_addr;
                    mem_wdata <= data_r[head];
                    valid_r[head] <= 0;
                    head <= head + 1;
                    cnt  <= cnt - 1 + disp0_en + disp1_en;
                end else begin
                    cnt <= cnt + disp0_en + disp1_en;
                end
            end else begin
                cnt <= cnt + disp0_en + disp1_en;
            end

            // --- dispatch new entries ---
            if (disp0_en) begin
                valid_r[tail]     <= 1;
                is_load_r[tail]   <= disp0_is_load;
                is_store_r[tail]  <= disp0_is_store;
                addr_rdy_r[tail]  <= disp0_ps_rdy;
                data_rdy_r[tail]  <= disp0_pt_rdy || disp0_is_load;
                committed_r[tail] <= 0;
                base_r[tail]      <= disp0_vs;
                data_r[tail]      <= disp0_vt;
                imm_r[tail]       <= disp0_imm;
                ps_r[tail]        <= disp0_ps;
                pt_r[tail]        <= disp0_pt;
                pd_r[tail]        <= disp0_pd;
                rob_tag_r[tail]   <= disp0_rob_tag;
                tail <= tail + 1;
            end
            if (disp1_en) begin
                valid_r[tail + disp0_en]     <= 1;
                is_load_r[tail + disp0_en]   <= disp1_is_load;
                is_store_r[tail + disp0_en]  <= disp1_is_store;
                addr_rdy_r[tail + disp0_en]  <= disp1_ps_rdy;
                data_rdy_r[tail + disp0_en]  <= disp1_pt_rdy || disp1_is_load;
                committed_r[tail + disp0_en] <= 0;
                base_r[tail + disp0_en]      <= disp1_vs;
                data_r[tail + disp0_en]      <= disp1_vt;
                imm_r[tail + disp0_en]       <= disp1_imm;
                ps_r[tail + disp0_en]        <= disp1_ps;
                pt_r[tail + disp0_en]        <= disp1_pt;
                pd_r[tail + disp0_en]        <= disp1_pd;
                rob_tag_r[tail + disp0_en]   <= disp1_rob_tag;
                tail <= tail + disp0_en + 1;
            end
        end
    end

    // combinational mem read addr (for load)
    always @(*) begin
        if (!mem_we && head_valid && head_ld && head_addr_rdy && !fwd_hit) begin
            mem_addr = head_addr;
            mem_re   = 1;
        end else if (!mem_we) begin
            mem_addr = 64'd0;
            mem_re   = 0;
        end
    end

endmodule