
// mem_module.sv — dual-port memory
// port 0: 2x instr fetch (32-bit each)
// port 1: data load/store (64-bit)
// store writes committed — gated by we

`ifndef MEM_SIZE
  `define MEM_SIZE (512 * 1024)
`endif

module memory #(
    parameter MEM_SIZE = `MEM_SIZE
) (
    input         clk,

    // instr fetch ports (4 for quad-issue / 64-byte fetch)
    input  [63:0] fetch_addr0,
    input  [63:0] fetch_addr1,
    input  [63:0] fetch_addr2,
    input  [63:0] fetch_addr3,
    output [31:0] instr_out0,
    output [31:0] instr_out1,
    output [31:0] instr_out2,
    output [31:0] instr_out3,

    // data port 0 (load/store)
    input  [63:0] data_addr,
    input  [63:0] write_data,
    input         we,
    output [63:0] read_data,

    // data port 1 (second load)
    input  [63:0] data_addr2,
    output [63:0] read_data2
);

    reg [7:0] bytes [0:MEM_SIZE-1];

    // instr fetch — little-endian word
    assign instr_out0 = {bytes[fetch_addr0+3], bytes[fetch_addr0+2],
                         bytes[fetch_addr0+1], bytes[fetch_addr0]};
    assign instr_out1 = {bytes[fetch_addr1+3], bytes[fetch_addr1+2],
                         bytes[fetch_addr1+1], bytes[fetch_addr1]};
    assign instr_out2 = {bytes[fetch_addr2+3], bytes[fetch_addr2+2],
                         bytes[fetch_addr2+1], bytes[fetch_addr2]};
    assign instr_out3 = {bytes[fetch_addr3+3], bytes[fetch_addr3+2],
                         bytes[fetch_addr3+1], bytes[fetch_addr3]};

    // data load port 0 — little-endian 64-bit
    assign read_data = {bytes[data_addr+7], bytes[data_addr+6],
                        bytes[data_addr+5], bytes[data_addr+4],
                        bytes[data_addr+3], bytes[data_addr+2],
                        bytes[data_addr+1], bytes[data_addr]};

    // data load port 1 — little-endian 64-bit
    assign read_data2 = {bytes[data_addr2+7], bytes[data_addr2+6],
                         bytes[data_addr2+5], bytes[data_addr2+4],
                         bytes[data_addr2+3], bytes[data_addr2+2],
                         bytes[data_addr2+1], bytes[data_addr2]};

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

    // dispatch
    input         disp0_en,
    input         disp0_is_load,
    input         disp0_is_store,
    input  [PHYS_W-1:0] disp0_ps,
    input  [PHYS_W-1:0] disp0_pt,
    input         disp0_ps_rdy,
    input         disp0_pt_rdy,
    input  [63:0] disp0_vs,
    input  [63:0] disp0_vt,
    input  [63:0] disp0_imm,
    input  [PHYS_W-1:0] disp0_pd,
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

    // CDB
    input         cdb0_en,
    input  [PHYS_W-1:0] cdb0_tag,
    input  [63:0] cdb0_val,
    input         cdb1_en,
    input  [PHYS_W-1:0] cdb1_tag,
    input  [63:0] cdb1_val,

    // commit
    input         commit_en,
    input  [5:0]  commit_rob_tag,

    // memory
    output reg [63:0] mem_addr,
    output reg [63:0] mem_wdata,
    output reg        mem_we,
    output reg        mem_re,
    input  [63:0]     mem_rdata,

    // load result
    output reg        ld_done,
    output reg [PHYS_W-1:0] ld_pd,
    output reg [63:0] ld_val,
    output reg [5:0]  ld_rob_tag,

    output wire full
);

    // state
    reg        valid_r    [0:NENTRIES-1];
    reg        is_load_r  [0:NENTRIES-1];
    reg        is_store_r [0:NENTRIES-1];
    reg        addr_rdy_r [0:NENTRIES-1];
    reg        data_rdy_r [0:NENTRIES-1];
    reg        committed_r[0:NENTRIES-1];
    reg [63:0] base_r     [0:NENTRIES-1];
    reg [63:0] data_r     [0:NENTRIES-1];
    reg [63:0] imm_r      [0:NENTRIES-1];
    reg [PHYS_W-1:0] ps_r [0:NENTRIES-1];
    reg [PHYS_W-1:0] pt_r [0:NENTRIES-1];
    reg [PHYS_W-1:0] pd_r [0:NENTRIES-1];
    reg [5:0]  rob_tag_r  [0:NENTRIES-1];

    reg [3:0] head, tail;
    reg [4:0] cnt;

    assign full = (cnt >= NENTRIES - 1);

    integer i, f;

    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < NENTRIES; i = i + 1)
                valid_r[i] <= 0;

            head <= 0;
            tail <= 0;
            cnt  <= 0;

            mem_we <= 0;
            mem_re <= 0;
            ld_done <= 0;
        end else begin
            // defaults
            mem_we  <= 0;
            mem_re  <= 0;
            ld_done <= 0;

            // ---------------- CDB SNOOP ----------------
            for (i = 0; i < NENTRIES; i = i + 1) begin
                if (valid_r[i]) begin
                    if (cdb0_en && !addr_rdy_r[i] && cdb0_tag == ps_r[i]) begin
                        addr_rdy_r[i] <= 1;
                        base_r[i]     <= cdb0_val;
                    end
                    if (cdb1_en && !addr_rdy_r[i] && cdb1_tag == ps_r[i]) begin
                        addr_rdy_r[i] <= 1;
                        base_r[i]     <= cdb1_val;
                    end

                    if (cdb0_en && !data_rdy_r[i] && cdb0_tag == pt_r[i]) begin
                        data_rdy_r[i] <= 1;
                        data_r[i]     <= cdb0_val;
                    end
                    if (cdb1_en && !data_rdy_r[i] && cdb1_tag == pt_r[i]) begin
                        data_rdy_r[i] <= 1;
                        data_r[i]     <= cdb1_val;
                    end
                end
            end

            // ---------------- COMMIT ----------------
            if (commit_en) begin
                for (i = 0; i < NENTRIES; i = i + 1) begin
                    if (valid_r[i] && is_store_r[i] &&
                        rob_tag_r[i] == commit_rob_tag)
                        committed_r[i] <= 1;
                end
            end

            // ---------------- EXECUTE HEAD ----------------
            if (valid_r[head] && addr_rdy_r[head]) begin

                // compute head addr locally
                reg [63:0] head_addr;
                head_addr = base_r[head] + imm_r[head];

                if (is_load_r[head]) begin
                    // -------- FORWARDING (SEQUENTIAL NOW) --------
                    reg fwd_hit;
                    reg [63:0] fwd_val;

                    fwd_hit = 0;
                    fwd_val = 64'd0;

                    for (f = 0; f < NENTRIES; f = f + 1) begin
                        if (valid_r[f] && is_store_r[f] &&
                            addr_rdy_r[f] && data_rdy_r[f] &&
                            (base_r[f] + imm_r[f]) == head_addr) begin
                            fwd_hit = 1;
                            fwd_val = data_r[f];
                        end
                    end

                    // retire load
                    valid_r[head] <= 0;
                    head <= head + 1;
                    cnt  <= cnt - 1 + disp0_en + disp1_en;

                    ld_done    <= 1;
                    ld_pd      <= pd_r[head];
                    ld_rob_tag <= rob_tag_r[head];

                    if (fwd_hit) begin
                        ld_val <= fwd_val;
                    end else begin
                        mem_addr <= head_addr;
                        mem_re   <= 1;
                        ld_val   <= mem_rdata;
                    end

                end else if (is_store_r[head] &&
                             data_rdy_r[head] &&
                             committed_r[head]) begin

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

            // ---------------- DISPATCH ----------------
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

endmodule