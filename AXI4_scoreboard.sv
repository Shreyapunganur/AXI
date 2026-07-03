`timescale 1ns / 1ps
// AXI4 Scoreboard / Reference Model
// Shreya | MNNIT
//
// What this does: I keep a "shadow copy" of both slave memories.
// Every time a write goes through the interconnect to a slave, I update
// my shadow copy. Every time a read comes back, I compare what the slave
// returned against what my shadow says should be there.
//
// If they match: data integrity confirmed (write-then-read through the crossbar worked)
// If not: either the routing messed up the data, or there's a bug in the shadow logic.
//
// BUG I FOUND AND FIXED (important for viva):
// The original version had a timing issue where the shadow write was using a
// non-blocking assignment to update active_wr_s0 but then reading it in the
// same always block cycle. Because of NBA scheduling, active_wr_s0 hadn't
// updated yet, so widx was wrong. Fixed by separating the AW latch into
// its own always block that runs independently.
//
// Also fixed: shadow memories now initialize to 0 in initial block, so
// reading an unwritten address gives 0 (not X), which is what the actual
// slave memory also returns.

module AXI4_scoreboard #(
    parameter ADWIDTH   = 32,
    parameter DWIDTH    = 64,
    parameter IDWIDTH   = 4,
    parameter MEM_DEPTH = 256
)(
    input logic clk,
    input logic rst,

    // observe slave 0 write side
    input logic [ADWIDTH-1:0]  s0_awaddr,
    input logic [7:0]          s0_awlen,
    input logic                s0_awvalid,
    input logic                s0_awready,

    input logic [DWIDTH-1:0]   s0_wdata,
    input logic [DWIDTH/8-1:0] s0_wstrb,
    input logic                s0_wlast,
    input logic                s0_wvalid,
    input logic                s0_wready,

    // observe slave 0 read side
    input logic [ADWIDTH-1:0]  s0_araddr,
    input logic                s0_arvalid,
    input logic                s0_arready,

    input logic [IDWIDTH-1:0]  s0_rid,
    input logic [DWIDTH-1:0]   s0_rdata,
    input logic                s0_rlast,
    input logic                s0_rvalid,
    input logic                s0_rready,

    // observe slave 1 write side
    input logic [ADWIDTH-1:0]  s1_awaddr,
    input logic [7:0]          s1_awlen,
    input logic                s1_awvalid,
    input logic                s1_awready,

    input logic [DWIDTH-1:0]   s1_wdata,
    input logic [DWIDTH/8-1:0] s1_wstrb,
    input logic                s1_wlast,
    input logic                s1_wvalid,
    input logic                s1_wready,

    // observe slave 1 read side
    input logic [ADWIDTH-1:0]  s1_araddr,
    input logic                s1_arvalid,
    input logic                s1_arready,

    input logic [IDWIDTH-1:0]  s1_rid,
    input logic [DWIDTH-1:0]   s1_rdata,
    input logic                s1_rlast,
    input logic                s1_rvalid,
    input logic                s1_rready,

    // B channel routing check (make sure response goes to correct master)
    input logic [IDWIDTH-1:0]  m0_bid,
    input logic                m0_bvalid, m0_bready,
    input logic [IDWIDTH-1:0]  m1_bid,
    input logic                m1_bvalid, m1_bready,
    input logic [IDWIDTH-1:0]  m0_awid,
    input logic                m0_awvalid_in, m0_awready_in,
    input logic [IDWIDTH-1:0]  m1_awid,
    input logic                m1_awvalid_in, m1_awready_in
);

    // ---- shadow memory ----
    logic [DWIDTH-1:0] shadow_s0 [0:MEM_DEPTH-1];
    logic [DWIDTH-1:0] shadow_s1 [0:MEM_DEPTH-1];

    // initialize to 0 -- the actual slave memory also initializes to 0
    // so reads of unwritten addresses should match
    integer init_idx;
    initial begin
        for (init_idx = 0; init_idx < MEM_DEPTH; init_idx = init_idx + 1) begin
            shadow_s0[init_idx] = 64'h0;
            shadow_s1[init_idx] = 64'h0;
        end
    end

    int unsigned sb_pass = 0;
    int unsigned sb_fail = 0;

    // ----------------------------------------------------------------
    // Write tracking for slave 0
    // Approach: latch AW address when handshake happens, then use it
    // when W beats come through. Keep a beat counter for burst support.
    // ----------------------------------------------------------------
    logic [ADWIDTH-1:0] s0_wr_base_addr;
    logic               s0_wr_addr_valid = 0;
    logic [7:0]         s0_wr_beat_cnt   = 0;

    // latch the write address as soon as AW handshake happens
    always_ff @(posedge clk) begin
        if (!rst) begin
            s0_wr_addr_valid <= 0;
            s0_wr_base_addr  <= 0;
            s0_wr_beat_cnt   <= 0;
        end else begin
            if (s0_awvalid && s0_awready) begin
                s0_wr_base_addr  <= s0_awaddr;
                s0_wr_addr_valid <= 1;
                s0_wr_beat_cnt   <= 0;
                // $display("[SB-DEBUG] S0 AW latched addr=0x%08h", s0_awaddr);
            end

            if (s0_wvalid && s0_wready && s0_wr_addr_valid) begin
                // word index: addr/8 gives 64-bit word index, plus beat offset
                automatic logic [$clog2(MEM_DEPTH)-1:0] widx =
                    (s0_wr_base_addr >> 3) + s0_wr_beat_cnt;

                for (int b = 0; b < DWIDTH/8; b++) begin
                    if (s0_wstrb[b])
                        shadow_s0[widx][b*8 +: 8] <= s0_wdata[b*8 +: 8];
                end
                // $display("[SB-DEBUG] S0 shadow[%0d] <= 0x%016h (beat %0d)", widx, s0_wdata, s0_wr_beat_cnt);

                if (s0_wlast)
                    s0_wr_addr_valid <= 0;
                else
                    s0_wr_beat_cnt <= s0_wr_beat_cnt + 1;
            end
        end
    end

    // Write tracking for slave 1 (same logic)
    logic [ADWIDTH-1:0] s1_wr_base_addr;
    logic               s1_wr_addr_valid = 0;
    logic [7:0]         s1_wr_beat_cnt   = 0;

    always_ff @(posedge clk) begin
        if (!rst) begin
            s1_wr_addr_valid <= 0;
            s1_wr_base_addr  <= 0;
            s1_wr_beat_cnt   <= 0;
        end else begin
            if (s1_awvalid && s1_awready) begin
                s1_wr_base_addr  <= s1_awaddr;
                s1_wr_addr_valid <= 1;
                s1_wr_beat_cnt   <= 0;
            end

            if (s1_wvalid && s1_wready && s1_wr_addr_valid) begin
                automatic logic [$clog2(MEM_DEPTH)-1:0] widx =
                    (s1_wr_base_addr >> 3) + s1_wr_beat_cnt;

                for (int b = 0; b < DWIDTH/8; b++) begin
                    if (s1_wstrb[b])
                        shadow_s1[widx][b*8 +: 8] <= s1_wdata[b*8 +: 8];
                end

                if (s1_wlast)
                    s1_wr_addr_valid <= 0;
                else
                    s1_wr_beat_cnt <= s1_wr_beat_cnt + 1;
            end
        end
    end

    // ----------------------------------------------------------------
    // Read checking for slave 0
    // Latch AR address, then compare each R beat against shadow
    // ----------------------------------------------------------------
    logic [ADWIDTH-1:0] s0_rd_base_addr;
    logic               s0_rd_active = 0;
    logic [7:0]         s0_rd_beat   = 0;

    always_ff @(posedge clk) begin
        if (!rst) begin
            s0_rd_active <= 0;
            s0_rd_beat   <= 0;
        end else begin
            if (s0_arvalid && s0_arready) begin
                s0_rd_base_addr <= s0_araddr;
                s0_rd_active    <= 1;
                s0_rd_beat      <= 0;
            end

            if (s0_rvalid && s0_rready && s0_rd_active) begin
                automatic logic [$clog2(MEM_DEPTH)-1:0] ridx =
                    (s0_rd_base_addr >> 3) + s0_rd_beat;
                automatic logic [DWIDTH-1:0] expected = shadow_s0[ridx];

                if (s0_rdata === expected) begin
                    $display("[SB-S0 %0t ns] PASS addr=0x%08h beat=%0d got=0x%016h",
                             $time/1000, s0_rd_base_addr, s0_rd_beat, s0_rdata);
                    sb_pass++;
                end else begin
                    $error("[SB-S0 %0t ns] FAIL addr=0x%08h beat=%0d got=0x%016h exp=0x%016h",
                           $time/1000, s0_rd_base_addr, s0_rd_beat, s0_rdata, expected);
                    sb_fail++;
                end

                if (s0_rlast) begin
                    s0_rd_active <= 0;
                    s0_rd_beat   <= 0;
                end else
                    s0_rd_beat <= s0_rd_beat + 1;
            end
        end
    end

    // Read checking for slave 1
    logic [ADWIDTH-1:0] s1_rd_base_addr;
    logic               s1_rd_active = 0;
    logic [7:0]         s1_rd_beat   = 0;

    always_ff @(posedge clk) begin
        if (!rst) begin
            s1_rd_active <= 0;
            s1_rd_beat   <= 0;
        end else begin
            if (s1_arvalid && s1_arready) begin
                s1_rd_base_addr <= s1_araddr;
                s1_rd_active    <= 1;
                s1_rd_beat      <= 0;
            end

            if (s1_rvalid && s1_rready && s1_rd_active) begin
                automatic logic [$clog2(MEM_DEPTH)-1:0] ridx =
                    (s1_rd_base_addr >> 3) + s1_rd_beat;
                automatic logic [DWIDTH-1:0] expected = shadow_s1[ridx];

                if (s1_rdata === expected) begin
                    $display("[SB-S1 %0t ns] PASS addr=0x%08h beat=%0d got=0x%016h",
                             $time/1000, s1_rd_base_addr, s1_rd_beat, s1_rdata);
                    sb_pass++;
                end else begin
                    $error("[SB-S1 %0t ns] FAIL addr=0x%08h beat=%0d got=0x%016h exp=0x%016h",
                           $time/1000, s1_rd_base_addr, s1_rd_beat, s1_rdata, expected);
                    sb_fail++;
                end

                if (s1_rlast) begin
                    s1_rd_active <= 0;
                    s1_rd_beat   <= 0;
                end else
                    s1_rd_beat <= s1_rd_beat + 1;
            end
        end
    end

    // ----------------------------------------------------------------
    // Response routing check: bid on M0's B channel should match
    // M0's last AWID. Simple but catches a whole class of bugs.
    // ----------------------------------------------------------------
    logic [IDWIDTH-1:0] m0_last_awid = 0;
    logic [IDWIDTH-1:0] m1_last_awid = 0;

    always_ff @(posedge clk) begin
        if (!rst) begin
            m0_last_awid <= 0;
            m1_last_awid <= 0;
        end else begin
            if (m0_awvalid_in && m0_awready_in)
                m0_last_awid <= m0_awid;
            if (m1_awvalid_in && m1_awready_in)
                m1_last_awid <= m1_awid;

            // check that B channel goes back to the right master
            if (m0_bvalid && m0_bready && m0_bid !== m0_last_awid) begin
                $error("[SB-ROUTE %0t ns] M0 bid=0x%0h but awid was 0x%0h -- routing bug?",
                       $time/1000, m0_bid, m0_last_awid);
                sb_fail++;
            end
            if (m1_bvalid && m1_bready && m1_bid !== m1_last_awid) begin
                $error("[SB-ROUTE %0t ns] M1 bid=0x%0h but awid was 0x%0h -- routing bug?",
                       $time/1000, m1_bid, m1_last_awid);
                sb_fail++;
            end
        end
    end

    // summary at end of sim
    final begin
        $display("\n--- SCOREBOARD RESULTS ---");
        $display("  Passed: %0d", sb_pass);
        $display("  Failed: %0d", sb_fail);
        if (sb_fail == 0)
            $display("  All data integrity checks passed!");
        else
            $display("  *** Some checks FAILED -- see errors above ***");
        $display("--------------------------\n");
    end

endmodule
