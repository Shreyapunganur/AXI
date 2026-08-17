`timescale 1ns/1ps
// =====================================================================
// axi4_slave.v
// -----------------------------------------------------------------------
// A memory-backed AXI4 slave. Every access succeeds (OKAY) - this file
// intentionally does not model exclusive access or error responses, to
// keep it focused on the two things that matter most for a portfolio
// piece: correct WSTRB byte-masking, and the out-of-order mechanism.
//
// THE "SLOTS" IDEA (same one used in axi4_master.v):
//   - Every accepted read gets a slot in the READ TABLE, holding its
//     address/len/burst/id plus a small RANDOM delay (1-8 cycles).
//   - Every completed write (all W beats received) gets a slot in the
//     B TABLE, holding its id plus the same kind of random delay.
//
// THIS RANDOM DELAY IS WHERE OUT-OF-ORDER COMPLETION COMES FROM: every
// cycle we scan the table low-to-high and let whichever slot's delay
// has reached zero go first. If transaction A (issued first) draws a
// delay of 8 and transaction B (issued right after, different ID)
// draws a delay of 2, B's response goes out first. Neither table cares
// what order things were ADDED in - only whether a slot is "ready".
//
// One AXI4 rule this file is careful about: once we start streaming a
// read slot's beats on RVALID, we finish that whole burst (through
// RLAST) before picking a different slot - a single burst's data is
// never split up by another transaction.
// =====================================================================
module axi4_slave #(
    parameter ADDR_WIDTH  = 32,
    parameter DATA_WIDTH  = 32,
    parameter ID_WIDTH    = 3,
    parameter STRB_WIDTH  = DATA_WIDTH/8,
    parameter MEM_WORDS   = 1024,
    parameter TAB_DEPTH   = 8            // read table / B table size
)(
    input  wire                   clk,
    input  wire                   rst_n,

    // write address channel
    input  wire [ID_WIDTH-1:0]    s_awid,
    input  wire [ADDR_WIDTH-1:0]  s_awaddr,
    input  wire [7:0]             s_awlen,
    input  wire [2:0]             s_awsize,
    input  wire [1:0]             s_awburst,
    input  wire                   s_awvalid,
    output wire                   s_awready,

    // write data channel
    input  wire [DATA_WIDTH-1:0]  s_wdata,
    input  wire [STRB_WIDTH-1:0]  s_wstrb,
    input  wire                   s_wlast,
    input  wire                   s_wvalid,
    output wire                   s_wready,

    // write response channel
    output wire [ID_WIDTH-1:0]    s_bid,
    output wire [1:0]             s_bresp,
    output wire                   s_bvalid,
    input  wire                   s_bready,

    // read address channel
    input  wire [ID_WIDTH-1:0]    s_arid,
    input  wire [ADDR_WIDTH-1:0]  s_araddr,
    input  wire [7:0]             s_arlen,
    input  wire [2:0]             s_arsize,
    input  wire [1:0]             s_arburst,
    input  wire                   s_arvalid,
    output wire                   s_arready,

    // read data channel
    output wire [ID_WIDTH-1:0]    s_rid,
    output wire [DATA_WIDTH-1:0]  s_rdata,
    output wire [1:0]             s_rresp,
    output wire                   s_rlast,
    output wire                   s_rvalid,
    input  wire                   s_rready
);

    localparam [1:0] RESP_OKAY = 2'b00;
    localparam [1:0] BURST_FIXED = 2'b00;
    localparam [1:0] BURST_WRAP  = 2'b10;

    localparam TAB_IDX_W = $clog2(TAB_DEPTH);  // 3 bits, indexes an 8-entry table

    reg [DATA_WIDTH-1:0] mem [0:MEM_WORDS-1];

    // -----------------------------------------------------------------
    // shared address-stepping function, used by both the write and the
    // read side to walk a burst beat by beat
    // -----------------------------------------------------------------
    function [ADDR_WIDTH-1:0] next_addr;
        input [ADDR_WIDTH-1:0] addr;
        input [7:0]            len;     // AxLEN (beats - 1)
        input [1:0]            burst;
        reg   [ADDR_WIDTH-1:0] burst_bytes, wrap_base;
        begin
            if (burst == BURST_FIXED) begin
                next_addr = addr;
            end else if (burst == BURST_WRAP) begin
                burst_bytes = 32'd4 * (len + 8'd1);
                wrap_base   = (addr / burst_bytes) * burst_bytes;
                if ((addr + 32'd4) >= (wrap_base + burst_bytes))
                    next_addr = wrap_base;
                else
                    next_addr = addr + 32'd4;
            end else begin // INCR
                next_addr = addr + 32'd4;
            end
        end
    endfunction

    // a cheap 1-8 pseudo-random delay. $random is simulation-only,
    // which is fine here - this slave is a verification model standing
    // in for "a real memory that takes a variable number of cycles",
    // not synthesizable hardware.
    function [3:0] rand_delay;
        input dummy;
        reg [31:0] r;
        begin
            r = $random;
            rand_delay = {1'b0, r[2:0]} + 4'd1;   // 1..8
        end
    endfunction

    integer i, k;

    // ===================================================================
    // WRITE SIDE: one write is received at a time (the crossbar makes
    // sure of that), address stepped beat by beat, WSTRB applied per
    // byte lane. On the final beat, its ID is dropped into the B TABLE
    // below to wait its random delay.
    // ===================================================================
    localparam WR_IDLE = 1'b0, WR_BEATS = 1'b1;
    reg                   wr_state;
    reg [ADDR_WIDTH-1:0]  wr_addr;
    reg [7:0]             wr_len;
    reg [1:0]             wr_burst;
    reg [ID_WIDTH-1:0]    wr_id;

    reg btab_has_space;   // does the B table have a free slot? (gates s_awready)

    assign s_awready = (wr_state == WR_IDLE) && btab_has_space;
    assign s_wready   = (wr_state == WR_BEATS);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_state <= WR_IDLE;
        end else begin
            case (wr_state)
                WR_IDLE: begin
                    if (s_awvalid && s_awready) begin
                        wr_addr  <= s_awaddr;
                        wr_len   <= s_awlen;
                        wr_burst <= s_awburst;
                        wr_id    <= s_awid;
                        wr_state <= WR_BEATS;
                    end
                end

                WR_BEATS: begin
                    if (s_wvalid && s_wready) begin
                        for (k = 0; k < STRB_WIDTH; k = k + 1)
                            if (s_wstrb[k])
                                mem[wr_addr[11:2]][k*8 +: 8] <= s_wdata[k*8 +: 8];

                        if (s_wlast)
                            wr_state <= WR_IDLE;
                        else
                            wr_addr <= next_addr(wr_addr, wr_len, wr_burst);
                    end
                end
            endcase
        end
    end

    // ===================================================================
    // B TABLE - completed writes waiting out their random delay before
    // their response goes out. Picking is a plain low-to-high scan: the
    // first slot whose delay has hit zero is the one that goes next.
    // ===================================================================
    reg                busy_b [0:TAB_DEPTH-1];
    reg [ID_WIDTH-1:0] id_b   [0:TAB_DEPTH-1];
    reg [3:0]          delay_b[0:TAB_DEPTH-1];

    always @(*) begin
        btab_has_space = 1'b0;
        for (i = 0; i < TAB_DEPTH; i = i + 1)
            if (!busy_b[i]) btab_has_space = 1'b1;
    end

    // slot ready to send its B response next (lowest index, delay==0)
    reg                  b_out_valid;
    reg [TAB_IDX_W-1:0]  b_pick_index;
    integer bp;
    always @(*) begin
        b_out_valid  = 1'b0;
        b_pick_index = 0;
        for (bp = 0; bp < TAB_DEPTH; bp = bp + 1) begin
            if (!b_out_valid && busy_b[bp] && (delay_b[bp] == 4'd0)) begin
                b_out_valid  = 1'b1;
                b_pick_index = bp[TAB_IDX_W-1:0];
            end
        end
    end

    reg found_free_b;   // scratch "already claimed a slot this pass?" flag

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < TAB_DEPTH; i = i + 1)
                busy_b[i] <= 1'b0;
        end else begin
            // new write outcome lands in the first free slot found by
            // scanning low to high (found_free_b just stops us from
            // claiming a second slot once the first match is made - it's
            // a loop-control scratch variable, not a stored signal)
            if (s_wvalid && s_wready && s_wlast) begin
                found_free_b = 1'b0;
                for (i = 0; i < TAB_DEPTH; i = i + 1) begin
                    if (!found_free_b && !busy_b[i]) begin
                        busy_b[i]  <= 1'b1;
                        id_b[i]    <= wr_id;
                        delay_b[i] <= rand_delay(1'b0);
                        found_free_b = 1'b1;
                    end
                end
            end

            // decrement all pending delays
            for (i = 0; i < TAB_DEPTH; i = i + 1)
                if (busy_b[i] && delay_b[i] != 4'd0)
                    delay_b[i] <= delay_b[i] - 4'd1;

            // free the slot that is actually being presented right now,
            // the moment it gets accepted (b_pick_index and s_bvalid
            // below are both combinational off the SAME current state,
            // so there's no risk of freeing the wrong slot)
            if (s_bvalid && s_bready)
                busy_b[b_pick_index] <= 1'b0;
        end
    end

    // the B response bus always just shows whatever the current pick is -
    // combinational, so it can never drift out of sync with b_pick_index
    assign s_bvalid = b_out_valid;
    assign s_bid    = id_b[b_pick_index];
    assign s_bresp  = RESP_OKAY;

    // ===================================================================
    // READ TABLE - same idea as the B table: each accepted read waits
    // out its own random delay, and whichever one hits zero first goes
    // first. The one extra piece here is that once a slot STARTS
    // streaming out its beats, it holds the R channel until its own
    // RLAST, so two bursts are never interleaved on the wire.
    // ===================================================================
    reg                   busy_r [0:TAB_DEPTH-1];
    reg [ID_WIDTH-1:0]    id_r   [0:TAB_DEPTH-1];
    reg [ADDR_WIDTH-1:0]  addr_r [0:TAB_DEPTH-1];   // mutable: steps forward each beat
    reg [7:0]             len_r  [0:TAB_DEPTH-1];
    reg [1:0]             burst_r[0:TAB_DEPTH-1];

    reg rdtab_has_space;
    always @(*) begin
        rdtab_has_space = 1'b0;
        for (i = 0; i < TAB_DEPTH; i = i + 1)
            if (!busy_r[i]) rdtab_has_space = 1'b1;
    end

    assign s_arready = rdtab_has_space;

    reg [3:0] delay_r[0:TAB_DEPTH-1];
    reg found_free_r;

    // true on the exact cycle the currently-streaming slot's last beat
    // is accepted - a plain wire, safe to read from more than one
    // always block (only WRITES to the same reg from two blocks are a
    // problem, and busy_r's only writer is the block right below)
    wire r_release = r_active && s_rready && r_this_is_last;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < TAB_DEPTH; i = i + 1)
                busy_r[i] <= 1'b0;
        end else begin
            // one pass, low to high: claim the first free slot and fill
            // in every one of its fields (found_free_r is just loop
            // control, same idea as found_free_b above)
            if (s_arvalid && s_arready) begin
                found_free_r = 1'b0;
                for (i = 0; i < TAB_DEPTH; i = i + 1) begin
                    if (!found_free_r && !busy_r[i]) begin
                        busy_r[i]   <= 1'b1;
                        id_r[i]     <= s_arid;
                        addr_r[i]   <= s_araddr;
                        len_r[i]    <= s_arlen;
                        burst_r[i]  <= s_arburst;
                        delay_r[i]  <= rand_delay(1'b0);
                        found_free_r = 1'b1;
                    end
                end
            end

            // release the slot that just finished streaming (single
            // place that ever clears busy_r, so there's no ambiguity
            // about which always block "wins")
            if (r_release)
                busy_r[r_slot] <= 1'b0;

            for (i = 0; i < TAB_DEPTH; i = i + 1)
                if (busy_r[i] && delay_r[i] != 4'd0)
                    delay_r[i] <= delay_r[i] - 4'd1;
        end
    end

    // pick which ready slot starts streaming next (low-to-high, delay==0)
    reg                  r_cand_valid;
    reg [TAB_IDX_W-1:0]  r_cand_index;
    integer rp;
    always @(*) begin
        r_cand_valid = 1'b0;
        r_cand_index = 0;
        for (rp = 0; rp < TAB_DEPTH; rp = rp + 1) begin
            if (!r_cand_valid && busy_r[rp] && (delay_r[rp] == 4'd0)) begin
                r_cand_valid = 1'b1;
                r_cand_index = rp[TAB_IDX_W-1:0];
            end
        end
    end

    reg                  r_active;       // currently streaming a burst?
    reg [TAB_IDX_W-1:0]  r_slot;         // which table entry
    reg [7:0]            r_beat;         // beats sent so far in this burst

    wire r_this_is_last = (r_beat == len_r[r_slot]);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_active <= 1'b0;
        end else begin
            if (!r_active) begin
                if (r_cand_valid) begin
                    r_active <= 1'b1;
                    r_slot   <= r_cand_index;
                    r_beat   <= 8'd0;
                end
            end else begin
                if (s_rready) begin           // s_rvalid is always 1 while r_active
                    if (r_this_is_last) begin
                        r_active <= 1'b0;
                    end else begin
                        addr_r[r_slot] <= next_addr(addr_r[r_slot], len_r[r_slot], burst_r[r_slot]);
                        r_beat         <= r_beat + 8'd1;
                    end
                end
            end
        end
    end

    assign s_rvalid = r_active;
    assign s_rid    = id_r[r_slot];
    assign s_rdata  = mem[addr_r[r_slot][11:2]];
    assign s_rresp  = RESP_OKAY;
    assign s_rlast  = r_this_is_last;

endmodule
