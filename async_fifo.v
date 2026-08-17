`timescale 1ns/1ps
// =====================================================================
// async_fifo.v
// -----------------------------------------------------------------------
// A standard dual-clock FIFO. Write side and read side run on two
// completely independent clocks. This is the one building block that
// makes the CDC bridge (axi4_cdc_bridge.v) safe to use between two
// clock domains.
//
// HOW IT STAYS SAFE ACROSS CLOCKS (the only tricky idea in this file):
//   The write pointer and read pointer each live in their OWN clock
//   domain. To compare them safely (for full/empty), each pointer is
//   converted to Gray code before crossing domains, then passed through
//   a 2-flop synchronizer. Gray code guarantees that only ONE bit
//   changes per pointer increment, so a synchronizer sampling it
//   mid-transition can only ever read one bit "late" - never a
//   completely wrong value. That's the whole trick.
//
// One extra pointer bit (ADDR_W+1 bits total, not ADDR_W) is what lets
// us tell "full" and "empty" apart even though both look like
// "write pointer == read pointer" in plain binary.
//
// Read data is "first word fall through": rd_data always shows the
// current front of the queue combinationally whenever !empty, so a
// caller just does  valid = !empty; data = rd_data; pop = valid&&ready.
// =====================================================================
module async_fifo #(
    parameter WIDTH  = 32,
    parameter DEPTH  = 8,                 // must be a power of 2
    parameter ADDR_W = $clog2(DEPTH)
)(
    // ---------------- write side ----------------
    input  wire             wr_clk,
    input  wire             wr_rst_n,
    input  wire             wr_en,
    input  wire [WIDTH-1:0] wr_data,
    output wire             full,

    // ---------------- read side ----------------
    input  wire             rd_clk,
    input  wire             rd_rst_n,
    input  wire             rd_en,
    output wire [WIDTH-1:0] rd_data,
    output wire             empty
);

    reg [WIDTH-1:0] mem [0:DEPTH-1];

    // binary pointers (used to address "mem") and their Gray-coded twins
    // (used whenever a pointer needs to cross into the other clock domain)
    reg [ADDR_W:0] wr_bin, wr_gray;
    reg [ADDR_W:0] rd_bin, rd_gray;

    wire [ADDR_W:0] wr_bin_next  = wr_bin + (wr_en && !full);
    wire [ADDR_W:0] wr_gray_next = (wr_bin_next >> 1) ^ wr_bin_next;

    wire [ADDR_W:0] rd_bin_next  = rd_bin + (rd_en && !empty);
    wire [ADDR_W:0] rd_gray_next = (rd_bin_next >> 1) ^ rd_bin_next;

    // ---- 2-flop synchronizers: this is the actual CDC crossing ----
    reg [ADDR_W:0] wr_gray_sync1, wr_gray_sync2;   // write ptr, synced into rd_clk
    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            wr_gray_sync1 <= 0;
            wr_gray_sync2 <= 0;
        end else begin
            wr_gray_sync1 <= wr_gray;
            wr_gray_sync2 <= wr_gray_sync1;
        end
    end

    reg [ADDR_W:0] rd_gray_sync1, rd_gray_sync2;   // read ptr, synced into wr_clk
    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            rd_gray_sync1 <= 0;
            rd_gray_sync2 <= 0;
        end else begin
            rd_gray_sync1 <= rd_gray;
            rd_gray_sync2 <= rd_gray_sync1;
        end
    end

    // ---- write-side pointer + memory write ----
    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wr_bin  <= 0;
            wr_gray <= 0;
        end else begin
            if (wr_en && !full)
                mem[wr_bin[ADDR_W-1:0]] <= wr_data;
            wr_bin  <= wr_bin_next;
            wr_gray <= wr_gray_next;
        end
    end

    // ---- read-side pointer ----
    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            rd_bin  <= 0;
            rd_gray <= 0;
        end else begin
            rd_bin  <= rd_bin_next;
            rd_gray <= rd_gray_next;
        end
    end

    assign rd_data = mem[rd_bin[ADDR_W-1:0]];

    // empty: read pointer has caught up to the (synced) write pointer
    assign empty = (rd_gray == wr_gray_sync2);

    // full: write pointer has lapped the (synced) read pointer.
    // In Gray code, "lapped by exactly DEPTH" shows up as matching the
    // read pointer with its top two bits flipped - a well-known result
    // of using one extra pointer bit. Comments in the README walk
    // through a small numeric example if you want to verify it by hand.
    assign full = (wr_gray == {~rd_gray_sync2[ADDR_W:ADDR_W-1], rd_gray_sync2[ADDR_W-2:0]});

endmodule
