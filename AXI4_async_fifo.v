`timescale 1ns / 1ps

// Asynchronous FIFO using Gray-code pointers
// Reference: Clifford Cummings SNUG 2002 paper "Simulation and Synthesis Techniques
// for Asynchronous FIFO Design" -- honestly one of the best papers I've read
//
// Why gray code? Because when you cross binary pointers across clock domains,
// multiple bits can change at once (e.g. 0111->1000 flips 4 bits). If the
// synchronizer samples during that transition you get garbage. Gray code changes
// only ONE bit per increment, so worst case you're off by 1 count.
//
// DEPTH must be a power of 2. I learned this the hard way -- gray code wrapping
// only works correctly with powers of 2. Got weird full/empty glitches with DEPTH=6.
//
// FWFT (First Word Fall Through): rdata is valid as soon as data enters, no need
// to assert pop first. Makes it easier to connect to AXI ready/valid handshake.

module AXI4_async_fifo #(
    parameter WIDTH = 64,
    parameter DEPTH = 8    // MUST be power of 2
)(
    // write side
    input  wire             wclk,
    input  wire             wrst_n,
    input  wire             push,
    input  wire [WIDTH-1:0] wdata,
    output wire             full,
    output wire             almost_full,    // useful for backpressure, 1 slot left

    // read side
    input  wire             rclk,
    input  wire             rrst_n,
    input  wire             pop,
    output wire [WIDTH-1:0] rdata,
    output wire             empty,
    output wire             almost_empty
);

    localparam AWIDTH = $clog2(DEPTH);  // address bits, e.g. 3 for DEPTH=8

    // actual memory -- simple 2D array
    reg [WIDTH-1:0] fifo_mem [0:DEPTH-1];

    // write pointer (extra MSB is the wrap-around flag for full detection)
    reg [AWIDTH:0] wptr_bin;
    reg [AWIDTH:0] wptr_gray;

    // read pointer
    reg [AWIDTH:0] rptr_bin;
    reg [AWIDTH:0] rptr_gray;

    // 2-FF synchronizers -- the whole point is to reduce metastability probability
    // to negligible levels. Two FFs is standard. Three is overkill for most designs
    // but you'd use it for very high-speed or safety-critical stuff.
    reg [AWIDTH:0] rptr_gray_sync1, rptr_gray_sync2;  // rptr synced into wclk domain
    reg [AWIDTH:0] wptr_gray_sync1, wptr_gray_sync2;  // wptr synced into rclk domain

    // binary to gray -- just XOR with right-shifted version
    function [AWIDTH:0] bin2gray;
        input [AWIDTH:0] b;
        bin2gray = b ^ (b >> 1);
    endfunction

    // ----------------------------------------------------------------
    // Write logic (wclk domain)
    // ----------------------------------------------------------------
    always @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n) begin
            wptr_bin  <= 0;
            wptr_gray <= 0;
        end else if (push && !full) begin
            fifo_mem[wptr_bin[AWIDTH-1:0]] <= wdata;
            wptr_bin  <= wptr_bin + 1'b1;
            wptr_gray <= bin2gray(wptr_bin + 1'b1);
        end
        // if push && full we just drop it -- the checker assertion will catch this
    end

    // ----------------------------------------------------------------
    // Read logic (rclk domain)
    // ----------------------------------------------------------------
    always @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) begin
            rptr_bin  <= 0;
            rptr_gray <= 0;
        end else if (pop && !empty) begin
            rptr_bin  <= rptr_bin + 1'b1;
            rptr_gray <= bin2gray(rptr_bin + 1'b1);
        end
    end

    // ----------------------------------------------------------------
    // 2-FF sync: rptr_gray -> wclk
    // Important: these regs must NOT have reset driven by rclk.
    // Both FFs use wclk so they're purely in the write domain.
    // ----------------------------------------------------------------
    always @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n) begin
            rptr_gray_sync1 <= 0;
            rptr_gray_sync2 <= 0;
        end else begin
            rptr_gray_sync1 <= rptr_gray;
            rptr_gray_sync2 <= rptr_gray_sync1;
        end
    end

    // 2-FF sync: wptr_gray -> rclk
    always @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) begin
            wptr_gray_sync1 <= 0;
            wptr_gray_sync2 <= 0;
        end else begin
            wptr_gray_sync1 <= wptr_gray;
            wptr_gray_sync2 <= wptr_gray_sync1;
        end
    end

    // ----------------------------------------------------------------
    // Full detection (write domain)
    // Full when write pointer has wrapped around and caught up to read pointer.
    // In gray code, this means: top bit is inverted, second bit is inverted,
    // rest are equal. This is the Cummings condition.
    // Took me a while to convince myself this is correct but it works.
    // ----------------------------------------------------------------
    assign full = (wptr_gray[AWIDTH]   == ~rptr_gray_sync2[AWIDTH])   &&
                  (wptr_gray[AWIDTH-1] == ~rptr_gray_sync2[AWIDTH-1]) &&
                  (wptr_gray[AWIDTH-2:0] == rptr_gray_sync2[AWIDTH-2:0]);

    assign almost_full = ((wptr_bin + 1'b1) == {~rptr_gray_sync2[AWIDTH],
                                                  rptr_gray_sync2[AWIDTH-1:0]});

    // ----------------------------------------------------------------
    // Empty detection (read domain)
    // Empty when both gray pointers are equal -- simple!
    // ----------------------------------------------------------------
    assign empty = (rptr_gray == wptr_gray_sync2);

    assign almost_empty = ((rptr_bin + 1'b1) == {~wptr_gray_sync2[AWIDTH],
                                                   wptr_gray_sync2[AWIDTH-1:0]});

    // FWFT: read data is just the current read address, no latency
    assign rdata = fifo_mem[rptr_bin[AWIDTH-1:0]];

endmodule
