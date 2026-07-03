`timescale 1ns / 1ps

// AXI4 Pipelined Master
// Written by: Shreya (ECE, MNNIT Allahabad)
//
// The key difference from the basic master is that I'm driving AW and W
// at the same cycle. In the original version I was waiting for awready
// before asserting wvalid -- that wastes a whole cycle per transaction.
// After reading the AXI4 spec (A3.4.1) I realized there's no rule saying
// W has to wait for AW. So now both go out together.
//
// Also added QoS ports (awqos/arqos) because the interconnect needs to
// know the priority of each transaction for arbitration.
//
// Tested on Vivado 2024.1 xsim. Synthesis not done yet (sim only for now).

module AXI4_master_pipelined #(
    parameter adwidth  = 32,
    parameter dwidth   = 64,
    parameter idwidth  = 4,
    parameter MASTER_ID = 1     // each master needs a unique ID so responses route back correctly
)(
    input  wire               clk,
    input  wire               rst,           // active LOW (to match slave)

    // simple handshake with testbench
    input  wire               start,
    input  wire               rd_wr,         // 1 = write, 0 = read
    input  wire [adwidth-1:0] addrin,
    input  wire [7:0]         burstlen,      // awlen value (beats - 1)
    input  wire [dwidth-1:0]  wdin,
    input  wire [3:0]         qosin,         // QoS tag, 0 = lowest, 15 = highest
    output reg  [dwidth-1:0]  rdout,
    output reg                done,

    // AW channel
    output reg  [idwidth-1:0]  awid,
    output reg  [adwidth-1:0]  awaddr,
    output reg  [7:0]          awlen,
    output reg  [2:0]          awsize,
    output reg  [1:0]          awburst,
    output reg  [3:0]          awqos,
    output reg                 awvalid,
    input  wire                awready,

    // W channel
    output reg  [dwidth-1:0]   wdata,
    output reg  [dwidth/8-1:0] wstrb,
    output reg                 wlast,
    output reg                 wvalid,
    input  wire                wready,

    // B channel
    input  wire [idwidth-1:0]  bid,
    input  wire [1:0]          bresp,
    input  wire                bvalid,
    output reg                 bready,

    // AR channel
    output reg  [idwidth-1:0]  arid,
    output reg  [adwidth-1:0]  araddr,
    output reg  [7:0]          arlen,
    output reg  [2:0]          arsize,
    output reg  [1:0]          arburst,
    output reg  [3:0]          arqos,
    output reg                 arvalid,
    input  wire                arready,

    // R channel
    input  wire [idwidth-1:0]  rid,
    input  wire [dwidth-1:0]   rdata,
    input  wire [1:0]          rresp,
    input  wire                rlast,
    input  wire                rvalid,
    output reg                 rready
);

    // FSM states -- kept the same names as basic master so easy to compare
    localparam [2:0]
        IDLE     = 3'd0,
        WR_ISSUE = 3'd1,   // new state: AW + W go out at same time
        WR_RESP  = 3'd3,   // skipped 2 intentionally to match old encoding for easy waveform comparison
        RD_ADDR  = 3'd4,
        RD_DATA  = 3'd5,
        XACT_DONE= 3'd6;

    reg [2:0] state;
    reg [7:0] beatcnt;
    reg [7:0] burst_lat;    // latched copy of burstlen at start of txn
    reg       aw_accepted;  // flag: did AW handshake complete already?

    // I added this debug wire to check in waveform whether the pipeline is
    // actually working (AW and W in same cycle)
    wire dbg_pipeline_active = (state == WR_ISSUE) && awvalid && wvalid;

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            state       <= IDLE;
            beatcnt     <= 8'd0;
            burst_lat   <= 8'd0;
            aw_accepted <= 1'b0;
            done        <= 1'b0;
            awvalid     <= 1'b0;
            awid        <= 0; awaddr <= 0; awlen <= 0;
            awsize      <= 3'b011;   // 8 bytes (64-bit bus)
            awburst     <= 2'b01;    // INCR -- most common, WRAP needs aligned addr so handle later
            awqos       <= 4'h0;
            wvalid      <= 1'b0;
            wdata       <= 0; wstrb <= 0; wlast <= 1'b0;
            bready      <= 1'b0;
            arid        <= 0; araddr <= 0; arlen <= 0;
            arsize      <= 3'b011;
            arburst     <= 2'b01;
            arqos       <= 4'h0;
            arvalid     <= 1'b0;
            rready      <= 1'b0;
            rdout       <= 0;
        end else begin
            done <= 1'b0; // pulse for one cycle only

            case (state)

                IDLE: begin
                    if (start) begin
                        burst_lat   <= burstlen;
                        beatcnt     <= 8'd0;
                        aw_accepted <= 1'b0;

                        if (rd_wr) begin
                            // ------ WRITE PATH ------
                            // drive AW and W simultaneously -- this is the pipelining
                            awid    <= MASTER_ID[idwidth-1:0];
                            awaddr  <= addrin;
                            awlen   <= burstlen;
                            awsize  <= 3'b011;
                            awburst <= 2'b01;
                            awqos   <= qosin;
                            awvalid <= 1'b1;

                            // W channel goes out at same cycle as AW
                            wdata  <= wdin;
                            wstrb  <= {(dwidth/8){1'b1}};   // all byte lanes active
                            wlast  <= (burstlen == 8'd0);    // if single beat, last is immediate
                            wvalid <= 1'b1;

                            state <= WR_ISSUE;
                        end else begin
                            // ------ READ PATH ------
                            arid    <= MASTER_ID[idwidth-1:0];
                            araddr  <= addrin;
                            arlen   <= burstlen;
                            arsize  <= 3'b011;
                            arburst <= 2'b01;
                            arqos   <= qosin;
                            arvalid <= 1'b1;
                            state   <= RD_ADDR;
                        end
                    end
                end

                WR_ISSUE: begin
                    // AW and W channels can complete in any order
                    // so I track them independently with aw_accepted flag

                    // check if AW handshake completed this cycle
                    if (awvalid && awready) begin
                        awvalid     <= 1'b0;
                        aw_accepted <= 1'b1;
                    end

                    // process W beats as they get accepted
                    if (wvalid && wready) begin
                        beatcnt <= beatcnt + 8'd1;

                        if (wlast) begin
                            // last beat done, go wait for B response
                            wvalid <= 1'b0;
                            wlast  <= 1'b0;
                            bready <= 1'b1;
                            state  <= WR_RESP;
                        end else begin
                            wdata <= wdin;
                            // wlast goes high one beat before we hit burst_lat
                            // e.g. burstlen=3 means 4 beats (0,1,2,3)
                            // so last fires when beatcnt+1 == burst_lat
                            wlast <= (beatcnt + 8'd1 == burst_lat);
                        end
                    end
                end

                WR_RESP: begin
                    // just waiting for slave to send B response
                    if (bvalid && bready) begin
                        bready <= 1'b0;
                        // could check bresp here for SLVERR but skipping for now
                        state  <= XACT_DONE;
                    end
                end

                RD_ADDR: begin
                    if (arvalid && arready) begin
                        arvalid <= 1'b0;
                        rready  <= 1'b1;
                        state   <= RD_DATA;
                    end
                end

                RD_DATA: begin
                    if (rvalid && rready) begin
                        rdout   <= rdata;  // only saves last beat -- fine for our test
                        beatcnt <= beatcnt + 8'd1;

                        if (rlast) begin
                            rready <= 1'b0;
                            state  <= XACT_DONE;
                        end
                    end
                end

                XACT_DONE: begin
                    done  <= 1'b1;
                    state <= IDLE;
                end

                default: state <= IDLE;

            endcase
        end
    end

endmodule
