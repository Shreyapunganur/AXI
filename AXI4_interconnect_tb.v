`timescale 1ns / 1ps
// =============================================================
// AXI4_interconnect_tb.v — Testbench for 2x2 Crossbar
//
// Topology:
//   Master 0 (CPU sim) ──┐             ┌── Slave 0 (0x0000_0000..0xFF)
//                         ├─ CROSSBAR ──┤
//   Master 1 (DMA sim) ──┘             └── Slave 1 (0x0001_0000..0xFF)
//
// Test cases:
//   TC1 : M0 writes S0, M1 writes S1  (concurrent, different slaves)
//   TC2 : M0 reads  S0, M1 reads  S1  (concurrent read)
//   TC3 : M0 reads  S1, M1 reads  S0  (cross-slave read)
//   TC4 : M0 and M1 both write S0     (arbitration — one stalls)
//   TC5 : 4-beat burst M0→S0, M1→S1  (concurrent bursts)
//   TC6 : Verify round-robin: alternate M0/M1 grants on S0
// =============================================================

module AXI4_interconnect_tb;

    localparam ADWIDTH  = 32;
    localparam DWIDTH   = 64;
    localparam IDWIDTH  = 4;
    localparam MDEPTH   = 256; // larger depth for two slaves
    localparam CLK_HALF = 5;

    reg clk = 0;
    reg rst;

    always #CLK_HALF clk = ~clk;

    // ----------------------------------------------------------
    // Master 0 control
    // ----------------------------------------------------------
    reg               m0_start    = 0;
    reg               m0_rd_wr    = 0;
    reg  [ADWIDTH-1:0] m0_addrin  = 0;
    reg  [7:0]         m0_burstlen= 0;
    reg  [DWIDTH-1:0]  m0_wdin    = 0;
    wire [DWIDTH-1:0]  m0_rdout;
    wire               m0_done;

    // Master 1 control
    reg               m1_start    = 0;
    reg               m1_rd_wr    = 0;
    reg  [ADWIDTH-1:0] m1_addrin  = 0;
    reg  [7:0]         m1_burstlen= 0;
    reg  [DWIDTH-1:0]  m1_wdin    = 0;
    wire [DWIDTH-1:0]  m1_rdout;
    wire               m1_done;

    // ----------------------------------------------------------
    // AXI4 wires — Master 0 ↔ Interconnect
    // ----------------------------------------------------------
    wire [IDWIDTH-1:0]  m0_awid,  m0_bid,  m0_arid,  m0_rid;
    wire [ADWIDTH-1:0]  m0_awaddr, m0_araddr;
    wire [7:0]          m0_awlen,  m0_arlen;
    wire [2:0]          m0_awsize, m0_arsize;
    wire [1:0]          m0_awburst,m0_arburst, m0_bresp, m0_rresp;
    wire                m0_awvalid,m0_awready,m0_wlast,m0_wvalid,m0_wready;
    wire                m0_bvalid, m0_bready, m0_arvalid,m0_arready;
    wire                m0_rlast,  m0_rvalid, m0_rready;
    wire [DWIDTH-1:0]   m0_wdata,  m0_rdata;
    wire [DWIDTH/8-1:0] m0_wstrb;

    // AXI4 wires — Master 1 ↔ Interconnect
    wire [IDWIDTH-1:0]  m1_awid,  m1_bid,  m1_arid,  m1_rid;
    wire [ADWIDTH-1:0]  m1_awaddr, m1_araddr;
    wire [7:0]          m1_awlen,  m1_arlen;
    wire [2:0]          m1_awsize, m1_arsize;
    wire [1:0]          m1_awburst,m1_arburst, m1_bresp, m1_rresp;
    wire                m1_awvalid,m1_awready,m1_wlast,m1_wvalid,m1_wready;
    wire                m1_bvalid, m1_bready, m1_arvalid,m1_arready;
    wire                m1_rlast,  m1_rvalid, m1_rready;
    wire [DWIDTH-1:0]   m1_wdata,  m1_rdata;
    wire [DWIDTH/8-1:0] m1_wstrb;

    // AXI4 wires — Interconnect ↔ Slave 0
    wire [IDWIDTH-1:0]  s0_awid,  s0_bid,  s0_arid,  s0_rid;
    wire [ADWIDTH-1:0]  s0_awaddr, s0_araddr;
    wire [7:0]          s0_awlen,  s0_arlen;
    wire [2:0]          s0_awsize, s0_arsize;
    wire [1:0]          s0_awburst,s0_arburst, s0_bresp, s0_rresp;
    wire                s0_awvalid,s0_awready,s0_wlast,s0_wvalid,s0_wready;
    wire                s0_bvalid, s0_bready, s0_arvalid,s0_arready;
    wire                s0_rlast,  s0_rvalid, s0_rready;
    wire [DWIDTH-1:0]   s0_wdata,  s0_rdata;
    wire [DWIDTH/8-1:0] s0_wstrb;

    // AXI4 wires — Interconnect ↔ Slave 1
    wire [IDWIDTH-1:0]  s1_awid,  s1_bid,  s1_arid,  s1_rid;
    wire [ADWIDTH-1:0]  s1_awaddr, s1_araddr;
    wire [7:0]          s1_awlen,  s1_arlen;
    wire [2:0]          s1_awsize, s1_arsize;
    wire [1:0]          s1_awburst,s1_arburst, s1_bresp, s1_rresp;
    wire                s1_awvalid,s1_awready,s1_wlast,s1_wvalid,s1_wready;
    wire                s1_bvalid, s1_bready, s1_arvalid,s1_arready;
    wire                s1_rlast,  s1_rvalid, s1_rready;
    wire [DWIDTH-1:0]   s1_wdata,  s1_rdata;
    wire [DWIDTH/8-1:0] s1_wstrb;

    // ----------------------------------------------------------
    // Master 0 instance (existing .v, untouched)
    // ----------------------------------------------------------
    AXI4_master #(.adwidth(ADWIDTH),.dwidth(DWIDTH),.idwidth(IDWIDTH)) u_m0 (
        .clk(clk),.rst(rst),
        .start(m0_start),.rd_wr(m0_rd_wr),
        .addrin(m0_addrin),.burstlen(m0_burstlen),
        .wdin(m0_wdin),.rdout(m0_rdout),.done(m0_done),
        .awid(m0_awid),.awaddr(m0_awaddr),.awlen(m0_awlen),
        .awsize(m0_awsize),.awburst(m0_awburst),
        .awvalid(m0_awvalid),.awready(m0_awready),
        .wdata(m0_wdata),.wstrb(m0_wstrb),.wlast(m0_wlast),
        .wvalid(m0_wvalid),.wready(m0_wready),
        .bid(m0_bid),.bresp(m0_bresp),.bvalid(m0_bvalid),.bready(m0_bready),
        .arid(m0_arid),.araddr(m0_araddr),.arlen(m0_arlen),
        .arsize(m0_arsize),.arburst(m0_arburst),
        .arvalid(m0_arvalid),.arready(m0_arready),
        .rid(m0_rid),.rdata(m0_rdata),.rresp(m0_rresp),
        .rlast(m0_rlast),.rvalid(m0_rvalid),.rready(m0_rready)
    );

    // Master 1 instance
    AXI4_master #(.adwidth(ADWIDTH),.dwidth(DWIDTH),.idwidth(IDWIDTH)) u_m1 (
        .clk(clk),.rst(rst),
        .start(m1_start),.rd_wr(m1_rd_wr),
        .addrin(m1_addrin),.burstlen(m1_burstlen),
        .wdin(m1_wdin),.rdout(m1_rdout),.done(m1_done),
        .awid(m1_awid),.awaddr(m1_awaddr),.awlen(m1_awlen),
        .awsize(m1_awsize),.awburst(m1_awburst),
        .awvalid(m1_awvalid),.awready(m1_awready),
        .wdata(m1_wdata),.wstrb(m1_wstrb),.wlast(m1_wlast),
        .wvalid(m1_wvalid),.wready(m1_wready),
        .bid(m1_bid),.bresp(m1_bresp),.bvalid(m1_bvalid),.bready(m1_bready),
        .arid(m1_arid),.araddr(m1_araddr),.arlen(m1_arlen),
        .arsize(m1_arsize),.arburst(m1_arburst),
        .arvalid(m1_arvalid),.arready(m1_arready),
        .rid(m1_rid),.rdata(m1_rdata),.rresp(m1_rresp),
        .rlast(m1_rlast),.rvalid(m1_rvalid),.rready(m1_rready)
    );

    // ----------------------------------------------------------
    // Interconnect instance
    // ----------------------------------------------------------
    AXI4_interconnect #(
        .ADWIDTH(ADWIDTH),.DWIDTH(DWIDTH),.IDWIDTH(IDWIDTH),
        .S0_BASE(32'h0000_0000),.S0_HIGH(32'h0000_07FF),
        .S1_BASE(32'h0001_0000),.S1_HIGH(32'h0001_07FF)
    ) u_icn (
        .clk(clk),.rst(rst),
        // M0
        .m0_awid(m0_awid),.m0_awaddr(m0_awaddr),.m0_awlen(m0_awlen),
        .m0_awsize(m0_awsize),.m0_awburst(m0_awburst),
        .m0_awvalid(m0_awvalid),.m0_awready(m0_awready),
        .m0_wdata(m0_wdata),.m0_wstrb(m0_wstrb),.m0_wlast(m0_wlast),
        .m0_wvalid(m0_wvalid),.m0_wready(m0_wready),
        .m0_bid(m0_bid),.m0_bresp(m0_bresp),
        .m0_bvalid(m0_bvalid),.m0_bready(m0_bready),
        .m0_arid(m0_arid),.m0_araddr(m0_araddr),.m0_arlen(m0_arlen),
        .m0_arsize(m0_arsize),.m0_arburst(m0_arburst),
        .m0_arvalid(m0_arvalid),.m0_arready(m0_arready),
        .m0_rid(m0_rid),.m0_rdata(m0_rdata),.m0_rresp(m0_rresp),
        .m0_rlast(m0_rlast),.m0_rvalid(m0_rvalid),.m0_rready(m0_rready),
        // M1
        .m1_awid(m1_awid),.m1_awaddr(m1_awaddr),.m1_awlen(m1_awlen),
        .m1_awsize(m1_awsize),.m1_awburst(m1_awburst),
        .m1_awvalid(m1_awvalid),.m1_awready(m1_awready),
        .m1_wdata(m1_wdata),.m1_wstrb(m1_wstrb),.m1_wlast(m1_wlast),
        .m1_wvalid(m1_wvalid),.m1_wready(m1_wready),
        .m1_bid(m1_bid),.m1_bresp(m1_bresp),
        .m1_bvalid(m1_bvalid),.m1_bready(m1_bready),
        .m1_arid(m1_arid),.m1_araddr(m1_araddr),.m1_arlen(m1_arlen),
        .m1_arsize(m1_arsize),.m1_arburst(m1_arburst),
        .m1_arvalid(m1_arvalid),.m1_arready(m1_arready),
        .m1_rid(m1_rid),.m1_rdata(m1_rdata),.m1_rresp(m1_rresp),
        .m1_rlast(m1_rlast),.m1_rvalid(m1_rvalid),.m1_rready(m1_rready),
        // S0
        .s0_awid(s0_awid),.s0_awaddr(s0_awaddr),.s0_awlen(s0_awlen),
        .s0_awsize(s0_awsize),.s0_awburst(s0_awburst),
        .s0_awvalid(s0_awvalid),.s0_awready(s0_awready),
        .s0_wdata(s0_wdata),.s0_wstrb(s0_wstrb),.s0_wlast(s0_wlast),
        .s0_wvalid(s0_wvalid),.s0_wready(s0_wready),
        .s0_bid(s0_bid),.s0_bresp(s0_bresp),
        .s0_bvalid(s0_bvalid),.s0_bready(s0_bready),
        .s0_arid(s0_arid),.s0_araddr(s0_araddr),.s0_arlen(s0_arlen),
        .s0_arsize(s0_arsize),.s0_arburst(s0_arburst),
        .s0_arvalid(s0_arvalid),.s0_arready(s0_arready),
        .s0_rid(s0_rid),.s0_rdata(s0_rdata),.s0_rresp(s0_rresp),
        .s0_rlast(s0_rlast),.s0_rvalid(s0_rvalid),.s0_rready(s0_rready),
        // S1
        .s1_awid(s1_awid),.s1_awaddr(s1_awaddr),.s1_awlen(s1_awlen),
        .s1_awsize(s1_awsize),.s1_awburst(s1_awburst),
        .s1_awvalid(s1_awvalid),.s1_awready(s1_awready),
        .s1_wdata(s1_wdata),.s1_wstrb(s1_wstrb),.s1_wlast(s1_wlast),
        .s1_wvalid(s1_wvalid),.s1_wready(s1_wready),
        .s1_bid(s1_bid),.s1_bresp(s1_bresp),
        .s1_bvalid(s1_bvalid),.s1_bready(s1_bready),
        .s1_arid(s1_arid),.s1_araddr(s1_araddr),.s1_arlen(s1_arlen),
        .s1_arsize(s1_arsize),.s1_arburst(s1_arburst),
        .s1_arvalid(s1_arvalid),.s1_arready(s1_arready),
        .s1_rid(s1_rid),.s1_rdata(s1_rdata),.s1_rresp(s1_rresp),
        .s1_rlast(s1_rlast),.s1_rvalid(s1_rvalid),.s1_rready(s1_rready)
    );

    // ----------------------------------------------------------
    // Slave 0 instance (existing .v, untouched)
    // ----------------------------------------------------------
    AXI4_slave #(
        .adwidth(ADWIDTH),.dwidth(DWIDTH),.idwidth(IDWIDTH),.mdepth(MDEPTH)
    ) u_s0 (
        .clk(clk),.rst(rst),
        .awid(s0_awid),.awaddr(s0_awaddr),.awlen(s0_awlen),
        .awsize(s0_awsize),.awburst(s0_awburst),
        .awvalid(s0_awvalid),.awready(s0_awready),
        .wdata(s0_wdata),.wstrb(s0_wstrb),.wlast(s0_wlast),
        .wvalid(s0_wvalid),.wready(s0_wready),
        .bid(s0_bid),.bresp(s0_bresp),.bvalid(s0_bvalid),.bready(s0_bready),
        .arid(s0_arid),.araddr(s0_araddr),.arlen(s0_arlen),
        .arsize(s0_arsize),.arburst(s0_arburst),
        .arvalid(s0_arvalid),.arready(s0_arready),
        .rid(s0_rid),.rdata(s0_rdata),.rresp(s0_rresp),
        .rlast(s0_rlast),.rvalid(s0_rvalid),.rready(s0_rready)
    );

    // Slave 1 instance
    AXI4_slave #(
        .adwidth(ADWIDTH),.dwidth(DWIDTH),.idwidth(IDWIDTH),.mdepth(MDEPTH)
    ) u_s1 (
        .clk(clk),.rst(rst),
        .awid(s1_awid),.awaddr(s1_awaddr),.awlen(s1_awlen),
        .awsize(s1_awsize),.awburst(s1_awburst),
        .awvalid(s1_awvalid),.awready(s1_awready),
        .wdata(s1_wdata),.wstrb(s1_wstrb),.wlast(s1_wlast),
        .wvalid(s1_wvalid),.wready(s1_wready),
        .bid(s1_bid),.bresp(s1_bresp),.bvalid(s1_bvalid),.bready(s1_bready),
        .arid(s1_arid),.araddr(s1_araddr),.arlen(s1_arlen),
        .arsize(s1_arsize),.arburst(s1_arburst),
        .arvalid(s1_arvalid),.arready(s1_arready),
        .rid(s1_rid),.rdata(s1_rdata),.rresp(s1_rresp),
        .rlast(s1_rlast),.rvalid(s1_rvalid),.rready(s1_rready)
    );

    // ----------------------------------------------------------
    // Watchdog
    // ----------------------------------------------------------
    integer timeout_cnt;
    always @(posedge clk) begin
        if (!rst) timeout_cnt <= 0;
        else begin
            timeout_cnt <= timeout_cnt + 1;
            if (timeout_cnt > 10000) begin
                $display("[TIMEOUT %0t] Hung — check interconnect FSM", $time);
                $finish;
            end
        end
    end

    // ----------------------------------------------------------
    // Per-master tasks
    // ----------------------------------------------------------
    task m0_write;
        input [ADWIDTH-1:0] addr;
        input [7:0]         blen;
        input [DWIDTH-1:0]  wdat;
        begin
            @(negedge clk);
            m0_rd_wr = 1; m0_addrin = addr;
            m0_burstlen = blen; m0_wdin = wdat; m0_start = 1;
            @(negedge clk); m0_start = 0;
            @(posedge clk);
            while (!m0_done) @(posedge clk);
            @(negedge clk);
        end
    endtask

    task m0_read;
        input  [ADWIDTH-1:0] addr;
        input  [7:0]         blen;
        output [DWIDTH-1:0]  rd_data;
        begin
            @(negedge clk);
            m0_rd_wr = 0; m0_addrin = addr;
            m0_burstlen = blen; m0_start = 1;
            @(negedge clk); m0_start = 0;
            @(posedge clk);
            while (!m0_done) @(posedge clk);
            rd_data = m0_rdout;
            @(negedge clk);
        end
    endtask

    task m1_write;
        input [ADWIDTH-1:0] addr;
        input [7:0]         blen;
        input [DWIDTH-1:0]  wdat;
        begin
            @(negedge clk);
            m1_rd_wr = 1; m1_addrin = addr;
            m1_burstlen = blen; m1_wdin = wdat; m1_start = 1;
            @(negedge clk); m1_start = 0;
            @(posedge clk);
            while (!m1_done) @(posedge clk);
            @(negedge clk);
        end
    endtask

    task m1_read;
        input  [ADWIDTH-1:0] addr;
        input  [7:0]         blen;
        output [DWIDTH-1:0]  rd_data;
        begin
            @(negedge clk);
            m1_rd_wr = 0; m1_addrin = addr;
            m1_burstlen = blen; m1_start = 1;
            @(negedge clk); m1_start = 0;
            @(posedge clk);
            while (!m1_done) @(posedge clk);
            rd_data = m1_rdout;
            @(negedge clk);
        end
    endtask

    // ----------------------------------------------------------
    // Result check
    // ----------------------------------------------------------
    integer pass_cnt;
    integer fail_cnt;

    task check_result;
        input [DWIDTH-1:0] got;
        input [DWIDTH-1:0] exp;
        input integer      tc_num;
        input [63:0]       master_id; // 0 or 1 for display
        begin
            if (got === exp) begin
                $display("  [%0t] TC%0d M%0d PASS  got=0x%016h",
                          $time, tc_num, master_id, got);
                pass_cnt = pass_cnt + 1;
            end else begin
                $display("  [%0t] TC%0d M%0d FAIL  got=0x%016h  exp=0x%016h",
                          $time, tc_num, master_id, got, exp);
                fail_cnt = fail_cnt + 1;
            end
        end
    endtask

    // ----------------------------------------------------------
    // Stimulus
    // ----------------------------------------------------------
    reg [DWIDTH-1:0] r0, r1;

    initial begin
        $dumpfile("AXI4_interconnect_tb.vcd");
        $dumpvars(0, AXI4_interconnect_tb);

        pass_cnt = 0; fail_cnt = 0;

        rst = 0; repeat(5) @(posedge clk);
        rst = 1; repeat(3) @(posedge clk);

        // -------------------------------------------------------
        // TC1: Concurrent writes — M0→S0, M1→S1 (different slaves)
        //      Both can proceed simultaneously (crossbar advantage)
        // -------------------------------------------------------
        $display("\n=== TC1: Concurrent WRITE  M0→S0(0x0000)  M1→S1(0x10000) ===");
        fork
            m0_write(32'h0000_0000, 8'd0, 64'hAAAA_AAAA_1111_1111);
            m1_write(32'h0001_0000, 8'd0, 64'hBBBB_BBBB_2222_2222);
        join
        $display("  Both writes done");

        // -------------------------------------------------------
        // TC2: Concurrent reads — M0←S0, M1←S1
        // -------------------------------------------------------
        $display("\n=== TC2: Concurrent READ   M0←S0(0x0000)  M1←S1(0x10000) ===");
        fork
            m0_read(32'h0000_0000, 8'd0, r0);
            m1_read(32'h0001_0000, 8'd0, r1);
        join
        check_result(r0, 64'hAAAA_AAAA_1111_1111, 2, 0);
        check_result(r1, 64'hBBBB_BBBB_2222_2222, 2, 1);

        // -------------------------------------------------------
        // TC3: Cross-slave access — M0→S1, M1→S0
        //      Write then read to verify routing
        // -------------------------------------------------------
        $display("\n=== TC3: Cross-slave WRITE M0→S1(0x10008)  M1→S0(0x0008) ===");
        m0_write(32'h0001_0008, 8'd0, 64'hCCCC_CCCC_3333_3333);
        m1_write(32'h0000_0008, 8'd0, 64'hDDDD_DDDD_4444_4444);

        $display("=== TC3: Cross-slave READ  M0←S1(0x10008)  M1←S0(0x0008) ===");
        m0_read(32'h0001_0008, 8'd0, r0);
        m1_read(32'h0000_0008, 8'd0, r1);
        check_result(r0, 64'hCCCC_CCCC_3333_3333, 3, 0);
        check_result(r1, 64'hDDDD_DDDD_4444_4444, 3, 1);

        // -------------------------------------------------------
        // TC4: Arbitration — both masters write to S0
        //      One stalls, the other proceeds; round-robin decides
        // -------------------------------------------------------
        $display("\n=== TC4: Arbitration — M0 and M1 both WRITE to S0 ===");
        fork
            m0_write(32'h0000_0010, 8'd0, 64'hEEEE_EEEE_5555_5555);
            m1_write(32'h0000_0018, 8'd0, 64'hFFFF_FFFF_6666_6666);
        join
        $display("  Arbitration done — one master was stalled, both completed");

        // Read back both addresses to verify no corruption
        m0_read(32'h0000_0010, 8'd0, r0);
        m1_read(32'h0000_0018, 8'd0, r1);
        check_result(r0, 64'hEEEE_EEEE_5555_5555, 4, 0);
        check_result(r1, 64'hFFFF_FFFF_6666_6666, 4, 1);

        // -------------------------------------------------------
        // TC5: 4-beat concurrent bursts — M0→S0 len=3, M1→S1 len=3
        // -------------------------------------------------------
        $display("\n=== TC5: 4-beat burst WRITE+READ  M0↔S0  M1↔S1 ===");
        fork
            m0_write(32'h0000_0020, 8'd3, 64'h1234_5678_9ABC_DEF0);
            m1_write(32'h0001_0020, 8'd3, 64'hFEDC_BA98_7654_3210);
        join
        fork
            m0_read(32'h0000_0020, 8'd3, r0);
            m1_read(32'h0001_0020, 8'd3, r1);
        join
        check_result(r0, 64'h1234_5678_9ABC_DEF0, 5, 0);
        check_result(r1, 64'hFEDC_BA98_7654_3210, 5, 1);

        // -------------------------------------------------------
        // TC6: Round-robin fairness — 4 consecutive M0+M1→S0
        //      grants should alternate: M0 M1 M0 M1
        // -------------------------------------------------------
        $display("\n=== TC6: Round-robin fairness — 4x contended writes to S0 ===");
        fork
            m0_write(32'h0000_0030, 8'd0, 64'hA0A0_A0A0_A0A0_A0A0);
            m1_write(32'h0000_0038, 8'd0, 64'hB1B1_B1B1_B1B1_B1B1);
        join
        fork
            m0_write(32'h0000_0040, 8'd0, 64'hC2C2_C2C2_C2C2_C2C2);
            m1_write(32'h0000_0048, 8'd0, 64'hD3D3_D3D3_D3D3_D3D3);
        join
        m0_read(32'h0000_0030, 8'd0, r0); check_result(r0,64'hA0A0_A0A0_A0A0_A0A0,6,0);
        m0_read(32'h0000_0040, 8'd0, r0); check_result(r0,64'hC2C2_C2C2_C2C2_C2C2,6,0);
        m1_read(32'h0000_0038, 8'd0, r1); check_result(r1,64'hB1B1_B1B1_B1B1_B1B1,6,1);
        m1_read(32'h0000_0048, 8'd0, r1); check_result(r1,64'hD3D3_D3D3_D3D3_D3D3,6,1);

        repeat(5) @(posedge clk);
        $display("\n========================================");
        $display("  INTERCONNECT TB COMPLETE");
        $display("  PASS : %0d", pass_cnt);
        $display("  FAIL : %0d", fail_cnt);
        $display("========================================\n");
        $finish;
    end

endmodule
