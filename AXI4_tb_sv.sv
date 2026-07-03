`timescale 1ns / 1ps
// =============================================================
// AXI4_tb_sv.sv — SystemVerilog Top-Level Testbench
//
// Replaces AXI4_tb.v as the simulation top.
// AXI4_master.v and AXI4_slave.v are instantiated UNTOUCHED.
// Adds:  AXI4_checker.sv  (SVA, passive)
//        AXI4_coverage.sv (covergroups, passive)
//
// Extended to 8 test cases to improve coverage hits.
// =============================================================

module AXI4_tb_sv;

    localparam ADWIDTH  = 32;
    localparam DWIDTH   = 64;
    localparam IDWIDTH  = 4;
    localparam MDEPTH   = 16;
    localparam CLK_HALF = 5;

    logic clk = 0;
    logic rst;

    always #CLK_HALF clk = ~clk;

    // ----------------------------------------------------------
    // AXI4 bus signals (logic type — SV equivalent of wire/reg)
    // ----------------------------------------------------------
    // AW
    logic [IDWIDTH-1:0]  awid;
    logic [ADWIDTH-1:0]  awaddr;
    logic [7:0]          awlen;
    logic [2:0]          awsize;
    logic [1:0]          awburst;
    logic                awvalid, awready;
    // W
    logic [DWIDTH-1:0]   wdata;
    logic [DWIDTH/8-1:0] wstrb;
    logic                wlast, wvalid, wready;
    // B
    logic [IDWIDTH-1:0]  bid;
    logic [1:0]          bresp;
    logic                bvalid, bready;
    // AR
    logic [IDWIDTH-1:0]  arid;
    logic [ADWIDTH-1:0]  araddr;
    logic [7:0]          arlen;
    logic [2:0]          arsize;
    logic [1:0]          arburst;
    logic                arvalid, arready;
    // R
    logic [IDWIDTH-1:0]  rid;
    logic [DWIDTH-1:0]   rdata;
    logic [1:0]          rresp;
    logic                rlast, rvalid, rready;

    // Testbench control
    logic               start    = 0;
    logic               rd_wr    = 0;
    logic [ADWIDTH-1:0] addrin   = '0;
    logic [7:0]         burstlen = '0;
    logic [DWIDTH-1:0]  wdin     = '0;
    logic [DWIDTH-1:0]  rdout;
    logic               done;

    // ----------------------------------------------------------
    // DUT: existing .v master — instantiated UNTOUCHED
    // ----------------------------------------------------------
    AXI4_master #(
        .adwidth (ADWIDTH),
        .dwidth  (DWIDTH),
        .idwidth (IDWIDTH)
    ) u_master (
        .clk      (clk),   .rst      (rst),
        .start    (start), .rd_wr    (rd_wr),
        .addrin   (addrin),.burstlen (burstlen),
        .wdin     (wdin),  .rdout    (rdout),  .done (done),
        .awid     (awid),  .awaddr   (awaddr), .awlen   (awlen),
        .awsize   (awsize),.awburst  (awburst),.awvalid (awvalid),
        .awready  (awready),
        .wdata    (wdata), .wstrb    (wstrb),  .wlast   (wlast),
        .wvalid   (wvalid),.wready   (wready),
        .bid      (bid),   .bresp    (bresp),  .bvalid  (bvalid),
        .bready   (bready),
        .arid     (arid),  .araddr   (araddr), .arlen   (arlen),
        .arsize   (arsize),.arburst  (arburst),.arvalid (arvalid),
        .arready  (arready),
        .rid      (rid),   .rdata    (rdata),  .rresp   (rresp),
        .rlast    (rlast), .rvalid   (rvalid), .rready  (rready)
    );

    // DUT: existing .v slave — instantiated UNTOUCHED
    AXI4_slave #(
        .adwidth (ADWIDTH),
        .dwidth  (DWIDTH),
        .idwidth (IDWIDTH),
        .mdepth  (MDEPTH)
    ) u_slave (
        .clk     (clk),   .rst     (rst),
        .awid    (awid),  .awaddr  (awaddr), .awlen   (awlen),
        .awsize  (awsize),.awburst (awburst),.awvalid (awvalid),
        .awready (awready),
        .wdata   (wdata), .wstrb   (wstrb),  .wlast   (wlast),
        .wvalid  (wvalid),.wready  (wready),
        .bid     (bid),   .bresp   (bresp),  .bvalid  (bvalid),
        .bready  (bready),
        .arid    (arid),  .araddr  (araddr), .arlen   (arlen),
        .arsize  (arsize),.arburst (arburst),.arvalid (arvalid),
        .arready (arready),
        .rid     (rid),   .rdata   (rdata),  .rresp   (rresp),
        .rlast   (rlast), .rvalid  (rvalid), .rready  (rready)
    );

    // ----------------------------------------------------------
    // SVA Checker — passive, no bus driving
    // ----------------------------------------------------------
    AXI4_checker #(
        .ADWIDTH (ADWIDTH),
        .DWIDTH  (DWIDTH),
        .IDWIDTH (IDWIDTH)
    ) u_checker (
        .clk     (clk),   .rst     (rst),
        .awid    (awid),  .awaddr  (awaddr), .awlen   (awlen),
        .awsize  (awsize),.awburst (awburst),.awvalid (awvalid),
        .awready (awready),
        .wdata   (wdata), .wstrb   (wstrb),  .wlast   (wlast),
        .wvalid  (wvalid),.wready  (wready),
        .bid     (bid),   .bresp   (bresp),  .bvalid  (bvalid),
        .bready  (bready),
        .arid    (arid),  .araddr  (araddr), .arlen   (arlen),
        .arsize  (arsize),.arburst (arburst),.arvalid (arvalid),
        .arready (arready),
        .rid     (rid),   .rdata   (rdata),  .rresp   (rresp),
        .rlast   (rlast), .rvalid  (rvalid), .rready  (rready)
    );

    // ----------------------------------------------------------
    // Coverage Collector — passive, no bus driving
    // ----------------------------------------------------------
    AXI4_coverage #(
        .ADWIDTH (ADWIDTH),
        .DWIDTH  (DWIDTH),
        .IDWIDTH (IDWIDTH)
    ) u_coverage (
        .clk     (clk),   .rst     (rst),
        .awid    (awid),  .awaddr  (awaddr), .awlen   (awlen),
        .awsize  (awsize),.awburst (awburst),.awvalid (awvalid),
        .awready (awready),
        .wdata   (wdata), .wstrb   (wstrb),  .wlast   (wlast),
        .wvalid  (wvalid),.wready  (wready),
        .bid     (bid),   .bresp   (bresp),  .bvalid  (bvalid),
        .bready  (bready),
        .arid    (arid),  .araddr  (araddr), .arlen   (arlen),
        .arsize  (arsize),.arburst (arburst),.arvalid (arvalid),
        .arready (arready),
        .rid     (rid),   .rdata   (rdata),  .rresp   (rresp),
        .rlast   (rlast), .rvalid  (rvalid), .rready  (rready)
    );

    // ----------------------------------------------------------
    // Watchdog
    // ----------------------------------------------------------
    int timeout_cnt;
    always_ff @(posedge clk) begin
        if (!rst) timeout_cnt <= 0;
        else begin
            timeout_cnt <= timeout_cnt + 1;
            if (timeout_cnt > 5000) begin
                $display("[TIMEOUT %0t] Simulation hung — check FSM states", $time);
                $finish;
            end
        end
    end

    // ----------------------------------------------------------
    // Tasks (automatic — thread-safe for future parallel use)
    // ----------------------------------------------------------
    task automatic do_write(
        input logic [ADWIDTH-1:0] addr,
        input logic [7:0]         blen,
        input logic [DWIDTH-1:0]  wdat
    );
        @(negedge clk);
        rd_wr = 1; addrin = addr; burstlen = blen; wdin = wdat; start = 1;
        @(negedge clk); start = 0;
        @(posedge clk);
        while (!done) @(posedge clk);
        @(negedge clk);
    endtask

    task automatic do_read(
        input  logic [ADWIDTH-1:0] addr,
        input  logic [7:0]         blen,
        output logic [DWIDTH-1:0]  rd_data
    );
        @(negedge clk);
        rd_wr = 0; addrin = addr; burstlen = blen; start = 1;
        @(negedge clk); start = 0;
        @(posedge clk);
        while (!done) @(posedge clk);
        rd_data = rdout;
        @(negedge clk);
    endtask

    int pass_cnt, fail_cnt;

    task automatic check_result(
        input logic [DWIDTH-1:0] got,
        input logic [DWIDTH-1:0] exp,
        input int                tc_num
    );
        if (got === exp) begin
            $display("  [%0t] TC%0d PASS  got=0x%016h", $time, tc_num, got);
            pass_cnt++;
        end else begin
            $display("  [%0t] TC%0d FAIL  got=0x%016h  exp=0x%016h",
                     $time, tc_num, got, exp);
            fail_cnt++;
        end
    endtask

    // ----------------------------------------------------------
    // Stimulus — 8 test cases for better coverage
    // ----------------------------------------------------------
    logic [DWIDTH-1:0] rd_buf;

    initial begin
        $dumpfile("AXI4_sv_tb.vcd");
        $dumpvars(0, AXI4_tb_sv);
        pass_cnt = 0; fail_cnt = 0;

        rst = 0; repeat(5) @(posedge clk);
        rst = 1; repeat(3) @(posedge clk);

        // --- TC1-TC6: original directed tests (unchanged) ---
        $display("\n=== TC1: Single-beat WRITE  addr=0x00  len=0 ===");
        do_write(32'h0000_0000, 8'd0, 64'hDEAD_BEEF_CAFE_1234);

        $display("\n=== TC2: 4-beat WRITE  addr=0x08  len=3 ===");
        do_write(32'h0000_0008, 8'd3, 64'hAAAA_0001_BBBB_0002);

        $display("\n=== TC3: Single-beat READ   addr=0x00  len=0 ===");
        do_read(32'h0000_0000, 8'd0, rd_buf);
        check_result(rd_buf, 64'hDEAD_BEEF_CAFE_1234, 3);

        $display("\n=== TC4: 4-beat READ   addr=0x08  len=3 ===");
        do_read(32'h0000_0008, 8'd3, rd_buf);
        check_result(rd_buf, 64'hAAAA_0001_BBBB_0002, 4);

        $display("\n=== TC5: Back-to-back W->R   addr=0x40  len=0 ===");
        do_write(32'h0000_0040, 8'd0, 64'h1234_5678_9ABC_DEF0);
        do_read (32'h0000_0040, 8'd0, rd_buf);
        check_result(rd_buf, 64'h1234_5678_9ABC_DEF0, 5);

        $display("\n=== TC6: 2-beat burst W->R   addr=0x18  len=1 ===");
        do_write(32'h0000_0018, 8'd1, 64'hCAFE_BABE_1234_5678);
        do_read (32'h0000_0018, 8'd1, rd_buf);
        check_result(rd_buf, 64'hCAFE_BABE_1234_5678, 6);

        // --- TC7-TC8: extra tests to hit more coverage bins ---
        $display("\n=== TC7: 8-beat WRITE+READ   addr=0x00  len=7 ===");
        do_write(32'h0000_0000, 8'd7, 64'hFEED_FACE_DEAD_BEEF);
        do_read (32'h0000_0000, 8'd7, rd_buf);
        check_result(rd_buf, 64'hFEED_FACE_DEAD_BEEF, 7);

        $display("\n=== TC8: Boundary WRITE+READ addr=0x78  len=0 ===");
        do_write(32'h0000_0078, 8'd0, 64'hC0DE_CAFE_1111_2222);
        do_read (32'h0000_0078, 8'd0, rd_buf);
        check_result(rd_buf, 64'hC0DE_CAFE_1111_2222, 8);

        repeat(5) @(posedge clk);
        $display("\n========================================");
        $display("  SIMULATION COMPLETE");
        $display("  PASS : %0d  FAIL : %0d", pass_cnt, fail_cnt);
        $display("========================================");
        $finish;
    end

endmodule
