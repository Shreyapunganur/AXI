`timescale 1ns/1ps
// Full Integration Testbench -- AXI4 Crossbar + CDC
// Shreya | MNNIT Allahabad
//
// Tests:
//   Phase 1 -- basic write/read through crossbar (sanity check)
//   Phase 2 -- concurrent access to different slaves (proves crossbar works)
//   Phase 3 -- contention on same slave with different QoS (M0 wins, QoS=0xA > 0x3)
//   Phase 4 -- CDC path: traffic goes from 100MHz domain through FIFO to 83MHz slave
//   Phase 5 -- burst transactions (len=7 = 8 beats)
//   Phase 6 -- 50 randomized transactions to hit coverage holes
//
// Topology:
//   M0 (pipelined) --> ICN_v2 --> S0 (direct, same clk)
//   M1 (pipelined) --> ICN_v2 --> CDC bridge --> S1 (different clk)
//
// Things I had to debug:
//   - CDC: rst_b needs to deassert separately (S1 is on clk_b, not clk_a)
//   - The 1000ns run in the tcl script cuts simulation short, do "run all" instead
//   - Scoreboard was giving false FAILs until I initialized shadow memory to 0
//   - xsim ignores "cover property" statements -- just warnings, not errors

`include "AXI4_rand_stimulus.sv"
import axi4_pkg::*;

module AXI4_tb_full;

    localparam ADWIDTH    = 32;
    localparam DWIDTH     = 64;
    localparam IDWIDTH    = 4;
    localparam MDEPTH     = 256;
    localparam CLK_A_HALF = 5;    // 100 MHz
    localparam CLK_B_HALF = 6;    // ~83 MHz, intentionally different from A

    localparam [ADWIDTH-1:0] S0_BASE = 32'h0000_0000;
    localparam [ADWIDTH-1:0] S0_HIGH = 32'h0000_07FF;
    localparam [ADWIDTH-1:0] S1_BASE = 32'h0001_0000;
    localparam [ADWIDTH-1:0] S1_HIGH = 32'h0001_07FF;

    // clocks -- two independent oscillators
    logic clk_a = 0, clk_b = 0;
    logic rst_a = 0, rst_b = 0;   // active low, like the master/slave

    always #CLK_A_HALF clk_a = ~clk_a;
    always #CLK_B_HALF clk_b = ~clk_b;

    // master control interface
    logic               m0_start=0, m0_rd_wr=0;
    logic [ADWIDTH-1:0] m0_addrin=0;
    logic [7:0]         m0_burstlen=0;
    logic [DWIDTH-1:0]  m0_wdin=0;
    logic [3:0]         m0_qosin=0;
    logic [DWIDTH-1:0]  m0_rdout;
    logic               m0_done;

    logic               m1_start=0, m1_rd_wr=0;
    logic [ADWIDTH-1:0] m1_addrin=0;
    logic [7:0]         m1_burstlen=0;
    logic [DWIDTH-1:0]  m1_wdin=0;
    logic [3:0]         m1_qosin=0;
    logic [DWIDTH-1:0]  m1_rdout;
    logic               m1_done;

    // AXI buses -- M0 <-> ICN
    logic [IDWIDTH-1:0] m0_awid, m0_bid, m0_arid, m0_rid;
    logic [ADWIDTH-1:0] m0_awaddr, m0_araddr;
    logic [7:0]         m0_awlen, m0_arlen;
    logic [2:0]         m0_awsize, m0_arsize;
    logic [1:0]         m0_awburst, m0_arburst, m0_bresp, m0_rresp;
    logic [3:0]         m0_awqos, m0_arqos;
    logic               m0_awvalid, m0_awready, m0_wlast, m0_wvalid, m0_wready;
    logic               m0_bvalid, m0_bready, m0_arvalid, m0_arready;
    logic               m0_rlast, m0_rvalid, m0_rready;
    logic [DWIDTH-1:0]  m0_wdata, m0_rdata;
    logic [DWIDTH/8-1:0] m0_wstrb;

    // AXI buses -- M1 <-> ICN
    logic [IDWIDTH-1:0] m1_awid, m1_bid, m1_arid, m1_rid;
    logic [ADWIDTH-1:0] m1_awaddr, m1_araddr;
    logic [7:0]         m1_awlen, m1_arlen;
    logic [2:0]         m1_awsize, m1_arsize;
    logic [1:0]         m1_awburst, m1_arburst, m1_bresp, m1_rresp;
    logic [3:0]         m1_awqos, m1_arqos;
    logic               m1_awvalid, m1_awready, m1_wlast, m1_wvalid, m1_wready;
    logic               m1_bvalid, m1_bready, m1_arvalid, m1_arready;
    logic               m1_rlast, m1_rvalid, m1_rready;
    logic [DWIDTH-1:0]  m1_wdata, m1_rdata;
    logic [DWIDTH/8-1:0] m1_wstrb;

    // ICN <-> S0 (direct path, same clk_a)
    logic [IDWIDTH-1:0] s0_awid, s0_bid, s0_arid, s0_rid;
    logic [ADWIDTH-1:0] s0_awaddr, s0_araddr;
    logic [7:0]         s0_awlen, s0_arlen;
    logic [2:0]         s0_awsize, s0_arsize;
    logic [1:0]         s0_awburst, s0_arburst, s0_bresp, s0_rresp;
    logic [3:0]         s0_awqos, s0_arqos;
    logic               s0_awvalid, s0_awready, s0_wlast, s0_wvalid, s0_wready;
    logic               s0_bvalid, s0_bready, s0_arvalid, s0_arready;
    logic               s0_rlast, s0_rvalid, s0_rready;
    logic [DWIDTH-1:0]  s0_wdata, s0_rdata;
    logic [DWIDTH/8-1:0] s0_wstrb;

    // ICN -> CDC bridge (A-side, clk_a)
    logic [IDWIDTH-1:0] ia_awid, ia_bid, ia_arid, ia_rid;
    logic [ADWIDTH-1:0] ia_awaddr, ia_araddr;
    logic [7:0]         ia_awlen, ia_arlen;
    logic [2:0]         ia_awsize, ia_arsize;
    logic [1:0]         ia_awburst, ia_arburst, ia_bresp, ia_rresp;
    logic [3:0]         ia_awqos, ia_arqos;
    logic               ia_awvalid, ia_awready, ia_wlast, ia_wvalid, ia_wready;
    logic               ia_bvalid, ia_bready, ia_arvalid, ia_arready;
    logic               ia_rlast, ia_rvalid, ia_rready;
    logic [DWIDTH-1:0]  ia_wdata, ia_rdata;
    logic [DWIDTH/8-1:0] ia_wstrb;

    // CDC bridge -> S1 (B-side, clk_b)
    logic [IDWIDTH-1:0] s1_awid, s1_bid, s1_arid, s1_rid;
    logic [ADWIDTH-1:0] s1_awaddr, s1_araddr;
    logic [7:0]         s1_awlen, s1_arlen;
    logic [2:0]         s1_awsize, s1_arsize;
    logic [1:0]         s1_awburst, s1_arburst, s1_bresp, s1_rresp;
    logic               s1_awvalid, s1_awready, s1_wlast, s1_wvalid, s1_wready;
    logic               s1_bvalid, s1_bready, s1_arvalid, s1_arready;
    logic               s1_rlast, s1_rvalid, s1_rready;
    logic [DWIDTH-1:0]  s1_wdata, s1_rdata;
    logic [DWIDTH/8-1:0] s1_wstrb;

    // for CDC assertions
    logic cdc_aw_full, cdc_r_full;
    wire  cdc_aw_push  = ia_awvalid && !cdc_aw_full;
    wire  cdc_r_push   = s1_rvalid  && !cdc_r_full;
    wire  cdc_active   = ia_awvalid || ia_arvalid || ia_wvalid;

    // -------------------------------------------------------------------
    // DUT instantiation
    // -------------------------------------------------------------------

    AXI4_master_pipelined #(.adwidth(ADWIDTH), .dwidth(DWIDTH),
                             .idwidth(IDWIDTH), .MASTER_ID(1)) u_m0 (
        .clk(clk_a), .rst(rst_a),
        .start(m0_start), .rd_wr(m0_rd_wr),
        .addrin(m0_addrin), .burstlen(m0_burstlen),
        .wdin(m0_wdin), .qosin(m0_qosin),
        .rdout(m0_rdout), .done(m0_done),
        .awid(m0_awid), .awaddr(m0_awaddr), .awlen(m0_awlen),
        .awsize(m0_awsize), .awburst(m0_awburst), .awqos(m0_awqos),
        .awvalid(m0_awvalid), .awready(m0_awready),
        .wdata(m0_wdata), .wstrb(m0_wstrb), .wlast(m0_wlast),
        .wvalid(m0_wvalid), .wready(m0_wready),
        .bid(m0_bid), .bresp(m0_bresp), .bvalid(m0_bvalid), .bready(m0_bready),
        .arid(m0_arid), .araddr(m0_araddr), .arlen(m0_arlen),
        .arsize(m0_arsize), .arburst(m0_arburst), .arqos(m0_arqos),
        .arvalid(m0_arvalid), .arready(m0_arready),
        .rid(m0_rid), .rdata(m0_rdata), .rresp(m0_rresp),
        .rlast(m0_rlast), .rvalid(m0_rvalid), .rready(m0_rready)
    );

    AXI4_master_pipelined #(.adwidth(ADWIDTH), .dwidth(DWIDTH),
                             .idwidth(IDWIDTH), .MASTER_ID(2)) u_m1 (
        .clk(clk_a), .rst(rst_a),
        .start(m1_start), .rd_wr(m1_rd_wr),
        .addrin(m1_addrin), .burstlen(m1_burstlen),
        .wdin(m1_wdin), .qosin(m1_qosin),
        .rdout(m1_rdout), .done(m1_done),
        .awid(m1_awid), .awaddr(m1_awaddr), .awlen(m1_awlen),
        .awsize(m1_awsize), .awburst(m1_awburst), .awqos(m1_awqos),
        .awvalid(m1_awvalid), .awready(m1_awready),
        .wdata(m1_wdata), .wstrb(m1_wstrb), .wlast(m1_wlast),
        .wvalid(m1_wvalid), .wready(m1_wready),
        .bid(m1_bid), .bresp(m1_bresp), .bvalid(m1_bvalid), .bready(m1_bready),
        .arid(m1_arid), .araddr(m1_araddr), .arlen(m1_arlen),
        .arsize(m1_arsize), .arburst(m1_arburst), .arqos(m1_arqos),
        .arvalid(m1_arvalid), .arready(m1_arready),
        .rid(m1_rid), .rdata(m1_rdata), .rresp(m1_rresp),
        .rlast(m1_rlast), .rvalid(m1_rvalid), .rready(m1_rready)
    );

    AXI4_interconnect_v2 #(
        .ADWIDTH(ADWIDTH), .DWIDTH(DWIDTH), .IDWIDTH(IDWIDTH),
        .S0_BASE(S0_BASE), .S0_HIGH(S0_HIGH),
        .S1_BASE(S1_BASE), .S1_HIGH(S1_HIGH)
    ) u_icn (
        .clk(clk_a), .rst(rst_a),
        .m0_awid(m0_awid), .m0_awaddr(m0_awaddr), .m0_awlen(m0_awlen),
        .m0_awsize(m0_awsize), .m0_awburst(m0_awburst), .m0_awqos(m0_awqos),
        .m0_awvalid(m0_awvalid), .m0_awready(m0_awready),
        .m0_wdata(m0_wdata), .m0_wstrb(m0_wstrb), .m0_wlast(m0_wlast),
        .m0_wvalid(m0_wvalid), .m0_wready(m0_wready),
        .m0_bid(m0_bid), .m0_bresp(m0_bresp), .m0_bvalid(m0_bvalid), .m0_bready(m0_bready),
        .m0_arid(m0_arid), .m0_araddr(m0_araddr), .m0_arlen(m0_arlen),
        .m0_arsize(m0_arsize), .m0_arburst(m0_arburst), .m0_arqos(m0_arqos),
        .m0_arvalid(m0_arvalid), .m0_arready(m0_arready),
        .m0_rid(m0_rid), .m0_rdata(m0_rdata), .m0_rresp(m0_rresp),
        .m0_rlast(m0_rlast), .m0_rvalid(m0_rvalid), .m0_rready(m0_rready),
        .m1_awid(m1_awid), .m1_awaddr(m1_awaddr), .m1_awlen(m1_awlen),
        .m1_awsize(m1_awsize), .m1_awburst(m1_awburst), .m1_awqos(m1_awqos),
        .m1_awvalid(m1_awvalid), .m1_awready(m1_awready),
        .m1_wdata(m1_wdata), .m1_wstrb(m1_wstrb), .m1_wlast(m1_wlast),
        .m1_wvalid(m1_wvalid), .m1_wready(m1_wready),
        .m1_bid(m1_bid), .m1_bresp(m1_bresp), .m1_bvalid(m1_bvalid), .m1_bready(m1_bready),
        .m1_arid(m1_arid), .m1_araddr(m1_araddr), .m1_arlen(m1_arlen),
        .m1_arsize(m1_arsize), .m1_arburst(m1_arburst), .m1_arqos(m1_arqos),
        .m1_arvalid(m1_arvalid), .m1_arready(m1_arready),
        .m1_rid(m1_rid), .m1_rdata(m1_rdata), .m1_rresp(m1_rresp),
        .m1_rlast(m1_rlast), .m1_rvalid(m1_rvalid), .m1_rready(m1_rready),
        .s0_awid(s0_awid), .s0_awaddr(s0_awaddr), .s0_awlen(s0_awlen),
        .s0_awsize(s0_awsize), .s0_awburst(s0_awburst), .s0_awqos(s0_awqos),
        .s0_awvalid(s0_awvalid), .s0_awready(s0_awready),
        .s0_wdata(s0_wdata), .s0_wstrb(s0_wstrb), .s0_wlast(s0_wlast),
        .s0_wvalid(s0_wvalid), .s0_wready(s0_wready),
        .s0_bid(s0_bid), .s0_bresp(s0_bresp), .s0_bvalid(s0_bvalid), .s0_bready(s0_bready),
        .s0_arid(s0_arid), .s0_araddr(s0_araddr), .s0_arlen(s0_arlen),
        .s0_arsize(s0_arsize), .s0_arburst(s0_arburst), .s0_arqos(s0_arqos),
        .s0_arvalid(s0_arvalid), .s0_arready(s0_arready),
        .s0_rid(s0_rid), .s0_rdata(s0_rdata), .s0_rresp(s0_rresp),
        .s0_rlast(s0_rlast), .s0_rvalid(s0_rvalid), .s0_rready(s0_rready),
        // S1 goes through CDC bridge, so connect ICN S1 ports to CDC A-side
        .s1_awid(ia_awid), .s1_awaddr(ia_awaddr), .s1_awlen(ia_awlen),
        .s1_awsize(ia_awsize), .s1_awburst(ia_awburst), .s1_awqos(ia_awqos),
        .s1_awvalid(ia_awvalid), .s1_awready(ia_awready),
        .s1_wdata(ia_wdata), .s1_wstrb(ia_wstrb), .s1_wlast(ia_wlast),
        .s1_wvalid(ia_wvalid), .s1_wready(ia_wready),
        .s1_bid(ia_bid), .s1_bresp(ia_bresp), .s1_bvalid(ia_bvalid), .s1_bready(ia_bready),
        .s1_arid(ia_arid), .s1_araddr(ia_araddr), .s1_arlen(ia_arlen),
        .s1_arsize(ia_arsize), .s1_arburst(ia_arburst), .s1_arqos(ia_arqos),
        .s1_arvalid(ia_arvalid), .s1_arready(ia_arready),
        .s1_rid(ia_rid), .s1_rdata(ia_rdata), .s1_rresp(ia_rresp),
        .s1_rlast(ia_rlast), .s1_rvalid(ia_rvalid), .s1_rready(ia_rready)
    );

    AXI4_cdc_bridge #(.ADWIDTH(ADWIDTH), .DWIDTH(DWIDTH),
                      .IDWIDTH(IDWIDTH), .FIFO_DEPTH(8)) u_cdc (
        .clk_a(clk_a), .rst_a_n(rst_a),
        .ia_awid(ia_awid), .ia_awaddr(ia_awaddr), .ia_awlen(ia_awlen),
        .ia_awsize(ia_awsize), .ia_awburst(ia_awburst), .ia_awqos(ia_awqos),
        .ia_awvalid(ia_awvalid), .ia_awready(ia_awready),
        .ia_wdata(ia_wdata), .ia_wstrb(ia_wstrb), .ia_wlast(ia_wlast),
        .ia_wvalid(ia_wvalid), .ia_wready(ia_wready),
        .ia_bid(ia_bid), .ia_bresp(ia_bresp), .ia_bvalid(ia_bvalid), .ia_bready(ia_bready),
        .ia_arid(ia_arid), .ia_araddr(ia_araddr), .ia_arlen(ia_arlen),
        .ia_arsize(ia_arsize), .ia_arburst(ia_arburst), .ia_arqos(ia_arqos),
        .ia_arvalid(ia_arvalid), .ia_arready(ia_arready),
        .ia_rid(ia_rid), .ia_rdata(ia_rdata), .ia_rresp(ia_rresp),
        .ia_rlast(ia_rlast), .ia_rvalid(ia_rvalid), .ia_rready(ia_rready),
        .clk_b(clk_b), .rst_b_n(rst_b),
        .ob_awid(s1_awid), .ob_awaddr(s1_awaddr), .ob_awlen(s1_awlen),
        .ob_awsize(s1_awsize), .ob_awburst(s1_awburst), .ob_awqos(),
        .ob_awvalid(s1_awvalid), .ob_awready(s1_awready),
        .ob_wdata(s1_wdata), .ob_wstrb(s1_wstrb), .ob_wlast(s1_wlast),
        .ob_wvalid(s1_wvalid), .ob_wready(s1_wready),
        .ob_bid(s1_bid), .ob_bresp(s1_bresp), .ob_bvalid(s1_bvalid), .ob_bready(s1_bready),
        .ob_arid(s1_arid), .ob_araddr(s1_araddr), .ob_arlen(s1_arlen),
        .ob_arsize(s1_arsize), .ob_arburst(s1_arburst), .ob_arqos(),
        .ob_arvalid(s1_arvalid), .ob_arready(s1_arready),
        .ob_rid(s1_rid), .ob_rdata(s1_rdata), .ob_rresp(s1_rresp),
        .ob_rlast(s1_rlast), .ob_rvalid(s1_rvalid), .ob_rready(s1_rready),
        .aw_fifo_full_o(cdc_aw_full), .r_fifo_full_o(cdc_r_full)
    );

    // S0 on clk_a
    AXI4_slave #(.adwidth(ADWIDTH), .dwidth(DWIDTH),
                 .idwidth(IDWIDTH), .mdepth(MDEPTH)) u_s0 (
        .clk(clk_a), .rst(rst_a),
        .awid(s0_awid), .awaddr(s0_awaddr), .awlen(s0_awlen),
        .awsize(s0_awsize), .awburst(s0_awburst), .awvalid(s0_awvalid), .awready(s0_awready),
        .wdata(s0_wdata), .wstrb(s0_wstrb), .wlast(s0_wlast),
        .wvalid(s0_wvalid), .wready(s0_wready),
        .bid(s0_bid), .bresp(s0_bresp), .bvalid(s0_bvalid), .bready(s0_bready),
        .arid(s0_arid), .araddr(s0_araddr), .arlen(s0_arlen),
        .arsize(s0_arsize), .arburst(s0_arburst), .arvalid(s0_arvalid), .arready(s0_arready),
        .rid(s0_rid), .rdata(s0_rdata), .rresp(s0_rresp),
        .rlast(s0_rlast), .rvalid(s0_rvalid), .rready(s0_rready)
    );

    // S1 on clk_b -- different clock domain, that's the whole CDC point
    AXI4_slave #(.adwidth(ADWIDTH), .dwidth(DWIDTH),
                 .idwidth(IDWIDTH), .mdepth(MDEPTH)) u_s1 (
        .clk(clk_b), .rst(rst_b),
        .awid(s1_awid), .awaddr(s1_awaddr), .awlen(s1_awlen),
        .awsize(s1_awsize), .awburst(s1_awburst), .awvalid(s1_awvalid), .awready(s1_awready),
        .wdata(s1_wdata), .wstrb(s1_wstrb), .wlast(s1_wlast),
        .wvalid(s1_wvalid), .wready(s1_wready),
        .bid(s1_bid), .bresp(s1_bresp), .bvalid(s1_bvalid), .bready(s1_bready),
        .arid(s1_arid), .araddr(s1_araddr), .arlen(s1_arlen),
        .arsize(s1_arsize), .arburst(s1_arburst), .arvalid(s1_arvalid), .arready(s1_arready),
        .rid(s1_rid), .rdata(s1_rdata), .rresp(s1_rresp),
        .rlast(s1_rlast), .rvalid(s1_rvalid), .rready(s1_rready)
    );

    // verification modules
    AXI4_checker_v2 #(.ADWIDTH(ADWIDTH), .DWIDTH(DWIDTH), .IDWIDTH(IDWIDTH),
                      .S0_BASE(S0_BASE), .S0_HIGH(S0_HIGH),
                      .S1_BASE(S1_BASE), .S1_HIGH(S1_HIGH)) u_chk (
        .clk(clk_a), .rst(rst_a),
        .m0_awid(m0_awid), .m0_awaddr(m0_awaddr), .m0_awlen(m0_awlen),
        .m0_awsize(m0_awsize), .m0_awburst(m0_awburst), .m0_awqos(m0_awqos),
        .m0_awvalid(m0_awvalid), .m0_awready(m0_awready),
        .m0_wdata(m0_wdata), .m0_wstrb(m0_wstrb), .m0_wlast(m0_wlast),
        .m0_wvalid(m0_wvalid), .m0_wready(m0_wready),
        .m0_bid(m0_bid), .m0_bresp(m0_bresp), .m0_bvalid(m0_bvalid), .m0_bready(m0_bready),
        .m0_arid(m0_arid), .m0_araddr(m0_araddr), .m0_arlen(m0_arlen),
        .m0_arsize(m0_arsize), .m0_arburst(m0_arburst), .m0_arqos(m0_arqos),
        .m0_arvalid(m0_arvalid), .m0_arready(m0_arready),
        .m0_rid(m0_rid), .m0_rdata(m0_rdata), .m0_rresp(m0_rresp),
        .m0_rlast(m0_rlast), .m0_rvalid(m0_rvalid), .m0_rready(m0_rready),
        .m1_awid(m1_awid), .m1_awaddr(m1_awaddr), .m1_awlen(m1_awlen),
        .m1_awsize(m1_awsize), .m1_awburst(m1_awburst), .m1_awqos(m1_awqos),
        .m1_awvalid(m1_awvalid), .m1_awready(m1_awready),
        .m1_wdata(m1_wdata), .m1_wstrb(m1_wstrb), .m1_wlast(m1_wlast),
        .m1_wvalid(m1_wvalid), .m1_wready(m1_wready),
        .m1_bid(m1_bid), .m1_bresp(m1_bresp), .m1_bvalid(m1_bvalid), .m1_bready(m1_bready),
        .m1_arid(m1_arid), .m1_araddr(m1_araddr), .m1_arlen(m1_arlen),
        .m1_arsize(m1_arsize), .m1_arburst(m1_arburst), .m1_arqos(m1_arqos),
        .m1_arvalid(m1_arvalid), .m1_arready(m1_arready),
        .m1_rid(m1_rid), .m1_rdata(m1_rdata), .m1_rresp(m1_rresp),
        .m1_rlast(m1_rlast), .m1_rvalid(m1_rvalid), .m1_rready(m1_rready),
        .cdc_aw_fifo_full(cdc_aw_full), .cdc_r_fifo_full(cdc_r_full),
        .cdc_aw_push(cdc_aw_push), .cdc_r_push(cdc_r_push)
    );

    AXI4_coverage_v2 #(.ADWIDTH(ADWIDTH), .DWIDTH(DWIDTH), .IDWIDTH(IDWIDTH),
                       .S0_BASE(S0_BASE), .S0_HIGH(S0_HIGH),
                       .S1_BASE(S1_BASE), .S1_HIGH(S1_HIGH)) u_cov (
        .clk(clk_a), .rst(rst_a),
        .m0_awaddr(m0_awaddr), .m0_awlen(m0_awlen), .m0_awsize(m0_awsize),
        .m0_awburst(m0_awburst), .m0_awqos(m0_awqos),
        .m0_awvalid(m0_awvalid), .m0_awready(m0_awready),
        .m0_wvalid(m0_wvalid), .m0_wready(m0_wready),
        .m0_bresp(m0_bresp), .m0_bvalid(m0_bvalid), .m0_bready(m0_bready),
        .m0_araddr(m0_araddr), .m0_arlen(m0_arlen), .m0_arsize(m0_arsize),
        .m0_arburst(m0_arburst), .m0_arqos(m0_arqos),
        .m0_arvalid(m0_arvalid), .m0_arready(m0_arready),
        .m0_rresp(m0_rresp), .m0_rvalid(m0_rvalid), .m0_rready(m0_rready), .m0_rlast(m0_rlast),
        .m1_awaddr(m1_awaddr), .m1_awlen(m1_awlen), .m1_awsize(m1_awsize),
        .m1_awburst(m1_awburst), .m1_awqos(m1_awqos),
        .m1_awvalid(m1_awvalid), .m1_awready(m1_awready),
        .m1_wvalid(m1_wvalid), .m1_wready(m1_wready),
        .m1_bresp(m1_bresp), .m1_bvalid(m1_bvalid), .m1_bready(m1_bready),
        .m1_araddr(m1_araddr), .m1_arlen(m1_arlen), .m1_arsize(m1_arsize),
        .m1_arburst(m1_arburst), .m1_arqos(m1_arqos),
        .m1_arvalid(m1_arvalid), .m1_arready(m1_arready),
        .m1_rresp(m1_rresp), .m1_rvalid(m1_rvalid), .m1_rready(m1_rready), .m1_rlast(m1_rlast),
        .cdc_active(cdc_active)
    );

    AXI4_scoreboard #(.ADWIDTH(ADWIDTH), .DWIDTH(DWIDTH),
                      .IDWIDTH(IDWIDTH), .MEM_DEPTH(MDEPTH)) u_sb (
        .clk(clk_a), .rst(rst_a),
        .s0_awaddr(s0_awaddr), .s0_awlen(s0_awlen),
        .s0_awvalid(s0_awvalid), .s0_awready(s0_awready),
        .s0_wdata(s0_wdata), .s0_wstrb(s0_wstrb), .s0_wlast(s0_wlast),
        .s0_wvalid(s0_wvalid), .s0_wready(s0_wready),
        .s0_araddr(s0_araddr), .s0_arvalid(s0_arvalid), .s0_arready(s0_arready),
        .s0_rid(s0_rid), .s0_rdata(s0_rdata), .s0_rlast(s0_rlast),
        .s0_rvalid(s0_rvalid), .s0_rready(s0_rready),
        .s1_awaddr(ia_awaddr), .s1_awlen(ia_awlen),
        .s1_awvalid(ia_awvalid), .s1_awready(ia_awready),
        .s1_wdata(ia_wdata), .s1_wstrb(ia_wstrb), .s1_wlast(ia_wlast),
        .s1_wvalid(ia_wvalid), .s1_wready(ia_wready),
        .s1_araddr(ia_araddr), .s1_arvalid(ia_arvalid), .s1_arready(ia_arready),
        .s1_rid(ia_rid), .s1_rdata(ia_rdata), .s1_rlast(ia_rlast),
        .s1_rvalid(ia_rvalid), .s1_rready(ia_rready),
        .m0_bid(m0_bid), .m0_bvalid(m0_bvalid), .m0_bready(m0_bready),
        .m1_bid(m1_bid), .m1_bvalid(m1_bvalid), .m1_bready(m1_bready),
        .m0_awid(m0_awid), .m0_awvalid_in(m0_awvalid), .m0_awready_in(m0_awready),
        .m1_awid(m1_awid), .m1_awvalid_in(m1_awvalid), .m1_awready_in(m1_awready)
    );

    AXI4_perf_monitor #(.ADWIDTH(ADWIDTH), .DWIDTH(DWIDTH), .IDWIDTH(IDWIDTH)) u_perf (
        .clk(clk_a), .rst(rst_a),
        .m0_awvalid(m0_awvalid), .m0_awready(m0_awready),
        .m0_wvalid(m0_wvalid), .m0_wready(m0_wready), .m0_wlast(m0_wlast),
        .m0_bvalid(m0_bvalid), .m0_bready(m0_bready),
        .m0_arvalid(m0_arvalid), .m0_arready(m0_arready),
        .m0_rvalid(m0_rvalid), .m0_rready(m0_rready), .m0_rlast(m0_rlast),
        .m1_awvalid(m1_awvalid), .m1_awready(m1_awready),
        .m1_wvalid(m1_wvalid), .m1_wready(m1_wready), .m1_wlast(m1_wlast),
        .m1_bvalid(m1_bvalid), .m1_bready(m1_bready),
        .m1_arvalid(m1_arvalid), .m1_arready(m1_arready),
        .m1_rvalid(m1_rvalid), .m1_rready(m1_rready), .m1_rlast(m1_rlast)
    );

    // watchdog -- don't want simulation hanging forever
    int wdog_cnt = 0;
    always_ff @(posedge clk_a) begin
        if (!rst_a) wdog_cnt <= 0;
        else begin
            wdog_cnt <= wdog_cnt + 1;
            if (wdog_cnt > 60000) begin
                $display("[WATCHDOG] sim seems stuck at %0t ns, killing it", $time/1000);
                $finish;
            end
        end
    end

    // -------------------------------------------------------------------
    // Tasks -- just drive the master control signals and wait for done
    // -------------------------------------------------------------------

    task automatic m0_write(input logic [ADWIDTH-1:0] addr, input logic [7:0] blen,
                             input logic [DWIDTH-1:0] wdat, input logic [3:0] qos);
        @(negedge clk_a);
        m0_rd_wr=1; m0_addrin=addr; m0_burstlen=blen; m0_wdin=wdat; m0_qosin=qos;
        m0_start=1;
        @(negedge clk_a); m0_start=0;
        @(posedge clk_a); while (!m0_done) @(posedge clk_a);
        @(negedge clk_a);
    endtask

    task automatic m0_read(input logic [ADWIDTH-1:0] addr, input logic [7:0] blen,
                            output logic [DWIDTH-1:0] rd, input logic [3:0] qos);
        @(negedge clk_a);
        m0_rd_wr=0; m0_addrin=addr; m0_burstlen=blen; m0_qosin=qos; m0_start=1;
        @(negedge clk_a); m0_start=0;
        @(posedge clk_a); while (!m0_done) @(posedge clk_a);
        rd = m0_rdout;
        @(negedge clk_a);
    endtask

    task automatic m1_write(input logic [ADWIDTH-1:0] addr, input logic [7:0] blen,
                             input logic [DWIDTH-1:0] wdat, input logic [3:0] qos);
        @(negedge clk_a);
        m1_rd_wr=1; m1_addrin=addr; m1_burstlen=blen; m1_wdin=wdat; m1_qosin=qos;
        m1_start=1;
        @(negedge clk_a); m1_start=0;
        @(posedge clk_a); while (!m1_done) @(posedge clk_a);
        @(negedge clk_a);
    endtask

    task automatic m1_read(input logic [ADWIDTH-1:0] addr, input logic [7:0] blen,
                            output logic [DWIDTH-1:0] rd, input logic [3:0] qos);
        @(negedge clk_a);
        m1_rd_wr=0; m1_addrin=addr; m1_burstlen=blen; m1_qosin=qos; m1_start=1;
        @(negedge clk_a); m1_start=0;
        @(posedge clk_a); while (!m1_done) @(posedge clk_a);
        rd = m1_rdout;
        @(negedge clk_a);
    endtask

    int pass_cnt = 0, fail_cnt = 0;

    task automatic check_data(input logic [DWIDTH-1:0] got, exp, input string label);
        if (got === exp) begin
            $display("  PASS [%s] 0x%016h", label, got);
            pass_cnt++;
        end else begin
            $display("  FAIL [%s] got=0x%016h expected=0x%016h", label, got, exp);
            fail_cnt++;
        end
    endtask

    // -------------------------------------------------------------------
    // Main test sequence
    // -------------------------------------------------------------------
    logic [DWIDTH-1:0] r0, r1;
    AXI4_transaction txn;

    initial begin
        $dumpfile("AXI4_full_tb.vcd");
        $dumpvars(0, AXI4_tb_full);

        // reset sequence -- rst_b is separate because S1 is on clk_b
        rst_a = 0; rst_b = 0;
        repeat(6) @(posedge clk_a);
        rst_a = 1;
        repeat(5) @(posedge clk_b);
        rst_b = 1;
        repeat(4) @(posedge clk_a);

        $display("\n=== Phase 1: Basic write/read (sanity check) ===");
        m0_write(32'h0000_0000, 8'd0, 64'hAAAA_1111_BBBB_2222, 4'h0);
        m0_read (32'h0000_0000, 8'd0, r0, 4'h0);
        check_data(r0, 64'hAAAA_1111_BBBB_2222, "P1:M0->S0");

        m1_write(32'h0000_0008, 8'd0, 64'hCCCC_3333_DDDD_4444, 4'h0);
        m1_read (32'h0000_0008, 8'd0, r1, 4'h0);
        check_data(r1, 64'hCCCC_3333_DDDD_4444, "P1:M1->S0");

        $display("\n=== Phase 2: Concurrent access to different slaves ===");
        // this is the key test -- M0 to S0 and M1 to S1 should NOT block each other
        $display("    (if crossbar works, both should complete faster than sequential)");
        fork
            m0_write(32'h0000_0010, 8'd0, 64'hEEEE_5555_FFFF_6666, 4'h0);
            m1_write(32'h0001_0000, 8'd0, 64'h1111_AAAA_2222_BBBB, 4'h0);
        join
        m0_read(32'h0000_0010, 8'd0, r0, 4'h0);
        check_data(r0, 64'hEEEE_5555_FFFF_6666, "P2:M0->S0");
        m1_read(32'h0001_0000, 8'd0, r1, 4'h0);
        check_data(r1, 64'h1111_AAAA_2222_BBBB, "P2:M1->S1(CDC)");

        $display("\n=== Phase 3: Contention on S0 with QoS ===");
        // M0 has QoS=0xA (10), M1 has QoS=0x3 (3), so M0 should win
        fork
            m0_write(32'h0000_0020, 8'd0, 64'hFACE_CAFE_0001_0002, 4'hA);
            m1_write(32'h0000_0028, 8'd0, 64'hDEAD_BEEF_0003_0004, 4'h3);
        join
        m0_read(32'h0000_0020, 8'd0, r0, 4'h0);
        check_data(r0, 64'hFACE_CAFE_0001_0002, "P3:M0 QoS=A (should win)");
        m1_read(32'h0000_0028, 8'd0, r1, 4'h0);
        check_data(r1, 64'hDEAD_BEEF_0003_0004, "P3:M1 QoS=3");

        $display("\n=== Phase 4: CDC path (data crossing clock domains) ===");
        // this one takes longer because of the FIFO synchronization delay
        m0_write(32'h0001_0008, 8'd0, 64'hC0DE_C0DE_FEED_FACE, 4'h0);
        m0_read (32'h0001_0008, 8'd0, r0, 4'h0);
        check_data(r0, 64'hC0DE_C0DE_FEED_FACE, "P4:M0->S1 via CDC");

        m1_write(32'h0001_0010, 8'd0, 64'h1234_5678_9ABC_DEF0, 4'h0);
        m1_read (32'h0001_0010, 8'd0, r1, 4'h0);
        check_data(r1, 64'h1234_5678_9ABC_DEF0, "P4:M1->S1 via CDC");

        $display("\n=== Phase 5: Burst transactions ===");
        m0_write(32'h0000_0040, 8'd7, 64'hBEEF_DEAD_CAFE_1234, 4'h0);
        m0_read (32'h0000_0040, 8'd7, r0, 4'h0);
        check_data(r0, 64'hBEEF_DEAD_CAFE_1234, "P5:M0 burst8->S0");

        fork
            m0_write(32'h0000_0080, 8'd3, 64'hA5A5_B6B6_C7C7_D8D8, 4'h5);
            m1_write(32'h0001_0020, 8'd3, 64'hE9E9_FAFA_0B0B_1C1C, 4'h5);
        join
        m0_read(32'h0000_0080, 8'd3, r0, 4'h0);
        check_data(r0, 64'hA5A5_B6B6_C7C7_D8D8, "P5:M0 burst4->S0");
        m1_read(32'h0001_0020, 8'd3, r1, 4'h0);
        check_data(r1, 64'hE9E9_FAFA_0B0B_1C1C, "P5:M1 burst4->S1(CDC)");

        $display("\n=== Phase 6: Constrained random (50 transactions) ===");
        txn = new();
        repeat(50) begin
            assert(txn.randomize()) else $fatal(1, "randomize() failed -- check constraints");
            $display("  txn: %s", txn.to_str());

            if (txn.rd_wr) begin
                if (txn.target_slave == 0)
                    m0_write(txn.addr, txn.len, txn.data, txn.qos);
                else
                    m1_write(txn.addr, txn.len, txn.data, txn.qos);
            end else begin
                if (txn.target_slave == 0)
                    m0_read(txn.addr, txn.len, r0, txn.qos);
                else
                    m1_read(txn.addr, txn.len, r1, txn.qos);
            end
        end

        repeat(20) @(posedge clk_a);

        $display("\n======================");
        $display("  Simulation done");
        $display("  PASS: %0d  FAIL: %0d", pass_cnt, fail_cnt);
        $display("  (scoreboard results above, coverage printed at end)");
        $display("======================\n");
        $finish;
    end

endmodule
