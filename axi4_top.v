`timescale 1ns/1ps
// =====================================================================
// axi4_top.v
// -----------------------------------------------------------------------
// Wires everything together:
//
//   Master 0 --\                              /-- Slave 0 (same clock
//               >-- axi4_crossbar (clk) ------<     as everything else)
//   Master 1 --/                              \-- axi4_cdc_bridge -- Slave 1
//                                                  (crosses into clk_s1)
//
// The testbench drives each master through its little command
// interface (cmd_*) and watches its response interface (rsp_*) - see
// axi4_master.v for what those mean. Everything else in here is just
// wiring.
// =====================================================================
module axi4_top #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter LID_WIDTH  = 2,               // per-master local ID width
    parameter ID_WIDTH   = LID_WIDTH + 1,   // expanded ID width used past the crossbar
    parameter STRB_WIDTH = DATA_WIDTH/8
)(
    input wire clk,          // masters, crossbar, Slave 0
    input wire clk_s1,       // Slave 1's own clock domain
    input wire rst_n,        // shared async reset (see note in axi4_cdc_bridge.v)

    // ---------------- Master 0 command / response interface ----------------
    input  wire                   m0_cmd_valid,
    output wire                   m0_cmd_ready,
    input  wire                   m0_cmd_write,
    input  wire [LID_WIDTH-1:0]   m0_cmd_id,
    input  wire [ADDR_WIDTH-1:0]  m0_cmd_addr,
    input  wire [7:0]             m0_cmd_len,
    input  wire [1:0]             m0_cmd_burst,
    input  wire [STRB_WIDTH-1:0]  m0_cmd_wstrb,
    input  wire [DATA_WIDTH-1:0]  m0_cmd_wseed,

    output wire                   m0_rsp_b_valid,
    output wire [LID_WIDTH-1:0]   m0_rsp_b_id,
    output wire [1:0]             m0_rsp_b_resp,
    output wire                   m0_rsp_r_valid,
    output wire [LID_WIDTH-1:0]   m0_rsp_r_id,
    output wire [1:0]             m0_rsp_r_resp,
    output wire [DATA_WIDTH-1:0]  m0_rsp_r_data,
    output wire                   m0_rsp_r_last,

    // ---------------- Master 1 command / response interface ----------------
    input  wire                   m1_cmd_valid,
    output wire                   m1_cmd_ready,
    input  wire                   m1_cmd_write,
    input  wire [LID_WIDTH-1:0]   m1_cmd_id,
    input  wire [ADDR_WIDTH-1:0]  m1_cmd_addr,
    input  wire [7:0]             m1_cmd_len,
    input  wire [1:0]             m1_cmd_burst,
    input  wire [STRB_WIDTH-1:0]  m1_cmd_wstrb,
    input  wire [DATA_WIDTH-1:0]  m1_cmd_wseed,

    output wire                   m1_rsp_b_valid,
    output wire [LID_WIDTH-1:0]   m1_rsp_b_id,
    output wire [1:0]             m1_rsp_b_resp,
    output wire                   m1_rsp_r_valid,
    output wire [LID_WIDTH-1:0]   m1_rsp_r_id,
    output wire [1:0]             m1_rsp_r_resp,
    output wire [DATA_WIDTH-1:0]  m1_rsp_r_data,
    output wire                   m1_rsp_r_last
);

    // ---------------- Master 0 <-> crossbar wires ----------------
    wire [LID_WIDTH-1:0]  m0_awid;   wire [ADDR_WIDTH-1:0] m0_awaddr; wire [7:0] m0_awlen;
    wire [2:0] m0_awsize; wire [1:0] m0_awburst; wire m0_awvalid, m0_awready;
    wire [DATA_WIDTH-1:0] m0_wdata;  wire [STRB_WIDTH-1:0] m0_wstrb; wire m0_wlast, m0_wvalid, m0_wready;
    wire [LID_WIDTH-1:0]  m0_bid;    wire [1:0] m0_bresp; wire m0_bvalid, m0_bready;
    wire [LID_WIDTH-1:0]  m0_arid;   wire [ADDR_WIDTH-1:0] m0_araddr; wire [7:0] m0_arlen;
    wire [2:0] m0_arsize; wire [1:0] m0_arburst; wire m0_arvalid, m0_arready;
    wire [LID_WIDTH-1:0]  m0_rid;    wire [DATA_WIDTH-1:0] m0_rdata; wire [1:0] m0_rresp;
    wire m0_rlast, m0_rvalid, m0_rready;

    axi4_master #(
        .ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH), .ID_WIDTH(LID_WIDTH)
    ) u_master0 (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(m0_cmd_valid), .cmd_ready(m0_cmd_ready), .cmd_write(m0_cmd_write),
        .cmd_id(m0_cmd_id), .cmd_addr(m0_cmd_addr), .cmd_len(m0_cmd_len), .cmd_burst(m0_cmd_burst),
        .cmd_wstrb(m0_cmd_wstrb), .cmd_wseed(m0_cmd_wseed),
        .rsp_b_valid(m0_rsp_b_valid), .rsp_b_id(m0_rsp_b_id), .rsp_b_resp(m0_rsp_b_resp),
        .rsp_r_valid(m0_rsp_r_valid), .rsp_r_id(m0_rsp_r_id), .rsp_r_resp(m0_rsp_r_resp),
        .rsp_r_data(m0_rsp_r_data), .rsp_r_last(m0_rsp_r_last),
        .m_awid(m0_awid), .m_awaddr(m0_awaddr), .m_awlen(m0_awlen), .m_awsize(m0_awsize),
        .m_awburst(m0_awburst), .m_awvalid(m0_awvalid), .m_awready(m0_awready),
        .m_wdata(m0_wdata), .m_wstrb(m0_wstrb), .m_wlast(m0_wlast), .m_wvalid(m0_wvalid), .m_wready(m0_wready),
        .m_bid(m0_bid), .m_bresp(m0_bresp), .m_bvalid(m0_bvalid), .m_bready(m0_bready),
        .m_arid(m0_arid), .m_araddr(m0_araddr), .m_arlen(m0_arlen), .m_arsize(m0_arsize),
        .m_arburst(m0_arburst), .m_arvalid(m0_arvalid), .m_arready(m0_arready),
        .m_rid(m0_rid), .m_rdata(m0_rdata), .m_rresp(m0_rresp), .m_rlast(m0_rlast),
        .m_rvalid(m0_rvalid), .m_rready(m0_rready)
    );

    // ---------------- Master 1 <-> crossbar wires ----------------
    wire [LID_WIDTH-1:0]  m1_awid;   wire [ADDR_WIDTH-1:0] m1_awaddr; wire [7:0] m1_awlen;
    wire [2:0] m1_awsize; wire [1:0] m1_awburst; wire m1_awvalid, m1_awready;
    wire [DATA_WIDTH-1:0] m1_wdata;  wire [STRB_WIDTH-1:0] m1_wstrb; wire m1_wlast, m1_wvalid, m1_wready;
    wire [LID_WIDTH-1:0]  m1_bid;    wire [1:0] m1_bresp; wire m1_bvalid, m1_bready;
    wire [LID_WIDTH-1:0]  m1_arid;   wire [ADDR_WIDTH-1:0] m1_araddr; wire [7:0] m1_arlen;
    wire [2:0] m1_arsize; wire [1:0] m1_arburst; wire m1_arvalid, m1_arready;
    wire [LID_WIDTH-1:0]  m1_rid;    wire [DATA_WIDTH-1:0] m1_rdata; wire [1:0] m1_rresp;
    wire m1_rlast, m1_rvalid, m1_rready;

    axi4_master #(
        .ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH), .ID_WIDTH(LID_WIDTH)
    ) u_master1 (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(m1_cmd_valid), .cmd_ready(m1_cmd_ready), .cmd_write(m1_cmd_write),
        .cmd_id(m1_cmd_id), .cmd_addr(m1_cmd_addr), .cmd_len(m1_cmd_len), .cmd_burst(m1_cmd_burst),
        .cmd_wstrb(m1_cmd_wstrb), .cmd_wseed(m1_cmd_wseed),
        .rsp_b_valid(m1_rsp_b_valid), .rsp_b_id(m1_rsp_b_id), .rsp_b_resp(m1_rsp_b_resp),
        .rsp_r_valid(m1_rsp_r_valid), .rsp_r_id(m1_rsp_r_id), .rsp_r_resp(m1_rsp_r_resp),
        .rsp_r_data(m1_rsp_r_data), .rsp_r_last(m1_rsp_r_last),
        .m_awid(m1_awid), .m_awaddr(m1_awaddr), .m_awlen(m1_awlen), .m_awsize(m1_awsize),
        .m_awburst(m1_awburst), .m_awvalid(m1_awvalid), .m_awready(m1_awready),
        .m_wdata(m1_wdata), .m_wstrb(m1_wstrb), .m_wlast(m1_wlast), .m_wvalid(m1_wvalid), .m_wready(m1_wready),
        .m_bid(m1_bid), .m_bresp(m1_bresp), .m_bvalid(m1_bvalid), .m_bready(m1_bready),
        .m_arid(m1_arid), .m_araddr(m1_araddr), .m_arlen(m1_arlen), .m_arsize(m1_arsize),
        .m_arburst(m1_arburst), .m_arvalid(m1_arvalid), .m_arready(m1_arready),
        .m_rid(m1_rid), .m_rdata(m1_rdata), .m_rresp(m1_rresp), .m_rlast(m1_rlast),
        .m_rvalid(m1_rvalid), .m_rready(m1_rready)
    );

    // ---------------- crossbar <-> Slave 0 wires ----------------
    wire [ID_WIDTH-1:0]   s0_awid;   wire [ADDR_WIDTH-1:0] s0_awaddr; wire [7:0] s0_awlen;
    wire [2:0] s0_awsize; wire [1:0] s0_awburst; wire s0_awvalid, s0_awready;
    wire [DATA_WIDTH-1:0] s0_wdata;  wire [STRB_WIDTH-1:0] s0_wstrb; wire s0_wlast, s0_wvalid, s0_wready;
    wire [ID_WIDTH-1:0]   s0_bid;    wire [1:0] s0_bresp; wire s0_bvalid, s0_bready;
    wire [ID_WIDTH-1:0]   s0_arid;   wire [ADDR_WIDTH-1:0] s0_araddr; wire [7:0] s0_arlen;
    wire [2:0] s0_arsize; wire [1:0] s0_arburst; wire s0_arvalid, s0_arready;
    wire [ID_WIDTH-1:0]   s0_rid;    wire [DATA_WIDTH-1:0] s0_rdata; wire [1:0] s0_rresp;
    wire s0_rlast, s0_rvalid, s0_rready;

    // ---------------- crossbar <-> Slave 1 wires (clk domain side, "a") ----
    wire [ID_WIDTH-1:0]   s1a_awid;   wire [ADDR_WIDTH-1:0] s1a_awaddr; wire [7:0] s1a_awlen;
    wire [2:0] s1a_awsize; wire [1:0] s1a_awburst; wire s1a_awvalid, s1a_awready;
    wire [DATA_WIDTH-1:0] s1a_wdata;  wire [STRB_WIDTH-1:0] s1a_wstrb; wire s1a_wlast, s1a_wvalid, s1a_wready;
    wire [ID_WIDTH-1:0]   s1a_bid;    wire [1:0] s1a_bresp; wire s1a_bvalid, s1a_bready;
    wire [ID_WIDTH-1:0]   s1a_arid;   wire [ADDR_WIDTH-1:0] s1a_araddr; wire [7:0] s1a_arlen;
    wire [2:0] s1a_arsize; wire [1:0] s1a_arburst; wire s1a_arvalid, s1a_arready;
    wire [ID_WIDTH-1:0]   s1a_rid;    wire [DATA_WIDTH-1:0] s1a_rdata; wire [1:0] s1a_rresp;
    wire s1a_rlast, s1a_rvalid, s1a_rready;

    axi4_crossbar #(
        .ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH), .LID_WIDTH(LID_WIDTH)
    ) u_crossbar (
        .clk(clk), .rst_n(rst_n),

        .mst0_awid(m0_awid), .mst0_awaddr(m0_awaddr), .mst0_awlen(m0_awlen), .mst0_awsize(m0_awsize),
        .mst0_awburst(m0_awburst), .mst0_awvalid(m0_awvalid), .mst0_awready(m0_awready),
        .mst0_wdata(m0_wdata), .mst0_wstrb(m0_wstrb), .mst0_wlast(m0_wlast), .mst0_wvalid(m0_wvalid), .mst0_wready(m0_wready),
        .mst0_bid(m0_bid), .mst0_bresp(m0_bresp), .mst0_bvalid(m0_bvalid), .mst0_bready(m0_bready),
        .mst0_arid(m0_arid), .mst0_araddr(m0_araddr), .mst0_arlen(m0_arlen), .mst0_arsize(m0_arsize),
        .mst0_arburst(m0_arburst), .mst0_arvalid(m0_arvalid), .mst0_arready(m0_arready),
        .mst0_rid(m0_rid), .mst0_rdata(m0_rdata), .mst0_rresp(m0_rresp), .mst0_rlast(m0_rlast),
        .mst0_rvalid(m0_rvalid), .mst0_rready(m0_rready),

        .mst1_awid(m1_awid), .mst1_awaddr(m1_awaddr), .mst1_awlen(m1_awlen), .mst1_awsize(m1_awsize),
        .mst1_awburst(m1_awburst), .mst1_awvalid(m1_awvalid), .mst1_awready(m1_awready),
        .mst1_wdata(m1_wdata), .mst1_wstrb(m1_wstrb), .mst1_wlast(m1_wlast), .mst1_wvalid(m1_wvalid), .mst1_wready(m1_wready),
        .mst1_bid(m1_bid), .mst1_bresp(m1_bresp), .mst1_bvalid(m1_bvalid), .mst1_bready(m1_bready),
        .mst1_arid(m1_arid), .mst1_araddr(m1_araddr), .mst1_arlen(m1_arlen), .mst1_arsize(m1_arsize),
        .mst1_arburst(m1_arburst), .mst1_arvalid(m1_arvalid), .mst1_arready(m1_arready),
        .mst1_rid(m1_rid), .mst1_rdata(m1_rdata), .mst1_rresp(m1_rresp), .mst1_rlast(m1_rlast),
        .mst1_rvalid(m1_rvalid), .mst1_rready(m1_rready),

        .slv0_awid(s0_awid), .slv0_awaddr(s0_awaddr), .slv0_awlen(s0_awlen), .slv0_awsize(s0_awsize),
        .slv0_awburst(s0_awburst), .slv0_awvalid(s0_awvalid), .slv0_awready(s0_awready),
        .slv0_wdata(s0_wdata), .slv0_wstrb(s0_wstrb), .slv0_wlast(s0_wlast), .slv0_wvalid(s0_wvalid), .slv0_wready(s0_wready),
        .slv0_bid(s0_bid), .slv0_bresp(s0_bresp), .slv0_bvalid(s0_bvalid), .slv0_bready(s0_bready),
        .slv0_arid(s0_arid), .slv0_araddr(s0_araddr), .slv0_arlen(s0_arlen), .slv0_arsize(s0_arsize),
        .slv0_arburst(s0_arburst), .slv0_arvalid(s0_arvalid), .slv0_arready(s0_arready),
        .slv0_rid(s0_rid), .slv0_rdata(s0_rdata), .slv0_rresp(s0_rresp), .slv0_rlast(s0_rlast),
        .slv0_rvalid(s0_rvalid), .slv0_rready(s0_rready),

        .slv1_awid(s1a_awid), .slv1_awaddr(s1a_awaddr), .slv1_awlen(s1a_awlen), .slv1_awsize(s1a_awsize),
        .slv1_awburst(s1a_awburst), .slv1_awvalid(s1a_awvalid), .slv1_awready(s1a_awready),
        .slv1_wdata(s1a_wdata), .slv1_wstrb(s1a_wstrb), .slv1_wlast(s1a_wlast), .slv1_wvalid(s1a_wvalid), .slv1_wready(s1a_wready),
        .slv1_bid(s1a_bid), .slv1_bresp(s1a_bresp), .slv1_bvalid(s1a_bvalid), .slv1_bready(s1a_bready),
        .slv1_arid(s1a_arid), .slv1_araddr(s1a_araddr), .slv1_arlen(s1a_arlen), .slv1_arsize(s1a_arsize),
        .slv1_arburst(s1a_arburst), .slv1_arvalid(s1a_arvalid), .slv1_arready(s1a_arready),
        .slv1_rid(s1a_rid), .slv1_rdata(s1a_rdata), .slv1_rresp(s1a_rresp), .slv1_rlast(s1a_rlast),
        .slv1_rvalid(s1a_rvalid), .slv1_rready(s1a_rready)
    );

    axi4_slave #(
        .ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH), .ID_WIDTH(ID_WIDTH)
    ) u_slave0 (
        .clk(clk), .rst_n(rst_n),
        .s_awid(s0_awid), .s_awaddr(s0_awaddr), .s_awlen(s0_awlen), .s_awsize(s0_awsize),
        .s_awburst(s0_awburst), .s_awvalid(s0_awvalid), .s_awready(s0_awready),
        .s_wdata(s0_wdata), .s_wstrb(s0_wstrb), .s_wlast(s0_wlast), .s_wvalid(s0_wvalid), .s_wready(s0_wready),
        .s_bid(s0_bid), .s_bresp(s0_bresp), .s_bvalid(s0_bvalid), .s_bready(s0_bready),
        .s_arid(s0_arid), .s_araddr(s0_araddr), .s_arlen(s0_arlen), .s_arsize(s0_arsize),
        .s_arburst(s0_arburst), .s_arvalid(s0_arvalid), .s_arready(s0_arready),
        .s_rid(s0_rid), .s_rdata(s0_rdata), .s_rresp(s0_rresp), .s_rlast(s0_rlast),
        .s_rvalid(s0_rvalid), .s_rready(s0_rready)
    );

    // ---------------- CDC bridge <-> Slave 1 wires (clk_s1 domain side, "b") ----
    wire [ID_WIDTH-1:0]   s1b_awid;   wire [ADDR_WIDTH-1:0] s1b_awaddr; wire [7:0] s1b_awlen;
    wire [2:0] s1b_awsize; wire [1:0] s1b_awburst; wire s1b_awvalid, s1b_awready;
    wire [DATA_WIDTH-1:0] s1b_wdata;  wire [STRB_WIDTH-1:0] s1b_wstrb; wire s1b_wlast, s1b_wvalid, s1b_wready;
    wire [ID_WIDTH-1:0]   s1b_bid;    wire [1:0] s1b_bresp; wire s1b_bvalid, s1b_bready;
    wire [ID_WIDTH-1:0]   s1b_arid;   wire [ADDR_WIDTH-1:0] s1b_araddr; wire [7:0] s1b_arlen;
    wire [2:0] s1b_arsize; wire [1:0] s1b_arburst; wire s1b_arvalid, s1b_arready;
    wire [ID_WIDTH-1:0]   s1b_rid;    wire [DATA_WIDTH-1:0] s1b_rdata; wire [1:0] s1b_rresp;
    wire s1b_rlast, s1b_rvalid, s1b_rready;

    axi4_cdc_bridge #(
        .ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH), .ID_WIDTH(ID_WIDTH)
    ) u_cdc_bridge (
        .clk_a(clk), .rst_a_n(rst_n),
        .a_awid(s1a_awid), .a_awaddr(s1a_awaddr), .a_awlen(s1a_awlen), .a_awsize(s1a_awsize),
        .a_awburst(s1a_awburst), .a_awvalid(s1a_awvalid), .a_awready(s1a_awready),
        .a_wdata(s1a_wdata), .a_wstrb(s1a_wstrb), .a_wlast(s1a_wlast), .a_wvalid(s1a_wvalid), .a_wready(s1a_wready),
        .a_bid(s1a_bid), .a_bresp(s1a_bresp), .a_bvalid(s1a_bvalid), .a_bready(s1a_bready),
        .a_arid(s1a_arid), .a_araddr(s1a_araddr), .a_arlen(s1a_arlen), .a_arsize(s1a_arsize),
        .a_arburst(s1a_arburst), .a_arvalid(s1a_arvalid), .a_arready(s1a_arready),
        .a_rid(s1a_rid), .a_rdata(s1a_rdata), .a_rresp(s1a_rresp), .a_rlast(s1a_rlast),
        .a_rvalid(s1a_rvalid), .a_rready(s1a_rready),

        .clk_b(clk_s1), .rst_b_n(rst_n),
        .b_awid(s1b_awid), .b_awaddr(s1b_awaddr), .b_awlen(s1b_awlen), .b_awsize(s1b_awsize),
        .b_awburst(s1b_awburst), .b_awvalid(s1b_awvalid), .b_awready(s1b_awready),
        .b_wdata(s1b_wdata), .b_wstrb(s1b_wstrb), .b_wlast(s1b_wlast), .b_wvalid(s1b_wvalid), .b_wready(s1b_wready),
        .b_bid(s1b_bid), .b_bresp(s1b_bresp), .b_bvalid(s1b_bvalid), .b_bready(s1b_bready),
        .b_arid(s1b_arid), .b_araddr(s1b_araddr), .b_arlen(s1b_arlen), .b_arsize(s1b_arsize),
        .b_arburst(s1b_arburst), .b_arvalid(s1b_arvalid), .b_arready(s1b_arready),
        .b_rid(s1b_rid), .b_rdata(s1b_rdata), .b_rresp(s1b_rresp), .b_rlast(s1b_rlast),
        .b_rvalid(s1b_rvalid), .b_rready(s1b_rready)
    );

    axi4_slave #(
        .ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH), .ID_WIDTH(ID_WIDTH)
    ) u_slave1 (
        .clk(clk_s1), .rst_n(rst_n),
        .s_awid(s1b_awid), .s_awaddr(s1b_awaddr), .s_awlen(s1b_awlen), .s_awsize(s1b_awsize),
        .s_awburst(s1b_awburst), .s_awvalid(s1b_awvalid), .s_awready(s1b_awready),
        .s_wdata(s1b_wdata), .s_wstrb(s1b_wstrb), .s_wlast(s1b_wlast), .s_wvalid(s1b_wvalid), .s_wready(s1b_wready),
        .s_bid(s1b_bid), .s_bresp(s1b_bresp), .s_bvalid(s1b_bvalid), .s_bready(s1b_bready),
        .s_arid(s1b_arid), .s_araddr(s1b_araddr), .s_arlen(s1b_arlen), .s_arsize(s1b_arsize),
        .s_arburst(s1b_arburst), .s_arvalid(s1b_arvalid), .s_arready(s1b_arready),
        .s_rid(s1b_rid), .s_rdata(s1b_rdata), .s_rresp(s1b_rresp), .s_rlast(s1b_rlast),
        .s_rvalid(s1b_rvalid), .s_rready(s1b_rready)
    );

endmodule
