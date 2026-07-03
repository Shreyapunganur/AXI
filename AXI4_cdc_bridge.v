`timescale 1ns/1ps
// =============================================================
// AXI4_cdc_bridge.v  —  AXI4 Clock-Domain Crossing Bridge
//
// Topology:
//   CLK_A (fast, interconnect side) ──► 5x Async FIFOs ──► CLK_B (slave side)
//
// One async FIFO per channel:
//   AW : CLK_A ──► CLK_B  (write address)
//   W  : CLK_A ──► CLK_B  (write data)
//   B  : CLK_B ──► CLK_A  (write response)
//   AR : CLK_A ──► CLK_B  (read address)
//   R  : CLK_B ──► CLK_A  (read data)
//
// Handshake preservation:
//   ia_*ready = !<channel>_fifo_full   (backpressure to upstream)
//   ob_*valid = !<channel>_fifo_empty  (data available downstream)
//   Push on: valid && !full
//   Pop  on: valid && ready (downstream handshake)
//
// Parameters:
//   ADWIDTH, DWIDTH, IDWIDTH — AXI bus widths
//   FIFO_DEPTH               — entries per FIFO (power of 2, >= 4)
// =============================================================

module AXI4_cdc_bridge #(
    parameter ADWIDTH    = 32,
    parameter DWIDTH     = 64,
    parameter IDWIDTH    = 4,
    parameter FIFO_DEPTH = 8
)(
    // =========== CLK_A side (interconnect-facing AXI slave) ===========
    input  wire               clk_a,
    input  wire               rst_a_n,

    // AW — input from interconnect
    input  wire [IDWIDTH-1:0] ia_awid,
    input  wire [ADWIDTH-1:0] ia_awaddr,
    input  wire [7:0]         ia_awlen,
    input  wire [2:0]         ia_awsize,
    input  wire [1:0]         ia_awburst,
    input  wire [3:0]         ia_awqos,
    input  wire               ia_awvalid,
    output wire               ia_awready,

    // W — input from interconnect
    input  wire [DWIDTH-1:0]  ia_wdata,
    input  wire [DWIDTH/8-1:0]ia_wstrb,
    input  wire               ia_wlast,
    input  wire               ia_wvalid,
    output wire               ia_wready,

    // B — output to interconnect
    output wire [IDWIDTH-1:0] ia_bid,
    output wire [1:0]         ia_bresp,
    output wire               ia_bvalid,
    input  wire               ia_bready,

    // AR — input from interconnect
    input  wire [IDWIDTH-1:0] ia_arid,
    input  wire [ADWIDTH-1:0] ia_araddr,
    input  wire [7:0]         ia_arlen,
    input  wire [2:0]         ia_arsize,
    input  wire [1:0]         ia_arburst,
    input  wire [3:0]         ia_arqos,
    input  wire               ia_arvalid,
    output wire               ia_arready,

    // R — output to interconnect
    output wire [IDWIDTH-1:0] ia_rid,
    output wire [DWIDTH-1:0]  ia_rdata,
    output wire [1:0]         ia_rresp,
    output wire               ia_rlast,
    output wire               ia_rvalid,
    input  wire               ia_rready,

    // =========== CLK_B side (slave-facing AXI master) =================
    input  wire               clk_b,
    input  wire               rst_b_n,

    // AW — output to slave
    output wire [IDWIDTH-1:0] ob_awid,
    output wire [ADWIDTH-1:0] ob_awaddr,
    output wire [7:0]         ob_awlen,
    output wire [2:0]         ob_awsize,
    output wire [1:0]         ob_awburst,
    output wire [3:0]         ob_awqos,
    output wire               ob_awvalid,
    input  wire               ob_awready,

    // W — output to slave
    output wire [DWIDTH-1:0]  ob_wdata,
    output wire [DWIDTH/8-1:0]ob_wstrb,
    output wire               ob_wlast,
    output wire               ob_wvalid,
    input  wire               ob_wready,

    // B — input from slave
    input  wire [IDWIDTH-1:0] ob_bid,
    input  wire [1:0]         ob_bresp,
    input  wire               ob_bvalid,
    output wire               ob_bready,

    // AR — output to slave
    output wire [IDWIDTH-1:0] ob_arid,
    output wire [ADWIDTH-1:0] ob_araddr,
    output wire [7:0]         ob_arlen,
    output wire [2:0]         ob_arsize,
    output wire [1:0]         ob_arburst,
    output wire [3:0]         ob_arqos,
    output wire               ob_arvalid,
    input  wire               ob_arready,

    // R — input from slave
    input  wire [IDWIDTH-1:0] ob_rid,
    input  wire [DWIDTH-1:0]  ob_rdata,
    input  wire [1:0]         ob_rresp,
    input  wire               ob_rlast,
    input  wire               ob_rvalid,
    output wire               ob_rready,

    // Debug / assertion hooks
    output wire               aw_fifo_full_o,
    output wire               r_fifo_full_o
);

    // ----------------------------------------------------------
    // FIFO widths
    // ----------------------------------------------------------
    localparam AW_W = IDWIDTH + ADWIDTH + 8 + 3 + 2 + 4; // id+addr+len+size+burst+qos
    localparam W_W  = DWIDTH + DWIDTH/8 + 1;              // data+strb+last
    localparam B_W  = IDWIDTH + 2;                         // id+resp
    localparam AR_W = AW_W;
    localparam R_W  = IDWIDTH + DWIDTH + 2 + 1;            // id+data+resp+last

    // ----------------------------------------------------------
    // FIFO full/empty wires
    // ----------------------------------------------------------
    wire aw_full, aw_empty;
    wire w_full,  w_empty;
    wire b_full,  b_empty;
    wire ar_full, ar_empty;
    wire r_full,  r_empty;

    assign aw_fifo_full_o = aw_full;
    assign r_fifo_full_o  = r_full;

    // ----------------------------------------------------------
    // AW FIFO  (CLK_A → CLK_B)
    // ----------------------------------------------------------
    wire [AW_W-1:0] aw_wdata = {ia_awid, ia_awaddr, ia_awlen,
                                 ia_awsize, ia_awburst, ia_awqos};
    wire [AW_W-1:0] aw_rdata;

    assign ia_awready  = !aw_full;
    wire   aw_push     = ia_awvalid && !aw_full;
    assign ob_awvalid  = !aw_empty;
    wire   aw_pop      = ob_awvalid && ob_awready;

    assign {ob_awid, ob_awaddr, ob_awlen,
            ob_awsize, ob_awburst, ob_awqos} = aw_rdata;

    AXI4_async_fifo #(.WIDTH(AW_W), .DEPTH(FIFO_DEPTH)) u_aw_fifo (
        .wclk(clk_a), .wrst_n(rst_a_n),
        .push(aw_push), .wdata(aw_wdata), .full(aw_full), .almost_full(),
        .rclk(clk_b),  .rrst_n(rst_b_n),
        .pop(aw_pop),  .rdata(aw_rdata),  .empty(aw_empty), .almost_empty()
    );

    // ----------------------------------------------------------
    // W FIFO  (CLK_A → CLK_B)
    // ----------------------------------------------------------
    wire [W_W-1:0] w_wdata = {ia_wdata, ia_wstrb, ia_wlast};
    wire [W_W-1:0] w_rdata;

    assign ia_wready  = !w_full;
    wire   w_push     = ia_wvalid && !w_full;
    assign ob_wvalid  = !w_empty;
    wire   w_pop      = ob_wvalid && ob_wready;

    assign {ob_wdata, ob_wstrb, ob_wlast} = w_rdata;

    AXI4_async_fifo #(.WIDTH(W_W), .DEPTH(FIFO_DEPTH)) u_w_fifo (
        .wclk(clk_a), .wrst_n(rst_a_n),
        .push(w_push), .wdata(w_wdata), .full(w_full), .almost_full(),
        .rclk(clk_b),  .rrst_n(rst_b_n),
        .pop(w_pop),   .rdata(w_rdata),  .empty(w_empty), .almost_empty()
    );

    // ----------------------------------------------------------
    // B FIFO  (CLK_B → CLK_A)
    // ----------------------------------------------------------
    wire [B_W-1:0] b_wdata = {ob_bid, ob_bresp};
    wire [B_W-1:0] b_rdata;

    assign ob_bready  = !b_full;
    wire   b_push     = ob_bvalid && !b_full;
    assign ia_bvalid  = !b_empty;
    wire   b_pop      = ia_bvalid && ia_bready;

    assign {ia_bid, ia_bresp} = b_rdata;

    AXI4_async_fifo #(.WIDTH(B_W), .DEPTH(FIFO_DEPTH)) u_b_fifo (
        .wclk(clk_b), .wrst_n(rst_b_n),
        .push(b_push), .wdata(b_wdata), .full(b_full), .almost_full(),
        .rclk(clk_a),  .rrst_n(rst_a_n),
        .pop(b_pop),   .rdata(b_rdata),  .empty(b_empty), .almost_empty()
    );

    // ----------------------------------------------------------
    // AR FIFO  (CLK_A → CLK_B)
    // ----------------------------------------------------------
    wire [AR_W-1:0] ar_wdata = {ia_arid, ia_araddr, ia_arlen,
                                  ia_arsize, ia_arburst, ia_arqos};
    wire [AR_W-1:0] ar_rdata;

    assign ia_arready  = !ar_full;
    wire   ar_push     = ia_arvalid && !ar_full;
    assign ob_arvalid  = !ar_empty;
    wire   ar_pop      = ob_arvalid && ob_arready;

    assign {ob_arid, ob_araddr, ob_arlen,
            ob_arsize, ob_arburst, ob_arqos} = ar_rdata;

    AXI4_async_fifo #(.WIDTH(AR_W), .DEPTH(FIFO_DEPTH)) u_ar_fifo (
        .wclk(clk_a), .wrst_n(rst_a_n),
        .push(ar_push), .wdata(ar_wdata), .full(ar_full), .almost_full(),
        .rclk(clk_b),   .rrst_n(rst_b_n),
        .pop(ar_pop),   .rdata(ar_rdata),  .empty(ar_empty), .almost_empty()
    );

    // ----------------------------------------------------------
    // R FIFO  (CLK_B → CLK_A)
    // ----------------------------------------------------------
    wire [R_W-1:0] r_wdata = {ob_rid, ob_rdata, ob_rresp, ob_rlast};
    wire [R_W-1:0] r_rdata;

    assign ob_rready  = !r_full;
    wire   r_push     = ob_rvalid && !r_full;
    assign ia_rvalid  = !r_empty;
    wire   r_pop      = ia_rvalid && ia_rready;

    assign {ia_rid, ia_rdata, ia_rresp, ia_rlast} = r_rdata;

    AXI4_async_fifo #(.WIDTH(R_W), .DEPTH(FIFO_DEPTH)) u_r_fifo (
        .wclk(clk_b), .wrst_n(rst_b_n),
        .push(r_push), .wdata(r_wdata), .full(r_full), .almost_full(),
        .rclk(clk_a),  .rrst_n(rst_a_n),
        .pop(r_pop),   .rdata(r_rdata),  .empty(r_empty), .almost_empty()
    );

endmodule
