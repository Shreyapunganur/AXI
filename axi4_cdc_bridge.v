`timescale 1ns/1ps
// =====================================================================
// axi4_cdc_bridge.v
// -----------------------------------------------------------------------
// Sits between the crossbar (clk_a domain) and Slave 1 (clk_b domain).
// From the crossbar's side, this looks exactly like a normal AXI4
// slave. From the slave's side, it looks exactly like a normal AXI4
// master. In between, each of the 5 AXI channels gets its own
// async_fifo (see async_fifo.v) to cross safely from one clock to the
// other - request channels (AW, W, AR) flow clk_a -> clk_b, response
// channels (B, R) flow the other way, clk_b -> clk_a.
//
// Each channel's handful of signals is packed into one wide bus before
// going into its FIFO, then unpacked on the way out - that's all the
// "pack"/"unpack" wires below are doing.
// =====================================================================
module axi4_cdc_bridge #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter ID_WIDTH   = 3,
    parameter STRB_WIDTH = DATA_WIDTH/8
)(
    // ---- crossbar-side (clk_a domain) - looks like an AXI4 SLAVE port ----
    input  wire                   clk_a,
    input  wire                   rst_a_n,
    input  wire [ID_WIDTH-1:0]    a_awid,
    input  wire [ADDR_WIDTH-1:0]  a_awaddr,
    input  wire [7:0]             a_awlen,
    input  wire [2:0]             a_awsize,
    input  wire [1:0]             a_awburst,
    input  wire                   a_awvalid,
    output wire                   a_awready,

    input  wire [DATA_WIDTH-1:0]  a_wdata,
    input  wire [STRB_WIDTH-1:0]  a_wstrb,
    input  wire                   a_wlast,
    input  wire                   a_wvalid,
    output wire                   a_wready,

    output wire [ID_WIDTH-1:0]    a_bid,
    output wire [1:0]             a_bresp,
    output wire                   a_bvalid,
    input  wire                   a_bready,

    input  wire [ID_WIDTH-1:0]    a_arid,
    input  wire [ADDR_WIDTH-1:0]  a_araddr,
    input  wire [7:0]             a_arlen,
    input  wire [2:0]             a_arsize,
    input  wire [1:0]             a_arburst,
    input  wire                   a_arvalid,
    output wire                   a_arready,

    output wire [ID_WIDTH-1:0]    a_rid,
    output wire [DATA_WIDTH-1:0]  a_rdata,
    output wire [1:0]             a_rresp,
    output wire                   a_rlast,
    output wire                   a_rvalid,
    input  wire                   a_rready,

    // ---- slave-side (clk_b domain) - looks like an AXI4 MASTER port -------
    input  wire                   clk_b,
    input  wire                   rst_b_n,
    output wire [ID_WIDTH-1:0]    b_awid,
    output wire [ADDR_WIDTH-1:0]  b_awaddr,
    output wire [7:0]             b_awlen,
    output wire [2:0]             b_awsize,
    output wire [1:0]             b_awburst,
    output wire                   b_awvalid,
    input  wire                   b_awready,

    output wire [DATA_WIDTH-1:0]  b_wdata,
    output wire [STRB_WIDTH-1:0]  b_wstrb,
    output wire                   b_wlast,
    output wire                   b_wvalid,
    input  wire                   b_wready,

    input  wire [ID_WIDTH-1:0]    b_bid,
    input  wire [1:0]             b_bresp,
    input  wire                   b_bvalid,
    output wire                   b_bready,

    output wire [ID_WIDTH-1:0]    b_arid,
    output wire [ADDR_WIDTH-1:0]  b_araddr,
    output wire [7:0]             b_arlen,
    output wire [2:0]             b_arsize,
    output wire [1:0]             b_arburst,
    output wire                   b_arvalid,
    input  wire                   b_arready,

    input  wire [ID_WIDTH-1:0]    b_rid,
    input  wire [DATA_WIDTH-1:0]  b_rdata,
    input  wire [1:0]             b_rresp,
    input  wire                   b_rlast,
    input  wire                   b_rvalid,
    output wire                   b_rready
);
    localparam AW_W = ID_WIDTH + ADDR_WIDTH + 8 + 3 + 2;
    localparam W_W  = DATA_WIDTH + STRB_WIDTH + 1;
    localparam B_W  = ID_WIDTH + 2;
    localparam AR_W = AW_W;
    localparam R_W  = ID_WIDTH + DATA_WIDTH + 2 + 1;

    // ---------------- AW channel: clk_a -> clk_b ----------------
    wire aw_full, aw_empty;
    wire [AW_W-1:0] aw_pack_in = {a_awid, a_awaddr, a_awlen, a_awsize, a_awburst};
    wire [AW_W-1:0] aw_pack_out;
    assign a_awready = !aw_full;
    assign b_awvalid = !aw_empty;
    assign {b_awid, b_awaddr, b_awlen, b_awsize, b_awburst} = aw_pack_out;

    async_fifo #(.WIDTH(AW_W), .DEPTH(8)) u_aw_fifo (
        .wr_clk(clk_a), .wr_rst_n(rst_a_n), .wr_en(a_awvalid && !aw_full), .wr_data(aw_pack_in), .full(aw_full),
        .rd_clk(clk_b), .rd_rst_n(rst_b_n), .rd_en(b_awvalid && b_awready), .rd_data(aw_pack_out), .empty(aw_empty)
    );

    // ---------------- W channel: clk_a -> clk_b ----------------
    wire w_full, w_empty;
    wire [W_W-1:0] w_pack_in = {a_wdata, a_wstrb, a_wlast};
    wire [W_W-1:0] w_pack_out;
    assign a_wready = !w_full;
    assign b_wvalid = !w_empty;
    assign {b_wdata, b_wstrb, b_wlast} = w_pack_out;

    async_fifo #(.WIDTH(W_W), .DEPTH(8)) u_w_fifo (
        .wr_clk(clk_a), .wr_rst_n(rst_a_n), .wr_en(a_wvalid && !w_full), .wr_data(w_pack_in), .full(w_full),
        .rd_clk(clk_b), .rd_rst_n(rst_b_n), .rd_en(b_wvalid && b_wready), .rd_data(w_pack_out), .empty(w_empty)
    );

    // ---------------- B channel: clk_b -> clk_a (response, reverse dir) ----------------
    wire b_full, b_empty;
    wire [B_W-1:0] b_pack_in = {b_bid, b_bresp};
    wire [B_W-1:0] b_pack_out;
    assign b_bready = !b_full;
    assign a_bvalid = !b_empty;
    assign {a_bid, a_bresp} = b_pack_out;

    async_fifo #(.WIDTH(B_W), .DEPTH(8)) u_b_fifo (
        .wr_clk(clk_b), .wr_rst_n(rst_b_n), .wr_en(b_bvalid && !b_full), .wr_data(b_pack_in), .full(b_full),
        .rd_clk(clk_a), .rd_rst_n(rst_a_n), .rd_en(a_bvalid && a_bready), .rd_data(b_pack_out), .empty(b_empty)
    );

    // ---------------- AR channel: clk_a -> clk_b ----------------
    wire ar_full, ar_empty;
    wire [AR_W-1:0] ar_pack_in = {a_arid, a_araddr, a_arlen, a_arsize, a_arburst};
    wire [AR_W-1:0] ar_pack_out;
    assign a_arready = !ar_full;
    assign b_arvalid = !ar_empty;
    assign {b_arid, b_araddr, b_arlen, b_arsize, b_arburst} = ar_pack_out;

    async_fifo #(.WIDTH(AR_W), .DEPTH(8)) u_ar_fifo (
        .wr_clk(clk_a), .wr_rst_n(rst_a_n), .wr_en(a_arvalid && !ar_full), .wr_data(ar_pack_in), .full(ar_full),
        .rd_clk(clk_b), .rd_rst_n(rst_b_n), .rd_en(b_arvalid && b_arready), .rd_data(ar_pack_out), .empty(ar_empty)
    );

    // ---------------- R channel: clk_b -> clk_a (response, reverse dir) ----------------
    wire r_full, r_empty;
    wire [R_W-1:0] r_pack_in = {b_rid, b_rdata, b_rresp, b_rlast};
    wire [R_W-1:0] r_pack_out;
    assign b_rready = !r_full;
    assign a_rvalid = !r_empty;
    assign {a_rid, a_rdata, a_rresp, a_rlast} = r_pack_out;

    async_fifo #(.WIDTH(R_W), .DEPTH(8)) u_r_fifo (
        .wr_clk(clk_b), .wr_rst_n(rst_b_n), .wr_en(b_rvalid && !r_full), .wr_data(r_pack_in), .full(r_full),
        .rd_clk(clk_a), .rd_rst_n(rst_a_n), .rd_en(a_rvalid && a_rready), .rd_data(r_pack_out), .empty(r_empty)
    );

endmodule
