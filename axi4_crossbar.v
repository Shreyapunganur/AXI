`timescale 1ns/1ps

module axi4_crossbar #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter LID_WIDTH  = 2,              
    parameter ID_WIDTH   = LID_WIDTH + 1,   
    parameter STRB_WIDTH = DATA_WIDTH/8
)(
    input wire clk,
    input wire rst_n,

    
    input  wire [LID_WIDTH-1:0]  mst0_awid,
    input  wire [ADDR_WIDTH-1:0] mst0_awaddr,
    input  wire [7:0]            mst0_awlen,
    input  wire [2:0]            mst0_awsize,
    input  wire [1:0]            mst0_awburst,
    input  wire                  mst0_awvalid,
    output wire                  mst0_awready,

    input  wire [DATA_WIDTH-1:0] mst0_wdata,
    input  wire [STRB_WIDTH-1:0] mst0_wstrb,
    input  wire                  mst0_wlast,
    input  wire                  mst0_wvalid,
    output wire                  mst0_wready,

    output wire [LID_WIDTH-1:0]  mst0_bid,
    output wire [1:0]            mst0_bresp,
    output wire                  mst0_bvalid,
    input  wire                  mst0_bready,

    input  wire [LID_WIDTH-1:0]  mst0_arid,
    input  wire [ADDR_WIDTH-1:0] mst0_araddr,
    input  wire [7:0]            mst0_arlen,
    input  wire [2:0]            mst0_arsize,
    input  wire [1:0]            mst0_arburst,
    input  wire                  mst0_arvalid,
    output wire                  mst0_arready,

    output wire [LID_WIDTH-1:0]  mst0_rid,
    output wire [DATA_WIDTH-1:0] mst0_rdata,
    output wire [1:0]            mst0_rresp,
    output wire                  mst0_rlast,
    output wire                  mst0_rvalid,
    input  wire                  mst0_rready,

   
    input  wire [LID_WIDTH-1:0]  mst1_awid,
    input  wire [ADDR_WIDTH-1:0] mst1_awaddr,
    input  wire [7:0]            mst1_awlen,
    input  wire [2:0]            mst1_awsize,
    input  wire [1:0]            mst1_awburst,
    input  wire                  mst1_awvalid,
    output wire                  mst1_awready,

    input  wire [DATA_WIDTH-1:0] mst1_wdata,
    input  wire [STRB_WIDTH-1:0] mst1_wstrb,
    input  wire                  mst1_wlast,
    input  wire                  mst1_wvalid,
    output wire                  mst1_wready,

    output wire [LID_WIDTH-1:0]  mst1_bid,
    output wire [1:0]            mst1_bresp,
    output wire                  mst1_bvalid,
    input  wire                  mst1_bready,

    input  wire [LID_WIDTH-1:0]  mst1_arid,
    input  wire [ADDR_WIDTH-1:0] mst1_araddr,
    input  wire [7:0]            mst1_arlen,
    input  wire [2:0]            mst1_arsize,
    input  wire [1:0]            mst1_arburst,
    input  wire                  mst1_arvalid,
    output wire                  mst1_arready,

    output wire [LID_WIDTH-1:0]  mst1_rid,
    output wire [DATA_WIDTH-1:0] mst1_rdata,
    output wire [1:0]            mst1_rresp,
    output wire                  mst1_rlast,
    output wire                  mst1_rvalid,
    input  wire                  mst1_rready,

   
    output wire [ID_WIDTH-1:0]   slv0_awid,
    output wire [ADDR_WIDTH-1:0] slv0_awaddr,
    output wire [7:0]            slv0_awlen,
    output wire [2:0]            slv0_awsize,
    output wire [1:0]            slv0_awburst,
    output wire                  slv0_awvalid,
    input  wire                  slv0_awready,

    output wire [DATA_WIDTH-1:0] slv0_wdata,
    output wire [STRB_WIDTH-1:0] slv0_wstrb,
    output wire                  slv0_wlast,
    output wire                  slv0_wvalid,
    input  wire                  slv0_wready,

    input  wire [ID_WIDTH-1:0]   slv0_bid,
    input  wire [1:0]            slv0_bresp,
    input  wire                  slv0_bvalid,
    output wire                  slv0_bready,

    output wire [ID_WIDTH-1:0]   slv0_arid,
    output wire [ADDR_WIDTH-1:0] slv0_araddr,
    output wire [7:0]            slv0_arlen,
    output wire [2:0]            slv0_arsize,
    output wire [1:0]            slv0_arburst,
    output wire                  slv0_arvalid,
    input  wire                  slv0_arready,

    input  wire [ID_WIDTH-1:0]   slv0_rid,
    input  wire [DATA_WIDTH-1:0] slv0_rdata,
    input  wire [1:0]            slv0_rresp,
    input  wire                  slv0_rlast,
    input  wire                  slv0_rvalid,
    output wire                  slv0_rready,

   
    output wire [ID_WIDTH-1:0]   slv1_awid,
    output wire [ADDR_WIDTH-1:0] slv1_awaddr,
    output wire [7:0]            slv1_awlen,
    output wire [2:0]            slv1_awsize,
    output wire [1:0]            slv1_awburst,
    output wire                  slv1_awvalid,
    input  wire                  slv1_awready,

    output wire [DATA_WIDTH-1:0] slv1_wdata,
    output wire [STRB_WIDTH-1:0] slv1_wstrb,
    output wire                  slv1_wlast,
    output wire                  slv1_wvalid,
    input  wire                  slv1_wready,

    input  wire [ID_WIDTH-1:0]   slv1_bid,
    input  wire [1:0]            slv1_bresp,
    input  wire                  slv1_bvalid,
    output wire                  slv1_bready,

    output wire [ID_WIDTH-1:0]   slv1_arid,
    output wire [ADDR_WIDTH-1:0] slv1_araddr,
    output wire [7:0]            slv1_arlen,
    output wire [2:0]            slv1_arsize,
    output wire [1:0]            slv1_arburst,
    output wire                  slv1_arvalid,
    input  wire                  slv1_arready,

    input  wire [ID_WIDTH-1:0]   slv1_rid,
    input  wire [DATA_WIDTH-1:0] slv1_rdata,
    input  wire [1:0]            slv1_rresp,
    input  wire                  slv1_rlast,
    input  wire                  slv1_rvalid,
    output wire                  slv1_rready
);


    wire mst0_aw_to_s1 = mst0_awaddr[16];
    wire mst1_aw_to_s1 = mst1_awaddr[16];
    wire mst0_ar_to_s1 = mst0_araddr[16];
    wire mst1_ar_to_s1 = mst1_araddr[16];

   
    localparam OWNER_NONE = 2'd0, OWNER_M0 = 2'd1, OWNER_M1 = 2'd2;

    reg [1:0] wr_owner0;
    reg       wr_rr0;        

    wire m0_wants_wr_s0 = mst0_awvalid && !mst0_aw_to_s1;
    wire m1_wants_wr_s0 = mst1_awvalid && !mst1_aw_to_s1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_owner0 <= OWNER_NONE;
            wr_rr0    <= 1'b0;
        end else if (wr_owner0 == OWNER_NONE) begin
            if (m0_wants_wr_s0 && m1_wants_wr_s0) begin
              
                wr_owner0 <= wr_rr0 ? OWNER_M0 : OWNER_M1;
                wr_rr0    <= !wr_rr0;
            end else if (m0_wants_wr_s0) begin
                wr_owner0 <= OWNER_M0;
            end else if (m1_wants_wr_s0) begin
                wr_owner0 <= OWNER_M1;
            end
        end else if (wr_owner0 == OWNER_M0) begin
            if (mst0_wvalid && slv0_wready && mst0_wlast)
                wr_owner0 <= OWNER_NONE;
        end else begin 
            if (mst1_wvalid && slv0_wready && mst1_wlast)
                wr_owner0 <= OWNER_NONE;
        end
    end

    assign slv0_awvalid = (wr_owner0 == OWNER_M0) ? mst0_awvalid :
                           (wr_owner0 == OWNER_M1) ? mst1_awvalid : 1'b0;
    assign slv0_awid    = (wr_owner0 == OWNER_M1) ? {1'b1, mst1_awid} : {1'b0, mst0_awid};
    assign slv0_awaddr  = (wr_owner0 == OWNER_M1) ? mst1_awaddr  : mst0_awaddr;
    assign slv0_awlen   = (wr_owner0 == OWNER_M1) ? mst1_awlen   : mst0_awlen;
    assign slv0_awsize  = (wr_owner0 == OWNER_M1) ? mst1_awsize  : mst0_awsize;
    assign slv0_awburst = (wr_owner0 == OWNER_M1) ? mst1_awburst : mst0_awburst;

    assign slv0_wvalid  = (wr_owner0 == OWNER_M0) ? mst0_wvalid :
                           (wr_owner0 == OWNER_M1) ? mst1_wvalid : 1'b0;
    assign slv0_wdata   = (wr_owner0 == OWNER_M1) ? mst1_wdata  : mst0_wdata;
    assign slv0_wstrb   = (wr_owner0 == OWNER_M1) ? mst1_wstrb  : mst0_wstrb;
    assign slv0_wlast   = (wr_owner0 == OWNER_M1) ? mst1_wlast  : mst0_wlast;

    reg [1:0] wr_owner1;
    reg       wr_rr1;

    wire m0_wants_wr_s1 = mst0_awvalid && mst0_aw_to_s1;
    wire m1_wants_wr_s1 = mst1_awvalid && mst1_aw_to_s1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_owner1 <= OWNER_NONE;
            wr_rr1    <= 1'b0;
        end else if (wr_owner1 == OWNER_NONE) begin
            if (m0_wants_wr_s1 && m1_wants_wr_s1) begin
                wr_owner1 <= wr_rr1 ? OWNER_M0 : OWNER_M1;
                wr_rr1    <= !wr_rr1;
            end else if (m0_wants_wr_s1) begin
                wr_owner1 <= OWNER_M0;
            end else if (m1_wants_wr_s1) begin
                wr_owner1 <= OWNER_M1;
            end
        end else if (wr_owner1 == OWNER_M0) begin
            if (mst0_wvalid && slv1_wready && mst0_wlast)
                wr_owner1 <= OWNER_NONE;
        end else begin // OWNER_M1
            if (mst1_wvalid && slv1_wready && mst1_wlast)
                wr_owner1 <= OWNER_NONE;
        end
    end

    assign slv1_awvalid = (wr_owner1 == OWNER_M0) ? mst0_awvalid :
                           (wr_owner1 == OWNER_M1) ? mst1_awvalid : 1'b0;
    assign slv1_awid    = (wr_owner1 == OWNER_M1) ? {1'b1, mst1_awid} : {1'b0, mst0_awid};
    assign slv1_awaddr  = (wr_owner1 == OWNER_M1) ? mst1_awaddr  : mst0_awaddr;
    assign slv1_awlen   = (wr_owner1 == OWNER_M1) ? mst1_awlen   : mst0_awlen;
    assign slv1_awsize  = (wr_owner1 == OWNER_M1) ? mst1_awsize  : mst0_awsize;
    assign slv1_awburst = (wr_owner1 == OWNER_M1) ? mst1_awburst : mst0_awburst;

    assign slv1_wvalid  = (wr_owner1 == OWNER_M0) ? mst0_wvalid :
                           (wr_owner1 == OWNER_M1) ? mst1_wvalid : 1'b0;
    assign slv1_wdata   = (wr_owner1 == OWNER_M1) ? mst1_wdata  : mst0_wdata;
    assign slv1_wstrb   = (wr_owner1 == OWNER_M1) ? mst1_wstrb  : mst0_wstrb;
    assign slv1_wlast   = (wr_owner1 == OWNER_M1) ? mst1_wlast  : mst0_wlast;

 
    reg ar_rr0, ar_rr1;

    wire m0_wants_ar_s0 = mst0_arvalid && !mst0_ar_to_s1;
    wire m1_wants_ar_s0 = mst1_arvalid && !mst1_ar_to_s1;
    wire ar_s0_grant_m1 = m1_wants_ar_s0 && (!m0_wants_ar_s0 || ar_rr0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            ar_rr0 <= 1'b0;
        else if (m0_wants_ar_s0 && m1_wants_ar_s0 && slv0_arready)
            ar_rr0 <= !ar_s0_grant_m1;
    end

    assign slv0_arvalid = ar_s0_grant_m1 ? mst1_arvalid : (m0_wants_ar_s0 ? mst0_arvalid : 1'b0);
    assign slv0_arid    = ar_s0_grant_m1 ? {1'b1, mst1_arid} : {1'b0, mst0_arid};
    assign slv0_araddr  = ar_s0_grant_m1 ? mst1_araddr  : mst0_araddr;
    assign slv0_arlen   = ar_s0_grant_m1 ? mst1_arlen   : mst0_arlen;
    assign slv0_arsize  = ar_s0_grant_m1 ? mst1_arsize  : mst0_arsize;
    assign slv0_arburst = ar_s0_grant_m1 ? mst1_arburst : mst0_arburst;

    wire m0_wants_ar_s1 = mst0_arvalid && mst0_ar_to_s1;
    wire m1_wants_ar_s1 = mst1_arvalid && mst1_ar_to_s1;
    wire ar_s1_grant_m1 = m1_wants_ar_s1 && (!m0_wants_ar_s1 || ar_rr1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            ar_rr1 <= 1'b0;
        else if (m0_wants_ar_s1 && m1_wants_ar_s1 && slv1_arready)
            ar_rr1 <= !ar_s1_grant_m1;
    end

    assign slv1_arvalid = ar_s1_grant_m1 ? mst1_arvalid : (m0_wants_ar_s1 ? mst0_arvalid : 1'b0);
    assign slv1_arid    = ar_s1_grant_m1 ? {1'b1, mst1_arid} : {1'b0, mst0_arid};
    assign slv1_araddr  = ar_s1_grant_m1 ? mst1_araddr  : mst0_araddr;
    assign slv1_arlen   = ar_s1_grant_m1 ? mst1_arlen   : mst0_arlen;
    assign slv1_arsize  = ar_s1_grant_m1 ? mst1_arsize  : mst0_arsize;
    assign slv1_arburst = ar_s1_grant_m1 ? mst1_arburst : mst0_arburst;

   
    assign mst0_awready = (wr_owner0 == OWNER_M0) ? slv0_awready :
                           (wr_owner1 == OWNER_M0) ? slv1_awready : 1'b0;

    assign mst0_wready  = (wr_owner0 == OWNER_M0) ? slv0_wready :
                           (wr_owner1 == OWNER_M0) ? slv1_wready : 1'b0;

    assign mst0_arready = mst0_ar_to_s1 ? (ar_s1_grant_m1  ? 1'b0 : slv1_arready)
                                          : (ar_s0_grant_m1  ? 1'b0 : slv0_arready);

    assign mst1_awready = (wr_owner0 == OWNER_M1) ? slv0_awready :
                           (wr_owner1 == OWNER_M1) ? slv1_awready : 1'b0;

    assign mst1_wready  = (wr_owner0 == OWNER_M1) ? slv0_wready :
                           (wr_owner1 == OWNER_M1) ? slv1_wready : 1'b0;

    assign mst1_arready = mst1_ar_to_s1 ? (ar_s1_grant_m1 ? slv1_arready : 1'b0)
                                          : (ar_s0_grant_m1 ? slv0_arready : 1'b0);

  
    wire slv0_b_for_m0 = slv0_bvalid && (slv0_bid[ID_WIDTH-1] == 1'b0);
    wire slv0_b_for_m1 = slv0_bvalid && (slv0_bid[ID_WIDTH-1] == 1'b1);
    wire slv1_b_for_m0 = slv1_bvalid && (slv1_bid[ID_WIDTH-1] == 1'b0);
    wire slv1_b_for_m1 = slv1_bvalid && (slv1_bid[ID_WIDTH-1] == 1'b1);

    assign mst0_bvalid = slv0_b_for_m0 | slv1_b_for_m0;
    assign mst0_bid    = slv0_b_for_m0 ? slv0_bid[LID_WIDTH-1:0] : slv1_bid[LID_WIDTH-1:0];
    assign mst0_bresp  = slv0_b_for_m0 ? slv0_bresp : slv1_bresp;

    assign mst1_bvalid = slv0_b_for_m1 | slv1_b_for_m1;
    assign mst1_bid    = slv0_b_for_m1 ? slv0_bid[LID_WIDTH-1:0] : slv1_bid[LID_WIDTH-1:0];
    assign mst1_bresp  = slv0_b_for_m1 ? slv0_bresp : slv1_bresp;

    assign slv0_bready = slv0_b_for_m0 ? mst0_bready :
                          slv0_b_for_m1 ? mst1_bready : 1'b0;

    assign slv1_bready = slv1_b_for_m0 ? (mst0_bready && !slv0_b_for_m0) :
                          slv1_b_for_m1 ? (mst1_bready && !slv0_b_for_m1) : 1'b0;


    localparam RSRC_NONE = 2'd0, RSRC_SLV0 = 2'd1, RSRC_SLV1 = 2'd2;

    wire slv0_r_for_m0 = slv0_rvalid && (slv0_rid[ID_WIDTH-1] == 1'b0);
    wire slv0_r_for_m1 = slv0_rvalid && (slv0_rid[ID_WIDTH-1] == 1'b1);
    wire slv1_r_for_m0 = slv1_rvalid && (slv1_rid[ID_WIDTH-1] == 1'b0);
    wire slv1_r_for_m1 = slv1_rvalid && (slv1_rid[ID_WIDTH-1] == 1'b1);

    reg [1:0] r_owner0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_owner0 <= RSRC_NONE;
        end else if (r_owner0 == RSRC_NONE) begin
            if (slv0_r_for_m0)
                r_owner0 <= RSRC_SLV0;
            else if (slv1_r_for_m0)
                r_owner0 <= RSRC_SLV1;
        end else if (r_owner0 == RSRC_SLV0) begin
            if (slv0_rvalid && mst0_rready && slv0_rlast)
                r_owner0 <= RSRC_NONE;
        end else begin // RSRC_SLV1
            if (slv1_rvalid && mst0_rready && slv1_rlast)
                r_owner0 <= RSRC_NONE;
        end
    end

    reg [1:0] r_owner1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_owner1 <= RSRC_NONE;
        end else if (r_owner1 == RSRC_NONE) begin
            if (slv0_r_for_m1)
                r_owner1 <= RSRC_SLV0;
            else if (slv1_r_for_m1)
                r_owner1 <= RSRC_SLV1;
        end else if (r_owner1 == RSRC_SLV0) begin
            if (slv0_rvalid && mst1_rready && slv0_rlast)
                r_owner1 <= RSRC_NONE;
        end else begin // RSRC_SLV1
            if (slv1_rvalid && mst1_rready && slv1_rlast)
                r_owner1 <= RSRC_NONE;
        end
    end

    assign mst0_rvalid = (r_owner0 == RSRC_SLV0) ? slv0_rvalid :
                          (r_owner0 == RSRC_SLV1) ? slv1_rvalid : 1'b0;
    assign mst0_rid    = (r_owner0 == RSRC_SLV0) ? slv0_rid[LID_WIDTH-1:0] : slv1_rid[LID_WIDTH-1:0];
    assign mst0_rdata  = (r_owner0 == RSRC_SLV0) ? slv0_rdata : slv1_rdata;
    assign mst0_rresp  = (r_owner0 == RSRC_SLV0) ? slv0_rresp : slv1_rresp;
    assign mst0_rlast  = (r_owner0 == RSRC_SLV0) ? slv0_rlast : slv1_rlast;

    assign mst1_rvalid = (r_owner1 == RSRC_SLV0) ? slv0_rvalid :
                          (r_owner1 == RSRC_SLV1) ? slv1_rvalid : 1'b0;
    assign mst1_rid    = (r_owner1 == RSRC_SLV0) ? slv0_rid[LID_WIDTH-1:0] : slv1_rid[LID_WIDTH-1:0];
    assign mst1_rdata  = (r_owner1 == RSRC_SLV0) ? slv0_rdata : slv1_rdata;
    assign mst1_rresp  = (r_owner1 == RSRC_SLV0) ? slv0_rresp : slv1_rresp;
    assign mst1_rlast  = (r_owner1 == RSRC_SLV0) ? slv0_rlast : slv1_rlast;


    assign slv0_rready = slv0_rid[ID_WIDTH-1] ?
                            ((r_owner1 == RSRC_SLV0) ? mst1_rready : 1'b0) :
                            ((r_owner0 == RSRC_SLV0) ? mst0_rready : 1'b0);

    assign slv1_rready = slv1_rid[ID_WIDTH-1] ?
                            ((r_owner1 == RSRC_SLV1) ? mst1_rready : 1'b0) :
                            ((r_owner0 == RSRC_SLV1) ? mst0_rready : 1'b0);

endmodule
