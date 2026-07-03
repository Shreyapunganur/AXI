`timescale 1ns / 1ps
// =============================================================
// AXI4_checker.sv — Protocol Checker (SVA)
//
// Plug-in alongside existing .v master/slave. Zero changes to .v.
// Instantiated in AXI4_tb_sv.sv as a passive observer.
//
// Assertions (13):
//   VALID_STABLE  : once valid, stays high until ready (all 5 channels)
//   WLAST_BEAT    : wlast fires on beat awlen, not before/after
//   WSTRB_NONZERO : no all-zero strobe on a live beat
//   BRESP_KNOWN   : no X/Z on bresp when bvalid
//   RRESP_KNOWN   : no X/Z on rresp when rvalid
//   BID_MATCH     : bid == latched awid at b-handshake
//   RID_MATCH     : rid == latched arid at r-handshake
//   NO_X_AWVALID  : awvalid never X/Z during active operation
//   NO_X_ARVALID  : arvalid never X/Z during active operation
// =============================================================

module AXI4_checker #(
    parameter ADWIDTH = 32,
    parameter DWIDTH  = 64,
    parameter IDWIDTH = 4
)(
    input logic                  clk,
    input logic                  rst,
    // AW channel
    input logic [IDWIDTH-1:0]    awid,
    input logic [ADWIDTH-1:0]    awaddr,
    input logic [7:0]            awlen,
    input logic [2:0]            awsize,
    input logic [1:0]            awburst,
    input logic                  awvalid,
    input logic                  awready,
    // W channel
    input logic [DWIDTH-1:0]     wdata,
    input logic [DWIDTH/8-1:0]   wstrb,
    input logic                  wlast,
    input logic                  wvalid,
    input logic                  wready,
    // B channel
    input logic [IDWIDTH-1:0]    bid,
    input logic [1:0]            bresp,
    input logic                  bvalid,
    input logic                  bready,
    // AR channel
    input logic [IDWIDTH-1:0]    arid,
    input logic [ADWIDTH-1:0]    araddr,
    input logic [7:0]            arlen,
    input logic [2:0]            arsize,
    input logic [1:0]            arburst,
    input logic                  arvalid,
    input logic                  arready,
    // R channel
    input logic [IDWIDTH-1:0]    rid,
    input logic [DWIDTH-1:0]     rdata,
    input logic [1:0]            rresp,
    input logic                  rlast,
    input logic                  rvalid,
    input logic                  rready
);

    // ----------------------------------------------------------
    // Internal tracking
    // ----------------------------------------------------------
    logic [7:0]         wbeat_cnt;       // counts W beats within a burst
    logic [7:0]         wburst_len_lat;  // awlen latched at AW handshake
    logic [IDWIDTH-1:0] awid_lat;        // awid latched at AW handshake
    logic [IDWIDTH-1:0] arid_lat;        // arid latched at AR handshake

    always_ff @(posedge clk or negedge rst) begin
        if (!rst) begin
            wbeat_cnt      <= '0;
            wburst_len_lat <= '0;
            awid_lat       <= '0;
            arid_lat       <= '0;
        end else begin
            if (awvalid && awready) begin
                wburst_len_lat <= awlen;
                awid_lat       <= awid;
            end
            if (wvalid && wready) begin
                if (wlast) wbeat_cnt <= '0;
                else        wbeat_cnt <= wbeat_cnt + 8'd1;
            end
            if (arvalid && arready)
                arid_lat <= arid;
        end
    end

    // ----------------------------------------------------------
    // Properties — VALID STABILITY (AXI4 spec A3.2.1)
    // Once valid is asserted it must not deassert until the
    // corresponding ready is high.
    // ----------------------------------------------------------
    property p_awvalid_stable;
        @(posedge clk) disable iff (!rst)
        (awvalid && !awready) |=> awvalid;
    endproperty

    property p_wvalid_stable;
        @(posedge clk) disable iff (!rst)
        (wvalid && !wready) |=> wvalid;
    endproperty

    property p_arvalid_stable;
        @(posedge clk) disable iff (!rst)
        (arvalid && !arready) |=> arvalid;
    endproperty

    property p_bvalid_stable;
        @(posedge clk) disable iff (!rst)
        (bvalid && !bready) |=> bvalid;
    endproperty

    property p_rvalid_stable;
        @(posedge clk) disable iff (!rst)
        (rvalid && !rready) |=> rvalid;
    endproperty

    // ----------------------------------------------------------
    // Property — WLAST beat alignment
    // wlast must fire exactly on beat number awlen (0-indexed).
    // wbeat_cnt resets to 0 after wlast, so at the last beat
    // wbeat_cnt == wburst_len_lat.
    // ----------------------------------------------------------
    property p_wlast_correct;
        @(posedge clk) disable iff (!rst)
        (wvalid && wready && wlast) |-> (wbeat_cnt == wburst_len_lat);
    endproperty

    // ----------------------------------------------------------
    // Property — WSTRB non-zero
    // At least one byte lane must be active per W beat.
    // ----------------------------------------------------------
    property p_wstrb_nonzero;
        @(posedge clk) disable iff (!rst)
        (wvalid && wready) |-> (wstrb != '0);
    endproperty

    // ----------------------------------------------------------
    // Properties — Response signal integrity
    // ----------------------------------------------------------
    property p_bresp_known;
        @(posedge clk) disable iff (!rst)
        bvalid |-> !$isunknown(bresp);
    endproperty

    property p_rresp_known;
        @(posedge clk) disable iff (!rst)
        rvalid |-> !$isunknown(rresp);
    endproperty

    // ----------------------------------------------------------
    // Properties — ID matching
    // ----------------------------------------------------------
    property p_bid_matches_awid;
        @(posedge clk) disable iff (!rst)
        (bvalid && bready) |-> (bid == awid_lat);
    endproperty

    property p_rid_matches_arid;
        @(posedge clk) disable iff (!rst)
        (rvalid && rready) |-> (rid == arid_lat);
    endproperty

    // ----------------------------------------------------------
    // Properties — No X/Z on control signals
    // ----------------------------------------------------------
    property p_no_x_awvalid;
        @(posedge clk) disable iff (!rst) !$isunknown(awvalid);
    endproperty

    property p_no_x_arvalid;
        @(posedge clk) disable iff (!rst) !$isunknown(arvalid);
    endproperty

    // ----------------------------------------------------------
    // Assertion instantiation
    // ----------------------------------------------------------
    AST_AWVALID_STABLE : assert property (p_awvalid_stable)
        else $error("[CHK %0t] FAIL awvalid dropped before awready", $time);

    AST_WVALID_STABLE  : assert property (p_wvalid_stable)
        else $error("[CHK %0t] FAIL wvalid dropped before wready", $time);

    AST_ARVALID_STABLE : assert property (p_arvalid_stable)
        else $error("[CHK %0t] FAIL arvalid dropped before arready", $time);

    AST_BVALID_STABLE  : assert property (p_bvalid_stable)
        else $error("[CHK %0t] FAIL bvalid dropped before bready", $time);

    AST_RVALID_STABLE  : assert property (p_rvalid_stable)
        else $error("[CHK %0t] FAIL rvalid dropped before rready", $time);

    AST_WLAST_CORRECT  : assert property (p_wlast_correct)
        else $error("[CHK %0t] FAIL wlast on beat %0d, expected %0d",
                    $time, wbeat_cnt, wburst_len_lat);

    AST_WSTRB_NONZERO  : assert property (p_wstrb_nonzero)
        else $error("[CHK %0t] FAIL all-zero wstrb on W beat", $time);

    AST_BRESP_KNOWN    : assert property (p_bresp_known)
        else $error("[CHK %0t] FAIL X/Z on bresp when bvalid", $time);

    AST_RRESP_KNOWN    : assert property (p_rresp_known)
        else $error("[CHK %0t] FAIL X/Z on rresp when rvalid", $time);

    AST_BID_AWID       : assert property (p_bid_matches_awid)
        else $error("[CHK %0t] FAIL bid=0x%0h != awid_lat=0x%0h",
                    $time, bid, awid_lat);

    AST_RID_ARID       : assert property (p_rid_matches_arid)
        else $error("[CHK %0t] FAIL rid=0x%0h != arid_lat=0x%0h",
                    $time, rid, arid_lat);

    AST_NO_X_AWVALID   : assert property (p_no_x_awvalid)
        else $error("[CHK %0t] FAIL X/Z on awvalid", $time);

    AST_NO_X_ARVALID   : assert property (p_no_x_arvalid)
        else $error("[CHK %0t] FAIL X/Z on arvalid", $time);

    // ----------------------------------------------------------
    // Cover points — scenarios that should be exercised
    // ----------------------------------------------------------
    COV_AW_HANDSHAKE : cover property (@(posedge clk) disable iff (!rst)
        awvalid && awready);
    COV_W_BURST4     : cover property (@(posedge clk) disable iff (!rst)
        awvalid && awready && (awlen == 8'd3));
    COV_W_BURST8     : cover property (@(posedge clk) disable iff (!rst)
        awvalid && awready && (awlen == 8'd7));
    COV_WLAST_SEEN   : cover property (@(posedge clk) disable iff (!rst)
        wvalid && wready && wlast);
    COV_B_OKAY       : cover property (@(posedge clk) disable iff (!rst)
        bvalid && bready && (bresp == 2'b00));
    COV_R_BURST4     : cover property (@(posedge clk) disable iff (!rst)
        arvalid && arready && (arlen == 8'd3));
    COV_R_LAST       : cover property (@(posedge clk) disable iff (!rst)
        rvalid && rready && rlast);

endmodule
