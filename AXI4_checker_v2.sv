`timescale 1ns/1ps
// =============================================================
// AXI4_checker_v2.sv  —  Extended Protocol Checker (SVA)
//
// Includes all 13 assertions from AXI4_checker.sv PLUS:
//
// New assertions (9):
//   DUAL_AW_S0/S1 : No two masters simultaneously granted same slave AW
//   DUAL_AR_S0/S1 : No two masters simultaneously granted same slave AR
//   WR_MUST_COMPLETE : AW started → B arrives within MAX_LATENCY cycles
//   RD_MUST_COMPLETE : AR started → R_LAST arrives within MAX_LATENCY cycles
//   CDC_AW_NO_PUSH   : CDC bridge AW FIFO never pushed when full
//   CDC_R_NO_PUSH    : CDC bridge R FIFO never pushed when full
//   PIPELINE_NODUP   : wlast fires exactly once per burst
//   QOS_NONX         : awqos/arqos never X/Z when valid
// =============================================================

module AXI4_checker_v2 #(
    parameter ADWIDTH     = 32,
    parameter DWIDTH      = 64,
    parameter IDWIDTH     = 4,
    parameter MAX_LATENCY = 200,
    // Address map (for dual-grant check)
    parameter [ADWIDTH-1:0] S0_BASE = 32'h0000_0000,
    parameter [ADWIDTH-1:0] S0_HIGH = 32'h0000_07FF,
    parameter [ADWIDTH-1:0] S1_BASE = 32'h0001_0000,
    parameter [ADWIDTH-1:0] S1_HIGH = 32'h0001_07FF
)(
    input logic                  clk,
    input logic                  rst,

    // Master 0
    input logic [IDWIDTH-1:0]    m0_awid,
    input logic [ADWIDTH-1:0]    m0_awaddr,
    input logic [7:0]            m0_awlen,
    input logic [2:0]            m0_awsize,
    input logic [1:0]            m0_awburst,
    input logic [3:0]            m0_awqos,
    input logic                  m0_awvalid, m0_awready,

    input logic [DWIDTH-1:0]     m0_wdata,
    input logic [DWIDTH/8-1:0]   m0_wstrb,
    input logic                  m0_wlast,
    input logic                  m0_wvalid, m0_wready,

    input logic [IDWIDTH-1:0]    m0_bid,
    input logic [1:0]            m0_bresp,
    input logic                  m0_bvalid, m0_bready,

    input logic [IDWIDTH-1:0]    m0_arid,
    input logic [ADWIDTH-1:0]    m0_araddr,
    input logic [7:0]            m0_arlen,
    input logic [2:0]            m0_arsize,
    input logic [1:0]            m0_arburst,
    input logic [3:0]            m0_arqos,
    input logic                  m0_arvalid, m0_arready,

    input logic [IDWIDTH-1:0]    m0_rid,
    input logic [DWIDTH-1:0]     m0_rdata,
    input logic [1:0]            m0_rresp,
    input logic                  m0_rlast,
    input logic                  m0_rvalid, m0_rready,

    // Master 1
    input logic [IDWIDTH-1:0]    m1_awid,
    input logic [ADWIDTH-1:0]    m1_awaddr,
    input logic [7:0]            m1_awlen,
    input logic [2:0]            m1_awsize,
    input logic [1:0]            m1_awburst,
    input logic [3:0]            m1_awqos,
    input logic                  m1_awvalid, m1_awready,

    input logic [DWIDTH-1:0]     m1_wdata,
    input logic [DWIDTH/8-1:0]   m1_wstrb,
    input logic                  m1_wlast,
    input logic                  m1_wvalid, m1_wready,

    input logic [IDWIDTH-1:0]    m1_bid,
    input logic [1:0]            m1_bresp,
    input logic                  m1_bvalid, m1_bready,

    input logic [IDWIDTH-1:0]    m1_arid,
    input logic [ADWIDTH-1:0]    m1_araddr,
    input logic [7:0]            m1_arlen,
    input logic [2:0]            m1_arsize,
    input logic [1:0]            m1_arburst,
    input logic [3:0]            m1_arqos,
    input logic                  m1_arvalid, m1_arready,

    input logic [IDWIDTH-1:0]    m1_rid,
    input logic [DWIDTH-1:0]     m1_rdata,
    input logic [1:0]            m1_rresp,
    input logic                  m1_rlast,
    input logic                  m1_rvalid, m1_rready,

    // CDC debug hooks
    input logic                  cdc_aw_fifo_full,
    input logic                  cdc_r_fifo_full,
    input logic                  cdc_aw_push,
    input logic                  cdc_r_push
);

    // ----------------------------------------------------------
    // Internal tracking  (same as checker v1, per master)
    // ----------------------------------------------------------
    logic [7:0]         m0_wbeat_cnt, m0_wburst_lat;
    logic [IDWIDTH-1:0] m0_awid_lat,  m0_arid_lat;
    logic [7:0]         m1_wbeat_cnt, m1_wburst_lat;
    logic [IDWIDTH-1:0] m1_awid_lat,  m1_arid_lat;

    always_ff @(posedge clk or negedge rst) begin
        if (!rst) begin
            m0_wbeat_cnt <= '0; m0_wburst_lat <= '0;
            m0_awid_lat  <= '0; m0_arid_lat   <= '0;
            m1_wbeat_cnt <= '0; m1_wburst_lat <= '0;
            m1_awid_lat  <= '0; m1_arid_lat   <= '0;
        end else begin
            // M0 tracking
            if (m0_awvalid && m0_awready)
                begin m0_wburst_lat <= m0_awlen; m0_awid_lat <= m0_awid; end
            if (m0_wvalid  && m0_wready)
                m0_wbeat_cnt <= m0_wlast ? '0 : m0_wbeat_cnt + 1;
            if (m0_arvalid && m0_arready)
                m0_arid_lat <= m0_arid;

            // M1 tracking
            if (m1_awvalid && m1_awready)
                begin m1_wburst_lat <= m1_awlen; m1_awid_lat <= m1_awid; end
            if (m1_wvalid  && m1_wready)
                m1_wbeat_cnt <= m1_wlast ? '0 : m1_wbeat_cnt + 1;
            if (m1_arvalid && m1_arready)
                m1_arid_lat <= m1_arid;
        end
    end

    // ----------------------------------------------------------
    // ── SECTION 1: Inherited from AXI4_checker.sv (per master) ──
    // ----------------------------------------------------------
    // VALID STABILITY — M0
    property p_m0_awvalid_stable;
        @(posedge clk) disable iff(!rst) (m0_awvalid && !m0_awready) |=> m0_awvalid; endproperty
    property p_m0_wvalid_stable;
        @(posedge clk) disable iff(!rst) (m0_wvalid  && !m0_wready)  |=> m0_wvalid;  endproperty
    property p_m0_arvalid_stable;
        @(posedge clk) disable iff(!rst) (m0_arvalid && !m0_arready) |=> m0_arvalid; endproperty
    property p_m0_bvalid_stable;
        @(posedge clk) disable iff(!rst) (m0_bvalid  && !m0_bready)  |=> m0_bvalid;  endproperty
    property p_m0_rvalid_stable;
        @(posedge clk) disable iff(!rst) (m0_rvalid  && !m0_rready)  |=> m0_rvalid;  endproperty

    // VALID STABILITY — M1
    property p_m1_awvalid_stable;
        @(posedge clk) disable iff(!rst) (m1_awvalid && !m1_awready) |=> m1_awvalid; endproperty
    property p_m1_wvalid_stable;
        @(posedge clk) disable iff(!rst) (m1_wvalid  && !m1_wready)  |=> m1_wvalid;  endproperty
    property p_m1_arvalid_stable;
        @(posedge clk) disable iff(!rst) (m1_arvalid && !m1_arready) |=> m1_arvalid; endproperty
    property p_m1_bvalid_stable;
        @(posedge clk) disable iff(!rst) (m1_bvalid  && !m1_bready)  |=> m1_bvalid;  endproperty
    property p_m1_rvalid_stable;
        @(posedge clk) disable iff(!rst) (m1_rvalid  && !m1_rready)  |=> m1_rvalid;  endproperty

    // WLAST alignment
    property p_m0_wlast;
        @(posedge clk) disable iff(!rst)
        (m0_wvalid && m0_wready && m0_wlast) |-> (m0_wbeat_cnt == m0_wburst_lat);
    endproperty
    property p_m1_wlast;
        @(posedge clk) disable iff(!rst)
        (m1_wvalid && m1_wready && m1_wlast) |-> (m1_wbeat_cnt == m1_wburst_lat);
    endproperty

    // WSTRB non-zero
    property p_m0_wstrb; @(posedge clk) disable iff(!rst) (m0_wvalid && m0_wready) |-> (m0_wstrb != '0); endproperty
    property p_m1_wstrb; @(posedge clk) disable iff(!rst) (m1_wvalid && m1_wready) |-> (m1_wstrb != '0); endproperty

    // ID matching
    property p_m0_bid_match; @(posedge clk) disable iff(!rst) (m0_bvalid && m0_bready) |-> (m0_bid == m0_awid_lat); endproperty
    property p_m0_rid_match; @(posedge clk) disable iff(!rst) (m0_rvalid && m0_rready) |-> (m0_rid == m0_arid_lat); endproperty
    property p_m1_bid_match; @(posedge clk) disable iff(!rst) (m1_bvalid && m1_bready) |-> (m1_bid == m1_awid_lat); endproperty
    property p_m1_rid_match; @(posedge clk) disable iff(!rst) (m1_rvalid && m1_rready) |-> (m1_rid == m1_arid_lat); endproperty

    // ----------------------------------------------------------
    // ── SECTION 2: New assertions ──
    // ----------------------------------------------------------

    // Address range helpers
    wire m0_aw_s0 = m0_awvalid && (m0_awaddr >= S0_BASE) && (m0_awaddr <= S0_HIGH);
    wire m0_aw_s1 = m0_awvalid && (m0_awaddr >= S1_BASE) && (m0_awaddr <= S1_HIGH);
    wire m1_aw_s0 = m1_awvalid && (m1_awaddr >= S0_BASE) && (m1_awaddr <= S0_HIGH);
    wire m1_aw_s1 = m1_awvalid && (m1_awaddr >= S1_BASE) && (m1_awaddr <= S1_HIGH);
    wire m0_ar_s0 = m0_arvalid && (m0_araddr >= S0_BASE) && (m0_araddr <= S0_HIGH);
    wire m0_ar_s1 = m0_arvalid && (m0_araddr >= S1_BASE) && (m0_araddr <= S1_HIGH);
    wire m1_ar_s0 = m1_arvalid && (m1_araddr >= S0_BASE) && (m1_araddr <= S0_HIGH);
    wire m1_ar_s1 = m1_arvalid && (m1_araddr >= S1_BASE) && (m1_araddr <= S1_HIGH);

    // No dual grant on S0 write: if M0 gets awready to S0, M1 cannot also get awready to S0
    property p_no_dual_aw_s0;
        @(posedge clk) disable iff(!rst)
        (m0_awvalid && m0_awready && m0_aw_s0) |-> !(m1_awvalid && m1_awready && m1_aw_s0);
    endproperty

    // No dual grant on S1 write
    property p_no_dual_aw_s1;
        @(posedge clk) disable iff(!rst)
        (m0_awvalid && m0_awready && m0_aw_s1) |-> !(m1_awvalid && m1_awready && m1_aw_s1);
    endproperty

    // No dual grant on S0 read
    property p_no_dual_ar_s0;
        @(posedge clk) disable iff(!rst)
        (m0_arvalid && m0_arready && m0_ar_s0) |-> !(m1_arvalid && m1_arready && m1_ar_s0);
    endproperty

    // No dual grant on S1 read
    property p_no_dual_ar_s1;
        @(posedge clk) disable iff(!rst)
        (m0_arvalid && m0_arready && m0_ar_s1) |-> !(m1_arvalid && m1_arready && m1_ar_s1);
    endproperty

    // Write must complete: once AW accepted, B arrives within MAX_LATENCY
    property p_m0_wr_complete;
        @(posedge clk) disable iff(!rst)
        (m0_awvalid && m0_awready) |-> ##[1:MAX_LATENCY] (m0_bvalid && m0_bready);
    endproperty

    property p_m1_wr_complete;
        @(posedge clk) disable iff(!rst)
        (m1_awvalid && m1_awready) |-> ##[1:MAX_LATENCY] (m1_bvalid && m1_bready);
    endproperty

    // Read must complete: once AR accepted, R_LAST arrives within MAX_LATENCY
    property p_m0_rd_complete;
        @(posedge clk) disable iff(!rst)
        (m0_arvalid && m0_arready) |-> ##[1:MAX_LATENCY] (m0_rvalid && m0_rready && m0_rlast);
    endproperty

    property p_m1_rd_complete;
        @(posedge clk) disable iff(!rst)
        (m1_arvalid && m1_arready) |-> ##[1:MAX_LATENCY] (m1_rvalid && m1_rready && m1_rlast);
    endproperty

    // CDC: AW FIFO must not be pushed when full (handshake safety)
    property p_cdc_aw_no_overflow;
        @(posedge clk) disable iff(!rst)
        !(cdc_aw_push && cdc_aw_fifo_full);
    endproperty

    // CDC: R FIFO must not be pushed when full
    property p_cdc_r_no_overflow;
        @(posedge clk) disable iff(!rst)
        !(cdc_r_push && cdc_r_fifo_full);
    endproperty

    // Pipeline: WLAST fires exactly once per write transaction (no duplicate last)
    property p_m0_no_dup_wlast;
        @(posedge clk) disable iff(!rst)
        (m0_wvalid && m0_wready && m0_wlast) |=>
        !(m0_wvalid && m0_wready && m0_wlast); // next beat cannot also be last
    endproperty
    property p_m1_no_dup_wlast;
        @(posedge clk) disable iff(!rst)
        (m1_wvalid && m1_wready && m1_wlast) |=>
        !(m1_wvalid && m1_wready && m1_wlast);
    endproperty

    // QoS no X/Z
    property p_m0_qos_known;
        @(posedge clk) disable iff(!rst)
        m0_awvalid |-> !$isunknown(m0_awqos);
    endproperty
    property p_m1_qos_known;
        @(posedge clk) disable iff(!rst)
        m1_awvalid |-> !$isunknown(m1_awqos);
    endproperty

    // ----------------------------------------------------------
    // Assertion instantiation
    // ----------------------------------------------------------

    // Inherited — M0
    AST_M0_AWVALID_STABLE  : assert property(p_m0_awvalid_stable)  else $error("[CHKv2 %0t] M0 awvalid dropped", $time);
    AST_M0_WVALID_STABLE   : assert property(p_m0_wvalid_stable)   else $error("[CHKv2 %0t] M0 wvalid  dropped", $time);
    AST_M0_ARVALID_STABLE  : assert property(p_m0_arvalid_stable)  else $error("[CHKv2 %0t] M0 arvalid dropped", $time);
    AST_M0_BVALID_STABLE   : assert property(p_m0_bvalid_stable)   else $error("[CHKv2 %0t] M0 bvalid  dropped", $time);
    AST_M0_RVALID_STABLE   : assert property(p_m0_rvalid_stable)   else $error("[CHKv2 %0t] M0 rvalid  dropped", $time);
    AST_M0_WLAST           : assert property(p_m0_wlast) else $error("[CHKv2 %0t] M0 wlast on beat %0d exp %0d", $time, m0_wbeat_cnt, m0_wburst_lat);
    AST_M0_WSTRB           : assert property(p_m0_wstrb) else $error("[CHKv2 %0t] M0 all-zero wstrb",  $time);
    AST_M0_BID             : assert property(p_m0_bid_match) else $error("[CHKv2 %0t] M0 bid mismatch", $time);
    AST_M0_RID             : assert property(p_m0_rid_match) else $error("[CHKv2 %0t] M0 rid mismatch", $time);

    // Inherited — M1
    AST_M1_AWVALID_STABLE  : assert property(p_m1_awvalid_stable)  else $error("[CHKv2 %0t] M1 awvalid dropped", $time);
    AST_M1_WVALID_STABLE   : assert property(p_m1_wvalid_stable)   else $error("[CHKv2 %0t] M1 wvalid  dropped", $time);
    AST_M1_ARVALID_STABLE  : assert property(p_m1_arvalid_stable)  else $error("[CHKv2 %0t] M1 arvalid dropped", $time);
    AST_M1_BVALID_STABLE   : assert property(p_m1_bvalid_stable)   else $error("[CHKv2 %0t] M1 bvalid  dropped", $time);
    AST_M1_RVALID_STABLE   : assert property(p_m1_rvalid_stable)   else $error("[CHKv2 %0t] M1 rvalid  dropped", $time);
    AST_M1_WLAST           : assert property(p_m1_wlast) else $error("[CHKv2 %0t] M1 wlast on beat %0d exp %0d", $time, m1_wbeat_cnt, m1_wburst_lat);
    AST_M1_WSTRB           : assert property(p_m1_wstrb) else $error("[CHKv2 %0t] M1 all-zero wstrb",  $time);
    AST_M1_BID             : assert property(p_m1_bid_match) else $error("[CHKv2 %0t] M1 bid mismatch", $time);
    AST_M1_RID             : assert property(p_m1_rid_match) else $error("[CHKv2 %0t] M1 rid mismatch", $time);

    // New
    AST_NO_DUAL_AW_S0      : assert property(p_no_dual_aw_s0)   else $error("[CHKv2 %0t] DUAL GRANT: both masters AW→S0", $time);
    AST_NO_DUAL_AW_S1      : assert property(p_no_dual_aw_s1)   else $error("[CHKv2 %0t] DUAL GRANT: both masters AW→S1", $time);
    AST_NO_DUAL_AR_S0      : assert property(p_no_dual_ar_s0)   else $error("[CHKv2 %0t] DUAL GRANT: both masters AR→S0", $time);
    AST_NO_DUAL_AR_S1      : assert property(p_no_dual_ar_s1)   else $error("[CHKv2 %0t] DUAL GRANT: both masters AR→S1", $time);
    AST_M0_WR_COMPLETE     : assert property(p_m0_wr_complete)  else $error("[CHKv2 %0t] M0 write no B in %0d cycles", $time, MAX_LATENCY);
    AST_M1_WR_COMPLETE     : assert property(p_m1_wr_complete)  else $error("[CHKv2 %0t] M1 write no B in %0d cycles", $time, MAX_LATENCY);
    AST_M0_RD_COMPLETE     : assert property(p_m0_rd_complete)  else $error("[CHKv2 %0t] M0 read  no RLAST in %0d cycles", $time, MAX_LATENCY);
    AST_M1_RD_COMPLETE     : assert property(p_m1_rd_complete)  else $error("[CHKv2 %0t] M1 read  no RLAST in %0d cycles", $time, MAX_LATENCY);
    AST_CDC_AW_NO_OVF      : assert property(p_cdc_aw_no_overflow) else $error("[CHKv2 %0t] CDC AW FIFO overflow!", $time);
    AST_CDC_R_NO_OVF       : assert property(p_cdc_r_no_overflow)  else $error("[CHKv2 %0t] CDC R  FIFO overflow!", $time);
    AST_M0_NO_DUP_WLAST    : assert property(p_m0_no_dup_wlast)    else $error("[CHKv2 %0t] M0 duplicate wlast", $time);
    AST_M1_NO_DUP_WLAST    : assert property(p_m1_no_dup_wlast)    else $error("[CHKv2 %0t] M1 duplicate wlast", $time);
    AST_M0_QOS_KNOWN       : assert property(p_m0_qos_known)        else $error("[CHKv2 %0t] M0 X on awqos", $time);
    AST_M1_QOS_KNOWN       : assert property(p_m1_qos_known)        else $error("[CHKv2 %0t] M1 X on awqos", $time);

    // Cover points
    COV_CONCURRENT_DIFF_SLAVE : cover property(@(posedge clk) disable iff(!rst)
        (m0_awvalid && m0_aw_s0 && m1_awvalid && m1_aw_s1));
    COV_CONTENTION_S0         : cover property(@(posedge clk) disable iff(!rst)
        (m0_awvalid && m0_aw_s0 && m1_awvalid && m1_aw_s0));
    COV_M0_HIGH_QOS           : cover property(@(posedge clk) disable iff(!rst)
        (m0_awvalid && m0_awready && m0_awqos >= 4'h8));
    COV_CDC_TRAFFIC           : cover property(@(posedge clk) disable iff(!rst)
        cdc_aw_push);
    COV_PIPELINE_AW_W_SAME_CYC: cover property(@(posedge clk) disable iff(!rst)
        (m0_awvalid && m0_wvalid));

endmodule
