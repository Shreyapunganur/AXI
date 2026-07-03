`timescale 1ns / 1ps
// =============================================================
// AXI4_interconnect.v — 2 Masters × 2 Slaves Crossbar
//
// Architecture:
//   - Per-slave write FSM  : WR_IDLE → WR_ADDR → WR_DATA → WR_RESP
//   - Per-slave read  FSM  : RD_IDLE → RD_ADDR → RD_DATA
//   - Address decode       : combinational, based on S0/S1 address ranges
//   - Arbitration          : round-robin per slave (registered priority)
//   - One transaction at a time per slave (no pipelining)
//
// Address map (configurable via parameters):
//   Slave 0 : S0_BASE .. S0_HIGH  (default 0x0000_0000..0x0000_00FF)
//   Slave 1 : S1_BASE .. S1_HIGH  (default 0x0001_0000..0x0001_00FF)
//
// Key design note:
//   All master-facing outputs are driven from always @(*) blocks,
//   selected by FSM state + grant registers. This avoids any
//   multiple-driver issues between the two slave FSMs.
// =============================================================

module AXI4_interconnect #(
    parameter ADWIDTH = 32,
    parameter DWIDTH  = 64,
    parameter IDWIDTH = 4,
    parameter [ADWIDTH-1:0] S0_BASE = 32'h0000_0000,
    parameter [ADWIDTH-1:0] S0_HIGH = 32'h0000_00FF,
    parameter [ADWIDTH-1:0] S1_BASE = 32'h0001_0000,
    parameter [ADWIDTH-1:0] S1_HIGH = 32'h0001_00FF
)(
    input  wire clk,
    input  wire rst,

    // ===================== MASTER 0 =====================
    input  wire [IDWIDTH-1:0]   m0_awid,
    input  wire [ADWIDTH-1:0]   m0_awaddr,
    input  wire [7:0]           m0_awlen,
    input  wire [2:0]           m0_awsize,
    input  wire [1:0]           m0_awburst,
    input  wire                 m0_awvalid,
    output reg                  m0_awready,

    input  wire [DWIDTH-1:0]    m0_wdata,
    input  wire [DWIDTH/8-1:0]  m0_wstrb,
    input  wire                 m0_wlast,
    input  wire                 m0_wvalid,
    output reg                  m0_wready,

    output reg  [IDWIDTH-1:0]   m0_bid,
    output reg  [1:0]           m0_bresp,
    output reg                  m0_bvalid,
    input  wire                 m0_bready,

    input  wire [IDWIDTH-1:0]   m0_arid,
    input  wire [ADWIDTH-1:0]   m0_araddr,
    input  wire [7:0]           m0_arlen,
    input  wire [2:0]           m0_arsize,
    input  wire [1:0]           m0_arburst,
    input  wire                 m0_arvalid,
    output reg                  m0_arready,

    output reg  [IDWIDTH-1:0]   m0_rid,
    output reg  [DWIDTH-1:0]    m0_rdata,
    output reg  [1:0]           m0_rresp,
    output reg                  m0_rlast,
    output reg                  m0_rvalid,
    input  wire                 m0_rready,

    // ===================== MASTER 1 =====================
    input  wire [IDWIDTH-1:0]   m1_awid,
    input  wire [ADWIDTH-1:0]   m1_awaddr,
    input  wire [7:0]           m1_awlen,
    input  wire [2:0]           m1_awsize,
    input  wire [1:0]           m1_awburst,
    input  wire                 m1_awvalid,
    output reg                  m1_awready,

    input  wire [DWIDTH-1:0]    m1_wdata,
    input  wire [DWIDTH/8-1:0]  m1_wstrb,
    input  wire                 m1_wlast,
    input  wire                 m1_wvalid,
    output reg                  m1_wready,

    output reg  [IDWIDTH-1:0]   m1_bid,
    output reg  [1:0]           m1_bresp,
    output reg                  m1_bvalid,
    input  wire                 m1_bready,

    input  wire [IDWIDTH-1:0]   m1_arid,
    input  wire [ADWIDTH-1:0]   m1_araddr,
    input  wire [7:0]           m1_arlen,
    input  wire [2:0]           m1_arsize,
    input  wire [1:0]           m1_arburst,
    input  wire                 m1_arvalid,
    output reg                  m1_arready,

    output reg  [IDWIDTH-1:0]   m1_rid,
    output reg  [DWIDTH-1:0]    m1_rdata,
    output reg  [1:0]           m1_rresp,
    output reg                  m1_rlast,
    output reg                  m1_rvalid,
    input  wire                 m1_rready,

    // ===================== SLAVE 0 =====================
    output reg  [IDWIDTH-1:0]   s0_awid,
    output reg  [ADWIDTH-1:0]   s0_awaddr,
    output reg  [7:0]           s0_awlen,
    output reg  [2:0]           s0_awsize,
    output reg  [1:0]           s0_awburst,
    output reg                  s0_awvalid,
    input  wire                 s0_awready,

    output reg  [DWIDTH-1:0]    s0_wdata,
    output reg  [DWIDTH/8-1:0]  s0_wstrb,
    output reg                  s0_wlast,
    output reg                  s0_wvalid,
    input  wire                 s0_wready,

    input  wire [IDWIDTH-1:0]   s0_bid,
    input  wire [1:0]           s0_bresp,
    input  wire                 s0_bvalid,
    output reg                  s0_bready,

    output reg  [IDWIDTH-1:0]   s0_arid,
    output reg  [ADWIDTH-1:0]   s0_araddr,
    output reg  [7:0]           s0_arlen,
    output reg  [2:0]           s0_arsize,
    output reg  [1:0]           s0_arburst,
    output reg                  s0_arvalid,
    input  wire                 s0_arready,

    input  wire [IDWIDTH-1:0]   s0_rid,
    input  wire [DWIDTH-1:0]    s0_rdata,
    input  wire [1:0]           s0_rresp,
    input  wire                 s0_rlast,
    input  wire                 s0_rvalid,
    output reg                  s0_rready,

    // ===================== SLAVE 1 =====================
    output reg  [IDWIDTH-1:0]   s1_awid,
    output reg  [ADWIDTH-1:0]   s1_awaddr,
    output reg  [7:0]           s1_awlen,
    output reg  [2:0]           s1_awsize,
    output reg  [1:0]           s1_awburst,
    output reg                  s1_awvalid,
    input  wire                 s1_awready,

    output reg  [DWIDTH-1:0]    s1_wdata,
    output reg  [DWIDTH/8-1:0]  s1_wstrb,
    output reg                  s1_wlast,
    output reg                  s1_wvalid,
    input  wire                 s1_wready,

    input  wire [IDWIDTH-1:0]   s1_bid,
    input  wire [1:0]           s1_bresp,
    input  wire                 s1_bvalid,
    output reg                  s1_bready,

    output reg  [IDWIDTH-1:0]   s1_arid,
    output reg  [ADWIDTH-1:0]   s1_araddr,
    output reg  [7:0]           s1_arlen,
    output reg  [2:0]           s1_arsize,
    output reg  [1:0]           s1_arburst,
    output reg                  s1_arvalid,
    input  wire                 s1_arready,

    input  wire [IDWIDTH-1:0]   s1_rid,
    input  wire [DWIDTH-1:0]    s1_rdata,
    input  wire [1:0]           s1_rresp,
    input  wire                 s1_rlast,
    input  wire                 s1_rvalid,
    output reg                  s1_rready
);

    // ----------------------------------------------------------
    // Address decode — combinational
    // ----------------------------------------------------------
    // Write: does master X's awaddr fall in slave Y's range?
    wire m0_aw_to_s0 = m0_awvalid &&
                       (m0_awaddr >= S0_BASE) && (m0_awaddr <= S0_HIGH);
    wire m0_aw_to_s1 = m0_awvalid &&
                       (m0_awaddr >= S1_BASE) && (m0_awaddr <= S1_HIGH);
    wire m1_aw_to_s0 = m1_awvalid &&
                       (m1_awaddr >= S0_BASE) && (m1_awaddr <= S0_HIGH);
    wire m1_aw_to_s1 = m1_awvalid &&
                       (m1_awaddr >= S1_BASE) && (m1_awaddr <= S1_HIGH);

    // Read: same for araddr
    wire m0_ar_to_s0 = m0_arvalid &&
                       (m0_araddr >= S0_BASE) && (m0_araddr <= S0_HIGH);
    wire m0_ar_to_s1 = m0_arvalid &&
                       (m0_araddr >= S1_BASE) && (m0_araddr <= S1_HIGH);
    wire m1_ar_to_s0 = m1_arvalid &&
                       (m1_araddr >= S0_BASE) && (m1_araddr <= S0_HIGH);
    wire m1_ar_to_s1 = m1_arvalid &&
                       (m1_araddr >= S1_BASE) && (m1_araddr <= S1_HIGH);

    // ----------------------------------------------------------
    // FSM state encoding
    // ----------------------------------------------------------
    localparam [2:0]
        WR_IDLE = 3'd0,
        WR_ADDR = 3'd1,
        WR_DATA = 3'd2,
        WR_RESP = 3'd3;

    localparam [1:0]
        RD_IDLE = 2'd0,
        RD_ADDR = 2'd1,
        RD_DATA = 2'd2;

    // Per-slave state and grant registers
    // Index 0 = Slave 0, Index 1 = Slave 1
    reg [2:0] wr_state [0:1]; // write FSM state
    reg       wr_grant [0:1]; // 0 = M0 holds write grant, 1 = M1
    reg       wr_rr    [0:1]; // round-robin: 0 = M0 has priority next

    reg [1:0] rd_state [0:1]; // read FSM state
    reg       rd_grant [0:1]; // 0 = M0 holds read grant, 1 = M1
    reg       rd_rr    [0:1]; // round-robin priority

    // ----------------------------------------------------------
    // Write FSM — Slave 0
    // ----------------------------------------------------------
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            wr_state[0] <= WR_IDLE;
            wr_grant[0] <= 1'b0;
            wr_rr[0]    <= 1'b0;
        end else begin
            case (wr_state[0])

                WR_IDLE: begin
                    if (m0_aw_to_s0 || m1_aw_to_s0) begin
                        // Arbitrate: round-robin when both request
                        if (m0_aw_to_s0 && m1_aw_to_s0) begin
                            wr_grant[0] <= wr_rr[0];     // 0→M0, 1→M1
                            wr_rr[0]    <= ~wr_rr[0];    // flip for next time
                        end else begin
                            wr_grant[0] <= m1_aw_to_s0 ? 1'b1 : 1'b0;
                        end
                        wr_state[0] <= WR_ADDR;
                    end
                end

                WR_ADDR: begin
                    // Hold until slave accepts AW
                    if (s0_awvalid && s0_awready)
                        wr_state[0] <= WR_DATA;
                end

                WR_DATA: begin
                    // Hold until last W beat is accepted
                    if (s0_wvalid && s0_wready && s0_wlast)
                        wr_state[0] <= WR_RESP;
                end

                WR_RESP: begin
                    // Wait for master to accept B response
                    if (s0_bvalid && s0_bready)
                        wr_state[0] <= WR_IDLE;
                end

                default: wr_state[0] <= WR_IDLE;
            endcase
        end
    end

    // ----------------------------------------------------------
    // Write FSM — Slave 1
    // ----------------------------------------------------------
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            wr_state[1] <= WR_IDLE;
            wr_grant[1] <= 1'b0;
            wr_rr[1]    <= 1'b0;
        end else begin
            case (wr_state[1])

                WR_IDLE: begin
                    if (m0_aw_to_s1 || m1_aw_to_s1) begin
                        if (m0_aw_to_s1 && m1_aw_to_s1) begin
                            wr_grant[1] <= wr_rr[1];
                            wr_rr[1]    <= ~wr_rr[1];
                        end else begin
                            wr_grant[1] <= m1_aw_to_s1 ? 1'b1 : 1'b0;
                        end
                        wr_state[1] <= WR_ADDR;
                    end
                end

                WR_ADDR: begin
                    if (s1_awvalid && s1_awready)
                        wr_state[1] <= WR_DATA;
                end

                WR_DATA: begin
                    if (s1_wvalid && s1_wready && s1_wlast)
                        wr_state[1] <= WR_RESP;
                end

                WR_RESP: begin
                    if (s1_bvalid && s1_bready)
                        wr_state[1] <= WR_IDLE;
                end

                default: wr_state[1] <= WR_IDLE;
            endcase
        end
    end

    // ----------------------------------------------------------
    // Read FSM — Slave 0
    // ----------------------------------------------------------
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            rd_state[0] <= RD_IDLE;
            rd_grant[0] <= 1'b0;
            rd_rr[0]    <= 1'b0;
        end else begin
            case (rd_state[0])

                RD_IDLE: begin
                    if (m0_ar_to_s0 || m1_ar_to_s0) begin
                        if (m0_ar_to_s0 && m1_ar_to_s0) begin
                            rd_grant[0] <= rd_rr[0];
                            rd_rr[0]    <= ~rd_rr[0];
                        end else begin
                            rd_grant[0] <= m1_ar_to_s0 ? 1'b1 : 1'b0;
                        end
                        rd_state[0] <= RD_ADDR;
                    end
                end

                RD_ADDR: begin
                    if (s0_arvalid && s0_arready)
                        rd_state[0] <= RD_DATA;
                end

                RD_DATA: begin
                    if (s0_rvalid && s0_rready && s0_rlast)
                        rd_state[0] <= RD_IDLE;
                end

                default: rd_state[0] <= RD_IDLE;
            endcase
        end
    end

    // ----------------------------------------------------------
    // Read FSM — Slave 1
    // ----------------------------------------------------------
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            rd_state[1] <= RD_IDLE;
            rd_grant[1] <= 1'b0;
            rd_rr[1]    <= 1'b0;
        end else begin
            case (rd_state[1])

                RD_IDLE: begin
                    if (m0_ar_to_s1 || m1_ar_to_s1) begin
                        if (m0_ar_to_s1 && m1_ar_to_s1) begin
                            rd_grant[1] <= rd_rr[1];
                            rd_rr[1]    <= ~rd_rr[1];
                        end else begin
                            rd_grant[1] <= m1_ar_to_s1 ? 1'b1 : 1'b0;
                        end
                        rd_state[1] <= RD_ADDR;
                    end
                end

                RD_ADDR: begin
                    if (s1_arvalid && s1_arready)
                        rd_state[1] <= RD_DATA;
                end

                RD_DATA: begin
                    if (s1_rvalid && s1_rready && s1_rlast)
                        rd_state[1] <= RD_IDLE;
                end

                default: rd_state[1] <= RD_IDLE;
            endcase
        end
    end

    // ----------------------------------------------------------
    // Slave-facing combinational mux
    // Select M0 or M1 signals based on grant + FSM state
    // ----------------------------------------------------------

    // ----- Slave 0 Write AW channel -----
    always @(*) begin
        if (wr_state[0] == WR_ADDR && wr_grant[0] == 1'b0) begin
            s0_awid = m0_awid; s0_awaddr = m0_awaddr; s0_awlen  = m0_awlen;
            s0_awsize = m0_awsize; s0_awburst = m0_awburst; s0_awvalid = m0_awvalid;
        end else if (wr_state[0] == WR_ADDR && wr_grant[0] == 1'b1) begin
            s0_awid = m1_awid; s0_awaddr = m1_awaddr; s0_awlen  = m1_awlen;
            s0_awsize = m1_awsize; s0_awburst = m1_awburst; s0_awvalid = m1_awvalid;
        end else begin
            s0_awid = 0; s0_awaddr = 0; s0_awlen = 0;
            s0_awsize = 0; s0_awburst = 0; s0_awvalid = 1'b0;
        end
    end

    // ----- Slave 0 Write W channel -----
    always @(*) begin
        if (wr_state[0] == WR_DATA && wr_grant[0] == 1'b0) begin
            s0_wdata = m0_wdata; s0_wstrb = m0_wstrb;
            s0_wlast = m0_wlast; s0_wvalid = m0_wvalid;
        end else if (wr_state[0] == WR_DATA && wr_grant[0] == 1'b1) begin
            s0_wdata = m1_wdata; s0_wstrb = m1_wstrb;
            s0_wlast = m1_wlast; s0_wvalid = m1_wvalid;
        end else begin
            s0_wdata = 0; s0_wstrb = 0; s0_wlast = 1'b0; s0_wvalid = 1'b0;
        end
    end

    // ----- Slave 0 Write B channel (bready) -----
    always @(*) begin
        s0_bready = (wr_state[0] == WR_RESP) &&
                    ((wr_grant[0] == 1'b0) ? m0_bready : m1_bready);
    end

    // ----- Slave 1 Write AW channel -----
    always @(*) begin
        if (wr_state[1] == WR_ADDR && wr_grant[1] == 1'b0) begin
            s1_awid = m0_awid; s1_awaddr = m0_awaddr; s1_awlen  = m0_awlen;
            s1_awsize = m0_awsize; s1_awburst = m0_awburst; s1_awvalid = m0_awvalid;
        end else if (wr_state[1] == WR_ADDR && wr_grant[1] == 1'b1) begin
            s1_awid = m1_awid; s1_awaddr = m1_awaddr; s1_awlen  = m1_awlen;
            s1_awsize = m1_awsize; s1_awburst = m1_awburst; s1_awvalid = m1_awvalid;
        end else begin
            s1_awid = 0; s1_awaddr = 0; s1_awlen = 0;
            s1_awsize = 0; s1_awburst = 0; s1_awvalid = 1'b0;
        end
    end

    // ----- Slave 1 Write W channel -----
    always @(*) begin
        if (wr_state[1] == WR_DATA && wr_grant[1] == 1'b0) begin
            s1_wdata = m0_wdata; s1_wstrb = m0_wstrb;
            s1_wlast = m0_wlast; s1_wvalid = m0_wvalid;
        end else if (wr_state[1] == WR_DATA && wr_grant[1] == 1'b1) begin
            s1_wdata = m1_wdata; s1_wstrb = m1_wstrb;
            s1_wlast = m1_wlast; s1_wvalid = m1_wvalid;
        end else begin
            s1_wdata = 0; s1_wstrb = 0; s1_wlast = 1'b0; s1_wvalid = 1'b0;
        end
    end

    // ----- Slave 1 Write B channel (bready) -----
    always @(*) begin
        s1_bready = (wr_state[1] == WR_RESP) &&
                    ((wr_grant[1] == 1'b0) ? m0_bready : m1_bready);
    end

    // ----- Slave 0 Read AR channel -----
    always @(*) begin
        if (rd_state[0] == RD_ADDR && rd_grant[0] == 1'b0) begin
            s0_arid = m0_arid; s0_araddr = m0_araddr; s0_arlen  = m0_arlen;
            s0_arsize = m0_arsize; s0_arburst = m0_arburst; s0_arvalid = m0_arvalid;
        end else if (rd_state[0] == RD_ADDR && rd_grant[0] == 1'b1) begin
            s0_arid = m1_arid; s0_araddr = m1_araddr; s0_arlen  = m1_arlen;
            s0_arsize = m1_arsize; s0_arburst = m1_arburst; s0_arvalid = m1_arvalid;
        end else begin
            s0_arid = 0; s0_araddr = 0; s0_arlen = 0;
            s0_arsize = 0; s0_arburst = 0; s0_arvalid = 1'b0;
        end
    end

    // ----- Slave 0 Read R channel (rready) -----
    always @(*) begin
        s0_rready = (rd_state[0] == RD_DATA) &&
                    ((rd_grant[0] == 1'b0) ? m0_rready : m1_rready);
    end

    // ----- Slave 1 Read AR channel -----
    always @(*) begin
        if (rd_state[1] == RD_ADDR && rd_grant[1] == 1'b0) begin
            s1_arid = m0_arid; s1_araddr = m0_araddr; s1_arlen  = m0_arlen;
            s1_arsize = m0_arsize; s1_arburst = m0_arburst; s1_arvalid = m0_arvalid;
        end else if (rd_state[1] == RD_ADDR && rd_grant[1] == 1'b1) begin
            s1_arid = m1_arid; s1_araddr = m1_araddr; s1_arlen  = m1_arlen;
            s1_arsize = m1_arsize; s1_arburst = m1_arburst; s1_arvalid = m1_arvalid;
        end else begin
            s1_arid = 0; s1_araddr = 0; s1_arlen = 0;
            s1_arsize = 0; s1_arburst = 0; s1_arvalid = 1'b0;
        end
    end

    // ----- Slave 1 Read R channel (rready) -----
    always @(*) begin
        s1_rready = (rd_state[1] == RD_DATA) &&
                    ((rd_grant[1] == 1'b0) ? m0_rready : m1_rready);
    end

    // ----------------------------------------------------------
    // Master-facing combinational output
    // Each master signal = OR across all slaves where it holds grant
    // Only one slave's FSM can hold grant for a given master at a time.
    // ----------------------------------------------------------

    // ----- Master 0 awready -----
    always @(*) begin
        m0_awready =
            (wr_state[0] == WR_ADDR && wr_grant[0] == 1'b0 && s0_awready) ||
            (wr_state[1] == WR_ADDR && wr_grant[1] == 1'b0 && s1_awready);
    end

    // ----- Master 1 awready -----
    always @(*) begin
        m1_awready =
            (wr_state[0] == WR_ADDR && wr_grant[0] == 1'b1 && s0_awready) ||
            (wr_state[1] == WR_ADDR && wr_grant[1] == 1'b1 && s1_awready);
    end

    // ----- Master 0 wready -----
    always @(*) begin
        m0_wready =
            (wr_state[0] == WR_DATA && wr_grant[0] == 1'b0 && s0_wready) ||
            (wr_state[1] == WR_DATA && wr_grant[1] == 1'b0 && s1_wready);
    end

    // ----- Master 1 wready -----
    always @(*) begin
        m1_wready =
            (wr_state[0] == WR_DATA && wr_grant[0] == 1'b1 && s0_wready) ||
            (wr_state[1] == WR_DATA && wr_grant[1] == 1'b1 && s1_wready);
    end

    // ----- Master 0 B channel (bid, bresp, bvalid) -----
    always @(*) begin
        if (wr_state[0] == WR_RESP && wr_grant[0] == 1'b0) begin
            m0_bid = s0_bid; m0_bresp = s0_bresp; m0_bvalid = s0_bvalid;
        end else if (wr_state[1] == WR_RESP && wr_grant[1] == 1'b0) begin
            m0_bid = s1_bid; m0_bresp = s1_bresp; m0_bvalid = s1_bvalid;
        end else begin
            m0_bid = 0; m0_bresp = 2'b00; m0_bvalid = 1'b0;
        end
    end

    // ----- Master 1 B channel -----
    always @(*) begin
        if (wr_state[0] == WR_RESP && wr_grant[0] == 1'b1) begin
            m1_bid = s0_bid; m1_bresp = s0_bresp; m1_bvalid = s0_bvalid;
        end else if (wr_state[1] == WR_RESP && wr_grant[1] == 1'b1) begin
            m1_bid = s1_bid; m1_bresp = s1_bresp; m1_bvalid = s1_bvalid;
        end else begin
            m1_bid = 0; m1_bresp = 2'b00; m1_bvalid = 1'b0;
        end
    end

    // ----- Master 0 arready -----
    always @(*) begin
        m0_arready =
            (rd_state[0] == RD_ADDR && rd_grant[0] == 1'b0 && s0_arready) ||
            (rd_state[1] == RD_ADDR && rd_grant[1] == 1'b0 && s1_arready);
    end

    // ----- Master 1 arready -----
    always @(*) begin
        m1_arready =
            (rd_state[0] == RD_ADDR && rd_grant[0] == 1'b1 && s0_arready) ||
            (rd_state[1] == RD_ADDR && rd_grant[1] == 1'b1 && s1_arready);
    end

    // ----- Master 0 R channel -----
    always @(*) begin
        if (rd_state[0] == RD_DATA && rd_grant[0] == 1'b0) begin
            m0_rid = s0_rid; m0_rdata = s0_rdata; m0_rresp = s0_rresp;
            m0_rlast = s0_rlast; m0_rvalid = s0_rvalid;
        end else if (rd_state[1] == RD_DATA && rd_grant[1] == 1'b0) begin
            m0_rid = s1_rid; m0_rdata = s1_rdata; m0_rresp = s1_rresp;
            m0_rlast = s1_rlast; m0_rvalid = s1_rvalid;
        end else begin
            m0_rid = 0; m0_rdata = 0; m0_rresp = 2'b00;
            m0_rlast = 1'b0; m0_rvalid = 1'b0;
        end
    end

    // ----- Master 1 R channel -----
    always @(*) begin
        if (rd_state[0] == RD_DATA && rd_grant[0] == 1'b1) begin
            m1_rid = s0_rid; m1_rdata = s0_rdata; m1_rresp = s0_rresp;
            m1_rlast = s0_rlast; m1_rvalid = s0_rvalid;
        end else if (rd_state[1] == RD_DATA && rd_grant[1] == 1'b1) begin
            m1_rid = s1_rid; m1_rdata = s1_rdata; m1_rresp = s1_rresp;
            m1_rlast = s1_rlast; m1_rvalid = s1_rvalid;
        end else begin
            m1_rid = 0; m1_rdata = 0; m1_rresp = 2'b00;
            m1_rlast = 1'b0; m1_rvalid = 1'b0;
        end
    end

endmodule
