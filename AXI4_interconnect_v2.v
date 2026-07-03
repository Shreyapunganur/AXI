`timescale 1ns / 1ps

// 2x2 AXI4 Crossbar Interconnect with QoS Arbitration
// Shreya | MNNIT Allahabad | 6th Sem Project
//
// This is the v2 of my basic interconnect. Main additions:
//   1. AWQOS/ARQOS ports on both masters -- priority field
//   2. Arbitration policy: higher QoS wins. Ties go round-robin.
//
// Architecture note: I have one write FSM and one read FSM per slave.
// So S0 has wr_state[0] + rd_state[0], S1 has wr_state[1] + rd_state[1].
// When a transaction comes in, the FSM picks which master gets access
// based on QoS and grants it. The other master just waits.
//
// The crossbar part: M0 and M1 can talk to DIFFERENT slaves simultaneously.
// That's the whole point -- if both go to S0 one waits, but if M0 goes to S0
// and M1 goes to S1 they proceed in parallel. Real bandwidth improvement.
//
// Known limitation: no ID remapping. If two masters use the same AWID value
// and go to the same slave, response routing could get confused. For this
// project both masters use different fixed IDs (1 and 2) so it works.

module AXI4_interconnect_v2 #(
    parameter ADWIDTH = 32,
    parameter DWIDTH  = 64,
    parameter IDWIDTH = 4,
    // address map -- slave 0 and slave 1 address ranges
    parameter [ADWIDTH-1:0] S0_BASE = 32'h0000_0000,
    parameter [ADWIDTH-1:0] S0_HIGH = 32'h0000_07FF,
    parameter [ADWIDTH-1:0] S1_BASE = 32'h0001_0000,
    parameter [ADWIDTH-1:0] S1_HIGH = 32'h0001_07FF
)(
    input wire clk,
    input wire rst,

    // ===== MASTER 0 =====
    input  wire [IDWIDTH-1:0]  m0_awid,
    input  wire [ADWIDTH-1:0]  m0_awaddr,
    input  wire [7:0]          m0_awlen,
    input  wire [2:0]          m0_awsize,
    input  wire [1:0]          m0_awburst,
    input  wire [3:0]          m0_awqos,
    input  wire                m0_awvalid,
    output reg                 m0_awready,

    input  wire [DWIDTH-1:0]   m0_wdata,
    input  wire [DWIDTH/8-1:0] m0_wstrb,
    input  wire                m0_wlast,
    input  wire                m0_wvalid,
    output reg                 m0_wready,

    output reg  [IDWIDTH-1:0]  m0_bid,
    output reg  [1:0]          m0_bresp,
    output reg                 m0_bvalid,
    input  wire                m0_bready,

    input  wire [IDWIDTH-1:0]  m0_arid,
    input  wire [ADWIDTH-1:0]  m0_araddr,
    input  wire [7:0]          m0_arlen,
    input  wire [2:0]          m0_arsize,
    input  wire [1:0]          m0_arburst,
    input  wire [3:0]          m0_arqos,
    input  wire                m0_arvalid,
    output reg                 m0_arready,

    output reg  [IDWIDTH-1:0]  m0_rid,
    output reg  [DWIDTH-1:0]   m0_rdata,
    output reg  [1:0]          m0_rresp,
    output reg                 m0_rlast,
    output reg                 m0_rvalid,
    input  wire                m0_rready,

    // ===== MASTER 1 =====
    input  wire [IDWIDTH-1:0]  m1_awid,
    input  wire [ADWIDTH-1:0]  m1_awaddr,
    input  wire [7:0]          m1_awlen,
    input  wire [2:0]          m1_awsize,
    input  wire [1:0]          m1_awburst,
    input  wire [3:0]          m1_awqos,
    input  wire                m1_awvalid,
    output reg                 m1_awready,

    input  wire [DWIDTH-1:0]   m1_wdata,
    input  wire [DWIDTH/8-1:0] m1_wstrb,
    input  wire                m1_wlast,
    input  wire                m1_wvalid,
    output reg                 m1_wready,

    output reg  [IDWIDTH-1:0]  m1_bid,
    output reg  [1:0]          m1_bresp,
    output reg                 m1_bvalid,
    input  wire                m1_bready,

    input  wire [IDWIDTH-1:0]  m1_arid,
    input  wire [ADWIDTH-1:0]  m1_araddr,
    input  wire [7:0]          m1_arlen,
    input  wire [2:0]          m1_arsize,
    input  wire [1:0]          m1_arburst,
    input  wire [3:0]          m1_arqos,
    input  wire                m1_arvalid,
    output reg                 m1_arready,

    output reg  [IDWIDTH-1:0]  m1_rid,
    output reg  [DWIDTH-1:0]   m1_rdata,
    output reg  [1:0]          m1_rresp,
    output reg                 m1_rlast,
    output reg                 m1_rvalid,
    input  wire                m1_rready,

    // ===== SLAVE 0 =====
    output reg  [IDWIDTH-1:0]  s0_awid,
    output reg  [ADWIDTH-1:0]  s0_awaddr,
    output reg  [7:0]          s0_awlen,
    output reg  [2:0]          s0_awsize,
    output reg  [1:0]          s0_awburst,
    output reg  [3:0]          s0_awqos,
    output reg                 s0_awvalid,
    input  wire                s0_awready,

    output reg  [DWIDTH-1:0]   s0_wdata,
    output reg  [DWIDTH/8-1:0] s0_wstrb,
    output reg                 s0_wlast,
    output reg                 s0_wvalid,
    input  wire                s0_wready,

    input  wire [IDWIDTH-1:0]  s0_bid,
    input  wire [1:0]          s0_bresp,
    input  wire                s0_bvalid,
    output reg                 s0_bready,

    output reg  [IDWIDTH-1:0]  s0_arid,
    output reg  [ADWIDTH-1:0]  s0_araddr,
    output reg  [7:0]          s0_arlen,
    output reg  [2:0]          s0_arsize,
    output reg  [1:0]          s0_arburst,
    output reg  [3:0]          s0_arqos,
    output reg                 s0_arvalid,
    input  wire                s0_arready,

    input  wire [IDWIDTH-1:0]  s0_rid,
    input  wire [DWIDTH-1:0]   s0_rdata,
    input  wire [1:0]          s0_rresp,
    input  wire                s0_rlast,
    input  wire                s0_rvalid,
    output reg                 s0_rready,

    // ===== SLAVE 1 =====
    output reg  [IDWIDTH-1:0]  s1_awid,
    output reg  [ADWIDTH-1:0]  s1_awaddr,
    output reg  [7:0]          s1_awlen,
    output reg  [2:0]          s1_awsize,
    output reg  [1:0]          s1_awburst,
    output reg  [3:0]          s1_awqos,
    output reg                 s1_awvalid,
    input  wire                s1_awready,

    output reg  [DWIDTH-1:0]   s1_wdata,
    output reg  [DWIDTH/8-1:0] s1_wstrb,
    output reg                 s1_wlast,
    output reg                 s1_wvalid,
    input  wire                s1_wready,

    input  wire [IDWIDTH-1:0]  s1_bid,
    input  wire [1:0]          s1_bresp,
    input  wire                s1_bvalid,
    output reg                 s1_bready,

    output reg  [IDWIDTH-1:0]  s1_arid,
    output reg  [ADWIDTH-1:0]  s1_araddr,
    output reg  [7:0]          s1_arlen,
    output reg  [2:0]          s1_arsize,
    output reg  [1:0]          s1_arburst,
    output reg  [3:0]          s1_arqos,
    output reg                 s1_arvalid,
    input  wire                s1_arready,

    input  wire [IDWIDTH-1:0]  s1_rid,
    input  wire [DWIDTH-1:0]   s1_rdata,
    input  wire [1:0]          s1_rresp,
    input  wire                s1_rlast,
    input  wire                s1_rvalid,
    output reg                 s1_rready
);

    // ----------------------------------------------------------------
    // Address decode -- combinational, just compare addr to ranges
    // ----------------------------------------------------------------
    wire m0_wants_s0_wr = m0_awvalid && (m0_awaddr >= S0_BASE) && (m0_awaddr <= S0_HIGH);
    wire m0_wants_s1_wr = m0_awvalid && (m0_awaddr >= S1_BASE) && (m0_awaddr <= S1_HIGH);
    wire m1_wants_s0_wr = m1_awvalid && (m1_awaddr >= S0_BASE) && (m1_awaddr <= S0_HIGH);
    wire m1_wants_s1_wr = m1_awvalid && (m1_awaddr >= S1_BASE) && (m1_awaddr <= S1_HIGH);

    wire m0_wants_s0_rd = m0_arvalid && (m0_araddr >= S0_BASE) && (m0_araddr <= S0_HIGH);
    wire m0_wants_s1_rd = m0_arvalid && (m0_araddr >= S1_BASE) && (m0_araddr <= S1_HIGH);
    wire m1_wants_s0_rd = m1_arvalid && (m1_araddr >= S0_BASE) && (m1_araddr <= S0_HIGH);
    wire m1_wants_s1_rd = m1_arvalid && (m1_araddr >= S1_BASE) && (m1_araddr <= S1_HIGH);

    // ----------------------------------------------------------------
    // FSM states
    // ----------------------------------------------------------------
    localparam [2:0] WR_IDLE=3'd0, WR_ADDR=3'd1, WR_DATA=3'd2, WR_RESP=3'd3;
    localparam [1:0] RD_IDLE=2'd0, RD_ADDR=2'd1, RD_DATA=2'd2;

    // per-slave FSM state and grant
    // grant: 0 = M0 has the slave, 1 = M1 has the slave
    reg [2:0] wr_state [0:1];
    reg       wr_grant [0:1];
    reg       wr_rr    [0:1];  // round-robin priority bit, flips after each contention

    reg [1:0] rd_state [0:1];
    reg       rd_grant [0:1];
    reg       rd_rr    [0:1];

    // ----------------------------------------------------------------
    // Write FSM for Slave 0
    // ----------------------------------------------------------------
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            wr_state[0] <= WR_IDLE;
            wr_grant[0] <= 1'b0;
            wr_rr[0]    <= 1'b0;
        end else begin
            case (wr_state[0])
                WR_IDLE: begin
                    if (m0_wants_s0_wr || m1_wants_s0_wr) begin
                        if (m0_wants_s0_wr && m1_wants_s0_wr) begin
                            // both want S0 -- QoS wins, tie goes round-robin
                            if (m0_awqos > m1_awqos)
                                wr_grant[0] <= 1'b0;
                            else if (m1_awqos > m0_awqos)
                                wr_grant[0] <= 1'b1;
                            else begin
                                wr_grant[0] <= wr_rr[0];
                                wr_rr[0]    <= ~wr_rr[0];
                            end
                        end else
                            wr_grant[0] <= m1_wants_s0_wr ? 1'b1 : 1'b0;

                        wr_state[0] <= WR_ADDR;
                    end
                end
                WR_ADDR: if (s0_awvalid && s0_awready) wr_state[0] <= WR_DATA;
                WR_DATA: if (s0_wvalid && s0_wready && s0_wlast) wr_state[0] <= WR_RESP;
                WR_RESP: if (s0_bvalid && s0_bready) wr_state[0] <= WR_IDLE;
                default:  wr_state[0] <= WR_IDLE;
            endcase
        end
    end

    // Write FSM for Slave 1 -- same logic, different slave
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            wr_state[1] <= WR_IDLE;
            wr_grant[1] <= 1'b0;
            wr_rr[1]    <= 1'b0;
        end else begin
            case (wr_state[1])
                WR_IDLE: begin
                    if (m0_wants_s1_wr || m1_wants_s1_wr) begin
                        if (m0_wants_s1_wr && m1_wants_s1_wr) begin
                            if (m0_awqos > m1_awqos)
                                wr_grant[1] <= 1'b0;
                            else if (m1_awqos > m0_awqos)
                                wr_grant[1] <= 1'b1;
                            else begin
                                wr_grant[1] <= wr_rr[1];
                                wr_rr[1]    <= ~wr_rr[1];
                            end
                        end else
                            wr_grant[1] <= m1_wants_s1_wr ? 1'b1 : 1'b0;

                        wr_state[1] <= WR_ADDR;
                    end
                end
                WR_ADDR: if (s1_awvalid && s1_awready) wr_state[1] <= WR_DATA;
                WR_DATA: if (s1_wvalid && s1_wready && s1_wlast) wr_state[1] <= WR_RESP;
                WR_RESP: if (s1_bvalid && s1_bready) wr_state[1] <= WR_IDLE;
                default:  wr_state[1] <= WR_IDLE;
            endcase
        end
    end

    // Read FSM for Slave 0
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            rd_state[0] <= RD_IDLE;
            rd_grant[0] <= 1'b0;
            rd_rr[0]    <= 1'b0;
        end else begin
            case (rd_state[0])
                RD_IDLE: begin
                    if (m0_wants_s0_rd || m1_wants_s0_rd) begin
                        if (m0_wants_s0_rd && m1_wants_s0_rd) begin
                            if (m0_arqos > m1_arqos)
                                rd_grant[0] <= 1'b0;
                            else if (m1_arqos > m0_arqos)
                                rd_grant[0] <= 1'b1;
                            else begin
                                rd_grant[0] <= rd_rr[0];
                                rd_rr[0]    <= ~rd_rr[0];
                            end
                        end else
                            rd_grant[0] <= m1_wants_s0_rd ? 1'b1 : 1'b0;

                        rd_state[0] <= RD_ADDR;
                    end
                end
                RD_ADDR: if (s0_arvalid && s0_arready) rd_state[0] <= RD_DATA;
                RD_DATA: if (s0_rvalid && s0_rready && s0_rlast) rd_state[0] <= RD_IDLE;
                default:  rd_state[0] <= RD_IDLE;
            endcase
        end
    end

    // Read FSM for Slave 1
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            rd_state[1] <= RD_IDLE;
            rd_grant[1] <= 1'b0;
            rd_rr[1]    <= 1'b0;
        end else begin
            case (rd_state[1])
                RD_IDLE: begin
                    if (m0_wants_s1_rd || m1_wants_s1_rd) begin
                        if (m0_wants_s1_rd && m1_wants_s1_rd) begin
                            if (m0_arqos > m1_arqos)
                                rd_grant[1] <= 1'b0;
                            else if (m1_arqos > m0_arqos)
                                rd_grant[1] <= 1'b1;
                            else begin
                                rd_grant[1] <= rd_rr[1];
                                rd_rr[1]    <= ~rd_rr[1];
                            end
                        end else
                            rd_grant[1] <= m1_wants_s1_rd ? 1'b1 : 1'b0;

                        rd_state[1] <= RD_ADDR;
                    end
                end
                RD_ADDR: if (s1_arvalid && s1_arready) rd_state[1] <= RD_DATA;
                RD_DATA: if (s1_rvalid && s1_rready && s1_rlast) rd_state[1] <= RD_IDLE;
                default:  rd_state[1] <= RD_IDLE;
            endcase
        end
    end

    // ----------------------------------------------------------------
    // Slave-facing muxes -- select which master drives each slave
    // These are combinational, selected by the FSM grant register.
    // ----------------------------------------------------------------

    // S0 AW
    always @(*) begin
        if (wr_state[0]==WR_ADDR && wr_grant[0]==1'b0)
            {s0_awid,s0_awaddr,s0_awlen,s0_awsize,s0_awburst,s0_awqos,s0_awvalid} =
            {m0_awid,m0_awaddr,m0_awlen,m0_awsize,m0_awburst,m0_awqos,m0_awvalid};
        else if (wr_state[0]==WR_ADDR && wr_grant[0]==1'b1)
            {s0_awid,s0_awaddr,s0_awlen,s0_awsize,s0_awburst,s0_awqos,s0_awvalid} =
            {m1_awid,m1_awaddr,m1_awlen,m1_awsize,m1_awburst,m1_awqos,m1_awvalid};
        else
            {s0_awid,s0_awaddr,s0_awlen,s0_awsize,s0_awburst,s0_awqos,s0_awvalid} =
            {(IDWIDTH)'(0),(ADWIDTH)'(0),8'h0,3'b0,2'b0,4'h0,1'b0};
    end

    // S0 W
    always @(*) begin
        if (wr_state[0]==WR_DATA && wr_grant[0]==1'b0)
            {s0_wdata,s0_wstrb,s0_wlast,s0_wvalid} =
            {m0_wdata,m0_wstrb,m0_wlast,m0_wvalid};
        else if (wr_state[0]==WR_DATA && wr_grant[0]==1'b1)
            {s0_wdata,s0_wstrb,s0_wlast,s0_wvalid} =
            {m1_wdata,m1_wstrb,m1_wlast,m1_wvalid};
        else
            {s0_wdata,s0_wstrb,s0_wlast,s0_wvalid} =
            {(DWIDTH)'(0),(DWIDTH/8)'(0),1'b0,1'b0};
    end

    always @(*) s0_bready = (wr_state[0]==WR_RESP) &&
                             (wr_grant[0]==1'b0 ? m0_bready : m1_bready);

    // S1 AW
    always @(*) begin
        if (wr_state[1]==WR_ADDR && wr_grant[1]==1'b0)
            {s1_awid,s1_awaddr,s1_awlen,s1_awsize,s1_awburst,s1_awqos,s1_awvalid} =
            {m0_awid,m0_awaddr,m0_awlen,m0_awsize,m0_awburst,m0_awqos,m0_awvalid};
        else if (wr_state[1]==WR_ADDR && wr_grant[1]==1'b1)
            {s1_awid,s1_awaddr,s1_awlen,s1_awsize,s1_awburst,s1_awqos,s1_awvalid} =
            {m1_awid,m1_awaddr,m1_awlen,m1_awsize,m1_awburst,m1_awqos,m1_awvalid};
        else
            {s1_awid,s1_awaddr,s1_awlen,s1_awsize,s1_awburst,s1_awqos,s1_awvalid} =
            {(IDWIDTH)'(0),(ADWIDTH)'(0),8'h0,3'b0,2'b0,4'h0,1'b0};
    end

    // S1 W
    always @(*) begin
        if (wr_state[1]==WR_DATA && wr_grant[1]==1'b0)
            {s1_wdata,s1_wstrb,s1_wlast,s1_wvalid} =
            {m0_wdata,m0_wstrb,m0_wlast,m0_wvalid};
        else if (wr_state[1]==WR_DATA && wr_grant[1]==1'b1)
            {s1_wdata,s1_wstrb,s1_wlast,s1_wvalid} =
            {m1_wdata,m1_wstrb,m1_wlast,m1_wvalid};
        else
            {s1_wdata,s1_wstrb,s1_wlast,s1_wvalid} =
            {(DWIDTH)'(0),(DWIDTH/8)'(0),1'b0,1'b0};
    end

    always @(*) s1_bready = (wr_state[1]==WR_RESP) &&
                             (wr_grant[1]==1'b0 ? m0_bready : m1_bready);

    // S0 AR
    always @(*) begin
        if (rd_state[0]==RD_ADDR && rd_grant[0]==1'b0)
            {s0_arid,s0_araddr,s0_arlen,s0_arsize,s0_arburst,s0_arqos,s0_arvalid} =
            {m0_arid,m0_araddr,m0_arlen,m0_arsize,m0_arburst,m0_arqos,m0_arvalid};
        else if (rd_state[0]==RD_ADDR && rd_grant[0]==1'b1)
            {s0_arid,s0_araddr,s0_arlen,s0_arsize,s0_arburst,s0_arqos,s0_arvalid} =
            {m1_arid,m1_araddr,m1_arlen,m1_arsize,m1_arburst,m1_arqos,m1_arvalid};
        else
            {s0_arid,s0_araddr,s0_arlen,s0_arsize,s0_arburst,s0_arqos,s0_arvalid} =
            {(IDWIDTH)'(0),(ADWIDTH)'(0),8'h0,3'b0,2'b0,4'h0,1'b0};
    end

    always @(*) s0_rready = (rd_state[0]==RD_DATA) &&
                             (rd_grant[0]==1'b0 ? m0_rready : m1_rready);

    // S1 AR
    always @(*) begin
        if (rd_state[1]==RD_ADDR && rd_grant[1]==1'b0)
            {s1_arid,s1_araddr,s1_arlen,s1_arsize,s1_arburst,s1_arqos,s1_arvalid} =
            {m0_arid,m0_araddr,m0_arlen,m0_arsize,m0_arburst,m0_arqos,m0_arvalid};
        else if (rd_state[1]==RD_ADDR && rd_grant[1]==1'b1)
            {s1_arid,s1_araddr,s1_arlen,s1_arsize,s1_arburst,s1_arqos,s1_arvalid} =
            {m1_arid,m1_araddr,m1_arlen,m1_arsize,m1_arburst,m1_arqos,m1_arvalid};
        else
            {s1_arid,s1_araddr,s1_arlen,s1_arsize,s1_arburst,s1_arqos,s1_arvalid} =
            {(IDWIDTH)'(0),(ADWIDTH)'(0),8'h0,3'b0,2'b0,4'h0,1'b0};
    end

    always @(*) s1_rready = (rd_state[1]==RD_DATA) &&
                             (rd_grant[1]==1'b0 ? m0_rready : m1_rready);

    // ----------------------------------------------------------------
    // Master-facing outputs -- steer awready/wready/bid/rdata back
    // to the correct master based on which slave is serving it
    // ----------------------------------------------------------------

    always @(*) begin
        m0_awready = (wr_state[0]==WR_ADDR && wr_grant[0]==1'b0 && s0_awready) ||
                     (wr_state[1]==WR_ADDR && wr_grant[1]==1'b0 && s1_awready);
        m1_awready = (wr_state[0]==WR_ADDR && wr_grant[0]==1'b1 && s0_awready) ||
                     (wr_state[1]==WR_ADDR && wr_grant[1]==1'b1 && s1_awready);
        m0_wready  = (wr_state[0]==WR_DATA && wr_grant[0]==1'b0 && s0_wready) ||
                     (wr_state[1]==WR_DATA && wr_grant[1]==1'b0 && s1_wready);
        m1_wready  = (wr_state[0]==WR_DATA && wr_grant[0]==1'b1 && s0_wready) ||
                     (wr_state[1]==WR_DATA && wr_grant[1]==1'b1 && s1_wready);
    end

    always @(*) begin
        if (wr_state[0]==WR_RESP && wr_grant[0]==1'b0)
            {m0_bid,m0_bresp,m0_bvalid} = {s0_bid,s0_bresp,s0_bvalid};
        else if (wr_state[1]==WR_RESP && wr_grant[1]==1'b0)
            {m0_bid,m0_bresp,m0_bvalid} = {s1_bid,s1_bresp,s1_bvalid};
        else
            {m0_bid,m0_bresp,m0_bvalid} = {(IDWIDTH)'(0),2'b00,1'b0};

        if (wr_state[0]==WR_RESP && wr_grant[0]==1'b1)
            {m1_bid,m1_bresp,m1_bvalid} = {s0_bid,s0_bresp,s0_bvalid};
        else if (wr_state[1]==WR_RESP && wr_grant[1]==1'b1)
            {m1_bid,m1_bresp,m1_bvalid} = {s1_bid,s1_bresp,s1_bvalid};
        else
            {m1_bid,m1_bresp,m1_bvalid} = {(IDWIDTH)'(0),2'b00,1'b0};
    end

    always @(*) begin
        m0_arready = (rd_state[0]==RD_ADDR && rd_grant[0]==1'b0 && s0_arready) ||
                     (rd_state[1]==RD_ADDR && rd_grant[1]==1'b0 && s1_arready);
        m1_arready = (rd_state[0]==RD_ADDR && rd_grant[0]==1'b1 && s0_arready) ||
                     (rd_state[1]==RD_ADDR && rd_grant[1]==1'b1 && s1_arready);
    end

    always @(*) begin
        if (rd_state[0]==RD_DATA && rd_grant[0]==1'b0)
            {m0_rid,m0_rdata,m0_rresp,m0_rlast,m0_rvalid} =
            {s0_rid,s0_rdata,s0_rresp,s0_rlast,s0_rvalid};
        else if (rd_state[1]==RD_DATA && rd_grant[1]==1'b0)
            {m0_rid,m0_rdata,m0_rresp,m0_rlast,m0_rvalid} =
            {s1_rid,s1_rdata,s1_rresp,s1_rlast,s1_rvalid};
        else
            {m0_rid,m0_rdata,m0_rresp,m0_rlast,m0_rvalid} =
            {(IDWIDTH)'(0),(DWIDTH)'(0),2'b00,1'b0,1'b0};

        if (rd_state[0]==RD_DATA && rd_grant[0]==1'b1)
            {m1_rid,m1_rdata,m1_rresp,m1_rlast,m1_rvalid} =
            {s0_rid,s0_rdata,s0_rresp,s0_rlast,s0_rvalid};
        else if (rd_state[1]==RD_DATA && rd_grant[1]==1'b1)
            {m1_rid,m1_rdata,m1_rresp,m1_rlast,m1_rvalid} =
            {s1_rid,s1_rdata,s1_rresp,s1_rlast,s1_rvalid};
        else
            {m1_rid,m1_rdata,m1_rresp,m1_rlast,m1_rvalid} =
            {(IDWIDTH)'(0),(DWIDTH)'(0),2'b00,1'b0,1'b0};
    end

endmodule
