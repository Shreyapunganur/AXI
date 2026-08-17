`timescale 1ns/1ps
// =====================================================================
// axi4_master.v
// -----------------------------------------------------------------------
// A simple AXI4 master. It turns "commands" from the testbench into
// legal AXI4 channel handshakes. It does not decide what traffic to
// send - axi4_tb_top.sv does that.
//
// THE ONE IDEA THIS FILE USES EVERYWHERE: SLOTS.
//   There are NUM_IDS "slots" (one per possible ID value). A slot is
//   just a small bundle of registers holding one command. The
//   testbench drops a command into slot [cmd_id]; that slot stays
//   "busy" until its response comes back. No queue, no head/tail
//   pointers - "the slot for ID 2" IS the storage for whatever
//   transaction is currently using ID 2.
//
// WHY THIS GIVES US OUT-OF-ORDER TRANSACTIONS FOR FREE:
//   - Issuing (sending AW/AR out) never waits for a response. Every
//     cycle we simply scan the slots low-to-high and issue whichever
//     busy-but-not-yet-issued slot we find first. So slot 3's request
//     can go out while slot 0 is still waiting on its response -
///    that's what "multiple outstanding transactions" means.
//   - Responses are captured completely independently (a separate
//     always block below), whenever B or R arrives, tagged with
//     whatever ID is on the wire. They are free to arrive in ANY
//     order.
//   - The only rule AXI4 requires - "same ID must complete in the
//     order it was issued" - falls out automatically: a slot can't be
//     reused (cmd_ready goes low) until its previous occupant's
//     response has actually arrived. Since a given ID can only ever
//     have ONE live transaction, there's nothing to reorder for that
//     ID in the first place.
// =====================================================================
module axi4_master #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter ID_WIDTH   = 2,             // local ID width -> NUM_IDS slots
    parameter STRB_WIDTH = DATA_WIDTH/8,
    parameter NUM_IDS    = (1 << ID_WIDTH)
)(
    input  wire                   clk,
    input  wire                   rst_n,

    // ---------------- command interface (driven by testbench) ----------
    input  wire                   cmd_valid,
    output wire                   cmd_ready,
    input  wire                   cmd_write,      // 1 = write , 0 = read
    input  wire [ID_WIDTH-1:0]    cmd_id,          // which slot to use
    input  wire [ADDR_WIDTH-1:0]  cmd_addr,
    input  wire [7:0]             cmd_len,        // beats - 1  (AxLEN)
    input  wire [1:0]             cmd_burst,      // FIXED/INCR/WRAP
    input  wire [STRB_WIDTH-1:0]  cmd_wstrb,      // strobes, used every beat
    input  wire [DATA_WIDTH-1:0]  cmd_wseed,      // beat i wdata = wseed + i

    // ---------------- response reporting (to scoreboard/coverage) ------
    // Two separate interfaces, mirroring the two real response channels,
    // so a write completion and a read beat landing on the SAME cycle
    // are both seen (a single shared bus would have to drop one).
    output reg                    rsp_b_valid,
    output reg  [ID_WIDTH-1:0]    rsp_b_id,
    output reg  [1:0]             rsp_b_resp,

    output reg                    rsp_r_valid,
    output reg  [ID_WIDTH-1:0]    rsp_r_id,
    output reg  [1:0]             rsp_r_resp,
    output reg  [DATA_WIDTH-1:0]  rsp_r_data,
    output reg                    rsp_r_last,

    // ---------------- AXI4 write address channel ------------------------
    output reg  [ID_WIDTH-1:0]    m_awid,
    output reg  [ADDR_WIDTH-1:0]  m_awaddr,
    output reg  [7:0]             m_awlen,
    output reg  [2:0]             m_awsize,
    output reg  [1:0]             m_awburst,
    output reg                    m_awvalid,
    input  wire                   m_awready,

    // ---------------- write data channel --------------------------------
    output reg  [DATA_WIDTH-1:0]  m_wdata,
    output reg  [STRB_WIDTH-1:0]  m_wstrb,
    output reg                    m_wlast,
    output reg                    m_wvalid,
    input  wire                   m_wready,

    // ---------------- write response channel -----------------------------
    input  wire [ID_WIDTH-1:0]    m_bid,
    input  wire [1:0]             m_bresp,
    input  wire                   m_bvalid,
    output wire                   m_bready,

    // ---------------- read address channel --------------------------------
    output reg  [ID_WIDTH-1:0]    m_arid,
    output reg  [ADDR_WIDTH-1:0]  m_araddr,
    output reg  [7:0]             m_arlen,
    output reg  [2:0]             m_arsize,
    output reg  [1:0]             m_arburst,
    output reg                    m_arvalid,
    input  wire                   m_arready,

    // ---------------- read data channel ------------------------------------
    input  wire [ID_WIDTH-1:0]    m_rid,
    input  wire [DATA_WIDTH-1:0]  m_rdata,
    input  wire [1:0]             m_rresp,
    input  wire                   m_rlast,
    input  wire                   m_rvalid,
    output wire                   m_rready
);

    localparam [2:0] AXSIZE_WORD = 3'b010;   // every beat is 4 bytes (see README)

    // -----------------------------------------------------------------
    // the slot array
    // -----------------------------------------------------------------
    reg                   slot_busy    [0:NUM_IDS-1];
    reg                   slot_issued  [0:NUM_IDS-1];  // AW/AR already sent?
    reg                   slot_write   [0:NUM_IDS-1];
    reg [ADDR_WIDTH-1:0]  slot_addr    [0:NUM_IDS-1];
    reg [7:0]             slot_len     [0:NUM_IDS-1];
    reg [1:0]             slot_burst   [0:NUM_IDS-1];
    reg [STRB_WIDTH-1:0]  slot_wstrb   [0:NUM_IDS-1];
    reg [DATA_WIDTH-1:0]  slot_wseed   [0:NUM_IDS-1];

    assign cmd_ready = !slot_busy[cmd_id];

    integer i;

    // accept a new command straight into its slot
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < NUM_IDS; i = i + 1)
                slot_busy[i] <= 1'b0;
        end else begin
            // new command claims its slot
            if (cmd_valid && cmd_ready) begin
                slot_busy[cmd_id]  <= 1'b1;
                slot_issued[cmd_id]<= 1'b0;
                slot_write[cmd_id] <= cmd_write;
                slot_addr[cmd_id]  <= cmd_addr;
                slot_len[cmd_id]   <= cmd_len;
                slot_burst[cmd_id] <= cmd_burst;
                slot_wstrb[cmd_id] <= cmd_wstrb;
                slot_wseed[cmd_id] <= cmd_wseed;
            end
            // a response frees its slot (this can freely overlap the
            // accept above as long as it's not the very same ID - and
            // cmd_ready already prevents reusing a still-busy ID)
            if (m_bvalid && m_bready)
                slot_busy[m_bid] <= 1'b0;
            if (m_rvalid && m_rready && m_rlast)
                slot_busy[m_rid] <= 1'b0;
        end
    end

    // -----------------------------------------------------------------
    // issue side: every cycle, find the lowest-numbered slot that is
    // busy but not yet issued, and send its AW/AR out. Simple priority
    // scan - no round-robin needed here, since these are all requests
    // from the SAME generator with no fairness concern between them.
    // -----------------------------------------------------------------
    reg                   pick_valid;
    reg [ID_WIDTH-1:0]    pick_id;
    always @(*) begin
        pick_valid = 1'b0;
        pick_id    = {ID_WIDTH{1'b0}};
        for (i = 0; i < NUM_IDS; i = i + 1) begin
            if (!pick_valid && slot_busy[i] && !slot_issued[i]) begin
                pick_valid = 1'b1;
                pick_id    = i[ID_WIDTH-1:0];
            end
        end
    end

    localparam S_IDLE = 2'd0, S_AW = 2'd1, S_W = 2'd2, S_AR = 2'd3;
    reg [1:0] state;
    reg [7:0] beat;
    reg [ID_WIDTH-1:0] cur_id;   // which slot the AW/W (or AR) phase belongs to

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            m_awvalid <= 1'b0;
            m_wvalid  <= 1'b0;
            m_wlast   <= 1'b0;
            m_arvalid <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    if (pick_valid) begin
                        cur_id <= pick_id;
                        if (slot_write[pick_id]) begin
                            m_awid    <= pick_id;
                            m_awaddr  <= slot_addr[pick_id];
                            m_awlen   <= slot_len[pick_id];
                            m_awsize  <= AXSIZE_WORD;
                            m_awburst <= slot_burst[pick_id];
                            m_awvalid <= 1'b1;
                            state     <= S_AW;
                        end else begin
                            m_arid    <= pick_id;
                            m_araddr  <= slot_addr[pick_id];
                            m_arlen   <= slot_len[pick_id];
                            m_arsize  <= AXSIZE_WORD;
                            m_arburst <= slot_burst[pick_id];
                            m_arvalid <= 1'b1;
                            state     <= S_AR;
                        end
                    end
                end

                S_AW: begin
                    if (m_awvalid && m_awready) begin
                        m_awvalid       <= 1'b0;
                        slot_issued[cur_id] <= 1'b1;
                        // start streaming write data, beat 0
                        beat     <= 8'd0;
                        m_wdata  <= slot_wseed[cur_id];
                        m_wstrb  <= slot_wstrb[cur_id];
                        m_wlast  <= (slot_len[cur_id] == 8'd0);
                        m_wvalid <= 1'b1;
                        state    <= S_W;
                    end
                end

                S_W: begin
                    if (m_wvalid && m_wready) begin
                        if (m_wlast) begin
                            m_wvalid <= 1'b0;
                            m_wlast  <= 1'b0;
                            state    <= S_IDLE;
                        end else begin
                            beat     <= beat + 8'd1;
                            m_wdata  <= slot_wseed[cur_id] + {24'd0, beat + 8'd1};
                            m_wlast  <= ((beat + 8'd1) == slot_len[cur_id]);
                            m_wvalid <= 1'b1;
                        end
                    end
                end

                S_AR: begin
                    if (m_arvalid && m_arready) begin
                        m_arvalid           <= 1'b0;
                        slot_issued[cur_id] <= 1'b1;
                        state                <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // -----------------------------------------------------------------
    // response capture - independent of the issue side above, so B/R
    // can land in any order relative to how requests were issued.
    // Always ready: this master never applies backpressure on responses.
    // -----------------------------------------------------------------
    assign m_bready = 1'b1;
    assign m_rready = 1'b1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rsp_b_valid <= 1'b0;
            rsp_r_valid <= 1'b0;
        end else begin
            rsp_b_valid <= 1'b0;
            rsp_r_valid <= 1'b0;

            if (m_bvalid && m_bready) begin
                rsp_b_valid <= 1'b1;
                rsp_b_id    <= m_bid;
                rsp_b_resp  <= m_bresp;
            end

            if (m_rvalid && m_rready) begin
                rsp_r_valid <= 1'b1;
                rsp_r_id    <= m_rid;
                rsp_r_resp  <= m_rresp;
                rsp_r_data  <= m_rdata;
                rsp_r_last  <= m_rlast;
            end
        end
    end

endmodule
