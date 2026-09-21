`timescale 1ns/1ps

module axi4_master #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter ID_WIDTH   = 2,            
    parameter STRB_WIDTH = DATA_WIDTH/8,
    parameter NUM_IDS    = (1 << ID_WIDTH)
)(
    input  wire                   clk,
    input  wire                   rst_n,

   
    input  wire                   cmd_valid,
    output wire                   cmd_ready,
    input  wire                   cmd_write,     
    input  wire [ID_WIDTH-1:0]    cmd_id,         
    input  wire [ADDR_WIDTH-1:0]  cmd_addr,
    input  wire [7:0]             cmd_len,        
    input  wire [1:0]             cmd_burst,      
    input  wire [STRB_WIDTH-1:0]  cmd_wstrb,    
    input  wire [DATA_WIDTH-1:0]  cmd_wseed,      
  
    output reg                    rsp_b_valid,
    output reg  [ID_WIDTH-1:0]    rsp_b_id,
    output reg  [1:0]             rsp_b_resp,

    output reg                    rsp_r_valid,
    output reg  [ID_WIDTH-1:0]    rsp_r_id,
    output reg  [1:0]             rsp_r_resp,
    output reg  [DATA_WIDTH-1:0]  rsp_r_data,
    output reg                    rsp_r_last,

   
    output reg  [ID_WIDTH-1:0]    m_awid,
    output reg  [ADDR_WIDTH-1:0]  m_awaddr,
    output reg  [7:0]             m_awlen,
    output reg  [2:0]             m_awsize,
    output reg  [1:0]             m_awburst,
    output reg                    m_awvalid,
    input  wire                   m_awready,

   
    output reg  [DATA_WIDTH-1:0]  m_wdata,
    output reg  [STRB_WIDTH-1:0]  m_wstrb,
    output reg                    m_wlast,
    output reg                    m_wvalid,
    input  wire                   m_wready,

   
    input  wire [ID_WIDTH-1:0]    m_bid,
    input  wire [1:0]             m_bresp,
    input  wire                   m_bvalid,
    output wire                   m_bready,

   
    output reg  [ID_WIDTH-1:0]    m_arid,
    output reg  [ADDR_WIDTH-1:0]  m_araddr,
    output reg  [7:0]             m_arlen,
    output reg  [2:0]             m_arsize,
    output reg  [1:0]             m_arburst,
    output reg                    m_arvalid,
    input  wire                   m_arready,

    
    input  wire [ID_WIDTH-1:0]    m_rid,
    input  wire [DATA_WIDTH-1:0]  m_rdata,
    input  wire [1:0]             m_rresp,
    input  wire                   m_rlast,
    input  wire                   m_rvalid,
    output wire                   m_rready
);

    localparam [2:0] AXSIZE_WORD = 3'b010;   

  
    reg                   slot_busy    [0:NUM_IDS-1];
    reg                   slot_issued  [0:NUM_IDS-1];  
    reg                   slot_write   [0:NUM_IDS-1];
    reg [ADDR_WIDTH-1:0]  slot_addr    [0:NUM_IDS-1];
    reg [7:0]             slot_len     [0:NUM_IDS-1];
    reg [1:0]             slot_burst   [0:NUM_IDS-1];
    reg [STRB_WIDTH-1:0]  slot_wstrb   [0:NUM_IDS-1];
    reg [DATA_WIDTH-1:0]  slot_wseed   [0:NUM_IDS-1];

    assign cmd_ready = !slot_busy[cmd_id];

    integer i;

   
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < NUM_IDS; i = i + 1)
                slot_busy[i] <= 1'b0;
        end else begin
          
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
           
            if (m_bvalid && m_bready)
                slot_busy[m_bid] <= 1'b0;
            if (m_rvalid && m_rready && m_rlast)
                slot_busy[m_rid] <= 1'b0;
        end
    end

   
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
    reg [ID_WIDTH-1:0] cur_id;   

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
