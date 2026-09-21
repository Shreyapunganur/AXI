`timescale 1ns/1ps

module axi4_slave #(
    parameter ADDR_WIDTH  = 32,
    parameter DATA_WIDTH  = 32,
    parameter ID_WIDTH    = 3,
    parameter STRB_WIDTH  = DATA_WIDTH/8,
    parameter MEM_WORDS   = 1024,
    parameter TAB_DEPTH   = 8           
)(
    input  wire                   clk,
    input  wire                   rst_n,

   
    input  wire [ID_WIDTH-1:0]    s_awid,
    input  wire [ADDR_WIDTH-1:0]  s_awaddr,
    input  wire [7:0]             s_awlen,
    input  wire [2:0]             s_awsize,
    input  wire [1:0]             s_awburst,
    input  wire                   s_awvalid,
    output wire                   s_awready,

   
    input  wire [DATA_WIDTH-1:0]  s_wdata,
    input  wire [STRB_WIDTH-1:0]  s_wstrb,
    input  wire                   s_wlast,
    input  wire                   s_wvalid,
    output wire                   s_wready,

    
    output wire [ID_WIDTH-1:0]    s_bid,
    output wire [1:0]             s_bresp,
    output wire                   s_bvalid,
    input  wire                   s_bready,

   
    input  wire [ID_WIDTH-1:0]    s_arid,
    input  wire [ADDR_WIDTH-1:0]  s_araddr,
    input  wire [7:0]             s_arlen,
    input  wire [2:0]             s_arsize,
    input  wire [1:0]             s_arburst,
    input  wire                   s_arvalid,
    output wire                   s_arready,

  
    output wire [ID_WIDTH-1:0]    s_rid,
    output wire [DATA_WIDTH-1:0]  s_rdata,
    output wire [1:0]             s_rresp,
    output wire                   s_rlast,
    output wire                   s_rvalid,
    input  wire                   s_rready
);

    localparam [1:0] RESP_OKAY = 2'b00;
    localparam [1:0] BURST_FIXED = 2'b00;
    localparam [1:0] BURST_WRAP  = 2'b10;

    localparam TAB_IDX_W = $clog2(TAB_DEPTH);  

    reg [DATA_WIDTH-1:0] mem [0:MEM_WORDS-1];

    function [ADDR_WIDTH-1:0] next_addr;
        input [ADDR_WIDTH-1:0] addr;
        input [7:0]            len;    
        input [1:0]            burst;
        reg   [ADDR_WIDTH-1:0] burst_bytes, wrap_base;
        begin
            if (burst == BURST_FIXED) begin
                next_addr = addr;
            end else if (burst == BURST_WRAP) begin
                burst_bytes = 32'd4 * (len + 8'd1);
                wrap_base   = (addr / burst_bytes) * burst_bytes;
                if ((addr + 32'd4) >= (wrap_base + burst_bytes))
                    next_addr = wrap_base;
                else
                    next_addr = addr + 32'd4;
            end else begin // INCR
                next_addr = addr + 32'd4;
            end
        end
    endfunction

 
    function [3:0] rand_delay;
        input dummy;
        reg [31:0] r;
        begin
            r = $random;
            rand_delay = {1'b0, r[2:0]} + 4'd1;   
        end
    endfunction

    integer i, k;

   
    localparam WR_IDLE = 1'b0, WR_BEATS = 1'b1;
    reg                   wr_state;
    reg [ADDR_WIDTH-1:0]  wr_addr;
    reg [7:0]             wr_len;
    reg [1:0]             wr_burst;
    reg [ID_WIDTH-1:0]    wr_id;

    reg btab_has_space;   

    assign s_awready = (wr_state == WR_IDLE) && btab_has_space;
    assign s_wready   = (wr_state == WR_BEATS);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_state <= WR_IDLE;
        end else begin
            case (wr_state)
                WR_IDLE: begin
                    if (s_awvalid && s_awready) begin
                        wr_addr  <= s_awaddr;
                        wr_len   <= s_awlen;
                        wr_burst <= s_awburst;
                        wr_id    <= s_awid;
                        wr_state <= WR_BEATS;
                    end
                end

                WR_BEATS: begin
                    if (s_wvalid && s_wready) begin
                        for (k = 0; k < STRB_WIDTH; k = k + 1)
                            if (s_wstrb[k])
                                mem[wr_addr[11:2]][k*8 +: 8] <= s_wdata[k*8 +: 8];

                        if (s_wlast)
                            wr_state <= WR_IDLE;
                        else
                            wr_addr <= next_addr(wr_addr, wr_len, wr_burst);
                    end
                end
            endcase
        end
    end

   
    reg                busy_b [0:TAB_DEPTH-1];
    reg [ID_WIDTH-1:0] id_b   [0:TAB_DEPTH-1];
    reg [3:0]          delay_b[0:TAB_DEPTH-1];

    always @(*) begin
        btab_has_space = 1'b0;
        for (i = 0; i < TAB_DEPTH; i = i + 1)
            if (!busy_b[i]) btab_has_space = 1'b1;
    end

    
    reg                  b_out_valid;
    reg [TAB_IDX_W-1:0]  b_pick_index;
    integer bp;
    always @(*) begin
        b_out_valid  = 1'b0;
        b_pick_index = 0;
        for (bp = 0; bp < TAB_DEPTH; bp = bp + 1) begin
            if (!b_out_valid && busy_b[bp] && (delay_b[bp] == 4'd0)) begin
                b_out_valid  = 1'b1;
                b_pick_index = bp[TAB_IDX_W-1:0];
            end
        end
    end

    reg found_free_b;   

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < TAB_DEPTH; i = i + 1)
                busy_b[i] <= 1'b0;
        end else begin
           
            if (s_wvalid && s_wready && s_wlast) begin
                found_free_b = 1'b0;
                for (i = 0; i < TAB_DEPTH; i = i + 1) begin
                    if (!found_free_b && !busy_b[i]) begin
                        busy_b[i]  <= 1'b1;
                        id_b[i]    <= wr_id;
                        delay_b[i] <= rand_delay(1'b0);
                        found_free_b = 1'b1;
                    end
                end
            end

           
            for (i = 0; i < TAB_DEPTH; i = i + 1)
                if (busy_b[i] && delay_b[i] != 4'd0)
                    delay_b[i] <= delay_b[i] - 4'd1;

           
            if (s_bvalid && s_bready)
                busy_b[b_pick_index] <= 1'b0;
        end
    end

    
    assign s_bvalid = b_out_valid;
    assign s_bid    = id_b[b_pick_index];
    assign s_bresp  = RESP_OKAY;

   
    reg                   busy_r [0:TAB_DEPTH-1];
    reg [ID_WIDTH-1:0]    id_r   [0:TAB_DEPTH-1];
    reg [ADDR_WIDTH-1:0]  addr_r [0:TAB_DEPTH-1];  
    reg [7:0]             len_r  [0:TAB_DEPTH-1];
    reg [1:0]             burst_r[0:TAB_DEPTH-1];

    reg rdtab_has_space;
    always @(*) begin
        rdtab_has_space = 1'b0;
        for (i = 0; i < TAB_DEPTH; i = i + 1)
            if (!busy_r[i]) rdtab_has_space = 1'b1;
    end

    assign s_arready = rdtab_has_space;

    reg [3:0] delay_r[0:TAB_DEPTH-1];
    reg found_free_r;

    
    wire r_release = r_active && s_rready && r_this_is_last;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < TAB_DEPTH; i = i + 1)
                busy_r[i] <= 1'b0;
        end else begin
           
            if (s_arvalid && s_arready) begin
                found_free_r = 1'b0;
                for (i = 0; i < TAB_DEPTH; i = i + 1) begin
                    if (!found_free_r && !busy_r[i]) begin
                        busy_r[i]   <= 1'b1;
                        id_r[i]     <= s_arid;
                        addr_r[i]   <= s_araddr;
                        len_r[i]    <= s_arlen;
                        burst_r[i]  <= s_arburst;
                        delay_r[i]  <= rand_delay(1'b0);
                        found_free_r = 1'b1;
                    end
                end
            end

            
            if (r_release)
                busy_r[r_slot] <= 1'b0;

            for (i = 0; i < TAB_DEPTH; i = i + 1)
                if (busy_r[i] && delay_r[i] != 4'd0)
                    delay_r[i] <= delay_r[i] - 4'd1;
        end
    end

  
    reg                  r_cand_valid;
    reg [TAB_IDX_W-1:0]  r_cand_index;
    integer rp;
    always @(*) begin
        r_cand_valid = 1'b0;
        r_cand_index = 0;
        for (rp = 0; rp < TAB_DEPTH; rp = rp + 1) begin
            if (!r_cand_valid && busy_r[rp] && (delay_r[rp] == 4'd0)) begin
                r_cand_valid = 1'b1;
                r_cand_index = rp[TAB_IDX_W-1:0];
            end
        end
    end

    reg                  r_active;       
    reg [TAB_IDX_W-1:0]  r_slot;         
    reg [7:0]            r_beat;         

    wire r_this_is_last = (r_beat == len_r[r_slot]);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_active <= 1'b0;
        end else begin
            if (!r_active) begin
                if (r_cand_valid) begin
                    r_active <= 1'b1;
                    r_slot   <= r_cand_index;
                    r_beat   <= 8'd0;
                end
            end else begin
                if (s_rready) begin         
                    if (r_this_is_last) begin
                        r_active <= 1'b0;
                    end else begin
                        addr_r[r_slot] <= next_addr(addr_r[r_slot], len_r[r_slot], burst_r[r_slot]);
                        r_beat         <= r_beat + 8'd1;
                    end
                end
            end
        end
    end

    assign s_rvalid = r_active;
    assign s_rid    = id_r[r_slot];
    assign s_rdata  = mem[addr_r[r_slot][11:2]];
    assign s_rresp  = RESP_OKAY;
    assign s_rlast  = r_this_is_last;

endmodule
