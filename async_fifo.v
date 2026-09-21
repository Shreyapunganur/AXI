`timescale 1ns/1ps

module async_fifo #(
    parameter WIDTH  = 32,
    parameter DEPTH  = 8,                 
    parameter ADDR_W = $clog2(DEPTH)
)(
  
    input  wire             wr_clk,
    input  wire             wr_rst_n,
    input  wire             wr_en,
    input  wire [WIDTH-1:0] wr_data,
    output wire             full,

 
    input  wire             rd_clk,
    input  wire             rd_rst_n,
    input  wire             rd_en,
    output wire [WIDTH-1:0] rd_data,
    output wire             empty
);

    reg [WIDTH-1:0] mem [0:DEPTH-1];

   

    reg [ADDR_W:0] wr_bin, wr_gray;
    reg [ADDR_W:0] rd_bin, rd_gray;

    wire [ADDR_W:0] wr_bin_next  = wr_bin + (wr_en && !full);
    wire [ADDR_W:0] wr_gray_next = (wr_bin_next >> 1) ^ wr_bin_next;

    wire [ADDR_W:0] rd_bin_next  = rd_bin + (rd_en && !empty);
    wire [ADDR_W:0] rd_gray_next = (rd_bin_next >> 1) ^ rd_bin_next;

  
    reg [ADDR_W:0] wr_gray_sync1, wr_gray_sync2;  
    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            wr_gray_sync1 <= 0;
            wr_gray_sync2 <= 0;
        end else begin
            wr_gray_sync1 <= wr_gray;
            wr_gray_sync2 <= wr_gray_sync1;
        end
    end

    reg [ADDR_W:0] rd_gray_sync1, rd_gray_sync2;  
    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            rd_gray_sync1 <= 0;
            rd_gray_sync2 <= 0;
        end else begin
            rd_gray_sync1 <= rd_gray;
            rd_gray_sync2 <= rd_gray_sync1;
        end
    end

   
    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wr_bin  <= 0;
            wr_gray <= 0;
        end else begin
            if (wr_en && !full)
                mem[wr_bin[ADDR_W-1:0]] <= wr_data;
            wr_bin  <= wr_bin_next;
            wr_gray <= wr_gray_next;
        end
    end

    
    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            rd_bin  <= 0;
            rd_gray <= 0;
        end else begin
            rd_bin  <= rd_bin_next;
            rd_gray <= rd_gray_next;
        end
    end

    assign rd_data = mem[rd_bin[ADDR_W-1:0]];

   
    assign empty = (rd_gray == wr_gray_sync2);

  
    assign full = (wr_gray == {~rd_gray_sync2[ADDR_W:ADDR_W-1], rd_gray_sync2[ADDR_W-2:0]});

endmodule
