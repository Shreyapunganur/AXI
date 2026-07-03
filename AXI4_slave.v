`timescale 1ns / 1ps

module AXI4_slave #(
    parameter adwidth = 32,
    parameter dwidth  = 64,
    parameter idwidth = 4,
    parameter mdepth  = 16)
(
    input  wire               clk,
    input  wire               rst,

    input  wire [idwidth-1:0]   awid,
    input  wire [adwidth-1:0]   awaddr,
    input  wire [7:0]           awlen,
    input  wire [2:0]           awsize,
    input  wire [1:0]           awburst,
    input  wire                 awvalid,
    output reg                  awready,

    input  wire [dwidth-1:0]    wdata,
    input  wire [dwidth/8-1:0]  wstrb,
    input  wire                 wlast,
    input  wire                 wvalid,
    output reg                  wready,

    output reg  [idwidth-1:0]   bid,
    output reg  [1:0]           bresp,
    output reg                  bvalid,
    input  wire                 bready,

    input  wire [idwidth-1:0]   arid,
    input  wire [adwidth-1:0]   araddr,
    input  wire [7:0]           arlen,
    input  wire [2:0]           arsize,
    input  wire [1:0]           arburst,
    input  wire                 arvalid,
    output reg                  arready,

    output reg  [idwidth-1:0]   rid,
    output reg  [dwidth-1:0]    rdata,
    output reg  [1:0]           rresp,
    output wire                 rlast,
    output reg                  rvalid,
    input  wire                 rready
);

    reg [dwidth-1:0] mem [0:mdepth-1];
    integer i;
    initial for (i = 0; i < mdepth; i = i+1) mem[i] = 0;

    localparam [1:0]
        wridle = 2'd0,
        wrdata = 2'd1,
        wrresp = 2'd2;

    reg [1:0]         wrstate;
    reg [idwidth-1:0] wrid;
    reg [adwidth-1:0] wraddr;
    reg [7:0]         wrlen;
    reg [2:0]         wrsize;
    reg [1:0]         wrburst;

    wire [$clog2(mdepth)-1:0] wridx = wraddr[$clog2(mdepth)+2:3];

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            wrstate <= wridle;
            awready <= 0;
            wready  <= 0;
            bvalid  <= 0;
            bresp   <= 0;
            bid     <= 0;
            wraddr  <= 0;
            wrlen   <= 0;
        end else begin
            case (wrstate)

                wridle: begin
                    awready <= 1;
                    wready  <= 0;
                    bvalid  <= 0;

                    if (awvalid && awready) begin
                        wrid    <= awid;
                        wraddr  <= awaddr;
                        wrlen   <= awlen;
                        wrsize  <= awsize;
                        wrburst <= awburst;
                        awready <= 0;
                        wready  <= 1;
                        wrstate <= wrdata;
                    end
                end

                wrdata: begin
                    if (wready && wvalid) begin
                        begin : do_write
                            integer b;
                            for (b = 0; b < (dwidth/8); b = b+1)
                                if (wstrb[b])
                                    mem[wridx][b*8 +: 8] <= wdata[b*8 +: 8];
                        end

                        if (wrburst == 2'b01)
                            wraddr <= wraddr + (1 << wrsize);

                        if (wlast) begin
                            wready  <= 0;
                            bvalid  <= 1;
                            bresp   <= 2'b00;
                            bid     <= wrid;
                            wrstate <= wrresp;
                        end
                    end
                end

                wrresp: begin
                    if (bvalid && bready) begin
                        bvalid  <= 0;
                        wrstate <= wridle;
                    end
                end

                default: wrstate <= wridle;

            endcase
        end
    end

    localparam [1:0]
        rdidle = 2'd0,
        rddata = 2'd1;

    reg [1:0]         rdstate;
    reg [idwidth-1:0] rdid;
    reg [adwidth-1:0] rdaddr;
    reg [7:0]         rdlen;
    reg [2:0]         rdsize;
    reg [1:0]         rdburst;
    reg [7:0]         rdbeat;

    wire [$clog2(mdepth)-1:0] rdidx = rdaddr[$clog2(mdepth)+2:3];

    assign rlast = (rdstate == rddata) && (rdbeat == rdlen);

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            rdstate <= rdidle;
            arready <= 0;
            rvalid  <= 0;
            rresp   <= 0;
            rid     <= 0;
            rdaddr  <= 0;
            rdbeat  <= 0;
        end else begin
            case (rdstate)

                rdidle: begin
                    arready <= 1;
                    rvalid  <= 0;

                    if (arvalid && arready) begin
                        rdid    <= arid;
                        rdaddr  <= araddr;
                        rdlen   <= arlen;
                        rdsize  <= arsize;
                        rdburst <= arburst;
                        rdbeat  <= 0;
                        arready <= 0;
                        rdstate <= rddata;
                    end
                end

                rddata: begin
                    rvalid <= 1;
                    rdata  <= mem[rdidx];
                    rresp  <= 2'b00;
                    rid    <= rdid;

                    if (rvalid && rready) begin
                        if (rdburst == 2'b01)
                            rdaddr <= rdaddr + (1 << rdsize);

                        if (rdbeat == rdlen) begin
                            rvalid  <= 0;
                            rdstate <= rdidle;
                        end else begin
                            rdbeat <= rdbeat + 1;
                        end
                    end
                end

                default: rdstate <= rdidle;

            endcase
        end
    end

endmodule
