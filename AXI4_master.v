`timescale 1ns / 1ps

module AXI4_master #(
    parameter adwidth = 32,
    parameter dwidth  = 64,
    parameter idwidth = 4)
(
    input  wire               clk,
    input  wire               rst,

    input  wire               start,
    input  wire               rd_wr,
    input  wire [adwidth-1:0] addrin,
    input  wire [7:0]         burstlen,
    input  wire [dwidth-1:0]  wdin,
    output reg  [dwidth-1:0]  rdout,
    output reg                done,

    output reg  [idwidth-1:0]   awid,
    output reg  [adwidth-1:0]   awaddr,
    output reg  [7:0]           awlen,
    output reg  [2:0]           awsize,
    output reg  [1:0]           awburst,
    output reg                  awvalid,
    input  wire                 awready,

    output reg  [dwidth-1:0]    wdata,
    output reg  [dwidth/8-1:0]  wstrb,
    output reg                  wlast,
    output reg                  wvalid,
    input  wire                 wready,

    input  wire [idwidth-1:0]   bid,
    input  wire [1:0]           bresp,
    input  wire                 bvalid,
    output reg                  bready,

    output reg  [idwidth-1:0]   arid,
    output reg  [adwidth-1:0]   araddr,
    output reg  [7:0]           arlen,
    output reg  [2:0]           arsize,
    output reg  [1:0]           arburst,
    output reg                  arvalid,
    input  wire                 arready,

    input  wire [idwidth-1:0]   rid,
    input  wire [dwidth-1:0]    rdata,
    input  wire [1:0]           rresp,
    input  wire                 rlast,
    input  wire                 rvalid,
    output reg                  rready
);

    localparam [2:0]
        idle   = 3'd0,
        wraddr = 3'd1,
        wrdata = 3'd2,
        wrresp = 3'd3,
        rdaddr = 3'd4,
        rddata = 3'd5,
        tdone  = 3'd6;

    reg [2:0] state;
    reg [7:0] beatcnt;
    reg [7:0] burstlatch;

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            state      <= idle;
            beatcnt    <= 0;
            burstlatch <= 0;
            done       <= 0;
            awid       <= 0; awaddr  <= 0; awlen   <= 0;
            awsize     <= 3'b011;
            awburst    <= 2'b01;
            awvalid    <= 0;
            wdata      <= 0; wstrb   <= 0; wlast   <= 0; wvalid  <= 0;
            bready     <= 0;
            arid       <= 0; araddr  <= 0; arlen   <= 0;
            arsize     <= 3'b011;
            arburst    <= 2'b01;
            arvalid    <= 0;
            rready     <= 0;
            rdout      <= 0;
        end else begin
            done <= 0;

            case (state)

                idle: begin
                    if (start) begin
                        burstlatch <= burstlen;
                        beatcnt    <= 0;

                        if (rd_wr) begin
                            awid    <= 4'h1;
                            awaddr  <= addrin;
                            awlen   <= burstlen;
                            awsize  <= 3'b011;
                            awburst <= 2'b01;
                            awvalid <= 1;
                            state   <= wraddr;
                        end else begin
                            arid    <= 4'h1;
                            araddr  <= addrin;
                            arlen   <= burstlen;
                            arsize  <= 3'b011;
                            arburst <= 2'b01;
                            arvalid <= 1;
                            state   <= rdaddr;
                        end
                    end
                end

                wraddr: begin
                    if (awready && awvalid) begin
                        awvalid <= 0;
                        wdata   <= wdin;
                        wstrb   <= {(dwidth/8){1'b1}};
                        wlast   <= (burstlatch == 0);
                        wvalid  <= 1;
                        state   <= wrdata;
                    end
                end

                wrdata: begin
                    if (wready && wvalid) begin
                        beatcnt <= beatcnt + 1;

                        if (wlast) begin
                            wvalid <= 0;
                            wlast  <= 0;
                            bready <= 1;
                            state  <= wrresp;
                        end else begin
                            wdata <= wdin;
                            wlast <= (beatcnt + 1 == burstlatch);
                        end
                    end
                end

                wrresp: begin
                    if (bvalid && bready) begin
                        bready <= 0;
                        state  <= tdone;
                    end
                end

                rdaddr: begin
                    if (arvalid && arready) begin
                        arvalid <= 0;
                        rready  <= 1;
                        state   <= rddata;
                    end
                end

                rddata: begin
                    if (rvalid && rready) begin
                        rdout   <= rdata;
                        beatcnt <= beatcnt + 1;

                        if (rlast) begin
                            rready <= 0;
                            state  <= tdone;
                        end
                    end
                end

                tdone: begin
                    done  <= 1;
                    state <= idle;
                end

                default: state <= idle;

            endcase
        end
    end

endmodule
