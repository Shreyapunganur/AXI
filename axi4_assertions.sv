`timescale 1ns/1ps

module axi4_assertions #(
    parameter ID_WIDTH   = 2,
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
)(
    input wire                   clk,
    input wire                   rst_n,

    input wire                   awvalid,
    input wire                   awready,
    input wire [ADDR_WIDTH-1:0]  awaddr,

    input wire                   wvalid,
    input wire                   wready,
    input wire                   wlast,

    input wire                   bvalid,
    input wire                   bready,

    input wire                   arvalid,
    input wire                   arready,
    input wire [ADDR_WIDTH-1:0]  araddr,

    input wire                   rvalid,
    input wire                   rready,
    input wire [ID_WIDTH-1:0]    rid,
    input wire                   rlast
);

    property p_valid_holds(valid, ready);
        @(posedge clk) disable iff (!rst_n)
        valid && !ready |=> valid;
    endproperty

    assert property (p_valid_holds(awvalid, awready))
        else $error("AWVALID dropped before AWREADY");

    assert property (p_valid_holds(wvalid, wready))
        else $error("WVALID dropped before WREADY");

    assert property (p_valid_holds(bvalid, bready))
        else $error("BVALID dropped before BREADY");

    assert property (p_valid_holds(arvalid, arready))
        else $error("ARVALID dropped before ARREADY");

    assert property (p_valid_holds(rvalid, rready))
        else $error("RVALID dropped before RREADY");


    property p_addr_stable(valid, ready, addr);
        @(posedge clk) disable iff (!rst_n)
        valid && !ready |=> $stable(addr);
    endproperty

    assert property (p_addr_stable(awvalid, awready, awaddr))
        else $error("AWADDR changed while AWVALID was pending");

    assert property (p_addr_stable(arvalid, arready, araddr))
        else $error("ARADDR changed while ARVALID was pending");


    reg                mid_burst;
    reg [ID_WIDTH-1:0] mid_burst_id;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mid_burst <= 1'b0;
        end
        else if (rvalid && rready) begin
            if (rlast)
                mid_burst <= 1'b0;
            else begin
                mid_burst    <= 1'b1;
                mid_burst_id <= rid;
            end
        end
    end

    property p_no_read_interleave;
        @(posedge clk) disable iff (!rst_n)
        (mid_burst && rvalid && rready) |-> (rid == mid_burst_id);
    endproperty

    assert property (p_no_read_interleave)
        else $error("Read data interleaved: RID changed before RLAST");

endmodule


bind axi4_master axi4_assertions #(
    .ID_WIDTH(ID_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH),
    .DATA_WIDTH(DATA_WIDTH)
) u_axi4_assertions (
    .clk(clk),
    .rst_n(rst_n),

    .awvalid(m_awvalid),
    .awready(m_awready),
    .awaddr(m_awaddr),

    .wvalid(m_wvalid),
    .wready(m_wready),
    .wlast(m_wlast),

    .bvalid(m_bvalid),
    .bready(m_bready),

    .arvalid(m_arvalid),
    .arready(m_arready),
    .araddr(m_araddr),

    .rvalid(m_rvalid),
    .rready(m_rready),
    .rid(m_rid),
    .rlast(m_rlast)
);
