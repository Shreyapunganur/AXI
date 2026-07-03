`timescale 1ns / 1ps
// =============================================================
// AXI4_coverage.sv — Functional Coverage Collector
//
// 4 Covergroups:
//   cg_write_addr  : awlen bins, awburst, awsize + cross(len x burst)
//   cg_write_resp  : bresp values (OKAY / EXOKAY / SLVERR / DECERR)
//   cg_read_addr   : arlen bins, arburst, arsize + cross(len x burst)
//   cg_read_data   : rresp values + rlast coverage
//
// Instantiate in AXI4_tb_sv.sv alongside the DUT.
// Call $get_coverage() or read the final block output for results.
// =============================================================

module AXI4_coverage #(
    parameter ADWIDTH = 32,
    parameter DWIDTH  = 64,
    parameter IDWIDTH = 4
)(
    input logic                  clk,
    input logic                  rst,
    // AW
    input logic [IDWIDTH-1:0]    awid,
    input logic [ADWIDTH-1:0]    awaddr,
    input logic [7:0]            awlen,
    input logic [2:0]            awsize,
    input logic [1:0]            awburst,
    input logic                  awvalid,
    input logic                  awready,
    // W
    input logic [DWIDTH-1:0]     wdata,
    input logic [DWIDTH/8-1:0]   wstrb,
    input logic                  wlast,
    input logic                  wvalid,
    input logic                  wready,
    // B
    input logic [IDWIDTH-1:0]    bid,
    input logic [1:0]            bresp,
    input logic                  bvalid,
    input logic                  bready,
    // AR
    input logic [IDWIDTH-1:0]    arid,
    input logic [ADWIDTH-1:0]    araddr,
    input logic [7:0]            arlen,
    input logic [2:0]            arsize,
    input logic [1:0]            arburst,
    input logic                  arvalid,
    input logic                  arready,
    // R
    input logic [IDWIDTH-1:0]    rid,
    input logic [DWIDTH-1:0]     rdata,
    input logic [1:0]            rresp,
    input logic                  rlast,
    input logic                  rvalid,
    input logic                  rready
);

    // ----------------------------------------------------------
    // Covergroup: Write Address Channel
    // Sampled on every AW handshake (awvalid && awready)
    // ----------------------------------------------------------
    covergroup cg_write_addr @(posedge clk);
        option.per_instance = 1;
        option.name         = "Write Address Channel";
        option.comment      = "Burst length, type, and size coverage for write transactions";

        // Burst length bins — common AXI4 burst sizes
        cp_awlen : coverpoint awlen iff (rst && awvalid && awready) {
            bins single   = {8'd0};
            bins burst_2  = {8'd1};
            bins burst_4  = {8'd3};
            bins burst_8  = {8'd7};
            bins burst_16 = {8'd15};
            bins other    = default;
        }

        // Burst type: FIXED / INCR / WRAP
        cp_awburst : coverpoint awburst iff (rst && awvalid && awready) {
            bins fixed = {2'b00};
            bins incr  = {2'b01};
            bins wrap  = {2'b10};
        }

        // Transfer size: 1B / 2B / 4B / 8B
        cp_awsize : coverpoint awsize iff (rst && awvalid && awready) {
            bins byte1 = {3'b000};
            bins byte2 = {3'b001};
            bins byte4 = {3'b010};
            bins byte8 = {3'b011};
        }

        // Cross: burst length × burst type
        // Target: all meaningful (len, burst) combos are exercised
        cx_len_x_burst : cross cp_awlen, cp_awburst;

    endgroup

    // ----------------------------------------------------------
    // Covergroup: Write Response Channel
    // ----------------------------------------------------------
    covergroup cg_write_resp @(posedge clk);
        option.name = "Write Response Channel";

        cp_bresp : coverpoint bresp iff (rst && bvalid && bready) {
            bins okay   = {2'b00};   // normal success
            bins exokay = {2'b01};   // exclusive access OK
            bins slverr = {2'b10};   // slave error
            bins decerr = {2'b11};   // decode error (no slave mapped)
        }
    endgroup

    // ----------------------------------------------------------
    // Covergroup: Read Address Channel
    // ----------------------------------------------------------
    covergroup cg_read_addr @(posedge clk);
        option.per_instance = 1;
        option.name         = "Read Address Channel";

        cp_arlen : coverpoint arlen iff (rst && arvalid && arready) {
            bins single   = {8'd0};
            bins burst_2  = {8'd1};
            bins burst_4  = {8'd3};
            bins burst_8  = {8'd7};
            bins burst_16 = {8'd15};
            bins other    = default;
        }

        cp_arburst : coverpoint arburst iff (rst && arvalid && arready) {
            bins fixed = {2'b00};
            bins incr  = {2'b01};
            bins wrap  = {2'b10};
        }

        cp_arsize : coverpoint arsize iff (rst && arvalid && arready) {
            bins byte1 = {3'b000};
            bins byte2 = {3'b001};
            bins byte4 = {3'b010};
            bins byte8 = {3'b011};
        }

        cx_len_x_burst : cross cp_arlen, cp_arburst;

    endgroup

    // ----------------------------------------------------------
    // Covergroup: Read Data Channel
    // ----------------------------------------------------------
    covergroup cg_read_data @(posedge clk);
        option.name = "Read Data Channel";

        cp_rresp : coverpoint rresp iff (rst && rvalid && rready) {
            bins okay   = {2'b00};
            bins exokay = {2'b01};
            bins slverr = {2'b10};
            bins decerr = {2'b11};
        }

        // Track that both mid-burst and last-beat beats are seen
        cp_rlast : coverpoint rlast iff (rst && rvalid && rready) {
            bins mid_beat  = {1'b0};
            bins last_beat = {1'b1};
        }
    endgroup

    // ----------------------------------------------------------
    // Instantiate all covergroups
    // ----------------------------------------------------------
    cg_write_addr cg_wa = new();
    cg_write_resp cg_wr = new();
    cg_read_addr  cg_ra = new();
    cg_read_data  cg_rd = new();

    // ----------------------------------------------------------
    // Coverage summary at end of simulation
    // ----------------------------------------------------------
    final begin
        $display("\n======= FUNCTIONAL COVERAGE REPORT =======");
        $display("  Write Address  : %6.2f%%", cg_wa.get_coverage());
        $display("  Write Response : %6.2f%%", cg_wr.get_coverage());
        $display("  Read  Address  : %6.2f%%", cg_ra.get_coverage());
        $display("  Read  Data     : %6.2f%%", cg_rd.get_coverage());
        $display("  Overall        : %6.2f%%",
            (cg_wa.get_coverage() + cg_wr.get_coverage() +
             cg_ra.get_coverage() + cg_rd.get_coverage()) / 4.0);
        $display("==========================================\n");
    end

endmodule
