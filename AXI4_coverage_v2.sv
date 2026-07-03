`timescale 1ns/1ps
// =============================================================
// AXI4_coverage_v2.sv  —  Extended Functional Coverage
//
// Covergroups:
//   cg_m0_write_addr  : M0 AW channel — len/burst/size bins
//   cg_m1_write_addr  : M1 AW channel
//   cg_write_resp     : B channel response codes
//   cg_m0_read_addr   : M0 AR channel
//   cg_m1_read_addr   : M1 AR channel
//   cg_routing        : master × slave routing matrix
//   cg_contention     : same-slave contention events
//   cg_concurrency    : different-slave concurrent events
//   cg_backpressure   : stall duration bins per channel
//   cg_qos            : QoS value distribution
//   cg_cdc            : CDC-path traffic
//   cg_pipeline       : pipelined (AW+W simultaneous) traffic
// =============================================================

module AXI4_coverage_v2 #(
    parameter ADWIDTH = 32,
    parameter DWIDTH  = 64,
    parameter IDWIDTH = 4,
    parameter [ADWIDTH-1:0] S0_BASE = 32'h0000_0000,
    parameter [ADWIDTH-1:0] S0_HIGH = 32'h0000_07FF,
    parameter [ADWIDTH-1:0] S1_BASE = 32'h0001_0000,
    parameter [ADWIDTH-1:0] S1_HIGH = 32'h0001_07FF
)(
    input logic clk,
    input logic rst,

    // Master 0
    input logic [ADWIDTH-1:0]  m0_awaddr,
    input logic [7:0]          m0_awlen,
    input logic [2:0]          m0_awsize,
    input logic [1:0]          m0_awburst,
    input logic [3:0]          m0_awqos,
    input logic                m0_awvalid, m0_awready,
    input logic                m0_wvalid,  m0_wready,
    input logic [1:0]          m0_bresp,
    input logic                m0_bvalid,  m0_bready,
    input logic [ADWIDTH-1:0]  m0_araddr,
    input logic [7:0]          m0_arlen,
    input logic [2:0]          m0_arsize,
    input logic [1:0]          m0_arburst,
    input logic [3:0]          m0_arqos,
    input logic                m0_arvalid, m0_arready,
    input logic [1:0]          m0_rresp,
    input logic                m0_rvalid,  m0_rready,  m0_rlast,

    // Master 1
    input logic [ADWIDTH-1:0]  m1_awaddr,
    input logic [7:0]          m1_awlen,
    input logic [2:0]          m1_awsize,
    input logic [1:0]          m1_awburst,
    input logic [3:0]          m1_awqos,
    input logic                m1_awvalid, m1_awready,
    input logic                m1_wvalid,  m1_wready,
    input logic [1:0]          m1_bresp,
    input logic                m1_bvalid,  m1_bready,
    input logic [ADWIDTH-1:0]  m1_araddr,
    input logic [7:0]          m1_arlen,
    input logic [2:0]          m1_arsize,
    input logic [1:0]          m1_arburst,
    input logic [3:0]          m1_arqos,
    input logic                m1_arvalid, m1_arready,
    input logic [1:0]          m1_rresp,
    input logic                m1_rvalid,  m1_rready,  m1_rlast,

    // CDC traffic indicator
    input logic                cdc_active   // 1 when CDC bridge is transferring
);

    // ----------------------------------------------------------
    // Derived routing signals
    // ----------------------------------------------------------
    wire m0_wr_s0 = m0_awvalid && m0_awready &&
                    (m0_awaddr >= S0_BASE) && (m0_awaddr <= S0_HIGH);
    wire m0_wr_s1 = m0_awvalid && m0_awready &&
                    (m0_awaddr >= S1_BASE) && (m0_awaddr <= S1_HIGH);
    wire m1_wr_s0 = m1_awvalid && m1_awready &&
                    (m1_awaddr >= S0_BASE) && (m1_awaddr <= S0_HIGH);
    wire m1_wr_s1 = m1_awvalid && m1_awready &&
                    (m1_awaddr >= S1_BASE) && (m1_awaddr <= S1_HIGH);

    wire m0_rd_s0 = m0_arvalid && m0_arready &&
                    (m0_araddr >= S0_BASE) && (m0_araddr <= S0_HIGH);
    wire m0_rd_s1 = m0_arvalid && m0_arready &&
                    (m0_araddr >= S1_BASE) && (m0_araddr <= S1_HIGH);
    wire m1_rd_s0 = m1_arvalid && m1_arready &&
                    (m1_araddr >= S0_BASE) && (m1_araddr <= S0_HIGH);
    wire m1_rd_s1 = m1_arvalid && m1_arready &&
                    (m1_araddr >= S1_BASE) && (m1_araddr <= S1_HIGH);

    // Contention: both masters request same slave in same cycle
    wire wr_contend_s0 = m0_awvalid && m1_awvalid &&
                         (m0_awaddr >= S0_BASE) && (m0_awaddr <= S0_HIGH) &&
                         (m1_awaddr >= S0_BASE) && (m1_awaddr <= S0_HIGH);
    wire wr_contend_s1 = m0_awvalid && m1_awvalid &&
                         (m0_awaddr >= S1_BASE) && (m0_awaddr <= S1_HIGH) &&
                         (m1_awaddr >= S1_BASE) && (m1_awaddr <= S1_HIGH);

    // Concurrency: both masters active on different slaves
    wire concurrent_diff = (m0_awvalid || m0_arvalid) &&
                            (m1_awvalid || m1_arvalid) &&
                            !wr_contend_s0 && !wr_contend_s1;

    // Pipeline: AW and W asserted same cycle (pipelined master signature)
    wire m0_pipeline = m0_awvalid && m0_wvalid;
    wire m1_pipeline = m1_awvalid && m1_wvalid;

    // Backpressure counters (saturating at 15 for bin purposes)
    logic [3:0] bp_m0_aw, bp_m0_w, bp_m0_ar, bp_m0_r;
    logic [3:0] bp_m1_aw, bp_m1_w, bp_m1_ar, bp_m1_r;

    always_ff @(posedge clk) begin
        if (!rst) begin
            bp_m0_aw <= 0; bp_m0_w <= 0; bp_m0_ar <= 0; bp_m0_r <= 0;
            bp_m1_aw <= 0; bp_m1_w <= 0; bp_m1_ar <= 0; bp_m1_r <= 0;
        end else begin
            bp_m0_aw <= (m0_awvalid && !m0_awready && bp_m0_aw < 15) ? bp_m0_aw + 1 :
                        (m0_awvalid &&  m0_awready)                   ? 4'd0 : bp_m0_aw;
            bp_m0_w  <= (m0_wvalid  && !m0_wready  && bp_m0_w  < 15) ? bp_m0_w  + 1 :
                        (m0_wvalid  &&  m0_wready)                    ? 4'd0 : bp_m0_w;
            bp_m0_ar <= (m0_arvalid && !m0_arready && bp_m0_ar < 15) ? bp_m0_ar + 1 :
                        (m0_arvalid &&  m0_arready)                   ? 4'd0 : bp_m0_ar;
            bp_m0_r  <= (m0_rvalid  && !m0_rready  && bp_m0_r  < 15) ? bp_m0_r  + 1 :
                        (m0_rvalid  &&  m0_rready)                    ? 4'd0 : bp_m0_r;

            bp_m1_aw <= (m1_awvalid && !m1_awready && bp_m1_aw < 15) ? bp_m1_aw + 1 :
                        (m1_awvalid &&  m1_awready)                   ? 4'd0 : bp_m1_aw;
            bp_m1_w  <= (m1_wvalid  && !m1_wready  && bp_m1_w  < 15) ? bp_m1_w  + 1 :
                        (m1_wvalid  &&  m1_wready)                    ? 4'd0 : bp_m1_w;
            bp_m1_ar <= (m1_arvalid && !m1_arready && bp_m1_ar < 15) ? bp_m1_ar + 1 :
                        (m1_arvalid &&  m1_arready)                   ? 4'd0 : bp_m1_ar;
            bp_m1_r  <= (m1_rvalid  && !m1_rready  && bp_m1_r  < 15) ? bp_m1_r  + 1 :
                        (m1_rvalid  &&  m1_rready)                    ? 4'd0 : bp_m1_r;
        end
    end

    // ----------------------------------------------------------
    // CG1: Master 0 Write Address
    // ----------------------------------------------------------
    covergroup cg_m0_write_addr @(posedge clk);
        option.name = "M0 Write Address";

        cp_len: coverpoint m0_awlen iff (rst && m0_awvalid && m0_awready) {
            bins single   = {8'd0};
            bins burst_2  = {8'd1};
            bins burst_4  = {8'd3};
            bins burst_8  = {8'd7};
            bins burst_16 = {8'd15};
            bins other    = default;
        }
        cp_burst: coverpoint m0_awburst iff (rst && m0_awvalid && m0_awready) {
            bins fixed = {2'b00};
            bins incr  = {2'b01};
            bins wrap  = {2'b10};
        }
        cp_size: coverpoint m0_awsize iff (rst && m0_awvalid && m0_awready) {
            bins byte1 = {3'b000};
            bins byte2 = {3'b001};
            bins byte4 = {3'b010};
            bins byte8 = {3'b011};
        }
        cx_len_x_burst: cross cp_len, cp_burst;
    endgroup

    // ----------------------------------------------------------
    // CG2: Master 1 Write Address
    // ----------------------------------------------------------
    covergroup cg_m1_write_addr @(posedge clk);
        option.name = "M1 Write Address";

        cp_len: coverpoint m1_awlen iff (rst && m1_awvalid && m1_awready) {
            bins single   = {8'd0};
            bins burst_2  = {8'd1};
            bins burst_4  = {8'd3};
            bins burst_8  = {8'd7};
            bins burst_16 = {8'd15};
            bins other    = default;
        }
        cp_burst: coverpoint m1_awburst iff (rst && m1_awvalid && m1_awready) {
            bins fixed = {2'b00};
            bins incr  = {2'b01};
            bins wrap  = {2'b10};
        }
        cx_len_x_burst: cross cp_len, cp_burst;
    endgroup

    // ----------------------------------------------------------
    // CG3: Write Response
    // ----------------------------------------------------------
    covergroup cg_write_resp @(posedge clk);
        option.name = "Write Response";
        cp_m0_bresp: coverpoint m0_bresp iff (rst && m0_bvalid && m0_bready) {
            bins okay   = {2'b00};
            bins exokay = {2'b01};
            bins slverr = {2'b10};
            bins decerr = {2'b11};
        }
        cp_m1_bresp: coverpoint m1_bresp iff (rst && m1_bvalid && m1_bready) {
            bins okay   = {2'b00};
            bins exokay = {2'b01};
            bins slverr = {2'b10};
            bins decerr = {2'b11};
        }
    endgroup

    // ----------------------------------------------------------
    // CG4: Master 0 Read Address
    // ----------------------------------------------------------
    covergroup cg_m0_read_addr @(posedge clk);
        option.name = "M0 Read Address";
        cp_len: coverpoint m0_arlen iff (rst && m0_arvalid && m0_arready) {
            bins single   = {8'd0};
            bins burst_2  = {8'd1};
            bins burst_4  = {8'd3};
            bins burst_8  = {8'd7};
            bins burst_16 = {8'd15};
            bins other    = default;
        }
        cp_burst: coverpoint m0_arburst iff (rst && m0_arvalid && m0_arready) {
            bins fixed = {2'b00};
            bins incr  = {2'b01};
            bins wrap  = {2'b10};
        }
        cx_len_x_burst: cross cp_len, cp_burst;
    endgroup

    // ----------------------------------------------------------
    // CG5: Master 1 Read Address
    // ----------------------------------------------------------
    covergroup cg_m1_read_addr @(posedge clk);
        option.name = "M1 Read Address";
        cp_len: coverpoint m1_arlen iff (rst && m1_arvalid && m1_arready) {
            bins single   = {8'd0};
            bins burst_2  = {8'd1};
            bins burst_4  = {8'd3};
            bins burst_8  = {8'd7};
            bins burst_16 = {8'd15};
            bins other    = default;
        }
        cp_burst: coverpoint m1_arburst iff (rst && m1_arvalid && m1_arready) {
            bins fixed = {2'b00};
            bins incr  = {2'b01};
            bins wrap  = {2'b10};
        }
        cx_len_x_burst: cross cp_len, cp_burst;
    endgroup

    // ----------------------------------------------------------
    // CG6: Routing Matrix  (both masters × both slaves)
    // ----------------------------------------------------------
    covergroup cg_routing @(posedge clk);
        option.name = "Master-Slave Routing";

        cp_wr_route: coverpoint {m0_wr_s0, m0_wr_s1, m1_wr_s0, m1_wr_s1}
            iff (rst) {
            bins m0_to_s0 = {4'b1000};
            bins m0_to_s1 = {4'b0100};
            bins m1_to_s0 = {4'b0010};
            bins m1_to_s1 = {4'b0001};
        }
        cp_rd_route: coverpoint {m0_rd_s0, m0_rd_s1, m1_rd_s0, m1_rd_s1}
            iff (rst) {
            bins m0_from_s0 = {4'b1000};
            bins m0_from_s1 = {4'b0100};
            bins m1_from_s0 = {4'b0010};
            bins m1_from_s1 = {4'b0001};
        }
    endgroup

    // ----------------------------------------------------------
    // CG7: Same-slave contention
    // ----------------------------------------------------------
    covergroup cg_contention @(posedge clk);
        option.name = "Same-Slave Contention";

        cp_wr_contend: coverpoint {wr_contend_s0, wr_contend_s1} iff (rst) {
            bins both_want_s0  = {2'b10};
            bins both_want_s1  = {2'b01};
            bins no_contention = {2'b00};
        }
    endgroup

    // ----------------------------------------------------------
    // CG8: Different-slave concurrency
    // ----------------------------------------------------------
    covergroup cg_concurrency @(posedge clk);
        option.name = "Different-Slave Concurrency";

        cp_concurrent: coverpoint concurrent_diff iff (rst) {
            bins concurrent = {1'b1};
            bins serial     = {1'b0};
        }
        cp_both_wr: coverpoint (m0_awvalid && m1_awvalid &&
                                !wr_contend_s0 && !wr_contend_s1) iff (rst) {
            bins both_writing_diff = {1'b1};
        }
        cp_both_rd: coverpoint (m0_arvalid && m1_arvalid) iff (rst) {
            bins both_reading = {1'b1};
        }
    endgroup

    // ----------------------------------------------------------
    // CG9: Backpressure duration bins
    // ----------------------------------------------------------
    covergroup cg_backpressure @(posedge clk);
        option.name = "Backpressure Duration";

        cp_m0_aw_bp: coverpoint bp_m0_aw iff (rst && m0_awvalid && !m0_awready) {
            bins short  = {[1:2]};
            bins bp_med = {[3:7]};
            bins long   = {[8:15]};
        }
        cp_m0_w_bp: coverpoint bp_m0_w iff (rst && m0_wvalid && !m0_wready) {
            bins short  = {[1:2]};
            bins bp_med = {[3:7]};
            bins long   = {[8:15]};
        }
        cp_m1_aw_bp: coverpoint bp_m1_aw iff (rst && m1_awvalid && !m1_awready) {
            bins short  = {[1:2]};
            bins bp_med = {[3:7]};
            bins long   = {[8:15]};
        }
        cp_m0_r_bp: coverpoint bp_m0_r iff (rst && m0_rvalid && !m0_rready) {
            bins short  = {[1:2]};
            bins bp_med = {[3:7]};
            bins long   = {[8:15]};
        }
    endgroup

    // ----------------------------------------------------------
    // CG10: QoS field value distribution
    // ----------------------------------------------------------
    covergroup cg_qos @(posedge clk);
        option.name = "QoS Field Values";

        cp_m0_awqos: coverpoint m0_awqos iff (rst && m0_awvalid && m0_awready) {
            bins low    = {[4'h0:4'h3]};
            bins mid    = {[4'h4:4'h7]};
            bins high   = {[4'h8:4'hB]};
            bins urgent = {[4'hC:4'hF]};
        }
        cp_m1_awqos: coverpoint m1_awqos iff (rst && m1_awvalid && m1_awready) {
            bins low    = {[4'h0:4'h3]};
            bins mid    = {[4'h4:4'h7]};
            bins high   = {[4'h8:4'hB]};
            bins urgent = {[4'hC:4'hF]};
        }
        cp_m0_arqos: coverpoint m0_arqos iff (rst && m0_arvalid && m0_arready) {
            bins low    = {[4'h0:4'h3]};
            bins mid    = {[4'h4:4'h7]};
            bins high   = {[4'h8:4'hB]};
            bins urgent = {[4'hC:4'hF]};
        }
        // QoS arbitration won by higher value
        cp_qos_contend: coverpoint {m0_awvalid, m1_awvalid,
                                     (m0_awqos > m1_awqos)}
            iff (rst && m0_awvalid && m1_awvalid) {
            bins m0_wins_qos = {3'b111};
            bins m1_wins_qos = {3'b110};
            bins equal_qos   = default;
        }
    endgroup

    // ----------------------------------------------------------
    // CG11: CDC-port traffic
    // ----------------------------------------------------------
    covergroup cg_cdc @(posedge clk);
        option.name = "CDC Bridge Traffic";

        cp_cdc_on: coverpoint cdc_active iff (rst) {
            bins cdc_transfer = {1'b1};
            bins no_cdc       = {1'b0};
        }
        // cross CDC with burst length (via M1 which goes through CDC to S1)
        cp_m1_len_via_cdc: coverpoint m1_awlen
            iff (rst && m1_awvalid && m1_awready &&
                 (m1_awaddr >= S1_BASE) && (m1_awaddr <= S1_HIGH)) {
            bins single  = {8'd0};
            bins burst_4 = {8'd3};
            bins burst_8 = {8'd7};
            bins other   = default;
        }
    endgroup

    // ----------------------------------------------------------
    // CG12: Pipeline traffic (AW + W same cycle)
    // ----------------------------------------------------------
    covergroup cg_pipeline @(posedge clk);
        option.name = "Pipeline Efficiency";

        cp_m0_pipe: coverpoint m0_pipeline iff (rst) {
            bins pipelined  = {1'b1};
            bins sequential = {1'b0};
        }
        cp_m1_pipe: coverpoint m1_pipeline iff (rst) {
            bins pipelined  = {1'b1};
            bins sequential = {1'b0};
        }
        cp_both_pipe: coverpoint (m0_pipeline && m1_pipeline) iff (rst) {
            bins both_pipelined = {1'b1};
        }
    endgroup

    // ----------------------------------------------------------
    // Instantiate all covergroups
    // ----------------------------------------------------------
    cg_m0_write_addr cg_m0wa = new();
    cg_m1_write_addr cg_m1wa = new();
    cg_write_resp    cg_wr   = new();
    cg_m0_read_addr  cg_m0ra = new();
    cg_m1_read_addr  cg_m1ra = new();
    cg_routing       cg_rt   = new();
    cg_contention    cg_ct   = new();
    cg_concurrency   cg_cc   = new();
    cg_backpressure  cg_bp   = new();
    cg_qos           cg_qos_ = new();
    cg_cdc           cg_cdc_ = new();
    cg_pipeline      cg_pip  = new();

    // ----------------------------------------------------------
    // Final report
    // ----------------------------------------------------------
    final begin
        $display("\n====== COVERAGE REPORT v2 ======");
        $display("  M0 Write Addr  : %6.2f%%", cg_m0wa.get_coverage());
        $display("  M1 Write Addr  : %6.2f%%", cg_m1wa.get_coverage());
        $display("  Write Response : %6.2f%%", cg_wr.get_coverage());
        $display("  M0 Read  Addr  : %6.2f%%", cg_m0ra.get_coverage());
        $display("  M1 Read  Addr  : %6.2f%%", cg_m1ra.get_coverage());
        $display("  Routing Matrix : %6.2f%%", cg_rt.get_coverage());
        $display("  Contention     : %6.2f%%", cg_ct.get_coverage());
        $display("  Concurrency    : %6.2f%%", cg_cc.get_coverage());
        $display("  Backpressure   : %6.2f%%", cg_bp.get_coverage());
        $display("  QoS            : %6.2f%%", cg_qos_.get_coverage());
        $display("  CDC Traffic    : %6.2f%%", cg_cdc_.get_coverage());
        $display("  Pipeline       : %6.2f%%", cg_pip.get_coverage());
        $display("  ─────────────────────────────");
        $display("  Overall        : %6.2f%%",
            (cg_m0wa.get_coverage() + cg_m1wa.get_coverage() +
             cg_wr.get_coverage()   + cg_m0ra.get_coverage() +
             cg_m1ra.get_coverage() + cg_rt.get_coverage()   +
             cg_ct.get_coverage()   + cg_cc.get_coverage()   +
             cg_bp.get_coverage()   + cg_qos_.get_coverage() +
             cg_cdc_.get_coverage() + cg_pip.get_coverage()) / 12.0);
        $display("================================\n");
    end

endmodule
