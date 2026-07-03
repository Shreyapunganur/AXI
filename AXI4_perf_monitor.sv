`timescale 1ns/1ps
// =============================================================
// AXI4_perf_monitor.sv  —  Performance Counter Module
//
// Metrics tracked (per master):
//   LATENCY      : cycles from AW/AR handshake → B / R_LAST
//   THROUGHPUT   : transactions completed in last 256-cycle window
//   STALLS       : cycles where valid=1 but ready=0 per channel
//   BACKPRESSURE : aggregate stall events across all channels
//   BURST STATS  : total beats transferred, avg burst length
// =============================================================

module AXI4_perf_monitor #(
    parameter ADWIDTH = 32,
    parameter DWIDTH  = 64,
    parameter IDWIDTH = 4
)(
    input logic clk,
    input logic rst,

    // Master 0 AXI signals
    input logic m0_awvalid, m0_awready,
    input logic m0_wvalid,  m0_wready,  m0_wlast,
    input logic m0_bvalid,  m0_bready,
    input logic m0_arvalid, m0_arready,
    input logic m0_rvalid,  m0_rready,  m0_rlast,

    // Master 1 AXI signals
    input logic m1_awvalid, m1_awready,
    input logic m1_wvalid,  m1_wready,  m1_wlast,
    input logic m1_bvalid,  m1_bready,
    input logic m1_arvalid, m1_arready,
    input logic m1_rvalid,  m1_rready,  m1_rlast
);

    // ----------------------------------------------------------
    // Stall counters — one per channel per master
    // Stall = valid asserted but ready not yet asserted
    // ----------------------------------------------------------
    longint unsigned stall_m0_aw, stall_m0_w, stall_m0_b,
                     stall_m0_ar, stall_m0_r;
    longint unsigned stall_m1_aw, stall_m1_w, stall_m1_b,
                     stall_m1_ar, stall_m1_r;

    always_ff @(posedge clk) begin
        if (!rst) begin
            stall_m0_aw <= 0; stall_m0_w <= 0; stall_m0_b <= 0;
            stall_m0_ar <= 0; stall_m0_r <= 0;
            stall_m1_aw <= 0; stall_m1_w <= 0; stall_m1_b <= 0;
            stall_m1_ar <= 0; stall_m1_r <= 0;
        end else begin
            if (m0_awvalid && !m0_awready) stall_m0_aw <= stall_m0_aw + 1;
            if (m0_wvalid  && !m0_wready)  stall_m0_w  <= stall_m0_w  + 1;
            if (m0_bvalid  && !m0_bready)  stall_m0_b  <= stall_m0_b  + 1;
            if (m0_arvalid && !m0_arready) stall_m0_ar <= stall_m0_ar + 1;
            if (m0_rvalid  && !m0_rready)  stall_m0_r  <= stall_m0_r  + 1;

            if (m1_awvalid && !m1_awready) stall_m1_aw <= stall_m1_aw + 1;
            if (m1_wvalid  && !m1_wready)  stall_m1_w  <= stall_m1_w  + 1;
            if (m1_bvalid  && !m1_bready)  stall_m1_b  <= stall_m1_b  + 1;
            if (m1_arvalid && !m1_arready) stall_m1_ar <= stall_m1_ar + 1;
            if (m1_rvalid  && !m1_rready)  stall_m1_r  <= stall_m1_r  + 1;
        end
    end

    // ----------------------------------------------------------
    // Write latency measurement
    // From AW handshake → B handshake
    // ----------------------------------------------------------
    longint unsigned wr_lat_start_m0, wr_lat_sum_m0, wr_lat_max_m0;
    longint unsigned wr_lat_start_m1, wr_lat_sum_m1, wr_lat_max_m1;
    longint unsigned wr_txn_cnt_m0,   wr_txn_cnt_m1;
    logic            wr_inflight_m0,  wr_inflight_m1;
    longint unsigned cycle_cnt;

    always_ff @(posedge clk) begin
        if (!rst) begin
            cycle_cnt <= 0;
            wr_lat_start_m0 <= 0; wr_lat_sum_m0 <= 0; wr_lat_max_m0 <= 0;
            wr_lat_start_m1 <= 0; wr_lat_sum_m1 <= 0; wr_lat_max_m1 <= 0;
            wr_txn_cnt_m0 <= 0;   wr_txn_cnt_m1 <= 0;
            wr_inflight_m0 <= 0;  wr_inflight_m1 <= 0;
        end else begin
            cycle_cnt <= cycle_cnt + 1;

            // M0 write latency
            if (m0_awvalid && m0_awready && !wr_inflight_m0) begin
                wr_lat_start_m0 <= cycle_cnt;
                wr_inflight_m0  <= 1;
            end
            if (m0_bvalid && m0_bready && wr_inflight_m0) begin
                automatic longint lat = cycle_cnt - wr_lat_start_m0;
                wr_lat_sum_m0  <= wr_lat_sum_m0 + lat;
                wr_txn_cnt_m0  <= wr_txn_cnt_m0 + 1;
                if (lat > wr_lat_max_m0) wr_lat_max_m0 <= lat;
                wr_inflight_m0 <= 0;
            end

            // M1 write latency
            if (m1_awvalid && m1_awready && !wr_inflight_m1) begin
                wr_lat_start_m1 <= cycle_cnt;
                wr_inflight_m1  <= 1;
            end
            if (m1_bvalid && m1_bready && wr_inflight_m1) begin
                automatic longint lat = cycle_cnt - wr_lat_start_m1;
                wr_lat_sum_m1  <= wr_lat_sum_m1 + lat;
                wr_txn_cnt_m1  <= wr_txn_cnt_m1 + 1;
                if (lat > wr_lat_max_m1) wr_lat_max_m1 <= lat;
                wr_inflight_m1 <= 0;
            end
        end
    end

    // ----------------------------------------------------------
    // Read latency: AR handshake → R_LAST handshake
    // ----------------------------------------------------------
    longint unsigned rd_lat_start_m0, rd_lat_sum_m0, rd_lat_max_m0;
    longint unsigned rd_lat_start_m1, rd_lat_sum_m1, rd_lat_max_m1;
    longint unsigned rd_txn_cnt_m0,   rd_txn_cnt_m1;
    logic            rd_inflight_m0,  rd_inflight_m1;

    always_ff @(posedge clk) begin
        if (!rst) begin
            rd_lat_start_m0 <= 0; rd_lat_sum_m0 <= 0; rd_lat_max_m0 <= 0;
            rd_lat_start_m1 <= 0; rd_lat_sum_m1 <= 0; rd_lat_max_m1 <= 0;
            rd_txn_cnt_m0 <= 0;   rd_txn_cnt_m1 <= 0;
            rd_inflight_m0 <= 0;  rd_inflight_m1 <= 0;
        end else begin
            if (m0_arvalid && m0_arready && !rd_inflight_m0) begin
                rd_lat_start_m0 <= cycle_cnt; rd_inflight_m0 <= 1;
            end
            if (m0_rvalid && m0_rready && m0_rlast && rd_inflight_m0) begin
                automatic longint lat = cycle_cnt - rd_lat_start_m0;
                rd_lat_sum_m0  <= rd_lat_sum_m0 + lat;
                rd_txn_cnt_m0  <= rd_txn_cnt_m0 + 1;
                if (lat > rd_lat_max_m0) rd_lat_max_m0 <= lat;
                rd_inflight_m0 <= 0;
            end

            if (m1_arvalid && m1_arready && !rd_inflight_m1) begin
                rd_lat_start_m1 <= cycle_cnt; rd_inflight_m1 <= 1;
            end
            if (m1_rvalid && m1_rready && m1_rlast && rd_inflight_m1) begin
                automatic longint lat = cycle_cnt - rd_lat_start_m1;
                rd_lat_sum_m1  <= rd_lat_sum_m1 + lat;
                rd_txn_cnt_m1  <= rd_txn_cnt_m1 + 1;
                if (lat > rd_lat_max_m1) rd_lat_max_m1 <= lat;
                rd_inflight_m1 <= 0;
            end
        end
    end

    // ----------------------------------------------------------
    // Beat throughput counters
    // ----------------------------------------------------------
    longint unsigned w_beats_m0, w_beats_m1;
    longint unsigned r_beats_m0, r_beats_m1;

    always_ff @(posedge clk) begin
        if (!rst) begin
            w_beats_m0 <= 0; w_beats_m1 <= 0;
            r_beats_m0 <= 0; r_beats_m1 <= 0;
        end else begin
            if (m0_wvalid && m0_wready) w_beats_m0 <= w_beats_m0 + 1;
            if (m1_wvalid && m1_wready) w_beats_m1 <= w_beats_m1 + 1;
            if (m0_rvalid && m0_rready) r_beats_m0 <= r_beats_m0 + 1;
            if (m1_rvalid && m1_rready) r_beats_m1 <= r_beats_m1 + 1;
        end
    end

    // ----------------------------------------------------------
    // Sliding-window throughput (256-cycle window)
    // ----------------------------------------------------------
    logic [7:0]  window_cnt;
    logic [15:0] txn_in_window_m0, txn_in_window_m1;
    logic [15:0] last_tput_m0,     last_tput_m1;

    always_ff @(posedge clk) begin
        if (!rst) begin
            window_cnt <= 0;
            txn_in_window_m0 <= 0; txn_in_window_m1 <= 0;
            last_tput_m0 <= 0;     last_tput_m1 <= 0;
        end else begin
            window_cnt <= window_cnt + 1;
            if (m0_bvalid && m0_bready) txn_in_window_m0 <= txn_in_window_m0 + 1;
            if (m1_bvalid && m1_bready) txn_in_window_m1 <= txn_in_window_m1 + 1;
            if (m0_rvalid && m0_rready && m0_rlast) txn_in_window_m0 <= txn_in_window_m0 + 1;
            if (m1_rvalid && m1_rready && m1_rlast) txn_in_window_m1 <= txn_in_window_m1 + 1;

            if (window_cnt == 8'hFF) begin // every 256 cycles
                last_tput_m0 <= txn_in_window_m0;
                last_tput_m1 <= txn_in_window_m1;
                txn_in_window_m0 <= 0;
                txn_in_window_m1 <= 0;
            end
        end
    end

    // ----------------------------------------------------------
    // Final report
    // ----------------------------------------------------------
    final begin
        $display("\n====== PERFORMANCE REPORT ======");
        $display("  Total simulation cycles : %0d", cycle_cnt);

        $display("\n  [Master 0 — Write]");
        $display("    Transactions completed : %0d", wr_txn_cnt_m0);
        $display("    Total W beats          : %0d", w_beats_m0);
        if (wr_txn_cnt_m0 > 0)
            $display("    Avg write latency      : %0d cycles",
                     wr_lat_sum_m0 / wr_txn_cnt_m0);
        $display("    Max write latency      : %0d cycles", wr_lat_max_m0);
        $display("    AW stalls              : %0d", stall_m0_aw);
        $display("    W  stalls              : %0d", stall_m0_w);
        $display("    B  stalls              : %0d", stall_m0_b);

        $display("\n  [Master 0 — Read]");
        $display("    Transactions completed : %0d", rd_txn_cnt_m0);
        $display("    Total R beats          : %0d", r_beats_m0);
        if (rd_txn_cnt_m0 > 0)
            $display("    Avg read  latency      : %0d cycles",
                     rd_lat_sum_m0 / rd_txn_cnt_m0);
        $display("    Max read  latency      : %0d cycles", rd_lat_max_m0);
        $display("    AR stalls              : %0d", stall_m0_ar);
        $display("    R  stalls              : %0d", stall_m0_r);

        $display("\n  [Master 1 — Write]");
        $display("    Transactions completed : %0d", wr_txn_cnt_m1);
        $display("    Total W beats          : %0d", w_beats_m1);
        if (wr_txn_cnt_m1 > 0)
            $display("    Avg write latency      : %0d cycles",
                     wr_lat_sum_m1 / wr_txn_cnt_m1);
        $display("    Max write latency      : %0d cycles", wr_lat_max_m1);
        $display("    AW stalls              : %0d", stall_m1_aw);

        $display("\n  [Master 1 — Read]");
        $display("    Transactions completed : %0d", rd_txn_cnt_m1);
        if (rd_txn_cnt_m1 > 0)
            $display("    Avg read  latency      : %0d cycles",
                     rd_lat_sum_m1 / rd_txn_cnt_m1);
        $display("    Max read  latency      : %0d cycles", rd_lat_max_m1);
        $display("================================\n");
    end

endmodule
