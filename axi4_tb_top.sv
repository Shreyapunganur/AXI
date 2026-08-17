`timescale 1ns/1ps
import axi4_scoreboard_pkg::*;
import axi4_coverage_pkg::*;

// =====================================================================
// axi4_tb_top.sv
// -----------------------------------------------------------------------
// Drives the whole design (axi4_top) through its command/response
// interfaces on both masters, in two phases:
//
//   DIRECTED PHASE - one scripted example of each thing this project
//   claims to do (each burst type, WSTRB, multiple outstanding IDs,
//   both masters hitting the same slave at once, the CDC bridge) so
//   there's always a clean, easy-to-point-at example of every feature
//   in the log, independent of what the random phase happens to roll.
//
//   RANDOM PHASE - a constrained-random `axi4_txn` class generates
//   burst type / length / address / WSTRB combinations and fires them
//   at whichever master is picked, to get broad coverage without
//   scripting every case by hand.
//
// Every issued command is mirrored into the scoreboard (which predicts
// the expected result) and the coverage collector (which records what
// was exercised); every response that comes back is checked against
// the scoreboard automatically. See axi4_scoreboard.sv / axi4_coverage.sv.
// =====================================================================
module axi4_tb_top;

    // ---------------- clocks & reset ----------------
    // clk and clk_s1 use deliberately different periods (10ns vs 14ns,
    // not a multiple of each other) so anything that crosses between
    // them is genuinely asynchronous, not just "same clock, different
    // name".
    reg clk = 0, clk_s1 = 0, rst_n = 0;
    always #5 clk    = ~clk;
    always #7 clk_s1 = ~clk_s1;

    // ---------------- master 0 command / response wires ----------------
    reg         m0_cmd_valid, m0_cmd_write;
    reg [1:0]   m0_cmd_id, m0_cmd_burst;
    reg [31:0]  m0_cmd_addr, m0_cmd_wseed;
    reg [7:0]   m0_cmd_len;
    reg [3:0]   m0_cmd_wstrb;
    wire        m0_cmd_ready;
    wire        m0_rsp_b_valid, m0_rsp_r_valid, m0_rsp_r_last;
    wire [1:0]  m0_rsp_b_id, m0_rsp_r_id, m0_rsp_b_resp, m0_rsp_r_resp;
    wire [31:0] m0_rsp_r_data;

    // ---------------- master 1 command / response wires ----------------
    reg         m1_cmd_valid, m1_cmd_write;
    reg [1:0]   m1_cmd_id, m1_cmd_burst;
    reg [31:0]  m1_cmd_addr, m1_cmd_wseed;
    reg [7:0]   m1_cmd_len;
    reg [3:0]   m1_cmd_wstrb;
    wire        m1_cmd_ready;
    wire        m1_rsp_b_valid, m1_rsp_r_valid, m1_rsp_r_last;
    wire [1:0]  m1_rsp_b_id, m1_rsp_r_id, m1_rsp_b_resp, m1_rsp_r_resp;
    wire [31:0] m1_rsp_r_data;

    axi4_top dut (
        .clk(clk), .clk_s1(clk_s1), .rst_n(rst_n),

        .m0_cmd_valid(m0_cmd_valid), .m0_cmd_ready(m0_cmd_ready), .m0_cmd_write(m0_cmd_write),
        .m0_cmd_id(m0_cmd_id), .m0_cmd_addr(m0_cmd_addr), .m0_cmd_len(m0_cmd_len),
        .m0_cmd_burst(m0_cmd_burst), .m0_cmd_wstrb(m0_cmd_wstrb), .m0_cmd_wseed(m0_cmd_wseed),
        .m0_rsp_b_valid(m0_rsp_b_valid), .m0_rsp_b_id(m0_rsp_b_id), .m0_rsp_b_resp(m0_rsp_b_resp),
        .m0_rsp_r_valid(m0_rsp_r_valid), .m0_rsp_r_id(m0_rsp_r_id), .m0_rsp_r_resp(m0_rsp_r_resp),
        .m0_rsp_r_data(m0_rsp_r_data), .m0_rsp_r_last(m0_rsp_r_last),

        .m1_cmd_valid(m1_cmd_valid), .m1_cmd_ready(m1_cmd_ready), .m1_cmd_write(m1_cmd_write),
        .m1_cmd_id(m1_cmd_id), .m1_cmd_addr(m1_cmd_addr), .m1_cmd_len(m1_cmd_len),
        .m1_cmd_burst(m1_cmd_burst), .m1_cmd_wstrb(m1_cmd_wstrb), .m1_cmd_wseed(m1_cmd_wseed),
        .m1_rsp_b_valid(m1_rsp_b_valid), .m1_rsp_b_id(m1_rsp_b_id), .m1_rsp_b_resp(m1_rsp_b_resp),
        .m1_rsp_r_valid(m1_rsp_r_valid), .m1_rsp_r_id(m1_rsp_r_id), .m1_rsp_r_resp(m1_rsp_r_resp),
        .m1_rsp_r_data(m1_rsp_r_data), .m1_rsp_r_last(m1_rsp_r_last)
    );

    // ===================================================================
    // constrained-random transaction
    // ===================================================================
    class axi4_txn;
        rand bit       is_write;
        rand bit [1:0] burst;
        rand bit [7:0] len;
        rand bit [3:0] wstrb;
        rand bit [9:0] word_addr;
        rand bit       target_slave1;
        bit [31:0]     addr;

        // WRAP bursts must use a length the hardware actually supports
        // (2, 4, 8, or 16 beats - i.e. len = beats-1 of 1,3,7,15)
        constraint c_burst_len {
            (burst == 2'b10) -> len inside {8'd1, 8'd3, 8'd7, 8'd15};
            (burst != 2'b10) -> len inside {[8'd0:8'd7]};
        }
        // favor INCR (the common case) while still hitting FIXED/WRAP often
        constraint c_burst_dist {
            burst dist {2'b00 := 1, 2'b01 := 3, 2'b10 := 1};
        }
        // stay inside the writable part of each slave's 1024-word memory
        constraint c_addr_range {
            word_addr inside {[10'd0:10'd900]};
        }
        constraint c_wstrb_dist {
            wstrb dist {4'b1111 := 5, 4'b0011 := 1, 4'b1100 := 1, 4'b0001 := 1};
        }

        function void post_randomize();
            addr        = 32'h0;
            addr[16]    = target_slave1;
            addr[11:2]  = word_addr;
        endfunction
    endclass

    axi4_scoreboard sb;
    axi4_coverage   cov;

    // ---------------- simple performance counters ----------------
    int  total_txns = 0, m0_txns = 0, m1_txns = 0;
    time first_issue_time = 0, last_complete_time = 0;

    // ---------------- issue-order tracking, feeds the OOO coverage bin --
    // a queue of {master, id} tags in the order they were ISSUED; when a
    // transaction completes, if it isn't at the front of this queue then
    // something issued earlier is still outstanding - that's the
    // definition of "this one finished out of order".
    bit [2:0] issue_order_q[$];

    function automatic bit mark_completed(bit master_id, bit [1:0] id);
        bit [2:0] tag;
        bit       ooo;
        tag = {master_id, id};
        ooo = 1'b0;
        if (issue_order_q.size() > 0) begin
            if (issue_order_q[0] === tag) begin
                void'(issue_order_q.pop_front());
            end else begin
                ooo = 1'b1;
                for (int k = 0; k < issue_order_q.size(); k++) begin
                    if (issue_order_q[k] === tag) begin
                        issue_order_q.delete(k);
                        break;
                    end
                end
            end
        end
        return ooo;
    endfunction

    // ---------------- issue tasks (one per master) ----------------
    task automatic issue_m0(
        input bit is_write, input bit [1:0] id, input bit [31:0] addr, input bit [7:0] len,
        input bit [1:0] burst, input bit [3:0] wstrb, input bit [31:0] seed
    );
        int slave_idx;
        slave_idx = addr[16];
        @(negedge clk);
        m0_cmd_valid = 1; m0_cmd_write = is_write; m0_cmd_id = id; m0_cmd_addr = addr;
        m0_cmd_len = len; m0_cmd_burst = burst; m0_cmd_wstrb = wstrb; m0_cmd_wseed = seed;
        wait (m0_cmd_ready);
        if (is_write) sb.predict_write(slave_idx, addr, len, burst, wstrb, seed);
        else          sb.predict_read(0, id, slave_idx, addr, len, burst);
        cov.sample_txn(0, is_write, burst, len, wstrb);
        issue_order_q.push_back({1'b0, id});
        total_txns++; m0_txns++;
        if (first_issue_time == 0) first_issue_time = $time;
        @(negedge clk);
        m0_cmd_valid = 0;
    endtask

    task automatic issue_m1(
        input bit is_write, input bit [1:0] id, input bit [31:0] addr, input bit [7:0] len,
        input bit [1:0] burst, input bit [3:0] wstrb, input bit [31:0] seed
    );
        int slave_idx;
        slave_idx = addr[16];
        @(negedge clk);
        m1_cmd_valid = 1; m1_cmd_write = is_write; m1_cmd_id = id; m1_cmd_addr = addr;
        m1_cmd_len = len; m1_cmd_burst = burst; m1_cmd_wstrb = wstrb; m1_cmd_wseed = seed;
        wait (m1_cmd_ready);
        if (is_write) sb.predict_write(slave_idx, addr, len, burst, wstrb, seed);
        else          sb.predict_read(1, id, slave_idx, addr, len, burst);
        cov.sample_txn(1, is_write, burst, len, wstrb);
        issue_order_q.push_back({1'b1, id});
        total_txns++; m1_txns++;
        @(negedge clk);
        m1_cmd_valid = 0;
    endtask

    // ---------------- response monitors: check + retire order tracking --
    always @(posedge clk) begin
        bit ooo;
        if (m0_rsp_b_valid) begin
            ooo = mark_completed(1'b0, m0_rsp_b_id);
            cov.sample_ordering(ooo);
            last_complete_time = $time;
        end
        if (m0_rsp_r_valid) begin
            sb.check_read_beat(0, m0_rsp_r_id, m0_rsp_r_data);
            if (m0_rsp_r_last) begin
                ooo = mark_completed(1'b0, m0_rsp_r_id);
                cov.sample_ordering(ooo);
                last_complete_time = $time;
            end
        end
        if (m1_rsp_b_valid) begin
            ooo = mark_completed(1'b1, m1_rsp_b_id);
            cov.sample_ordering(ooo);
            last_complete_time = $time;
        end
        if (m1_rsp_r_valid) begin
            sb.check_read_beat(1, m1_rsp_r_id, m1_rsp_r_data);
            if (m1_rsp_r_last) begin
                ooo = mark_completed(1'b1, m1_rsp_r_id);
                cov.sample_ordering(ooo);
                last_complete_time = $time;
            end
        end
    end

    // ===================================================================
    // main stimulus
    // ===================================================================
    bit [1:0] next_id_m0 = 0, next_id_m1 = 0;

    initial begin
        axi4_txn txn;
        sb  = new();
        cov = new();

        $dumpfile("axi4_tb.vcd");
        $dumpvars(0, axi4_tb_top);

        m0_cmd_valid = 0;
        m1_cmd_valid = 0;
        #23 rst_n = 1;
        @(negedge clk);

        $display("\n================ DIRECTED PHASE ================");

        // one of each burst type, master 0, slave 0
        issue_m0(1, 0, 32'h0000_0040, 8'd3, 2'b01, 4'b1111, 32'hAAAA_0000); // INCR
        issue_m0(0, 1, 32'h0000_0040, 8'd3, 2'b01, 4'b0000, 32'h0);
        issue_m0(1, 0, 32'h0000_0100, 8'd3, 2'b00, 4'b1111, 32'hBEEF_0000); // FIXED
        issue_m0(0, 1, 32'h0000_0100, 8'd0, 2'b00, 4'b0000, 32'h0);
        issue_m0(1, 0, 32'h0000_0208, 8'd3, 2'b10, 4'b1111, 32'hC0DE_0000); // WRAP
        issue_m0(0, 1, 32'h0000_0200, 8'd0, 2'b00, 4'b0000, 32'h0);
        repeat (20) @(posedge clk);

        // WSTRB: full write, then partial byte write, reading back both times
        issue_m0(1, 2, 32'h0000_0300, 8'd0, 2'b01, 4'b1111, 32'h1234_5678);
        issue_m0(0, 1, 32'h0000_0300, 8'd0, 2'b00, 4'b0000, 32'h0);
        issue_m0(1, 2, 32'h0000_0300, 8'd0, 2'b01, 4'b0001, 32'hFFFF_FF00);
        issue_m0(0, 1, 32'h0000_0300, 8'd0, 2'b00, 4'b0000, 32'h0);
        repeat (20) @(posedge clk);

        // multiple outstanding, different ids - directly exercises OOO
        issue_m0(0, 0, 32'h0000_0040, 8'd0, 2'b01, 4'b0000, 32'h0);
        issue_m0(0, 1, 32'h0000_0100, 8'd0, 2'b01, 4'b0000, 32'h0);
        issue_m0(0, 2, 32'h0000_0300, 8'd0, 2'b01, 4'b0000, 32'h0);
        issue_m0(0, 3, 32'h0000_0300, 8'd0, 2'b01, 4'b0000, 32'h0);
        repeat (40) @(posedge clk);

        // both masters writing the SAME slave at once - exercises round robin
        fork
            issue_m0(1, 0, 32'h0000_0400, 8'd1, 2'b01, 4'b1111, 32'h1111_0000);
            issue_m1(1, 0, 32'h0000_0410, 8'd1, 2'b01, 4'b1111, 32'h2222_0000);
        join
        repeat (30) @(posedge clk);

        // through the CDC bridge (slave 1, its own clock domain)
        issue_m0(1, 0, 32'h0001_0040, 8'd1, 2'b01, 4'b1111, 32'h5A5A_0000);
        issue_m0(0, 1, 32'h0001_0040, 8'd1, 2'b01, 4'b0000, 32'h0);
        repeat (30) @(posedge clk);

        $display("\n================ RANDOM PHASE ================");
        for (int i = 0; i < 60; i = i + 1) begin
            bit use_m1;
            txn = new();
            void'(txn.randomize());
            use_m1 = ($urandom_range(0, 1) == 1);
            if (use_m1) begin
                issue_m1(txn.is_write, next_id_m1, txn.addr, txn.len, txn.burst, txn.wstrb,
                         {24'h0, next_id_m1, txn.word_addr[5:0]} ^ 32'h5A5A_0000);
                next_id_m1 = next_id_m1 + 1;
            end else begin
                issue_m0(txn.is_write, next_id_m0, txn.addr, txn.len, txn.burst, txn.wstrb,
                         {24'h0, next_id_m0, txn.word_addr[5:0]} ^ 32'hA5A5_0000);
                next_id_m0 = next_id_m0 + 1;
            end
        end

        repeat (200) @(posedge clk);   // let everything still outstanding drain

        $display("\n================ SUMMARY ================");
        $display(" transactions issued : %0d  (M0=%0d, M1=%0d)", total_txns, m0_txns, m1_txns);
        sb.report();
        cov.report();

        if (sb.fail_count == 0)
            $display("\n*** ALL CHECKS PASSED ***");
        else
            $display("\n*** %0d CHECK(S) FAILED ***", sb.fail_count);

        $finish;
    end

    initial begin
        #100000;
        $display("TIMEOUT - test hung");
        $finish;
    end

endmodule
