`timescale 1ns/1ps
// =====================================================================
// axi4_scoreboard.sv
// -----------------------------------------------------------------------
// A reference model, wrapped in a package so axi4_tb_top.sv can just
// `import` it (no `include` needed anywhere).
//
// THE IDEA: this class keeps its OWN copy of both slaves' memory
// ("shadow memory"), computed purely from what commands the testbench
// SENT - it never looks at the DUT's internals. Two calls drive it:
//
//   predict_write(...)  - called the moment a write is issued. Steps
//                         through the burst exactly the way the real
//                         slave does (same address-stepping rule) and
//                         updates the shadow memory, respecting WSTRB.
//
//   predict_read(...)   - called the moment a read is issued. Steps
//                         through the burst the same way and PUSHES the
//                         expected word for every beat into a small
//                         per-(master,id) queue.
//
//   check_read_beat(...) - called every time a real R beat arrives from
//                         the DUT. Pops the next expected word off that
//                         same queue and compares.
//
// Because there's a separate queue per (master, id), this works
// correctly no matter what order responses actually arrive in - out of
// order completion across different IDs doesn't confuse it at all,
// since each ID's own beats are still checked in the order THAT id's
// beats were predicted.
// =====================================================================
package axi4_scoreboard_pkg;

    class axi4_scoreboard;
        // shadow memory: [slave][word]. Matches MEM_WORDS in axi4_slave.v.
        logic [31:0] mem [2][1024];

        int pass_count = 0;
        int fail_count = 0;

        // one expected-data buffer per master (0/1) per id (0-3). A
        // fixed array with a head/count pair here, instead of a `queue`
        // - same "small fixed slot, plain index" idea used everywhere
        // in the RTL, and it's the one outstanding-per-id max (16 beats,
        // matching the largest WRAP burst) so a fixed size is exact,
        // not a guess.
        logic [31:0] exp_data [2][4][16];
        int          exp_count[2][4];   // how many beats are still expected
        int          exp_head [2][4];   // index of the next beat to check

        function new();
        endfunction

        // identical address-stepping rule to axi4_slave.v's next_addr
        // function - if this ever drifts out of sync with the RTL, the
        // scoreboard will start reporting false mismatches immediately,
        // which is exactly the kind of loud failure you want here.
        function automatic logic [31:0] next_addr(logic [31:0] addr, bit [7:0] len, bit [1:0] burst);
            logic [31:0] burst_bytes, wrap_base;
            if (burst == 2'b00) begin              // FIXED
                next_addr = addr;
            end else if (burst == 2'b10) begin      // WRAP
                burst_bytes = 4 * (len + 1);
                wrap_base   = (addr / burst_bytes) * burst_bytes;
                if ((addr + 4) >= (wrap_base + burst_bytes))
                    next_addr = wrap_base;
                else
                    next_addr = addr + 4;
            end else begin                          // INCR
                next_addr = addr + 4;
            end
        endfunction

        function automatic void predict_write(
            int slave_idx, logic [31:0] addr, bit [7:0] len, bit [1:0] burst,
            bit [3:0] wstrb, logic [31:0] seed
        );
            logic [31:0] a, wdata, old_word, merged;
            a = addr;
            for (int i = 0; i <= len; i++) begin
                wdata    = seed + i;
                old_word = mem[slave_idx][a[11:2]];
                merged   = old_word;
                for (int b = 0; b < 4; b++)
                    if (wstrb[b]) merged[b*8 +: 8] = wdata[b*8 +: 8];
                mem[slave_idx][a[11:2]] = merged;
                a = next_addr(a, len, burst);
            end
        endfunction

        function automatic void predict_read(
            int master_idx, int id, int slave_idx,
            logic [31:0] addr, bit [7:0] len, bit [1:0] burst
        );
            logic [31:0] a;
            a = addr;
            exp_head[master_idx][id]  = 0;
            exp_count[master_idx][id] = len + 1;
            for (int i = 0; i <= len; i++) begin
                exp_data[master_idx][id][i] = mem[slave_idx][a[11:2]];
                a = next_addr(a, len, burst);
            end
        endfunction

        function automatic void check_read_beat(int master_idx, int id, logic [31:0] got_data);
            logic [31:0] exp_data_word;
            if (exp_count[master_idx][id] == 0) begin
                $display("SCOREBOARD ERROR: unexpected read beat on master=%0d id=%0d (nothing was predicted)", master_idx, id);
                fail_count++;
                return;
            end
            exp_data_word = exp_data[master_idx][id][exp_head[master_idx][id]];
            exp_head[master_idx][id]  = exp_head[master_idx][id] + 1;
            exp_count[master_idx][id] = exp_count[master_idx][id] - 1;
            if (exp_data_word !== got_data) begin
                $display("SCOREBOARD MISMATCH: master=%0d id=%0d expected=%08h got=%08h", master_idx, id, exp_data_word, got_data);
                fail_count++;
            end else begin
                pass_count++;
            end
        endfunction

        function automatic void report();
            $display("---------------------------------------------------");
            $display(" SCOREBOARD: %0d checks passed, %0d failed", pass_count, fail_count);
            $display("---------------------------------------------------");
        endfunction
    endclass

endpackage
