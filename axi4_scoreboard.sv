`timescale 1ns/1ps

package axi4_scoreboard_pkg;

    class axi4_scoreboard;

        logic [31:0] mem [2][1024];

        int pass_count = 0;
        int fail_count = 0;

        logic [31:0] exp_data [2][4][16];
        int          exp_count[2][4];
        int          exp_head [2][4];

        function new();
        endfunction

        function automatic logic [31:0] next_addr(
            logic [31:0] addr,
            bit [7:0] len,
            bit [1:0] burst
        );
            logic [31:0] burst_bytes, wrap_base;

            if (burst == 2'b00) begin
                next_addr = addr;
            end
            else if (burst == 2'b10) begin
                burst_bytes = 4 * (len + 1);
                wrap_base   = (addr / burst_bytes) * burst_bytes;

                if ((addr + 4) >= (wrap_base + burst_bytes))
                    next_addr = wrap_base;
                else
                    next_addr = addr + 4;
            end
            else begin
                next_addr = addr + 4;
            end
        endfunction

        function automatic void predict_write(
            int slave_idx,
            logic [31:0] addr,
            bit [7:0] len,
            bit [1:0] burst,
            bit [3:0] wstrb,
            logic [31:0] seed
        );
            logic [31:0] a, wdata, old_word, merged;

            a = addr;

            for (int i = 0; i <= len; i++) begin
                wdata    = seed + i;
                old_word = mem[slave_idx][a[11:2]];
                merged   = old_word;

                for (int b = 0; b < 4; b++)
                    if (wstrb[b])
                        merged[b*8 +: 8] = wdata[b*8 +: 8];

                mem[slave_idx][a[11:2]] = merged;
                a = next_addr(a, len, burst);
            end
        endfunction

        function automatic void predict_read(
            int master_idx,
            int id,
            int slave_idx,
            logic [31:0] addr,
            bit [7:0] len,
            bit [1:0] burst
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

        function automatic void check_read_beat(
            int master_idx,
            int id,
            logic [31:0] got_data
        );
            logic [31:0] exp_data_word;

            if (exp_count[master_idx][id] == 0) begin
                $display(
                    "SCOREBOARD ERROR: unexpected read beat on master=%0d id=%0d (nothing was predicted)",
                    master_idx,
                    id
                );
                fail_count++;
                return;
            end

            exp_data_word = exp_data[master_idx][id][exp_head[master_idx][id]];
            exp_head[master_idx][id]  = exp_head[master_idx][id] + 1;
            exp_count[master_idx][id] = exp_count[master_idx][id] - 1;

            if (exp_data_word !== got_data) begin
                $display(
                    "SCOREBOARD MISMATCH: master=%0d id=%0d expected=%08h got=%08h",
                    master_idx,
                    id,
                    exp_data_word,
                    got_data
                );
                fail_count++;
            end
            else begin
                pass_count++;
            end
        endfunction

        function automatic void report();
            $display("---------------------------------------------------");
            $display(
                " SCOREBOARD: %0d checks passed, %0d failed",
                pass_count,
                fail_count
            );
            $display("---------------------------------------------------");
        endfunction

    endclass

endpackage
