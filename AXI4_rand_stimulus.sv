`timescale 1ns/1ps
// =============================================================
// AXI4_rand_stimulus.sv  —  Constrained-Random Transaction Class
//
// Usage in testbench:
//   import axi4_pkg::*;
//   AXI4_transaction t = new();
//   repeat(N) begin
//     assert(t.randomize());
//     // apply t.addr, t.len, t.burst, t.qos, t.data to master
//   end
// =============================================================

package axi4_pkg;

    class AXI4_transaction;

        // Randomizable fields
        rand logic        rd_wr;         // 1=write  0=read
        rand logic [31:0] addr;
        rand logic [7:0]  len;           // AXI awlen (beats - 1)
        rand logic [1:0]  burst;         // FIXED=00 INCR=01 WRAP=10
        rand logic [3:0]  qos;
        rand logic [63:0] data;
        rand logic        target_slave;  // 0=S0  1=S1 (CDC path)

        // Address bounds (set before randomize if needed)
        logic [31:0] s0_base = 32'h0000_0000;
        logic [31:0] s0_high = 32'h0000_07F8;
        logic [31:0] s1_base = 32'h0001_0000;
        logic [31:0] s1_high = 32'h0001_07F8;

        // ----------------------------------------------------------
        // Constraints
        // ----------------------------------------------------------

        // Address: must be within the selected slave's range
        constraint c_addr_range {
            if (target_slave == 0)
                addr inside {[s0_base : s0_high]};
            else
                addr inside {[s1_base : s1_high]};
        }

        // Address: must be 8-byte aligned (64-bit bus = 3 LSBs zero)
        constraint c_align { addr[2:0] == 3'b000; }

        // Burst length distribution — weighted toward common lengths
        constraint c_len_dist {
            len dist { 8'd0  := 35,   // single beat  (most common)
                       8'd1  := 20,   // 2-beat
                       8'd3  := 20,   // 4-beat
                       8'd7  := 15,   // 8-beat
                       8'd15 := 10    // 16-beat
                     };
        }

        // Burst type — mostly INCR, some FIXED and WRAP
        constraint c_burst_dist {
            burst dist { 2'b01 := 70,   // INCR
                         2'b00 := 15,   // FIXED
                         2'b10 := 15    // WRAP
                       };
        }

        // WRAP burst requires power-of-2 lengths
        constraint c_wrap_len {
            burst == 2'b10 -> len inside {8'd1, 8'd3, 8'd7, 8'd15};
        }

        // WRAP burst: address must be naturally aligned to burst boundary
        // boundary = (len+1) * 8 bytes
        constraint c_wrap_align {
            burst == 2'b10 && len == 8'd1  -> addr[3:0]  == 4'h0;
            burst == 2'b10 && len == 8'd3  -> addr[4:0]  == 5'h0;
            burst == 2'b10 && len == 8'd7  -> addr[5:0]  == 6'h0;
            burst == 2'b10 && len == 8'd15 -> addr[6:0]  == 7'h0;
        }

        // QoS — uniform distribution across all 16 values
        constraint c_qos_dist {
            qos dist { [4'h0:4'h3] := 25,
                       [4'h4:4'h7] := 25,
                       [4'h8:4'hB] := 25,
                       [4'hC:4'hF] := 25 };
        }

        // CDC slave (S1) accessed less often (it has extra latency)
        constraint c_slave_dist {
            target_slave dist { 0 := 60, 1 := 40 };
        }

        // Write slightly more than read
        constraint c_rw_dist {
            rd_wr dist { 1 := 55, 0 := 45 };
        }

        // ----------------------------------------------------------
        // Display helper
        // ----------------------------------------------------------
        function string to_str();
            $sformat(to_str,
                "%s addr=0x%08h len=%3d burst=%02b qos=%2d slave=%0d data=0x%016h",
                rd_wr ? "WR" : "RD",
                addr, len, burst, qos, target_slave, data);
        endfunction

        // ----------------------------------------------------------
        // Post-randomize hook for debug
        // ----------------------------------------------------------
        function void post_randomize();
            // Could add transaction ID assignment here in future
        endfunction

    endclass : AXI4_transaction

endpackage : axi4_pkg
