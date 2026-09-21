`timescale 1ns/1ps

package axi4_coverage_pkg;

    class axi4_coverage;

        bit [1:0] cg_burst_type;
        bit [7:0] cg_burst_len;
        bit [3:0] cg_wstrb;
        bit       cg_is_write;
        int       cg_master_id;
        bit       cg_ooo;

        covergroup cg_transaction;

            option.per_instance = 1;

            cp_burst: coverpoint cg_burst_type {
                bins fixed_b = {2'b00};
                bins incr_b  = {2'b01};
                bins wrap_b  = {2'b10};
            }

            cp_len: coverpoint cg_burst_len {
                bins single      = {0};
                bins short_burst = {[1:3]};
                bins long_burst  = {[4:15]};
            }

            cp_wstrb: coverpoint cg_wstrb {
                bins full_word  = {4'b1111};
                bins lower_half = {4'b0011};
                bins upper_half = {4'b1100};
                bins one_byte   = {
                    4'b0001,
                    4'b0010,
                    4'b0100,
                    4'b1000
                };
            }

            cp_dir: coverpoint cg_is_write {
                bins write = {1};
                bins read  = {0};
            }

            cp_master: coverpoint cg_master_id {
                bins m0 = {0};
                bins m1 = {1};
            }

            x_burst_dir:    cross cp_burst, cp_dir;
            x_master_burst: cross cp_master, cp_burst;

        endgroup

        covergroup cg_ordering;

            option.per_instance = 1;

            cp_ooo: coverpoint cg_ooo {
                bins in_order     = {0};
                bins out_of_order = {1};
            }

        endgroup

        function new();
            cg_transaction = new();
            cg_ordering    = new();
        endfunction

        function automatic void sample_txn(
            int       master_id,
            bit       is_write,
            bit [1:0] burst,
            bit [7:0] len,
            bit [3:0] wstrb
        );

            cg_master_id  = master_id;
            cg_is_write   = is_write;
            cg_burst_type = burst;
            cg_burst_len  = len;
            cg_wstrb      = wstrb;

            cg_transaction.sample();

        endfunction

        function automatic void sample_ordering(bit ooo);

            cg_ooo = ooo;
            cg_ordering.sample();

        endfunction

        function automatic void report();

            $display("---------------------------------------------------");

            $display(
                " COVERAGE: transaction group = %0.1f%%, ordering group = %0.1f%%",
                cg_transaction.get_coverage(),
                cg_ordering.get_coverage()
            );

            $display("---------------------------------------------------");

        endfunction

    endclass

endpackage
