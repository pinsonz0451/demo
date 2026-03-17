// =============================================================================
// File    : tb_int_to_fp_acc.v
// Desc    : Testbench for int_to_fp_acc.v (INT-to-FP Accumulation Unit)
//
// Compile & run (Icarus Verilog):
//   iverilog -g2001 -o sim_acc.out \
//            int_to_fp21.v fp21_adder.v fp21_to_fp16.v int_to_fp_acc.v tb_int_to_fp_acc.v
//   vvp sim_acc.out
//   gtkwave tb_int_to_fp_acc.vcd   (optional waveform view)
//
// -----------------------------------------------------------------------
// DUT parameters:
//   NUM_COL = 4   (reduced for manual verification tractability)
//   PSUM_W  = 32
//
// -----------------------------------------------------------------------
// Signal driving convention:
//   Inputs driven at NEGEDGE clk.
//   Outputs sampled AFTER POSEDGE clk (when fp16_valid is HIGH).
// =============================================================================

`timescale 1ns/1ps

module tb_int_to_fp_acc;
    // =========================================================================
    // Parameters
    // =========================================================================
    localparam NUM_COL = 4;
    localparam PSUM_W  = 32;
    localparam CLK_T   = 10;

    // =========================================================================
    // DUT I/O
    // =========================================================================
    reg                        clk;
    reg                        rst_n;
    reg                        mac_valid;
    reg                        writeback;
    reg                        acc_clear;
    reg  [4:0]                 emin;
    reg  [NUM_COL*PSUM_W-1:0]  mac_result_flat;
    wire [NUM_COL*16-1:0]      fp16_result_flat;
    wire                       fp16_valid;

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    int_to_fp_acc #(
        .NUM_COL (NUM_COL),
        .PSUM_W  (PSUM_W)
    ) dut (
        .clk              (clk),
        .rst_n            (rst_n),
        .mac_valid        (mac_valid),
        .writeback        (writeback),
        .acc_clear        (acc_clear),
        .emin             (emin),
        .mac_result_flat  (mac_result_flat),
        .fp16_result_flat (fp16_result_flat),
        .fp16_valid       (fp16_valid)
    );

    // =========================================================================
    // Clock generation
    // =========================================================================
    initial clk = 1'b0;
    always #(CLK_T/2) clk = ~clk;

    // =========================================================================
    // Shared variables
    // =========================================================================
    integer pass_cnt;
    integer fail_cnt;
    reg ok_flag;

    // =========================================================================
    // Tasks
    // =========================================================================
    task do_reset;
        begin
            rst_n           = 1'b0;
            mac_valid       = 1'b0;
            writeback       = 1'b0;
            acc_clear       = 1'b0;
            emin            = 5'd0;
            mac_result_flat = {(NUM_COL*PSUM_W){1'b0}};
            repeat(3) @(posedge clk);
            @(negedge clk);
            rst_n = 1'b1;
            @(posedge clk); #1;
        end
    endtask

    // Pulse acc_clear to zero out internal FP21 accumulators
    task do_clear;
        begin
            @(negedge clk);
            acc_clear = 1'b1;
            @(negedge clk);
            acc_clear = 1'b0;
        end
    endtask

    // Drive an integer array into the accumulator for 1 cycle
    task drive_accumulation;
        input signed [31:0] val0, val1, val2, val3;
        begin
            @(negedge clk);
            mac_result_flat[0*32 +: 32] = val0;
            mac_result_flat[1*32 +: 32] = val1;
            mac_result_flat[2*32 +: 32] = val2;
            mac_result_flat[3*32 +: 32] = val3;
            mac_valid = 1'b1;
            @(negedge clk);
            mac_valid = 1'b0;
        end
    endtask

    // Trigger write-back and check results
    task check_result;
        input [31:0] tc_id;
        input [15:0] exp0, exp1, exp2, exp3;
        reg [15:0] got0, got1, got2, got3;
        begin
            ok_flag = 1'b1;

            // Trigger writeback at negedge
            @(negedge clk);
            writeback = 1'b1;
            @(negedge clk);
            writeback = 1'b0;

            // Wait for fp16_valid to be HIGH
            while (!fp16_valid) begin
                @(posedge clk);
            end
            #1; // sampling delay

            got0 = fp16_result_flat[0*16 +: 16];
            got1 = fp16_result_flat[1*16 +: 16];
            got2 = fp16_result_flat[2*16 +: 16];
            got3 = fp16_result_flat[3*16 +: 16];

            if (got0 !== exp0) begin $display("[TC%0d FAIL] Col0 expected %04X, got %04X", tc_id, exp0, got0); ok_flag = 1'b0; end
            if (got1 !== exp1) begin $display("[TC%0d FAIL] Col1 expected %04X, got %04X", tc_id, exp1, got1); ok_flag = 1'b0; end
            if (got2 !== exp2) begin $display("[TC%0d FAIL] Col2 expected %04X, got %04X", tc_id, exp2, got2); ok_flag = 1'b0; end
            if (got3 !== exp3) begin $display("[TC%0d FAIL] Col3 expected %04X, got %04X", tc_id, exp3, got3); ok_flag = 1'b0; end

            if (ok_flag) begin
                $display("[TC%0d PASS] Outputs match expected FP16 values.", tc_id);
                pass_cnt = pass_cnt + 1;
            end else begin
                fail_cnt = fail_cnt + 1;
            end
        end
    endtask


    // =========================================================================
    // Main Test Sequence
    // =========================================================================
    initial begin
        $dumpfile("tb_int_to_fp_acc.vcd");
        $dumpvars(0, tb_int_to_fp_acc);

        pass_cnt = 0;
        fail_cnt = 0;

        do_reset;
        $display("=================================================");
        $display(" INT-to-FP Accumulation Unit Testbench Start");
        $display("=================================================");

        // ---------------------------------------------------------------------
        // TC1: Single Positive Integer Conversion
        //   Input: +32 for all columns
        //   Math: +32 = +1.0 * 2^5.
        //   e_bias = emin - 10 - 15 = 0  =>  emin = 25
        //   FP16 Exp = 5 + 0 + 15 = 20 (5'h14). Mantissa = 0.
        //   Expected FP16 = 16'b0_10100_0000000000 = 16'h5000
        // ---------------------------------------------------------------------
        $display("--- TC1: Single Positive Int (+32) ---");
        emin = 5'd25;    // e_bias = 25 - 10 - 15 = 0
        do_clear;
        drive_accumulation(32'd32, 32'd32, 32'd32, 32'd32);
        check_result(1, 16'h5000, 16'h5000, 16'h5000, 16'h5000);

        // ---------------------------------------------------------------------
        // TC2: Mixed Positive and Negative (Testing 2's Complement LZD)
        //   Col0: +32 -> 16'h5000
        //   Col1: -32 -> 16'hD000 (Sign bit is 1)
        //   Col2: +16 -> Exp = 4+15 = 19 (5'h13) -> 16'h4C00
        //   Col3: -16 -> 16'hCC00
        // ---------------------------------------------------------------------
        $display("--- TC2: Mixed Positive and Negative ---");
        emin = 5'd25;    // e_bias = 25 - 10 - 15 = 0
        do_clear;
        drive_accumulation(32'd32, -32'd32, 32'd16, -32'd16);
        check_result(2, 16'h5000, 16'hD000, 16'h4C00, 16'hCC00);

        // ---------------------------------------------------------------------
        // TC3: Temporal Accumulation (10 + 22 = 32)
        //   Verifies that fp21_adder works correctly across cycles.
        //   Cycle 1: Add +10.  Cycle 2: Add +22.
        //   Result should exactly equal +32 -> 16'h5000.
        // ---------------------------------------------------------------------
        $display("--- TC3: Temporal Accumulation (10 + 22 = 32) ---");
        emin = 5'd25;    // e_bias = 25 - 10 - 15 = 0
        do_clear;
        drive_accumulation(32'd10, 32'd10, 32'd10, 32'd10);
        drive_accumulation(32'd22, 32'd22, 32'd22, 32'd22);
        check_result(3, 16'h5000, 16'h5000, 16'h5000, 16'h5000);

        // ---------------------------------------------------------------------
        // TC4: Cancellation to Zero (+64 then -64 = 0)
        //   Tests the sum_zero flag and exact cancellation logic.
        //   Expected FP16 = +0.0 -> 16'h0000
        // ---------------------------------------------------------------------
        $display("--- TC4: Cancellation to Zero (+64 then -64) ---");
        emin = 5'd25;    // e_bias = 25 - 10 - 15 = 0
        do_clear;
        drive_accumulation(32'd64, 32'd64, 32'd64, 32'd64);
        drive_accumulation(-32'd64, -32'd64, -32'd64, -32'd64);
        check_result(4, 16'h0000, 16'h0000, 16'h0000, 16'h0000);

        // ---------------------------------------------------------------------
        // TC5: emin Compensation (Exponent adjustment)
        //   Input +32, e_bias = emin - 10 - 15 = 30 - 25 = 5.
        //   FP16 Exp = 5(abs) + 5(bias) + 15 = 25 (5'h19)
        //   Expected FP16 = 16'b0_11001_0000000000 = 16'h6400
        // ---------------------------------------------------------------------
        $display("--- TC5: emin Compensation (emin = 30, e_bias = 5) ---");
        emin = 5'd30;    // e_bias = 30 - 10 - 15 = 5
        do_clear;
        drive_accumulation(32'd32, 32'd32, 32'd32, 32'd32);
        check_result(5, 16'h6400, 16'h6400, 16'h6400, 16'h6400);

        // ---------------------------------------------------------------------
        // TC6: Clear function and Fractional Mantissa (1536)
        //   1. Do NOT clear, add 1536 (verifying previous values are overwritten by do_clear in reality, but here we explicitly don't call it to verify if we just accum).
        //   Wait, we WANT to test do_clear. Let's do_clear first.
        //   Input: +1536 = 1024 + 512 = 2^10 * 1.5
        //   Exp = 10. FP16 Exp = 10 + 0 + 15 = 25 (5'h19).
        //   Mantissa 0.5 = 10'b1000000000.
        //   Expected FP16 = 16'b0_11001_1000000000 = 16'h6600
        // ---------------------------------------------------------------------
        $display("--- TC6: Clear & Fractional Mantissa (1536) ---");
        emin = 5'd25;    // e_bias = 0
        do_clear; 
        drive_accumulation(32'd1536, 32'd1536, 32'd1536, 32'd1536);
        check_result(6, 16'h6600, 16'h6600, 16'h6600, 16'h6600);

        // =====================================================================
        // Final Summary
        // =====================================================================
        $display("=================================================");
        $display("  RESULTS: %0d PASSED  /  %0d FAILED  (total %0d)",
                 pass_cnt, fail_cnt, pass_cnt + fail_cnt);
        $display("=================================================");
        if (fail_cnt == 0)
            $display("  >>> ALL TESTS PASSED <<<");
        else
            $display("  >>> SOME TESTS FAILED - review output above <<<");
        
        $finish;
    end

endmodule