// =============================================================================
//  tb_aligner.v  —  Testbench for aligner.v
// =============================================================================

`timescale 1ns / 1ps

module tb_aligner;

    // =========================================================================
    // Parameters
    // =========================================================================
    localparam NUM_CH   = 4;
    localparam EXP_W    = 5;
    localparam MAN_W    = 10;
    localparam FP_W     = 16;
    localparam MAN_TC_W = MAN_W + 2; // 12  (sign + hidden + mantissa)

    // =========================================================================
    // DUT I/O
    // =========================================================================
    reg                      clk;
    reg                      rst_n;
    reg                      load;
    reg  [EXP_W-1:0]         emax_r;
    reg  [EXP_W-1:0]         emin_r;
    reg  [NUM_CH*FP_W-1:0]   fp_vec_r;
    wire [NUM_CH-1:0]        bs_out;
    wire                     bs_valid;
    wire                     bs_done;
    wire [EXP_W+3:0]         total_cyc_out; // <--- 新增：接住进度条输出

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    aligner #(
        .NUM_CH (NUM_CH),
        .EXP_W  (EXP_W),
        .MAN_W  (MAN_W),
        .FP_W   (FP_W)
    ) dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .load          (load),
        .emax          (emax_r),
        .emin          (emin_r),
        .fp_vec_in     (fp_vec_r),
        .bs_out        (bs_out),
        .bs_valid      (bs_valid),
        .bs_done       (bs_done),
        .total_cyc_out (total_cyc_out) // <--- 新增：连接新端口
    );

    // =========================================================================
    // Clock  100 MHz
    // =========================================================================
    initial clk = 1'b0;
    always  #5 clk = ~clk;

    // =========================================================================
    // Global simulation state  (all module-level to satisfy Verilog-2001)
    // =========================================================================
    integer     err_total; // cumulative error counter
    reg [255:0] tc_name;   // test label used by run_test / $display
    integer     rng_seed;  // fixed seed for TC11

    // Variables shared by run_test
    integer     tk_cyc;
    integer     tk_c;
    integer     tk_ch;
    integer     tk_err;
    reg         tk_exp_b;
    reg         tk_act_b;

    // Variables for TC11 (randomised)
    integer     rv, rc, rch, r_errs;
    reg [EXP_W-1:0]       r_emax, r_emin;
    reg [NUM_CH*FP_W-1:0] r_vec;
    reg [FP_W-1:0]        r_fp;
    integer               r_tot;
    reg                   r_exp, r_act;

    // Variables for TC10 (reset)
    integer rst_ok;

    // Variables for sanity check
    reg [FP_W-1:0]      san_fp;
    reg [MAN_TC_W-1:0]  san_tc;
    reg                 san_exp [1:16];
    integer             san_i, san_ok;

    // =========================================================================
    //                         REFERENCE MODEL
    // =========================================================================
    function [MAN_TC_W-1:0] f_mantc;
        input [FP_W-1:0] fp;
        reg               s;
        reg  [MAN_W:0]    mag;
        begin
            s       = fp[FP_W-1];
            mag     = {1'b1, fp[MAN_W-1:0]};
            f_mantc = s ? {1'b1, (~mag + 1'b1)}
                        : {1'b0, mag};
        end
    endfunction

    function f_bit_ref;
        input [FP_W-1:0]  fp;
        input [EXP_W-1:0] em, ei;
        input integer     c;
        reg   [EXP_W-1:0] e;
        reg               s;
        reg   [MAN_TC_W-1:0]  tc;
        integer           a; 
        begin
            s  = fp[FP_W-1];
            e  = fp[FP_W-2 -: EXP_W];
            tc = f_mantc(fp);
            a  = em - e;
            if (c <= a)
                f_bit_ref = s;
            else if (c <= a + MAN_TC_W)
                f_bit_ref = tc[MAN_TC_W-1 - (c - a - 1)];
            else
                f_bit_ref = 1'b0;
        end
    endfunction

    // =========================================================================
    //  Task: run_test
    // =========================================================================
    task run_test;
        input [EXP_W-1:0]       t_em;
        input [EXP_W-1:0]       t_ei;
        input [NUM_CH*FP_W-1:0] t_vec;
        begin
            tk_cyc = (t_em - t_ei) + MAN_W + 2;
            tk_err = 0;

            // ---------- Apply load on a negedge ----------
            @(negedge clk);
            emax_r   = t_em;
            emin_r   = t_ei;
            fp_vec_r = t_vec;
            load     = 1'b1;
            
            // <--- 新增：校验 total_cyc_out 的输出是否正确 --->
            #1; // 给组合逻辑一点响应时间
            if (total_cyc_out !== tk_cyc) begin
                $display("[FAIL] %s: total_cyc_out error! Expected %0d, Got %0d", tc_name, tk_cyc, total_cyc_out);
                tk_err = tk_err + 1;
            end

            @(negedge clk);  // crosses posedge P1: DUT latches, running<-1
            load = 1'b0;

            // ---------- Check every output cycle (cycles 1 .. tk_cyc) ----------
            for (tk_c = 1; tk_c <= tk_cyc; tk_c = tk_c + 1) begin
                @(posedge clk);
                #1;  // P2 for tk_c=1, P3 for tk_c=2, ...

                // bs_valid must be HIGH throughout valid output
                if (bs_valid !== 1'b1) begin
                    $display("[FAIL] %s: bs_valid LOW at cycle %0d/%0d", tc_name, tk_c, tk_cyc);
                    tk_err = tk_err + 1;
                end

                // bs_done: HIGH only on the last cycle
                if (tk_c == tk_cyc) begin
                    if (bs_done !== 1'b1) begin
                        $display("[FAIL] %s: bs_done missing on last cycle %0d", tc_name, tk_cyc);
                        tk_err = tk_err + 1;
                    end
                end else begin
                    if (bs_done === 1'b1) begin
                        $display("[FAIL] %s: bs_done early at cycle %0d/%0d", tc_name, tk_c, tk_cyc);
                        tk_err = tk_err + 1;
                    end
                end

                // Per-channel bit verification
                for (tk_ch = 0; tk_ch < NUM_CH; tk_ch = tk_ch + 1) begin
                    tk_exp_b = f_bit_ref(t_vec[tk_ch*FP_W +: FP_W], t_em, t_ei, tk_c);
                    tk_act_b = bs_out[tk_ch];
                    if (tk_act_b !== tk_exp_b) begin
                        $display("[FAIL] %s: ch=%0d cyc=%0d exp=%b got=%b  fp=0x%04h  em=%0d ei=%0d",
                                 tc_name, tk_ch, tk_c, tk_exp_b, tk_act_b,
                                 t_vec[tk_ch*FP_W +: FP_W], t_em, t_ei);
                        tk_err = tk_err + 1;
                    end
                end
            end

            // ---------- bs_valid must deassert one cycle after the last bit ----------
            @(posedge clk);
            #1;
            if (bs_valid !== 1'b0) begin
                $display("[FAIL] %s: bs_valid still HIGH after last cycle", tc_name);
                tk_err = tk_err + 1;
            end
            if (|bs_out !== 1'b0) begin
                $display("[FAIL] %s: bs_out not zero after last cycle (got %b)", tc_name, bs_out);
                tk_err = tk_err + 1;
            end

            // ---------- Print result ----------
            err_total = err_total + tk_err;
            if (tk_err == 0)
                $display("[PASS] %-36s  em=%02d ei=%02d cyc=%0d", tc_name, t_em, t_ei, tk_cyc);
            else
                $display("[FAIL] %-36s  (%0d error(s))", tc_name, tk_err);
        end
    endtask

    // =========================================================================
    //  Task: sanity_check
    // =========================================================================
    task sanity_check;
        begin
            san_fp = {1'b0, 5'd20, 10'b0000000000}; // 0x5000 = +32.0

            // Hand-computed expected bits (cycle 1 = MSB of aligned integer)
            san_exp[ 1] = 1'b0; // man_tc[11]: TC sign bit = 0  (positive)
            san_exp[ 2] = 1'b1; // man_tc[10]: hidden bit  = 1
            san_exp[ 3] = 1'b0; // man_tc[9] : M[9] = 0
            san_exp[ 4] = 1'b0;
            san_exp[ 5] = 1'b0;
            san_exp[ 6] = 1'b0;
            san_exp[ 7] = 1'b0;
            san_exp[ 8] = 1'b0;
            san_exp[ 9] = 1'b0;
            san_exp[10] = 1'b0;
            san_exp[11] = 1'b0;
            san_exp[12] = 1'b0; // man_tc[0] : M[0] = 0
            san_exp[13] = 1'b0; // zero-pad 1
            san_exp[14] = 1'b0; // zero-pad 2
            san_exp[15] = 1'b0; // zero-pad 3
            san_exp[16] = 1'b0; // zero-pad 4

            san_tc = f_mantc(san_fp);
            san_ok = 1;
            $display("--- SANITY: FP16=0x%04h (+32.0)  Emax=20 Emin=16 ---", san_fp);
            $display("    f_mantc = 12'b%b  (expected 12'b010000000000)", san_tc);
            if (san_tc !== 12'b010000000000) begin
                $display("[SANITY FAIL] f_mantc wrong! got 12'h%03h", san_tc);
                san_ok  = 0;
                err_total = err_total + 1;
            end

            for (san_i = 1; san_i <= 16; san_i = san_i + 1) begin
                if (f_bit_ref(san_fp, 5'd20, 5'd16, san_i) !== san_exp[san_i]) begin
                    $display("[SANITY FAIL] cyc=%02d hand=%b ref=%b",
                             san_i, san_exp[san_i], f_bit_ref(san_fp, 5'd20, 5'd16, san_i));
                    san_ok  = 0;
                    err_total = err_total + 1;
                end
            end

            if (san_ok)
                $display("[SANITY PASS] Reference model matches hand computation");
            $display("");
        end
    endtask

    // =========================================================================
    // Main test sequence
    // =========================================================================
    initial begin
        $dumpfile("tb_aligner.vcd");
        $dumpvars(0, tb_aligner);

        err_total = 0;
        rng_seed  = 42;

        rst_n    = 1'b0;
        load     = 1'b0;
        emax_r   = {EXP_W{1'b0}};
        emin_r   = {EXP_W{1'b0}};
        fp_vec_r = {(NUM_CH*FP_W){1'b0}};
        
        // Reset sequence
        repeat (4) @(posedge clk);
        @(negedge clk); rst_n = 1'b1;
        repeat (2) @(posedge clk);

        // ---- SANITY: reference model self-check ----
        sanity_check();
        
        $display("========== Directed Tests ==========");

        // ---- TC1: All positive, E=Emax  (0 align + 12 mant + 4 zero = 16 cyc) ----
        tc_name = "TC1_all_pos_E=Emax";
        run_test(5'd20, 5'd16,
            { {1'b0, 5'd20, 10'b1111111111},
              {1'b0, 5'd20, 10'b0101010101},
              {1'b0, 5'd20, 10'b1000000000},
              {1'b0, 5'd20, 10'b0000000000} });
              
        // ---- TC2: All negative, E=Emin  (4 align + 12 mant + 0 zero = 16 cyc) ----
        tc_name = "TC2_all_neg_E=Emin";
        run_test(5'd20, 5'd16,
            { {1'b1, 5'd16, 10'b1111111111},
              {1'b1, 5'd16, 10'b0101010101},
              {1'b1, 5'd16, 10'b1000000000},
              {1'b1, 5'd16, 10'b0000000000} });
              
        // ---- TC3: Mixed sign and exponent ----
        tc_name = "TC3_mixed_sign_exp";
        run_test(5'd20, 5'd16,
            { {1'b1, 5'd20, 10'b1100110011},
              {1'b0, 5'd19, 10'b0001000100},
              {1'b1, 5'd17, 10'b1010101010},
              {1'b0, 5'd16, 10'b0110011001} });
              
        // ---- TC4: Emax==Emin  (total_cyc = 12, mantissa only) ----
        tc_name = "TC4_Emax=Emin_12cyc";
        run_test(5'd15, 5'd15,
            { {1'b1, 5'd15, 10'b1111000011},
              {1'b0, 5'd15, 10'b0000111100},
              {1'b1, 5'd15, 10'b1100110011},
              {1'b0, 5'd15, 10'b0011001100} });
              
        // ---- TC5: Wide window Emax-Emin=10  (total_cyc = 22) ----
        tc_name = "TC5_wide_window_22cyc";
        run_test(5'd15, 5'd5,
            { {1'b0, 5'd15, 10'b1010101010},
              {1'b0, 5'd12, 10'b0011110000},
              {1'b1, 5'd10, 10'b1100001111},
              {1'b0, 5'd5,  10'b0101010101} });

        // ---- TC6: Maximum mantissa M=1111111111 ----
        tc_name = "TC6_max_mantissa";
        run_test(5'd10, 5'd8,
            { {1'b1, 5'd10, 10'b1111111111},
              {1'b0, 5'd10, 10'b1111111111},
              {1'b1, 5'd8,  10'b1111111111},
              {1'b0, 5'd8,  10'b1111111111} });
              
        // ---- TC7: Zero mantissa M=0  (verifies 2's-comp of exact power-of-2) ----
        tc_name = "TC7_zero_mantissa";
        run_test(5'd16, 5'd14,
            { {1'b1, 5'd16, 10'b0000000000},
              {1'b0, 5'd15, 10'b0000000000},
              {1'b1, 5'd14, 10'b0000000000},
              {1'b0, 5'd16, 10'b0000000000} });
              
        // ---- TC8: Boundary exponents E=Emax-1 and E=Emin+1 ----
        tc_name = "TC8_1cyc_boundaries";
        run_test(5'd20, 5'd16,
            { {1'b0, 5'd19, 10'b1010101010},
              {1'b1, 5'd19, 10'b0101010101},
              {1'b0, 5'd17, 10'b1111000011},
              {1'b1, 5'd17, 10'b0000111100} });
              
        // ---- TC9: Back-to-back vectors (no idle gap) ----
        tc_name = "TC9a_b2b_vec1";
        run_test(5'd18, 5'd16,
            { {1'b0, 5'd18, 10'b1010101010},
              {1'b1, 5'd17, 10'b0101010101},
              {1'b0, 5'd16, 10'b1111000011},
              {1'b1, 5'd18, 10'b0000111100} });
              
        tc_name = "TC9b_b2b_vec2";
        run_test(5'd18, 5'd16,
            { {1'b1, 5'd16, 10'b0011001100},
              {1'b0, 5'd18, 10'b1100110011},
              {1'b1, 5'd17, 10'b1001100110},
              {1'b0, 5'd16, 10'b0110011001} });

        // ---- TC10: Reset mid-operation then recovery ----
        $display("\n--- TC10: reset mid-operation ---");
        rst_ok = 1;
        @(negedge clk);
        emax_r   = 5'd20;
        emin_r   = 5'd16;
        fp_vec_r = { {1'b0,5'd18,10'b1010101010},
                     {1'b1,5'd17,10'b0101010101},
                     {1'b0,5'd16,10'b1111000011},
                     {1'b1,5'd18,10'b0000111100} };
        load = 1'b1;
        @(negedge clk); load = 1'b0;

        repeat (5) @(posedge clk);

        @(negedge clk); rst_n = 1'b0;
        @(posedge clk);
        #1;

        if (bs_valid !== 1'b0) begin
            $display("[FAIL] TC10a: bs_valid=%b after reset (expected 0)", bs_valid);
            err_total = err_total + 1; rst_ok = 0;
        end
        if (bs_done !== 1'b0) begin
            $display("[FAIL] TC10a: bs_done=%b  after reset (expected 0)", bs_done);
            err_total = err_total + 1; rst_ok = 0;
        end
        if (|bs_out !== 1'b0) begin
            $display("[FAIL] TC10a: bs_out=%b after reset (expected 0)", bs_out);
            err_total = err_total + 1; rst_ok = 0;
        end
        if (rst_ok)
            $display("[PASS] TC10a: all outputs cleared on rst_n=0");
            
        @(negedge clk); rst_n = 1'b1;
        repeat (2) @(posedge clk);

        tc_name = "TC10b_post_reset";
        run_test(5'd18, 5'd16,
            { {1'b0, 5'd18, 10'b1100110011},
              {1'b1, 5'd17, 10'b0011001100},
              {1'b0, 5'd16, 10'b1010101010},
              {1'b1, 5'd18, 10'b0101010101} });

        // ---- TC11: Randomised — 30 vectors, seed=42 ----
        $display("\n--- TC11: randomised test  30 vectors x %0d channels ---", NUM_CH);
        r_errs = 0;

        for (rv = 0; rv < 30; rv = rv + 1) begin
            r_emin = ({$random(rng_seed)} % 20) + 1;
            r_emax = r_emin + ({$random(rng_seed)} % 8) + 1;
            if (r_emax > 5'd30) r_emax = 5'd30;
            
            r_vec = {(NUM_CH*FP_W){1'b0}};
            for (rch = 0; rch < NUM_CH; rch = rch + 1) begin
                r_fp[FP_W-1]           = $random(rng_seed) % 2;
                r_fp[FP_W-2 -: EXP_W]  = r_emin + ($random(rng_seed) % (r_emax - r_emin + 1));
                r_fp[MAN_W-1:0]        = $random(rng_seed);
                r_vec[rch*FP_W +: FP_W] = r_fp;
            end

            r_tot = (r_emax - r_emin) + MAN_W + 2;
            
            @(negedge clk);
            emax_r   = r_emax;
            emin_r   = r_emin;
            fp_vec_r = r_vec;
            load     = 1'b1;
            
            // <--- 新增：在随机测试中也校验 total_cyc_out --->
            #1;
            if (total_cyc_out !== r_tot) begin
                $display("[FAIL] TC11 v=%0d: total_cyc_out error! Expected %0d, Got %0d", rv, r_tot, total_cyc_out);
                r_errs = r_errs + 1;
            end

            @(negedge clk); load = 1'b0;
            
            for (rc = 1; rc <= r_tot; rc = rc + 1) begin
                @(posedge clk);
                #1;
                for (rch = 0; rch < NUM_CH; rch = rch + 1) begin
                    r_exp = f_bit_ref(r_vec[rch*FP_W +: FP_W], r_emax, r_emin, rc);
                    r_act = bs_out[rch];
                    if (r_act !== r_exp) begin
                        $display("[FAIL] TC11 v=%0d ch=%0d cyc=%0d exp=%b got=%b  fp=0x%04h  em=%0d ei=%0d",
                                 rv, rch, rc, r_exp, r_act, r_vec[rch*FP_W +: FP_W], r_emax, r_emin);
                        r_errs = r_errs + 1;
                    end
                end
            end
            @(posedge clk);
            #1;   // one idle cycle between vectors
        end

        err_total = err_total + r_errs;
        if (r_errs == 0)
            $display("[PASS] TC11_randomised: 30 vectors, 0 errors");
        else
            $display("[FAIL] TC11_randomised: %0d error(s)", r_errs);

        // ---- Final summary ----
        $display("\n==========================================");
        if (err_total == 0)
            $display("  SIMULATION COMPLETE - ALL TESTS PASSED");
        else
            $display("  SIMULATION COMPLETE - TOTAL FAILURES: %0d", err_total);
        $display("==========================================\n");

        $finish;
    end

    // =========================================================================
    // Watchdog: abort if simulation hangs
    // =========================================================================
    initial begin
        #2_000_000;
        $display("[TIMEOUT] Simulation exceeded 2 ms limit");
        $finish;
    end

endmodule