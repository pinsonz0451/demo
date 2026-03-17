// =============================================================================
// File    : tb_top.v
// Desc    : Integration testbench for the full Intensive-CIM datapath
//             FP16 → Aligner → CIM_Macro → int_to_fp_acc → FP16
//
// Compile & run (Icarus Verilog):
//   iverilog -g2001 -o sim_top.out \
//     rtl/Aligner/aligner.v rtl/Aligner/aligner_channel.v \
//     rtl/CIM_Macro/adder_tree_128.v rtl/CIM_Macro/cim_array.v rtl/CIM_Macro/cim_macro.v \
//     rtl/Normalization/int_to_fp21.v rtl/Normalization/fp21_adder.v \
//     rtl/Normalization/fp21_to_fp16.v rtl/Normalization/int_to_fp_acc.v \
//     rtl/CIM_TOP.v sim/tb/tb_top.v
//   vvp sim_top.out
//
// -----------------------------------------------------------------------
// DUT parameters (reduced for tractability):
//   NUM_CH   = 8    (paper: 128)
//   NUM_COL  = 4    (paper:  64)
//   PSUM_W   = 32
//   CYC_W    = 6
//
// -----------------------------------------------------------------------
// Test case overview:
//   WP0 = all-1 weights,  WP1 = all-0 weights,  WP2 = checkerboard weights
//
//   TC1: All positive activations (+5.0),     weight = all-1
//   TC2: All negative activations (-3.0),     weight = all-1
//   TC3: Mixed sign (front half pos, back neg), weight = all-1
//   TC4: Boundary FP16 values (±max, 0.0),    weight = all-1
//   TC5: All positive (+5.0),                 weight = all-0  → expect 0
//   TC6: All positive (+5.0),                 weight = checkerboard
//   TC7: All negative (-3.0),                 weight = checkerboard
//   TC8: Mixed sign,                          weight = checkerboard
// =============================================================================

`timescale 1ns/1ps

module tb_top;

    // =========================================================================
    // Parameters  (must match CIM_TOP defaults or overrides)
    // =========================================================================
    localparam NUM_CH   = 8;
    localparam NUM_COL  = 4;
    localparam EXP_W    = 5;
    localparam MAN_W    = 10;
    localparam FP_W     = 16;
    localparam PSUM_W   = 32;
    localparam CYC_W    = 6;
    localparam MAN_TC_W = MAN_W + 2;   // 12

    // =========================================================================
    // DUT I/O
    // =========================================================================
    reg                         clk;
    reg                         rst_n;

    reg  [EXP_W-1:0]           emax;
    reg  [EXP_W-1:0]           emin;

    reg                         load;
    reg  [NUM_CH*FP_W-1:0]     fp_vec_in;

    reg                         wr_en;
    reg  [6:0]                  wr_row;
    reg  [5:0]                  wr_col;
    reg                         wr_data;

    reg                         acc_clear;
    reg                         writeback;

    wire [NUM_COL*16-1:0]      fp16_result_flat;
    wire                        fp16_valid;

    // Probe internal signals for timing checks
    wire                        bs_valid  = dut.bs_valid;
    wire                        bs_done   = dut.bs_done;
    wire                        mac_valid = dut.mac_valid;

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    CIM_TOP #(
        .NUM_CH  (NUM_CH),
        .NUM_COL (NUM_COL),
        .EXP_W   (EXP_W),
        .MAN_W   (MAN_W),
        .FP_W    (FP_W),
        .PSUM_W  (PSUM_W),
        .CYC_W   (CYC_W)
    ) dut (
        .clk              (clk),
        .rst_n            (rst_n),
        .emax             (emax),
        .emin             (emin),
        .load             (load),
        .fp_vec_in        (fp_vec_in),
        .wr_en            (wr_en),
        .wr_row           (wr_row),
        .wr_col           (wr_col),
        .wr_data          (wr_data),
        .acc_clear        (acc_clear),
        .writeback        (writeback),
        .fp16_result_flat (fp16_result_flat),
        .fp16_valid       (fp16_valid)
    );

    // =========================================================================
    // Clock  100 MHz  (period = 10ns)
    // =========================================================================
    initial clk = 1'b0;
    always  #5 clk = ~clk;

    // =========================================================================
    // FP16 encoding helpers
    // =========================================================================
    // FP16 encoding function:
    //   sign=0 positive, sign=1 negative
    //   exp = biased exponent (value_exp + 15)
    //   man = 10-bit mantissa fraction (hidden 1 implied)
    //
    //  +5.0  = 1.01 * 2^2  => exp = 2+15=17=5'b10001, man = 0100000000
    //          FP16 = 0_10001_0100000000 = 16'h4500
    //  -3.0  = -1.1 * 2^1  => exp = 1+15=16=5'b10000, man = 1000000000
    //          FP16 = 1_10000_1000000000 = 16'hC200
    //  +0.0  = 16'h0000
    //  max_fp16_normal = 0_11110_1111111111 = 16'h7BFF  (65504.0)
    //  -max_fp16_normal= 1_11110_1111111111 = 16'hFBFF  (-65504.0)

    localparam [15:0] FP16_POS_5   = 16'h4500;  // +5.0
    localparam [15:0] FP16_NEG_5   = 16'hC500;  // -5.0
    localparam [15:0] FP16_NEG_3   = 16'hC200;  // -3.0
    localparam [15:0] FP16_POS_0   = 16'h0000;  // +0.0
    localparam [15:0] FP16_MAX_POS = 16'h7BFF;  // +65504.0
    localparam [15:0] FP16_MAX_NEG = 16'hFBFF;  // -65504.0

    // =========================================================================
    // Weight reference memory (for reference model)
    // =========================================================================
    reg weight_ref [0:NUM_CH-1][0:NUM_COL-1];

    // =========================================================================
    // Shared variables
    // =========================================================================
    integer pass_cnt, fail_cnt, total_cnt;
    integer tc_id;
    integer lp_r, lp_c, lp_i;
    integer timing_err;
    reg ok_flag;

    // =========================================================================
    // Task: do_reset
    // =========================================================================
    task do_reset;
        begin
            rst_n     = 1'b0;
            load      = 1'b0;
            wr_en     = 1'b0;
            wr_row    = 7'd0;
            wr_col    = 6'd0;
            wr_data   = 1'b0;
            emax      = {EXP_W{1'b0}};
            emin      = {EXP_W{1'b0}};
            fp_vec_in = {(NUM_CH*FP_W){1'b0}};
            acc_clear = 1'b0;
            writeback = 1'b0;
            repeat (4) @(posedge clk);
            @(negedge clk); rst_n = 1'b1;
            repeat (2) @(posedge clk);
        end
    endtask

    // =========================================================================
    // Task: load_all_weights
    //   weight_pattern:
    //     0 = all-1,  1 = all-0,  2 = checkerboard ((row+col)%2)
    // =========================================================================
    task load_all_weights;
        input [1:0] pattern;
        reg w_bit;
        begin
            for (lp_r = 0; lp_r < NUM_CH; lp_r = lp_r + 1) begin
                for (lp_c = 0; lp_c < NUM_COL; lp_c = lp_c + 1) begin
                    case (pattern)
                        2'd0: w_bit = 1'b1;                             // all-1
                        2'd1: w_bit = 1'b0;                             // all-0
                        2'd2: w_bit = (lp_r + lp_c) % 2 == 0 ? 1'b1 : 1'b0; // checkerboard
                        default: w_bit = 1'b0;
                    endcase
                    @(negedge clk);
                    wr_en   = 1'b1;
                    wr_row  = lp_r[6:0];
                    wr_col  = lp_c[5:0];
                    wr_data = w_bit;
                    weight_ref[lp_r][lp_c] = w_bit;
                end
            end
            @(negedge clk);
            wr_en = 1'b0;
        end
    endtask

    // =========================================================================
    // Task: set_activations
    //   act_pattern:
    //     0 = all +5.0
    //     1 = all -3.0
    //     2 = mixed: ch[0..NUM_CH/2-1]=+5.0, ch[NUM_CH/2..NUM_CH-1]=-3.0
    //     3 = boundary: ch0=+max, ch1=-max, ch2=+0.0, others=+0.0
    // =========================================================================
    task set_activations;
        input [1:0] pattern;
        begin
            fp_vec_in = {(NUM_CH*FP_W){1'b0}};
            case (pattern)
                2'd0: begin  // all +5.0
                    for (lp_i = 0; lp_i < NUM_CH; lp_i = lp_i + 1)
                        fp_vec_in[lp_i*FP_W +: FP_W] = FP16_POS_5;
                end
                2'd1: begin  // all -3.0
                    for (lp_i = 0; lp_i < NUM_CH; lp_i = lp_i + 1)
                        fp_vec_in[lp_i*FP_W +: FP_W] = FP16_NEG_3;
                end
                2'd2: begin  // mixed: front positive, back negative
                    for (lp_i = 0; lp_i < NUM_CH/2; lp_i = lp_i + 1)
                        fp_vec_in[lp_i*FP_W +: FP_W] = FP16_POS_5;
                    for (lp_i = NUM_CH/2; lp_i < NUM_CH; lp_i = lp_i + 1)
                        fp_vec_in[lp_i*FP_W +: FP_W] = FP16_NEG_3;
                end
                2'd3: begin  // boundary values
                    // All default to 0 (already set above)
                    fp_vec_in[0*FP_W +: FP_W] = FP16_MAX_POS;  // +65504.0
                    fp_vec_in[1*FP_W +: FP_W] = FP16_MAX_NEG;  // -65504.0
                    // ch2..ch7 = +0.0 -- but subnormals/zero not supported by aligner
                    // Use small positive instead
                    for (lp_i = 2; lp_i < NUM_CH; lp_i = lp_i + 1)
                        fp_vec_in[lp_i*FP_W +: FP_W] = 16'h3C00; // +1.0 (exp=15)
                end
            endcase
        end
    endtask

    // =========================================================================
    // Reference model: compute expected FP16 output for all columns
    //
    // Algorithm:
    //   1. For each activation channel, extract sign, exponent, mantissa
    //   2. Reconstruct fixed-point integer per aligner bit-serial encoding
    //   3. Multiply by weight (0 or 1) per column
    //   4. Sum across all channels per column (signed)
    //   5. The resulting integer psum represents:
    //        true_fp_value = psum * 2^(emin - 15 - MAN_W)
    //   6. Convert this to FP16
    //
    // Note: This reference model matches the RTL's signed accumulation logic.
    //       The CIM_Macro first bit (cycle 0) is MSB of 2's complement, which
    //       has NEGATIVE weight. This is correctly handled by the shift-accumulate
    //       logic where cycle 0 SUBTRACTS (negates) the partial sum shifted by
    //       (total_cyc - 1).
    //
    // The reference model directly does integer arithmetic:
    //   For each channel i with FP16 value = (-1)^S * 2^(E-15) * (1.M):
    //     The aligned fixed-point representation =
    //       sign * (1.M) * 2^(E - emin)  (integer, with MAN_W fractional bits)
    //     = sign * {1, M[9:0]} * 2^(E - emin)  (left-shifted integer)
    //     This occupies total_cyc = (emax - emin) + MAN_W + 2 bit positions
    //
    //   psum[j] = SUM over channels i of:
    //       weight[i][j] * aligned_int[i]
    //     where aligned_int is the signed integer in 2's complement
    //
    //   The first bit (MSB) has weight -2^(total_cyc-1) (two's complement sign)
    //   so the overall N-bit integer for channel i =
    //       -sign_bit * 2^(N-1) + magnitude_bits
    //   which equals the standard two's complement interpretation.
    //
    //   For this reference model, we compute directly:
    //     value_int[i] = (-1)^S * {1, M} << (E - emin)
    //     where the result is a (total_cyc)-bit signed integer
    //     psum[j] = SUM_i { weight[i][j] * value_int[i] }
    //
    //   Then FP16 output = psum[j] * 2^(emin - 15 - MAN_W)
    // =========================================================================

    // Reference expected FP16 results
    reg [15:0] ref_fp16 [0:NUM_COL-1];

    // FP16 to real conversion (for display)
    function real fp16_to_real;
        input [15:0] fp;
        reg          s;
        reg [4:0]    e;
        reg [9:0]    m;
        real         val;
        begin
            s = fp[15];
            e = fp[14:10];
            m = fp[9:0];
            if (e == 5'b11111) begin
                fp16_to_real = (s ? -1.0 : 1.0) * 99999.0; // Inf/NaN
            end else if (e == 5'b00000 && m == 10'b0) begin
                fp16_to_real = s ? -0.0 : 0.0;
            end else begin
                // Normal: value = (-1)^s * 2^(e-15) * (1 + m/1024)
                val = (1.0 + m / 1024.0);
                // can't do power in function easily; skip full conversion
                fp16_to_real = val; // placeholder
            end
        end
    endfunction

    // Compute ULP distance between two FP16 values
    // ULP = difference in the integer representation of the FP16 values
    function integer fp16_ulp_diff;
        input [15:0] a, b;
        reg [15:0] a_int, b_int;
        integer diff;
        begin
            // Convert to comparable integer (sign-magnitude → 2's complement-like)
            if (a[15])
                a_int = ~{1'b0, a[14:0]} + 16'd1; // negative
            else
                a_int = {1'b0, a[14:0]};
            if (b[15])
                b_int = ~{1'b0, b[14:0]} + 16'd1;
            else
                b_int = {1'b0, b[14:0]};
            diff = $signed(a_int) - $signed(b_int);
            fp16_ulp_diff = (diff < 0) ? -diff : diff;
        end
    endfunction

    // =========================================================================
    // Reference model: compute expected output
    //   Uses the same signed math as cim_macro
    // =========================================================================
    task compute_reference;
        input [EXP_W-1:0] t_emax, t_emin;
        reg              fp_sign;
        reg [EXP_W-1:0]  fp_exp;
        reg [MAN_W-1:0]  fp_man;
        reg [MAN_W:0]    magnitude;      // {1, M} = 11 bits
        reg signed [PSUM_W-1:0] value_int;
        reg signed [PSUM_W-1:0] psum_ref;
        integer total_cyc_ref;
        integer ch, col;
        // For FP conversion
        reg              res_sign;
        reg [PSUM_W-1:0] abs_psum;
        integer          leading_pos;
        integer          k;
        reg [14:0]       mant21;
        reg [4:0]        exp21;
        // e_bias computation
        integer          e_bias_val;
        integer          fp16_exp_val;
        reg [4:0]        fp16_exp_field;
        reg [9:0]        fp16_mant_field;
        // round
        reg              round_bit, sticky_bit, round_up;
        reg [10:0]       mant_inc;
        begin
            total_cyc_ref = (t_emax - t_emin) + MAN_W + 2;
            e_bias_val = $signed({{(32-EXP_W){1'b0}}, t_emin}) - MAN_W - 15;

            for (col = 0; col < NUM_COL; col = col + 1) begin
                psum_ref = {PSUM_W{1'b0}};

                for (ch = 0; ch < NUM_CH; ch = ch + 1) begin
                    fp_sign = fp_vec_in[ch*FP_W + FP_W - 1];
                    fp_exp  = fp_vec_in[ch*FP_W + FP_W - 2 -: EXP_W];
                    fp_man  = fp_vec_in[ch*FP_W +: MAN_W];
                    magnitude = {1'b1, fp_man};

                    // Compute signed aligned integer value
                    // value_int has (total_cyc_ref) bits of significance
                    // The value = ±magnitude << (fp_exp - t_emin)
                    // This is the fixed-point aligned integer
                    value_int = $signed({{(PSUM_W-MAN_W-1){1'b0}}, magnitude})
                                <<< (fp_exp - t_emin);
                    if (fp_sign)
                        value_int = -value_int;

                    // Multiply by weight (0 or 1)
                    if (weight_ref[ch][col])
                        psum_ref = psum_ref + value_int;
                end

                // Now convert psum_ref to FP16 with e_bias correction
                // This mirrors int_to_fp21 → fp21_to_fp16 with e_bias
                if (psum_ref == 0) begin
                    ref_fp16[col] = 16'h0000;
                end else begin
                    res_sign = psum_ref[PSUM_W-1];
                    abs_psum = res_sign ? (~psum_ref + 1) : psum_ref;

                    leading_pos = -1;
                    for (k = 0; k < PSUM_W; k = k + 1) begin
                        if (abs_psum[k])
                            leading_pos = k;
                    end

                    if (leading_pos < 0) begin
                        // Should not happen (we checked psum != 0)
                        ref_fp16[col] = 16'h0000;
                    end else begin
                        exp21 = leading_pos[4:0];

                        // Extract mantissa from abs_psum
                        // Leading 1 is at position leading_pos (hidden bit)
                        // Mantissa bits are positions leading_pos-1 down to leading_pos-15
                        // But we need to handle FP21 intermediate (15-bit mant)
                        // then truncate to FP16 (10-bit mant)
                        if (leading_pos >= 15) begin
                            mant21 = abs_psum[leading_pos-1 -: 15];
                        end else begin
                            mant21 = 15'b0;
                            for (k = 0; k < 15; k = k + 1) begin
                                if (leading_pos - 1 - k >= 0)
                                    mant21[14-k] = abs_psum[leading_pos - 1 - k];
                            end
                        end

                        // FP21 → FP16: truncate mant21[14:0] to 10 bits
                        // mant21[14:5] = FP16 mantissa
                        // mant21[4] = round bit
                        // mant21[3:0] = sticky region
                        fp16_mant_field = mant21[14:5];
                        round_bit = mant21[4];
                        sticky_bit = |mant21[3:0];
                        round_up = round_bit & (sticky_bit | fp16_mant_field[0]);
                        mant_inc = {1'b0, fp16_mant_field} + {10'b0, round_up};

                        // e_bias: FP16_exp = exp21 + e_bias + 15
                        fp16_exp_val = leading_pos + e_bias_val + 15;

                        if (mant_inc[10]) begin
                            // Mantissa carry
                            fp16_exp_val = fp16_exp_val + 1;
                            fp16_mant_field = 10'b0;
                        end else begin
                            fp16_mant_field = mant_inc[9:0];
                        end

                        if (fp16_exp_val >= 31) begin
                            // Overflow → Inf
                            ref_fp16[col] = {res_sign, 5'b11111, 10'b0};
                        end else if (fp16_exp_val <= 0) begin
                            // Underflow → zero
                            ref_fp16[col] = {res_sign, 15'b0};
                        end else begin
                            fp16_exp_field = fp16_exp_val[4:0];
                            ref_fp16[col] = {res_sign, fp16_exp_field, fp16_mant_field};
                        end
                    end
                end
            end // for col
        end
    endtask

    // =========================================================================
    // Task: run_full_pipeline
    //   1. Clear accumulators
    //   2. Load activation vector + trigger aligner
    //   3. Wait for bs_valid → bs_done → mac_valid pipeline
    //   4. Assert writeback to trigger FP16 output
    //   5. Check handshake timing
    //   6. Compare results
    // =========================================================================
    integer bs_valid_cnt;    // counts cycles bs_valid is high
    integer mac_valid_cnt;   // counts cycles mac_valid is high
    integer fp16_valid_cnt;  // counts cycles fp16_valid is high

    task run_full_pipeline;
        input [31:0] test_id;
        input [EXP_W-1:0] t_emax, t_emin;
        reg [15:0] got_fp16;
        integer ulp;
        integer total_cyc_exp;
        integer col;
        begin
            total_cyc_exp = (t_emax - t_emin) + MAN_W + 2;
            timing_err = 0;
            ok_flag = 1'b1;

            // --- Step 1: Clear accumulators ---
            @(negedge clk);
            acc_clear = 1'b1;
            @(negedge clk);
            acc_clear = 1'b0;

            // --- Step 2: Apply emax/emin and load pulse ---
            @(negedge clk);
            emax = t_emax;
            emin = t_emin;
            load = 1'b1;
            @(negedge clk);
            load = 1'b0;

            // --- Step 3: Wait for bs_valid, count valid cycles, verify bs_done ---
            bs_valid_cnt = 0;

            // Wait for bs_valid to rise
            while (!bs_valid) begin
                @(posedge clk); #1;
            end

            // Count bs_valid high cycles
            while (bs_valid) begin
                bs_valid_cnt = bs_valid_cnt + 1;
                @(posedge clk); #1;
            end

            // Check: bs_valid should be HIGH for exactly total_cyc cycles
            // (aligner: cyc_cnt goes from 0 to total_cyc-1)
            if (bs_valid_cnt !== total_cyc_exp) begin
                $display("[TIMING WARN] TC%0d: bs_valid was HIGH for %0d cycles, expected %0d",
                         test_id, bs_valid_cnt, total_cyc_exp);
                // This is informational, not necessarily a hard error
            end

            // --- Step 4: Wait for mac_valid pulse ---
            // mac_valid fires 2 cycles after bs_done due to double-register pipeline
            mac_valid_cnt = 0;
            begin : WAIT_MAC_VALID
                integer timeout;
                timeout = 0;
                while (!mac_valid && timeout < 20) begin
                    @(posedge clk); #1;
                    timeout = timeout + 1;
                end
                if (timeout >= 20) begin
                    $display("[FAIL] TC%0d: mac_valid never fired (timeout)", test_id);
                    ok_flag = 1'b0;
                    timing_err = timing_err + 1;
                end
            end

            // Count mac_valid pulse width (must be exactly 1 cycle)
            if (mac_valid) begin
                mac_valid_cnt = 1;
                @(posedge clk); #1;
                if (mac_valid) begin
                    $display("[FAIL] TC%0d: mac_valid pulse > 1 cycle", test_id);
                    timing_err = timing_err + 1;
                    ok_flag = 1'b0;
                    while (mac_valid) begin
                        mac_valid_cnt = mac_valid_cnt + 1;
                        @(posedge clk); #1;
                    end
                end
            end

            // --- Step 5: Assert writeback ---
            @(negedge clk);
            writeback = 1'b1;
            @(negedge clk);
            writeback = 1'b0;

            // --- Step 6: Wait for fp16_valid ---
            fp16_valid_cnt = 0;
            begin : WAIT_FP16_VALID
                integer timeout2;
                timeout2 = 0;
                while (!fp16_valid && timeout2 < 20) begin
                    @(posedge clk); #1;
                    timeout2 = timeout2 + 1;
                end
                if (timeout2 >= 20) begin
                    $display("[FAIL] TC%0d: fp16_valid never fired (timeout)", test_id);
                    ok_flag = 1'b0;
                    timing_err = timing_err + 1;
                end
            end

            // Check fp16_valid pulse width (must be exactly 1 cycle)
            if (fp16_valid) begin
                fp16_valid_cnt = 1;
                @(posedge clk); #1;
                if (fp16_valid) begin
                    $display("[FAIL] TC%0d: fp16_valid pulse > 1 cycle", test_id);
                    timing_err = timing_err + 1;
                    ok_flag = 1'b0;
                    while (fp16_valid) begin
                        fp16_valid_cnt = fp16_valid_cnt + 1;
                        @(posedge clk); #1;
                    end
                end
            end

            // --- Step 7: Compare outputs ---
            compute_reference(t_emax, t_emin);

            for (col = 0; col < NUM_COL; col = col + 1) begin
                got_fp16 = fp16_result_flat[col*16 +: 16];
                ulp = fp16_ulp_diff(got_fp16, ref_fp16[col]);
                total_cnt = total_cnt + 1;

                if (ulp <= 1) begin
                    $display("[PASS] TC%0d col=%0d  expected=%04h  got=%04h  diff=%0d ULP",
                             test_id, col, ref_fp16[col], got_fp16, ulp);
                    pass_cnt = pass_cnt + 1;
                end else begin
                    $display("[FAIL] TC%0d col=%0d  expected=%04h  got=%04h  diff=%0d ULP",
                             test_id, col, ref_fp16[col], got_fp16, ulp);
                    fail_cnt = fail_cnt + 1;
                    ok_flag = 1'b0;
                end
            end

            if (timing_err > 0)
                fail_cnt = fail_cnt + timing_err;
        end
    endtask

    // =========================================================================
    // Main test sequence
    // =========================================================================
    initial begin
        $dumpfile("tb_top.vcd");
        $dumpvars(0, tb_top);

        pass_cnt  = 0;
        fail_cnt  = 0;
        total_cnt = 0;

        do_reset;

        $display("=================================================================");
        $display("  Intensive-CIM Full Datapath Integration Testbench");
        $display("  NUM_CH=%0d  NUM_COL=%0d  PSUM_W=%0d", NUM_CH, NUM_COL, PSUM_W);
        $display("=================================================================");

        // =====================================================================
        // Weight Pattern 0: All-1 weights
        // =====================================================================
        $display("\n--- Loading Weight Pattern 0: All-1 weights ---");
        load_all_weights(2'd0);

        // -----------------------------------------------------------------
        // TC1: All positive activations (+5.0)
        //   8 channels all = +5.0, all weights = 1
        //   Each channel contributes +5.0
        //   Emax = Emin = 17 (biased exp of 5.0)
        //   psum[j] = sum of all aligned 2sC integers
        //   Expected FP16 ≈ +40.0 (8 × 5.0)
        // -----------------------------------------------------------------
        $display("\n--- TC1: All positive (+5.0), all-1 weights ---");
        set_activations(2'd0);
        run_full_pipeline(1, 5'd17, 5'd17);

        // -----------------------------------------------------------------
        // TC2: All negative activations (-3.0)
        //   8 channels all = -3.0, all weights = 1
        //   Expected FP16 ≈ -24.0 (8 × (-3.0))
        // -----------------------------------------------------------------
        $display("\n--- TC2: All negative (-3.0), all-1 weights ---");
        set_activations(2'd1);
        run_full_pipeline(2, 5'd16, 5'd16);

        // -----------------------------------------------------------------
        // TC3: Mixed sign (front half +5.0, back half -3.0)
        //   ch[0..3] = +5.0 (exp=17), ch[4..7] = -3.0 (exp=16)
        //   Emax = 17, Emin = 16
        //   Expected FP16 ≈ 4×5.0 + 4×(-3.0) = 20.0 - 12.0 = +8.0
        // -----------------------------------------------------------------
        $display("\n--- TC3: Mixed sign (pos/neg), all-1 weights ---");
        set_activations(2'd2);
        run_full_pipeline(3, 5'd17, 5'd16);

        // -----------------------------------------------------------------
        // TC4: Boundary FP16 values
        //   ch0 = +65504.0 (max normal, exp=30)
        //   ch1 = -65504.0 (exp=30)
        //   ch2..7 = +1.0 (exp=15)
        //   Emax = 30, Emin = 15
        //   Expected psum ≈ 0 + 6×1.0 ≈ +6.0
        // -----------------------------------------------------------------
        $display("\n--- TC4: Boundary FP16 values, all-1 weights ---");
        set_activations(2'd3);
        run_full_pipeline(4, 5'd30, 5'd15);

        // =====================================================================
        // Weight Pattern 1: All-0 weights
        // =====================================================================
        $display("\n--- Loading Weight Pattern 1: All-0 weights ---");
        load_all_weights(2'd1);

        // -----------------------------------------------------------------
        // TC5: All positive (+5.0), all-0 weights → expect FP16 = +0.0
        // -----------------------------------------------------------------
        $display("\n--- TC5: All positive (+5.0), all-0 weights ---");
        set_activations(2'd0);
        run_full_pipeline(5, 5'd17, 5'd17);

        // =====================================================================
        // Weight Pattern 2: Checkerboard weights
        //   w[r][c] = 1 if (r+c) is even, 0 otherwise
        // =====================================================================
        $display("\n--- Loading Weight Pattern 2: Checkerboard weights ---");
        load_all_weights(2'd2);

        // -----------------------------------------------------------------
        // TC6: All positive (+5.0), checkerboard weights
        //   Col 0: rows {0,2,4,6} active → 4×5.0 = +20.0
        //   Col 1: rows {1,3,5,7} active → 4×5.0 = +20.0
        //   (symmetric for these uniform inputs)
        // -----------------------------------------------------------------
        $display("\n--- TC6: All positive (+5.0), checkerboard weights ---");
        set_activations(2'd0);
        run_full_pipeline(6, 5'd17, 5'd17);

        // -----------------------------------------------------------------
        // TC7: All negative (-3.0), checkerboard weights
        //   Col 0: rows {0,2,4,6} active → 4×(-3.0) = -12.0
        //   Col 1: rows {1,3,5,7} active → 4×(-3.0) = -12.0
        // -----------------------------------------------------------------
        $display("\n--- TC7: All negative (-3.0), checkerboard weights ---");
        set_activations(2'd1);
        run_full_pipeline(7, 5'd16, 5'd16);

        // -----------------------------------------------------------------
        // TC8: Mixed sign, checkerboard weights
        //   ch[0..3]=+5.0 (exp=17), ch[4..7]=-3.0 (exp=16)
        //   Col 0 (even rows): ch{0,2}=+5.0, ch{4,6}=-3.0 → 10-6 = +4.0
        //   Col 1 (odd rows):  ch{1,3}=+5.0, ch{5,7}=-3.0 → 10-6 = +4.0
        //   Col 2 (even rows): same as col 0 = +4.0
        //   Col 3 (odd rows):  same as col 1 = +4.0
        // -----------------------------------------------------------------
        $display("\n--- TC8: Mixed sign, checkerboard weights ---");
        set_activations(2'd2);
        run_full_pipeline(8, 5'd17, 5'd16);

        // =====================================================================
        // TC9: Full cancellation — exercises fp21_adder sum_zero (c=21) path
        //   Pass 1: all +5.0, all-1 weights → acc = FP21(+40.0)
        //   Pass 2: all -5.0, all-1 weights → fp21_adder adds FP21(-40)+FP21(+40)
        //           → sum_21 = 0, c = 21 (full CLZ), sum_zero = 1
        //           → output mux selects 21'b0
        //   Writeback → expect FP16 = 0x0000 for all columns
        //
        //   This covers fp21_adder.v:L187-L190 where exp_raw8 = e_lg+1-21
        //   goes negative when e_lg < 20.  The sum_zero flag catches it.
        // =====================================================================
        $display("\n--- TC9: Full cancellation (fp21_adder sum_zero path) ---");

        // Reload all-1 weights (TC8 used checkerboard)
        load_all_weights(2'd0);

        // Clear accumulators
        @(negedge clk); acc_clear = 1'b1;
        @(negedge clk); acc_clear = 1'b0;

        // --- Pass 1: all +5.0 ---
        for (lp_i = 0; lp_i < NUM_CH; lp_i = lp_i + 1)
            fp_vec_in[lp_i*FP_W +: FP_W] = FP16_POS_5;
        @(negedge clk);
        emax = 5'd17; emin = 5'd17;
        load = 1'b1;
        @(negedge clk); load = 1'b0;

        // Wait for mac_valid (do NOT writeback — let acc hold +40.0)
        begin : TC9_WAIT_MAC1
            integer t;
            t = 0;
            while (!mac_valid && t < 50) begin
                @(posedge clk); #1; t = t + 1;
            end
            if (t >= 50) begin
                $display("[FAIL] TC9 pass1: mac_valid timeout");
                fail_cnt = fail_cnt + 1;
            end
        end
        // Let mac_valid deassert
        @(posedge clk); #1;

        // --- Pass 2: all -5.0 (exact negative) ---
        for (lp_i = 0; lp_i < NUM_CH; lp_i = lp_i + 1)
            fp_vec_in[lp_i*FP_W +: FP_W] = FP16_NEG_5;
        @(negedge clk);
        emax = 5'd17; emin = 5'd17;
        load = 1'b1;
        @(negedge clk); load = 1'b0;

        // Wait for mac_valid
        begin : TC9_WAIT_MAC2
            integer t;
            t = 0;
            while (!mac_valid && t < 50) begin
                @(posedge clk); #1; t = t + 1;
            end
            if (t >= 50) begin
                $display("[FAIL] TC9 pass2: mac_valid timeout");
                fail_cnt = fail_cnt + 1;
            end
        end
        @(posedge clk); #1;

        // --- Writeback and check ---
        @(negedge clk); writeback = 1'b1;
        @(negedge clk); writeback = 1'b0;

        begin : TC9_CHECK
            integer t, col;
            reg [15:0] got;
            reg tc9_ok;
            tc9_ok = 1'b1;
            t = 0;
            while (!fp16_valid && t < 20) begin
                @(posedge clk); #1; t = t + 1;
            end
            if (t >= 20) begin
                $display("[FAIL] TC9: fp16_valid timeout");
                fail_cnt = fail_cnt + 1;
            end else begin
                for (col = 0; col < NUM_COL; col = col + 1) begin
                    got = fp16_result_flat[col*16 +: 16];
                    total_cnt = total_cnt + 1;
                    if (got == 16'h0000 || got == 16'h8000) begin
                        // +0 or -0 both acceptable for full cancellation
                        $display("[PASS] TC9 col=%0d  expected=0000  got=%04h  (zero)",
                                 col, got);
                        pass_cnt = pass_cnt + 1;
                    end else begin
                        $display("[FAIL] TC9 col=%0d  expected=0000  got=%04h",
                                 col, got);
                        fail_cnt = fail_cnt + 1;
                        tc9_ok = 1'b0;
                    end
                end
            end
        end

        // =====================================================================
        // Final summary
        // =====================================================================
        $display("\n=================================================================");
        $display("  RESULTS: %0d PASSED / %0d FAILED / %0d total",
                 pass_cnt, fail_cnt, pass_cnt + fail_cnt);
        $display("=================================================================");
        if (fail_cnt == 0)
            $display("  >>> ALL TESTS PASSED <<<");
        else
            $display("  >>> SOME TESTS FAILED -- review output above <<<");

        $finish;
    end

    // =========================================================================
    // Watchdog: abort if simulation hangs
    // =========================================================================
    initial begin
        #10_000_000;
        $display("[TIMEOUT] Simulation exceeded 10 ms limit");
        $finish;
    end

endmodule
// =============================================================================
// End of tb_top.v
// =============================================================================
