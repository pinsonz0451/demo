// =============================================================================
// File    : fp21_adder.v
// Module  : fp21_adder
// Project : Intensive-CIM Core  (INT-to-FP accumulation, Fig. 8)
//
// Function
// --------
//   Adds two FP21 numbers.  Result = A + B in FP21 format.
//
// FP21 Format  {sign[1], exp[4:0], mant[14:0]}
//   Value  : (-1)^S × 2^E × (1.M)   where E is the ABSOLUTE exponent.
//   Zero   : 21'b0  (unique representation)
//
// Algorithm  (IEEE 754-like, faithful rounding)
// -----
//   1. Identify the operand with larger |magnitude|.
//   2. Restore hidden bits; extend significands with 4 guard bits.
//        sig_x = {1'b1, mant_x[14:0], 4'b0}   [20-bit]
//        actual value = sig_x × 2^(e_x − 19)
//   3. Align the smaller significand (right-shift by exponent difference).
//   4. Add (same sign) or subtract (opposite sign) using 21-bit arithmetic
//      (extra bit catches the carry-out from addition).
//   5. Count leading zeros (CLZ) of the 21-bit sum to find normalization shift.
//   6. Normalize using a 40-bit window:
//        sum_wide = {sum_21, 19'b0}
//        norm_wide = shift so leading 1 is at bit 38
//        mant[14:0] = norm_wide[37:23]
//        guard       = norm_wide[22]
//        sticky      = |norm_wide[21:0]
//   7. Round-to-nearest, ties-to-even.
//   8. Compute result exponent:
//        result_exp = e_large + 1 − c  (c = CLZ of 21-bit sum)
//
// Special cases handled
//   · Either input zero  → return the other operand unchanged.
//   · Full cancellation  → return 21'b0.
//   · Exponent overflow (> 31) → saturate to {sign, 5'b11111, 15'b0}.
//   · Exponent underflow (< 0) → flush to zero.
//
// Latency    : combinational (0 clock cycles)
// =============================================================================
`timescale 1ns/1ps

module fp21_adder (
    input  [20:0] a,   // FP21 operand A
    input  [20:0] b,   // FP21 operand B
    output [20:0] y    // FP21 result  A + B
);

    // =========================================================================
    // Field extraction
    // =========================================================================
    wire        s_a = a[20],    s_b = b[20];
    wire [4:0]  e_a = a[19:15], e_b = b[19:15];
    wire [14:0] m_a = a[14:0],  m_b = b[14:0];
    wire zero_a = (a == 21'b0);
    wire zero_b = (b == 21'b0);

    // =========================================================================
    // Extended significands  {hidden_1, mantissa[14:0], guard[3:0]}  = 20 bits
    //
    //   The 4 guard bits (initialised to 0) carry sub-LSB information through
    //   the alignment shift, improving rounding accuracy.
    //   actual value of sig_x  =  sig_x × 2^(e_x − 19)
    // =========================================================================
    wire [19:0] sig_a = {1'b1, m_a, 4'b0};
    wire [19:0] sig_b = {1'b1, m_b, 4'b0};

    // =========================================================================
    // Identify operand with larger |magnitude|
    //   |A| ≥ |B|  iff  e_a > e_b,
    //               or  e_a == e_b  AND  m_a >= m_b
    // =========================================================================
    wire a_ge_b = (e_a > e_b) | ((e_a == e_b) & (m_a >= m_b));

    wire [4:0]  e_lg  = a_ge_b ? e_a : e_b;
    wire [19:0] sig_lg = a_ge_b ? sig_a : sig_b;
    wire        s_lg  = a_ge_b ? s_a   : s_b;

    wire [4:0]  e_sm  = a_ge_b ? e_b   : e_a;
    wire [19:0] sig_sm = a_ge_b ? sig_b : sig_a;
    wire        s_sm  = a_ge_b ? s_b   : s_a;

    // =========================================================================
    // Align smaller operand
    //   Right-shift sig_sm by the exponent difference.
    //   Capped at 20: a shift ≥ 20 moves the entire significand into the sticky
    //   region (result is 0 after the shift).
    // =========================================================================
    wire [4:0]  ediff     = e_lg - e_sm;
    wire [19:0] sig_sm_al = (ediff >= 5'd20) ? 20'b0 : (sig_sm >> ediff);

    // =========================================================================
    // Signed 21-bit addition  (extra bit absorbs carry from same-sign add)
    //   same_sign : ADD magnitudes; bit 20 may be set (overflow into 2^20).
    //   diff_sign : SUBTRACT; result ≥ 0 because |lg| ≥ |sm| after alignment.
    // =========================================================================
    wire same_sign = (s_lg == s_sm);

    wire [20:0] sum_21 = same_sign ?
                         ({1'b0, sig_lg} + {1'b0, sig_sm_al}) :
                         ({1'b0, sig_lg} - {1'b0, sig_sm_al});

    // Result sign = sign of the operand with larger |magnitude|.
    wire s_res = s_lg;

    // =========================================================================
    // CLZ of 21-bit sum  (for normalization)
    //   Returns 0..20 for non-zero sum; 21 for sum == 0 (full cancellation).
    // =========================================================================
    function [4:0] clz21;
        input [20:0] x;
        casex (x)
            21'b1xxxxxxxxxxxxxxxxxxxx: clz21 = 5'd0;
            21'b01xxxxxxxxxxxxxxxxxxx: clz21 = 5'd1;
            21'b001xxxxxxxxxxxxxxxxxx: clz21 = 5'd2;
            21'b0001xxxxxxxxxxxxxxxxx: clz21 = 5'd3;
            21'b00001xxxxxxxxxxxxxxxx: clz21 = 5'd4;
            21'b000001xxxxxxxxxxxxxxx: clz21 = 5'd5;
            21'b0000001xxxxxxxxxxxxxx: clz21 = 5'd6;
            21'b00000001xxxxxxxxxxxxx: clz21 = 5'd7;
            21'b000000001xxxxxxxxxxxx: clz21 = 5'd8;
            21'b0000000001xxxxxxxxxxx: clz21 = 5'd9;
            21'b00000000001xxxxxxxxxx: clz21 = 5'd10;
            21'b000000000001xxxxxxxxx: clz21 = 5'd11;
            21'b0000000000001xxxxxxxx: clz21 = 5'd12;
            21'b00000000000001xxxxxxx: clz21 = 5'd13;
            21'b000000000000001xxxxxx: clz21 = 5'd14;
            21'b0000000000000001xxxxx: clz21 = 5'd15;
            21'b00000000000000001xxxx: clz21 = 5'd16;
            21'b000000000000000001xxx: clz21 = 5'd17;
            21'b0000000000000000001xx: clz21 = 5'd18;
            21'b00000000000000000001x: clz21 = 5'd19;
            21'b000000000000000000001: clz21 = 5'd20;
            default:                   clz21 = 5'd21;   // sum == 0
        endcase
    endfunction

    wire [4:0] c        = clz21(sum_21);
    wire       sum_zero = (c == 5'd21);

    // =========================================================================
    // Normalization using a 40-bit sliding window
    //
    //   sum_wide = {sum_21, 19'b0}
    //     In sum_wide, the leading 1 of sum_21 is at bit (39 − c).
    //     We want the leading 1 at bit 38 (so bit 38 becomes the hidden bit).
    //
    //   Required shift:
    //     c == 0 (carry at bit 20):  shift RIGHT by 1   → leading 1 at 38
    //     c >= 1                  :  shift LEFT  by c−1 → leading 1 at 38
    //
    //   After shift:
    //     norm_wide[38]    = 1        hidden bit  (NOT stored)
    //     norm_wide[37:23] = 15 bits  mantissa
    //     norm_wide[22]    = G        guard bit   (first bit after mantissa)
    //     norm_wide[21:0]  = 22 bits  sticky region
    // =========================================================================
    wire [39:0] sum_wide  = {sum_21, 19'b0};

    wire [39:0] norm_wide = (c == 5'd0) ? (sum_wide >> 1) :
                                          (sum_wide << (c - 5'd1));

    wire [14:0] mant_norm  = norm_wide[37:23];
    wire        guard_bit  = norm_wide[22];
    wire        sticky_all = |norm_wide[21:0];

    // =========================================================================
    // Round-to-nearest, ties-to-even
    //   Round up iff:  G = 1  AND  (sticky ≠ 0  OR  LSB(mantissa) = 1)
    // =========================================================================
    wire round_up = guard_bit & (sticky_all | mant_norm[0]);

    wire [15:0] mant_inc   = {1'b0, mant_norm} + {15'b0, round_up};
    wire        mant_carry = mant_inc[15];

    // =========================================================================
    // Result exponent (8-bit signed for safe over/underflow detection)
    //
    //   Derivation:
    //     sig_lg represents value = sig_lg × 2^(e_lg − 19)
    //     sum_21 represents value = sum_21 × 2^(e_lg − 19)
    //     sum_21 ≈ 2^(20 − c) × 1.something   (leading 1 at bit 20−c)
    //     actual = 2^(20−c) × 2^(e_lg−19) = 2^(e_lg + 1 − c)
    //     result_exp = e_lg + 1 − c  [+ 1 if mant_carry from rounding]
    // =========================================================================
    wire signed [7:0] exp_raw8 = $signed({3'b0, e_lg})
                                 + 8'sd1
                                 - $signed({3'b0, c})
                                 + (mant_carry ? 8'sd1 : 8'sd0);

    wire exp_overflow  = (exp_raw8 > 8'sd31);
    wire exp_underflow = exp_raw8[7];   // sign bit → negative exponent

    wire [4:0]  exp_out  = exp_raw8[4:0];
    wire [14:0] mant_out = mant_carry ? 15'b0 : mant_inc[14:0];

    // =========================================================================
    // Output multiplexer  (priority: zero → underflow → overflow → normal)
    // =========================================================================
    assign y = zero_a                       ? b                             :
               zero_b                       ? a                             :
               (sum_zero | exp_underflow)   ? 21'b0                         :
               exp_overflow                 ? {s_res, 5'b11111, 15'b0}      :
                                             {s_res, exp_out, mant_out};

endmodule
// =============================================================================
// End of fp21_adder.v
// =============================================================================
