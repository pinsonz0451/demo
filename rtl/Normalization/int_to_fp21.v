// =============================================================================
// File    : int_to_fp21.v
// Module  : int_to_fp21
// Project : Intensive-CIM Core  (INT-to-FP accumulation, Fig. 8)
//
// Function
// --------
//   Converts a 32-bit signed integer (psum from cim_macro) to the custom
//   21-bit FP format used internally for partial-sum accumulation.
//
// FP21 Format  {sign[1], exp[4:0], mant[14:0]}  = 21 bits total
//
//   Bit 20    : sign (0 = positive, 1 = negative)
//   Bits 19:15: exponent E  — 5-bit ABSOLUTE (no bias); range 0..30.
//               E = floor(log2(|psum|)) = bit-position of the leading 1.
//   Bits 14:0 : mantissa M  — 15-bit fractional part (hidden 1 NOT stored).
//
//   Value     : (-1)^sign × 2^E × (1 + M / 2^15)
//   Zero      : all bits 0  (21'b0 is the unique zero representation)
//
// Why FP21?  (from the paper, Section IV-B)
//   psum values are always integers, so E can be stored as the absolute
//   bit-position without bias, simplifying normalization.  Five extra
//   mantissa bits (15 vs FP16's 10) are reserved to prevent intermediate
//   precision loss during repeated FP21 accumulations.
//
// Rounding   : round-to-nearest, ties-to-even (IEEE 754 default mode)
//
// Latency    : combinational (0 clock cycles)
//
// Usage
// -----
//   int_to_fp21 u0 (
//       .psum  (mac_result_flat[j*32 +: 32]),   // from cim_macro
//       .fp21  (fp21_new[j])
//   );
// =============================================================================
`timescale 1ns/1ps

module int_to_fp21 (
    input  signed [31:0] psum,   // 32-bit signed integer  (from cim_macro psum[j])
    output        [20:0] fp21    // custom 21-bit FP result
);

    // =========================================================================
    // Step 1 : Sign extraction and magnitude (absolute value)
    // =========================================================================
    wire        sign_bit = psum[31];

    //   Two's-complement negation.
    //   Edge case: psum = -2^31 → ~psum+1 = 32'h80000000 (same bit pattern).
    //   Handled correctly: exp=31, mant=0, FP21 = -1.0 × 2^31 = -2^31. ✓
    wire [31:0] mag = sign_bit ? (~psum + 32'd1) : psum;

    // =========================================================================
    // Step 2 : Count Leading Zeros (CLZ) — priority encoder (synthesis-safe)
    //   Returns 0..31 for non-zero mag; 32 for mag == 0.
    //
    //   casex is used so that only the position of the leading 1 matters;
    //   all lower bits are masked as "don't care" (x).
    // =========================================================================
    function [5:0] clz32;
        input [31:0] x;
        casex (x)
            32'b1xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx: clz32 = 6'd0;
            32'b01xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx: clz32 = 6'd1;
            32'b001xxxxxxxxxxxxxxxxxxxxxxxxxxxxx: clz32 = 6'd2;
            32'b0001xxxxxxxxxxxxxxxxxxxxxxxxxxxx: clz32 = 6'd3;
            32'b00001xxxxxxxxxxxxxxxxxxxxxxxxxxx: clz32 = 6'd4;
            32'b000001xxxxxxxxxxxxxxxxxxxxxxxxxx: clz32 = 6'd5;
            32'b0000001xxxxxxxxxxxxxxxxxxxxxxxxx: clz32 = 6'd6;
            32'b00000001xxxxxxxxxxxxxxxxxxxxxxxx: clz32 = 6'd7;
            32'b000000001xxxxxxxxxxxxxxxxxxxxxxx: clz32 = 6'd8;
            32'b0000000001xxxxxxxxxxxxxxxxxxxxxx: clz32 = 6'd9;
            32'b00000000001xxxxxxxxxxxxxxxxxxxxx: clz32 = 6'd10;
            32'b000000000001xxxxxxxxxxxxxxxxxxxx: clz32 = 6'd11;
            32'b0000000000001xxxxxxxxxxxxxxxxxxx: clz32 = 6'd12;
            32'b00000000000001xxxxxxxxxxxxxxxxxx: clz32 = 6'd13;
            32'b000000000000001xxxxxxxxxxxxxxxxx: clz32 = 6'd14;
            32'b0000000000000001xxxxxxxxxxxxxxxx: clz32 = 6'd15;
            32'b00000000000000001xxxxxxxxxxxxxxx: clz32 = 6'd16;
            32'b000000000000000001xxxxxxxxxxxxxx: clz32 = 6'd17;
            32'b0000000000000000001xxxxxxxxxxxxx: clz32 = 6'd18;
            32'b00000000000000000001xxxxxxxxxxxx: clz32 = 6'd19;
            32'b000000000000000000001xxxxxxxxxxx: clz32 = 6'd20;
            32'b0000000000000000000001xxxxxxxxxx: clz32 = 6'd21;
            32'b00000000000000000000001xxxxxxxxx: clz32 = 6'd22;
            32'b000000000000000000000001xxxxxxxx: clz32 = 6'd23;
            32'b0000000000000000000000001xxxxxxx: clz32 = 6'd24;
            32'b00000000000000000000000001xxxxxx: clz32 = 6'd25;
            32'b000000000000000000000000001xxxxx: clz32 = 6'd26;
            32'b0000000000000000000000000001xxxx: clz32 = 6'd27;
            32'b00000000000000000000000000001xxx: clz32 = 6'd28;
            32'b000000000000000000000000000001xx: clz32 = 6'd29;
            32'b0000000000000000000000000000001x: clz32 = 6'd30;
            32'b00000000000000000000000000000001: clz32 = 6'd31;
            default:                              clz32 = 6'd32;  // mag == 0
        endcase
    endfunction

    wire [5:0] clz     = clz32(mag);
    wire       is_zero = (clz == 6'd32);

    // =========================================================================
    // Step 3 : Exponent = bit-position of the leading 1 (0-indexed from bit 0)
    //   e.g.  mag = 100 = 0b1100100  → leading 1 at bit 6 → exp = 6
    //         mag = 1                 → leading 1 at bit 0 → exp = 0
    // =========================================================================
    wire [4:0] exp_raw = 5'd31 - clz[4:0];
    // (When is_zero, clz[4:0] = 0, exp_raw = 31 — masked by is_zero at output)

    // =========================================================================
    // Step 4 : Normalize — shift mag left by clz so the leading 1 lands at
    //   bit 31 (the hidden-bit position in a 32-bit aligned view).
    //
    //   shifted[31]    = 1        hidden bit (NOT stored in FP21)
    //   shifted[30:16] = 15 bits  mantissa
    //   shifted[15]    = R        round bit
    //   shifted[14:0]  = 15 bits  sticky bits
    // =========================================================================
    wire [31:0] shifted   = mag << clz;
    wire [14:0] mant_raw  = shifted[30:16];
    wire        round_bit = shifted[15];
    wire        sticky    = |shifted[14:0];

    // =========================================================================
    // Step 5 : Round-to-nearest, ties-to-even
    //   Round up iff:  R = 1  AND  (sticky ≠ 0  OR  LSB of mantissa = 1)
    // =========================================================================
    wire round_up = round_bit & (sticky | mant_raw[0]);

    wire [15:0] mant_inc   = {1'b0, mant_raw} + {15'b0, round_up};
    wire        mant_carry = mant_inc[15];

    // If rounding overflows the 15-bit mantissa, increment exponent and
    // set mantissa to zero (1.11...1 + ulp → 10.00...0 → normalized 1.00..0).
    wire [4:0]  exp_final  = mant_carry ? (exp_raw  + 5'd1) : exp_raw;
    wire [14:0] mant_final = mant_carry ? 15'b0             : mant_inc[14:0];

    // =========================================================================
    // Step 6 : Pack output
    //   Zero input  →  21'b0 (all-zero is the unique FP21 zero representation)
    //   Normal      →  {sign_bit, exp_final[4:0], mant_final[14:0]}
    // =========================================================================
    assign fp21 = is_zero ? 21'b0 : {sign_bit, exp_final, mant_final};

endmodule
// =============================================================================
// End of int_to_fp21.v
// =============================================================================
