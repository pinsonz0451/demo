// =============================================================================
// File    : fp21_to_fp16.v
// Module  : fp21_to_fp16
// Project : Intensive-CIM Core  (INT-to-FP write-back, Fig. 8)
//
// Function
// --------
//   Converts a 21-bit FP accumulator value to IEEE-754 FP16, applying the
//   exponent bias correction required by the FP-to-INT alignment in aligner.v.
//   This implements the "write-back step" (dashed box) in Fig. 8 of the paper.
//
// FP21 → FP16 mapping
// -------------------
//   FP21: {sign[1], exp_abs[4:0], mant21[14:0]}
//         exp_abs is the ABSOLUTE exponent (no bias).
//
//   FP16: {sign[1], exp_biased[4:0], mant16[9:0]}
//         exp_biased uses bias = 15  (IEEE 754 FP16 convention).
//
//   FP16_exp_stored = FP21_exp_abs + e_bias + 15
//
//   The e_bias term corrects for the alignment offset introduced by
//   aligner.v.  For FP16 activations aligned to maximum exponent Emax
//   (biased field value), with Mb = 10 mantissa bits:
//
//       e_bias = Emin_act - 15   (the unbiased minimum activation exponent)
//
//   A common approximation for 1-bit weights + FP16 activations:
//       e_bias ≈ 5 - Emax_field   (see Section IV-B note in the paper)
//
//   e_bias is exposed as a signed programmable port so the host controller
//   can set the correct value per NN layer.
//
// Mantissa truncation
//   mant21[14:0]  →  mant16[9:0]   =  mant21[14:5]  (round-to-nearest, TTE)
//   mant21[4]   = round bit
//   mant21[3:0] = sticky bits
//
// Special cases
//   · FP21 == 0             → FP16 = 0
//   · FP16 exp_stored > 30  → FP16 = ±Inf  {sign, 5'b11111, 10'b0}
//   · FP16 exp_stored ≤ 0   → FP16 = 0 (flush denormals for simplicity)
//
// Parameters
//   EBIAS_W : bit-width of e_bias port (signed).  Default = 6 (range −32..+31).
//             Increase if e_bias values outside this range are needed.
//
// Latency    : combinational
// =============================================================================
`timescale 1ns/1ps

module fp21_to_fp16 #(
    parameter EBIAS_W = 6    // signed width of e_bias  (range: −2^(N-1) .. 2^(N-1)-1)
)(
    input  [20:0]                fp21,    // FP21 accumulator value
    input  signed [EBIAS_W-1:0] e_bias,  // exponent correction (see notes above)
    output [15:0]                fp16     // IEEE-754 FP16 result
);

    // =========================================================================
    // Field extraction
    // =========================================================================
    wire        s_21    = fp21[20];
    wire [4:0]  e_21    = fp21[19:15];   // absolute exponent (no bias)
    wire [14:0] m_21    = fp21[14:0];
    wire        is_zero = (fp21 == 21'b0);

    // =========================================================================
    // FP16 biased exponent  =  e_21 (absolute) + e_bias + 15 (FP16 bias)
    //
    //   Use 8-bit signed arithmetic so that over/underflow is detectable.
    //   e_bias is sign-extended from EBIAS_W to 8 bits before addition.
    // =========================================================================
    wire signed [7:0] e_bias_8 = {{(8-EBIAS_W){e_bias[EBIAS_W-1]}}, e_bias};

    wire signed [7:0] exp_biased = $signed({3'b0, e_21}) + e_bias_8 + 8'sd15;

    // =========================================================================
    // Mantissa truncation with round-to-nearest, ties-to-even
    //   mant21[14:5]  → 10 MSBs of FP16 mantissa
    //   mant21[4]     → round bit (R)
    //   mant21[3:0]   → sticky bits
    // =========================================================================
    wire [9:0]  mant10_raw = m_21[14:5];
    wire        rnd_bit    = m_21[4];
    wire        sticky     = |m_21[3:0];
    wire        round_up   = rnd_bit & (sticky | mant10_raw[0]);

    wire [10:0] mant_inc   = {1'b0, mant10_raw} + {10'b0, round_up};
    wire        mant_carry = mant_inc[10];

    // If mantissa rounds up to 1.00..0, increment the biased exponent.
    // Use a 9-bit signed intermediate to catch subsequent over/underflow.
    wire signed [8:0] exp_adj = $signed({1'b0, exp_biased}) +
                                (mant_carry ? 9'sd1 : 9'sd0);

    // =========================================================================
    // Overflow and underflow detection (after rounding adjustment)
    //   exp_adj >= 31 → overflow  → return ±Inf
    //   exp_adj <=  0 → underflow → return ±0  (flush-to-zero; no denormals)
    // =========================================================================
    wire out_overflow  = (exp_adj >= 9'sd31);
    wire out_underflow = (exp_adj <= 9'sd0) | is_zero;

    // =========================================================================
    // Output fields
    // =========================================================================
    wire [4:0]  fp16_exp  = out_overflow  ? 5'b11111 : exp_adj[4:0];
    wire [9:0]  fp16_mant = mant_carry    ? 10'b0    : mant_inc[9:0];

    assign fp16 = out_underflow ? {s_21, 15'b0}            :   // ±zero
                  out_overflow  ? {s_21, 5'b11111, 10'b0}  :   // ±inf
                                  {s_21, fp16_exp, fp16_mant};

endmodule
// =============================================================================
// End of fp21_to_fp16.v
// =============================================================================
