// =============================================================================
// File    : int_to_fp_acc.v
// Module  : int_to_fp_acc
// Project : Intensive-CIM Core  (INT-to-FP accumulation unit, Fig. 8)
//
// Overview
// --------
//   This module implements the entire INT-to-FP accumulation path shown in
//   Fig. 8 of the paper.  It is the final stage of the Intensive-CIM Core:
//
//     cim_macro → [int_to_fp_acc] → local SRAM → write-back → FP16 output
//
//   Three sub-operations are performed:
//
//   (A) Accumulation  (triggered by mac_valid)
//     1. Each integer psum[j] from cim_macro is converted to FP21
//        (int_to_fp21 instance, combinational).
//     2. The FP21 result is added to the stored FP21 accumulator acc[j]
//        (fp21_adder instance, combinational).
//     3. The updated FP21 value is registered back into acc[j].
//
//   (B) Write-back  (triggered by writeback)
//     1. Each acc[j] is converted to FP16 using e_bias correction
//        (fp21_to_fp16 instance, combinational).
//     2. The FP16 values are latched into fp16_result_flat.
//     3. fp16_valid fires for one clock cycle.
//
//   (C) Clear  (triggered by acc_clear)
//     All FP21 accumulators are zeroed, ready for a new accumulation epoch.
//
// Architecture
// ------------
//   All NUM_COL channels are processed in parallel (one instance of each
//   sub-module per channel, using a generate loop).  The pipeline is
//   single-cycle: a single mac_valid pulse reads the current acc[], adds the
//   new FP21, and writes back in one posedge.
//
//                  mac_result_flat
//                       │
//              ┌─────────────────────┐   (generate × NUM_COL)
//              │  int_to_fp21[j]     │─────────────┐
//              │  (combinational)    │             │
//              └─────────────────────┘             │ fp21_new[j]
//                                                  ▼
//              ┌─────────────────────┐
//   acc[j] ───►│  fp21_adder[j]      │──► fp21_sum[j] ──► acc[j]  (on mac_valid)
//              │  (combinational)    │
//              └─────────────────────┘
//                       │ acc[j]
//                       ▼
//              ┌─────────────────────┐
//              │  fp21_to_fp16[j]    │──► fp16_out[j] ──► fp16_result_flat  (on writeback)
//              │  (combinational)    │
//              └─────────────────────┘
//
// Signal interface
// ----------------
//   mac_result_flat   : packed NUM_COL × PSUM_W signed integers from cim_macro
//   mac_valid         : one-cycle pulse (connects to cim_macro.mac_valid)
//   e_bias            : signed exponent correction, set per NN layer by host
//   acc_clear         : synchronous clear of all FP21 accumulators
//                       (assert before the first mac_valid of a new tile/layer)
//   writeback         : one-cycle pulse; triggers FP16 output
//                       (assert after the last mac_valid of a tile/layer)
//   fp16_result_flat  : packed NUM_COL × 16-bit FP16 output
//   fp16_valid        : one-cycle pulse, one posedge after writeback
//
// Priority when signals arrive simultaneously:
//   acc_clear > mac_valid  (clear takes precedence over accumulate)
//   writeback  reads acc[] BEFORE any acc_clear / mac_valid update
//   (Verilog NBA semantics: latch reads register BEFORE the write)
//
// Connection to aligner.v + cim_macro.v
// ----------------------------------------
//   .mac_result_flat  ← cim_macro.mac_result_flat
//   .mac_valid        ← cim_macro.mac_valid
//   .e_bias           ← signed(Emin_act - 15)  for FP16 activations
//                        (approximately 5 - Emax_field for 1-bit weights)
//
// Parameters
// ----------
//   NUM_COL   : number of CIM macro output columns.  Default = 64.
//   PSUM_W    : integer psum width.  Default = 32.  Must match cim_macro.
//   EBIAS_W   : signed bit-width of e_bias port.  Default = 6 (range −32..+31).
//
// Local SRAM note
//   The accumulator array acc[] is implemented as flip-flops here.
//   For full-chip integration it can be replaced by an SRAM macro
//   (NUM_COL × 21-bit depth).  The read/write control remains the same.
//
// Synthesis note
//   The generate loop creates NUM_COL instances of each sub-module.
//   For NUM_COL = 64 this synthesises to 64 × (int_to_fp21 + fp21_adder +
//   fp21_to_fp16) combinational cones in parallel, which is the intended
//   microarchitecture matching the paper.
// =============================================================================
`timescale 1ns/1ps

module int_to_fp_acc #(
    parameter NUM_COL  = 64,    // output channels  (must match cim_macro)
    parameter PSUM_W   = 32,    // integer psum width (must match cim_macro)
    parameter EXP_W    = 5,     // FP exponent field width  (FP16 = 5)
    parameter MAN_W    = 10,    // FP mantissa field width  (FP16 = 10)
    // EBIAS_W: internal signed width for e_bias computation.
    // e_bias = emin - MAN_W - 15.  With MAN_W=10, EXP_W=5:
    //   range = [0-10-15, 31-10-15] = [-25, +6]  → 6 bits sufficient.
    parameter EBIAS_W  = 6
)(
    input  clk,
    input  rst_n,

    // ---- From cim_macro ----
    // mac_result_flat[j*PSUM_W +: PSUM_W] = signed integer psum for column j
    input  [NUM_COL*PSUM_W-1:0]  mac_result_flat,
    input                         mac_valid,        // one-cycle pulse

    // ---- Control (from host / CIM controller) ----
    // emin: lower exponent bound of the intensive region, same as aligner.emin.
    // e_bias is computed internally as: e_bias = emin - MAN_W - 15
    input  [EXP_W-1:0]           emin,       // from aligner / host controller
    input                         acc_clear,  // synchronous clear of all acc[]
    input                         writeback,  // one-cycle pulse: output FP16

    // ---- FP16 output ----
    // fp16_result_flat[j*16 +: 16] = FP16 result for channel j
    output reg [NUM_COL*16-1:0]  fp16_result_flat,
    output reg                    fp16_valid         // one-cycle pulse
);

    // =========================================================================
    // =========================================================================
    // e_bias: exponent correction for FP21 → FP16 write-back.
    //
    //   Derivation:
    //     aligner aligns each FP activation so that the bit-serial integer
    //     represents  A_int = A × 2^(MAN_W − (emin−15))
    //     where (emin−15) is the unbiased minimum exponent (FP16 bias = 15).
    //
    //     The true FP value = psum × 2^((emin−15) − MAN_W)
    //
    //     fp21_to_fp16 uses:  FP16_exp = FP21_exp_abs + e_bias + 15
    //     We need:            FP16_exp = FP21_exp_abs + (emin−15) − MAN_W + 15
    //     Therefore:          e_bias   = emin − MAN_W − 15
    // =========================================================================
    // e_bias = emin - MAN_W - 15  (all terms sign-extended to EBIAS_W bits)
    // MAN_W and 15 are parameters/constants; cast via localparam to avoid
    // Verilog-2001 issues with direct parameter bit-selects.
    localparam signed [EBIAS_W-1:0] LP_MANW = MAN_W;
    localparam signed [EBIAS_W-1:0] LP_15   = 15;
    wire signed [EBIAS_W-1:0] e_bias =
        $signed({{(EBIAS_W-EXP_W){1'b0}}, emin}) - LP_MANW - LP_15;

    // =========================================================================
    // FP21 accumulator register file
    //   Depth : NUM_COL
    //   Width : 21 bits per entry  {sign[1], exp[4:0], mant[14:0]}
    //   Reset : all entries = 21'b0 (zero)
    // =========================================================================
    reg [20:0] acc [0:NUM_COL-1];

    // =========================================================================
    // Combinational conversion arrays (one entry per column)
    //   fp21_new[j] : INT32 converted to FP21  (input to adder)
    //   fp21_sum[j] : FP21_new + acc[j]        (write-back to acc)
    //   fp16_out[j] : acc[j] converted to FP16 (registered on writeback)
    // =========================================================================
    wire [20:0] fp21_new [0:NUM_COL-1];
    wire [20:0] fp21_sum [0:NUM_COL-1];
    wire [15:0] fp16_out [0:NUM_COL-1];

    // =========================================================================
    // Generate block: instantiate sub-modules for all NUM_COL channels
    // =========================================================================
    genvar j;
    generate
        for (j = 0; j < NUM_COL; j = j + 1) begin : GEN_CH

            // -----------------------------------------------------------------
            // Stage 1 : integer psum → FP21
            // -----------------------------------------------------------------
            int_to_fp21 u_int_to_fp21 (
                .psum (mac_result_flat[j*PSUM_W +: PSUM_W]),
                .fp21 (fp21_new[j])
            );

            // -----------------------------------------------------------------
            // Stage 2 : FP21 accumulation  (add new FP21 to stored acc[j])
            // -----------------------------------------------------------------
            fp21_adder u_fp21_adder (
                .a (fp21_new[j]),
                .b (acc[j]),
                .y (fp21_sum[j])
            );

            // -----------------------------------------------------------------
            // Stage 3 : FP21 → FP16 for write-back
            //   Reads acc[j] COMBINATIONALLY so the conversion is always
            //   current.  The result fp16_out[j] is registered only when
            //   writeback is asserted.
            // -----------------------------------------------------------------
            fp21_to_fp16 #(.EBIAS_W(EBIAS_W)) u_fp21_to_fp16 (
                .fp21   (acc[j]),
                .e_bias (e_bias),
                .fp16   (fp16_out[j])
            );

        end
    endgenerate

    // =========================================================================
    // Sequential: accumulator update + output register
    // =========================================================================
    integer ci;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // ---- Asynchronous reset: zero all accumulators and outputs ----
            for (ci = 0; ci < NUM_COL; ci = ci + 1)
                acc[ci] <= 21'b0;
            fp16_valid <= 1'b0;

        end else begin
            // Default: fp16_valid deasserts every cycle unless writeback fires.
            fp16_valid <= 1'b0;

            // ---- (C) Synchronous clear (higher priority than mac_valid) ----
            if (acc_clear) begin
                for (ci = 0; ci < NUM_COL; ci = ci + 1)
                    acc[ci] <= 21'b0;

            // ---- (A) Accumulation: acc[j] ← FP21_new[j] + acc[j] --------
            end else if (mac_valid) begin
                for (ci = 0; ci < NUM_COL; ci = ci + 1)
                    acc[ci] <= fp21_sum[ci];
            end

            // ---- (B) Write-back: latch FP16 results and pulse fp16_valid --
            //   Verilog NBA semantics: fp16_out[ci] reads acc[ci] BEFORE
            //   the NBAs above take effect, so writeback sees the acc[] value
            //   from the PREVIOUS cycle (i.e., before any current-cycle update).
            //   Typical usage: writeback is asserted one cycle AFTER the last
            //   mac_valid, so this timing is correct.
            if (writeback) begin
                for (ci = 0; ci < NUM_COL; ci = ci + 1)
                    fp16_result_flat[ci*16 +: 16] <= fp16_out[ci];
                fp16_valid <= 1'b1;
            end
        end
    end

endmodule
// =============================================================================
// End of int_to_fp_acc.v
// =============================================================================
