// =============================================================================
// File    : CIM_TOP.v
// Module  : CIM_TOP
// Project : Intensive-CIM Core (JSSC 2024 Fig. 5 / Fig. 8 simplified)
//
// Description
// -----------
//   Top-level wrapper that connects the three pipeline stages of the
//   Intensive-CIM Core in the correct signal order:
//
//     Stage 1 : aligner       — FP16 vector → bit-serial INT stream
//     Stage 2 : cim_macro     — bit-serial MAC with 1-bit SRAM weights
//     Stage 3 : int_to_fp_acc — INT psum accumulation → FP21 → FP16 output
//
//   Data-path:
//
//     fp_vec_in ──► [aligner] ──bs_out/valid/done/total_cyc──►
//                  [cim_macro] ──mac_result_flat/mac_valid──►
//                  [int_to_fp_acc] ──► fp16_result_flat / fp16_valid
//
//   Control signals driven by this wrapper:
//     ┌────────────────────────────────────────────────────────────┐
//     │  Host sets:                                                │
//     │    emax, emin   — intensive exponent window               │
//     │    load         — one-cycle pulse: load new FP16 vector   │
//     │    acc_clear    — synchronous clear before new tile       │
//     │    writeback    — one-cycle pulse: commit FP16 result     │
//     │    wr_en/row/col/data — weight SRAM write interface       │
//     └────────────────────────────────────────────────────────────┘
//
// Simplifications vs. the paper (undergraduate thesis scope)
//   • No ping-pong weight update (wr_en must be LOW during MAC)
//   • No Low-MACV adder truncation
//   • No block-wise or random sparsity skipping
//   • 1-bit weights only
//   • Single tile; no pipeline overlap between weight load and MAC
//
// Parameter notes
//   NUM_CH   : FP activation channels = weight rows.
//              aligner.NUM_CH == cim_macro.NUM_ROW.  Default = 128.
//   NUM_COL  : Weight columns = output channels.
//              cim_macro.NUM_COL == int_to_fp_acc.NUM_COL.  Default = 64.
//   EXP_W    : FP exponent field width.  5 for FP16.
//   MAN_W    : FP mantissa field width.  10 for FP16.
//   FP_W     : Total FP word width.      16 for FP16 (= 1+EXP_W+MAN_W).
//   PSUM_W   : Signed psum accumulator width.  32 bits (safe for FP16).
//   CYC_W    : total_cyc bus width.  6 bits (max value = 31+10+2 = 43).
//
// Interface timing (one-vector operation)
//
//  clk          : __|¯|_|¯|_|¯|_|¯|_ ... _|¯|_|¯|_|¯|_
//
//  [Weight load phase — must complete before load pulse]
//  wr_en        : _____|¯¯¯¯¯¯¯¯¯¯¯¯¯¯|______________
//  wr_row/col/  : ------<valid data>----______________
//  wr_data
//
//  [Accumulator clear — one cycle before load]
//  acc_clear    : ________________________|¯|__________
//
//  [Activation load and MAC]
//  load         : __________________________|¯|________
//  fp_vec_in    : --------------------------<stable>---
//  bs_valid     : ____________________________|¯¯¯¯¯¯¯|
//  bs_done      : ___________________________________|¯|
//  mac_valid    : _________________________________________|¯|_ (2 cycles after bs_done)
//
//  [Write-back — assert one cycle after mac_valid]
//  writeback    : ________________________________________|¯|__
//  fp16_valid   : __________________________________________|¯|__
//
// =============================================================================
`timescale 1ns/1ps

module CIM_TOP #(
    parameter NUM_CH  = 128,  // FP activation channels  (= weight rows)
    parameter NUM_COL = 64,   // Output channels          (= weight columns)
    parameter EXP_W   = 5,    // FP exponent field width  (FP16 = 5)
    parameter MAN_W   = 10,   // FP mantissa field width  (FP16 = 10)
    parameter FP_W    = 16,   // Total FP word width      (FP16 = 16)
    parameter PSUM_W  = 32,   // Signed psum accumulator width
    parameter CYC_W   = 6     // total_cyc bus width (6 bits → max 43 for FP16)
)(
    input  wire clk,
    input  wire rst_n,

    // ---- Exponent window (set by host before each tile/layer) ----
    input  wire [EXP_W-1:0]         emax,       // biased maximum exponent of intensive region
    input  wire [EXP_W-1:0]         emin,       // biased minimum exponent of intensive region

    // ---- FP16 activation input ----
    input  wire                      load,       // one-cycle pulse: load fp_vec_in into aligner
    input  wire [NUM_CH*FP_W-1:0]   fp_vec_in,  // packed NUM_CH FP16 activations

    // ---- Weight SRAM write interface ----
    //   Must be driven ONLY when bs_valid = 0 (between MAC operations).
    input  wire                      wr_en,
    input  wire [6:0]                wr_row,     // row address  [0, NUM_CH-1]
    input  wire [5:0]                wr_col,     // column address [0, NUM_COL-1]
    input  wire                      wr_data,    // 1-bit weight

    // ---- Accumulation control ----
    input  wire                      acc_clear,  // sync clear all FP21 accumulators
    input  wire                      writeback,  // one-cycle: commit acc[] → FP16

    // ---- FP16 output ----
    output wire [NUM_COL*16-1:0]     fp16_result_flat,
    output wire                      fp16_valid         // one-cycle pulse
);

    // =========================================================================
    // Compile-time parameter validation
    // =========================================================================
    generate
        if (CYC_W < 6) begin : CHECK_CYC_W
            // CYC_W must be >= 6 to hold max total_cyc = 43 for FP16
            INVALID_CYC_W_PARAMETER invalid();
        end
    endgenerate

    // =========================================================================
    // Internal wires: Stage 1 → Stage 2
    // =========================================================================
    wire [NUM_CH-1:0]  bs_out;           // bit-serial activations
    wire               bs_valid;         // HIGH for all MAC cycles
    wire               bs_done;          // HIGH on last MAC cycle
    wire [EXP_W+3:0]   total_cyc_wide;   // CNT_W = EXP_W+4 bits from aligner

    // total_cyc bus to cim_macro is CYC_W bits wide.
    // CYC_W = 6 is wide enough for all FP16 intensive configurations (max = 43).
    // The upper bits of total_cyc_wide are zero for practical Emax−Emin ranges,
    // so the truncation is safe when CYC_W >= actual total_cyc bit-length needed.
    wire [CYC_W-1:0]   total_cyc = total_cyc_wide[CYC_W-1:0];

    // =========================================================================
    // Internal wires: Stage 2 → Stage 3
    // =========================================================================
    wire [NUM_COL*PSUM_W-1:0] mac_result_flat;
    wire                       mac_valid;

    // =========================================================================
    // Stage 1 : aligner
    //   Converts the FP16 activation vector into bit-serial INT representation.
    //   Outputs bs_out[NUM_CH-1:0] over total_cyc clock cycles.
    // =========================================================================
    aligner #(
        .NUM_CH (NUM_CH),
        .EXP_W  (EXP_W),
        .MAN_W  (MAN_W),
        .FP_W   (FP_W)
    ) u_aligner (
        .clk          (clk),
        .rst_n        (rst_n),
        .load         (load),
        .emax         (emax),
        .emin         (emin),
        .fp_vec_in    (fp_vec_in),
        .bs_out       (bs_out),
        .bs_valid     (bs_valid),
        .bs_done      (bs_done),
        .total_cyc_out(total_cyc_wide)
    );

    // =========================================================================
    // Stage 2 : cim_macro
    //   Performs bit-serial MAC between bs_out and 1-bit SRAM weights.
    //   Outputs NUM_COL signed 32-bit psum values when mac_valid fires.
    // =========================================================================
    cim_macro #(
        .NUM_ROW (NUM_CH),
        .NUM_COL (NUM_COL),
        .PSUM_W  (PSUM_W),
        .CYC_W   (CYC_W)
    ) u_cim_macro (
        .clk             (clk),
        .rst_n           (rst_n),
        // Weight SRAM write interface (pass-through from top)
        .wr_en           (wr_en),
        .wr_row          (wr_row),
        .wr_col          (wr_col),
        .wr_data         (wr_data),
        // Bit-serial activation from Stage 1
        .act_bs          (bs_out),
        .bs_valid        (bs_valid),
        .bs_done         (bs_done),
        .total_cyc       (total_cyc),
        // MAC outputs to Stage 3
        .mac_result_flat (mac_result_flat),
        .mac_valid       (mac_valid)
    );

    // =========================================================================
    // Stage 3 : int_to_fp_acc
    //   Accumulates psum into FP21 registers, then converts to FP16 on
    //   write-back.  e_bias is computed internally from emin and MAN_W.
    // =========================================================================
    int_to_fp_acc #(
        .NUM_COL (NUM_COL),
        .PSUM_W  (PSUM_W),
        .EXP_W   (EXP_W),
        .MAN_W   (MAN_W),
        .EBIAS_W (6)
    ) u_int_to_fp_acc (
        .clk             (clk),
        .rst_n           (rst_n),
        // From Stage 2
        .mac_result_flat (mac_result_flat),
        .mac_valid       (mac_valid),
        // Exponent window (for e_bias calculation)
        .emin            (emin),
        // Accumulation control
        .acc_clear       (acc_clear),
        .writeback       (writeback),
        // FP16 output
        .fp16_result_flat(fp16_result_flat),
        .fp16_valid      (fp16_valid)
    );

endmodule
// =============================================================================
// End of CIM_TOP.v
// =============================================================================
