// =============================================================================
//  aligner.v  —  Bit-Serial FP-to-INT Alignment Unit
//
//  Reference:
//    "A 28-nm Floating-Point CIM Processor Using Intensive-CIM
//     Sparse-Digital Architecture", IEEE JSSC Vol.59 No.8, Aug. 2024
//    Fig. 7 — Concise bit-serial FP-to-INT transfer
//
//  Overview:
//    Converts a vector of NUM_CH FP16 activation values into bit-serial
//    two's-complement integer representation, producing NUM_CH output
//    bits per clock cycle for direct feeding into the CIM macro.
//
//  FP16 field encoding  (IEEE-754 half-precision):
//    bit  [15]      = sign  (S)
//    bits [14:10]   = biased exponent  (E, 5 bits, bias = 15)
//    bits [9:0]     = mantissa fraction  (M, 10 bits; hidden "1" not stored)
//    value = (-1)^S * 2^(E-15) * (1.M)
//
//  Per-channel bit-serial output sequence:
//    Total cycles = (Emax - Emin) + MAN_W + 2
//
//    ┌─────────────────────┬──────────────────────────┬──────────────────┐
//    │  (Emax – E) cycles  │   (MAN_W + 2) cycles     │  (E–Emin) cycles │
//    │   sign extension    │  {S, hidden=1, M} in 2sC │   zero padding   │
//    └─────────────────────┴──────────────────────────┴──────────────────┘
//    MSB output first (big-endian bit order, sign at left / cycle 1)
//
//    The three phases together produce the aligned two's-complement
//    fixed-point integer for each FP value, all channels synchronized
//    to the same Emax reference.
//
//  Two's-complement conversion  (Fig. 7a — "Transfer to 2's complement"):
//    mantissa word = {hidden=1, M[9:0]}   (11 bits, unsigned magnitude)
//    positive (S=0):  man_tc = { 0, mantissa word }
//    negative (S=1):  man_tc = two's-complement negate
//                            = { 1, (~mantissa_word + 1) }
//
//  File contents:
//    1. aligner           — top module (cycle controller + NUM_CH channels)
//    2. aligner_channel   — single-channel state machine
//
//  Assumptions:
//    • Input FP values are in the intensive region: Emin ≤ fp_exp ≤ Emax.
//      Out-of-range (sparse/long-tail) data is handled by the separate
//      sparse-digital core and must NOT be passed to this module.
//    • `load` is a single-cycle pulse.  First output bit appears the cycle
//      AFTER `load` is asserted.
//    • Subnormal FP values (biased exponent == 0) are not supported.
// =============================================================================

`timescale 1ns / 1ps

// =============================================================================
//  aligner  (top-level)
//
//  Ports:
//    clk        — system clock (rising-edge triggered)
//    rst_n      — active-low synchronous reset
//    load       — 1-cycle pulse: latch new FP16 vector and begin conversion
//    emax       — upper exponent bound of intensive region  [EXP_W bits]
//    emin       — lower exponent bound of intensive region  [EXP_W bits]
//    fp_vec_in  — packed FP16 activation vector
//                   ch = fp_vec_in[ch*FP_W +: FP_W],  ch ∈ [0, NUM_CH)
//    bs_out     — bit-serial output: bs_out[ch] = channel ch's current bit
//    bs_valid   — high for every valid output cycle (NUM_CH bits all valid)
//    bs_done    — high on the very last valid output cycle of a vector
// =============================================================================
module aligner #(
    parameter NUM_CH = 128,   // number of FP activation channels (paper: 128)
    parameter EXP_W  = 5,     // exponent field width  (FP16 = 5)
    parameter MAN_W  = 10,    // mantissa field width  (FP16 = 10)
    parameter FP_W   = 16     // total FP word width   (FP16 = 16 = 1+5+10)
)(
    input  wire                    clk,
    input  wire                    rst_n,

    // Control
    input  wire                    load,
    input  wire [EXP_W-1:0]       emax,
    input  wire [EXP_W-1:0]       emin,

    // Packed FP16 activation vector
    input  wire [NUM_CH*FP_W-1:0] fp_vec_in,

    // Bit-serial output
    output wire [NUM_CH-1:0]       bs_out,
    output wire                    bs_valid,
    output wire                    bs_done,
    // Expose total_cyc so cim_macro can be driven without an extra counter.
    // Width = EXP_W+4 bits (= CNT_W), matching the internal wire.
    output wire [EXP_W+3:0]       total_cyc_out
);

    // -------------------------------------------------------------------------
    // Cycle counter
    //   total_cyc = (Emax - Emin) + MAN_W + 2  (number of valid output cycles)
    //   CNT_W must hold values up to 2^EXP_W + MAN_W + 2 - 1.
    //   With EXP_W=5, MAN_W=10: max = 31 + 10 + 2 = 43 → 6 bits sufficient.
    //   Use EXP_W+4 = 9 bits for safe margin.
    //   cyc_cnt counts from 0 to total_cyc-1 (total_cyc valid cycles).
    // -------------------------------------------------------------------------
    localparam CNT_W = EXP_W + 4;

    reg  [CNT_W-1:0] cyc_cnt;
    reg              running;

    // Combinational: total cycles for current emax/emin setting
    wire [CNT_W-1:0] total_cyc = {{(CNT_W-EXP_W){1'b0}}, (emax - emin)}
                                 + (MAN_W + 2);

    assign bs_valid = running;
    assign bs_done      = running & (cyc_cnt == total_cyc - 1);
    assign total_cyc_out = total_cyc;   // expose to downstream modules

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cyc_cnt <= {CNT_W{1'b0}};
            running <= 1'b0;
        end else if (load) begin
            // Latch emax/emin for this vector; counter starts next cycle
            cyc_cnt <= {CNT_W{1'b0}};
            running <= 1'b1;
        end else if (running) begin
            if (cyc_cnt == total_cyc - 1) begin
                running <= 1'b0;
                cyc_cnt <= {CNT_W{1'b0}};
            end else begin
                cyc_cnt <= cyc_cnt + 1'b1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Per-channel aligner instances
    // -------------------------------------------------------------------------
    genvar ch;
    generate
        for (ch = 0; ch < NUM_CH; ch = ch + 1) begin : gen_channels
            aligner_channel #(
                .EXP_W (EXP_W),
                .MAN_W (MAN_W),
                .FP_W  (FP_W)
            ) u_ch (
                .clk     (clk),
                .rst_n   (rst_n),
                .load    (load),
                .emax    (emax),
                .emin    (emin),
                .fp_in   (fp_vec_in[ch*FP_W +: FP_W]),
                .bit_out (bs_out[ch])
            );
        end
    endgenerate

endmodule

// =============================================================================
//  Cycle & bit-count reference table (FP16, MAN_W=10, EXP_W=5)
//
//  Notation:  E = fp_exp (biased),  Emax, Emin ∈ [0,31]
//  Example:   Emax=20, Emin=16  →  total_cyc = (20-16)+10+2 = 16 cycles
//
//  Channel with E=20 (= Emax):  0 align | 12 mantissa | 4 zero  = 16 cycles
//  Channel with E=19:            1 align | 12 mantissa | 3 zero  = 16 cycles
//  Channel with E=18:            2 align | 12 mantissa | 2 zero  = 16 cycles
//  Channel with E=17:            3 align | 12 mantissa | 1 zero  = 16 cycles
//  Channel with E=16 (= Emin):  4 align | 12 mantissa | 0 zero  = 16 cycles
//
//  All channels produce an output on every cycle → 128-bit wide bus to CIM macro.
//
//  Timing diagram (ch0 = E=Emax, ch1 = E=Emax-1, relative to cycle after load):
//
//  Cycle:  |  1  |  2  |  3  |  4  |  5  ...  | 12  | 13  | 14  | 15  | 16 |
//  ch0:    |  S  |  h  |M[9] |M[8] |  ...      |M[0] |  0  |  0  |  0  |  0 |
//  ch1:    |  S  |  S  |  h  |M[9] |  ...      |M[1] |M[0] |  0  |  0  |  0 |
//          |←align→|←──────── mantissa ────────→|←─ zero-pad ─→|
//
//  Where  S = sign-extension bit (= two's-complement MSB)
//         h = hidden bit (always 1 for normal FP)
//         M[k] = mantissa bit k
// =============================================================================

