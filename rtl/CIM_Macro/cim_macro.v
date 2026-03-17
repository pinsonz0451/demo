// =============================================================================
// File    : cim_macro.v
// Module  : cim_macro
// Desc    : Top-level CIM Macro. Instantiates cim_array and adds the
//           shift-accumulate control loop that converts bit-serial
//           partial sums into final integer MAC results.
//
// -----------------------------------------------------------------------
// Three-module hierarchy:
//
//   adder_tree_128.v   ← Layer 1: 128×1b → 8b combinational sum tree
//          ↑
//   cim_array.v        ← Layer 2: SRAM + AND + adder trees
//          ↑
//   cim_macro.v        ← Layer 3: shift-accumulate + output  [THIS FILE]
//
// -----------------------------------------------------------------------
// Dataflow overview:
//
//  act_bs[127:0]  ──────────────────────────────────────────────────────┐
//  (from aligner)                                                        │
//                  ┌──────────────────────────────────────────────────┐ │
//                  │              cim_array                           │◄┘
//                  │  weight_mem + AND array + 64×adder_tree_128      │
//                  └────────────────────┬─────────────────────────────┘
//                                       │ partial_sum_flat (64×8b, comb.)
//                  ┌────────────────────▼─────────────────────────────┐
//                  │         Shift-Accumulate  (this module)          │
//                  │                                                   │
//                  │  cycle 1 (first_valid):                          │
//                  │    psum[j] ← partial_sum[j] << (total_cyc-1)    │
//                  │                                                   │
//                  │  cycles 2..N (mac_active):                       │
//                  │    psum[j] += partial_sum[j] << shift_cnt        │
//                  │    shift_cnt decrements each cycle               │
//                  └────────────────────┬─────────────────────────────┘
//                                       │
//                   mac_result_flat  (64×PSUM_W, combinational from psum)
//                   mac_valid        (one-cycle pulse, one cycle after bs_done)
//
// -----------------------------------------------------------------------
// Timing diagram (example: total_cyc = 4):
//
//  clk        : __|¯|_|¯|_|¯|_|¯|_|¯|_|¯|_|¯|_
//  bs_valid   : ________|¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯|______
//  bs_done    : ________________________|¯|______
//  first_valid: ________|¯|____________________
//  cycle#     :          1    2    3    4
//  shift used :         N-1  N-2  N-3   0       (= 3,2,1,0 for N=4)
//  shift_cnt  :          -    2    1    0        (pre-loaded at cycle 1)
//  mac_active :          |¯¯¯¯¯¯¯¯¯¯¯¯¯¯|
//  psum update:         wr1  acc  acc  acc
//  mac_valid  : ______________________________|¯|_  (one cycle after bs_done)
//
// -----------------------------------------------------------------------
// Shift-weight explanation:
//   aligner outputs MSB first (cycle 1 = highest bit position).
//   Therefore cycle 1's partial sum carries positional weight 2^(N-1),
//   cycle 2 carries 2^(N-2), ..., cycle N carries 2^0.
//   The accumulated result is:
//     psum[j] = Σ_{c=1}^{N} partial_sum_c[j] × 2^(N-c)
//
// -----------------------------------------------------------------------
// Output bit-width analysis (PSUM_W = 32):
//   max partial_sum per cycle = 128  (all 128 weights = 1, all acts = 1)
//   max psum = 128 × (2^0 + 2^1 + ... + 2^(N-1))
//            = 128 × (2^N - 1)
//   For N = 24 (FP16 intensive worst case):
//            = 128 × (2^24 - 1) ≈ 2^31  → needs 32 bits. PSUM_W=32 is safe.
//
// -----------------------------------------------------------------------
// Simplifications vs. paper (appropriate for thesis scope):
//   1. No ping-pong weight update  -- weights fixed during inference
//   2. No Low-MACV adder truncation -- full-precision adder tree
//   3. No block-wise sparsity      -- all cells always active
//   4. 1-bit weight only           -- no INT8/INT16 weight packing
//   5. Weight write is one-cell-per-cycle via wr_row/wr_col
// =============================================================================

`timescale 1ns/1ps

module cim_macro #(
    parameter NUM_ROW = 128,  // Input activation channels (= weight rows)
    parameter NUM_COL = 64,   // Output channels           (= weight columns)
    parameter PSUM_W  = 32,   // Accumulator width per column [bits]
                               // Must satisfy: PSUM_W >= 8 + max_total_cyc - 1
                               // FP16 intensive: max total_cyc ≈ 24 → 32b OK
    parameter CYC_W   = 6     // Bit-width of total_cyc port.
                               // FP16 max total_cyc = (31)+10+2 = 43 → 6 bits.
                               // For intensive region only (Emax-Emin≤4): 5 bits
                               // is sufficient; 6 is the safe general default.
)(
    input  wire                        clk,
    input  wire                        rst_n,   // active-low synchronous reset

    // -------------------------------------------------------------------------
    // Port Group A : Weight Loading
    //   Pass-through to cim_array's weight SRAM.
    //   wr_en MUST be LOW while bs_valid is HIGH.
    // -------------------------------------------------------------------------
    input  wire                        wr_en,    // write enable
    input  wire [6:0]                  wr_row,   // row address [0, NUM_ROW-1]
    input  wire [5:0]                  wr_col,   // col address [0, NUM_COL-1]
    input  wire                        wr_data,  // 1-bit weight to write

    // -------------------------------------------------------------------------
    // Port Group B : Bit-Serial Activation  (driven by aligner.v)
    //
    //   act_bs    : 128 channels each present 1 bit per cycle, MSB first.
    //   bs_valid  : HIGH for the entire MAC window (total_cyc cycles).
    //   bs_done   : HIGH on the LAST cycle only (cycle N).
    //   total_cyc : = (Emax - Emin) + Mb + 2  (sent once per vector).
    //
    //   These signals connect directly to aligner.v output ports.
    // -------------------------------------------------------------------------
    input  wire [NUM_ROW-1:0]          act_bs,
    input  wire                        bs_valid,
    input  wire                        bs_done,
    input  wire [CYC_W-1:0]           total_cyc,

    // -------------------------------------------------------------------------
    // Port Group C : MAC Result Output
    //
    //   mac_result_flat : combinational readout of psum[] registers.
    //     Bit slice [j*PSUM_W +: PSUM_W] = final MAC result for column j.
    //     Stable from the cycle mac_valid=1 until the next vector starts.
    //
    //   mac_valid : one-cycle pulse, fires one cycle after bs_done.
    //     Consumer (INT-to-FP accumulation unit) should latch mac_result_flat
    //     on the rising edge of mac_valid.
    // -------------------------------------------------------------------------
    output wire [NUM_COL*PSUM_W-1:0]   mac_result_flat,
    output reg                          mac_valid
);

    // =========================================================================
    // Block 1 : Instantiate cim_array
    //
    //   cim_array provides a purely combinational partial_sum_flat each cycle.
    //   It also contains the weight SRAM (synchronous write, async read).
    // =========================================================================
    wire [NUM_COL*8-1:0] partial_sum_flat;  // 64 × 8-bit, combinational

    cim_array #(
        .NUM_ROW (NUM_ROW),
        .NUM_COL (NUM_COL)
    ) u_cim_array (
        .clk              (clk),
        .rst_n            (rst_n),
        // Weight loading
        .wr_en            (wr_en),
        .wr_row           (wr_row),
        .wr_col           (wr_col),
        .wr_data          (wr_data),
        // Bit-serial activation
        .act_bs           (act_bs),
        // Per-cycle partial sum output
        .partial_sum_flat (partial_sum_flat)
    );

    // =========================================================================
    // Block 2 : First-Valid Edge Detector
    //
    //   first_valid is HIGH for exactly ONE cycle: the first cycle of a new
    //   MAC vector operation (rising edge of bs_valid).
    //
    //   This pulse serves two purposes:
    //     a) Triggers psum[] initialization (overwrite, not add) to clear
    //        any residual value from the previous vector.
    //     b) Triggers shift_cnt preload so it's ready for cycle 2.
    // =========================================================================
    reg bs_valid_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) bs_valid_d <= 1'b0;
        else        bs_valid_d <= bs_valid;
    end

    wire first_valid = bs_valid & ~bs_valid_d;  // rising edge of bs_valid

    // =========================================================================
    // Block 3 : Shift Counter  (shift_cnt) and MAC Active Flag (mac_active)
    //
    //   shift_cnt tracks the positional weight (bit position) to apply to
    //   the current cycle's partial_sum before accumulating into psum[]:
    //
    //     Cycle 1 (first_valid):  shift amount = total_cyc - 1  (MSB position)
    //                             shift_cnt ← total_cyc - 2  (ready for cycle 2)
    //     Cycle 2:                shift amount = shift_cnt = N-2
    //                             shift_cnt ← N-3
    //     ...
    //     Cycle N (bs_done):      shift amount = shift_cnt = 0
    //
    //   mac_active is HIGH from cycle 2 onwards until (and including) bs_done.
    //   Cycle 1 is handled separately via first_valid, so mac_active covers
    //   cycles 2..N only.  This prevents double-accumulation on cycle 1.
    //
    //   Priority:
    //     bs_done deasserts mac_active in the same cycle it is asserted,
    //     so mac_active goes LOW exactly at the end of cycle N.
    // =========================================================================
    reg [CYC_W-1:0] shift_cnt;
    reg              mac_active;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shift_cnt  <= {CYC_W{1'b0}};
            mac_active <= 1'b0;

        end else begin

            if (first_valid) begin
                // Cycle 1: preload shift_cnt for cycle 2 = N-2
                // Cycle 1's shift (N-1) is used inline in Block 4 below.
                mac_active <= 1'b1;
                shift_cnt  <= total_cyc - {{(CYC_W-2){1'b0}}, 2'b10}; // N-2

            end else if (mac_active & bs_valid) begin
                // Cycles 2..N: decrement shift_cnt each cycle
                shift_cnt  <= shift_cnt - {{(CYC_W-1){1'b0}}, 1'b1};  // -1
            end

            // Deassert mac_active at the end of the last cycle (bs_done=1)
            if (bs_done)
                mac_active <= 1'b0;

        end
    end

    // =========================================================================
    // Block 4 : Accumulator Array  (psum[])
    //
    //   psum[j] is a PSUM_W-bit accumulator register for output column j.
    //   There are NUM_COL = 64 such registers.
    //
    //   Update policy:
    //
    //     Cycle 1 (first_valid):
    //       OVERWRITE (not add) to clear any previous vector's residue.
    //       psum[j] ← zero_extend(partial_sum[j]) << (total_cyc - 1)
    //
    //     Cycles 2..N (mac_active & bs_valid):
    //       ACCUMULATE.
    //       psum[j] += zero_extend(partial_sum[j]) << shift_cnt
    //
    //   After cycle N (bs_done), psum[j] holds the complete integer MAC
    //   result for this vector, ready to be read by the downstream unit.
    //
    //   Note: the for loop over ci uses an integer variable.  This is
    //   valid Verilog-2001 for behavioral simulation and RTL synthesis.
    // =========================================================================
    reg signed [PSUM_W-1:0] psum [0:NUM_COL-1];

    integer ci;  // loop variable for column index

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset: clear all 64 accumulators
            for (ci = 0; ci < NUM_COL; ci = ci + 1)
                psum[ci] <= {PSUM_W{1'b0}};

        end else if (first_valid) begin
            // Cycle 1: initialize psum with MSB-weighted partial sum
            // Shift amount = total_cyc - 1
            for (ci = 0; ci < NUM_COL; ci = ci + 1)
                psum[ci] <= - (( {{(PSUM_W-8){1'b0}},
                               partial_sum_flat[ci*8 +: 8]}
                            ) << (total_cyc - {{(CYC_W-1){1'b0}}, 1'b1}));

        end else if (mac_active & bs_valid) begin
            // Cycles 2..N: accumulate with current shift_cnt
            // NOTE: extra outer parentheses are REQUIRED.
            // In Verilog, binary '+' has higher precedence than '<<'.
            // Without them:  psum + (B) << C = (psum + B) << C  ← WRONG
            // With them:     psum + ((B) << C)                  ← correct
            for (ci = 0; ci < NUM_COL; ci = ci + 1)
                psum[ci] <= psum[ci] +
                            (( {{(PSUM_W-8){1'b0}},
                                partial_sum_flat[ci*8 +: 8]}
                             ) << shift_cnt);
        end
        // No else branch: psum holds its value between vector operations.
    end

    // =========================================================================
    // Block 5 : Output Logic
    //
    //   mac_result_flat
    //     Combinational (wire) readout of psum[] register array.
    //     Becomes correct and stable one cycle AFTER bs_done, because:
    //       - At posedge(bs_done): psum[] latches the last partial sum (NBA).
    //       - At posedge(bs_done+1): mac_valid=1, psum[] is now fully settled.
    //     mac_result_flat remains stable until the next first_valid pulse
    //     overwrites psum[] at the start of the next vector.
    //
    //   mac_valid
    //     Registered one-cycle pulse.
    //     Pipeline: bs_done → bs_done_d (1 cycle) → mac_valid (1 cycle pulse)
    //     So mac_valid fires exactly one clock cycle after bs_done.
    //     Consumer must sample mac_result_flat while mac_valid is HIGH.
    // =========================================================================
    genvar out_j;
    generate
        for (out_j = 0; out_j < NUM_COL; out_j = out_j + 1) begin : OUT_MAP
            assign mac_result_flat[out_j*PSUM_W +: PSUM_W] = psum[out_j];
        end
    endgenerate

    reg bs_done_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bs_done_d <= 1'b0;
            mac_valid <= 1'b0;
        end else begin
            bs_done_d <= bs_done;
            mac_valid <= bs_done_d;  // one cycle after bs_done
        end
    end

endmodule
// =============================================================================
// End of cim_macro.v
// =============================================================================
