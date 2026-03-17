// =============================================================================
// File    : cim_array.v
// Module  : cim_array
// Desc    : CIM core array: weight SRAM + AND multiply array + adder trees.
//           Performs one cycle of bit-serial MAC for all 64 output columns.
//
// -----------------------------------------------------------------------
// Role in the three-module hierarchy:
//
//   adder_tree_128.v   ← Layer 1: pure combinational sum tree
//          ↑
//   cim_array.v        ← Layer 2: SRAM + AND + 64 adder trees  [THIS FILE]
//          ↑
//   cim_macro.v        ← Layer 3: shift-accumulate control + output
//
// -----------------------------------------------------------------------
// Internal structure:
//
//   ┌─────────────────────────────────────────────────────────────────┐
//   │  weight_mem[128][64]   (1-bit per cell, synchronous write)      │
//   │  Read port: combinational, feeds AND array directly             │
//   └───────────────────────┬─────────────────────────────────────────┘
//                           │ weight_mem[i][j]  for all i, j
//   act_bs[127:0] ──────────┼──────────────────────────────────────────
//                           │
//   ┌───────────────────────▼─────────────────────────────────────────┐
//   │  AND Array  (128 × 64 = 8192 AND gates, all combinational)     │
//   │  and_vec[j][i] = act_bs[i] & weight_mem[i][j]                  │
//   └───────────────────────┬─────────────────────────────────────────┘
//                           │ and_vec[j][127:0]  for each column j
//   ┌───────────────────────▼─────────────────────────────────────────┐
//   │  64 × adder_tree_128  (each sums 128 bits → 8-bit result)      │
//   │  partial_sum[j] = Σ_{i=0}^{127} and_vec[j][i]                  │
//   └───────────────────────┬─────────────────────────────────────────┘
//                           │
//             partial_sum_flat[64*8-1 : 0]
//             Bit slice [j*8 +: 8] = partial sum for column j
//
// -----------------------------------------------------------------------
// Key design decisions:
//
//   1. Weight SRAM is 1-bit per cell.  Each row of the memory is a
//      64-bit register word; bit[col] holds the weight for that cell.
//
//   2. Weight write is SYNCHRONOUS (posedge clk), one cell per cycle.
//      The upstream controller (cim_macro) must ensure wr_en=0 during
//      MAC execution to avoid read-write hazards.
//
//   3. All combinational paths (AND array + adder trees) have no
//      pipeline registers.  Timing closure is the synthesizer's job.
//
//   4. partial_sum_flat is purely combinational.  It changes immediately
//      when act_bs changes.  The accumulator in cim_macro registers it.
// =============================================================================

`timescale 1ns/1ps

module cim_array #(
    parameter NUM_ROW = 128,   // Number of input activation channels
    parameter NUM_COL = 64     // Number of output channels
)(
    input  wire                       clk,    // clock (for SRAM write only)
    input  wire                       rst_n,  // active-low synchronous reset

    // -------------------------------------------------------------------------
    // Port Group A : Weight Loading Interface
    //   Write 1-bit weights one cell at a time.
    //   wr_en MUST be LOW during MAC cycles.
    // -------------------------------------------------------------------------
    input  wire                       wr_en,   // write enable, active-high
    input  wire [6:0]                 wr_row,  // row address [0, NUM_ROW-1]
    input  wire [5:0]                 wr_col,  // col address [0, NUM_COL-1]
    input  wire                       wr_data, // 1-bit weight value to write

    // -------------------------------------------------------------------------
    // Port Group B : Bit-Serial Activation Input
    //   Driven each cycle by cim_macro while bs_valid is HIGH.
    //   act_bs must be stable for the entire cycle.
    // -------------------------------------------------------------------------
    input  wire [NUM_ROW-1:0]         act_bs,  // 128 bit-serial activation bits

    // -------------------------------------------------------------------------
    // Port Group C : Per-Cycle Partial Sum Output (combinational)
    //   partial_sum_flat[ j*8 +: 8 ] = number of act_bs[i]=1 bits where
    //   weight_mem[i][j]=1, for column j.
    //   This is the raw dot-product for this single bit-serial cycle.
    //   cim_macro shift-accumulates this over total_cyc cycles.
    // -------------------------------------------------------------------------
    output wire [NUM_COL*8-1:0]       partial_sum_flat  // 64 × 8-bit, combinational
);

    // =========================================================================
    // Block 1 : Weight SRAM
    //
    //   weight_mem[i] is a NUM_COL-bit word.
    //   weight_mem[i][j] = weight value at row i, column j.
    //
    //   Organized this way (row-indexed) so that the AND array can read
    //   all columns of a given row simultaneously via weight_mem[i][j].
    //
    //   Write : synchronous on posedge clk.
    //   Read  : asynchronous (purely combinational assignment below).
    // =========================================================================
    reg [NUM_COL-1:0] weight_mem [0:NUM_ROW-1];

    integer init_i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Clear all weights on reset.
            for (init_i = 0; init_i < NUM_ROW; init_i = init_i + 1)
                weight_mem[init_i] <= {NUM_COL{1'b0}};
        end else if (wr_en) begin
            // Single-cell write: update bit wr_col in row wr_row.
            weight_mem[wr_row][wr_col] <= wr_data;
        end
    end

    // =========================================================================
    // Block 2 : AND Multiply Array  +  64 × adder_tree_128
    //
    //   For each output column j:
    //     1. Compute 128 AND products: and_vec[i] = act_bs[i] & weight_mem[i][j]
    //     2. Sum them with adder_tree_128: partial_sum = Σ and_vec
    //     3. Pack into partial_sum_flat[j*8 +: 8]
    //
    //   The entire Block 2 is purely combinational.
    //   Synthesis will flatten the generate loops into gate-level logic.
    // =========================================================================
    genvar j, k;
    generate
        for (j = 0; j < NUM_COL; j = j + 1) begin : COL_INST

            // ------------------------------------------------------------------
            // Step 2a : AND products for column j
            //   and_vec[k] = 1  iff  activation channel k is 1  AND
            //                        weight at (row=k, col=j) is 1.
            //   This is 1-bit × 1-bit multiplication.
            // ------------------------------------------------------------------
            wire [NUM_ROW-1:0] and_vec;
            for (k = 0; k < NUM_ROW; k = k + 1) begin : AND_CELL
                assign and_vec[k] = act_bs[k] & weight_mem[k][j];
            end

            // ------------------------------------------------------------------
            // Step 2b : 128-input adder tree for column j
            //   Instantiate adder_tree_128, summing all 128 AND products.
            //   Output: 8-bit unsigned count (how many products are 1).
            // ------------------------------------------------------------------
            wire [7:0] col_psum;
            adder_tree_128 u_atree (
                .in  ({{(128-NUM_ROW){1'b0}}, and_vec}),  // zero-extend if NUM_ROW < 128
                .sum (col_psum)
            );

            // ------------------------------------------------------------------
            // Step 2c : Pack into the flat output bus
            // ------------------------------------------------------------------
            assign partial_sum_flat[j*8 +: 8] = col_psum;

        end
    endgenerate

endmodule
// =============================================================================
// End of cim_array.v
// =============================================================================
