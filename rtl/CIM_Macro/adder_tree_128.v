// =============================================================================
// File    : adder_tree_128.v
// Module  : adder_tree_128
// Desc    : 128-input 1-bit balanced binary adder tree.
//           Pure combinational logic. No clock, no reset.
//
// Function:
//   sum = in[0] + in[1] + ... + in[127]   (unsigned integer sum)
//
// Tree structure (7 stages):
//   ST1 : 128 × 1b  →  64 × 2b
//   ST2 :  64 × 2b  →  32 × 3b
//   ST3 :  32 × 3b  →  16 × 4b
//   ST4 :  16 × 4b  →   8 × 5b
//   ST5 :   8 × 5b  →   4 × 6b
//   ST6 :   4 × 6b  →   2 × 7b
//   ST7 :   2 × 7b  →   1 × 8b  (output)
//
// Output range: 0 ~ 128, requiring exactly 8 bits.
//
// Usage in cim_array.v:
//   Instantiated 64 times, once per output column.
//   Each instance sums 128 AND-product bits for that column.
// =============================================================================

`timescale 1ns/1ps

module adder_tree_128 (
    input  wire [127:0] in,   // 128 one-bit inputs (AND products from one column)
    output wire [  7:0] sum   // unsigned 8-bit sum, range [0, 128]
);

    genvar i;

    // -------------------------------------------------------------------------
    // Stage 1 : 128 × 1b  →  64 × 2b
    //   Each pair of adjacent 1-bit inputs is added to produce a 2-bit result.
    //   Zero-extend to prevent width mismatch in the addition.
    // -------------------------------------------------------------------------
    wire [1:0] s1 [0:63];
    generate
        for (i = 0; i < 64; i = i + 1) begin : ST1
            assign s1[i] = {1'b0, in[2*i]} + {1'b0, in[2*i+1]};
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Stage 2 :  64 × 2b  →  32 × 3b
    // -------------------------------------------------------------------------
    wire [2:0] s2 [0:31];
    generate
        for (i = 0; i < 32; i = i + 1) begin : ST2
            assign s2[i] = {1'b0, s1[2*i]} + {1'b0, s1[2*i+1]};
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Stage 3 :  32 × 3b  →  16 × 4b
    // -------------------------------------------------------------------------
    wire [3:0] s3 [0:15];
    generate
        for (i = 0; i < 16; i = i + 1) begin : ST3
            assign s3[i] = {1'b0, s2[2*i]} + {1'b0, s2[2*i+1]};
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Stage 4 :  16 × 4b  →   8 × 5b
    // -------------------------------------------------------------------------
    wire [4:0] s4 [0:7];
    generate
        for (i = 0; i < 8; i = i + 1) begin : ST4
            assign s4[i] = {1'b0, s3[2*i]} + {1'b0, s3[2*i+1]};
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Stage 5 :   8 × 5b  →   4 × 6b
    // -------------------------------------------------------------------------
    wire [5:0] s5 [0:3];
    generate
        for (i = 0; i < 4; i = i + 1) begin : ST5
            assign s5[i] = {1'b0, s4[2*i]} + {1'b0, s4[2*i+1]};
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Stage 6 :   4 × 6b  →   2 × 7b
    // -------------------------------------------------------------------------
    wire [6:0] s6 [0:1];
    generate
        for (i = 0; i < 2; i = i + 1) begin : ST6
            assign s6[i] = {1'b0, s5[2*i]} + {1'b0, s5[2*i+1]};
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Stage 7 :   2 × 7b  →   1 × 8b  (final output)
    // -------------------------------------------------------------------------
    assign sum = {1'b0, s6[0]} + {1'b0, s6[1]};

endmodule
// =============================================================================
// End of adder_tree_128.v
// =============================================================================
