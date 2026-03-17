// =============================================================================
//  aligner_channel  (single-channel, instantiated NUM_CH times)
//
//  Implements the per-channel transfer logic of Fig. 7(a):
//
//  State machine:
//    S_IDLE    — inactive, waiting for next load
//    S_ALIGN   — Phase 1: sign-extension alignment
//                  Output sign bit (two's-complement MSB extension) each
//                  cycle while exp_cur < Emax.  exp_cur increments by 1
//                  per cycle until it reaches Emax, then transition to
//                  S_MANT.  Produces (Emax - E) output bits.
//    S_MANT    — Phase 2: mantissa serial output
//                  Output MSB of man_shift register, left-shift by 1 each
//                  cycle.  Register was loaded with two's-complement of
//                  {sign, hidden=1, mantissa}.  Produces MAN_W+2 bits.
//    S_ZERO    — Phase 3: zero right-padding
//                  Output logic 0 each cycle.  Counts down zero_rem from
//                  (E - Emin) to 0.  Produces (E - Emin) bits.
// =============================================================================
module aligner_channel #(
    parameter EXP_W = 5,
    parameter MAN_W = 10,
    parameter FP_W  = 16
)(
    input  wire              clk,
    input  wire              rst_n,
    input  wire              load,
    input  wire [EXP_W-1:0] emax,
    input  wire [EXP_W-1:0] emin,
    input  wire [FP_W-1:0]  fp_in,
    output reg               bit_out
);

    // -------------------------------------------------------------------------
    // FP16 field extraction
    // -------------------------------------------------------------------------
    wire              fp_sign = fp_in[FP_W-1];
    wire [EXP_W-1:0] fp_exp  = fp_in[FP_W-2 -: EXP_W];   // bits [14:10]
    wire [MAN_W-1:0] fp_man  = fp_in[MAN_W-1:0];           // bits  [9:0]

    // -------------------------------------------------------------------------
    // Two's-complement mantissa word  (Fig. 7a — "Transfer to 2's complement")
    //
    //   MAN_TC_W = MAN_W + 2  (sign | hidden | mantissa)
    //
    //   magnitude = {1'b1, fp_man}          (11-bit unsigned, hidden bit explicit)
    //
    //   fp_sign = 0  →  man_tc = {1'b0, magnitude}          (positive)
    //   fp_sign = 1  →  man_tc = {1'b1, ~magnitude + 1}     (negative, 2sC)
    //
    //   Overflow safety:  magnitude ≥ 2^MAN_W (hidden bit = 1)
    //     ∴  ~magnitude ≤ 2^MAN_W - 1  and  ~magnitude + 1 ≤ 2^MAN_W
    //     which always fits in MAN_W+1 bits. No overflow possible.
    // -------------------------------------------------------------------------
    localparam MAN_TC_W = MAN_W + 2;    // 12 bits for FP16

    wire [MAN_W:0]      magnitude = {1'b1, fp_man};
    wire [MAN_TC_W-1:0] man_tc    = fp_sign
                                    ? {1'b1, (~magnitude + 1'b1)}   // negative
                                    : {1'b0,   magnitude          }; // positive

    // -------------------------------------------------------------------------
    // State encoding
    // -------------------------------------------------------------------------
    localparam [1:0]
        S_IDLE  = 2'd0,
        S_ALIGN = 2'd1,
        S_MANT  = 2'd2,
        S_ZERO  = 2'd3;

    // -------------------------------------------------------------------------
    // Internal registers
    // -------------------------------------------------------------------------
    reg [1:0]                        state;
    reg [EXP_W-1:0]                  exp_cur;    // exponent counter (ALIGN phase)
    reg [MAN_TC_W-1:0]               man_shift;  // two's-complement shift register
    reg [EXP_W-1:0]                  zero_rem;   // remaining zero-padding cycles
    reg [$clog2(MAN_TC_W+1)-1:0]     mant_cnt;   // mantissa bits output so far
    reg                              sign_bit;   // latched sign for ALIGN output

    // -------------------------------------------------------------------------
    // Sequential state machine
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            bit_out   <= 1'b0;
            exp_cur   <= {EXP_W{1'b0}};
            man_shift <= {MAN_TC_W{1'b0}};
            zero_rem  <= {EXP_W{1'b0}};
            mant_cnt  <= 0;
            sign_bit  <= 1'b0;
        end

        // ─────────────────────────────────────────────────────────────────────
        // LOAD: latch new FP16 value and initialise state for next cycle.
        //       bit_out is NOT driven here; the first valid bit appears the
        //       cycle AFTER load, matching the bs_valid timing in the top module.
        // ─────────────────────────────────────────────────────────────────────
        else if (load) begin
            sign_bit  <= fp_sign;
            zero_rem  <= fp_exp - emin;   // (E – Emin) zero-pad cycles; 0 if E==Emin

            // If exponent is below Emax, sign-extension alignment is needed first.
            // If exponent equals Emax, proceed directly to mantissa output.
            //
            // bit_out is driven HERE (not deferred to next cycle) so that it
            // is aligned with bs_valid which goes HIGH on this same posedge.
            if (fp_exp < emax) begin
                bit_out   <= fp_sign;           // first output = sign extension
                man_shift <= man_tc;
                mant_cnt  <= 0;
                if (fp_exp + 1'b1 == emax) begin
                    // Only 1 alignment bit needed — already output above.
                    // Go directly to S_MANT.
                    state   <= S_MANT;
                    exp_cur <= emax;
                end else begin
                    // Multiple alignment bits needed — enter S_ALIGN.
                    // exp_cur starts at fp_exp+1 because one cycle is consumed here.
                    state   <= S_ALIGN;
                    exp_cur <= fp_exp + 1'b1;
                end
            end else begin
                state     <= S_MANT;
                bit_out   <= man_tc[MAN_TC_W-1]; // first output = MSB of 2sC word
                man_shift <= {man_tc[MAN_TC_W-2:0], 1'b0}; // advance shift reg
                mant_cnt  <= 1;                  // one bit already output
                exp_cur   <= fp_exp;
            end
        end

        // ─────────────────────────────────────────────────────────────────────
        // Normal output phases
        // ─────────────────────────────────────────────────────────────────────
        else begin
            case (state)

                // ─────────────────────────────────────────────────────────────
                // S_ALIGN  —  Phase 1: sign-extension bits
                //
                //   Outputs the sign bit (two's-complement arithmetic shift)
                //   each cycle until exp_cur reaches Emax.
                //   exp_cur increments by 1 per cycle.
                //   Duration: (Emax − E) cycles.
                // ─────────────────────────────────────────────────────────────
                S_ALIGN: begin
                    bit_out <= sign_bit;

                    if (exp_cur + 1'b1 == emax) begin
                        // Last alignment cycle — transition on next clock edge
                        exp_cur  <= emax;
                        state    <= S_MANT;
                        mant_cnt <= 0;
                    end else begin
                        exp_cur <= exp_cur + 1'b1;
                    end
                end

                // ─────────────────────────────────────────────────────────────
                // S_MANT  —  Phase 2: two's-complement mantissa word, MSB first
                //
                //   man_shift was loaded with man_tc = {S, hidden=1, M} in 2sC.
                //   Each cycle: output MSB, left-shift register by 1 (fill 0).
                //   Duration: MAN_W + 2 cycles  (12 cycles for FP16).
                //   After last bit, transition to S_ZERO (if padding needed)
                //   or S_IDLE (no padding, E == Emin).
                // ─────────────────────────────────────────────────────────────
                S_MANT: begin
                    bit_out   <= man_shift[MAN_TC_W-1];                    // MSB out
                    man_shift <= {man_shift[MAN_TC_W-2:0], 1'b0};          // left shift
                    mant_cnt  <= mant_cnt + 1'b1;

                    // Check uses OLD (pre-increment) value of mant_cnt
                    if (mant_cnt == MAN_TC_W - 1) begin
                        state <= (|zero_rem) ? S_ZERO : S_IDLE;
                    end
                end

                // ─────────────────────────────────────────────────────────────
                // S_ZERO  —  Phase 3: zero right-padding
                //
                //   Outputs logic 0 each cycle to right-align the mantissa word
                //   to the fixed-point LSB position.
                //   Duration: (E − Emin) cycles.
                // ─────────────────────────────────────────────────────────────
                S_ZERO: begin
                    bit_out  <= 1'b0;
                    zero_rem <= zero_rem - 1'b1;
                    if (zero_rem == 1'b1)
                        state <= S_IDLE;
                end

                // ─────────────────────────────────────────────────────────────
                // S_IDLE  —  inactive
                // ─────────────────────────────────────────────────────────────
                default: begin
                    bit_out <= 1'b0;
                end

            endcase
        end
    end

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
