// =============================================================================
// File    : tb_cim_macro.v
// Desc    : Testbench for signed cim_macro.v
//
// Compile & run:
//   iverilog -g2001 -o sim_cim.out \
//            adder_tree_128.v cim_array.v cim_macro.v tb_cim_macro.v
//   vvp sim_cim.out
//   gtkwave tb_cim_macro.vcd
//
// -----------------------------------------------------------------------
// DUT parameters (reduced for tractability):
//   NUM_ROW = 8   (paper: 128)
//   NUM_COL = 4   (paper:  64)
//   PSUM_W  = 32
//   CYC_W   = 5
//
// -----------------------------------------------------------------------
// SIGNED BIT-SERIAL ENCODING REVIEW
//
//   aligner.v outputs the bit-serial stream MSB first (sign bit first).
//   For a two's-complement N-bit integer X:
//
//     X  =  -x[N-1]*2^(N-1)  +  Sum_{k=0}^{N-2} x[k]*2^k
//
//   Mapping to drive_mac cycles (cycle index c = 0..N-1):
//     c = 0  ->  sign bit  ->  weight = -2^(N-1)   (SUBTRACT in psum)
//     c = 1  ->  bit N-2   ->  weight = +2^(N-2)   (ADD)
//     ...
//     c = N-1 -> bit 0     ->  weight = +2^0        (ADD)
//
//   Examples (4-bit integers, total_cyc = 4):
//     +5 = 0101  -> c0=0, c1=1, c2=0, c3=1
//     -3 = 1101  -> c0=1, c1=1, c2=0, c3=1
//     -8 = 1000  -> c0=1, c1=0, c2=0, c3=0  (minimum 4-bit value)
//     +7 = 0111  -> c0=0, c1=1, c2=1, c3=1  (maximum 4-bit value)
//
// -----------------------------------------------------------------------
// act_flat layout:
//   act_flat[c * NUM_ROW +: NUM_ROW] = act_bs presented at cycle c.
//   Cycle 0 is the SIGN BIT cycle (first, highest shift weight).
//
// -----------------------------------------------------------------------
// Test case overview:
//   TC0  : Reference model sanity (no DUT) -- single channel = -3
//   TC1  : All channels = +5,  all weights = 1  -> expected = +40
//   TC2  : All channels = -8,  all weights = 1  -> expected = -64
//   TC3  : Rows 0-3 = +5, rows 4-7 = -3, all weights = 1  -> expected = +8
//   TC4  : Zero result: rows 0-3 = +4, rows 4-7 = -4  -> expected = 0
//   TC5  : total_cyc=1 (only sign bit), rows{1,3,5,7} active -> exp = -4
//   TC6  : total_cyc=5, all channels = -5, all weights = 1  -> exp = -40
//   TC7  : Checkerboard weights, mixed signs  -> expected = +4
//   TC8a : Back-to-back vector A (= TC1)  -> expected = +40
//   TC8b : Back-to-back vector B (verify no residue)  -> expected = +7
//   TC9  : Reset mid-operation -> mac_valid must NOT fire
//   TC9b : Post-reset recovery  -> expected = +40
//   TC10 : Cyclic-identity weights, mixed signs  -> expected = +4
// =============================================================================

`timescale 1ns/1ps

module tb_cim_macro;

    // =========================================================================
    // Parameters
    // =========================================================================
    localparam NUM_ROW = 8;
    localparam NUM_COL = 4;
    localparam PSUM_W  = 32;
    localparam CYC_W   = 5;
    localparam CLK_T   = 10;

    localparam MAX_CYC = 16;
    localparam AF_W    = MAX_CYC * NUM_ROW;   // = 128 bits

    // =========================================================================
    // DUT ports
    // =========================================================================
    reg                        clk;
    reg                        rst_n;
    reg                        wr_en;
    reg  [6:0]                 wr_row;
    reg  [5:0]                 wr_col;
    reg                        wr_data;
    reg  [NUM_ROW-1:0]         act_bs;
    reg                        bs_valid;
    reg                        bs_done;
    reg  [CYC_W-1:0]          total_cyc;
    wire [NUM_COL*PSUM_W-1:0]  mac_result_flat;
    wire                        mac_valid;

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    cim_macro #(
        .NUM_ROW (NUM_ROW),
        .NUM_COL (NUM_COL),
        .PSUM_W  (PSUM_W),
        .CYC_W   (CYC_W)
    ) dut (
        .clk             (clk),
        .rst_n           (rst_n),
        .wr_en           (wr_en),
        .wr_row          (wr_row),
        .wr_col          (wr_col),
        .wr_data         (wr_data),
        .act_bs          (act_bs),
        .bs_valid        (bs_valid),
        .bs_done         (bs_done),
        .total_cyc       (total_cyc),
        .mac_result_flat (mac_result_flat),
        .mac_valid       (mac_valid)
    );

    // =========================================================================
    // Clock
    // =========================================================================
    initial clk = 1'b0;
    always  #(CLK_T/2) clk = ~clk;

    // =========================================================================
    // Module-level shared state
    // =========================================================================
    reg weight_ref [0:NUM_ROW-1][0:NUM_COL-1];
    reg signed [PSUM_W-1:0] exp_result [0:NUM_COL-1];

    integer lp_r, lp_c, lp_cyc, lp_col, lp_k;
    reg ok_flag;
    reg [AF_W-1:0] act_flat;
    integer pass_cnt, fail_cnt;

    // =========================================================================
    // Task: do_reset
    // =========================================================================
    task do_reset;
        begin
            rst_n     = 1'b0;
            wr_en     = 1'b0;
            wr_row    = 7'd0;
            wr_col    = 6'd0;
            wr_data   = 1'b0;
            act_bs    = {NUM_ROW{1'b0}};
            bs_valid  = 1'b0;
            bs_done   = 1'b0;
            total_cyc = {CYC_W{1'b0}};
            repeat(3) @(posedge clk);
            @(negedge clk); rst_n = 1'b1;
            @(posedge clk); #1;
        end
    endtask

    // =========================================================================
    // Task: load_weights
    //   w_flat bit index = r * NUM_COL + c = weight(row r, col c).
    // =========================================================================
    task load_weights;
        input [31:0] w_flat;
        begin
            wr_en = 1'b0;
            for (lp_r = 0; lp_r < NUM_ROW; lp_r = lp_r + 1) begin
                for (lp_c = 0; lp_c < NUM_COL; lp_c = lp_c + 1) begin
                    @(negedge clk);
                    wr_en   = 1'b1;
                    wr_row  = lp_r[6:0];
                    wr_col  = lp_c[5:0];
                    wr_data = w_flat[lp_r * NUM_COL + lp_c];
                    weight_ref[lp_r][lp_c] = w_flat[lp_r * NUM_COL + lp_c];
                end
            end
            @(negedge clk); wr_en = 1'b0;
        end
    endtask

    // =========================================================================
    // Task: compute_expected  -- SIGNED reference model
    //
    //   c=0 : sign bit -> SUBTRACT (ps << (N-1))
    //   c>0 : magnitude bit -> ADD (ps << (N-1-c))
    // =========================================================================
    task compute_expected;
        input      [CYC_W-1:0] total_n;
        input      [AF_W-1:0]  af;
        reg        [NUM_ROW-1:0] act_c;
        reg        [7:0]         ps;
        reg signed [PSUM_W-1:0] contrib;
        begin
            for (lp_col = 0; lp_col < NUM_COL; lp_col = lp_col + 1)
                exp_result[lp_col] = {PSUM_W{1'b0}};

            for (lp_cyc = 0; lp_cyc < total_n; lp_cyc = lp_cyc + 1) begin
                act_c = af[lp_cyc * NUM_ROW +: NUM_ROW];
                for (lp_col = 0; lp_col < NUM_COL; lp_col = lp_col + 1) begin
                    ps = 8'd0;
                    for (lp_r = 0; lp_r < NUM_ROW; lp_r = lp_r + 1)
                        if (act_c[lp_r] & weight_ref[lp_r][lp_col])
                            ps = ps + 8'd1;
                    contrib = $signed({{(PSUM_W-8){1'b0}}, ps})
                              << (total_n - 1 - lp_cyc);
                    if (lp_cyc == 0)
                        exp_result[lp_col] = exp_result[lp_col] - contrib;
                    else
                        exp_result[lp_col] = exp_result[lp_col] + contrib;
                end
            end
        end
    endtask

    // =========================================================================
    // Task: drive_mac
    // =========================================================================
    task drive_mac;
        input [CYC_W-1:0] total_n;
        input [AF_W-1:0]  af;
        begin
            wr_en     = 1'b0;
            bs_valid  = 1'b0;
            bs_done   = 1'b0;
            act_bs    = {NUM_ROW{1'b0}};
            total_cyc = total_n;
            for (lp_cyc = 0; lp_cyc < total_n; lp_cyc = lp_cyc + 1) begin
                @(negedge clk);
                act_bs   = af[lp_cyc * NUM_ROW +: NUM_ROW];
                bs_valid = 1'b1;
                bs_done  = (lp_cyc == total_n - 1) ? 1'b1 : 1'b0;
            end
            @(negedge clk);
            act_bs   = {NUM_ROW{1'b0}};
            bs_valid = 1'b0;
            bs_done  = 1'b0;
        end
    endtask

    // =========================================================================
    // Task: check_result
    // =========================================================================
    task check_result;
        input [31:0] tc_id;
        reg signed [PSUM_W-1:0] got;
        begin
            ok_flag = 1'b1;
            @(posedge clk); #1;
            if (!mac_valid) begin
                $display("[TC%0d FAIL] mac_valid did not fire", tc_id);
                ok_flag = 1'b0;
            end
            for (lp_col = 0; lp_col < NUM_COL; lp_col = lp_col + 1) begin
                got = $signed(mac_result_flat[lp_col * PSUM_W +: PSUM_W]);
                if (got !== exp_result[lp_col]) begin
                    $display("[TC%0d FAIL] col=%0d  expected=%0d  got=%0d",
                             tc_id, lp_col,
                             $signed(exp_result[lp_col]), got);
                    ok_flag = 1'b0;
                end
            end
            @(posedge clk); #1;
            if (mac_valid) begin
                $display("[TC%0d FAIL] mac_valid stayed HIGH >1 cycle", tc_id);
                ok_flag = 1'b0;
            end
            if (ok_flag) begin
                $display("[TC%0d PASS]", tc_id);
                pass_cnt = pass_cnt + 1;
            end else
                fail_cnt = fail_cnt + 1;
        end
    endtask

    // =========================================================================
    // Main test sequence
    // =========================================================================
    initial begin
        $dumpfile("tb_cim_macro.vcd");
        $dumpvars(0, tb_cim_macro);
        pass_cnt = 0;
        fail_cnt = 0;
        act_flat = {AF_W{1'b0}};
        do_reset;

        $display("=====================================================");
        $display(" CIM Macro Testbench  (SIGNED)");
        $display(" NUM_ROW=%0d  NUM_COL=%0d  PSUM_W=%0d", NUM_ROW, NUM_COL, PSUM_W);
        $display("=====================================================");

        // -----------------------------------------------------------------
        // TC0 : Reference model sanity (no DUT)
        //   Encode -3 = 1101 (4-bit) on row 0, weight[0][0]=1 only.
        //   c0(sign)=1 -> -1x8=-8; c1=1 -> +1x4=+4; c2=0; c3=1 -> +1x1=+1
        //   col0 = -8+4+0+1 = -3; col1,2,3 = 0
        // -----------------------------------------------------------------
        $display("--- TC0: Ref model sanity (value=-3) ---");
        for (lp_r = 0; lp_r < NUM_ROW; lp_r = lp_r + 1)
            for (lp_c = 0; lp_c < NUM_COL; lp_c = lp_c + 1)
                weight_ref[lp_r][lp_c] = (lp_r==0 && lp_c==0) ? 1 : 0;
        act_flat = {AF_W{1'b0}};
        act_flat[0*NUM_ROW +: NUM_ROW] = 8'h01;
        act_flat[1*NUM_ROW +: NUM_ROW] = 8'h01;
        act_flat[2*NUM_ROW +: NUM_ROW] = 8'h00;
        act_flat[3*NUM_ROW +: NUM_ROW] = 8'h01;
        compute_expected(5'd4, act_flat);
        ok_flag = 1'b1;
        if (exp_result[0] !== -32'sd3) begin
            $display("[TC0 FAIL] col0 ref=%0d expected=-3", $signed(exp_result[0]));
            ok_flag = 1'b0;
        end
        for (lp_col = 1; lp_col < NUM_COL; lp_col = lp_col + 1)
            if (exp_result[lp_col] !== 32'sd0) begin
                $display("[TC0 FAIL] col%0d ref=%0d expected=0",
                         lp_col, $signed(exp_result[lp_col]));
                ok_flag = 1'b0;
            end
        if (ok_flag) begin
            $display("[TC0 PASS] col0=-3, others=0");
            pass_cnt = pass_cnt + 1;
        end else
            fail_cnt = fail_cnt + 1;

        // -----------------------------------------------------------------
        // TC1 : All channels = +5 = 0101 (4-bit), all weights = 1
        //   c0(sign): 8'h00 -> 0;  c1: 8'hFF -> +8x4=+32
        //   c2: 8'h00 -> 0;        c3: 8'hFF -> +8x1=+8
        //   psum = 0+32+0+8 = +40;  Verify: 8x(+5) = +40
        // -----------------------------------------------------------------
        $display("--- TC1: All channels=+5, all weights=1, expected=+40 ---");
        load_weights(32'hFFFF_FFFF);
        act_flat = {AF_W{1'b0}};
        act_flat[0*NUM_ROW +: NUM_ROW] = 8'h00;
        act_flat[1*NUM_ROW +: NUM_ROW] = 8'hFF;
        act_flat[2*NUM_ROW +: NUM_ROW] = 8'h00;
        act_flat[3*NUM_ROW +: NUM_ROW] = 8'hFF;
        compute_expected(5'd4, act_flat);
        drive_mac(5'd4, act_flat);
        check_result(1);

        // -----------------------------------------------------------------
        // TC2 : All channels = -8 = 1000 (4-bit), all weights = 1
        //   c0(sign): 8'hFF -> 8 -> -8x8=-64; c1,c2,c3: 8'h00 -> 0
        //   psum = -64;  Verify: 8x(-8) = -64
        // -----------------------------------------------------------------
        $display("--- TC2: All channels=-8, all weights=1, expected=-64 ---");
        load_weights(32'hFFFF_FFFF);
        act_flat = {AF_W{1'b0}};
        act_flat[0*NUM_ROW +: NUM_ROW] = 8'hFF;
        act_flat[1*NUM_ROW +: NUM_ROW] = 8'h00;
        act_flat[2*NUM_ROW +: NUM_ROW] = 8'h00;
        act_flat[3*NUM_ROW +: NUM_ROW] = 8'h00;
        compute_expected(5'd4, act_flat);
        drive_mac(5'd4, act_flat);
        check_result(2);

        // -----------------------------------------------------------------
        // TC3 : Rows 0-3=+5, rows 4-7=-3, all weights=1
        //   +5=0101, -3=1101  (4-bit)
        //   c0(sign): 8'hF0 -> ps=4 -> -4x8=-32
        //   c1: 8'hFF -> ps=8 -> +8x4=+32
        //   c2: 8'h00 -> 0
        //   c3: 8'hFF -> ps=8 -> +8x1=+8
        //   psum = -32+32+0+8 = +8;  Verify: 4x(+5)+4x(-3) = 20-12 = +8
        // -----------------------------------------------------------------
        $display("--- TC3: Rows 0-3=+5, rows 4-7=-3, all weights=1, expected=+8 ---");
        load_weights(32'hFFFF_FFFF);
        act_flat = {AF_W{1'b0}};
        act_flat[0*NUM_ROW +: NUM_ROW] = 8'hF0;
        act_flat[1*NUM_ROW +: NUM_ROW] = 8'hFF;
        act_flat[2*NUM_ROW +: NUM_ROW] = 8'h00;
        act_flat[3*NUM_ROW +: NUM_ROW] = 8'hFF;
        compute_expected(5'd4, act_flat);
        drive_mac(5'd4, act_flat);
        check_result(3);

        // -----------------------------------------------------------------
        // TC4 : Rows 0-3=+4, rows 4-7=-4, all weights=1 -> expected=0
        //   +4=0100, -4=1100  (4-bit)
        //   c0(sign): 8'hF0 -> ps=4 -> -4x8=-32
        //   c1: 8'hFF -> ps=8 -> +8x4=+32
        //   c2,c3: 8'h00 -> 0
        //   psum = 0;  Verify: 4x(+4)+4x(-4) = 0
        // -----------------------------------------------------------------
        $display("--- TC4: Rows 0-3=+4, rows 4-7=-4, expected=0 ---");
        load_weights(32'hFFFF_FFFF);
        act_flat = {AF_W{1'b0}};
        act_flat[0*NUM_ROW +: NUM_ROW] = 8'hF0;
        act_flat[1*NUM_ROW +: NUM_ROW] = 8'hFF;
        act_flat[2*NUM_ROW +: NUM_ROW] = 8'h00;
        act_flat[3*NUM_ROW +: NUM_ROW] = 8'h00;
        compute_expected(5'd4, act_flat);
        drive_mac(5'd4, act_flat);
        check_result(4);

        // -----------------------------------------------------------------
        // TC5 : total_cyc=1 edge case (only sign bit cycle)
        //   N=1: single cycle is sign bit, shift=0.
        //   psum = -(partial_sum << 0) = -partial_sum
        //   Rows {1,3,5,7} active (8'hAA), all weights=1.
        //   ps=4 -> psum = -4
        // -----------------------------------------------------------------
        $display("--- TC5: total_cyc=1 (only sign bit), expected=-4 ---");
        load_weights(32'hFFFF_FFFF);
        act_flat = {AF_W{1'b0}};
        act_flat[0*NUM_ROW +: NUM_ROW] = 8'hAA;
        compute_expected(5'd1, act_flat);
        drive_mac(5'd1, act_flat);
        check_result(5);

        // -----------------------------------------------------------------
        // TC6 : total_cyc=5, all channels=-5, all weights=1
        //   -5 = 11011 (5-bit 2's complement: -16+8+0+2+1=-5)
        //   c0(sign,shift=4): 8'hFF -> 8 -> -8x16=-128
        //   c1(shift=3):      8'hFF -> 8 -> +8x8=+64
        //   c2(shift=2):      8'h00 -> 0
        //   c3(shift=1):      8'hFF -> 8 -> +8x2=+16
        //   c4(shift=0):      8'hFF -> 8 -> +8x1=+8
        //   psum = -128+64+0+16+8 = -40;  Verify: 8x(-5) = -40
        // -----------------------------------------------------------------
        $display("--- TC6: total_cyc=5, all channels=-5, expected=-40 ---");
        load_weights(32'hFFFF_FFFF);
        act_flat = {AF_W{1'b0}};
        act_flat[0*NUM_ROW +: NUM_ROW] = 8'hFF;
        act_flat[1*NUM_ROW +: NUM_ROW] = 8'hFF;
        act_flat[2*NUM_ROW +: NUM_ROW] = 8'h00;
        act_flat[3*NUM_ROW +: NUM_ROW] = 8'hFF;
        act_flat[4*NUM_ROW +: NUM_ROW] = 8'hFF;
        compute_expected(5'd5, act_flat);
        drive_mac(5'd5, act_flat);
        check_result(6);

        // -----------------------------------------------------------------
        // TC7 : Checkerboard weights (w_flat=32'h5A5A_5A5A), mixed signs
        //   col0 active rows: {1,3,5,7};  col1 active rows: {0,2,4,6}
        //   Rows 0-3 = +3 = 011 (3-bit); rows 4-7 = -1 = 111 (3-bit)
        //   c0(sign,shift=2): 8'hF0; c1(shift=1): 8'hFF; c2(shift=0): 8'hFF
        //   For col0 (rows{1,3,5,7}):
        //     c0: {5,7} active -> ps=2 -> -2x4=-8
        //     c1: {1,3,5,7}    -> ps=4 -> +4x2=+8
        //     c2: {1,3,5,7}    -> ps=4 -> +4x1=+4
        //     psum = +4;  Verify: 2x(+3)+2x(-1) = 6-2 = +4
        //   All columns = +4 by symmetry.
        // -----------------------------------------------------------------
        $display("--- TC7: Checkerboard weights, mixed signs, expected=+4 ---");
        load_weights(32'h5A5A_5A5A);
        act_flat = {AF_W{1'b0}};
        act_flat[0*NUM_ROW +: NUM_ROW] = 8'hF0;
        act_flat[1*NUM_ROW +: NUM_ROW] = 8'hFF;
        act_flat[2*NUM_ROW +: NUM_ROW] = 8'hFF;
        compute_expected(5'd3, act_flat);
        drive_mac(5'd3, act_flat);
        check_result(7);

        // -----------------------------------------------------------------
        // TC8a/b : Back-to-back vectors
        //   TC8a: same as TC1 -> expected = +40
        //   TC8b: row 0 = +7 = 0111 (4-bit), all weights=1
        //     c0(sign): 8'h00->0; c1-c3: 8'h01->+1x4=4,+1x2=2,+1x1=1
        //     psum = 0+4+2+1 = +7
        //   Critical: if psum NOT overwritten, would be 40+7=47 (WRONG).
        // -----------------------------------------------------------------
        $display("--- TC8a: Back-to-back vector A, expected=+40 ---");
        load_weights(32'hFFFF_FFFF);
        act_flat = {AF_W{1'b0}};
        act_flat[0*NUM_ROW +: NUM_ROW] = 8'h00;
        act_flat[1*NUM_ROW +: NUM_ROW] = 8'hFF;
        act_flat[2*NUM_ROW +: NUM_ROW] = 8'h00;
        act_flat[3*NUM_ROW +: NUM_ROW] = 8'hFF;
        compute_expected(5'd4, act_flat);
        drive_mac(5'd4, act_flat);
        check_result(8);

        $display("--- TC8b: Back-to-back vector B (row0=+7), expected=+7 ---");
        act_flat = {AF_W{1'b0}};
        act_flat[0*NUM_ROW +: NUM_ROW] = 8'h00;
        act_flat[1*NUM_ROW +: NUM_ROW] = 8'h01;
        act_flat[2*NUM_ROW +: NUM_ROW] = 8'h01;
        act_flat[3*NUM_ROW +: NUM_ROW] = 8'h01;
        compute_expected(5'd4, act_flat);
        drive_mac(5'd4, act_flat);
        check_result(9);

        // -----------------------------------------------------------------
        // TC9 : Reset mid-operation
        //   Drive cycles 0 and 1 then assert rst_n=0.
        //   mac_valid must NOT fire during or after reset.
        // -----------------------------------------------------------------
        $display("--- TC9: Reset mid-operation (mac_valid must NOT fire) ---");
        load_weights(32'hFFFF_FFFF);
        total_cyc = 5'd4;
        @(negedge clk); act_bs=8'h00; bs_valid=1'b1; bs_done=1'b0;
        @(negedge clk); act_bs=8'hFF; bs_valid=1'b1; bs_done=1'b0;
        @(negedge clk);
        act_bs=8'h00; bs_valid=1'b0; bs_done=1'b0;
        rst_n = 1'b0;
        ok_flag = 1'b1;
        for (lp_k = 0; lp_k < 3; lp_k = lp_k + 1) begin
            @(posedge clk); #1;
            if (mac_valid) begin
                $display("[TC9 FAIL] mac_valid fired during reset (posedge %0d)", lp_k);
                ok_flag = 1'b0;
            end
        end
        @(negedge clk); rst_n = 1'b1;
        @(posedge clk); #1;
        if (mac_valid) begin
            $display("[TC9 FAIL] mac_valid fired after reset release");
            ok_flag = 1'b0;
        end
        if (ok_flag) begin
            $display("[TC9 PASS] mac_valid correctly suppressed throughout reset");
            pass_cnt = pass_cnt + 1;
        end else
            fail_cnt = fail_cnt + 1;

        // -----------------------------------------------------------------
        // TC9b : Post-reset recovery  -> expected = +40
        // -----------------------------------------------------------------
        $display("--- TC9b: Post-reset recovery, expected=+40 ---");
        load_weights(32'hFFFF_FFFF);
        act_flat = {AF_W{1'b0}};
        act_flat[0*NUM_ROW +: NUM_ROW] = 8'h00;
        act_flat[1*NUM_ROW +: NUM_ROW] = 8'hFF;
        act_flat[2*NUM_ROW +: NUM_ROW] = 8'h00;
        act_flat[3*NUM_ROW +: NUM_ROW] = 8'hFF;
        compute_expected(5'd4, act_flat);
        drive_mac(5'd4, act_flat);
        check_result(10);

        // -----------------------------------------------------------------
        // TC10 : Cyclic-identity weights (w_flat=32'h8421_8421), mixed signs
        //   col0 active rows:{0,4}; col1:{1,5}; col2:{2,6}; col3:{3,7}
        //   Rows 0-3=+6=0110; rows 4-7=-2=1110  (4-bit 2's complement)
        //   c0(sign,shift=3): 8'hF0 -> ps=1 per col -> -1x8=-8
        //   c1(shift=2):      8'hFF -> ps=2 per col -> +2x4=+8
        //   c2(shift=1):      8'hFF -> ps=2 per col -> +2x2=+4
        //   c3(shift=0):      8'h00 -> ps=0 -> 0
        //   psum = -8+8+4 = +4;  Verify: (+6)+(-2) = +4 per column
        // -----------------------------------------------------------------
        $display("--- TC10: Cyclic-identity weights, mixed signs, expected=+4 ---");
        load_weights(32'h8421_8421);
        act_flat = {AF_W{1'b0}};
        act_flat[0*NUM_ROW +: NUM_ROW] = 8'hF0;
        act_flat[1*NUM_ROW +: NUM_ROW] = 8'hFF;
        act_flat[2*NUM_ROW +: NUM_ROW] = 8'hFF;
        act_flat[3*NUM_ROW +: NUM_ROW] = 8'h00;
        compute_expected(5'd4, act_flat);
        drive_mac(5'd4, act_flat);
        check_result(11);

        // -----------------------------------------------------------------
        // Final summary
        // -----------------------------------------------------------------
        $display("=====================================================");
        $display("  RESULTS: %0d PASSED  /  %0d FAILED  (total %0d)",
                 pass_cnt, fail_cnt, pass_cnt + fail_cnt);
        $display("=====================================================");
        if (fail_cnt == 0)
            $display("  >>> ALL TESTS PASSED <<<");
        else
            $display("  >>> SOME TESTS FAILED -- review output above <<<");
        $finish;
    end

endmodule
// =============================================================================
// End of tb_cim_macro.v
// =============================================================================
