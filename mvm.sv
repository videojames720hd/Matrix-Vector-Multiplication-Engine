/***************************************************/
/* ECE 327: Digital Hardware Systems - Spring 2026 */
/* Lab 4                                           */
/* Matrix Vector Multiplication (MVM) Module       */
/***************************************************/

module mvm # (
    parameter IWIDTH = 8,
    parameter OWIDTH = 32,
    parameter MEM_DATAW = IWIDTH * 8,
    parameter VEC_MEM_DEPTH = 256,
    parameter VEC_ADDRW = $clog2(VEC_MEM_DEPTH),
    parameter MAT_MEM_DEPTH = 512,
    parameter MAT_ADDRW = $clog2(MAT_MEM_DEPTH),
    parameter NUM_OLANES = 256
)(
    input clk,
    input rst,
    input [MEM_DATAW-1:0] i_vec_wdata,
    input [VEC_ADDRW-1:0] i_vec_waddr,
    input i_vec_wen,
    input [MEM_DATAW-1:0] i_mat_wdata,
    input [MAT_ADDRW-1:0] i_mat_waddr,
    input [NUM_OLANES-1:0] i_mat_wen,
    input i_start,
    input [VEC_ADDRW-1:0] i_vec_start_addr,
    input [VEC_ADDRW:0] i_vec_num_words,
    input [MAT_ADDRW-1:0] i_mat_start_addr,
    input [MAT_ADDRW:0] i_mat_num_rows_per_olane,
    output o_busy,
    output [OWIDTH*NUM_OLANES-1:0] o_result,
    output o_valid
);

/******* Your code starts here *******/

// Overview
// --------
// ctrl issues one (vec_raddr, mat_raddr) per cycle. All NUM_OLANES lanes read
// the same address from their own matrix memory and multiply it by the shared
// vector word (8 elements), then accumulate over the words of a row.
//
// Lanes are handled in pairs (even/odd) so that one DSP computes two 8x8
// products per cycle (see the "pair" generate block). Vector data and the
// matrix read address are fanned out through registered trees
// (1 -> NUM_L1 -> NUM_GROUPS -> NUM_PAIRS) to meet timing across the chip.
//
// Timing (cycle 0 = ctrl outputs a word; every stage below is one register):
//   vec : r_vec_raddr 1, r_vec_raddr2 2, BRAM 3-4, r_vec_l0 5, r_vec_l1 6,
//         r_vec_l2 7, r_vecp 8, r_b1 9, r_b2 10
//   mat : r_mat_raddr 1, r_mat_raddr1q 2, r_mat_raddr2 3, r_mat_raddr3 4,
//         mem 5-6, r_w*2 7, r_w*3 8, r_a1/r_d1 9, r_ad 10
//   dsp : r_m 11, r_p 12, r_pa/r_pb 13, r_qa/r_qb 14, r_sa/r_sb 15
//   ctl : *_pipe[12] 13, r_*_g 14, r_*_p 15  -> accum inputs at cycle 15
//   out : accum ovalid 16
// Both paths deliver data at cycle 8: the vec path spends its cycles on
// fanout registers after the memory, the mat path fans out the address
// before the memory. r_b2 delays b one cycle to match the pre-adder (r_ad).
// If you add or remove any stage, rebalance the control tap (PIPE_DEPTH-3).

localparam PIPE_DEPTH = 15;
localparam GROUP_SIZE = 16;
localparam NUM_GROUPS = (NUM_OLANES + GROUP_SIZE - 1) / GROUP_SIZE;
localparam NUM_L1 = (NUM_GROUPS + 3) / 4;
localparam NUM_PAIRS = NUM_OLANES / 2;
// Matrix memory primitive per pair: first BRAM_PAIRS use block RAM, the next
// URAM_PAIRS use UltraRAM, the rest use LUTRAM. Tuned to the device budget.
localparam BRAM_PAIRS = 71;
localparam URAM_PAIRS = 32;

logic [VEC_ADDRW-1:0] vec_raddr;
logic [MAT_ADDRW-1:0] mat_raddr;
logic ctrl_first;
logic ctrl_last;
logic ctrl_ovalid;
logic ctrl_busy;
logic [PIPE_DEPTH-1:0] first_pipe;
logic [PIPE_DEPTH-1:0] last_pipe;
logic [PIPE_DEPTH-1:0] valid_pipe;
logic signed [OWIDTH-1:0] accum_result [0:NUM_OLANES-1];
logic accum_ovalid [0:NUM_OLANES-1];

logic [VEC_ADDRW-1:0] r_vec_raddr;
logic [MAT_ADDRW-1:0] r_mat_raddr;
logic [VEC_ADDRW-1:0] r_vec_raddr2;
(* keep = "true" *) logic [MAT_ADDRW-1:0] r_mat_raddr1q [0:NUM_L1-1];
(* keep = "true" *) logic [MAT_ADDRW-1:0] r_mat_raddr2 [0:NUM_GROUPS-1];
logic [MEM_DATAW-1:0] vec_dout;
logic [MEM_DATAW-1:0] r_vec_l0;
(* keep = "true" *) logic [MEM_DATAW-1:0] r_vec_l1 [0:NUM_L1-1];
(* keep = "true" *) logic [MEM_DATAW-1:0] r_vec_l2 [0:NUM_GROUPS-1];
(* keep = "true" *) logic r_valid_g [0:NUM_GROUPS-1];
(* keep = "true" *) logic r_first_g [0:NUM_GROUPS-1];
(* keep = "true" *) logic r_last_g [0:NUM_GROUPS-1];

ctrl # (
    .VEC_ADDRW(VEC_ADDRW),
    .MAT_ADDRW(MAT_ADDRW)
) ctrl_inst (
    .clk(clk),
    .rst(rst),
    .start(i_start),
    .vec_start_addr(i_vec_start_addr),
    .vec_num_words(i_vec_num_words),
    .mat_start_addr(i_mat_start_addr),
    .mat_num_rows_per_olane(i_mat_num_rows_per_olane),
    .vec_raddr(vec_raddr),
    .mat_raddr(mat_raddr),
    .accum_first(ctrl_first),
    .accum_last(ctrl_last),
    .ovalid(ctrl_ovalid),
    .busy(ctrl_busy)
);

always_ff @(posedge clk) begin
    r_vec_raddr <= vec_raddr;
    r_mat_raddr <= mat_raddr;
    r_vec_raddr2 <= r_vec_raddr;
end

xpm_memory_sdpram # (
    .ADDR_WIDTH_A(VEC_ADDRW),
    .ADDR_WIDTH_B(VEC_ADDRW),
    .AUTO_SLEEP_TIME(0),
    .BYTE_WRITE_WIDTH_A(MEM_DATAW),
    .CASCADE_HEIGHT(0),
    .CLOCKING_MODE("common_clock"),
    .ECC_MODE("no_ecc"),
    .MEMORY_INIT_FILE("none"),
    .MEMORY_INIT_PARAM("0"),
    .MEMORY_OPTIMIZATION("true"),
    .MEMORY_PRIMITIVE("block"),
    .MEMORY_SIZE(VEC_MEM_DEPTH * MEM_DATAW),
    .MESSAGE_CONTROL(0),
    .READ_DATA_WIDTH_B(MEM_DATAW),
    .READ_LATENCY_B(2),
    .READ_RESET_VALUE_B("0"),
    .RST_MODE_A("SYNC"),
    .RST_MODE_B("SYNC"),
    .SIM_ASSERT_CHK(0),
    .USE_EMBEDDED_CONSTRAINT(0),
    .USE_MEM_INIT(0),
    .WAKEUP_TIME("disable_sleep"),
    .WRITE_DATA_WIDTH_A(MEM_DATAW),
    .WRITE_MODE_B("read_first")
) vec_mem_inst (
    .clka(clk),
    .clkb(clk),
    .ena(1'b1),
    .wea(i_vec_wen),
    .addra(i_vec_waddr),
    .dina(i_vec_wdata),
    .enb(1'b1),
    .rstb(1'b0),
    .regceb(1'b1),
    .addrb(r_vec_raddr2),
    .doutb(vec_dout),
    .sleep(1'b0),
    .injectsbiterra(1'b0),
    .injectdbiterra(1'b0),
    .sbiterrb(),
    .dbiterrb()
);

always_ff @(posedge clk) begin
    r_vec_l0 <= vec_dout;
end

always_ff @(posedge clk) begin
    if (rst) begin
        first_pipe <= '0;
        last_pipe <= '0;
        valid_pipe <= '0;
    end else begin
        first_pipe <= {first_pipe[PIPE_DEPTH-2:0], ctrl_first};
        last_pipe <= {last_pipe[PIPE_DEPTH-2:0], ctrl_last};
        valid_pipe <= {valid_pipe[PIPE_DEPTH-2:0], ctrl_ovalid};
    end
end

// Control shift registers: bit k holds the ctrl signal delayed k+1 cycles.
// Only bit PIPE_DEPTH-3 feeds the datapath; the upper bits exist so that
// o_busy stays high while words are still in flight.

// Fanout tree level 1: one copy per 4 groups
genvar j;
generate
for (j = 0; j < NUM_L1; j = j + 1) begin : vtree
    always_ff @(posedge clk) begin
        r_vec_l1[j] <= r_vec_l0;
        r_mat_raddr1q[j] <= r_mat_raddr;
    end
end
endgenerate

genvar q;
generate
for (q = 0; q < NUM_GROUPS; q = q + 1) begin : grp

    // Fanout tree level 2: one copy per group of GROUP_SIZE lanes

    always_ff @(posedge clk) begin
        r_mat_raddr2[q] <= r_mat_raddr1q[q/4];
        r_vec_l2[q] <= r_vec_l1[q/4];
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            r_valid_g[q] <= 1'b0;
            r_first_g[q] <= 1'b0;
            r_last_g[q] <= 1'b0;
        end else begin
            r_valid_g[q] <= valid_pipe[PIPE_DEPTH-3];
            r_first_g[q] <= first_pipe[PIPE_DEPTH-3];
            r_last_g[q] <= last_pipe[PIPE_DEPTH-3];
        end
    end

end
endgenerate

genvar p;
generate
for (p = 0; p < NUM_PAIRS; p = p + 1) begin : pair

    // One pair = output lanes 2p (even, "e", memory -> d) and 2p+1 (odd,
    // "o", memory -> a). Both share the vector element b.
    //
    // DSP packing (assumes IWIDTH = 8, DSP48E2 27x18 multiplier):
    //   ad = (a << 18) + d               27-bit pre-adder
    //   m  = ad * b = (a*b << 18) + d*b
    //   m[17:0]  (signed) = d*b           exact, since |d*b| <= 2^14
    //   m[37:18] (signed) = a*b - m[17]   the low half borrows 1 when d*b < 0
    // So the even lane sums the low halves, and the odd lane sums the high
    // halves plus r_corr = number of negative low products.
    // Adder widths grow by 1 bit per tree level (18->19->20->22, 20->21->22->24).

    localparam GRP = (2 * p) / GROUP_SIZE;

    logic [MEM_DATAW-1:0] mat_dout_e;
    logic [MEM_DATAW-1:0] mat_dout_o;
    (* keep = "true" *) logic [MAT_ADDRW-1:0] r_mat_raddr3;
    (* keep = "true" *) logic r_valid_p;
    (* keep = "true" *) logic r_first_p;
    (* keep = "true" *) logic r_last_p;
    (* shreg_extract = "no" *) logic [MEM_DATAW-1:0] r_we2;
    (* shreg_extract = "no" *) logic [MEM_DATAW-1:0] r_we3;
    (* shreg_extract = "no" *) logic [MEM_DATAW-1:0] r_wo2;
    (* shreg_extract = "no" *) logic [MEM_DATAW-1:0] r_wo3;
    (* keep = "true" *) logic [MEM_DATAW-1:0] r_vecp;

    logic signed [IWIDTH-1:0] r_b1 [0:7];
    logic signed [IWIDTH-1:0] r_a1 [0:7];
    logic signed [IWIDTH-1:0] r_d1 [0:7];
    logic signed [26:0] r_ad [0:7];
    logic signed [IWIDTH-1:0] r_b2 [0:7];
    (* use_dsp = "yes" *) logic signed [44:0] r_m [0:7];
    logic signed [44:0] r_p [0:7];
    logic [3:0] r_corr;
    logic [3:0] r_corr2;
    logic signed [18:0] r_pb [0:3];
    logic signed [20:0] r_pa [0:3];
    logic signed [19:0] r_qb [0:1];
    logic signed [21:0] r_qa [0:1];
    logic signed [21:0] r_sb;
    logic signed [23:0] r_sa;

    integer i;

    xpm_memory_sdpram # (
        .ADDR_WIDTH_A(MAT_ADDRW),
        .ADDR_WIDTH_B(MAT_ADDRW),
        .AUTO_SLEEP_TIME(0),
        .BYTE_WRITE_WIDTH_A(MEM_DATAW),
        .CASCADE_HEIGHT(0),
        .CLOCKING_MODE("common_clock"),
        .ECC_MODE("no_ecc"),
        .MEMORY_INIT_FILE("none"),
        .MEMORY_INIT_PARAM("0"),
        .MEMORY_OPTIMIZATION("true"),
        .MEMORY_PRIMITIVE(p < BRAM_PAIRS ? "block" : (p < BRAM_PAIRS + URAM_PAIRS ? "ultra" : "distributed")),
        .MEMORY_SIZE(MAT_MEM_DEPTH * MEM_DATAW),
        .MESSAGE_CONTROL(0),
        .READ_DATA_WIDTH_B(MEM_DATAW),
        .READ_LATENCY_B(2),
        .READ_RESET_VALUE_B("0"),
        .RST_MODE_A("SYNC"),
        .RST_MODE_B("SYNC"),
        .SIM_ASSERT_CHK(0),
        .USE_EMBEDDED_CONSTRAINT(0),
        .USE_MEM_INIT(0),
        .WAKEUP_TIME("disable_sleep"),
        .WRITE_DATA_WIDTH_A(MEM_DATAW),
        .WRITE_MODE_B("read_first")
    ) mat_mem_e_inst (
        .clka(clk),
        .clkb(clk),
        .ena(1'b1),
        .wea(i_mat_wen[2*p]),
        .addra(i_mat_waddr),
        .dina(i_mat_wdata),
        .enb(1'b1),
        .rstb(1'b0),
        .regceb(1'b1),
        .addrb(r_mat_raddr3),
        .doutb(mat_dout_e),
        .sleep(1'b0),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .sbiterrb(),
        .dbiterrb()
    );

    xpm_memory_sdpram # (
        .ADDR_WIDTH_A(MAT_ADDRW),
        .ADDR_WIDTH_B(MAT_ADDRW),
        .AUTO_SLEEP_TIME(0),
        .BYTE_WRITE_WIDTH_A(MEM_DATAW),
        .CASCADE_HEIGHT(0),
        .CLOCKING_MODE("common_clock"),
        .ECC_MODE("no_ecc"),
        .MEMORY_INIT_FILE("none"),
        .MEMORY_INIT_PARAM("0"),
        .MEMORY_OPTIMIZATION("true"),
        .MEMORY_PRIMITIVE(p < BRAM_PAIRS ? "block" : (p < BRAM_PAIRS + URAM_PAIRS ? "ultra" : "distributed")),
        .MEMORY_SIZE(MAT_MEM_DEPTH * MEM_DATAW),
        .MESSAGE_CONTROL(0),
        .READ_DATA_WIDTH_B(MEM_DATAW),
        .READ_LATENCY_B(2),
        .READ_RESET_VALUE_B("0"),
        .RST_MODE_A("SYNC"),
        .RST_MODE_B("SYNC"),
        .SIM_ASSERT_CHK(0),
        .USE_EMBEDDED_CONSTRAINT(0),
        .USE_MEM_INIT(0),
        .WAKEUP_TIME("disable_sleep"),
        .WRITE_DATA_WIDTH_A(MEM_DATAW),
        .WRITE_MODE_B("read_first")
    ) mat_mem_o_inst (
        .clka(clk),
        .clkb(clk),
        .ena(1'b1),
        .wea(i_mat_wen[2*p+1]),
        .addra(i_mat_waddr),
        .dina(i_mat_wdata),
        .enb(1'b1),
        .rstb(1'b0),
        .regceb(1'b1),
        .addrb(r_mat_raddr3),
        .doutb(mat_dout_o),
        .sleep(1'b0),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .sbiterrb(),
        .dbiterrb()
    );

    always_ff @(posedge clk) begin
        if (rst) begin
            r_valid_p <= 1'b0;
            r_first_p <= 1'b0;
            r_last_p <= 1'b0;
        end else begin
            r_valid_p <= r_valid_g[GRP];
            r_first_p <= r_first_g[GRP];
            r_last_p <= r_last_g[GRP];
        end
    end

    // Fanout level 3 (per pair) and two registers on the memory outputs
    always_ff @(posedge clk) begin
        r_mat_raddr3 <= r_mat_raddr2[GRP];
        r_we2 <= mat_dout_e;
        r_we3 <= r_we2;
        r_wo2 <= mat_dout_o;
        r_wo3 <= r_wo2;
        r_vecp <= r_vec_l2[GRP];
    end

    // DSP pipeline: A/B/D input regs -> pre-adder (AD) -> M -> P,
    // then a 3-level adder tree for each half
    always_ff @(posedge clk) begin
        for (i = 0; i < 8; i = i + 1) begin
            r_b1[i] <= r_vecp[i*IWIDTH +: IWIDTH];
            r_a1[i] <= r_wo3[i*IWIDTH +: IWIDTH];
            r_d1[i] <= r_we3[i*IWIDTH +: IWIDTH];
            r_ad[i] <= (27'(signed'(r_a1[i])) <<< 18) + 27'(signed'(r_d1[i]));
            r_b2[i] <= r_b1[i];
            r_m[i] <= r_ad[i] * r_b2[i];
            r_p[i] <= r_m[i];
        end
        for (i = 0; i < 4; i = i + 1) begin
            r_pb[i] <= signed'(r_p[2*i][17:0]) + signed'(r_p[2*i+1][17:0]);
            r_pa[i] <= signed'(r_p[2*i][37:18]) + signed'(r_p[2*i+1][37:18]);
        end
        r_corr <= {3'b0, r_p[0][17]} + {3'b0, r_p[1][17]} + {3'b0, r_p[2][17]} + {3'b0, r_p[3][17]}
                + {3'b0, r_p[4][17]} + {3'b0, r_p[5][17]} + {3'b0, r_p[6][17]} + {3'b0, r_p[7][17]};
        r_qb[0] <= r_pb[0] + r_pb[1];
        r_qb[1] <= r_pb[2] + r_pb[3];
        r_qa[0] <= r_pa[0] + r_pa[1];
        r_qa[1] <= r_pa[2] + r_pa[3];
        r_corr2 <= r_corr;
        r_sb <= r_qb[0] + r_qb[1];
        r_sa <= r_qa[0] + r_qa[1] + signed'({1'b0, r_corr2});
    end

    accum # (
        .DATAW(OWIDTH),
        .ACCUMW(OWIDTH)
    ) accum_e_inst (
        .clk(clk),
        .rst(rst),
        .data(OWIDTH'(r_sb)),
        .ivalid(r_valid_p),
        .first(r_first_p),
        .last(r_last_p),
        .result(accum_result[2*p]),
        .ovalid(accum_ovalid[2*p])
    );

    accum # (
        .DATAW(OWIDTH),
        .ACCUMW(OWIDTH)
    ) accum_o_inst (
        .clk(clk),
        .rst(rst),
        .data(OWIDTH'(r_sa)),
        .ivalid(r_valid_p),
        .first(r_first_p),
        .last(r_last_p),
        .result(accum_result[2*p+1]),
        .ovalid(accum_ovalid[2*p+1])
    );

    assign o_result[(2*p)*OWIDTH +: OWIDTH] = accum_result[2*p];
    assign o_result[(2*p+1)*OWIDTH +: OWIDTH] = accum_result[2*p+1];

end
endgenerate

// All lanes run in lockstep, so lane 0 is representative
assign o_valid = accum_ovalid[0];
assign o_busy = ctrl_busy || (|valid_pipe) || accum_ovalid[0];

/******* Your code ends here ********/

endmodule