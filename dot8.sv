/***************************************************/
/* ECE 327: Digital Hardware Systems - Spring 2026 */
/* Lab 4                                           */
/* 8-Lane Dot Product Module                       */
/***************************************************/

module dot8 # (
    parameter IWIDTH = 8,
    parameter OWIDTH = 32
)(
    input clk,
    input rst,
    input signed [8*IWIDTH-1:0] vec0,
    input signed [8*IWIDTH-1:0] vec1,
    input ivalid,
    output signed [OWIDTH-1:0] result,
    output ovalid
);

/******* Your code starts here *******/

localparam VALID_DEPTH = 7;

logic signed [IWIDTH-1:0] r_vec0 [0:7];
logic signed [IWIDTH-1:0] r_vec1 [0:7];
logic signed [IWIDTH-1:0] rr_vec0 [0:7];
logic signed [IWIDTH-1:0] rr_vec1 [0:7];
(* use_dsp = "yes" *) logic signed [2*IWIDTH-1:0] r_mult [0:7];
logic signed [2*IWIDTH-1:0] rr_mult [0:7];
logic signed [2*IWIDTH:0] r_sum1 [0:3];
logic signed [2*IWIDTH+1:0] r_sum2 [0:1];
logic signed [2*IWIDTH+2:0] r_sum3;
logic [VALID_DEPTH-1:0] r_valid;

integer i;

always_ff @(posedge clk) begin
    for (i = 0; i < 8; i = i + 1) begin
        r_vec0[i] <= vec0[i*IWIDTH +: IWIDTH];
        r_vec1[i] <= vec1[i*IWIDTH +: IWIDTH];
        rr_vec0[i] <= r_vec0[i];
        rr_vec1[i] <= r_vec1[i];
        r_mult[i] <= rr_vec0[i] * rr_vec1[i];
        rr_mult[i] <= r_mult[i];
    end
    for (i = 0; i < 4; i = i + 1) begin
        r_sum1[i] <= rr_mult[2*i] + rr_mult[2*i+1];
    end
    for (i = 0; i < 2; i = i + 1) begin
        r_sum2[i] <= r_sum1[2*i] + r_sum1[2*i+1];
    end
    r_sum3 <= r_sum2[0] + r_sum2[1];
end

always_ff @(posedge clk) begin
    if (rst) begin
        r_valid <= '0;
    end else begin
        r_valid <= {r_valid[VALID_DEPTH-2:0], ivalid};
    end
end

assign result = r_sum3;
assign ovalid = r_valid[VALID_DEPTH-1];

/******* Your code ends here ********/

endmodule