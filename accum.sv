/***************************************************/
/* ECE 327: Digital Hardware Systems - Spring 2026 */
/* Lab 4                                           */
/* Accumulator Module                              */
/***************************************************/

module accum # (
    parameter DATAW = 32,
    parameter ACCUMW = 32
)(
    input  clk,
    input  rst,
    input  signed [DATAW-1:0] data,
    input  ivalid,
    input  first,
    input  last,
    output signed [ACCUMW-1:0] result,
    output ovalid
);

/******* Your code starts here *******/

// first: load data (start a new row); otherwise add to the running sum.
// ovalid pulses one cycle after the input marked last, with the final sum.
// r_acc has no reset: it is only observed when ovalid is high.

logic signed [ACCUMW-1:0] r_acc;
logic r_ovalid;

always_ff @(posedge clk) begin
    if (rst) begin
        r_ovalid <= 1'b0;
    end else begin
        r_ovalid <= ivalid && last;
    end
    if (ivalid) begin
        if (first) begin
            r_acc <= data;
        end else begin
            r_acc <= r_acc + data;
        end
    end
end

assign result = r_acc;
assign ovalid = r_ovalid;

/******* Your code ends here ********/

endmodule