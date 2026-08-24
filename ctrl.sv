/***************************************************/
/* ECE 327: Digital Hardware Systems - Spring 2026 */
/* Lab 4                                           */
/* MVM Control FSM                                 */
/***************************************************/

module ctrl # (
    parameter VEC_ADDRW = 8,
    parameter MAT_ADDRW = 9,
    parameter VEC_SIZEW = VEC_ADDRW + 1,
    parameter MAT_SIZEW = MAT_ADDRW + 1
    
)(
    input  clk,
    input  rst,
    input  start,
    input  [VEC_ADDRW-1:0] vec_start_addr,
    input  [VEC_SIZEW-1:0] vec_num_words,
    input  [MAT_ADDRW-1:0] mat_start_addr,
    input  [MAT_SIZEW-1:0] mat_num_rows_per_olane,
    output [VEC_ADDRW-1:0] vec_raddr,
    output [MAT_ADDRW-1:0] mat_raddr,
    output accum_first,
    output accum_last,
    output ovalid,
    output busy
);

/******* Your code starts here *******/

localparam IDLE = 1'b0;
localparam COMPUTE = 1'b1;

logic state;
logic [VEC_ADDRW-1:0] r_vec_start;
logic [VEC_SIZEW-1:0] r_words_m2;
logic [MAT_SIZEW-1:0] r_rows_m2;
logic r_words_is1;
logic [VEC_SIZEW-1:0] word_idx;
logic [MAT_SIZEW-1:0] row_idx;
logic [VEC_ADDRW-1:0] r_vec_raddr;
logic [MAT_ADDRW-1:0] r_mat_raddr;
logic r_first;
logic r_last;
logic r_row_last;
logic r_ovalid;
logic r_busy;

always_ff @(posedge clk) begin
    if (rst) begin
        state <= IDLE;
        r_first <= 1'b0;
        r_last <= 1'b0;
        r_ovalid <= 1'b0;
        r_busy <= 1'b0;
    end else if (state == IDLE) begin
        r_vec_start <= vec_start_addr;
        r_words_m2 <= vec_num_words - 'd2;
        r_rows_m2 <= mat_num_rows_per_olane - 'd2;
        r_words_is1 <= (vec_num_words == 'd1);
        if (start) begin
            state <= COMPUTE;
            r_vec_raddr <= vec_start_addr;
            r_mat_raddr <= mat_start_addr;
            word_idx <= 'd0;
            row_idx <= 'd0;
            r_first <= 1'b1;
            r_last <= (vec_num_words == 'd1);
            r_row_last <= (mat_num_rows_per_olane == 'd1);
            r_ovalid <= 1'b1;
            r_busy <= 1'b1;
        end
    end else begin
        r_mat_raddr <= r_mat_raddr + 'd1;
        if (r_last) begin
            if (r_row_last) begin
                state <= IDLE;
                r_first <= 1'b0;
                r_last <= 1'b0;
                r_ovalid <= 1'b0;
                r_busy <= 1'b0;
            end else begin
                row_idx <= row_idx + 'd1;
                word_idx <= 'd0;
                r_vec_raddr <= r_vec_start;
                r_first <= 1'b1;
                r_last <= r_words_is1;
                r_row_last <= (row_idx == r_rows_m2);
            end
        end else begin
            word_idx <= word_idx + 'd1;
            r_vec_raddr <= r_vec_raddr + 'd1;
            r_first <= 1'b0;
            r_last <= (word_idx == r_words_m2);
        end
    end
end

assign vec_raddr = r_vec_raddr;
assign mat_raddr = r_mat_raddr;
assign accum_first = r_first;
assign accum_last = r_last;
assign ovalid = r_ovalid;
assign busy = r_busy;

/******* Your code ends here ********/

endmodule