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

// Walks vec_num_words vector words for each of mat_num_rows_per_olane rows.
// Every cycle in COMPUTE issues one (vec_raddr, mat_raddr) pair:
//   - mat_raddr increments every cycle (matrix rows are stored back to back)
//   - vec_raddr increments within a row, then rewinds to vec_start_addr
// accum_first / accum_last mark the first/last word of each row.
// All outputs are registered.

localparam IDLE    = 1'b0;
localparam COMPUTE = 1'b1;

logic state;

// Job parameters, captured while IDLE. Counts are stored as (n - 2) so the
// "last" flags can be computed one cycle early from registered compares.
logic [VEC_ADDRW-1:0] r_vec_start;
logic [VEC_SIZEW-1:0] r_words_m2;
logic [MAT_SIZEW-1:0] r_rows_m2;
logic                 r_words_is1;

// Position within the job
logic [VEC_SIZEW-1:0] word_idx;
logic [MAT_SIZEW-1:0] row_idx;
logic                 r_row_last;

// Output registers
logic [VEC_ADDRW-1:0] r_vec_raddr;
logic [MAT_ADDRW-1:0] r_mat_raddr;
logic                 r_first;
logic                 r_last;

always_ff @(posedge clk) begin
    if (rst) begin
        state   <= IDLE;
        r_first <= 1'b0;
        r_last  <= 1'b0;
    end else if (state == IDLE) begin
        r_vec_start <= vec_start_addr;
        r_words_m2  <= vec_num_words - 'd2;
        r_rows_m2   <= mat_num_rows_per_olane - 'd2;
        r_words_is1 <= (vec_num_words == 'd1);
        if (start) begin
            state       <= COMPUTE;
            r_vec_raddr <= vec_start_addr;
            r_mat_raddr <= mat_start_addr;
            word_idx    <= 'd0;
            row_idx     <= 'd0;
            r_first     <= 1'b1;
            r_last      <= (vec_num_words == 'd1);
            r_row_last  <= (mat_num_rows_per_olane == 'd1);
        end
    end else begin // COMPUTE
        r_mat_raddr <= r_mat_raddr + 'd1;
        if (r_last) begin
            if (r_row_last) begin
                // Just issued the last word of the last row: done
                state   <= IDLE;
                r_first <= 1'b0;
                r_last  <= 1'b0;
            end else begin
                // Next row: rewind the vector
                row_idx     <= row_idx + 'd1;
                word_idx    <= 'd0;
                r_vec_raddr <= r_vec_start;
                r_first     <= 1'b1;
                r_last      <= r_words_is1;
                r_row_last  <= (row_idx == r_rows_m2);
            end
        end else begin
            // Next word in the same row
            word_idx    <= word_idx + 'd1;
            r_vec_raddr <= r_vec_raddr + 'd1;
            r_first     <= 1'b0;
            r_last      <= (word_idx == r_words_m2);
        end
    end
end

assign vec_raddr   = r_vec_raddr;
assign mat_raddr   = r_mat_raddr;
assign accum_first = r_first;
assign accum_last  = r_last;
// ovalid and busy were separate registers that always equaled
// (state == COMPUTE); state is itself a register, so this is identical.
assign ovalid      = (state == COMPUTE);
assign busy        = (state == COMPUTE);

/******* Your code ends here ********/

endmodule