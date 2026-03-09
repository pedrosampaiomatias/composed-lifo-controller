`timescale 1ns / 1ps

module producer_fsm (
    input  logic       clk,
    input  logic       rst_n,
    // Credit-based flow control from LIFO
    input  logic       credit_valid,   // Pulses high when credits are issued
    input  logic [4:0] credit_amount,  // Number of credits being issued
    input  logic       lifo_empty,
    output logic       push,
    output logic       pop,
    output logic [7:0] data_out
);

    // FSM state encoding
    typedef enum logic [2:0] {
        ST_EVAL   = 3'b000,  // Evaluate phase (Fill vs Drain)
        ST_HDR    = 3'b001,  // Push header 0xAA
        ST_PHIGH  = 3'b010,  // Push counter MSB
        ST_PLOW   = 3'b011,  // Push counter LSB
        ST_FTR    = 3'b100,  // Push CRC-8 footer
        ST_DRAIN  = 3'b101   // Drain LIFO until empty
    } state_t;

    state_t state, next_state;

    // 16-bit packet counter — wraps at 1000
    localparam logic [15:0] PKT_CNT_MAX = 16'd999;

    logic [15:0] pkt_cnt;
    logic        inc_cnt;

    // Credit balance — tracks how many pushes the producer is allowed
    logic [4:0] credit_balance;

    // Effective balance includes same-cycle credit grants (combinational)
    logic [4:0] effective_balance;
    assign effective_balance = credit_valid ? (credit_balance + credit_amount) : credit_balance;

    // CRC-8/MAXIM: polynomial 0x31 (x^8 + x^5 + x^4 + 1), init 0x00
    logic [7:0] crc_reg;
    logic [7:0] crc_next;
    logic       crc_feed;      // High when feeding a byte into CRC
    logic [7:0] crc_data_in;   // Byte being fed into CRC

    // ---------------------------------------------------------------
    // CRC-8 byte-at-a-time computation (bit-serial within the byte)
    // Polynomial: 0x31 = 8'b0011_0001
    // ---------------------------------------------------------------
    function automatic logic [7:0] crc8_byte(
        input logic [7:0] crc_in,
        input logic [7:0] data
    );
        logic [7:0] c;
        c = crc_in;
        for (int i = 7; i >= 0; i--) begin
            if (c[7] ^ data[i]) begin
                c = {c[6:0], 1'b0} ^ 8'h31;
            end else begin
                c = {c[6:0], 1'b0};
            end
        end
        return c;
    endfunction

    assign crc_next = crc8_byte(crc_reg, crc_data_in);

    // ---------------------------------------------------------------
    // Sequential: FSM state, packet counter, credit balance, CRC
    // ---------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= ST_EVAL;
            pkt_cnt        <= 16'd0;
            credit_balance <= 5'd0;
            crc_reg        <= 8'h00;
        end else begin
            state <= next_state;

            // Credit accumulation
            if (credit_valid && push) begin
                // Simultaneous credit grant and consumption
                credit_balance <= credit_balance + credit_amount - 5'd1;
            end else if (credit_valid) begin
                credit_balance <= credit_balance + credit_amount;
            end else if (push) begin
                credit_balance <= credit_balance - 5'd1;
            end

            // Packet counter with wrap at 1000
            if (inc_cnt) begin
                if (pkt_cnt == PKT_CNT_MAX)
                    pkt_cnt <= 16'd0;
                else
                    pkt_cnt <= pkt_cnt + 1'b1;
            end

            // CRC register update
            if (crc_feed) begin
                crc_reg <= crc_next;
            end else if (next_state == ST_HDR && state != ST_HDR) begin
                // Reset CRC at the start of a new packet
                crc_reg <= 8'h00;
            end
        end
    end

    // ---------------------------------------------------------------
    // Combinational: Next state and output decode
    // ---------------------------------------------------------------
    always_comb begin
        // Defaults
        next_state  = state;
        push        = 1'b0;
        pop         = 1'b0;
        data_out    = 8'h00;
        inc_cnt     = 1'b0;
        crc_feed    = 1'b0;
        crc_data_in = 8'h00;

        unique case (state)
            ST_EVAL: begin
                // Use effective_balance to account for same-cycle credit grants
                if (effective_balance >= 5'd4) begin
                    next_state = ST_HDR;
                end else begin
                    next_state = ST_DRAIN;
                end
            end

            ST_HDR: begin
                push        = 1'b1;
                data_out    = 8'hAA;
                crc_feed    = 1'b1;
                crc_data_in = 8'hAA;
                next_state  = ST_PHIGH;
            end

            ST_PHIGH: begin
                push        = 1'b1;
                data_out    = pkt_cnt[15:8];
                crc_feed    = 1'b1;
                crc_data_in = pkt_cnt[15:8];
                next_state  = ST_PLOW;
            end

            ST_PLOW: begin
                push        = 1'b1;
                data_out    = pkt_cnt[7:0];
                crc_feed    = 1'b1;
                crc_data_in = pkt_cnt[7:0];
                next_state  = ST_FTR;
            end

            ST_FTR: begin
                push       = 1'b1;
                data_out   = crc_reg;   // Emit accumulated CRC-8 of (header, payload_hi, payload_lo)
                inc_cnt    = 1'b1;
                next_state = ST_EVAL;
            end

            ST_DRAIN: begin
                if (!lifo_empty) begin
                    pop = 1'b1;
                end else begin
                    next_state = ST_EVAL;
                end
            end

            default: next_state = ST_EVAL;
        endcase
    end

endmodule