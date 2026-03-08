`timescale 1ns / 1ps

module producer_fsm (
    input  logic       clk,
    input  logic       rst_n,
    input  logic       lifo_burst_ready,
    input  logic       lifo_empty,
    output logic       push,
    output logic       pop,
    output logic [7:0] data_out
);

    // Strict SystemVerilog typing for the state machine
    typedef enum logic [2:0] {
        ST_EVAL   = 3'b000, // Evaluates Phase (Fill vs Drain)
        ST_HDR    = 3'b001, // Pushes 0xAA
        ST_PHIGH  = 3'b010, // Pushes Counter MSB
        ST_PLOW   = 3'b011, // Pushes Counter LSB
        ST_FTR    = 3'b100, // Pushes 0x55
        ST_DRAIN  = 3'b101  // Drains LIFO until empty
    } state_t;

    state_t state, next_state;
    
    logic [15:0] pkt_cnt;
    logic        inc_cnt;

    // Sequential FSM and Counter Update
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= ST_EVAL;
            pkt_cnt <= 16'd0;
        end else begin
            state <= next_state;
            
            // Counter strictly increments only when flagged by the Footer state
            if (inc_cnt) begin
                pkt_cnt <= pkt_cnt + 1'b1;
            end
        end
    end

    // Combinational Next State and Output Decoding
    // BUG: Pushes LSB before MSB (wrong byte order) — tests expect MSB then LSB
    always_comb begin
        // Default assignments to prevent inferred latches
        next_state = state;
        push       = 1'b0;
        pop        = 1'b0;
        data_out   = 8'h00;
        inc_cnt    = 1'b0;

        unique case (state)
            ST_EVAL: begin
                // Phase decision point. Safe from combinatorial LIFO delays.
                if (lifo_burst_ready) begin
                    next_state = ST_HDR;
                end else begin
                    next_state = ST_DRAIN;
                end
            end

            ST_HDR: begin
                push       = 1'b1;
                data_out   = 8'hAA;
                next_state = ST_PHIGH;
            end

            ST_PHIGH: begin
                // BUG: Pushing LSB here instead of MSB
                push       = 1'b1;
                data_out   = pkt_cnt[7:0];
                next_state = ST_PLOW;
            end

            ST_PLOW: begin
                // BUG: Pushing MSB here instead of LSB
                push       = 1'b1;
                data_out   = pkt_cnt[15:8];
                next_state = ST_FTR;
            end

            ST_FTR: begin
                push       = 1'b1;
                data_out   = 8'h55;
                inc_cnt    = 1'b1;        // Flag counter to increment on the next clock edge
                next_state = ST_EVAL;     // Return to evaluation phase
            end

            ST_DRAIN: begin
                if (!lifo_empty) begin
                    pop = 1'b1;
                    // Stay in ST_DRAIN while popping
                end else begin
                    // Immediately transition back to FILL_PHASE evaluation
                    next_state = ST_EVAL; 
                end
            end
            
            default: next_state = ST_EVAL;
        endcase
    end

endmodule
