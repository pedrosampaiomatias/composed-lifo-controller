`timescale 1ns / 1ps

module sync_lifo_parity #(
    parameter int DATA_WIDTH = 8,
    parameter int DEPTH      = 16,
    parameter int BURST_SIZE = 4   // Credit grant quantum
)(
    input  logic                  clk,
    input  logic                  rst_n,
    input  logic                  push,
    input  logic                  pop,
    input  logic [DATA_WIDTH-1:0] data_in,
    output logic [DATA_WIDTH-1:0] data_out,
    output logic                  empty,
    output logic                  full,
    output logic                  parity_err,
    // Credit-based flow control
    output logic                  credit_valid,   // High for 1 cycle when credits are issued
    output logic [$clog2(DEPTH):0] credit_amount  // Number of credits being issued
);

    localparam int SP_WIDTH = $clog2(DEPTH) + 1;
    
    logic [SP_WIDTH-1:0] sp;
    
    // Memory width expanded by 1 to store the parity bit at the MSB
    logic [DATA_WIDTH:0] mem [0:DEPTH-1];

    assign empty = (sp == '0);
    assign full  = (sp == DEPTH[SP_WIDTH-1:0]);

    logic push_valid;
    logic pop_valid;
    
    assign push_valid = push && !full;
    assign pop_valid  = pop && !empty;

    // Odd parity calculation for incoming data (XNOR reduction)
    logic in_parity;
    logic [DATA_WIDTH:0] write_word;
    
    assign in_parity  = ~^data_in; 
    assign write_word = {in_parity, data_in};

    // ---------------------------------------------------------------
    // Credit-based flow control
    // ---------------------------------------------------------------
    // `credits_outstanding` tracks how many of the previously granted
    // credits have yet to be consumed by pushes.  A new batch of
    // BURST_SIZE credits is issued only when:
    //   1) credits_outstanding == 0 (previous batch fully consumed), AND
    //   2) free space >= BURST_SIZE
    //
    // credit_valid / credit_amount are combinational outputs so the
    // producer can latch the new credits on the same rising edge.
    // credits_outstanding is updated sequentially.
    // ---------------------------------------------------------------
    logic [SP_WIDTH-1:0] credits_outstanding;
    logic [SP_WIDTH-1:0] free_slots;

    assign free_slots = DEPTH[SP_WIDTH-1:0] - sp;

    // Combinational credit grant decision
    assign credit_valid  = (credits_outstanding == '0) &&
                           (free_slots >= BURST_SIZE[SP_WIDTH-1:0]);
    assign credit_amount = credit_valid ? BURST_SIZE[$clog2(DEPTH):0] : '0;

    // Sequential credit tracking
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            credits_outstanding <= '0;
        end else begin
            if (credit_valid && push_valid) begin
                // New batch issued same cycle as a push: load batch minus 1
                credits_outstanding <= BURST_SIZE[SP_WIDTH-1:0] - 1'b1;
            end else if (credit_valid) begin
                // New batch issued, no push this cycle
                credits_outstanding <= BURST_SIZE[SP_WIDTH-1:0];
            end else if (push_valid && credits_outstanding != '0) begin
                // Consume one credit
                credits_outstanding <= credits_outstanding - 1'b1;
            end
        end
    end

    // Stack Pointer and Memory Write Logic
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sp <= '0;
        end else begin
            unique case ({push_valid, pop_valid})
                2'b10: begin // Push only
                    mem[sp] <= write_word;
                    sp <= sp + 1'b1;
                end
                2'b01: begin // Pop only
                    sp <= sp - 1'b1;
                end
                2'b11: begin // Simultaneous Push and Pop
                    // Replace the top of the stack with the new word+parity
                    mem[sp-1] <= write_word; 
                end
                default: ; 
            endcase
        end
    end

    // Synchronous Read Logic and Parity Checking
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_out   <= '0;
            parity_err <= 1'b0;
        end else if (pop_valid) begin
            if (push_valid) begin
                // Write-through: Output new data, no memory corruption possible yet
                data_out   <= data_in;
                parity_err <= 1'b0; 
            end else begin
                // Normal pop: Extract data and check against stored parity
                data_out   <= mem[sp-1][DATA_WIDTH-1:0];
                
                // Compare stored parity (MSB) with calculated odd parity of the read data
                parity_err <= (mem[sp-1][DATA_WIDTH] != ~^mem[sp-1][DATA_WIDTH-1:0]);
            end
        end
    end

endmodule