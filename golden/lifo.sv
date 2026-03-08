`timescale 1ns / 1ps

module sync_lifo_parity #(
    parameter int DATA_WIDTH = 8,
    parameter int DEPTH      = 16,
    parameter int BURST_SIZE = 4   // Number of words required for a single packet
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
    output logic                  burst_ready // High when enough space exists for a full packet
);

    localparam int SP_WIDTH = $clog2(DEPTH) + 1;
    
    logic [SP_WIDTH-1:0] sp;
    
    // Memory width expanded by 1 to store the parity bit at the MSB
    logic [DATA_WIDTH:0] mem [0:DEPTH-1];

    assign empty = (sp == '0);
    assign full  = (sp == DEPTH[SP_WIDTH-1:0]);

    // Calculate if there is enough space for a burst.
    // We compare against a constant computed at synthesis time to save area.
    assign burst_ready = (sp <= (DEPTH[SP_WIDTH-1:0] - BURST_SIZE[SP_WIDTH-1:0]));

    logic push_valid;
    logic pop_valid;
    
    assign push_valid = push && !full;
    assign pop_valid  = pop && !empty;

    // Odd parity calculation for incoming data (XNOR reduction)
    logic in_parity;
    logic [DATA_WIDTH:0] write_word;
    
    assign in_parity  = ~^data_in; 
    assign write_word = {in_parity, data_in};

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