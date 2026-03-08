`timescale 1ns / 1ps

module top_composed_lifo (
    input  logic                  clk,
    input  logic                  rst_n,
    
    // Exposing LIFO status for the Cocotb testbench verification
    output logic [7:0]            lifo_data_out,
    output logic                  lifo_empty,
    output logic                  lifo_full,
    output logic                  lifo_burst_ready,
    output logic                  parity_err
);

    // Design constants — fixed to match the Producer FSM's packet structure
    localparam int DATA_WIDTH = 8;
    localparam int DEPTH      = 16;
    localparam int BURST_SIZE = 4;

    // Internal interconnects between Producer and LIFO
    logic                  int_push;
    logic                  int_pop;
    logic [DATA_WIDTH-1:0] int_data;

    // Instantiate the Producer FSM
    producer_fsm u_producer (
        .clk              (clk),
        .rst_n            (rst_n),
        .lifo_burst_ready (lifo_burst_ready),
        .lifo_empty       (lifo_empty),
        .push             (int_push),
        .pop              (int_pop),
        .data_out         (int_data)
    );

    // Instantiate the Packet-Aware Parity LIFO
    sync_lifo_parity #(
        .DATA_WIDTH(DATA_WIDTH),
        .DEPTH     (DEPTH),
        .BURST_SIZE(BURST_SIZE)
    ) u_lifo (
        .clk         (clk),
        .rst_n       (rst_n),
        .push        (int_push),
        .pop         (int_pop),
        .data_in     (int_data),
        .data_out    (lifo_data_out),
        .empty       (lifo_empty),
        .full        (lifo_full),
        .parity_err  (parity_err),
        .burst_ready (lifo_burst_ready)
    );

endmodule