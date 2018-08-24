/*
 * Copyright (c) 2018 Fabian Schmid
 *
 * All rights reserved.
 *
 * Limiter block that limits the signal to be between min_val_i and max_val_i.
 *
 * Based on code by David Leibrandt (National Institute of Standards and
 * Technology)
 */
`timescale 1ns / 1ps

module red_pitaya_limit_block(
    input  wire                 clk_i,
    input  wire signed [14-1:0] min_val_i, // minimum allowed value
    input  wire signed [14-1:0] max_val_i, // maximum allowed value
    input  wire signed [14-1:0] signal_i,
    output reg         [1:0]    railed_o, // 0th bit: lower rail, 1st bit:
                                          // upper rail
    output reg signed  [14-1:0] signal_o
);

always @(posedge clk_i) begin
    if (signal_i >= max_val_i) begin
        signal_o <= max_val_i;
        railed_o <= 2'b10;
    end else if (signal_i <= min_val_i) begin
        signal_o <= min_val_i;
        railed_o <= 2'b01;
    end else begin
        signal_o <= signal_i;
        railed_o <= 2'b00;
    end
end

endmodule
