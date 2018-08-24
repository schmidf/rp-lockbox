/*
 * Copyright (c) 2018, Fabian Schmid
 *
 * All rights reserved.
 */

`timescale 1ns / 1ps
module testbench;

initial begin
    $dumpfile("limit_block_tb.vcd");
    $dumpvars(0, testbench);
    #19 limit_input <= 14'd0;
    #19 limit_input <= 14'd5000;
    #19 limit_input <= 14'd0;
    #19 limit_input <= -14'd5000;
    #19 limit_input <= -14'd5000;
    #19 min <= 14'd1000;
    #19 limit_input <= 14'd6000;
    #19 max <= 14'd2000;
    #20 $finish;
end

/* Make a regular pulsing clock. */
reg clk = 0;
always #4 clk = !clk;

wire [1:0] railed;

reg signed [13:0] limit_input = 14'd0;
wire signed [13:0] limit_output;

reg signed [13:0] min = -14'd4000;
reg signed [13:0] max = 14'd4000;

red_pitaya_limit_block limit(
    .clk_i(clk),
    .min_val_i(min),
    .max_val_i(max),
    .signal_i(limit_input),
    .railed_o(railed),
    .signal_o(limit_output)
);

endmodule // testbench
