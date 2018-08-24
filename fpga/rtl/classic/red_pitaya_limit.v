/**
 * Copyright (c) 2018 Fabian Schmid
 *
 * All rights reserved.
 *
 * Limit the output value to the configured range. The status of the limiters
 * is wired out for use in other modules.
 */

`timescale 1ns / 1ps

module red_pitaya_limit (
   // signals
   input                  clk_i,
   input                  rstn_i,
   input  signed [14-1:0] dat_a_i,
   input  signed [14-1:0] dat_b_i,
   output signed [14-1:0] dat_a_o,
   output signed [14-1:0] dat_b_o,
   output        [1:0]    dat_a_railed_o, // 0th bit: lower rail, 1st bit:
   output        [1:0]    dat_b_railed_o, // upper rail

   // system bus
   input      [32-1:0] sys_addr,
   input      [32-1:0] sys_wdata,
   input               sys_wen,
   input               sys_ren,
   output reg [32-1:0] sys_rdata,
   output reg          sys_err,
   output reg          sys_ack
);

// Configuration registers
reg signed [14-1:0] min_val_a;
reg signed [14-1:0] max_val_a;
reg signed [14-1:0] min_val_b;
reg signed [14-1:0] max_val_b;

// Limiters
red_pitaya_limit_block limit_a (
    .clk_i (clk_i), // clock
    // configuration
    .min_val_i (min_val_a),
    .max_val_i (max_val_a),
    // data
    .signal_i (dat_a_i),
    .railed_o (dat_a_railed_o),
    .signal_o (dat_a_o)
);

red_pitaya_limit_block limit_b (
    .clk_i (clk_i), // clock
    // configuration
    .min_val_i (min_val_b),
    .max_val_i (max_val_b),
    // data
    .signal_i (dat_b_i),
    .railed_o (dat_b_railed_o),
    .signal_o (dat_b_o)
);

// System bus connection
always @(posedge clk_i) begin
    if (rstn_i == 1'b0) begin
        min_val_a <= 14'h2000; // smallest 14 bit signed integer
        max_val_a <= 14'h1fff; // largest 14 bit signed integer
        min_val_b <= 14'h2000;
        max_val_b <= 14'h1fff;
    end
    else begin
        if (sys_wen) begin
            if (sys_addr[19:0]==16'h0) min_val_a <= sys_wdata[14-1:0];
            if (sys_addr[19:0]==16'h4) max_val_a <= sys_wdata[14-1:0];
            if (sys_addr[19:0]==16'h8) min_val_b <= sys_wdata[14-1:0];
            if (sys_addr[19:0]==16'hC) max_val_b <= sys_wdata[14-1:0];
        end
    end
end

wire sys_en;
assign sys_en = sys_wen | sys_ren;

always @(posedge clk_i)
if (rstn_i == 1'b0) begin
    sys_err <= 1'b0 ;
    sys_ack <= 1'b0 ;
end else begin
    sys_err <= 1'b0 ;

    casez (sys_addr[19:0])
        20'h0: begin sys_ack <= sys_en; sys_rdata <= {{32-14{1'b0}}, min_val_a}; end
        20'h4: begin sys_ack <= sys_en; sys_rdata <= {{32-14{1'b0}}, max_val_a}; end
        20'h8: begin sys_ack <= sys_en; sys_rdata <= {{32-14{1'b0}}, min_val_b}; end
        20'hC: begin sys_ack <= sys_en; sys_rdata <= {{32-14{1'b0}}, max_val_b}; end
        default: begin sys_ack <= sys_en; sys_rdata <= 32'h0; end
    endcase
end

endmodule
