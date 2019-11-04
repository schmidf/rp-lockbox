/**
 * Copyright (c) 2018, Fabian Schmid
 *
 * All rights reserved.
 *
 * $Id: red_pitaya_pid_block.v 961 2014-01-21 11:40:39Z matej.oblak $
 *
 * @brief Red Pitaya PID controller.
 *
 * @Author Matej Oblak
 *
 * (c) Red Pitaya  http://www.redpitaya.com
 *
 * This part of code is written in Verilog hardware description language (HDL).
 * Please visit http://en.wikipedia.org/wiki/Verilog
 * for more details on the language used herein.
 */



/**
 * GENERAL DESCRIPTION:
 *
 * Proportional-integral-derivative (PID) controller.
 *
 *
 *        /---\         /---\      /-----------\
 *   IN --| - |----+--> | P | ---> | SUM & SAT | ---> OUT
 *        \---/    |    \---/      \-----------/
 *          ^      |                   ^  ^
 *          |      |    /---\          |  |
 *   set ----      +--> | I | ---------   |
 *   point         |    \---/             |
 *                 |                      |
 *                 |    /---\             |
 *                 ---> | D | ------------
 *                      \---/
 *
 *
 * Proportional-integral-derivative (PID) controller is made from three parts. 
 *
 * Error which is difference between set point and input signal is driven into
 * propotional, integral and derivative part. Each calculates its own value which
 * is then summed and saturated before given to output.
 *
 * Integral part has also separate input to reset integrator value to 0.
 * 
 */

`timescale 1ns / 1ps
module red_pitaya_pid_block #(
   parameter     PSR     = 12                   ,  // p gain = Kp >> PSR
   parameter     ISR     = 28                   ,  // i gain = Ki >> ISR
   parameter     DSR     = 10                   ,
   parameter     KP_BITS = 24                   ,
   parameter     KI_BITS = 24                   
)
(
   // data
   input                        clk_i           ,  // clock
   input                        rstn_i          ,  // reset - active low
   input         [    1: 0]     railed_i        ,  // output railed
   input                        hold_i          ,  // hold PID state
   input signed  [ 14-1: 0]     dat_i           ,  // input data
   output signed [ 14-1: 0]     dat_o           ,  // output data

   // settings
   input signed [ 14-1: 0]      set_sp_i        ,  // set point
   input        [ KP_BITS-1: 0] set_kp_i        ,  // Kp
   input        [ KI_BITS-1: 0] set_ki_i        ,  // Ki
   input        [ 14-1: 0]      set_kd_i        ,  // Kd
   input                        inverted_i      ,  // feedback sign
   input                        int_rst_i       ,  // integrator reset
   input                        int_ctr_rst_i   ,  // integrator reset to center of the output range
   input signed [ 14-1: 0]      int_ctr_val_i      // center value of the output range
);

//---------------------------------------------------------------------------------
//  Set point error calculation
reg signed [ 15-1: 0] error        ;

always @(posedge clk_i) begin
   if (rstn_i == 1'b0) begin
      error <= 15'h0 ;
   end
   else begin
       if (inverted_i == 1'b0)
          error <= dat_i - set_sp_i;
      else
          error <= -(dat_i - set_sp_i);
   end
end

//---------------------------------------------------------------------------------
//  Proportional part
reg signed   [KP_BITS+1+15-PSR-1: 0] kp_reg   ;
wire signed  [KP_BITS+1+15-1: 0]     kp_mult  ;
wire signed  [KP_BITS+1-1: 0]        kp_signed;  // Required to make signed arithmetic work
assign kp_signed = set_kp_i                   ;

always @(posedge clk_i) begin
   if (rstn_i == 1'b0) begin
      kp_reg  <= {KP_BITS+1+15-PSR{1'b0}};
   end
   else if (hold_i)
      kp_reg <= kp_reg;
   else begin
      kp_reg <= kp_mult[KP_BITS+1+15-1:PSR] ;
   end
end

assign kp_mult = error * kp_signed;

//---------------------------------------------------------------------------------
//  Integrator
reg signed  [KI_BITS+1+15-1: 0] ki_mult  ;
wire signed [15+ISR+1-1: 0]     int_sum  ;
reg signed  [15+ISR-1: 0]       int_reg  ;
wire signed [15-1: 0]           int_shr  ;  // Twice the DAC range (14 bit) should be enough
wire signed [KI_BITS+1-1: 0]    ki_signed;  // Required to make signed arithmetic work
assign ki_signed = set_ki_i              ;

always @(posedge clk_i) begin
   if (rstn_i == 1'b0) begin
      ki_mult  <= {KI_BITS+1+15{1'b0}};
      int_reg  <= {15+ISR{1'b0}};
   end
   else begin
      ki_mult <= error * ki_signed;

      if (int_rst_i)
         int_reg <= {15+ISR{1'b0}}; // reset
      else if (int_ctr_rst_i)
         int_reg <= {int_ctr_val_i[13], int_ctr_val_i, {ISR{1'b0}}}; // reset to center of output range
      else if (int_sum[15+ISR:15+ISR-1] == 2'b01) // positive saturation
         int_reg <= {1'b0, {15+ISR-1{1'b1}}}; // max positive
      else if (int_sum[15+ISR:15+ISR-1] == 2'b10) // negative saturation
         int_reg <= {1'b1, {15+ISR-1{1'b0}}}; // max negative
      else if ((railed_i[0] && (ki_mult < 0)) // anti-windup lower rail
            || (railed_i[1] && (ki_mult > 0)) // anti-windup upper rail
            || (hold_i)) 
         int_reg <= int_reg;
      else
         int_reg <= int_sum[15+ISR-1:0]; // use sum as it is
   end
end

assign int_sum = ki_mult + int_reg;
assign int_shr = int_reg[15+ISR-1:ISR];

//---------------------------------------------------------------------------------
//  Derivative

wire  [    29-1: 0] kd_mult       ;
reg   [29-DSR-1: 0] kd_reg        ;
reg   [29-DSR-1: 0] kd_reg_r      ;
reg   [29-DSR  : 0] kd_reg_s      ;


always @(posedge clk_i) begin
   if (rstn_i == 1'b0) begin
      kd_reg   <= {29-DSR{1'b0}};
      kd_reg_r <= {29-DSR{1'b0}};
      kd_reg_s <= {29-DSR+1{1'b0}};
   end
   else if (hold_i)
      kd_reg <= kd_reg;
   else begin
      kd_reg   <= kd_mult[29-1:DSR] ;
      kd_reg_r <= kd_reg;
      kd_reg_s <= $signed(kd_reg) - $signed(kd_reg_r);
   end
end

assign kd_mult = $signed(error) * $signed(set_kd_i) ;

//---------------------------------------------------------------------------------
//  Sum together - saturate output
wire signed  [   33-1: 0] pid_sum     ; // biggest posible bit-width
reg signed   [   14-1: 0] pid_out     ;

always @(posedge clk_i) begin
   if (rstn_i == 1'b0) begin
      pid_out    <= 14'b0 ;
   end
   else begin
      if ({pid_sum[33-1],|pid_sum[32-2:13]} == 2'b01) //positive overflow
         pid_out <= 14'h1FFF ;
      else if ({pid_sum[33-1],&pid_sum[33-2:13]} == 2'b10) //negative overflow
         pid_out <= 14'h2000 ;
      else
         pid_out <= pid_sum[14-1:0] ;
   end
end

assign pid_sum = kp_reg + $signed(int_shr) + $signed(kd_reg_s) ;

assign dat_o = pid_out ;

endmodule
