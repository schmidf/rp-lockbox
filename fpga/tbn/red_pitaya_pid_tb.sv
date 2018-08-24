/**
 * Copyright (c) 2018, Fabian Schmid
 *
 * All rights reserved.
 *
 * $Id: pid_tb.v 961 2014-01-21 11:40:39Z matej.oblak $
 *
 * @brief Red Pitaya MIMO PID testbench.
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
 * Testbench for MIMO PID controller.
 *
 * Testing pid module using only one of the four sections. For system which it
 * controls simple model is used (acts as system of first order). 
 * 
 */

`timescale 1ns / 1ps

module pid_tb #(
  // time periods
  realtime  TP = 8.0ns  // 125MHz
);

////////////////////////////////////////////////////////////////////////////////
// signal generation
////////////////////////////////////////////////////////////////////////////////

logic              clk ;
logic              rstn;

// ADC clock
initial        clk = 1'b0;
always #(TP/2) clk = ~clk;

// ADC reset
initial begin
  rstn = 1'b0;
  repeat(4) @(posedge clk);
  rstn = 1'b1;
end

////////////////////////////////////////////////////////////////////////////////
// test sequence
// Simple module - first order system
////////////////////////////////////////////////////////////////////////////////

enum {PID11, PID12, PID21, PID22} PID_TO_TEST = PID22;

logic signed [ 14-1: 0] dat_a_in        ;
logic signed [ 14-1: 0] dat_b_in        ;
logic signed [ 14-1: 0] dat_a_out       ;
logic signed [ 14-1: 0] dat_b_out       ;

logic signed [14-1: 0] limit_a_out;
logic signed [14-1: 0] limit_b_out;
logic [1:0] dat_a_railed;
logic [1:0] dat_b_railed;

logic [ 32-1: 0] pid_sys_addr        ;
logic [ 32-1: 0] pid_sys_wdata       ;
logic            pid_sys_wen         ;
logic            pid_sys_ren         ;
logic [ 32-1: 0] pid_sys_rdata       ;
logic            pid_sys_err         ;
logic            pid_sys_ack         ;

logic [ 32-1: 0] limit_sys_addr        ;
logic [ 32-1: 0] limit_sys_wdata       ;
logic            limit_sys_wen         ;
logic            limit_sys_ren         ;
logic [ 32-1: 0] limit_sys_rdata       ;
logic            limit_sys_err         ;
logic            limit_sys_ack         ;

logic [ 32-1: 0] pid_config            ;

genvar k;
reg  [14-1: 0] uut_reg [0:21];
reg  [20-1: 0] uut_sum ;
integer        uut_scl ;
reg  [14-1: 0] uut_aaa ;
reg  [14-1: 0] uut_bbb ;
reg  [14-1: 0] uut_out ;
reg            uut_en  ;

generate
for (k=0; k<21; k=k+1) begin : delay
   always @(posedge clk)
      uut_reg[k+1] <= uut_reg[k] ;
end
endgenerate

always @(posedge clk) begin
   if (PID_TO_TEST == PID11 || PID_TO_TEST == PID12) begin
      uut_reg[0] <= dat_a_out ;
      uut_sum <= $signed(dat_a_out)   + $signed(uut_reg[ 0]) + $signed(uut_reg[ 1]) + $signed(uut_reg[ 2]) + $signed(uut_reg[ 3])
               + $signed(uut_reg[ 4]) + $signed(uut_reg[ 5]) + $signed(uut_reg[ 6]) + $signed(uut_reg[ 7]) + $signed(uut_reg[ 8])
               + $signed(uut_reg[ 9]) + $signed(uut_reg[10]) + $signed(uut_reg[11]) + $signed(uut_reg[12]) + $signed(uut_reg[13])
               + $signed(uut_reg[14]) + $signed(uut_reg[15]) + $signed(uut_reg[16]) + $signed(uut_reg[17]) + $signed(uut_reg[18]) ;
   end else begin
      uut_reg[0] <= dat_b_out ;
      uut_sum <= $signed(dat_b_out)   + $signed(uut_reg[ 0]) + $signed(uut_reg[ 1]) + $signed(uut_reg[ 2]) + $signed(uut_reg[ 3])
               + $signed(uut_reg[ 4]) + $signed(uut_reg[ 5]) + $signed(uut_reg[ 6]) + $signed(uut_reg[ 7]) + $signed(uut_reg[ 8])
               + $signed(uut_reg[ 9]) + $signed(uut_reg[10]) + $signed(uut_reg[11]) + $signed(uut_reg[12]) + $signed(uut_reg[13])
               + $signed(uut_reg[14]) + $signed(uut_reg[15]) + $signed(uut_reg[16]) + $signed(uut_reg[17]) + $signed(uut_reg[18]) ;
   end
   
   uut_scl <= $signed(uut_sum) / 20 ; // make simple avarage
   uut_aaa <= uut_scl ;
   uut_bbb <= uut_aaa ;
   uut_out <= uut_scl ;
   
   if (PID_TO_TEST == PID11 || PID_TO_TEST == PID21) begin
       dat_a_in <= uut_en ? uut_out : 14'h0 ;
   end else begin
       dat_b_in <= uut_en ? uut_out : 14'h0 ;
   end
end

initial begin
   if (PID_TO_TEST == PID11 || PID_TO_TEST == PID21) begin
       dat_b_in <= 14'd2000;
   end else begin
       dat_a_in <= 14'd2000;
   end
   uut_en   <=  1'b0 ;

   wait (rstn)
   repeat(10) @(posedge clk);
   //PID settings
   pid_config = 32'b11111111;
   pid_bus.write(32'h00, pid_config);  // negative gain sign, all integrators reset
   pid_bus.write(32'h10+PID_TO_TEST*16, 14'd7000  );  // set point
   pid_bus.write(32'h14+PID_TO_TEST*16, 24'd2000);  // Kp
   pid_bus.write(32'h18+PID_TO_TEST*16, 24'd1000000 );  // Ki
   pid_bus.write(32'h1C+PID_TO_TEST*16, 24'd0  );  // Kd
   repeat(100) @(posedge clk);

   uut_en <= 1'b1 ;
   repeat(20) @(posedge clk);

   pid_config &= ~(32'b1 << PID_TO_TEST);  // int reset off
   pid_bus.write(32'h00, pid_config);
   repeat(2000) @(posedge clk);

   pid_bus.write(32'h50+PID_TO_TEST*12, 14'd1000); // relock upper limit
   pid_bus.write(32'h54+PID_TO_TEST*12, 14'd6000); // relock lower limit
   pid_bus.write(32'h58+PID_TO_TEST*12, 24'd1000000); // relock step size
   pid_config |= (32'b1 << PID_TO_TEST+16);  // enable relock
   pid_bus.write(32'h00, pid_config);
   repeat(2000) @(posedge clk);

   if (PID_TO_TEST == PID11 || PID_TO_TEST == PID21) begin
       dat_b_in <= 14'd0; // leave locked window
   end else begin
       dat_a_in <= 14'd0; // leave locked window
   end
   repeat(10000) @(posedge clk);

   if (PID_TO_TEST == PID11 || PID_TO_TEST == PID21) begin
       dat_b_in <= 14'd3000; // re-enter locked window
   end else begin
       dat_a_in <= 14'd3000; // re-enter locked window
   end
   repeat(10000) @(posedge clk);

   pid_bus.write(32'h10+PID_TO_TEST*16, 14'd8191);  // set point at limit
   repeat(10000) @(posedge clk);

   if (PID_TO_TEST == PID11 || PID_TO_TEST == PID21) begin
       dat_b_in <= 14'd0; // leave locked window, this should trigger a integrator reset
   end else begin
       dat_a_in <= 14'd0; // leave locked window, this should trigger a integrator reset
   end
   repeat(10000) @(posedge clk);
   $finish;
end

////////////////////////////////////////////////////////////////////////////////
// module instances
////////////////////////////////////////////////////////////////////////////////

sys_bus_model pid_bus (
  // system signals
  .clk          (clk      ),
  .rstn         (rstn     ),
  // bus protocol signals
  .sys_addr     (pid_sys_addr ),
  .sys_wdata    (pid_sys_wdata),
  .sys_wen      (pid_sys_wen  ),
  .sys_ren      (pid_sys_ren  ),
  .sys_rdata    (pid_sys_rdata),
  .sys_err      (pid_sys_err  ),
  .sys_ack      (pid_sys_ack  ) 
);

red_pitaya_pid pid (
   // signals
  .clk_i        (clk      ),  // clock
  .rstn_i       (rstn     ),  // reset - active low
  .dat_a_i      (dat_a_in ),  // in 1
  .dat_b_i      (dat_b_in ),  // in 2
  .railed_a_i   (dat_a_railed),
  .railed_b_i   (dat_b_railed),
  .dat_a_o      (dat_a_out),  // out 1
  .dat_b_o      (dat_b_out),  // out 2
   // System bus
  .sys_addr     (pid_sys_addr ),
  .sys_wdata    (pid_sys_wdata),
  .sys_wen      (pid_sys_wen  ),
  .sys_ren      (pid_sys_ren  ),
  .sys_rdata    (pid_sys_rdata),
  .sys_err      (pid_sys_err  ),
  .sys_ack      (pid_sys_ack  )
);

sys_bus_model limit_bus (
  // system signals
  .clk          (clk      ),
  .rstn         (rstn     ),
  // bus protocol signals
  .sys_addr     (limit_sys_addr ),
  .sys_wdata    (limit_sys_wdata),
  .sys_wen      (limit_sys_wen  ),
  .sys_ren      (limit_sys_ren  ),
  .sys_rdata    (limit_sys_rdata),
  .sys_err      (limit_sys_err  ),
  .sys_ack      (limit_sys_ack  ) 
);

red_pitaya_limit limit (
    // signals
    .clk_i          (clk            ),
    .rstn_i         (rstn           ),
    .dat_a_i        (dat_a_out      ),
    .dat_b_i        (dat_b_out      ),
    .dat_a_o        (limit_a_out    ),
    .dat_b_o        (limit_b_out    ),
    .dat_a_railed_o (dat_a_railed   ),
    .dat_b_railed_o (dat_b_railed   ),
    // System bus
    .sys_addr       (limit_sys_addr ),
    .sys_wdata      (limit_sys_wdata),
    .sys_wen        (limit_sys_wen  ),
    .sys_ren        (limit_sys_ren  ),
    .sys_rdata      (limit_sys_rdata),
    .sys_err        (limit_sys_err  ),
    .sys_ack        (limit_sys_ack  )
);

////////////////////////////////////////////////////////////////////////////////
// waveforms
////////////////////////////////////////////////////////////////////////////////

initial begin
  $dumpfile("pid_tb.vcd");
  $dumpvars(0, pid_tb);
end

endmodule: pid_tb

