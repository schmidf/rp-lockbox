/**
 * Copyright (c) 2018 Fabian Schmid
 *
 * All rights reserved.
 *
 * $Id: red_pitaya_pid.v 961 2014-01-21 11:40:39Z matej.oblak $
 *
 * @brief Red Pitaya MIMO PID controller.
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
 * Multiple input multiple output controller.
 *
 *
 *                 /-------\       /-----------\
 *   CHA -----+--> | PID11 | ------| SUM & SAT | ---> CHA
 *            |    \-------/       \-----------/
 *            |                            ^
 *            |    /-------\               |
 *            ---> | PID21 | ----------    |
 *                 \-------/           |   |
 *                                     |   |
 *  INPUT                              |   |         OUTPUT
 *                                     |   |
 *                 /-------\           |   |
 *            ---> | PID12 | --------------
 *            |    \-------/           |    
 *            |                        ˇ
 *            |    /-------\       /-----------\
 *   CHB -----+--> | PID22 | ------| SUM & SAT | ---> CHB
 *                 \-------/       \-----------/
 *
 *
 * MIMO controller is build from four equal submodules, each can have 
 * different settings.
 *
 * Each output is sum of two controllers with different input. That sum is also
 * saturated to protect from wrapping.
 * 
 */
`timescale 1ns / 1ps
module red_pitaya_pid (
   // signals
   input                 clk_i           ,  //!< processing clock
   input                 rstn_i          ,  //!< processing reset - active low
   input      [ 14-1: 0] dat_a_i         ,  //!< input data CHA
   input      [ 14-1: 0] dat_b_i         ,  //!< input data CHB
   input      [    1: 0] railed_a_i      ,  //!< input CHA railed
   input      [    1: 0] railed_b_i      ,  //!< input CHA railed
   input      [ 12-1: 0] relock_a_i      ,
   input      [ 12-1: 0] relock_b_i      ,
   input      [ 12-1: 0] relock_c_i      ,
   input      [ 12-1: 0] relock_d_i      ,
   output     [ 14-1: 0] dat_a_o         ,  //!< output data CHA
   output     [ 14-1: 0] dat_b_o         ,  //!< output data CHB
  
   // system bus
   input      [ 32-1: 0] sys_addr        ,  //!< bus address
   input      [ 32-1: 0] sys_wdata       ,  //!< bus write data
   input                 sys_wen         ,  //!< bus write enable
   input                 sys_ren         ,  //!< bus read enable
   output reg [ 32-1: 0] sys_rdata       ,  //!< bus read data
   output reg            sys_err         ,  //!< bus error indicator
   output reg            sys_ack            //!< bus acknowledge signal
);

localparam  PSR = 12         ;              // p gain = Kp >> PSR
localparam  ISR = 28         ;              // i gain = Ki >> ISR
localparam  DSR = 10         ;
localparam  KP_BITS = 24     ;
localparam  KI_BITS = 24     ;
localparam  KD_BITS = 24     ;
localparam  RELOCK_STEP_BITS = 24;
localparam  RELOCK_STEPSR = 18;

reg                                relock_enabled   [3:0];
reg         [12-1:0]               relock_minval    [3:0];
reg         [12-1:0]               relock_maxval    [3:0];
reg         [RELOCK_STEP_BITS-1:0] relock_stepsize  [3:0];
wire                               relock_clear_o   [3:0];
wire signed [14-1:0]               relock_signal_o  [3:0];
wire                               relock_hold_o    [3:0];
wire        [12-1:0]               relock_signal_i  [3:0];
//wire                               relock_hold_i    [3:0];
wire                               relock_railed_i  [3:0];

assign relock_signal_i[0] = relock_a_i;
assign relock_signal_i[1] = relock_b_i;
assign relock_signal_i[2] = relock_c_i;
assign relock_signal_i[3] = relock_d_i;

assign relock_railed_i[0] = railed_a_i;
assign relock_railed_i[1] = railed_a_i;
assign relock_railed_i[2] = railed_b_i;
assign relock_railed_i[3] = railed_b_i;

//---------------------------------------------------------------------------------
//  PID 11: In1, Out1

wire signed [14-1: 0     ] pid_11_out;
reg         [14-1: 0     ] set_11_sp;
reg         [KP_BITS-1: 0] set_11_kp;
reg         [KI_BITS-1: 0] set_11_ki;
reg         [14-1:0]       set_11_kd;
reg                        pid_11_inverted;
reg                        set_11_irst  ;
reg                        set_11_irst_when_railed;
reg                        set_11_ihold ;
wire                       pid_11_irst;
wire                       pid_11_ihold;

wire signed [15-1:0] pid_11_sum;
wire signed [14-1:0] pid_11_sat;

assign pid_11_irst = set_11_irst ||
                    (set_11_irst_when_railed && (railed_a_i[0] || railed_a_i[1])) ||
                    relock_clear_o[0]; 
assign pid_11_ihold = relock_hold_o[0] || set_11_ihold;
assign pid_11_sum = pid_11_out + relock_signal_o[0];
assign pid_11_sat = (^pid_11_sum[15-1:15-2]) ? {pid_11_sum[15-1], {13{~pid_11_sum[15-1]}}} : pid_11_sum[14-1:0];

red_pitaya_pid_block #(
  .PSR     (  PSR   ),
  .ISR     (  ISR   ),
  .DSR     (  DSR   ),
  .KP_BITS ( KP_BITS),
  .KI_BITS ( KI_BITS)
) i_pid11 (
   // data
  .clk_i        (  clk_i          ),  // clock
  .rstn_i       (  rstn_i         ),  // reset - active low
  .railed_i     (  railed_a_i     ),  // output railed 
  .hold_i       (  pid_11_ihold   ),  // integrator hold
  .dat_i        (  dat_a_i        ),  // input data
  .dat_o        (  pid_11_out     ),  // output data

   // settings
  .set_sp_i     (  set_11_sp      ),  // set point
  .set_kp_i     (  set_11_kp      ),  // Kp
  .set_ki_i     (  set_11_ki      ),  // Ki
  .set_kd_i     (  set_11_kd      ),  // Kd
  .inverted_i   (  pid_11_inverted),  // feedback sign
  .int_rst_i    (  pid_11_irst    )   // integrator reset
);

pid_relock #(
    .STEPSR(RELOCK_STEPSR),
    .STEP_BITS(RELOCK_STEP_BITS)
) relock_11 (
    .clk_i(clk_i),
    .on_i(relock_enabled[0]),
    .min_val_i(relock_minval[0]),
    .max_val_i(relock_maxval[0]),
    .stepsize_i(relock_stepsize[0]),
    .signal_i(relock_signal_i[0]),
    .railed_i(relock_railed_i[0]),
    .hold_i(set_11_ihold),
    .hold_o(relock_hold_o[0]),
    .clear_o(relock_clear_o[0]),
    .signal_o(relock_signal_o[0])
);

//---------------------------------------------------------------------------------
//  PID 21: In1, Out2

wire signed [ 14-1: 0] pid_21_out   ;
reg  [ 14-1: 0] set_21_sp    ;
reg  [ KP_BITS-1: 0] set_21_kp    ;
reg  [ KI_BITS-1: 0] set_21_ki    ;
reg  [ 14-1: 0] set_21_kd    ;
reg             pid_21_inverted;
reg             set_21_irst  ;
reg             set_21_irst_when_railed;
reg             set_21_ihold ;
wire            pid_21_irst;
wire            pid_21_ihold;

wire signed [15-1:0] pid_21_sum;
wire signed [14-1:0] pid_21_sat;

assign pid_21_irst = set_21_irst ||
                    (set_21_irst_when_railed && (railed_b_i[0] || railed_b_i[1])) ||
                    relock_clear_o[2]; 
assign pid_21_ihold = relock_hold_o[2] || set_21_ihold;
assign pid_21_sum = pid_21_out + relock_signal_o[2];
assign pid_21_sat = (^pid_21_sum[15-1:15-2]) ? {pid_21_sum[15-1], {13{~pid_21_sum[15-1]}}} : pid_21_sum[14-1:0];

red_pitaya_pid_block #(
  .PSR     (  PSR   ),
  .ISR     (  ISR   ),
  .DSR     (  DSR   ),
  .KP_BITS ( KP_BITS),
  .KI_BITS ( KI_BITS)
) i_pid21 (
   // data
  .clk_i        (  clk_i          ),  // clock
  .rstn_i       (  rstn_i         ),  // reset - active low
  .railed_i     (  railed_b_i     ),  // output railed 
  .hold_i       (  pid_21_ihold   ),  // integrator hold
  .dat_i        (  dat_a_i        ),  // input data
  .dat_o        (  pid_21_out     ),  // output data

   // settings
  .set_sp_i     (  set_21_sp      ),  // set point
  .set_kp_i     (  set_21_kp      ),  // Kp
  .set_ki_i     (  set_21_ki      ),  // Ki
  .set_kd_i     (  set_21_kd      ),  // Kd
  .inverted_i   (  pid_21_inverted),  // feedback sign
  .int_rst_i    (  pid_21_irst    )   // integrator reset
);

pid_relock #(
    .STEPSR(RELOCK_STEPSR),
    .STEP_BITS(RELOCK_STEP_BITS)
) relock_21 (
    .clk_i(clk_i),
    .on_i(relock_enabled[2]),
    .min_val_i(relock_minval[2]),
    .max_val_i(relock_maxval[2]),
    .stepsize_i(relock_stepsize[2]),
    .signal_i(relock_signal_i[2]),
    .railed_i(relock_railed_i[2]),
    .hold_i(set_21_ihold),
    .hold_o(relock_hold_o[2]),
    .clear_o(relock_clear_o[2]),
    .signal_o(relock_signal_o[2])
);

//---------------------------------------------------------------------------------
//  PID 12: In2, Out1

wire signed [ 14-1: 0] pid_12_out   ;
reg  [ 14-1: 0] set_12_sp    ;
reg  [ KP_BITS-1: 0] set_12_kp    ;
reg  [ KI_BITS-1: 0] set_12_ki    ;
reg  [ 14-1: 0] set_12_kd    ;
reg             pid_12_inverted;
reg             set_12_irst  ;
reg             set_12_irst_when_railed;
reg             set_12_ihold ;
wire            pid_12_irst;
wire            pid_12_ihold;

wire signed [15-1:0] pid_12_sum;
wire signed [14-1:0] pid_12_sat;

assign pid_12_irst = set_12_irst ||
                    (set_12_irst_when_railed && (railed_a_i[0] || railed_a_i[1])) ||
                    relock_clear_o[1]; 
assign pid_12_ihold = relock_hold_o[1] || set_12_ihold;
assign pid_12_sum = pid_12_out + relock_signal_o[1];
assign pid_12_sat = (^pid_12_sum[15-1:15-2]) ? {pid_12_sum[15-1], {13{~pid_12_sum[15-1]}}} : pid_12_sum[14-1:0];

red_pitaya_pid_block #(
  .PSR     (  PSR   ),
  .ISR     (  ISR   ),
  .DSR     (  DSR   ),
  .KP_BITS ( KP_BITS),
  .KI_BITS ( KI_BITS)
) i_pid12 (
   // data
  .clk_i        (  clk_i          ),  // clock
  .rstn_i       (  rstn_i         ),  // reset - active low
  .railed_i     (  railed_a_i     ),  // output railed 
  .hold_i       (  pid_12_ihold   ),  // integrator hold
  .dat_i        (  dat_b_i        ),  // input data
  .dat_o        (  pid_12_out     ),  // output data

   // settings
  .set_sp_i     (  set_12_sp      ),  // set point
  .set_kp_i     (  set_12_kp      ),  // Kp
  .set_ki_i     (  set_12_ki      ),  // Ki
  .set_kd_i     (  set_12_kd      ),  // Kd
  .inverted_i   (  pid_12_inverted),  // feedback sign
  .int_rst_i    (  pid_12_irst    )   // integrator reset
);

pid_relock #(
    .STEPSR(RELOCK_STEPSR),
    .STEP_BITS(RELOCK_STEP_BITS)
) relock_12 (
    .clk_i(clk_i),
    .on_i(relock_enabled[1]),
    .min_val_i(relock_minval[1]),
    .max_val_i(relock_maxval[1]),
    .stepsize_i(relock_stepsize[1]),
    .signal_i(relock_signal_i[1]),
    .railed_i(relock_railed_i[1]),
    .hold_i(set_12_ihold),
    .hold_o(relock_hold_o[1]),
    .clear_o(relock_clear_o[1]),
    .signal_o(relock_signal_o[1])
);

//---------------------------------------------------------------------------------
//  PID 22 In2, Out2

wire signed [ 14-1: 0] pid_22_out   ;
reg  [ 14-1: 0] set_22_sp    ;
reg  [ KP_BITS-1: 0] set_22_kp    ;
reg  [ KI_BITS-1: 0] set_22_ki    ;
reg  [ 14-1: 0] set_22_kd    ;
reg             pid_22_inverted;
reg             set_22_irst  ;
reg             set_22_irst_when_railed;
reg             set_22_ihold ;
wire            pid_22_irst;
wire            pid_22_ihold;

wire signed [15-1:0] pid_22_sum;
wire signed [14-1:0] pid_22_sat;

assign pid_22_irst = set_22_irst ||
                    (set_22_irst_when_railed && (railed_b_i[0] || railed_b_i[1])) ||
                    relock_clear_o[3]; 
assign pid_22_ihold = relock_hold_o[3] || set_22_ihold;
assign pid_22_sum = pid_22_out + relock_signal_o[3];
assign pid_22_sat = (^pid_22_sum[15-1:15-2]) ? {pid_22_sum[15-1], {13{~pid_22_sum[15-1]}}} : pid_22_sum[14-1:0];

red_pitaya_pid_block #(
  .PSR     (  PSR   ),
  .ISR     (  ISR   ),
  .DSR     (  DSR   ),
  .KP_BITS ( KP_BITS),
  .KI_BITS ( KI_BITS)
) i_pid22 (
   // data
  .clk_i        (  clk_i          ),  // clock
  .rstn_i       (  rstn_i         ),  // reset - active low
  .railed_i     (  railed_b_i     ),  // output railed 
  .hold_i       (  pid_22_ihold   ),  // integrator hold
  .dat_i        (  dat_b_i        ),  // input data
  .dat_o        (  pid_22_out     ),  // output data

   // settings
  .set_sp_i     (  set_22_sp      ),  // set point
  .set_kp_i     (  set_22_kp      ),  // Kp
  .set_ki_i     (  set_22_ki      ),  // Ki
  .set_kd_i     (  set_22_kd      ),  // Kd
  .inverted_i   (  pid_22_inverted),  // feedback sign
  .int_rst_i    (  pid_22_irst    )   // integrator reset
);

pid_relock #(
    .STEPSR(RELOCK_STEPSR),
    .STEP_BITS(RELOCK_STEP_BITS)
) relock_22 (
    .clk_i(clk_i),
    .on_i(relock_enabled[3]),
    .min_val_i(relock_minval[3]),
    .max_val_i(relock_maxval[3]),
    .stepsize_i(relock_stepsize[3]),
    .signal_i(relock_signal_i[3]),
    .railed_i(relock_railed_i[3]),
    .hold_i(set_22_ihold),
    .hold_o(relock_hold_o[3]),
    .clear_o(relock_clear_o[3]),
    .signal_o(relock_signal_o[3])
);

//---------------------------------------------------------------------------------
//  Sum and saturation

wire [ 15-1: 0] out_1_sum   ;
reg  [ 14-1: 0] out_1_sat   ;
wire [ 15-1: 0] out_2_sum   ;
reg  [ 14-1: 0] out_2_sat   ;

assign out_1_sum = $signed(pid_11_sat) + $signed(pid_12_sat);
assign out_2_sum = $signed(pid_22_sat) + $signed(pid_21_sat);

always @(posedge clk_i) begin
   if (rstn_i == 1'b0) begin
      out_1_sat <= 14'd0 ;
      out_2_sat <= 14'd0 ;
   end
   else begin
      if (out_1_sum[15-1:15-2]==2'b01) // postitive sat
         out_1_sat <= 14'h1FFF ;
      else if (out_1_sum[15-1:15-2]==2'b10) // negative sat
         out_1_sat <= 14'h2000 ;
      else
         out_1_sat <= out_1_sum[14-1:0] ;

      if (out_2_sum[15-1:15-2]==2'b01) // postitive sat
         out_2_sat <= 14'h1FFF ;
      else if (out_2_sum[15-1:15-2]==2'b10) // negative sat
         out_2_sat <= 14'h2000 ;
      else
         out_2_sat <= out_2_sum[14-1:0] ;
   end
end

assign dat_a_o = out_1_sat ;
assign dat_b_o = out_2_sat ;

//---------------------------------------------------------------------------------
//
//  System bus connection

always @(posedge clk_i) begin
   if (rstn_i == 1'b0) begin
      set_11_sp          <= 14'd0 ;
      set_11_kp          <= {KP_BITS{1'b0}} ;
      set_11_ki          <= {KI_BITS{1'b0}} ;
      set_11_kd          <= 14'd0 ;
      pid_11_inverted    <=  1'b0 ;
      set_11_irst        <=  1'b1 ;
      set_12_sp          <= 14'd0 ;
      set_12_kp          <= {KP_BITS{1'b0}} ;
      set_12_ki          <= {KI_BITS{1'b0}} ;
      set_12_kd          <= 14'd0 ;
      pid_12_inverted    <=  1'b0 ;
      set_12_irst        <=  1'b1 ;
      set_21_sp          <= 14'd0 ;
      set_21_kp          <= {KP_BITS{1'b0}} ;
      set_21_ki          <= {KI_BITS{1'b0}} ;
      set_21_kd          <= 14'd0 ;
      pid_21_inverted    <=  1'b0 ;
      set_21_irst        <=  1'b1 ;
      set_22_sp          <= 14'd0 ;
      set_22_kp          <= {KP_BITS{1'b0}} ;
      set_22_ki          <= {KI_BITS{1'b0}} ;
      set_22_kd          <= 14'd0 ;
      pid_22_inverted    <=  1'b0 ;
      set_22_irst        <=  1'b1 ;
      relock_minval[0]   <= 12'd0;
      relock_maxval[0]   <= 12'd0;
      relock_stepsize[0] <= {RELOCK_STEP_BITS{1'b0}};
      relock_enabled[0]  <=1'b0;
      relock_minval[1]   <= 12'd0;
      relock_maxval[1]   <= 12'd0;
      relock_stepsize[1] <= {RELOCK_STEP_BITS{1'b0}};
      relock_enabled[1]  <=1'b0;
      relock_minval[2]   <= 12'd0;
      relock_maxval[2]   <= 12'd0;
      relock_stepsize[2] <= {RELOCK_STEP_BITS{1'b0}};
      relock_enabled[2]  <=1'b0;
      relock_minval[3]   <= 12'd0;
      relock_maxval[3]   <= 12'd0;
      relock_stepsize[3] <= {RELOCK_STEP_BITS{1'b0}};
      relock_enabled[3]  <=1'b0;
   end
   else begin
      if (sys_wen) begin
         if (sys_addr[19:0]==16'h0) {
             relock_enabled[3],
             relock_enabled[2],
             relock_enabled[1],
             relock_enabled[0],
             set_22_ihold,
             set_21_ihold,
             set_12_ihold,
             set_11_ihold,
             set_22_irst_when_railed,
             set_21_irst_when_railed,
             set_12_irst_when_railed,
             set_11_irst_when_railed,
             pid_22_inverted,
             pid_21_inverted,
             pid_12_inverted,
             pid_11_inverted,
             set_22_irst,
             set_21_irst,
             set_12_irst,
             set_11_irst
             } <= sys_wdata[ 20-1:0] ;

         if (sys_addr[19:0]==16'h10)    set_11_sp  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h14)    set_11_kp  <= sys_wdata[KP_BITS-1:0] ;
         if (sys_addr[19:0]==16'h18)    set_11_ki  <= sys_wdata[KI_BITS-1:0] ;
         if (sys_addr[19:0]==16'h1C)    set_11_kd  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h20)    set_12_sp  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h24)    set_12_kp  <= sys_wdata[KP_BITS-1:0] ;
         if (sys_addr[19:0]==16'h28)    set_12_ki  <= sys_wdata[KI_BITS-1:0] ;
         if (sys_addr[19:0]==16'h2C)    set_12_kd  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h30)    set_21_sp  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h34)    set_21_kp  <= sys_wdata[KP_BITS-1:0] ;
         if (sys_addr[19:0]==16'h38)    set_21_ki  <= sys_wdata[KI_BITS-1:0] ;
         if (sys_addr[19:0]==16'h3C)    set_21_kd  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h40)    set_22_sp  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h44)    set_22_kp  <= sys_wdata[KP_BITS-1:0] ;
         if (sys_addr[19:0]==16'h48)    set_22_ki  <= sys_wdata[KI_BITS-1:0] ;
         if (sys_addr[19:0]==16'h4C)    set_22_kd  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h50)    relock_minval[0]  <= sys_wdata[12-1:0] ;
         if (sys_addr[19:0]==16'h54)    relock_maxval[0]  <= sys_wdata[12-1:0] ;
         if (sys_addr[19:0]==16'h58)    relock_stepsize[0]  <= sys_wdata[RELOCK_STEP_BITS-1:0] ;
         if (sys_addr[19:0]==16'h5C)    relock_minval[1]  <= sys_wdata[12-1:0] ;
         if (sys_addr[19:0]==16'h60)    relock_maxval[1]  <= sys_wdata[12-1:0] ;
         if (sys_addr[19:0]==16'h64)    relock_stepsize[1]  <= sys_wdata[RELOCK_STEP_BITS-1:0] ;
         if (sys_addr[19:0]==16'h68)    relock_minval[2]  <= sys_wdata[12-1:0] ;
         if (sys_addr[19:0]==16'h6C)    relock_maxval[2]  <= sys_wdata[12-1:0] ;
         if (sys_addr[19:0]==16'h70)    relock_stepsize[2]  <= sys_wdata[RELOCK_STEP_BITS-1:0] ;
         if (sys_addr[19:0]==16'h74)    relock_minval[3]  <= sys_wdata[12-1:0] ;
         if (sys_addr[19:0]==16'h78)    relock_maxval[3]  <= sys_wdata[12-1:0] ;
         if (sys_addr[19:0]==16'h7C)    relock_stepsize[3]  <= sys_wdata[RELOCK_STEP_BITS-1:0] ;
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
      20'h00 : begin sys_ack <= sys_en;          sys_rdata <= {
          {32-20{1'b0}},
          relock_enabled[3],
          relock_enabled[2],
          relock_enabled[1],
          relock_enabled[0],
          set_22_ihold,
          set_21_ihold,
          set_12_ihold,
          set_11_ihold,
          set_22_irst_when_railed,
          set_21_irst_when_railed,
          set_12_irst_when_railed,
          set_11_irst_when_railed,
          pid_22_inverted,
          pid_21_inverted,
          pid_12_inverted,
          pid_11_inverted,
          set_22_irst,
          set_21_irst,
          set_12_irst,
          set_11_irst}; end 

      20'h10 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_11_sp}          ; end 
      20'h14 : begin sys_ack <= sys_en;          sys_rdata <= {{32-KP_BITS{1'b0}}, set_11_kp}          ; end 
      20'h18 : begin sys_ack <= sys_en;          sys_rdata <= {{32-KI_BITS{1'b0}}, set_11_ki}          ; end 
      20'h1C : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_11_kd}          ; end 

      20'h20 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_12_sp}          ; end 
      20'h24 : begin sys_ack <= sys_en;          sys_rdata <= {{32-KP_BITS{1'b0}}, set_12_kp}          ; end 
      20'h28 : begin sys_ack <= sys_en;          sys_rdata <= {{32-KI_BITS{1'b0}}, set_12_ki}          ; end 
      20'h2C : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_12_kd}          ; end 

      20'h30 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_21_sp}          ; end 
      20'h34 : begin sys_ack <= sys_en;          sys_rdata <= {{32-KP_BITS{1'b0}}, set_21_kp}          ; end 
      20'h38 : begin sys_ack <= sys_en;          sys_rdata <= {{32-KI_BITS{1'b0}}, set_21_ki}          ; end 
      20'h3C : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_21_kd}          ; end 

      20'h40 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_22_sp}          ; end 
      20'h44 : begin sys_ack <= sys_en;          sys_rdata <= {{32-KP_BITS{1'b0}}, set_22_kp}          ; end 
      20'h48 : begin sys_ack <= sys_en;          sys_rdata <= {{32-KI_BITS{1'b0}}, set_22_ki}          ; end 
      20'h4C : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_22_kd}          ; end 

      20'h50 : begin sys_ack <= sys_en;          sys_rdata <= {{32-12{1'b0}}, relock_minval[0]}          ; end 
      20'h54 : begin sys_ack <= sys_en;          sys_rdata <= {{32-12{1'b0}}, relock_maxval[0]}          ; end 
      20'h58 : begin sys_ack <= sys_en;          sys_rdata <= {{32-RELOCK_STEP_BITS{1'b0}}, relock_stepsize[0]}          ; end 

      20'h5C : begin sys_ack <= sys_en;          sys_rdata <= {{32-12{1'b0}}, relock_minval[1]}          ; end 
      20'h60 : begin sys_ack <= sys_en;          sys_rdata <= {{32-12{1'b0}}, relock_maxval[1]}          ; end 
      20'h64 : begin sys_ack <= sys_en;          sys_rdata <= {{32-RELOCK_STEP_BITS{1'b0}}, relock_stepsize[1]}          ; end 

      20'h68 : begin sys_ack <= sys_en;          sys_rdata <= {{32-12{1'b0}}, relock_minval[2]}          ; end 
      20'h6C : begin sys_ack <= sys_en;          sys_rdata <= {{32-12{1'b0}}, relock_maxval[2]}          ; end 
      20'h70 : begin sys_ack <= sys_en;          sys_rdata <= {{32-RELOCK_STEP_BITS{1'b0}}, relock_stepsize[2]}          ; end 

      20'h74 : begin sys_ack <= sys_en;          sys_rdata <= {{32-12{1'b0}}, relock_minval[3]}          ; end 
      20'h78 : begin sys_ack <= sys_en;          sys_rdata <= {{32-12{1'b0}}, relock_maxval[3]}          ; end 
      20'h7C : begin sys_ack <= sys_en;          sys_rdata <= {{32-RELOCK_STEP_BITS{1'b0}}, relock_stepsize[3]}          ; end 

     default : begin sys_ack <= sys_en;          sys_rdata <=  32'h0                              ; end
   endcase
end

endmodule
