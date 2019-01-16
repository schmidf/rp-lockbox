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

wire        [14-1: 0    ] pid_in               [3:0];
wire signed [14-1: 0    ] pid_out              [3:0];
reg         [14-1: 0    ] set_sp               [3:0];
reg         [KP_BITS-1:0] set_kp               [3:0];
reg         [KI_BITS-1:0] set_ki               [3:0];
reg         [14-1:0]      set_kd               [3:0];
reg         [3:0]         pid_inverted              ;
reg         [3:0]         set_irst                  ;
reg         [3:0]         set_irst_when_railed      ;
reg         [3:0]         set_hold                 ;
wire                      pid_irst             [3:0];
wire                      pid_hold             [3:0];
wire        [1:0]         pid_railed_i         [3:0];

wire signed [15-1:0]      pid_sum              [3:0];
wire signed [14-1:0]      pid_sat              [3:0];


reg         [3:0]                  relock_enabled;
reg         [12-1:0]               relock_minval    [3:0];
reg         [12-1:0]               relock_maxval    [3:0];
reg         [RELOCK_STEP_BITS-1:0] relock_stepsize  [3:0];
reg         [2-1:0]                relock_source    [3:0];
wire                               relock_clear_o   [3:0];
wire signed [14-1:0]               relock_signal_o  [3:0];
wire                               relock_hold_o    [3:0];
wire        [12-1:0]               relock_signal_i  [3:0];
wire                               relock_hold_i    [3:0];

wire        [12-1:0]               relock_i         [3:0];
assign relock_i[0] = relock_a_i;
assign relock_i[1] = relock_b_i;
assign relock_i[2] = relock_c_i;
assign relock_i[3] = relock_d_i;

genvar pid_index;

generate for (pid_index = 0; pid_index < 4; pid_index = pid_index + 1) begin
    assign pid_hold[pid_index] = relock_hold_o[pid_index] || set_hold[pid_index];
    assign pid_sum[pid_index] = pid_out[pid_index] + relock_signal_o[pid_index];
    assign pid_sat[pid_index] = (^pid_sum[pid_index][15-1:15-2]) ?
                                {pid_sum[pid_index][15-1], {13{~pid_sum[pid_index][15-1]}}} :
                                pid_sum[pid_index][14-1:0];

    assign relock_signal_i[pid_index] = relock_i[relock_source[pid_index]];

    red_pitaya_pid_block #(
      .PSR     (  PSR   ),
      .ISR     (  ISR   ),
      .DSR     (  DSR   ),
      .KP_BITS ( KP_BITS),
      .KI_BITS ( KI_BITS)
    ) i_pid (
       // data
      .clk_i        (  clk_i                  ),  // clock
      .rstn_i       (  rstn_i                 ),  // reset - active low
      .railed_i     (  pid_railed_i[pid_index]),  // output railed 
      .hold_i       (  pid_hold[pid_index]    ),  // PID internal state hold
      .dat_i        (  pid_in[pid_index]      ),  // input data
      .dat_o        (  pid_out[pid_index]     ),  // output data

       // settings
      .set_sp_i     (  set_sp[pid_index]      ),  // set point
      .set_kp_i     (  set_kp[pid_index]      ),  // Kp
      .set_ki_i     (  set_ki[pid_index]      ),  // Ki
      .set_kd_i     (  set_kd[pid_index]      ),  // Kd
      .inverted_i   (  pid_inverted[pid_index]),  // feedback sign
      .int_rst_i    (  pid_irst[pid_index]    )   // integrator reset
    );

    pid_relock #(
        .STEPSR(RELOCK_STEPSR),
        .STEP_BITS(RELOCK_STEP_BITS)
    ) i_relock (
        .clk_i(clk_i),
        .on_i(relock_enabled[pid_index]),
        .min_val_i(relock_minval[pid_index]),
        .max_val_i(relock_maxval[pid_index]),
        .stepsize_i(relock_stepsize[pid_index]),
        .signal_i(relock_signal_i[pid_index]),
        .railed_i(pid_railed_i[pid_index]),
        .hold_i(relock_hold_i[pid_index]),
        .hold_o(relock_hold_o[pid_index]),
        .clear_o(relock_clear_o[pid_index]),
        .signal_o(relock_signal_o[pid_index])
    );
end
endgenerate

assign pid_in[0] = dat_a_i;
assign pid_in[1] = dat_b_i;
assign pid_in[2] = dat_a_i;
assign pid_in[3] = dat_b_i;

assign pid_irst[0] = set_irst[0] || (set_irst_when_railed[0] && (railed_a_i[0] || railed_a_i[1]))
                     || relock_clear_o[0]; 
assign pid_irst[1] = set_irst[1] || (set_irst_when_railed[1] && (railed_a_i[0] || railed_a_i[1]))
                     || relock_clear_o[1]; 
assign pid_irst[2] = set_irst[2] || (set_irst_when_railed[2] && (railed_b_i[0] || railed_b_i[1]))
                     || relock_clear_o[2]; 
assign pid_irst[3] = set_irst[3] || (set_irst_when_railed[3] && (railed_b_i[0] || railed_b_i[1]))
                     || relock_clear_o[3]; 

assign pid_railed_i[0] = railed_a_i;
assign pid_railed_i[1] = railed_a_i;
assign pid_railed_i[2] = railed_b_i;
assign pid_railed_i[3] = railed_b_i;

assign relock_hold_i[0] = set_hold[0];
assign relock_hold_i[1] = set_hold[1];
assign relock_hold_i[2] = set_hold[2];
assign relock_hold_i[3] = set_hold[3];

//---------------------------------------------------------------------------------
//  Sum and saturation

wire [ 15-1: 0] out_1_sum   ;
reg  [ 14-1: 0] out_1_sat   ;
wire [ 15-1: 0] out_2_sum   ;
reg  [ 14-1: 0] out_2_sat   ;

assign out_1_sum = $signed(pid_sat[0]) + $signed(pid_sat[1]);
assign out_2_sum = $signed(pid_sat[3]) + $signed(pid_sat[2]);

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

// Numerical parameters write
generate for (pid_index = 0; pid_index < 4; pid_index = pid_index + 1) begin
    always @(posedge clk_i) begin
       if (rstn_i == 1'b0) begin
          set_sp[pid_index]          <= 14'd0 ;
          set_kp[pid_index]          <= {KP_BITS{1'b0}} ;
          set_ki[pid_index]          <= {KI_BITS{1'b0}} ;
          set_kd[pid_index]          <= 14'd0 ;
          relock_minval[pid_index]   <= 12'd0;
          relock_maxval[pid_index]   <= 12'd0;
          relock_stepsize[pid_index] <= {RELOCK_STEP_BITS{1'b0}};
          relock_source[pid_index]   <= 2'd0;
       end
       else begin
          if (sys_wen) begin
             if (sys_addr[19:0]==('h10+4*pid_index))
                 set_sp[pid_index] <= sys_wdata[14-1:0];
             if (sys_addr[19:0]==('h20+4*pid_index))
                 set_kp[pid_index] <= sys_wdata[KP_BITS-1:0];
             if (sys_addr[19:0]==('h30+4*pid_index))
                 set_ki[pid_index] <= sys_wdata[KI_BITS-1:0];
             if (sys_addr[19:0]==('h40+4*pid_index))
                 set_kd[pid_index] <= sys_wdata[14-1:0];
             if (sys_addr[19:0]==('h50+4*pid_index))
                 relock_minval[pid_index]  <= sys_wdata[12-1:0] ;
             if (sys_addr[19:0]==('h60+4*pid_index))
                 relock_maxval[pid_index]  <= sys_wdata[12-1:0] ;
             if (sys_addr[19:0]==('h70+4*pid_index))
                 relock_stepsize[pid_index]  <= sys_wdata[RELOCK_STEP_BITS-1:0] ;
             if (sys_addr[19:0]==('h80+4*pid_index))
                 relock_source[pid_index]  <= sys_wdata[2-1:0] ;
          end
       end
    end
end
endgenerate

// Flags write
always @(posedge clk_i) begin
    if (rstn_i == 1'b0) begin
          relock_enabled <=  4'b0   ;
          set_hold       <=  4'b0   ;
          pid_inverted   <=  4'b0   ;
          set_irst       <=  4'b1111;
    end
    else begin
        if (rstn_i & sys_wen & sys_addr[19:0]==16'h0)
            {relock_enabled,
             set_hold,
             set_irst_when_railed,
             pid_inverted,
             set_irst}
            <= sys_wdata[20-1:0]; 
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
       20'h00: begin
          sys_ack <= sys_en;
          sys_rdata <= {{32-20{1'b0}}, relock_enabled, set_hold, set_irst_when_railed,
                        pid_inverted, set_irst};
      end 

      20'h1?: begin sys_ack <= sys_en; sys_rdata <= {{32-14{1'b0}}, set_sp[sys_addr[3:0] >> 2]}; end 
      20'h2?: begin sys_ack <= sys_en; sys_rdata <= {{32-KP_BITS{1'b0}}, set_kp[sys_addr[3:0] >> 2]}; end 
      20'h3?: begin sys_ack <= sys_en; sys_rdata <= {{32-KI_BITS{1'b0}}, set_ki[sys_addr[3:0] >> 2]}; end 
      20'h4?: begin sys_ack <= sys_en; sys_rdata <= {{32-14{1'b0}}, set_kd[sys_addr[3:0] >> 2]}; end 

      20'h5?: begin sys_ack <= sys_en; sys_rdata <= {{32-12{1'b0}}, relock_minval[sys_addr[3:0] >> 2]}; end 
      20'h6?: begin sys_ack <= sys_en; sys_rdata <= {{32-12{1'b0}}, relock_maxval[sys_addr[3:0] >> 2]}; end 
      20'h7?: begin sys_ack <= sys_en; sys_rdata <= {{32-RELOCK_STEP_BITS{1'b0}}, relock_stepsize[sys_addr[3:0] >> 2]}; end 
      20'h8?: begin sys_ack <= sys_en; sys_rdata <= {{32-2{1'b0}}, relock_source[sys_addr[3:0] >> 2]}; end 

     default: begin sys_ack <= sys_en; sys_rdata <=  32'h0; end
   endcase
end

endmodule
