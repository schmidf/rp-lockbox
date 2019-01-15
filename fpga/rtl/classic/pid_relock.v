/*
 * Copyright (c) 2018 Fabian Schmid
 *
 * All rights reserved.
 *
 * Generates a triangle wave sweep of increasing amplitude if signal_i is not
 * between min_val_i and max_val_i.
 *
 * Based on code by David Leibrandt (National Institute of Standards and 
 * Technology)
 */
`timescale 1ns / 1ps

module pid_relock #(
    parameter STEPSR    = 18,// slew rate (DAC counts per clock cycle) =
                             //stepsize >> STEPSR
    parameter STEP_BITS = 24
)
(
    input  wire                        clk_i,
    input  wire                        on_i,
    input  wire        [12-1:0]        min_val_i,
    input  wire        [12-1:0]        max_val_i,
    input  wire        [STEP_BITS-1:0] stepsize_i,
    input  wire        [12-1:0]        signal_i,
    input  wire        [1:0]           railed_i, // 0th bit: lower rail, 1st 
                                                 // bit: upper rail
    input  wire                        hold_i,
    output wire                        hold_o,
    output reg                         clear_o,
    output wire signed [14-1:0]        signal_o
);

// Are we locked?
reg locked_f;
always @(posedge clk_i) begin
    if (((min_val_i < signal_i) && (signal_i < max_val_i)) || !on_i) begin
        locked_f <= 1'b1;
        clear_o <= 1'b0;
    end else begin
        locked_f <= 1'b0;
        if (locked_f && (railed_i[0] || railed_i[1]))
            /* if we just became unlocked and we're railed, send the signal to
              reset the loop filter integrators
            */
            clear_o <= 1'b1;
        else
            clear_o <= 1'b0;
    end
end

assign hold_o = (on_i && (!locked_f));

// State machine definitions
localparam ZERO      = 2'b00;
localparam GOINGUP   = 2'b01;
localparam GOINGDOWN = 2'b10;

// State
reg        [1:0]         state_f;
reg signed [14+STEPSR:0] current_val_f;
reg signed [14+STEPSR:0] sweep_amplitude_f;

// State machine
always @(posedge clk_i) begin
    if (!on_i) begin // relock off
        current_val_f <= {14+STEPSR+1{1'b0}};
        sweep_amplitude_f <= {14+STEPSR+1{1'b0}};
        state_f <= ZERO;
    end else begin
        if (!hold_i) begin
            if (state_f == GOINGUP)
                current_val_f <= current_val_f + $signed(stepsize_i);
            else if (state_f == GOINGDOWN)
                current_val_f <= current_val_f - $signed(stepsize_i);
            else
                current_val_f <= {14+STEPSR+1{1'b0}};
            
            if (locked_f) begin // if we're locked, go towards zero
                sweep_amplitude_f <= {14+STEPSR+1{1'b0}};
                if (current_val_f > $signed(stepsize_i))
                    state_f <= GOINGDOWN;
                else if (current_val_f < -$signed(stepsize_i))
                    state_f <= GOINGUP;
                else
                    state_f <= ZERO;
            end else begin // otherwise, implement a sweep of increasing amplitude
                if (state_f == ZERO) begin
                    state_f <= GOINGUP;
                end else if ((current_val_f > sweep_amplitude_f) || railed_i[1]) begin
                    state_f <= GOINGDOWN;
                    if (state_f == GOINGUP) begin // increase the sweep amplitude every time we transition from GOINGUP to GOINGDOWN
                        if (sweep_amplitude_f == {14+STEPSR+1{1'b0}})
                            sweep_amplitude_f <= $signed(stepsize_i << 8); // start amplitude = 256*stepsize
                        else if (sweep_amplitude_f < (14'b01111111111111 << STEPSR))
                            sweep_amplitude_f <= (sweep_amplitude_f <<< 1); // double sweep amplitude
                    end
                end else if ((current_val_f < -sweep_amplitude_f) || railed_i[0]) begin
                    state_f <= GOINGUP;
                end
            end
        end
    end
end

// saturation
assign signal_o = (^current_val_f[14+STEPSR:14+STEPSR-1])?
                  {current_val_f[14+STEPSR], {13{~current_val_f[14+STEPSR]}}}:
                  current_val_f[14+STEPSR-1:STEPSR];

endmodule
