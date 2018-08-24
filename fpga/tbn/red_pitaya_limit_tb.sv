/**
 * Copyright (c) 2018, Fabian Schmid
 *
 * All rights reserved.
 *
 * Testbench for the output limiter.
 *
 * Only Channel A is tested, since the two channels use identical modules.
 */
`timescale 1ns / 1ps

module limit_tb #(
    // time periods
    realtime TP = 8.0ns // 125MHz
);

////////////////////////////////////////////////////////////////////////////////
// signal generation
////////////////////////////////////////////////////////////////////////////////

logic clk ;
logic rstn;

// ADC clock
initial clk = 1'b0;
always #(TP/2) clk = ~clk;

// ADC reset
initial begin
    rstn = 1'b0;
    repeat(4) @(posedge clk);
    rstn = 1'b1;
end

////////////////////////////////////////////////////////////////////////////////
// test sequence
////////////////////////////////////////////////////////////////////////////////

logic [14-1:0] dat_a_in;
logic [14-1:0] dat_a_out;
logic [   1:0] dat_a_railed_out;

logic [32-1:0] sys_addr;
logic [32-1:0] sys_wdata;
logic          sys_wen;
logic          sys_ren;
logic [32-1:0] sys_rdata;
logic           sys_err;
logic           sys_ack;

parameter UPPER_LIMIT = 14'd1000;
parameter UPPER_LIMIT_2 = 14'd0;
parameter LOWER_LIMIT = -14'd1000;

initial begin
    dat_a_in <= 14'd0 ;

    wait (rstn)
    repeat(10) @(posedge clk);
    // limiter settings
    bus.write(32'h0, LOWER_LIMIT); // Channel A min
    bus.write(32'h4, UPPER_LIMIT); // Channel A max
    repeat(10) @(posedge clk);

    dat_a_in <= 14'd2000; // exceed upper limit
    repeat(10) @(posedge clk);
    assert (dat_a_out == UPPER_LIMIT)
        else $error("Failed upper limit test.");
    assert (dat_a_railed_out == 2'b10)
        else $error("Failed upper limit railed output test.");

    dat_a_in <= -14'd2000; // exceed lower limit
    repeat(10) @(posedge clk);
    assert (dat_a_out == LOWER_LIMIT)
        else $error("Failed lower limit test.");
    assert (dat_a_railed_out == 2'b01)
        else $error("Failed lower limit railed output test.");

    dat_a_in <= 14'd500; // output within limits
    repeat(10) @(posedge clk);
    assert (dat_a_out == dat_a_in)
        else $error("Failed input within limits test.");
    assert (dat_a_railed_out == 2'b00)
        else $error("Failed input within limits railed output test.");

    bus.write(32'h4, UPPER_LIMIT_2);  // set upper limit below current value
    repeat(10) @(posedge clk);
    assert (dat_a_out == UPPER_LIMIT_2)
        else $error("Failed upper limit below current value test.");
    assert (dat_a_railed_out == 2'b10)
        else $error("Failed upper limit below current value railed output test.");

    $finish;
end

////////////////////////////////////////////////////////////////////////////////
// module instances
////////////////////////////////////////////////////////////////////////////////

sys_bus_model bus (
    // system signals
    .clk       (clk      ),
    .rstn      (rstn     ),
    // bus protocol signals
    .sys_addr  (sys_addr ),
    .sys_wdata (sys_wdata),
    .sys_wen   (sys_wen  ),
    .sys_ren   (sys_ren  ),
    .sys_rdata (sys_rdata),
    .sys_err   (sys_err  ),
    .sys_ack   (sys_ack  )
);

red_pitaya_limit limit(
    // signals
    .clk_i          (clk             ),
    .rstn_i         (rstn            ),
    .dat_a_i        (dat_a_in        ),
    .dat_a_o        (dat_a_out       ),
    .dat_a_railed_o (dat_a_railed_out),
    // System bus
    .sys_addr       (sys_addr        ),
    .sys_wdata      (sys_wdata       ),
    .sys_wen        (sys_wen         ),
    .sys_ren        (sys_ren         ),
    .sys_rdata      (sys_rdata       ),
    .sys_err        (sys_err         ),
    .sys_ack        (sys_ack         )
);

////////////////////////////////////////////////////////////////////////////////
// waveforms
////////////////////////////////////////////////////////////////////////////////

initial begin
  $dumpfile("limit_tb.vcd");
  $dumpvars(0, limit_tb);
end

endmodule: limit_tb
