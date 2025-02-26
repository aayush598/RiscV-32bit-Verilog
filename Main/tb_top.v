`timescale 1ns / 1ps

module tb_top();

    // Testbench Signals
    reg clk;
    reg reset;

    // Instantiate the DUT (Device Under Test)
    top uut (
        .clk(clk),
        .reset(reset)
    );

    // Clock Generation (50 MHz => 20ns period)
    always #10 clk = ~clk;

    initial begin
        // Initialize signals
        clk = 0;
        reset = 1;

        // Hold reset for some cycles
        #10;
        reset = 0;

        // Run the simulation for some time
        #800;

        // Finish simulation
        $finish;
    end

    // Dump waveforms for GTKWave
    initial begin
        $dumpfile("waveform.vcd"); // Output waveform file
        $dumpvars(0, tb_top);      // Dump all variables in tb_top
    end

endmodule
