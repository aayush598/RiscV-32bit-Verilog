`timescale 1ns / 1ps

module tb_mux2to1;

    // Testbench Signals
    reg [31:0] in0, in1;  // Inputs
    reg sel;              // Select signal
    wire [31:0] out;      // Output

    // Instantiate the Multiplexer
    mux2to1 uut (
        .in0(in0),
        .in1(in1),
        .sel(sel),
        .out(out)
    );

    // Test Sequence
    initial begin
        // VCD Waveform Dump Setup
        $dumpfile("mux2to1.vcd");  // Creates waveform file
        $dumpvars(0, tb_mux2to1);  // Dumps all signals
        
        // Initialize Inputs
        in0 = 32'hAAAA_AAAA;  // Pattern 1
        in1 = 32'h5555_5555;  // Pattern 2
        sel = 0;  // Select input0

        #10 sel = 1;  // Expect output to be in1 (0x55555555)
        #10 sel = 0;  // Expect output to be in0 (0xAAAAAAAA)
        #10 in0 = 32'hFFFF_FFFF;  // Update in0, select is still 0
        #10 sel = 1;  // Select input1 again
        #10 in1 = 32'h1234_5678;  // Update in1, select is still 1
        #10 sel = 0;  // Select input0

        #20 $finish;  // End simulation
    end

    // Monitor Values
    initial begin
        $monitor("Time=%0t | in0=%h | in1=%h | sel=%b | out=%h", 
                 $time, in0, in1, sel, out);
    end

endmodule
