`timescale 1ns / 1ps

module tb_pc_plus4;

    // Testbench Signals
    reg [31:0] pc_in;
    wire [31:0] pc_out;

    // Instantiate the PCplus4 module
    pc_plus4 uut (
        .pc_in(pc_in),
        .pc_out(pc_out)
    );

    // Test Sequence
    initial begin
        // VCD Waveform Dump Setup
        $dumpfile("pc_plus4.vcd");  // Creates the waveform file
        $dumpvars(0, tb_pc_plus4);  // Dumps all signals
        
        // Test Cases
        pc_in = 32'h00000000; #10;  // Expect pc_out = 4
        pc_in = 32'h00000004; #10;  // Expect pc_out = 8
        pc_in = 32'h00000008; #10;  // Expect pc_out = 12
        pc_in = 32'h0000000C; #10;  // Expect pc_out = 16
        pc_in = 32'h00000100; #10;  // Expect pc_out = 0x00000104
        pc_in = 32'hFFFFFFFC; #10;  // Expect pc_out = 0x00000000 (rollover)

        #50 $finish; // End Simulation
    end

    // Monitor Values
    initial begin
        $monitor("Time=%0t | pc_in=%h | pc_out=%h", $time, pc_in, pc_out);
    end

endmodule
