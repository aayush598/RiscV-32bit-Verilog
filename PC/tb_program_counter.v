`timescale 1ns / 1ps

module tb_program_counter;

    // Testbench Signals
    reg clk;
    reg reset;
    reg [31:0] pc_in;
    wire [31:0] pc_out;

    // Clock Generation (50 MHz = 20ns period)
    always #10 clk = ~clk;

    // Instantiate the Program Counter
    program_counter uut (
        .clk(clk),
        .reset(reset),
        .pc_in(pc_in),
        .pc_out(pc_out)
    );

    // Test Sequence
    initial begin
        // VCD Waveform Dump Setup
        $dumpfile("dump.vcd");  // Creates the waveform file
        $dumpvars(0, tb_program_counter);  // Dumps all signals
        
        // Initialize Signals
        clk = 0;
        reset = 1;
        pc_in = 32'h00000000;  // Start from address 0
        
        #15 reset = 0;  
        #10 pc_in = 32'h00000004;  // Expect `pc_out = 0` at this time
        #20 pc_in = 32'h00000008;  // Expect `pc_out = 4` at this time
        #20 pc_in = 32'h0000000C;  // Expect `pc_out = 8` at this time
        #20 pc_in = 32'h00000100;  // Expect `pc_out = 12` at this time

        #50 $finish;
    end

    // Monitor Values
    initial begin
        $monitor("Time=%0t | clk=%b | reset=%b | pc_in=%h | pc_out=%h", 
                 $time, clk, reset, pc_in, pc_out);
    end

endmodule
