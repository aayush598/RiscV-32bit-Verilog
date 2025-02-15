`timescale 1ns / 1ps

module tb_data_memory;

    // Testbench Signals
    reg clk;
    reg mem_write;
    reg mem_read;
    reg [9:0] address;
    reg [31:0] write_data;
    wire [31:0] read_data;

    // Instantiate the Data Memory Module
    data_memory uut (
        .clk(clk),
        .mem_write(mem_write),
        .mem_read(mem_read),
        .address(address),
        .write_data(write_data),
        .read_data(read_data)
    );

    // Clock Generation (50 MHz = 20ns period)
    always #10 clk = ~clk;

    // Test Sequence
    initial begin
        // VCD Waveform Dump Setup
        $dumpfile("dump.vcd");  // Creates waveform file
        $dumpvars(0, tb_data_memory);  // Dumps all signals
        
        // Initialize Signals
        clk = 0;
        mem_write = 0;
        mem_read = 0;
        address = 10'b0000000000;
        write_data = 32'h00000000;

        #15; // Small delay before starting

        // WRITE OPERATION
        address = 10'd5;         // Target address = 5
        write_data = 32'hDEADBEEF; // Data to write
        mem_write = 1;           // Enable write
        #20;
        mem_write = 0;           // Disable write

        // READ OPERATION
        mem_read = 1;            // Enable read
        #20;
        mem_read = 0;            // Disable read

        // Another WRITE OPERATION
        address = 10'd10;         // Target address = 10
        write_data = 32'h12345678;
        mem_write = 1;
        #20;
        mem_write = 0;

        // READ BACK NEWLY WRITTEN DATA
        address = 10'd10;
        mem_read = 1;
        #20;
        mem_read = 0;

        // Finish Simulation
        #50 $finish;
    end

    // Monitor Values
    initial begin
        $monitor("Time=%0t | clk=%b | addr=%d | mem_write=%b | write_data=%h | mem_read=%b | read_data=%h",
                 $time, clk, address, mem_write, write_data, mem_read, read_data);
    end

endmodule
