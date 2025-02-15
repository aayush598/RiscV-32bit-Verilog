`timescale 1ns / 1ps

// ----------------------------------------------------
// Testbench: Register File
// Description: Verifies register file read/write operations
//              while ensuring x0 remains zero.
// ----------------------------------------------------

module tb_register_file;

    // Testbench Signals
    reg clk;
    reg reset;
    reg reg_write;
    reg [4:0] rs1, rs2, rd;
    reg [31:0] write_data;
    wire [31:0] read_data1, read_data2;

    // Clock Generation (50 MHz = 20ns period)
    always #10 clk = ~clk;

    // Instantiate the Register File
    register_file uut (
        .clk(clk),
        .reset(reset),
        .reg_write(reg_write),
        .rs1(rs1),
        .rs2(rs2),
        .rd(rd),
        .write_data(write_data),
        .read_data1(read_data1),
        .read_data2(read_data2)
    );

    // Test Sequence
    initial begin
        // VCD Waveform Dump Setup
        $dumpfile("dump.vcd");  // Creates the waveform file
        $dumpvars(0, tb_register_file);  // Dumps all signals
        
        // Initialize Signals
        clk = 0;
        reset = 1;
        reg_write = 0;
        rd = 5'b00000;  // No writes initially
        write_data = 32'h00000000;
        rs1 = 5'b00000;
        rs2 = 5'b00000;

        #15 reset = 0;  // Deassert reset
        
        // Write data to register x5
        #10 reg_write = 1; rd = 5'b00101; write_data = 32'hDEADBEEF;
        
        // Write data to register x10
        #20 rd = 5'b01010; write_data = 32'hCAFEBABE;
        
        // Read from x5 and x10
        #20 reg_write = 0; rs1 = 5'b00101; rs2 = 5'b01010;
        
        // Write to register x0 (should remain zero)
        #20 reg_write = 1; rd = 5'b00000; write_data = 32'hFFFFFFFF;
        
        // Read x0 to check it is still zero
        #20 reg_write = 0; rs1 = 5'b00000;
        
        #50 $finish;
    end

    // Monitor Values
    initial begin
        $monitor("Time=%0t | clk=%b | reset=%b | reg_write=%b | rs1=%d, rs2=%d | rd=%d | write_data=%h | read_data1=%h | read_data2=%h", 
                 $time, clk, reset, reg_write, rs1, rs2, rd, write_data, read_data1, read_data2);
    end

endmodule
