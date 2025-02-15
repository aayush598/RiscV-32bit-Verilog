`timescale 1ns / 1ps

// -----------------------------------------------
// Module: Data Memory
// Description: Implements memory for load/store operations
// Author: [Your Name]
// Date: [Date]
// -----------------------------------------------

module data_memory (
    input wire clk,                // Clock signal
    input wire mem_write,          // Memory write enable
    input wire mem_read,           // Memory read enable
    input wire [9:0] address,      // Memory address (10-bit: 1024 words)
    input wire [31:0] write_data,  // Data to be written
    output reg [31:0] read_data    // Data read from memory
);

    // Define 1024 x 32-bit memory
    reg [31:0] memory [0:1023];

    // Read Operation (Asynchronous)
    always @(*) begin
        if (mem_read)
            read_data = memory[address];
        else
            read_data = 32'h00000000;
    end

    // Write Operation (Synchronous)
    always @(posedge clk) begin
        if (mem_write)
            memory[address] <= write_data;
    end

endmodule
