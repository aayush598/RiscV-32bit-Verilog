`timescale 1ns / 1ps

// ------------------------------------------------------
// Module: Instruction Memory
// Description: Stores and provides instruction based on the given address
// Author: [Your Name]
// Date: [Date]
// ------------------------------------------------------

module instruction_memory (
    input wire [31:0] addr,     // Address from Program Counter (PC)
    output wire [31:0] instr    // Instruction at given address
);

// Instruction Memory (64 words of 32-bit each)
reg [31:0] memory [0:63];

// Initialize the instruction memory with some values
initial begin
    $readmemh("instructions.mem", memory); // Load instructions from file
end

// Fetch instruction based on address (word-aligned)
assign instr = memory[addr[7:2]]; // Shifting ensures word alignment

endmodule
