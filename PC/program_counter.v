`timescale 1ns / 1ps

// -----------------------------------------------
// Module: Program Counter (PC)
// Description: Stores and updates the instruction address
// Author: [Your Name]
// Date: [Date]
// -----------------------------------------------

module program_counter (
    input wire clk,        // Clock signal
    input wire reset,      // Active high reset
    input wire [31:0] pc_in,  // Next PC value
    output reg [31:0] pc_out  // Current PC value
);

// Synchronous PC update with reset
always @(posedge clk or posedge reset) begin
    if (reset) 
        pc_out <= 32'h00000000;  // Reset PC to 0
    else 
        pc_out <= pc_in;  // Update PC with next address
end

endmodule
