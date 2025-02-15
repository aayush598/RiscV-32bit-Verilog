`timescale 1ns / 1ps

// -----------------------------------------------
// Module: PCplus4
// Description: Adds 4 to the PC value to generate the next instruction address
// Author: [Your Name]
// Date: [Date]
// -----------------------------------------------

module pc_plus4 (
    input wire [31:0] pc_in,   // Current PC value
    output wire [31:0] pc_out  // Next PC value (PC + 4)
);

// Add 4 to PC input
assign pc_out = pc_in + 4;

endmodule
