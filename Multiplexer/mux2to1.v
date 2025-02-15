`timescale 1ns / 1ps

// -----------------------------------------------
// Module: 2-to-1 Multiplexer
// Description: Selects one of two inputs based on a control signal
// Author: [Your Name]
// Date: [Date]
// -----------------------------------------------

module mux2to1 (
    input wire [31:0] in0,  // First input
    input wire [31:0] in1,  // Second input
    input wire sel,         // Select signal
    output wire [31:0] out  // Output based on sel
);

// Multiplexer logic
assign out = (sel) ? in1 : in0;

endmodule
