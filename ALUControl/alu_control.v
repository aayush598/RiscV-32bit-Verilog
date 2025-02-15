`timescale 1ns / 1ps

// --------------------------------------------------
// Module: ALU Control
// Description: Determines ALU operation based on 
//              function codes from the instruction
// Author: [Your Name]
// Date: [Date]
// --------------------------------------------------

module alu_control (
    input wire [1:0] alu_op,        // ALU Operation Control from Control Unit
    input wire [3:0] funct,         // Function code from Instruction
    output reg [3:0] alu_control_out // Control signal for ALU
);

// ALU Control Logic
always @(*) begin
    case (alu_op)
        2'b00: alu_control_out = 4'b0010;  // Load/Store (Addition)
        2'b01: alu_control_out = 4'b0110;  // Branch (Subtraction)
        2'b10: begin
            case (funct)
                4'b0000: alu_control_out = 4'b0010;  // ADD
                4'b1000: alu_control_out = 4'b0110;  // SUB
                4'b0111: alu_control_out = 4'b0000;  // AND
                4'b0110: alu_control_out = 4'b0001;  // OR
                4'b0100: alu_control_out = 4'b1100;  // XOR
                default: alu_control_out = 4'b1111;  // Invalid Operation
            endcase
        end
        default: alu_control_out = 4'b1111; // Default case (Invalid)
    endcase
end

endmodule
