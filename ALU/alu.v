`timescale 1ns / 1ps

// -----------------------------------------------
// Module: Arithmetic Logic Unit (ALU)
// Description: Performs arithmetic and logical operations
// Author: [Your Name]
// Date: [Date]
// -----------------------------------------------

module alu (
    input wire [31:0] operand_a,  // First input operand
    input wire [31:0] operand_b,  // Second input operand
    input wire [3:0] alu_control, // Control signal for ALU operation
    output reg [31:0] alu_result, // Output result of the ALU
    output reg zero_flag          // Zero flag (1 if result is zero)
);

// ALU Operations (4-bit control encoding)
localparam ADD  = 4'b0000;  // Addition
localparam SUB  = 4'b0001;  // Subtraction
localparam AND  = 4'b0010;  // Bitwise AND
localparam OR   = 4'b0011;  // Bitwise OR
localparam XOR  = 4'b0100;  // Bitwise XOR
localparam SLL  = 4'b0101;  // Logical Shift Left
localparam SRL  = 4'b0110;  // Logical Shift Right
localparam SRA  = 4'b0111;  // Arithmetic Shift Right
localparam SLT  = 4'b1000;  // Set Less Than (signed)
localparam SLTU = 4'b1001;  // Set Less Than (unsigned)

// ALU Computation
always @(*) begin
    case (alu_control)
        ADD:  alu_result = operand_a + operand_b;
        SUB:  alu_result = operand_a - operand_b;
        AND:  alu_result = operand_a & operand_b;
        OR:   alu_result = operand_a | operand_b;
        XOR:  alu_result = operand_a ^ operand_b;
        SLL:  alu_result = operand_a << operand_b[4:0];  // Shift by lower 5 bits
        SRL:  alu_result = operand_a >> operand_b[4:0];
        SRA:  alu_result = $signed(operand_a) >>> operand_b[4:0]; // Arithmetic right shift
        SLT:  alu_result = ($signed(operand_a) < $signed(operand_b)) ? 32'b1 : 32'b0;
        SLTU: alu_result = (operand_a < operand_b) ? 32'b1 : 32'b0;
        default: alu_result = 32'h00000000;  // Default case (NOP)
    endcase
    // Set Zero Flag
    zero_flag = (alu_result == 32'h00000000) ? 1'b1 : 1'b0;
end

endmodule