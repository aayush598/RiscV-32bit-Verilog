`timescale 1ns / 1ps

// -----------------------------------------------
// Module: Control Unit
// Description: Generates control signals based on instruction opcode
// Author: [Your Name]
// Date: [Date]
// -----------------------------------------------

module control_unit (
    input wire [6:0] opcode,  // 7-bit opcode field from instruction
    output reg regWrite,      // Register write enable
    output reg aluSrc,        // ALU source select
    output reg memToReg,      // Memory to register select
    output reg memRead,       // Memory read enable
    output reg memWrite,      // Memory write enable
    output reg branch,        // Branch instruction flag
    output reg [1:0] aluOp    // ALU operation control
);

always @(*) begin
    // Default control signals
    regWrite  = 0;
    aluSrc    = 0;
    memToReg  = 0;
    memRead   = 0;
    memWrite  = 0;
    branch    = 0;
    aluOp     = 2'b00;

    case (opcode)
        7'b0110011: begin // R-Type (ADD, SUB, AND, OR, etc.)
            regWrite  = 1;
            aluSrc    = 0;
            memToReg  = 0;
            memRead   = 0;
            memWrite  = 0;
            branch    = 0;
            aluOp     = 2'b10;
        end
        7'b0000011: begin // Load (LW)
            regWrite  = 1;
            aluSrc    = 1;
            memToReg  = 1;
            memRead   = 1;
            memWrite  = 0;
            branch    = 0;
            aluOp     = 2'b00;
        end
        7'b0100011: begin // Store (SW)
            regWrite  = 0;
            aluSrc    = 1;
            memToReg  = 0;
            memRead   = 0;
            memWrite  = 1;
            branch    = 0;
            aluOp     = 2'b00;
        end
        7'b1100011: begin // Branch (BEQ)
            regWrite  = 0;
            aluSrc    = 0;
            memToReg  = 0;
            memRead   = 0;
            memWrite  = 0;
            branch    = 1;
            aluOp     = 2'b01;
        end
        default: begin // Default case (NOP or unsupported opcode)
            regWrite  = 0;
            aluSrc    = 0;
            memToReg  = 0;
            memRead   = 0;
            memWrite  = 0;
            branch    = 0;
            aluOp     = 2'b00;
        end
    endcase
end

endmodule
