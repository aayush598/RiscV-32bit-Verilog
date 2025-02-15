`timescale 1ns / 1ps

// -----------------------------------------------
// Module: Immediate Generator
// Description: Extracts and sign-extends immediate values from instruction
// Author: [Your Name]
// Date: [Date]
// -----------------------------------------------

module immediate_generator (
    input wire [31:0] instruction,  // 32-bit instruction
    output reg [31:0] immediate     // Sign-extended immediate output
);

// Extracts the immediate field based on instruction format
always @(*) begin
    case (instruction[6:0])  // Check opcode field (bits 6:0)
        7'b0010011: // I-type (ADDI, ANDI, ORI, etc.)
            immediate = {{20{instruction[31]}}, instruction[31:20]};

        7'b0000011: // Load (LW, LB, LH, etc.)
            immediate = {{20{instruction[31]}}, instruction[31:20]};

        7'b0100011: // S-type (Store instructions SW, SB, SH)
            immediate = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};

        7'b1100011: // B-type (Branch instructions BEQ, BNE, etc.)
            immediate = {{19{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0};

        7'b1101111: // J-type (JAL)
            immediate = {{11{instruction[31]}}, instruction[31], instruction[19:12], instruction[20], instruction[30:21], 1'b0};

        7'b1100111: // I-type (JALR)
            immediate = {{20{instruction[31]}}, instruction[31:20]};

        7'b0110111, // U-type (LUI)
        7'b0010111: // U-type (AUIPC)
            immediate = {instruction[31:12], 12'b0}; // Upper immediate
        
        default: 
            immediate = 32'h00000000; // Default case for unknown instructions
    endcase
end

endmodule
