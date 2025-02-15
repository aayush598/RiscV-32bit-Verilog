`timescale 1ns / 1ps

// ----------------------------------------------------
// Module: Register File
// Description: Implements a set of 32 registers (x0 - x31)
//              with two read ports and one write port.
// Author: [Your Name]
// Date: [Date]
// ----------------------------------------------------

module register_file (
    input wire clk,             // Clock signal
    input wire reset,           // Active high reset
    input wire reg_write,       // Write enable signal
    input wire [4:0] rs1,       // Read register 1 address
    input wire [4:0] rs2,       // Read register 2 address
    input wire [4:0] rd,        // Write register address
    input wire [31:0] write_data, // Data to be written
    output wire [31:0] read_data1, // Read data from rs1
    output wire [31:0] read_data2  // Read data from rs2
);

    // 32 registers (x0 - x31) initialized to zero
    reg [31:0] registers [0:31];

    integer i;

    // Read ports (Asynchronous)
    assign read_data1 = registers[rs1];
    assign read_data2 = registers[rs2];

    // Write operation (Synchronous, x0 remains zero)
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 32; i = i + 1) begin
                registers[i] <= 32'h00000000;  // Reset all registers to 0
            end
        end
        else if (reg_write && rd != 5'b00000) begin
            registers[rd] <= write_data;  // Write data to rd (except x0)
        end
    end

endmodule
