`timescale 1ns / 1ps

// --------------------------------------------------
// Testbench: ALU Control
// Description: Verifies ALU Control logic based on 
//              ALUOp and funct values
// Author: [Your Name]
// Date: [Date]
// --------------------------------------------------

module tb_alu_control;

    // Testbench Signals
    reg [1:0] alu_op;
    reg [3:0] funct;
    wire [3:0] alu_control_out;

    // Instantiate the ALU Control Module
    alu_control uut (
        .alu_op(alu_op),
        .funct(funct),
        .alu_control_out(alu_control_out)
    );

    // Test Sequence
    initial begin
        // VCD Waveform Dump Setup
        $dumpfile("alu_control.vcd");  // Creates the waveform file
        $dumpvars(0, tb_alu_control);  // Dumps all signals

        // Test Case 1: Load/Store (Addition)
        alu_op = 2'b00; funct = 4'b0000; #10;
        $display("Load/Store: alu_op=%b, funct=%b -> alu_control_out=%b", alu_op, funct, alu_control_out);

        // Test Case 2: Branch (Subtraction)
        alu_op = 2'b01; funct = 4'b0000; #10;
        $display("Branch: alu_op=%b, funct=%b -> alu_control_out=%b", alu_op, funct, alu_control_out);

        // Test Case 3: R-Type ADD
        alu_op = 2'b10; funct = 4'b0000; #10;
        $display("R-Type ADD: alu_op=%b, funct=%b -> alu_control_out=%b", alu_op, funct, alu_control_out);

        // Test Case 4: R-Type SUB
        alu_op = 2'b10; funct = 4'b1000; #10;
        $display("R-Type SUB: alu_op=%b, funct=%b -> alu_control_out=%b", alu_op, funct, alu_control_out);

        // Test Case 5: R-Type AND
        alu_op = 2'b10; funct = 4'b0111; #10;
        $display("R-Type AND: alu_op=%b, funct=%b -> alu_control_out=%b", alu_op, funct, alu_control_out);

        // Test Case 6: R-Type OR
        alu_op = 2'b10; funct = 4'b0110; #10;
        $display("R-Type OR: alu_op=%b, funct=%b -> alu_control_out=%b", alu_op, funct, alu_control_out);

        // Test Case 7: R-Type XOR
        alu_op = 2'b10; funct = 4'b0100; #10;
        $display("R-Type XOR: alu_op=%b, funct=%b -> alu_control_out=%b", alu_op, funct, alu_control_out);

        // Test Case 8: Invalid Operation
        alu_op = 2'b10; funct = 4'b1111; #10;
        $display("Invalid: alu_op=%b, funct=%b -> alu_control_out=%b", alu_op, funct, alu_control_out);

        // End Simulation
        #20 $finish;
    end

endmodule
