`timescale 1ns / 1ps

module tb_immediate_generator;

    // Testbench Signals
    reg [31:0] instruction;
    wire [31:0] immediate;

    // Instantiate Immediate Generator
    immediate_generator uut (
        .instruction(instruction),
        .immediate(immediate)
    );

    // VCD Dump Setup
    initial begin
        $dumpfile("immediate_generator.vcd");
        $dumpvars(0, tb_immediate_generator);
    end

    // Test Sequence
    initial begin
        $monitor("Time=%0t | instruction=%h | immediate=%h", 
                 $time, instruction, immediate);
        
        // I-Type (ADDI)
        instruction = 32'b000000000000_00000_000_00000_0010011; #10;  // ADDI with immediate 0
        instruction = 32'b111111111111_00000_000_00000_0010011; #10;  // ADDI with immediate -1 (sign-extended)

        // Load Instruction (LW)
        instruction = 32'b000000000100_00000_010_00000_0000011; #10;  // LW with immediate 4

        // Store Instruction (SW)
        instruction = 32'b0000000_00001_00010_010_00011_0100011; #10; // SW with immediate 3
        
        // Branch Instruction (BEQ)
        instruction = 32'b0000000_00001_00010_000_0000001_1100011; #10; // BEQ with branch offset 1

        // JAL Instruction (J-Type)
        instruction = 32'b00000000000000000001_00000_1101111; #10;  // JAL with offset 1

        // JALR Instruction (I-Type)
        instruction = 32'b000000000101_00000_000_00000_1100111; #10;  // JALR with immediate 5

        // LUI Instruction (U-Type)
        instruction = 32'b00000000000000000000_00000_0110111; #10;  // LUI

        // AUIPC Instruction (U-Type)
        instruction = 32'b00000000000000000000_00000_0010111; #10;  // AUIPC
        
        #50 $finish;
    end

endmodule
