`timescale 1ns / 1ps

module tb_alu;

    // Testbench Signals
    reg [31:0] operand_a;
    reg [31:0] operand_b;
    reg [3:0] alu_control;
    wire [31:0] alu_result;
    wire zero_flag;

    // Instantiate the ALU
    alu uut (
        .operand_a(operand_a),
        .operand_b(operand_b),
        .alu_control(alu_control),
        .alu_result(alu_result),
        .zero_flag(zero_flag)
    );

    // Test Sequence
    initial begin
        // VCD Waveform Dump Setup
        $dumpfile("alu_dump.vcd");  // Creates the waveform file
        $dumpvars(0, tb_alu);  // Dumps all signals
        
        // Test Cases
        operand_a = 32'h00000005;  operand_b = 32'h00000003;  alu_control = 4'b0000; #10; // ADD
        operand_a = 32'h00000005;  operand_b = 32'h00000003;  alu_control = 4'b0001; #10; // SUB
        operand_a = 32'hF0000000;  operand_b = 32'h00000001;  alu_control = 4'b0010; #10; // AND
        operand_a = 32'hF0000000;  operand_b = 32'h00000001;  alu_control = 4'b0011; #10; // OR
        operand_a = 32'h00000005;  operand_b = 32'h00000003;  alu_control = 4'b0100; #10; // XOR
        operand_a = 32'h00000001;  operand_b = 32'h00000002;  alu_control = 4'b0101; #10; // SLL
        operand_a = 32'h00000010;  operand_b = 32'h00000002;  alu_control = 4'b0110; #10; // SRL
        operand_a = 32'hF0000000;  operand_b = 32'h00000003;  alu_control = 4'b0111; #10; // SRA
        operand_a = 32'h00000005;  operand_b = 32'h0000000A;  alu_control = 4'b1000; #10; // SLT
        operand_a = 32'hFFFFFFFF;  operand_b = 32'h00000001;  alu_control = 4'b1001; #10; // SLTU
        
        #50 $finish;
    end

    // Monitor Output
    initial begin
        $monitor("Time=%0t | ALU Control=%b | A=%h | B=%h | Result=%h | Zero=%b", 
                 $time, alu_control, operand_a, operand_b, alu_result, zero_flag);
    end

endmodule
