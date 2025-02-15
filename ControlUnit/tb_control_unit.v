`timescale 1ns / 1ps

module tb_control_unit;

    // Testbench Signals
    reg [6:0] opcode;
    wire regWrite, aluSrc, memToReg, memRead, memWrite, branch;
    wire [1:0] aluOp;

    // Instantiate the Control Unit
    control_unit uut (
        .opcode(opcode),
        .regWrite(regWrite),
        .aluSrc(aluSrc),
        .memToReg(memToReg),
        .memRead(memRead),
        .memWrite(memWrite),
        .branch(branch),
        .aluOp(aluOp)
    );

    // Test Sequence
    initial begin
        // VCD Waveform Dump Setup
        $dumpfile("control_unit.vcd");
        $dumpvars(0, tb_control_unit);
        
        // Test R-Type (ADD, SUB, etc.)
        opcode = 7'b0110011; #10;
        $display("R-Type | regWrite=%b, aluSrc=%b, memToReg=%b, memRead=%b, memWrite=%b, branch=%b, aluOp=%b", 
                 regWrite, aluSrc, memToReg, memRead, memWrite, branch, aluOp);

        // Test Load (LW)
        opcode = 7'b0000011; #10;
        $display("LW     | regWrite=%b, aluSrc=%b, memToReg=%b, memRead=%b, memWrite=%b, branch=%b, aluOp=%b", 
                 regWrite, aluSrc, memToReg, memRead, memWrite, branch, aluOp);

        // Test Store (SW)
        opcode = 7'b0100011; #10;
        $display("SW     | regWrite=%b, aluSrc=%b, memToReg=%b, memRead=%b, memWrite=%b, branch=%b, aluOp=%b", 
                 regWrite, aluSrc, memToReg, memRead, memWrite, branch, aluOp);

        // Test Branch (BEQ)
        opcode = 7'b1100011; #10;
        $display("BEQ    | regWrite=%b, aluSrc=%b, memToReg=%b, memRead=%b, memWrite=%b, branch=%b, aluOp=%b", 
                 regWrite, aluSrc, memToReg, memRead, memWrite, branch, aluOp);

        // Test Default / NOP
        opcode = 7'b1111111; #10;
        $display("NOP    | regWrite=%b, aluSrc=%b, memToReg=%b, memRead=%b, memWrite=%b, branch=%b, aluOp=%b", 
                 regWrite, aluSrc, memToReg, memRead, memWrite, branch, aluOp);

        // Finish Simulation
        #20;
        $finish;
    end

endmodule
