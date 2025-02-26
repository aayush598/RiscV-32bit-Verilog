/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Project Name  : RISC V 32 BIT CPU
%% File Name     : top.v
%% Author        : Aayush Gid
%%
%% Description   : This module represents the top-level design of a 32-bit RISC-V CPU.
%%                It integrates various functional blocks, including the instruction fetch,
%%                decode, execute, memory access, and write-back stages.
%%                The design follows a pipelined architecture for optimized performance.
%%
%%                The module ensures synchronous operation with a clock signal and utilizes
%%                a reset signal for initialization. It is structured to support different 
%%                instruction formats and efficiently handle data flow within the processor.
%%
%% Copyright     : © 2025 Aayush Gid. All rights reserved.
%%                Unauthorized copying or distribution of this code, via any medium,
%%                is strictly prohibited.
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Modification / Updation History
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Date          : 23-02-2025
%% Version       : 1.0
%% Change Desc   : Initial version of the top-level RISC-V CPU design.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/


`timescale 1ns / 1ps

/* verilator lint_off DECLFILENAME */
module Program_counter(clk, reset, PC_in, PC_out);

    input clk;
    input reset;
    input [31:0] PC_in;
    output reg [31:0] PC_out;

    always @(posedge clk or posedge reset)
        begin
            PC_out = (reset)? 32'h00000000 : PC_in;
        end

endmodule

module PC_Plus_4(clk, PC_in, PC_out);
    input clk;
    input [31:0] PC_in;
    output [31:0] PC_out;

    assign  PC_out =  PC_in + 4;
endmodule

module Adder(PC_in, Imm_in, Sum);
    input [31:0] PC_in;
    input [31:0] Imm_in;
    output [31:0] Sum;

    assign Sum= PC_in + Imm_in;
endmodule

module AND_Gate(A, B, o_Y);
    input A;
    input B;
    output o_Y;
    assign o_Y = A & B ;
endmodule

module mux(A, B, sel, Y);
    input [31:0] A;
    input [31:0] B;
    input wire sel;
    output reg [31:0] Y;
    always @(*)
        begin
            Y = (sel) ? B : A ;
        end
endmodule

module mux3(A, B, sel, Y);
    input [31:0] A;
    input [31:0] B;
    input wire sel;
    output reg [31:0] Y;
    always @(*)
        begin
            Y = (sel) ? B : A ;
        end
endmodule

module Instruction_Memory(clk, reset, PC_in, Instruction_out);
    input clk;
    input reset;
    input [4:0] PC_in;
    output reg [31:0] Instruction_out;
    
    reg [31:0] Memory[31:0];

    initial begin
        $readmemh("instructions.mem", Memory);
    end

    always @(posedge clk or posedge reset) begin
        Instruction_out <= (reset)? 32'h00000000 : Memory[PC_in];
    end
endmodule

module Control(i_Instruction, o_Branch, o_MemRead, o_MemtoReg, o_ALUOp, o_MemWrite, o_ALUSrc, o_RegWrite);
    input [6:0] i_Instruction;
    output reg o_Branch;
    output reg o_MemtoReg;
    output reg [1:0] o_ALUOp;
    output reg o_MemWrite;
    output reg o_ALUSrc;
    output reg o_RegWrite;
    output reg o_MemRead;

    always @(*)
    begin
        case (i_Instruction)
        7'b0110011 :                // Register
            begin
                o_Branch = 1'b0;
                o_MemRead = 1'b0;
                o_MemtoReg = 1'b0;
                o_ALUOp = 2'b10;
                o_MemWrite = 1'b0;
                o_ALUSrc = 1'b0;
                o_RegWrite = 1'b1;
            end
        7'b0000011 :                // Load
            begin
                o_Branch = 1'b0;
                o_MemRead = 1'b1;
                o_MemtoReg = 1'b1;
                o_ALUOp = 2'b00;
                o_MemWrite = 1'b0;
                o_ALUSrc = 1'b1;
                o_RegWrite = 1'b1;
            end
        7'b0100011 :                // Store
            begin
                o_Branch = 1'b0;
                o_MemRead = 1'b0;
                o_MemtoReg = 1'b0;
                o_ALUOp = 2'b00;
                o_MemWrite = 1'b1;
                o_ALUSrc = 1'b1;
                o_RegWrite = 1'b0;
            end
        7'b1100011 :                // Branch
            begin
                o_Branch = 1'b1;
                o_MemRead = 1'b0;
                o_MemtoReg = 1'b0;
                o_ALUOp = 2'b01;
                o_MemWrite = 1'b0;
                o_ALUSrc = 1'b0;
                o_RegWrite = 1'b0;
            end
        7'b0010011 :                // Arithmetic
            begin
                o_Branch = 1'b0;
                o_MemRead = 1'b0;
                o_MemtoReg = 1'b0;
                o_ALUOp = 2'b10;
                o_MemWrite = 1'b0;
                o_ALUSrc = 1'b1;
                o_RegWrite = 1'b1;
            end
        default:
            begin
                o_Branch   = 1'b0;
                o_MemRead  = 1'b0;
                o_MemtoReg = 1'b0;
                o_ALUOp    = 2'b00;
                o_MemWrite = 1'b0;
                o_ALUSrc   = 1'b0;
                o_RegWrite = 1'b0;
            end

    endcase

    end
endmodule

module ImmediateGenerator(clk, i_Instruction, o_Immediate);
    input clk;
    input [31:0] i_Instruction;
    output reg [31:0] o_Immediate;    

    initial begin
        o_Immediate = 32'd0;
    end
    always @(i_Instruction)
        begin
            case(i_Instruction[6:0])
                // I-type Instructions (Immediate Load, Arithmetic)
                7'b0010011, 7'b0000011: 
                    o_Immediate = {{20{i_Instruction[31]}}, i_Instruction[31:20]};

                // S-type Instructions (Store Operations)
                7'b0100011: 
                    o_Immediate = {{20{i_Instruction[31]}}, i_Instruction[31:25], i_Instruction[11:7]};
                    
                // B-type Instructions (Branch Operations)
                7'b1100011: 
                    o_Immediate = {{20{i_Instruction[31]}}, i_Instruction[7], i_Instruction[30:25], i_Instruction[11:8], 1'b0};
                
                default : o_Immediate = 32'h00000000;
            endcase
        end
endmodule

module ALU_Control(i_Func7, i_Func3, i_ALUOp, o_Operation);
    input i_Func7;
    input [2:0] i_Func3;
    input [1:0] i_ALUOp;
    output reg [3:0] o_Operation;

    always  @(*) begin
        case({i_ALUOp, i_Func7, i_Func3})
            6'b00_0_000: o_Operation = 4'b0010;

            // Branch operations
            6'b01_0_000: o_Operation = 4'b0110; // SUB (for BEQ)
            6'b01_0_001: o_Operation = 4'b0110; // BNE
            6'b01_0_100: o_Operation = 4'b1010; // BLT
            6'b01_0_101: o_Operation = 4'b1011; // BGE
            6'b01_0_110: o_Operation = 4'b1100; // BLTU
            6'b01_0_111: o_Operation = 4'b1101; // BGEU

            // R TYPE INSTRUCTIONS
            6'b10_0_000: o_Operation = 4'b0010; // ADD
            6'b10_1_000: o_Operation = 4'b0110; // SUB
            6'b10_0_111: o_Operation = 4'b0000; // AND
            6'b10_0_110: o_Operation = 4'b0001; // OR
            6'b10_0_100: o_Operation = 4'b1000; // XOR
            6'b10_0_001: o_Operation = 4'b0100; // SLL (Shift Left Logical)
            6'b10_0_101: o_Operation = 4'b0101; // SRL (Shift Right Logical)
            6'b10_1_101: o_Operation = 4'b0111; // SRA (Shift Right Arithmetic)
            6'b10_0_010: o_Operation = 4'b1010; // SLT (Set Less Than)
            6'b10_0_011: o_Operation = 4'b1011; // SLTU (Set Less Than Unsigned)

            // I TYPE INSTRUCTIONS
            6'b11_0_000: o_Operation = 4'b0010; // ADDI
            6'b11_0_111: o_Operation = 4'b0000; // ANDI
            6'b11_0_110: o_Operation = 4'b0001; // ORI
            6'b11_0_100: o_Operation = 4'b1000; // XORI
            6'b11_0_001: o_Operation = 4'b0100; // SLLI (Shift Left Logical Immediate)
            6'b11_0_101: o_Operation = 4'b0101; // SRLI (Shift Right Logical Immediate)
            6'b11_1_101: o_Operation = 4'b0111; // SRAI (Shift Right Arithmetic Immediate)
            6'b11_0_010: o_Operation = 4'b1010; // SLTI (Set Less Than Immediate)
            6'b11_0_011: o_Operation = 4'b1011; // SLTUI (Set Less Than Unsigned Immediate)

            // LOAD and STORE Instructions (Use ADD for address calculation)
            6'b00_0_010: o_Operation = 4'b0010; // LW (Load Word)
            6'b00_0_000: o_Operation = 4'b0010; // LB (Load Byte)
            6'b00_0_100: o_Operation = 4'b0010; // LBU (Load Byte Unsigned)
            6'b00_0_001: o_Operation = 4'b0010; // LH (Load Halfword)
            6'b00_0_101: o_Operation = 4'b0010; // LHU (Load Halfword Unsigned)
            6'b00_0_010: o_Operation = 4'b0010; // SW (Store Word)
            6'b00_0_000: o_Operation = 4'b0010; // SB (Store Byte)
            6'b00_0_001: o_Operation = 4'b0010; // SH (Store Halfword)

            default: o_Operation = 4'b1111;
        endcase
    end
endmodule

module ALU(A, B, i_Operation, o_ALU_Result, o_Zero);
    input [31:0] A;
    input [31:0] B;
    input [3:0] i_Operation;
    output reg [31:0] o_ALU_Result;
    output reg o_Zero;

    always @(*) begin
        case (i_Operation)
            4'b0000 : o_ALU_Result = A & B;   // AND
            4'b0001 : o_ALU_Result = A | B;   // OR
            4'b0010 : o_ALU_Result = A + B;   // ADD
            4'b0100 : o_ALU_Result = A << B;   // SLL (Shift Left Logical)
            4'b0101 : o_ALU_Result = A >> B;   // SRL (Shift Right Logical)
            4'b0110 : o_ALU_Result = A - B;   // SUB
            4'b0111 : o_ALU_Result = $signed(A) >>> B;   // SRA (Shift Right Arithmetic)
            4'b1000 : o_ALU_Result = (A ^ B); // XOR
            4'b1010 : o_ALU_Result = ($signed(A) < $signed(B)) ? 32'h00000001 : 32'h00000000; // SLT
            4'b1011 : o_ALU_Result = (A < B) ? 32'h00000001 : 32'h00000000; // SLTU

            4'b1111 : o_ALU_Result = 32'h00000000; // Zero output
        endcase


        o_Zero = (o_ALU_Result == 32'h00000000) ?  1'b1 : 1'b0;
    end
endmodule

module Data_Memory(i_MemWrite, i_MemRead, i_Address, i_WriteData, o_ReadData, clk);
    input wire clk;
    input wire i_MemWrite;
    input wire i_MemRead;
    input wire [4:0] i_Address;
    input wire [31:0] i_WriteData;
    output reg [31:0] o_ReadData;

    reg [31:0] memory [0:31];

    // Load memory from file
    initial begin
        $readmemh("DataMemory.mem", memory, 0, 31); // Explicit range for compatibility
    end

    // Read Operation (Asynchronous)
    always @(*) begin
        if (i_MemRead)
            o_ReadData = memory[i_Address];
        else
            o_ReadData = 32'h00000000;
    end

    // Write Operation (Synchronous)
    always @(posedge clk) begin
        if (i_MemWrite)
            memory[i_Address] <= i_WriteData;
    end

endmodule




module Register(clk, reset, i_RS1, i_RS2, i_RD1,i_WriteData, i_RegWrite ,o_ReadData1, o_ReadData2);
	input clk;
    input reset;
    input [4:0] i_RS1;
    input [4:0] i_RS2;
    input [4:0] i_RD1;
    input i_RegWrite;
    input [31:0] i_WriteData;
    output  [31:0] o_ReadData1;
    output  [31:0] o_ReadData2;
    // integer k;
    reg [31:0] Reg_Memory[0:31];

    // Read initial register values from register.mem
    initial begin
        $readmemh("register.mem", Reg_Memory);
    end

    assign o_ReadData1 = Reg_Memory[i_RS1];
    assign o_ReadData2 = Reg_Memory[i_RS2];
    
    always @(posedge clk or posedge reset)
    begin        
        if (reset) begin
            // for (k = 0; k < 32; k = k + 1) begin
            //     Reg_Memory[k] <= 32'h00000000; // Reset all registers
            // end
        end
        else if (i_RegWrite && i_RD1 != 5'b00000) begin
            Reg_Memory[i_RD1] <= i_WriteData; // Write data to register
        end
        $writememh("register.mem", Reg_Memory);
    end

    
endmodule
/* verilator lint_on DECLFILENAME */

module top(clk, reset);
    input clk;
    input reset;
    wire [31:0] PC_out, PCPlus4, SumOut, InstructionOut, Mux1Out, Mux2Out, ReadData1Out, ReadData2Out, ImmediateOut, ALUResult, DM_read, Mux3Out;
    wire Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite, zero, Select;
    wire [1:0] ALUOp ;
    wire [3:0] Control_out ;

    Program_counter PC(.clk(clk), .reset(reset), .PC_in(Mux1Out) , .PC_out(PC_out));
    PC_Plus_4 PC4 (.PC_in(PC_out), .PC_out(PCPlus4));
    Adder Add (.PC_in(PC_out), .Imm_in(ImmediateOut), .Sum(SumOut));
    AND_Gate AND (.A(Branch), .B(zero), .o_Y(Select));
    mux Mux1 (.A(PCPlus4), .B(SumOut), .sel(Select), .Y(Mux1Out));
    Instruction_Memory IM ( .clk(clk), .reset(reset), .PC_in(PC_out[6:2]), .Instruction_out(InstructionOut) );
    Control Control (.i_Instruction(InstructionOut[6:0]), .o_Branch(Branch), .o_MemRead(MemRead), .o_MemtoReg(MemtoReg), .o_ALUOp(ALUOp), .o_MemWrite(MemWrite), .o_ALUSrc(ALUSrc), .o_RegWrite(RegWrite));
    ImmediateGenerator ImmGen (.clk(clk), .i_Instruction(InstructionOut), .o_Immediate(ImmediateOut));
    Register Reg (.clk(clk), .reset(reset), .i_RS1(InstructionOut[19:15]), .i_RS2(InstructionOut[24:20]), .i_RD1(InstructionOut[11:7]), .i_WriteData(Mux3Out), .i_RegWrite(RegWrite), .o_ReadData1(ReadData1Out), .o_ReadData2(ReadData2Out));
    mux Mux2 (.A(ReadData2Out), .B(ImmediateOut), .sel(ALUSrc), .Y(Mux2Out));
    ALU_Control AC (.i_Func7(InstructionOut[30]), .i_Func3(InstructionOut[14:12]), .i_ALUOp(ALUOp), .o_Operation(Control_out));
    ALU ALU2 (.A(ReadData1Out), .B(Mux2Out), .i_Operation(Control_out), .o_ALU_Result(ALUResult), .o_Zero(zero));
    Data_Memory DM (.i_MemWrite(MemWrite), .i_MemRead(MemRead), .i_Address(ALUResult[4:0]), .i_WriteData(ReadData2Out),  .o_ReadData(DM_read) ,.clk(clk));
    mux3 Mux3 (.A(ALUResult), .B(DM_read), .sel(MemtoReg), .Y(Mux3Out));

    // always @(*) begin
    //     $display("PC_out : %h | Instruction_out : %h | Immediate_out : %h | PC_in : %h | Immediate_in : %h | Sum : %h | Select : %h | Mux1Out : %h", PC_out, InstructionOut, ImmediateOut, PC_out, ImmediateOut, SumOut, Select, Mux1Out);
    // end
endmodule