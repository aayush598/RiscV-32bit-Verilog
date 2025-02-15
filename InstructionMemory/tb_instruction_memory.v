`timescale 1ns / 1ps

module tb_instruction_memory;

    // Testbench Signals
    reg [31:0] addr;
    wire [31:0] instr;

    // Instantiate the Instruction Memory
    instruction_memory uut (
        .addr(addr),
        .instr(instr)
    );

    // VCD Dump Setup
    initial begin
        $dumpfile("instruction_mem.vcd");
        $dumpvars(0, tb_instruction_memory);
    end

    // Test Sequence
    initial begin
        // Monitor values
        $monitor("Time=%0t | Address=%h | Instruction=%h", $time, addr, instr);

        // Initialize test
        addr = 32'h00000000; #10;  // Expect instruction at 0
        addr = 32'h00000004; #10;  // Expect instruction at 4
        addr = 32'h00000008; #10;  // Expect instruction at 8
        addr = 32'h0000000C; #10;  // Expect instruction at 12
        addr = 32'h00000010; #10;  // Expect instruction at 16

        #50 $finish;  // End simulation
    end

endmodule
