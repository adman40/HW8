`timescale 1ns/1ps

module tb_tinker_core;
    reg clk;
    reg reset;
    reg [31:0] add_inst; // addition instruction variable

    // Instantiate the design under test.
    tinker_core uut (
        .clk(clk),
        .reset(reset)
    );

    // Delay clock generation so initialization can occur first.
    initial begin
        clk = 0;
        #25;            // Delay clock start by 25 ns
        forever #5 clk = ~clk;
    end

    // Generate reset: assert for 20 ns, then deassert.
    initial begin
        reset = 1;
        #20;
        reset = 0;
    end

    // Load the addition instruction into memory at address 0x2000.
    // The instruction is formatted as:
    //   opcode   = 5'h18 (add: rs + rt)
    //   rd       = 5'd1   (destination register R1)
    //   rs       = 5'd2   (first source register R2)
    //   rt       = 5'd3   (second source register R3)
    //   literal  = 12'd0  (unused for addition)
    // This creates an instruction word: {opcode, rd, rs, rt, literal}
    initial begin
        @(negedge reset);
        #1; // small delay after reset deassertion
        add_inst = {5'h18, 5'd1, 5'd2, 5'd3, 12'd0};
        uut.instructionMemInstance.bytes[32'h00002000]     = add_inst[7:0];
        uut.instructionMemInstance.bytes[32'h00002000 + 1] = add_inst[15:8];
        uut.instructionMemInstance.bytes[32'h00002000 + 2] = add_inst[23:16];
        uut.instructionMemInstance.bytes[32'h00002000 + 3] = add_inst[31:24];
        $display("Addition instruction loaded: %h", add_inst);
    end

    // Initialize registers R2 and R3.
    // R2 will be set to 10 and R3 to 15 so that R1 = 10 + 15 = 25.
    initial begin
        @(negedge reset);
        #1; // small delay after reset deassertion
        uut.reg_file.registers[2] = 64'd10;
        uut.reg_file.registers[3] = 64'd15;
        $display("Registers initialized: R2 = 10, R3 = 15");
    end

    // Monitor the result after allowing a few clock cycles for execution.
    initial begin
        #150; // wait enough time for the instruction to be fetched and executed
        $display("Final value in R1: %d", uut.reg_file.registers[1]);
        if (uut.reg_file.registers[1] == 25)
            $display("Test Passed: R1 = R2 + R3 = 10 + 15 = 25");
        else
            $display("Test Failed: Expected 25 but got %d", uut.reg_file.registers[1]);
        $finish;
    end

endmodule