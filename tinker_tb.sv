`timescale 1ns / 1ps

module tinker_tb;

    // Testbench Signals
    logic [31:0] instruction;
    logic [63:0] registers [0:31]; // Monitor register values

    // Instantiate tinker_core
    tinker_core uut (
        .instruction(instruction)
    );

    // Task to display register contents
    task print_registers;
        integer i;
        $display("Register File:");
        for (i = 0; i < 8; i = i + 1) begin // Print first 8 registers for simplicity
            $display("R[%0d] = %d", i, uut.reg_file.registers[i]);
        end
    endtask

    initial begin
        $display("Starting Test...");

        // Initialize registers (write values to rs and rt)
        uut.reg_file.registers[1] = 10;  // rs = R1 = 10
        uut.reg_file.registers[2] = 20;  // rt = R2 = 20

        print_registers();

        // ADD Instruction: R3 = R1 + R2
        // Format: [OPCODE(5)][RD(5)][RS(5)][RT(5)][LITERAL(12)]
        instruction = {5'h18, 5'd3, 5'd1, 5'd2, 12'b0}; // Opcode 5'h18 = ADD, R3 = R1 + R2

        #10; // Wait for combinational logic to settle
        print_registers(); // Display after addition

        // Check result
        if (uut.reg_file.registers[3] == 30) begin
            $display("✅ Test Passed: R3 contains 30");
        end else begin
            $display("❌ Test Failed: R3 = %d (Expected: 30)", uut.reg_file.registers[3]);
        end

        $stop; // End simulation
    end
endmodule