`timescale 1ns/1ps
module tinker_tb;

  // Clock and reset signals
  reg clk;
  reg reset;

  // Instantiate the Device Under Test (DUT)
  tinker_core uut (
    .clk(clk),
    .reset(reset)
  );

  // Clock generation: 10 ns period (5 ns high, 5 ns low)
  initial begin
    clk = 0;
    forever #5 clk = ~clk;
  end

  // Reset generation and simulation control
  initial begin
    // Assert reset for a few cycles
    reset = 1;
    #20;  // hold reset for 20 ns
    reset = 0;
    
    // Optionally, you could initialize memory or drive inputs via hierarchical references here.
    // For example, if you wish to load a test instruction into memory, you can access:
    // uut.memory.bytes[32'h2000] = <your 8-bit value>;
    
    // Run simulation for additional time to observe behavior
    #200;
    $finish;
  end

  // Monitor the program counter from the DUT for debug purposes
  initial begin
    $monitor("Time = %0t ns: Program Counter = %h", $time, uut.programCounter);
  end

endmodule