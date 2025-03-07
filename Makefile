# Verilog Compiler and Simulator
IVERILOG = iverilog
VVP = vvp

# Source Files
SRC = tinker.sv alu.sv regs.sv decoder.sv

# Output Executable
OUT = tinker_sim

# Default Target: Compile and Run Simulation
all: compile run

# Compile Verilog Code
compile:
	$(IVERILOG) -g2012 -o $(OUT) $(SRC)

# Run the Simulation
run:
	$(VVP) $(OUT)

# Clean Generated Files
clean:
	rm -f $(OUT) *.vcd