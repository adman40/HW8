# Verilog Compiler and Simulator
IVERILOG = iverilog
VVP = vvp
GTKWAVE = gtkwave

# Source Files
SRC = tinker.sv alu.sv regs.sv decoder.sv testbench.sv

# Output Executable
OUT = tinker_sim

# Default Target: Compile and Run Simulation
all: compile run

# Compile Verilog Code
compile:
	$(IVERILOG) -o $(OUT) $(SRC)

# Run the Simulation
run:
	$(VVP) $(OUT)

# View Waveform in GTKWave (If using .vcd)
view:
	$(GTKWAVE) waveform.vcd &

# Clean Generated Files
clean:
	rm -f $(OUT) *.vcd