module register_file(
    input [4:0] readReg1, readReg2, writeReg,
    input [63:0] writeData,
    input allowWrite,
    output logic [63:0] reg1Data,
    output logic [63:0] reg2Data,
    output logic [63:0] registers [0:31]
);
    // initialize 32 registers 
    logic [63:0] registers [0:31];
    initial begin
        integer i;
        for (i = 0; i < 32; i = i + 1)
            registers[i] = 64'b0;
    end

    // read regs
    assign reg1Data = registers[readReg1];
    assign reg2Data = registers[readReg2];

    // write reg 
    always @(*) begin
        if (allowWrite && writeReg != 0) 
            registers[writeReg] = writeData;
    end
endmodule