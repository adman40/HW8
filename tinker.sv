module tinker_core(
    input logic [31:0] instruction
);

   logic [4:0] opcode, rd, rs, rt;
   logic [11:0] literal;
   logic [63:0] reg1Data, reg2Data, regDestData, aluResult;

   logic [63:0] registers [0:31];

   decoder decoder (
    .instruction(instruction),
    .opcode(opcode),
    .rd(rd),
    .rs(rs),
    .rt(rt),
    .literal(literal)
   );

   register_file reg_file (
    .readReg1(rs),
    .readReg2(rt),
    .writeReg(rd),
    .writeData(aluResult),
    .allowWrite(1'b1),
    .reg1Data(reg1Data),
    .reg2Data(reg2Data),
    .registers(registers)
   );

   assign regDestData = registers[rd];

   alu alu (
    .rd(regDestData),
    .rs(reg1Data),
    .rt(reg2Data),
    .opcode(opcode),
    .literal(literal),
    .result(aluResult)
   );

endmodule