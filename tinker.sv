module tinker_core(
    input logic [31:0] instruction
);

   logic [63:0] writeBackData;
   logic [4:0] opcode, rd, rs, rt;
   logic [11:0] literal;
   logic [63:0] reg1Data, reg2Data, regDestData, aluResult, fpuResult;

   decoder decoder(
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
    .writeData(writeBackData),
    .allowWrite(1'b1),
    .reg1Data(reg1Data),
    .reg2Data(reg2Data)
   );

   assign regDestData = (rd == rs) ? reg1Data : (rd == rt) ? reg2Data : 64'b0;

   alu aluInst(
    .rd(regDestData),
    .rs(reg1Data),
    .rt(reg2Data),
    .opcode(opcode),
    .literal(literal),
    .result(aluResult)
   );

   fpu fpuInst(
       .opcode(opcode),
       .rs(reg1Data),
       .rt(reg2Data),
       .result(fpuResult)
   );

   always @(*) begin
       case(opcode)
           5'h14, 5'h15, 5'h16, 5'h17:
               writeBackData = fpuResult;
            default: 
               writeBackData = aluResult;
       endcase
   end

endmodule

module register_file(
    input [4:0] readReg1, readReg2, writeReg,
    input [63:0] writeData,
    input allowWrite,
    output logic [63:0] reg1Data,
    output logic [63:0] reg2Data,
    output logic [63:0] registers [0:31]
);

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

module decoder (
    input [31:0] instruction,
    output logic [4:0] opcode,
    output logic [4:0] rd, rs, rt,
    output logic [11:0] literal
);

   always @(*) begin
       opcode = instruction[31:27];
       rd = instruction[26:22];
       rs = instruction[21:17];
       rt = instruction[16:12];
       literal = instruction[11:0];
   end

endmodule

module alu(
    input [63:0] rd,
    input [63:0] rs,
    input [63:0] rt,
    input [4:0] opcode,
    input [11:0] literal,
    output logic [63:0] result // should always be rd
);

    always @(*) begin
        case (opcode)
        // integer arithmetic
        5'h18: result = rs + rt; //add
        5'h19: result = rd + literal; //addi
        5'h1a: result = rs - rt; //sub
        5'h1b: result = rd - literal; //subi
        5'h1c: result = rs * rt; // mul
        5'h1d: result = rs / rt; // div
        // logic instrucitons
        5'h0: result = rs & rt; // and
        5'h1: result = rs | rt; // or
        5'h2: result = rs ^ rt; // xor
        5'h3: result = ~rs; // not
        5'h4: result = rs >> rt; // shftr
        5'h5: result = rd >> literal; // shftri
        5'h6: result = rs << rt; // shftl
        5'h7: result = rd << literal; // shftli
        // data movement instructions (maybe wrong idk)
        5'h11: result = rs; // mov 2
        5'h12: result = (rd & 64'hFFFFFFFFFFFFF000 ) | literal; // mov 3
        // floating arithmetic
        default: result = 0;
        endcase 
    end
endmodule

module fpu (
    input [63:0] rs,
    input [63:0] rt,
    input [4:0] opcode,
    output logic [63:0] result 
);
    always@(*) begin
        case(opcode)
        5'h14: result = $realtobits($bitstoreal(rs) + $bitstoreal(rt)); // floating add
        5'h15: result = $realtobits($bitstoreal(rs) - $bitstoreal(rt)); // floating sub
        5'h16: result = $realtobits($bitstoreal(rs) * $bitstoreal(rt)); // floating mul
        5'h17: result = $realtobits($bitstoreal(rs) / $bitstoreal(rt)); // floating div
        default: result = 64'b0;
        endcase
    end
endmodule
        