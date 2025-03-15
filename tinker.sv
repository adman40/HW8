module tinker_core(
    input clk,
    input reset
);

   parameter MEM_SIZE = 524288;

   reg [31:0] programCounter;
   wire [31:0] nextProgramCounter;
   always @(posedge clk or posedge reset) begin
       if (reset)
           programCounter <= 32'h00002000;
       else 
           programCounter <= nextProgramCounter;
   end

   wire [31:0] fetchInstruction;
   memory #(.MEM_SIZE(MEM_SIZE)) instructionMemInstance (
    .clk(clk),
    .reset(reset),
    .address(programCounter),
    .writeEnable(1'b0),
    .writeData(64'b0),
    .fetchAddress(programCounter),
    .dataAddress(32'b0),
    .fetchInstruction(fetchInstruction),
    .dataOut()
   );

   wire [31:0] instruction;
   fetch fetchInst (
    .programCounter(programCounter),
    .fetchInstruction(fetchInstruction),
    .instruction(instruction)
   );

   wire [63:0] reg1Data, reg2Data;
   wire [4:0] regReadAddressA, regReadAddressB, regWriteAddress;
   wire [63:0] regWriteData;
   wire regWriteEnable;

   register_file reg_file (
        .clk(clk),
        .reset(reset),
        .readReg1(regReadAddrA),
        .readReg2(regReadAddrB),
        .writeReg(regWriteAddr),
        .writeData(regWriteData),
        .allowWrite(regWriteEnable),
        .reg1Data(reg1Data),
        .reg2Data(reg2Data)
   );

   wire [31:0] memAddress;
   wire memWriteEnabled;
   wire [63:0] memWriteData;
   wire [31:0] dataAddress;
   wire [63:0] memData;

   memory #(.MEM_SIZE(MEM_SIZE)) memory(
    .clk(clk),
    .reset(reset),
    .address(memAddress),
    .writeEnable(memWriteEnabled),
    .writeData(memWriteData),
    .fetchAddress(32'b0),
    .dataAddress(dataAddress),
    .fetchInstruction(),
    .dataOut(memData)
   );

   wire [31:0] controlNextPC;
   wire [63:0] controlWriteBackData;
   wire controlMemWriteEnabled;
   wire [31:0] controlMemAddress;
   wire [63:0] controlMemWriteData;
   wire [31:0] controlDataAddress;
   wire controlWriteEnable;
   wire [4:0] controlWriteReg;
   wire[4:0] controlRegFileAddressA;
   wire[4:0] controlgRegFileAddressB;

   control controlInst (
    .clk(clk),
    .reset(reset),
    .instruction(instruction),
    .programCounter(programCounter),
    .portAData(reg1Data),
    .portBData(reg2Data),
    .memData(memData),
    .programCounterNextState(controlNextPC),
    .writeBackData(controlWriteBackData),
    .memWriteEnabled(controlMemWriteEnabled),
    .memAddress(controlMemAddress),
    .memWriteData(controlMemWriteData),
    .dataAddress(controlDataAddress),
    .writeEnable(controlWriteEnable),
    .writeRegister(controlWriteReg),
    .regFileAddressA(controlRegFileAddressA),
    .regFileAddressB(controlgRegFileAddressB)
   );

   assign nextProgramCounter = controlNextPC;
   assign regWriteData = controlWriteBackData;
   assign regWriteAddress = controlWriteReg;
   assign regWriteEnable = controlWriteEnable;
   assign regReadAddressA = controlRegFileAddressA;
   assign regReadAddressB = controlgRegFileAddressB;
   assign memAddress = controlMemAddress;
   assign memWriteEnabled = controlMemWriteEnabled;
   assign memWriteData = controlMemWriteData;
   assign dataAddress = controlDataAddress;

endmodule

module register_file(
    input logic clk,
    input logic reset,
    input [4:0] readReg1, readReg2, writeReg,
    input [63:0] writeData,
    input allowWrite,
    output logic [63:0] reg1Data,
    output logic [63:0] reg2Data,
    output logic [63:0] registers [0:31]
);


   parameter MEM_SIZE = 524288;

    initial begin
        integer i;
        for (i = 0; i < 32; i = i + 1)
            registers[i] = 64'b0;
        registers[31] = 64'h00010000;
    end

    // read regs
    assign reg1Data = registers[readReg1];
    assign reg2Data = registers[readReg2];
    integer i;
    // write reg 
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 32; i = i + 1)
                registers[i] = 64'b0;
            registers[31] = 64'h00010000;
        end
        else if (allowWrite && writeReg != 0) 
            registers[writeReg] <= writeData;
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

module memory (
    input logic clk,
    input logic reset,
    input logic [31:0] address,
    input logic writeEnable,
    input logic [63:0] writeData,
    input logic [31:0] fetchAddress,
    input logic [31:0] dataAddress,
    output logic [31:0] fetchInstruction,
    output logic [63:0] dataOut
);
    parameter MEM_SIZE = 524288;
    reg [7:0] bytes [0:MEM_SIZE-1];

    always @(posedge clk) begin
        if (reset)begin
        end
        if (writeEnable) begin
            bytes[address]   <= writeData[63:56];
            bytes[address+1] <= writeData[55:48];
            bytes[address+2] <= writeData[47:40];
            bytes[address+3] <= writeData[39:32];
            bytes[address+4] <= writeData[31:24];
            bytes[address+5] <= writeData[23:16];
            bytes[address+6] <= writeData[15:8];
            bytes[address+7] <= writeData[7:0];
        end
    end

    assign fetchInstruction = {
        bytes[fetchAddress + 3],
        bytes[fetchAddress + 2],
        bytes[fetchAddress + 1],
        bytes[fetchAddress]
    };

    assign dataOut = {
        bytes[dataAddress + 7],
        bytes[dataAddress + 6],
        bytes[dataAddress + 5],
        bytes[dataAddress + 4],
        bytes[dataAddress + 3],
        bytes[dataAddress + 2],
        bytes[dataAddress + 1],
        bytes[dataAddress]
    };
endmodule

module fetch (
    input logic [31:0] programCounter,
    input logic [31:0] fetchInstruction,
    output logic [31:0] instruction
);
assign instruction = fetchInstruction;
endmodule

module control (
    input logic clk,
    input logic reset,
    input logic [31:0] instruction,
    input logic [31:0] programCounter,
    input logic [63:0] portAData,
    input logic [63:0] portBData,
    input logic [63:0] memData,
    output reg [31:0] programCounterNextState,
    output reg [63:0] writeBackData,
    output reg memWriteEnabled,
    output reg [31:0] memAddress,
    output reg [63:0] memWriteData,
    output reg [31:0] dataAddress,
    output reg writeEnable,
    output reg [4:0] writeRegister,
    output reg [4:0] regFileAddressA,
    output reg [4:0] regFileAddressB
);

    wire [4:0] opcode, rd, rs, rt;
    wire [11:0] literal;

    decoder decoder (
        .instruction(instruction),
        .opcode(opcode),
        .rd(rd),
        .rs(rs),
        .rt(rt),
        .literal(literal)
    );

    wire [63:0] aluResult;
    logic [63:0] rdData;
    logic [63:0] rsData;
    logic [63:0] rtData;

    always @(*) begin
        case (opcode)
            5'h18, 5'h1a, 5'h1c, 5'h1d, 5'h0, 5'h1, 5'h2, 5'h4, 5'h6, 5'h14, 5'h15, 5'h16, 5'h17: begin
            rsData = portAData;
            rtData = portBData;
            end
            5'h19, 5'h1b, 5'h5, 5'h7, 5'h12: begin
            rdData = portAData;
            end
            5'h3, 5'h11: begin
            rsData = portAData;
            end
            default: begin
                rdData = 64'b0;
                rsData = 64'b0;
                rtData = 64'b0;
            end
        endcase
    end

    alu aluInst (
        .rd(rdData),
        .rs(rsData),
        .rt(rtData),
        .opcode(opcode),
        .literal(literal),
        .result(aluResult)
    );

    wire [63:0] fpuResult;

    fpu fpuInst(
       .opcode(opcode),
       .rs(rsData),
       .rt(rtData),
       .result(fpuResult)
   ); 

   always @(*) begin
    regFileAddressA = rs;
    regFileAddressB = rt;
       case (opcode)
           5'hb:
               regFileAddressB = rd;
           5'h8, 5'h9, 5'h19, 5'h1b, 5'h5, 5'h7, 5'h12:
               regFileAddressA = rd;
           5'hc: begin
               regFileAddressA = rd;
               regFileAddressB = 5'd31;
           end
           5'hd:
               regFileAddressA = 5'd31;
           5'he: 
               regFileAddressB = rd;
           5'h13: begin
               regFileAddressA = rd;
               regFileAddressB = rs;
           end
       endcase
   end

   always @(*) begin
       programCounterNextState = programCounter + 4;
       writeBackData = 64'b0;
       writeEnable = 1'b0;
       writeRegister = rd;
       memWriteEnabled = 1'b0;
       memAddress = 32'b0;
       memWriteData = 64'b0;
       dataAddress = 32'b0;

       case (opcode)
           5'h0, 5'h1, 5'h2, 5'h3, 5'h4, 5'h5, 5'h6, 5'h7,
           5'h18, 5'h19, 5'h1a, 5'h1b, 5'h1c, 5'h1d, 5'h12: begin
               writeBackData = aluResult;
               writeEnable = 1'b1;
           end
           5'h8: begin
               programCounterNextState = portAData;
           end
           5'h9: begin
                programCounterNextState = programCounter + portAData[31:0];
           end
           5'ha: begin
                programCounterNextState = programCounter + {{20{literal[11]}}, literal};
           end
           5'hb: begin
                if (portAData != 0) 
                    programCounterNextState = portBData;
                else 
                    programCounterNextState = programCounter + 4;
           end
           5'hc: begin
               memWriteEnabled = 1'b1;
               memAddress = portBData - 8;
               memWriteData = programCounter + 4;
               programCounterNextState = portAData;
           end
           5'hd: begin
               dataAddress = portAData - 8;
               programCounterNextState = memData[31:0];
           end
           5'he: begin
               if ($signed(portAData) > $signed(portBData))
                   programCounterNextState = portBData;
                else 
                   programCounterNextState = programCounter + 4;
           end
           5'h10: begin
               dataAddress = portAData + {20'b0, literal};
               writeBackData = memData;
               writeEnable = 1'b1;
           end
           5'h11: begin
               writeBackData = portAData;
               writeEnable = 1'b1;
           end
           5'h13: begin
               memWriteEnabled = 1'b1;
               memAddress = portAData + {20'b0, literal};
               memWriteData = portBData;
           end
           5'h14, 5'h15, 5'h16, 5'h17: begin
               writeBackData = fpuResult;
               writeEnable = 1'b1;
           end
           default: begin
               programCounterNextState = programCounter + 4;
           end
       endcase
   end
endmodule
