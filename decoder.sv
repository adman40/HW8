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