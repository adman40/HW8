module alu (
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
        5'h14: result = $realtobits($bitstoreal(rs) + $bitstoreal(rt)); // floating add
        5'h15: result = $realtobits($bitstoreal(rs) - $bitstoreal(rt)); // floating sub
        5'h16: result = $realtobits($bitstoreal(rs) * $bitstoreal(rt)); // floating mul
        5'h17: result = $realtobits($bitstoreal(rs) / $bitstoreal(rt)); // floating div
        endcase 
    end
endmodule
        







