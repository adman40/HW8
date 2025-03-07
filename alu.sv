module alu (
    input [63:0] param1,
    input [63:0] param2,
    input [4:0] opcode,
    output logic [63:0] result
);

    always @(*) begin
        case (opcode)
        // integer arithmetic
        5'h18: result = param1 + param2; //add
        5'h19: result = param1 + param2; //addi
        5'h1a: result = param1 - param2; //sub
        5'h1b: result = param1 - param2; //subi
        5'h1c: result = param1 * param2; // mul
        5'h1d: result = param1 / param2; // div
        // logic instrucitons
        5'h0: result = param1 & param2; // and
        5'h1: result = param1 | param2; // or
        5'h2: result = param1 ^ param2; // xor
        5'h3: result = ~param1; // not
        5'h4: result = param1 >> param2; // shftr
        5'h5: result = param1 >> param2; // shftri
        5'h6: result = param1 << param2; // shftl
        5'h7: result = param1 << param2; // shftli
        // data movement instructions (maybe wrong idk)
        5'h11: result = param1;
        5'h12: result = (param1 & 64'hFFFFFFFFFFFFF000 ) | param2;
        // floating arithmetic
        5'h14: result = $realtobits($bitstoreal(param1) + $bitstoreal(param2)); // floating add
        5'h15: result = $realtobits($bitstoreal(param1) - $bitstoreal(param2)); // floating sub
        5'h16: result = $realtobits($bitstoreal(param1) * $bitstoreal(param2)); // floating mul
        5'h17: result = $realtobits($bitstoreal(param1) / $bitstoreal(param2)); // floating div
        







