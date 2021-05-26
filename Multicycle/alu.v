module alu(input [31:0] aluIn1, input [31:0] aluIn2, input [1:0] aluOp, output reg [31:0] aluOut0, output reg [31:0] aluOut1, output reg zero);
    
	// Write your code here
    // out0 corresponds to the lower 32 bits of the result
    // out1 corresponds to the higher 32 bits of the result
	
    always @(aluIn1 or aluIn2 or aluOp) 
    begin
        
        case (aluOp)

            2'b00 : {aluOut1, aluOut0} = aluIn1 + aluIn2;

            2'b01 : {aluOut1, aluOut0} = aluIn1 - aluIn2;

            2'b10 : {aluOut1, aluOut0} = aluIn1 * aluIn2;
        
        endcase

        if ({aluOut1, aluOut0} == 64'b0000000000000000000000000000000000000000000000000000000000000000)
        begin
            zero = 1;
        end

        else
        begin
            zero = 0;
        end
        
    end

endmodule