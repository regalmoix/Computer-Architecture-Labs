module control_circuit 
(
    input clk, 
    input reset, 
    input [5:0] opcode, 
    input [5:0] funct, 
                        
                        
    output reg IorD, 
    output reg memRead, 
    output reg IRWrite, 
    output reg regDest, 
    output reg regWrite,
    output reg aluSrcA, 
    output reg [1:0] aluSrcB, 
    output reg [1:0] aluOp, 
    output reg hiWrite,
    output reg loWrite, 
    output reg [1:0] memToReg, 
    output reg [1:0] pcSrc, 
    output reg pcWrite,
    output reg branch 
);

    reg [3:0] state;
    reg [3:0] next;


    //STATES 

    parameter s0  = 4'b0000;
    parameter s1  = 4'b0001;
    parameter s2  = 4'b0010;
    parameter s3  = 4'b0011;
    
    parameter s4  = 4'b0100;
    parameter s5  = 4'b0101;
    parameter s6  = 4'b0110;
    parameter s7  = 4'b0111;

    parameter s8  = 4'b1000;
    parameter s9  = 4'b1001;
    parameter s10 = 4'b1010;
    parameter s11 = 4'b1011;
    
    parameter s12 = 4'b1100;


    //INSTRUCTIONS:
    parameter mult = 6'b011000;
    parameter mfhi = 6'b010000;

    parameter beq  = 6'b000100;
    parameter addi = 6'b001000;
    parameter lw   = 6'b100011;

    parameter j    = 6'b000010;


    // Write your code here

    // state must update on every negedge of clk

    // the outputs have to be assigned as per the value of state

    always @ (negedge clk)
    begin

        if (reset)
            state <= s0;    //init. Maybe fetch also depends??
        
        else
            state <= next;
          
    end



    always @ (state or opcode)  //function seems irrelevant so can be ignored for current processor
    begin
        
        case (state)
        
            s0 : next = s1;
            s1 : next = s2;
            
            s2 :    begin
                        case (opcode)
                            addi : next = s3;
                            mult : next = s4;
                            mfhi : next = s5;
                            lw   : next = s6;
                            j    : next = s7;
                            beq  : next = s8;
                        endcase
                    end
                    
            s3 : next = s9;
            s4 : next = s10;
            s5 : next = s1;
            s6 : next = s11;
            s7 : next = s1;
            s8 : next = s1;
            s9 : next = s1;
            s10: next = s1;
            s11: next = s12;
            s12: next = s1;

            default : next = s0;        //init states
        endcase
    end


    always @ (state)
    begin
        
        IorD    = 1'b0;
        memRead = 1'b0;
        IRWrite = 1'b0;
        regDest = 1'b0;
        regWrite= 1'b0;

        aluSrcA = 1'b0;
        aluSrcB = 2'b00;

        aluOp   = 2'b00;

        hiWrite = 1'b0;
        loWrite = 1'b0;

        memToReg= 2'b00;

        pcSrc   = 2'b00;
        pcWrite = 1'b0;

        branch  = 1'b0;


        case (state)
            
            s1  :
                    begin
                        memRead = 1'b1;
                        IRWrite = 1'b1;
                        aluSrcB = 2'b01;
                        pcSrc   = 2'b10;
                        pcWrite = 1'b1;
                    end
            

            s2  :
                    begin
                        aluSrcB = 2'b11;
                    end
            
            s3  :
                    begin
                        aluSrcA = 1'b1;
                        aluSrcB = 2'b10;
                    end
            
            s4  :
                    begin
                        aluSrcA = 1'b1;
                        aluOp   = 2'b10;
                    end
            

            s5  :
                    begin
                    regDest = 1'b1; 
                    regWrite= 1'b1;
                    memToReg= 2'b01;
                    end
            
            s6  :
                    begin
                        aluSrcA = 1'b1;
                        aluSrcB = 2'b10;
                    end
            
            s7  :
                    begin
                        pcSrc   = 2'b01;
                        pcWrite = 1'b1;
                    end
            
            s8  :
                    begin
                        aluSrcA = 1'b1;
                        aluOp   = 2'b01;
                        branch  = 1'b1;
                    end
            
            s9  :
                    begin
                        regWrite= 1'b1;
                        memToReg= 2'b10;
                    end
            

            s10 :
                    begin
                        hiWrite = 1'b1;
                        loWrite = 1'b1;
                    end
            
            s11 :
                    begin
                        IorD    = 1'b1;
                        memRead = 1'b1;
                    end

            s12 :
                    begin
                        regWrite= 1'b1;
                    end

        endcase

    end

endmodule