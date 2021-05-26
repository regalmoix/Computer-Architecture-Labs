//RegisterFile.v

//D flip-flop implementation
module D_ff( input clk, input reset, input regWrite, input decOut1b , input d, output reg q);
    always @ (negedge clk)
    begin
    if(reset==1)
        q=0;
    else
        if(regWrite == 1 && decOut1b==1)
        begin
            q=d;
        end
    end
endmodule

//32 bit register
module register32bit(input clk, input reset, input regWrite, input decoderOut1bit, input[31:0] writeData, output[31:0] regOut);
//A N-bit register consists of N D flip-flops, each storing a bit of data.
//In this case, there will be 32 instances of D_ff, each taking writeData[0]...[31] as the d input and regOut[0]...[31] as the q output
//WRITE CODE HERE

    D_ff ff0(clk, reset, regWrite, decoderOut1bit, writeData[0], regOut[0]);
    D_ff ff1(clk, reset, regWrite, decoderOut1bit, writeData[1], regOut[1]);
    D_ff ff2(clk, reset, regWrite, decoderOut1bit, writeData[2], regOut[2]);
    D_ff ff3(clk, reset, regWrite, decoderOut1bit, writeData[3], regOut[3]);
    D_ff ff4(clk, reset, regWrite, decoderOut1bit, writeData[4], regOut[4]);
    D_ff ff5(clk, reset, regWrite, decoderOut1bit, writeData[5], regOut[5]);
    D_ff ff6(clk, reset, regWrite, decoderOut1bit, writeData[6], regOut[6]);
    D_ff ff7(clk, reset, regWrite, decoderOut1bit, writeData[7], regOut[7]);
    D_ff ff8(clk, reset, regWrite, decoderOut1bit, writeData[8], regOut[8]);
    D_ff ff9(clk, reset, regWrite, decoderOut1bit, writeData[9], regOut[9]);
    D_ff ff10(clk, reset, regWrite, decoderOut1bit, writeData[10], regOut[10]);
    D_ff ff11(clk, reset, regWrite, decoderOut1bit, writeData[11], regOut[11]);
    D_ff ff12(clk, reset, regWrite, decoderOut1bit, writeData[12], regOut[12]);
    D_ff ff13(clk, reset, regWrite, decoderOut1bit, writeData[13], regOut[13]);
    D_ff ff14(clk, reset, regWrite, decoderOut1bit, writeData[14], regOut[14]);
    D_ff ff15(clk, reset, regWrite, decoderOut1bit, writeData[15], regOut[15]);
    D_ff ff16(clk, reset, regWrite, decoderOut1bit, writeData[16], regOut[16]);
    D_ff ff17(clk, reset, regWrite, decoderOut1bit, writeData[17], regOut[17]);
    D_ff ff18(clk, reset, regWrite, decoderOut1bit, writeData[18], regOut[18]);
    D_ff ff19(clk, reset, regWrite, decoderOut1bit, writeData[19], regOut[19]);
    D_ff ff20(clk, reset, regWrite, decoderOut1bit, writeData[20], regOut[20]);
    D_ff ff21(clk, reset, regWrite, decoderOut1bit, writeData[21], regOut[21]);
    D_ff ff22(clk, reset, regWrite, decoderOut1bit, writeData[22], regOut[22]);
    D_ff ff23(clk, reset, regWrite, decoderOut1bit, writeData[23], regOut[23]);
    D_ff ff24(clk, reset, regWrite, decoderOut1bit, writeData[24], regOut[24]);
    D_ff ff25(clk, reset, regWrite, decoderOut1bit, writeData[25], regOut[25]);
    D_ff ff26(clk, reset, regWrite, decoderOut1bit, writeData[26], regOut[26]);
    D_ff ff27(clk, reset, regWrite, decoderOut1bit, writeData[27], regOut[27]);
    D_ff ff28(clk, reset, regWrite, decoderOut1bit, writeData[28], regOut[28]);
    D_ff ff29(clk, reset, regWrite, decoderOut1bit, writeData[29], regOut[29]);
    D_ff ff30(clk, reset, regWrite, decoderOut1bit, writeData[30], regOut[30]);
    D_ff ff31(clk, reset, regWrite, decoderOut1bit, writeData[31], regOut[31]);

endmodule


//Active high decoder
module decoder_5to32(input[4:0] in, output reg[31:0] decOut);
//WRITE CODE HERE

    always @ (in)
    begin
        
        case (in)
            
            5'b00000 : decOut = 32'b00000000000000000000000000000001;

            5'b00001 : decOut = 32'b00000000000000000000000000000010;

            5'b00010 : decOut = 32'b00000000000000000000000000000100;

            5'b00011 : decOut = 32'b00000000000000000000000000001000;

            5'b00100 : decOut = 32'b00000000000000000000000000010000;

            5'b00101 : decOut = 32'b00000000000000000000000000100000;

            5'b00110 : decOut = 32'b00000000000000000000000001000000;

            5'b00111 : decOut = 32'b00000000000000000000000010000000;

            5'b01000 : decOut = 32'b00000000000000000000000100000000;

            5'b01001 : decOut = 32'b00000000000000000000001000000000;

            5'b01010 : decOut = 32'b00000000000000000000010000000000;

            5'b01011 : decOut = 32'b00000000000000000000100000000000;

            5'b01100 : decOut = 32'b00000000000000000001000000000000;

            5'b01101 : decOut = 32'b00000000000000000010000000000000;

            5'b01110 : decOut = 32'b00000000000000000100000000000000;

            5'b01111 : decOut = 32'b00000000000000001000000000000000;

            5'b10000 : decOut = 32'b00000000000000010000000000000000;

            5'b10001 : decOut = 32'b00000000000000100000000000000000;

            5'b10010 : decOut = 32'b00000000000001000000000000000000;

            5'b10011 : decOut = 32'b00000000000010000000000000000000;

            5'b10100 : decOut = 32'b00000000000100000000000000000000;

            5'b10101 : decOut = 32'b00000000001000000000000000000000;

            5'b10110 : decOut = 32'b00000000010000000000000000000000;

            5'b10111 : decOut = 32'b00000000100000000000000000000000;

            5'b11000 : decOut = 32'b00000001000000000000000000000000;

            5'b11001 : decOut = 32'b00000010000000000000000000000000;

            5'b11010 : decOut = 32'b00000100000000000000000000000000;

            5'b11011 : decOut = 32'b00001000000000000000000000000000;

            5'b11100 : decOut = 32'b00010000000000000000000000000000;

            5'b11101 : decOut = 32'b00100000000000000000000000000000;

            5'b11110 : decOut = 32'b01000000000000000000000000000000;

            5'b11111 : decOut = 32'b10000000000000000000000000000000;

            

            default : decOut = 32'b00000000000000000000000000000000;
        endcase
        
    end
  
endmodule

// select 0 = in0 1 = in1
module mux2to1_5bit(input [4:0] in0, input [4:0] in1, input select, output reg [4:0] muxOut);   
    //WRITE CODE HERE
    always @ (in0 or in1 or select)
    begin

        	case (select)
			1'b0 : muxOut = in0;
			1'b1 : muxOut = in1;
			default : muxOut = 5'bxxxxx;

		    endcase
	end
	
endmodule

module mux32to1_32bit(input[31:0] in0, input[31:0] in1, input[31:0] in2, input[31:0] in3, input[31:0] in4, input[31:0] in5, input[31:0] in6, input[31:0] in7,
    input[31:0] in8, input[31:0] in9, input[31:0] in10, input[31:0] in11, input[31:0] in12, input[31:0] in13, input[31:0] in14, input[31:0] in15,
    input[31:0] in16, input[31:0] in17, input[31:0] in18, input[31:0] in19, input[31:0] in20, input[31:0] in21, input[31:0] in22, input[31:0] in23,
    input[31:0] in24, input[31:0] in25, input[31:0] in26, input[31:0] in27, input[31:0] in28, input[31:0] in29, input[31:0] in30, input[31:0] in31,input[4:0] select,output reg[31:0] muxOut);
   //WRITE CODE HERE

    always @ (in0 or in1 or in2 or in3 or in4 or in5 or in6 or in7 or in8 or in9 or in10 or in11 or in12 or in13 or in14 or in15 or in16 or in17 or in18 or in19 or in20 or in21 or in22 or in23 or in24 or in25 or in26 or in27 or in28 or in29 or in30 or in31 or select) 
    begin 
        case (select)

            5'b00000 : muxOut = in0;

            5'b00001 : muxOut = in1;

            5'b00010 : muxOut = in2;

            5'b00011 : muxOut = in3;

            5'b00100 : muxOut = in4;

            5'b00101 : muxOut = in5;

            5'b00110 : muxOut = in6;

            5'b00111 : muxOut = in7;

            5'b01000 : muxOut = in8;

            5'b01001 : muxOut = in9;

            5'b01010 : muxOut = in10;

            5'b01011 : muxOut = in11;

            5'b01100 : muxOut = in12;

            5'b01101 : muxOut = in13;

            5'b01110 : muxOut = in14;

            5'b01111 : muxOut = in15;

            5'b10000 : muxOut = in16;

            5'b10001 : muxOut = in17;

            5'b10010 : muxOut = in18;

            5'b10011 : muxOut = in19;

            5'b10100 : muxOut = in20;

            5'b10101 : muxOut = in21;

            5'b10110 : muxOut = in22;

            5'b10111 : muxOut = in23;

            5'b11000 : muxOut = in24;

            5'b11001 : muxOut = in25;

            5'b11010 : muxOut = in26;

            5'b11011 : muxOut = in27;

            5'b11100 : muxOut = in28;

            5'b11101 : muxOut = in29;

            5'b11110 : muxOut = in30;

            5'b11111 : muxOut = in31;


        endcase


    end
	
endmodule

//register set with 32 registers
module registerSet(input clk, input reset, input regWrite, input[31:0] decoderOut, input[31:0] writeData, 
    output [31:0] outR0, output[31:0] outR1, output[31:0] outR2, output[31:0] outR3, output[31:0] outR4, output[31:0] outR5, output[31:0] outR6, output[31:0] outR7,
    output [31:0] outR8, output[31:0] outR9, output[31:0] outR10, output[31:0] outR11, output[31:0] outR12, output[31:0] outR13, output[31:0] outR14, output[31:0] outR15,
    output [31:0] outR16, output[31:0] outR17, output[31:0] outR18, output[31:0] outR19, output[31:0] outR20, output[31:0] outR21, output[31:0] outR22, output[31:0] outR23,
    output [31:0] outR24, output[31:0] outR25, output[31:0] outR26, output[31:0] outR27, output[31:0] outR28, output[31:0] outR29, output[31:0] outR30, output[31:0] outR31);
    //WRITE CODE HERE

    // clk common
    // reset common
    // regWrite
    // pass appropriate decoderOut[i]
    // common writedata
    //appropriate outRi

    //register32bit r1(clk, reset, regWrite, decoderOut[i], writeData, outRi);
    // Generated using C++ code

    register32bit r0 ( clk, reset, regWrite, decoderOut[0], writeData, outR0 ); 
    register32bit r1 ( clk, reset, regWrite, decoderOut[1], writeData, outR1 ); 
    register32bit r2 ( clk, reset, regWrite, decoderOut[2], writeData, outR2 ); 
    register32bit r3 ( clk, reset, regWrite, decoderOut[3], writeData, outR3 ); 
    register32bit r4 ( clk, reset, regWrite, decoderOut[4], writeData, outR4 ); 
    register32bit r5 ( clk, reset, regWrite, decoderOut[5], writeData, outR5 ); 
    register32bit r6 ( clk, reset, regWrite, decoderOut[6], writeData, outR6 ); 
    register32bit r7 ( clk, reset, regWrite, decoderOut[7], writeData, outR7 ); 
    register32bit r8 ( clk, reset, regWrite, decoderOut[8], writeData, outR8 ); 
    register32bit r9 ( clk, reset, regWrite, decoderOut[9], writeData, outR9 ); 
    register32bit r10 ( clk, reset, regWrite, decoderOut[10], writeData, outR10 ); 
    register32bit r11 ( clk, reset, regWrite, decoderOut[11], writeData, outR11 ); 
    register32bit r12 ( clk, reset, regWrite, decoderOut[12], writeData, outR12 ); 
    register32bit r13 ( clk, reset, regWrite, decoderOut[13], writeData, outR13 ); 
    register32bit r14 ( clk, reset, regWrite, decoderOut[14], writeData, outR14 ); 
    register32bit r15 ( clk, reset, regWrite, decoderOut[15], writeData, outR15 ); 
    register32bit r16 ( clk, reset, regWrite, decoderOut[16], writeData, outR16 ); 
    register32bit r17 ( clk, reset, regWrite, decoderOut[17], writeData, outR17 ); 
    register32bit r18 ( clk, reset, regWrite, decoderOut[18], writeData, outR18 ); 
    register32bit r19 ( clk, reset, regWrite, decoderOut[19], writeData, outR19 ); 
    register32bit r20 ( clk, reset, regWrite, decoderOut[20], writeData, outR20 ); 
    register32bit r21 ( clk, reset, regWrite, decoderOut[21], writeData, outR21 ); 
    register32bit r22 ( clk, reset, regWrite, decoderOut[22], writeData, outR22 ); 
    register32bit r23 ( clk, reset, regWrite, decoderOut[23], writeData, outR23 ); 
    register32bit r24 ( clk, reset, regWrite, decoderOut[24], writeData, outR24 ); 
    register32bit r25 ( clk, reset, regWrite, decoderOut[25], writeData, outR25 ); 
    register32bit r26 ( clk, reset, regWrite, decoderOut[26], writeData, outR26 ); 
    register32bit r27 ( clk, reset, regWrite, decoderOut[27], writeData, outR27 ); 
    register32bit r28 ( clk, reset, regWrite, decoderOut[28], writeData, outR28 ); 
    register32bit r29 ( clk, reset, regWrite, decoderOut[29], writeData, outR29 ); 
    register32bit r30 ( clk, reset, regWrite, decoderOut[30], writeData, outR30 ); 
    register32bit r31 ( clk, reset, regWrite, decoderOut[31], writeData, outR31 ); 


endmodule

module registerFile(input clk, input reset, input regWrite, input[4:0] rs, input[4:0] rt, input[4:0] rd0, input[4:0] rd1, input[31:0] writeData, input select, output[31:0] regRs, output[31:0] regRt);
    //WRITE CODE HERE

    wire[4:0] decoder_input;

    mux2to1_5bit entrymux (rd0, rd1, select, decoder_input);

    wire[31:0] decoder_output;

    decoder_5to32 d (decoder_input, decoder_output);


    wire[31:0] outR0;
    wire[31:0] outR1;
    wire[31:0] outR2;
    wire[31:0] outR3;
    wire[31:0] outR4;
    wire[31:0] outR5;
    wire[31:0] outR6;
    wire[31:0] outR7;
    wire[31:0] outR8;
    wire[31:0] outR9;
    wire[31:0] outR10;
    wire[31:0] outR11;
    wire[31:0] outR12;
    wire[31:0] outR13;
    wire[31:0] outR14;
    wire[31:0] outR15;
    wire[31:0] outR16;
    wire[31:0] outR17;
    wire[31:0] outR18;
    wire[31:0] outR19;
    wire[31:0] outR20;
    wire[31:0] outR21;
    wire[31:0] outR22;
    wire[31:0] outR23;
    wire[31:0] outR24;
    wire[31:0] outR25;
    wire[31:0] outR26;
    wire[31:0] outR27;
    wire[31:0] outR28;
    wire[31:0] outR29;
    wire[31:0] outR30;
    wire[31:0] outR31;

    //wire[31:0] sout;
    //wire[31:0] tout;

    registerSet rset (clk, reset, regWrite, decoder_output, writeData, outR0, outR1, outR2, outR3, outR4, outR5, outR6, outR7, outR8, outR9, outR10, outR11, outR12, outR13, outR14, outR15, outR16, outR17, outR18, outR19, outR20, outR21, outR22, outR23, outR24, outR25, outR26, outR27, outR28, outR29, outR30, outR31);

    mux32to1_32bit s (outR0, outR1, outR2, outR3, outR4, outR5, outR6, outR7, outR8, outR9, outR10, outR11, outR12, outR13, outR14, outR15, outR16, outR17, outR18, outR19, outR20, outR21, outR22, outR23, outR24, outR25, outR26, outR27, outR28, outR29, outR30, outR31, rs, regRs);
    mux32to1_32bit t (outR0, outR1, outR2, outR3, outR4, outR5, outR6, outR7, outR8, outR9, outR10, outR11, outR12, outR13, outR14, outR15, outR16, outR17, outR18, outR19, outR20, outR21, outR22, outR23, outR24, outR25, outR26, outR27, outR28, outR29, outR30, outR31, rt, regRt);


endmodule


module testbench();
    //inputs
    reg clk,reset,regWrite,select;
    reg [4:0] rs,rt,rd1,rd0;
    reg [31:0] writeData;
    //outputs
    wire [31:0] outR0;
    wire [31:0] outR1;
    //instantiate module
    registerFile uut(clk,reset,regWrite,rs,rt,rd0,rd1,writeData,select,outR0,outR1);

    always begin #5 clk=~clk; end
    initial
    begin 
        $dumpfile("2018A7PS0235G_Lab2.vcd"); //replace yourID with your BITS ID here
        $dumpvars(0,testbench);
        clk=0; reset=1; rs=5'd0; rt=5'd1; rd1=5'd0; rd0=5'd2; //reset is active high, so outR0 and outR1 will be 0
        #5 reset=0; select=1; regWrite=1; rd1=5'd1; writeData=32'd1; //1 is written to register 1, since regWrite is active and rd1 is selected
        #10 rd1=5'd3; writeData=32'd3; //3 is written to register 3
        #10 rd1=5'd10; writeData=32'd10; rs=5'd1; rt=5'd3; //10 is written to register 10, outR0 is 1 and outR1 is 3 (values of reg1 and reg3)
        #10 rd1=5'd27; writeData=32'd27; rs=5'd0; rt=5'd10; //27 is written to register 27, outR0 is 0 and outR1 is 10
        #10 select=0; writeData=32'd21; //21 is written to register 2 since select now selects rd0
        #10 rd0=5'd4; writeData=32'd4; //4 is written to register 4
        #10 rd0=5'd16; writeData=32'd16; //16 is written to register 16
        #10 regWrite=0; rs=5'd0; rt=5'd1; //regWrite is 0, no more values will be written. outR0 is 0 and outR1 is 1
        #10 rs=5'd3;rt=5'd2; //outR0 is 3 and outR1 is 21
        #10 rs=5'd10;rt=5'd4; //outR0 is 10 and outR1 is 4
        #10 rs=5'd27;rt=5'd16; //outR0 is 27 and outR1 is 16
        #10 $finish;
    end
endmodule 