//ShifterAndALU.v

// select 0 = in0 1 = in1
module mux2to1_3bit(input [2:0] in0, input [2:0] in1, input select, output reg [2:0] muxOut);
  //WRITE CODE HERE

	/*
	wire selectComplement;
	not (selectComplement, select);

	wire[2:0] temp1;
	wire[2:0] temp2;
	*/

	always @ (in0 or in1 or select)
	begin
	/*
		and (temp1[0], in0[0], selectComplement);
		and (temp1[1], in0[1], selectComplement);
		and (temp1[2], in0[2], selectComplement);

		and (temp2[0], in1[0], select);
		and (temp2[1], in1[1], select);
		and (temp2[2], in1[2], select);

		or (muxOut[0], temp1[0], temp2[0]);
		or (muxOut[1], temp1[1], temp2[1]);
		or (muxOut[2], temp1[2], temp2[2]);
	*/
		/*
		if (select == 1'b0)
		begin
			muxOut = in0;
		end
		else if (select == 1'b1)
		begin
			muxOut = in1;
		end
		*/

		/*
		muxOut[0] = in0[0]&(~select) | in1[0]&select;
		muxOut[1] = in0[1]&(~select) | in1[1]&select;
		muxOut[2] = in0[2]&(~select) | in1[2]&select;
		*/


		case (select)
			1'b0 : muxOut = in0;
			1'b1 : muxOut = in1;
			default : muxOut = 3'bxxx;

		endcase
		
	end
	
endmodule

// select 0 = in0 1 = in1
module mux2to1_8bit(input [7:0] in0, input [7:0] in1, input select, output reg [7:0] muxOut);
   //WRITE CODE HERE

	always @ (in0 or in1 or select)
	begin
		/*
		wire selectComplement;
		not (selectComplement, select);

		wire[7:0] temp1;
		wire[7:0] temp2;

		and (temp1[0], in0[0], selectComplement);
		and (temp1[1], in0[1], selectComplement);
		and (temp1[2], in0[2], selectComplement);
		and (temp1[3], in0[3], selectComplement);
		and (temp1[4], in0[4], selectComplement);
		and (temp1[5], in0[5], selectComplement);
		and (temp1[6], in0[6], selectComplement);
		and (temp1[7], in0[7], selectComplement);

		and (temp2[0], in1[0], select);
		and (temp2[1], in1[1], select);
		and (temp2[2], in1[2], select);
		and (temp2[3], in1[3], select);
		and (temp2[4], in1[4], select);
		and (temp2[5], in1[5], select);
		and (temp2[6], in1[6], select);
		and (temp2[7], in1[7], select);


		or (muxOut[0], temp1[0], temp2[0]);
		or (muxOut[1], temp1[1], temp2[1]);
		or (muxOut[2], temp1[2], temp2[2]);
		or (muxOut[3], temp1[3], temp2[3]);
		or (muxOut[4], temp1[4], temp2[4]);
		or (muxOut[5], temp1[5], temp2[5]);
		or (muxOut[6], temp1[6], temp2[6]);
		or (muxOut[7], temp1[7], temp2[7]);
		*/

		/*
		muxOut[0] = in0[0]&(~select) | in1[0]&select;
		muxOut[1] = in0[1]&(~select) | in1[1]&select;
		muxOut[2] = in0[2]&(~select) | in1[2]&select;
		muxOut[3] = in0[3]&(~select) | in1[3]&select;
		muxOut[4] = in0[4]&(~select) | in1[4]&select;
		muxOut[5] = in0[5]&(~select) | in1[5]&select;
		muxOut[6] = in0[6]&(~select) | in1[6]&select;
		muxOut[7] = in0[7]&(~select) | in1[7]&select;
		*/
		/*
		if (select == 0)
			muxOut = in0;
		
		else
			muxOut = in1;
		*/
		/*
		if (select == 1'b0)
		begin
			muxOut = in0;
		end
			
		else if (select == 1'b1)
		begin
			muxOut = in1;
		end

		*/

		case (select)
			1'b0 : muxOut = in0;
			1'b1 : muxOut = in1;
			default : muxOut = 8'bxxxxxxxx;

		endcase
	end

endmodule


module mux8to1_1bit(input in0, input in1, input in2, input in3, input in4, input in5, input in6, input in7, input[2:0] select, output reg muxOut);
   //WRITE CODE HERE
		
	always @ (in0 or in1 or in2 or in3 or in4 or in5 or in6 or in7 or select) 
	begin 
		case (select) 

			3'b000 : muxOut = in0; 

			3'b001 : muxOut = in1; 

			3'b010 : muxOut = in2; 

			3'b011 : muxOut = in3; 

			3'b100 : muxOut = in4; 

			3'b101 : muxOut = in5; 

			3'b110 : muxOut = in6; 

			3'b111 : muxOut = in7; 

			default : muxOut = 1'bx;//in0; 

		endcase 
	end  

endmodule

module barrelshifter(input[2:0] shiftAmt, input[7:0] b, input[2:0] oper, output[7:0] shiftOut);	
		//WRITE CODE HERE
	   // for self reference
	   // b is the 8 bit input
	   // b -> shiftOut
	   
		//always @ (shiftAmt or b or oper)
		//begin

		wire[2:0] temp0;	//temp[i] serves as the 3 bit select input to column i of multiplexers
		wire[2:0] temp1;
		wire[2:0] temp2;

		wire[7:0] s;
		wire[7:0] r;
		wire[7:0] o;

		//end


		mux2to1_3bit m0 (3'b000, oper, shiftAmt[0], temp0);
		mux2to1_3bit m1 (3'b000, oper, shiftAmt[1], temp1);
		mux2to1_3bit m2 (3'b000, oper, shiftAmt[2], temp2);


		//mux[i][j] is denotes column i (0,1,2) jth mux (0..7)

		//NOTE : 6, 7 always set to z. (dont care actually)

		//			*		0*	  1*	2*	  3*	4*    5*	6*	 7*	   sel*	 o/p*
		mux8to1_1bit m00 (b[0], b[1], b[1], b[1], 1'b0, b[7], 1'bz, 1'bz, temp0, s[0]);
		mux8to1_1bit m01 (b[1], b[2], b[2], b[2], b[0], b[0], 1'bz, 1'bz, temp0, s[1]);
		mux8to1_1bit m02 (b[2], b[3], b[3], b[3], b[1], b[1], 1'bz, 1'bz, temp0, s[2]);
		mux8to1_1bit m03 (b[3], b[4], b[4], b[4], b[2], b[2], 1'bz, 1'bz, temp0, s[3]);
		mux8to1_1bit m04 (b[4], b[5], b[5], b[5], b[3], b[3], 1'bz, 1'bz, temp0, s[4]);
		mux8to1_1bit m05 (b[5], b[6], b[6], b[6], b[4], b[4], 1'bz, 1'bz, temp0, s[5]);
		mux8to1_1bit m06 (b[6], b[7], b[7], b[7], b[5], b[5], 1'bz, 1'bz, temp0, s[6]);
		mux8to1_1bit m07 (b[7], b[7], 1'b0, b[0], b[6], b[6], 1'bz, 1'bz, temp0, s[7]);

		//			*		0*	  1*	2*	  3*	4*    5*	6*	 7*	   sel*	 o/p*
		mux8to1_1bit m10 (s[0], s[2], s[2], s[2], 1'b0, s[6], 1'bz, 1'bz, temp1, r[0]);	//*
		mux8to1_1bit m11 (s[1], s[3], s[3], s[3], 1'b0, s[7], 1'bz, 1'bz, temp1, r[1]);	//*
		mux8to1_1bit m12 (s[2], s[4], s[4], s[4], s[0], s[0], 1'bz, 1'bz, temp1, r[2]);
		mux8to1_1bit m13 (s[3], s[5], s[5], s[5], s[1], s[1], 1'bz, 1'bz, temp1, r[3]);
		mux8to1_1bit m14 (s[4], s[6], s[6], s[6], s[2], s[2], 1'bz, 1'bz, temp1, r[4]);	//changed 4566 to 4666
		mux8to1_1bit m15 (s[5], s[7], s[7], s[7], s[3], s[3], 1'bz, 1'bz, temp1, r[5]);
		mux8to1_1bit m16 (s[6], s[7], 1'b0, s[0], s[4], s[4], 1'bz, 1'bz, temp1, r[6]);
		mux8to1_1bit m17 (s[7], s[7], 1'b0, s[1], s[5], s[5], 1'bz, 1'bz, temp1, r[7]);

		//			*		0*	  1*	2*	  3*	4*    5*	6*	 7*	   sel*	 o/p*
		mux8to1_1bit m20 (r[0], r[4], r[4], r[4], 1'b0, r[4], 1'bz, 1'bz, temp2, o[0]);	//*
		mux8to1_1bit m21 (r[1], r[5], r[5], r[5], 1'b0, r[5], 1'bz, 1'bz, temp2, o[1]); //*
		mux8to1_1bit m22 (r[2], r[6], r[6], r[6], 1'b0, r[6], 1'bz, 1'bz, temp2, o[2]); //*
		mux8to1_1bit m23 (r[3], r[7], r[7], r[7], 1'b0, r[7], 1'bz, 1'bz, temp2, o[3]); //*
		mux8to1_1bit m24 (r[4], r[7], 1'b0, r[0], r[0], r[0], 1'bz, 1'bz, temp2, o[4]);	//*
		mux8to1_1bit m25 (r[5], r[7], 1'b0, r[1], r[1], r[1], 1'bz, 1'bz, temp2, o[5]); //*
		mux8to1_1bit m26 (r[6], r[7], 1'b0, r[2], r[2], r[2], 1'bz, 1'bz, temp2, o[6]); //*	
		mux8to1_1bit m27 (r[7], r[7], 1'b0, r[3], r[3], r[3], 1'bz, 1'bz, temp2, o[7]); //*
		/*
		always @ (shiftAmt or b or oper)
		begin
		if (o == 3'bxxx)
			assign shiftOut = shiftAmt;
		
		else
		*/
		assign shiftOut = o;
		//end

endmodule

// Alu operations are: 00 for alu1, 01 for add, 10 for sub(alu1-alu2) , 11 for AND, 100 for OR and 101 for NOT(alu1)
module alu(input [7:0] aluIn1, input [7:0] aluIn2, input [2:0]aluOp, output reg [7:0] aluOut);
       //WRITE CODE HERE
	   
		always @ (aluIn1 or aluIn2 or aluOp)
		begin
			case (aluOp)

				3'b000 :	aluOut = aluIn1;
			
				3'b001 :	aluOut = aluIn1 + aluIn2;

				3'b010 : aluOut = aluIn1 - aluIn2;

				3'b011 : aluOut = aluIn1 & aluIn2;
				
				3'b100 : aluOut = aluIn1 | aluIn2;

				3'b101 : aluOut = ~aluIn1;

				default: aluOut = 8'bxxxxxxxx;

			endcase
		end
endmodule


module shifterAndALU(input [7:0]inp1, input [7:0] inp2, input [2:0]shiftlmm, input selShiftAmt, input [2:0]oper, input selOut, output [7:0] out);
	   //WRITE CODE HERE

		wire[7:0] aluOut1;
		wire[7:0] shiftOut1;
		wire[2:0] shiftAmt1;

		//always @ (inp1 or inp2 or shiftlmm or selShiftAmt or oper or selOut)
		//begin
			
		
		alu ALU1 (inp1, inp2, oper, aluOut1);

		

		mux2to1_3bit muxtemp (inp2[2:0], shiftlmm, selShiftAmt, shiftAmt1);


		barrelshifter BS (shiftAmt1, inp1, oper, shiftOut1);

		mux2to1_8bit finalmux (aluOut1, shiftOut1, selOut, out);

		//end
endmodule

//TestBench ALU

module testbenchALU();
	// Input
	reg [7:0] inp1, inp2;
	reg [2:0] aluOp;
	reg [2:0] shiftlmm;
	reg selShiftAmt, selOut;
	// Output
	wire [7:0] aluOut;

	shifterAndALU shifterAndALU_Test (inp1, inp2, shiftlmm, selShiftAmt, aluOp, selOut, aluOut);

	initial
		begin
			$dumpfile("testALU.vcd");
     		$dumpvars(0,testbenchALU);
			inp1 = 8'd80; //80 in binary is 1010000
			inp2 = 8'd20; //20 in binary is 10100   
			shiftlmm = 3'b010; 
			
			aluOp=3'd0; selOut = 1'b0;// normal output = 80

			#10 aluOp = 3'd0; selOut = 1'b1; selShiftAmt = 1'b1; //No shift output = 80

			#10 aluOp=3'd1; selOut = 1'b0;// normal add	output => 20 + 80 = 100

			#10 aluOp = 3'd1; selOut = 1'b1; selShiftAmt = 1'b1; // arithmetic shift right of 80 by 2 places = 20

			#10 aluOp=3'd2; selOut = 1'b0; // normal sub output => 80 - 20 = 60

			#10 aluOp = 3'd2; selOut = 1'b1; selShiftAmt = 1'b0; // logical shift right of 80 by 4 places = 0

			#10 aluOp=3'd3; selOut = 1'b0;// normal and output => 80 & 20 = 16

			#10 aluOp = 3'd3; selOut = 1'b1; selShiftAmt = 1'b0; // Circular Shift Right of 80 by 4 places = 5

			#10 aluOp=3'd4; selOut = 1'b0;// normal or output => 80 | 20 = 84

			#10 aluOp = 3'd4; selOut = 1'b1; selShiftAmt = 1'b1; // Logical Shift Left of 80 by 2 bits = 64

			#10 aluOp=3'd5; selOut = 1'b0; // normal not of 80 = 175

			#10 aluOp = 3'd5; selOut = 1'b1; selShiftAmt = 1'b0; // Circular left shift of 80 by 4 bits = 5

			#10 inp1=8'd15; inp2=8'd26; aluOp=3'd2; selOut = 1'b0;//sub overflow 
			// (15 - 26) = -11 and its a 8 bit number so, 256-11 = 245 output => 245 (since it is unsigned decimal)

			#10 inp1=8'd150; inp2=8'd150; aluOp=3'd1; selOut = 1'b0;// add overflow
			//(150+150) = 300 and its a 8 bit number so, 300-256 = 44 output=> 44.

			#10 inp1=8'b0000_0000; aluOp=3'd5; selOut = 1'b0;//not overflow
			// not(0) = all 1. Since its a 8 bit number output=>255

			#10 $finish;
		end

endmodule
 
