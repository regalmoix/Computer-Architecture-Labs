`include "alu.v"
`include "control.v"
`include "dff.v"
`include "memory.v"
`include "mux.v"
`include "register_file.v"
`include "registers.v"
`include "sign_ext.v"

module multi_cycle (input clk, input reset, output [31:0] result);
    
    // Write your code here

    // make sure that the im module is instantiated as "instruction memory"  

    //PC
    wire [31:0]     inPC;
    wire [31:0]     outPC;
    wire            regWrPC;

    //MUX
    wire [4:0]      PC_IM_MUX_Out;
    wire [31:0]     A_ALU_MUX_IN_1;
    wire [31:0]     B_ALU_MUX_IN_0;


    //ALU
    wire [31:0]     inALUA;
    wire [31:0]     inALUB;
    wire [31:0]     ALU_OUT0;
    wire [31:0]     ALU_OUT1;
    wire            zero;
    
    //Control 
    wire            IorD; 
    wire            memRead; 
    wire            IRWrite; 
    wire            regDest; 
    wire            regWrite;
    wire            aluSrcA;
    wire [1:0]      aluSrcB; 
    wire [1:0]      aluOp;
    wire            hiWrite;
    wire            loWrite; 
    wire [1:0]      memToReg; 
    wire [1:0]      pcSrc;
    wire            pcWrite;
    wire            branch;


    //IR
    wire [31:0]     IRout;
    wire [5:0]      opcode;
    wire [5:0]      funct;
    wire [15:0]     immediate;


    //DM
    wire [31:0]     DMout;


    //MDR
    wire [31:0]     MDRout;


    //Temps
    wire            bz_and;
    wire[31:0]      Out0bus;
    wire[31:0]      Out1bus;
    wire[31:0]      result;
    wire[31:0]      HIOut;
    wire[31:0]      LOOut;
    wire[31:0]      JUMP_ADDR;

    //Inst Mem
    wire [31:0]     imOut;

    //SignExt16->32
    wire [31:0]     extOp;

    //RegisterFile
    wire [4:0]      rd;
    wire [4:0]      rs;
    wire [4:0]      rt;
    wire [31:0]     writeData;
    wire [31:0]     rf_outA;
    wire [31:0]     rf_outB;





    and a1(bz_and, branch, zero);
    or o1(regWrPC, bz_and, pcWrite);

    //Instance of PC 
    intermediate_reg #(32) PC (clk, reset, regWrPC, inPC, outPC);


    
    //The mux after PC
    mux2to1 #(5) PC_IM_MUX (outPC[6:2], Out0bus[6:2], IorD, PC_IM_MUX_Out);


    //Instance of Instruction Memory 
    im #(32, 5) instruction_memory (clk, reset, PC_IM_MUX_Out, memRead, imOut);

    //IR
    intermediate_reg #(32) IR (clk, reset, IRWrite, imOut, IRout);
    assign opcode = IRout[31:26];
    assign funct = IRout[5:0];
    assign immediate = IRout[15:0];

    assign rs = IRout[25:21];
    assign rt = IRout[20:16];

    //Control Unit
    control_circuit CU (clk, reset, opcode, funct, 
                        IorD, memRead, IRWrite, regDest, regWrite, 
                        aluSrcA, aluSrcB, aluOp, 
                        hiWrite, loWrite, memToReg, 
                        pcSrc, pcWrite, branch);


    //MDR
    intermediate_reg #(32) MDR (clk, reset, 1'b1, imOut, MDRout);


    //muxIR-RF
    mux2to1 #(5) muxIR_RF (rt, IRout[15:11], regDest, rd);

    //RegFile
    register_file RF (clk, reset, regWrite, rs, rt, rd, writeData, rf_outA, rf_outB);

    //Registers A and B
    intermediate_reg #(32) A (clk, reset, 1'b1, rf_outA, A_ALU_MUX_IN_1);
    intermediate_reg #(32) B (clk, reset, 1'b1, rf_outB, B_ALU_MUX_IN_0);


    //mux A->ALU
    mux2to1 #(32) A_ALU_MUX (outPC, A_ALU_MUX_IN_1, aluSrcA, inALUA);

    //signext
    sign_ext signext (immediate, extOp);
    
    //mux B->ALU
    mux4to1 #(32) B_ALU_MUX (B_ALU_MUX_IN_0, 32'd4, extOp, {extOp[29:0],2'b00}, aluSrcB, inALUB);



    //ALU
    alu ALU1 (inALUA, inALUB, aluOp, ALU_OUT0, ALU_OUT1, zero);

    //Out Regs
    intermediate_reg Out0 (clk, reset, 1'b1, ALU_OUT0, Out0bus);
    intermediate_reg Out1 (clk, reset, 1'b1, ALU_OUT1, Out1bus);

    assign result = Out0bus;


    //JUMP MUX
    assign JUMP_ADDR = {outPC[31:28], IRout[25:0], 2'b00};
    mux4to1 #(32) JUMP_MUX (Out0bus, JUMP_ADDR, ALU_OUT0, 32'd0, pcSrc, inPC);

    //Hi Lo Regs
    intermediate_reg Hi (clk, reset, hiWrite, Out1bus, HIOut);
    intermediate_reg Lo (clk, reset, loWrite, Out0bus, LOOut);

    //mux Hi - RegFile
    mux4to1 #(32) HI_RF_MUX (MDRout, HIOut, Out0bus, 32'd0, memToReg, writeData);



endmodule