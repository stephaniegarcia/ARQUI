//steph
module alu(
           input [31:0] A,B,
           input [5:0] functn,
           output reg [31:0] out,
           input inputCarry,
           output reg negativeFlag, zeroFlag, carryFlag, overflowFlag
    );
    always @(*)
    begin
        case (functn)
        6'b100000: // ADD
            begin
            {carryFlag, out} = A + B;
             overflowFlag = ((A[31] == B[31]) && (A[31] != out[31])) ? 1'b1 : 1'b0;
            end
        6'b100001: // ADDU
            begin
            {carryFlag, out} = A + B;
            end
        6'b001000: //ADDI
           begin
            {carryFlag, out} = A + B;
            overflowFlag = ((A[31] == B[31]) && (A[31] != out[31])) ? 1'b1 : 1'b0;
            end
        6'b000101: //ADDIU
           begin
            {carryFlag, out} = A + B;
            end
        6'b100010: // SUB
            begin
            {carryFlag, out} = A - B;
            overflowFlag = (A[31] == 0 && B[31] == 1 && out[31] == 1) || (A[31] == 1 && B[31] == 0 && out[31] == 0);
            end
        6'b100011: // SUBU
            begin
            {carryFlag, out} = A - B;
            overflowFlag = (A[31] == 0 && B[31] == 1 && out[31] == 1) || (A[31] == 1 && B[31] == 0 && out[31] == 0);
            end
        6'b011001: // MULTU
            begin
            out = A * B;
            end
        6'b101010: //  SLT
            begin
            if(A < B)
            begin
                out = 1'b1;
            end
            else out = 1'b0;
            end
        6'b101011: //  SLTU
            begin
            if(A < B)
            begin
                out = 1'b1;
            end
            else out = 1'b0;
            end
        6'b001010: //  SLTI 
            begin
            if(A < B)
            begin
                out = 1'b1;
            end
            else out = 1'b0;
            end
        6'b001011: //  SLTIU 
            begin
            if(A < B)
            begin
                out = 1'b1;
            end
            else out = 1'b0;
            end
        6'b100001: //  CLO 
            begin
            if(A < B)
            begin
                out = 1'b0;
            end
            else out = 0;
            end
        6'b100000: //  CLZ 
            begin
            if(A < B)
            begin
                out = 1;
            end
            else out = 0;
            end
        6'b100100: //  AND
            begin
            out =  A & B;
            end
        6'b001100: //  ANDI 
            begin
            out = A & B;
            end
        6'b100101: //  OR
            begin
            out =  A | B;
            end
        6'b001101: //  ORI 
            begin
            out = A | B;
            end
        6'b100110: //  XOR
           begin
            out =  A ^ B;
            end
        6'b001110: //  XORI 
           begin
            out = A ^ B;
            end
        6'b100111: // NOR
           begin
            out = ~(A | B);
            end
        6'b001111: // LUI 
           begin
            out = ~(A | B);
            end
        6'b000000: // SLL
           begin
            out = A << B;
            end
        6'b000100: // SLLV
          begin
            out = A << B;
            end
        6'b000011: // SRA
           begin
            out = A >> B;

            end
        6'b000111: // SRAV
           begin
            out = A >> B;

            end
        6'b000010: // SRL
           begin
            out = A >> B;

            end
        6'b000110: // SRLV
           begin
            out = A >> B;
            end
        endcase
    //-----FLAGS------
    negativeFlag = out[31]; //NEGATIVE FLAG
    zeroFlag = out == 0; //ZERO FLAG
    end
endmodule

//---------------------------------------TESTER------------------------------------------
`timescale 1ns / 1ps
module tester;
//Inputs
 reg[31:0] A,B;
 reg[5:0] functn;
 wire negativeFlag, zeroFlag, carryFlag, overflowFlag;

//Outputs
 wire[31:0] out;
 integer i;
 alu test_unit(A,B,functn,out,inputCarry,negativeFlag,zeroFlag,carryFlag,overflowFlag);
    initial begin
        A = 32'b10000000000000000000000000000000;
        B = 32'b10000000000000000000000000000010;
       // A = 32'd4294967295;
        //B = 32'd4;
        //A = 32'h3;
       // B = 32'h4;
        //ADD TEST
        functn = 6'b100000;
        #10;
        #3 $display("A: %b, B: %b,Function: %b, Binary Output: %b, Decimal Output: %d, Negative Flag: %b, Zero Flag: %d, Carry Flag: %d, Overflow Flag: %d",A,B,functn,out,out,negativeFlag,zeroFlag,carryFlag,overflowFlag);
        #10;
    //     //SUB TEST
    //      functn = 6'b000001;
    //       #3 $display(   functn: %b, Binary Output: %b, Decimal Output: %d, Negative Flag: %d, Zero Flag: %d, Carry Flag: %d, Overflow Flag: %d"  functn,out,out,negativeFlag,zeroFlag,carryFlag,overflowFlag);
    //     #10;
    //     //AND TEST
    //      functn = 6'b000010;
    //     #3 $display( functn: %b, Binary Output: %b, Decimal Output: %d, Zero Flag: %d"  functn,out,out,zeroFlag);
    //     #10;
    //     //OR TEST
    //      functn = 6'b000011;
    //     #3 $display( functn: %b, Binary Output: %b, Decimal Output: %d, Zero Flag: %d"  functn,out,out,zeroFlag);
    //     #10;
    //     //XOR TEST
    //      functn = 6'b000100;
    //    #3 $display(  functn: %b, Binary Output: %b, Decimal Output: %d, Zero Flag: %d"   functn,out,out,zeroFlag);
    //     #10;
    //     //NOR TEST
    //      functn = 6'b001001;
    //   #3 $display(   functn: %b, Binary Output: %b, Decimal Output: %d, Zero Flag: %d"    functn,out,out,zeroFlag);
    //     #10;
    //     //SLL TEST
    //      functn = 6'b000110;
    //     #3 $display( functn: %b, Binary Output: %b, Decimal Output: %d, Zero Flag: %d"  functn,out,out,zeroFlag);
    //     #10;
    //      //SRL TEST
    //      functn = 6'b000111;
    //      #3 $display(    functn: %b, Binary Output: %b, Decimal Output: %d, Zero Flag: %d" functn,out,out,zeroFlag);
    //     #10;
    end
endmodule
