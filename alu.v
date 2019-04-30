//steph
module alu(
           input [31:0] A,B,             
           input [5:0] opcode,
           output reg [31:0] out,
           input inputCarry,
           output reg negativeFlag, zeroFlag, carryFlag, overflowFlag     
    );
    always @(*)
    begin
        case(opcode)
        6'b000000: // ADD
            begin
            {carryFlag, out} = A + B; 
             overflowFlag = ((A[31] == B[31]) && (A[31] != out[31])) ? 1'b1 : 1'b0;
            end
        6'b000000: // ADDU
            begin
            {carryFlag, out} = A + B; 
            end
        6'b001000: //ADDI 
           begin
            {carryFlag, out} = A + 16'b0000000000000000; 
            overflowFlag = ((A[31] == B[31]) && (A[31] != out[31])) ? 1'b1 : 1'b0;
            end
        6'b000101: //ADDIU
           begin
            {carryFlag, out} = A + 16'b0000000000000000; 
            end
        6'b000001: // SUB fixed
            begin
            {carryFlag, out} = A - B;
            overflowFlag = (A[31] == 0 && B[31] == 1 && out[31] == 1) || (A[31] == 1 && B[31] == 0 && out[31] == 0);
            end
        6'b000001: // SUBU
            begin
            {carryFlag, out} = A - B;
            overflowFlag = (A[31] == 0 && B[31] == 1 && out[31] == 1) || (A[31] == 1 && B[31] == 0 && out[31] == 0);
            end
        6'b000001: // MULTU
            begin
            out = A * B;
            end
        6'b000010: //  SLT 
            begin
            if(A < B) 
            begin
                out = 1'b1;
            end
            else out = 1'b0;
            end
        6'b000010: //  SLTU
            begin
            if(A < B) 
            begin
                out = 1'b1;
            end
            else out = 1'b0;
            end    
        6'b000010: //  SLTI ??
            begin
            if(A <  16'b0000000000000000) 
            begin
                out = 1'b1;
            end
            else out = 1'b0;
            end
        6'b000010: //  SLTIU ??
            begin
            if(A <  16'b0000000000000000) 
            begin
                out = 1'b1;
            end
            else out = 1'b0;
            end
        6'b000010: //  CLO ??
            begin
            if(A < B) 
            begin
                out = 1'b0;
            end
            else out = 0;
            end
        6'b000010: //  CLZ ??
            begin
            if(A < B) 
            begin
                out = 1;
            end
            else out = 0;
            end
        6'b000010: //  AND 
            begin
            out =  A & B; 
            end
        6'b000010: //  ANDI ??
            begin
            out = A & 16'b0000000000000000;
            end
        6'b000011: //  OR
            begin
            out =  A | B; 
            end
        6'b000011: //  ORI ??
            begin
            out = A |  16'b0000000000000000;  
            end
        6'b000100: //  XOR
           begin
            out =  A ^ B;        
            end
        6'b000100: //  XORI ??
           begin
            out = A ^ 16'b0000000000000000;  
            end
        6'b001001: // NOR
           begin
            out = ~(A | B); 
            end
        6'b001001: // LUI ??
           begin
            out = ~(A | B); 
            end
        6'b000110: // SLL
           begin
            out = A << B; 
            end
        6'b000110: // SLLV
          begin
            out = A << B; 
            end
        6'b000111: // SRA
           begin
            out = A >> B; 
  
            end
        6'b000111: // SRAV
           begin
            out = A >> B; 
  
            end
        6'b000111: // SRL 
           begin
            out = A >> B; 
  
            end
        6'b000111: // SRLV
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
 reg[5:0] opcode;
 wire negativeFlag, zeroFlag, carryFlag, overflowFlag;

//Outputs
 wire[31:0] out;
 integer i;
 alu test_unit(A,B,opcode,out,inputCarry,negativeFlag,zeroFlag,carryFlag,overflowFlag);
    initial begin
  
        A = 32'h06;
        B = 32'h0A;  
        //ADD TEST
        opcode = 6'b000000;
        #10;
        #3 $display("Opcode: %b, Binary Output: %b, Decimal Output: %d, Negative Flag: %d, Zero Flag: %d, Carry Flag: %d, Overflow Flag: %d",opcode,out,out,negativeFlag,zeroFlag,carryFlag,overflowFlag);
        #10;
        //SUB TEST
        opcode = 6'b000001;
          #3 $display("Opcode: %b, Binary Output: %b, Decimal Output: %d, Negative Flag: %d, Zero Flag: %d, Carry Flag: %d, Overflow Flag: %d",opcode,out,out,negativeFlag,zeroFlag,carryFlag,overflowFlag);
        #10;
        //AND TEST
        opcode = 6'b000010;
        #3 $display("Opcode: %b, Binary Output: %b, Decimal Output: %d, Zero Flag: %d",opcode,out,out,zeroFlag);
        #10;
        //OR TEST
        opcode = 6'b000011;
        #3 $display("Opcode: %b, Binary Output: %b, Decimal Output: %d, Zero Flag: %d",opcode,out,out,zeroFlag);
        #10;
        //XOR TEST
        opcode = 6'b000100;
       #3 $display("Opcode: %b, Binary Output: %b, Decimal Output: %d, Zero Flag: %d",opcode,out,out,zeroFlag);
        #10;
        //NOR TEST
        opcode = 6'b001001;
      #3 $display("Opcode: %b, Binary Output: %b, Decimal Output: %d, Zero Flag: %d",opcode,out,out,zeroFlag);
        #10;
        //SLL TEST
        opcode = 6'b000110;
        #3 $display("Opcode: %b, Binary Output: %b, Decimal Output: %d, Zero Flag: %d",opcode,out,out,zeroFlag);
        #10;
         //SRL TEST
        opcode = 6'b000111;
         #3 $display("Opcode: %b, Binary Output: %b, Decimal Output: %d, Zero Flag: %d",opcode,out,out,zeroFlag);
        #10;
    end  
endmodule