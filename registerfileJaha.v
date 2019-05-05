/*
Author: Jahannie Torres-Rodríguez
  Requirements:
	-PA,PB,PC
    -SA,SB,SC
    -Lds
    -Binary Decoder (1)
    -Registers (32)
    -Mux (2)1
*/




module registerFile(out_PA, out_PB, in_PC, in_SA, in_SB, in_SC, in_RFL, in_clk);
	output [31:0] out_PA;
	output [31:0] out_PB;
	 input [31:0] in_PC;
	 input [4:0] in_SA;
	 input [4:0] in_SB;
	 input [4:0] in_SC;

	 input in_RFL;

	 input in_clk;

  	 //Lds
     wire[31:0] Ld;

  	//Binary Decoder
     decoder_5x32 binaryDecoder(in_SC, Ld, in_RFL);


	// 32 Registers
	wire[31:0] Q0;
	wire[31:0] Q1;
	wire[31:0] Q2;
	wire[31:0] Q3;
	wire[31:0] Q4;
	wire[31:0] Q5;
	wire[31:0] Q6;
	wire[31:0] Q7;
	wire[31:0] Q8;
	wire[31:0] Q9;
wire[31:0] Q10;
	wire[31:0] Q11;
	wire[31:0] Q12;
	wire[31:0] Q13;
	wire[31:0] Q14;
	wire[31:0] Q15;
	wire[31:0] Q16;
	wire[31:0] Q17;
wire[31:0] Q18;
	wire[31:0] Q19;
	wire[31:0] Q20;
	wire[31:0] Q21;
	wire[31:0] Q22;
	wire[31:0] Q23;
	wire[31:0] Q24;
	wire[31:0] Q25;
	wire[31:0] Q26;
	wire[31:0] Q27;
	wire[31:0] Q28;
	wire[31:0] Q29;
wire[31:0] Q30;
	wire[31:0] Q31;

      registers r00 (Q0, 32'b0000000000000000000000, Ld[0], in_clk);
      registers r01 (Q1, in_PC, Ld[1], in_clk);
      registers r02 (Q2, in_PC, Ld[2], in_clk);
      registers r03 (Q3, in_PC, Ld[3], in_clk);
      registers r04 (Q4, in_PC, Ld[4], in_clk);
      registers r05 (Q5, in_PC, Ld[5], in_clk);
      registers r06 (Q6, in_PC, Ld[6], in_clk);
      registers r07 (Q7, in_PC, Ld[7], in_clk);
      registers r08 (Q8, in_PC, Ld[8], in_clk);
      registers r09 (Q9, in_PC, Ld[9], in_clk);
      registers r10 (Q10, in_PC, Ld[10], in_clk);
      registers r11 (Q11, in_PC, Ld[11], in_clk);
      registers r12 (Q12, in_PC, Ld[12], in_clk);
      registers r13 (Q13, in_PC, Ld[13], in_clk);
      registers r14 (Q14, in_PC, Ld[14], in_clk);
      registers r15 (Q15, in_PC, Ld[15], in_clk);
      registers r16 (Q16, in_PC, Ld[16], in_clk);
      registers r17 (Q17, in_PC, Ld[17], in_clk);
      registers r18 (Q18, in_PC, Ld[18], in_clk);
      registers r19 (Q19, in_PC, Ld[19], in_clk);
      registers r20 (Q20, in_PC, Ld[20], in_clk);
      registers r21 (Q21, in_PC, Ld[21], in_clk);
      registers r22 (Q22, in_PC, Ld[22], in_clk);
      registers r23 (Q23, in_PC, Ld[23], in_clk);
      registers r24 (Q24, in_PC, Ld[24], in_clk);
      registers r25 (Q25, in_PC, Ld[25], in_clk);
      registers r26 (Q26, in_PC, Ld[26], in_clk);
      registers r027 (Q27, in_PC, Ld[27], in_clk);
      registers r28 (Q28, in_PC, Ld[28], in_clk);
      registers r29 (Q29, in_PC, Ld[29], in_clk);
      registers r30 (Q30, in_PC, Ld[30], in_clk);
      registers r31 (Q31, in_PC, Ld[31], in_clk);

	//Mux
mux mux_A(Q0, Q1, Q2, Q3, Q4, Q5, Q6, Q7, Q8, Q9, Q10, Q11, Q12, Q13, Q14, Q15, Q16, Q17, Q18, Q19, Q20, Q21, Q22, Q23, Q24, Q25, Q26, Q27, Q28, Q29, Q30, Q31, in_SA, out_PA);

mux mux_B(Q0, Q1, Q2, Q3, Q4, Q5, Q6, Q7, Q8, Q9, Q10, Q11, Q12, Q13, Q14, Q15, Q16, Q17, Q18, Q19, Q20, Q21, Q22, Q23, Q24, Q25, Q26, Q27, Q28, Q29, Q30, Q31, in_SB, out_PB);

	endmodule

	//Binary Decoder

	module decoder_5x32 (in_D , out_E, in_Ld);
		 input [4:0] in_D;
		 output reg [31:0] out_E ;
		 input in_Ld;

		 always @ (in_Ld)
		 begin
		  out_E= 0;
		  if (in_Ld == 1'b1) begin
		   case (in_D)
			5'b00000 : out_E = 32'b00000000000000000000000000000001;
			5'b00001 : out_E = 32'b00000000000000000000000000000010;
			5'b00010 : out_E = 32'b00000000000000000000000000000100;
			5'b00011 : out_E = 32'b00000000000000000000000000001000;
			5'b00100 : out_E = 32'b00000000000000000000000000010000;
			5'b00101 : out_E = 32'b00000000000000000000000000100000;
			5'b00110 : out_E = 32'b00000000000000000000000001000000;
			5'b00111 : out_E = 32'b00000000000000000000000010000000;
			5'b01000 : out_E = 32'b00000000000000000000000100000000;
			5'b01001 : out_E = 32'b00000000000000000000001000000000;
			5'b01010 : out_E = 32'b00000000000000000000010000000000;
			5'b01011 : out_E = 32'b00000000000000000000100000000000;
			5'b01100 : out_E = 32'b00000000000000000001000000000000;
			5'b01101 : out_E = 32'b00000000000000000010000000000000;
			5'b01110 : out_E = 32'b00000000000000000100000000000000;
			5'b01111 : out_E = 32'b00000000000000001000000000000000;
			5'b10000 : out_E = 32'b00000000000000010000000000000000;
			5'b10001 : out_E = 32'b00000000000000100000000000000000;
			5'b10010 : out_E = 32'b00000000000001000000000000000000;
		 	5'b10011 : out_E = 32'b00000000000010000000000000000000;
		    5'b10100 : out_E = 32'b00000000000100000000000000000000;
		    5'b10101 : out_E = 32'b00000000001000000000000000000000;
		    5'b10110 : out_E = 32'b00000000010000000000000000000000;
			5'b10111 : out_E = 32'b00000000100000000000000000000000;
		    5'b11000 : out_E = 32'b00000001000000000000000000000000;
		    5'b11001 : out_E = 32'b00000010000000000000000000000000;
			5'b11010 : out_E = 32'b00000100000000000000000000000000;
		    5'b11011 : out_E = 32'b00001000000000000000000000000000;
		    5'b11100 : out_E = 32'b00010000000000000000000000000000;
			5'b11101 : out_E = 32'b00100000000000000000000000000000;
		    5'b11110 : out_E = 32'b01000000000000000000000000000000;
			5'b11111 : out_E = 32'b10000000000000000000000000000000;
		 endcase
		end
	   end
	endmodule


	//Registers

	module registers(out_Q, in_D, Ld, clk);
			 output reg[31:0] out_Q;
			 input [31:0] in_D;
			 input Ld;
			 input clk;

			 always @(posedge clk)
			 begin
				if (Ld)
				begin
				out_Q <= in_D;
				end
			 end
	endmodule

	//Mux

	module mux(in_data0, in_data1, in_data2, in_data3, in_data4, in_data5, in_data6, in_data7, in_data8, in_data9, in_data10, in_data11, in_data12, in_data13, in_data14, in_data15, in_data16, in_data17, in_data18, in_data19, in_data20, in_data21, in_data22, in_data23, in_data24, in_data25, in_data26, in_data27, in_data28, in_data29, in_data30, in_data31, select, out_data);
			input [31:0] in_data0;
			input [31:0] in_data1;
			input [31:0] in_data2;
			input [31:0] in_data3;
			input [31:0] in_data4;
			input [31:0] in_data5;
			input [31:0] in_data6;
			input [31:0] in_data7;
			input [31:0] in_data8;
			input [31:0] in_data9;
			input [31:0] in_data10;
			input [31:0] in_data11;
			input [31:0] in_data12;
			input [31:0] in_data13;
			input [31:0] in_data14;
			input [31:0] in_data15;
			input [31:0] in_data16;
			input [31:0] in_data17;
			input [31:0] in_data18;
			input [31:0] in_data19;
			input [31:0] in_data20;
			input [31:0] in_data21;
			input [31:0] in_data22;
			input [31:0] in_data23;
			input [31:0] in_data24;
			input [31:0] in_data25;
			input [31:0] in_data26;
			input [31:0] in_data27;
			input [31:0] in_data28;
			input [31:0] in_data29;
			input [31:0] in_data30;
			input [31:0] in_data31;
			input [4:0] select;
			output reg [31:0] out_data;

			always @(in_data0, in_data1, in_data2, in_data3, in_data4, in_data5, in_data6, in_data7, in_data8, in_data9, in_data10, in_data11, in_data12, in_data13, in_data14, in_data15, in_data16, in_data17, in_data18, in_data19, in_data20, in_data21, in_data22, in_data23, in_data24, in_data25, in_data26, in_data27, in_data28, in_data29, in_data30, in_data31, select)
			//always @(*)
			begin
			case (select)
			5'b00000 : out_data = in_data0;
			5'b00001 : out_data = in_data1;
			5'b00010 : out_data = in_data2;
			5'b00011 : out_data = in_data3;
			5'b00100 : out_data = in_data4;
			5'b00101 : out_data = in_data5;
			5'b00110 : out_data = in_data6;
			5'b00111 : out_data = in_data7;
			5'b01000 : out_data = in_data8;
			5'b01001 : out_data = in_data9;
			5'b01010 : out_data = in_data10;
			5'b01011 : out_data = in_data11;
			5'b01100 : out_data = in_data12;
			5'b01101 : out_data = in_data13;
			5'b01110 : out_data = in_data14;
			5'b01111 : out_data = in_data15;
			5'b10000 : out_data = in_data16;
			5'b10001 : out_data = in_data17;
			5'b10010 : out_data = in_data18;
			5'b10011 : out_data = in_data19;
			5'b10100 : out_data = in_data20;
			5'b10101 : out_data = in_data21;
			5'b10110 : out_data = in_data22;
			5'b10111 : out_data = in_data23;
			5'b11000 : out_data = in_data24;
			5'b11001 : out_data = in_data25;
			5'b11010 : out_data = in_data26;
			5'b11011 : out_data = in_data27;
      			5'b11100 : out_data = in_data28;
			5'b11101 : out_data = in_data29;
			5'b11110 : out_data = in_data30;
			5'b11111 : out_data = in_data31;
	  endcase
	  end
	endmodule


//-----------------TEST-----------------

// module RegFile_test;
//   wire [31:0] out_PA;
//   wire [31:0] out_PB;
//   reg [31:0] in_PC;
//   reg [4:0] in_SA;
//   reg [4:0] in_SB;
//   reg [4:0] in_SC;
//   reg in_RFL;
//   reg in_clk;
  
  


// registerFile regFile(out_PA, out_PB, in_PC, in_SA, in_SB, in_SC, in_RFL, in_clk);

 
  
// initial begin
    
    
//     in_PC = 32'b00000000000000000000000000000100;
//     in_SC = 32'b00000000000000000000000000000100;
// 	#5 in_RFL= 1;
//     #5 in_clk= 1;    
    
//   	#5 in_RFL= 0;
//     #5 in_clk= 0; 
  
//     in_SA = 32'b00000000000000000000000000000100;
//     in_SB = 32'b00000000000000000000000000000011;
    
//  	#5	in_RFL= 1;
//   	#5  in_clk= 1;   
    
//     $display ("DATA en out_PA %b reg0 %d reg1 %d", out_PA, regFile.Q0, regFile.Q1);
//     $display ("DATA en out_PB %b reg0 %d reg1 %d", out_PB, regFile.Q0, regFile.Q1);
//         $display(" \n\nReg0 %d  reg1 %d reg2 %d reg3 %d reg4 %d reg5 %d\n \n\nReg6 %d  reg7 %d reg8 %d reg9 %d reg10 %d reg11 %d\n \n\nReg12 %d  reg13 %d reg14 %d reg15 %d reg16 %d reg17 %d\n \n\nReg18 %d  reg19 %d reg20 %d reg21 %d reg22 %d reg23 %d\n", regFile.Q0,regFile.Q1,regFile.Q2,regFile.Q3,regFile.Q4,regFile.Q5, regFile.Q6,regFile.Q7,regFile.Q8,regFile.Q9,regFile.Q10,regFile.Q11, regFile.Q12,regFile.Q13,regFile.Q14,regFile.Q15,regFile.Q16,regFile.Q17, regFile.Q18,regFile.Q19,regFile.Q20,regFile.Q21,regFile.Q22,regFile.Q23);

//     #5 in_RFL= 0;
//     #5 in_clk= 0; 
  
//     in_PC = 32'b11111111111111111111111111111111;
//     in_SC = 32'b00000000000000000000000000000011;
// 	#5 in_RFL= 1;
//     #5 in_clk= 1;    
    
//   	#5 in_RFL= 0;
//     #5 in_clk= 0; 
  
//     in_SA = 32'b00000000000000000000000000000100;
//     in_SB = 32'b00000000000000000000000000000011;
    
//  	#5	in_RFL= 1;
//   	#5  in_clk= 1;   

//     $display ("DATA en out_PA %b reg0 %d reg1 %d", out_PA, regFile.Q0, regFile.Q1);
//     $display ("DATA en out_PB %b reg0 %d reg1 %d", out_PB, regFile.Q0, regFile.Q1);

// end

// endmodule
