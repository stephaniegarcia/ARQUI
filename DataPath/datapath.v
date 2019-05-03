module datapath_tb;
	reg Clk = 0;
	reg Clear;
	integer code, count;
	reg [7:0] data;
initial begin
Clear = 1'b1;
#50 Clear = 1'b0;
end
	datapath cpu(Clear, Clk);
	integer fo;
	integer index;
	
	//Precharge memory
initial begin
	// $display("--FETCHING INSTRUCTIONS--");
	$display("  testcode_mips1.txt");
    fo = $fopen("testcode_mips1.txt","r"); 
	count = 9'd0;
	index = 0;
	
	while (!$feof(fo)) begin
		code = $fscanf(fo, "%b", data);
		cpu.RAM.Mem[index]=data;
		index = index + 1;
	end
  cpu.RAM.MOC = 0;
	$fclose(fo);
	$display("--FETCHING FINISHED--");
end 

initial begin

  $monitor("State: %d mar_out %d pc_out %d npc_out %d  IR  %b", cpu.st, cpu.MAR_out, cpu.PC_out, cpu.nPC_out, cpu.IR_out);

	while(count < 268) begin 
      $display("Mem[ %d ]= %b    MOC  %b", count, cpu.RAM.Mem[count], cpu.RAM.MOC);
			count = count + 9'd1;

	end
end


initial begin
Clk =1'b0;
forever #5 Clk = ~Clk;
end
	
	 // sim time
parameter sim_time = 9000;
initial #sim_time $finish;
	
endmodule


module ram512x8(output reg [31:0] DataOut, output reg MOC, input ReadWrite, MOV,input[31:0] Address, 
				 input[31:0] DataIn,
				 input [5:0] OP //gives word type (byte, halfword, word)
				 ); 
	reg[7:0] Mem[0:511]; //512 bytes of memory total
	always @(posedge MOV, Address, ReadWrite)
		begin
			if(MOV)
			begin
				MOC = 1'b0;
				if(ReadWrite) begin //Read
					case(OP)
					//Word (32bits)
						6'b100011:begin
							DataOut[31:24] = Mem[Address];
							DataOut[23:16] = Mem[Address+1];
							DataOut[15:8]  = Mem[Address+2];
							DataOut[7:0]   = Mem[Address+3];
							end
					//Half-Word (16 bits)
						6'b100101:begin
							DataOut[31:16]  = 16'h0000; //16 bits zeroes
							DataOut[15:8]  = Mem[Address];
							DataOut[7:0]   = Mem[Address+1];
							end
					//Byte (8 bits)
						6'b100100:begin
							DataOut[31:8]  = 24'h000000; //24 bits zeroes
							DataOut[7:0]    = Mem[Address];	
              
						end
					endcase
      MOC = 1'b1;
					end
				else //Write
				begin
					case(OP)
					//Word (32bits)
						6'b101011:begin 
							Mem[Address] = DataIn[31:24];
							Mem[Address+1] = DataIn[23:16];
							Mem[Address+2] = DataIn[15:8];
							Mem[Address+3] = DataIn[7:0];
						end
							 
					//Half-Word	(16bits)
						6'b101001:begin
							Mem[Address] = DataIn[15:8];
							Mem[Address+1] = DataIn[7:0];
						end
					//Byte (8 bits)
						6'b101000:begin
							Mem[Address] = DataIn[7:0];
						end
					endcase	
				end
      MOC = 1'b1;
			end
		end
    // initial begin
    //   $monitor("DO: %b MOV %b  MOC %b  OP %b", DataOut, MOV, MOC, OP);
     
    // end
endmodule

module mux_2x1_32(output reg [31:0] O, input Sel, input [31:0] r0, input [31:0] r1);
 always @ (r0,r1,Sel) begin	
    case(Sel)
    1'd0: O = r0;
    1'd1: O = r1;  
   endcase
 end
//   initial begin 
// #5
//  $display("Mux B:   Sel %b O %b", Sel, O);
//  #5
//  $display("Mux B:   Sel %b O %b", Sel, O);
//  #5
//  $display("Mux B:   Sel %b O %b", Sel, O); 
//  end
endmodule

module mux_2x1_5(output reg [4:0] O, input Sel, input [4:0] r0, input [4:0] r1);
 always @ (r0,r1,Sel) begin	
    case(Sel)
    1'd0: O = r0;
    1'd1: O = r1;  
   endcase
 end
endmodule

module mux_4x1_32(output reg [31:0] O, input [1:0] Sel, input [31:0] r0, input [31:0] r1, input [31:0] r2, input [31:0] r3);
 always @ (r0,r1,r2,r3,Sel) begin	
    case(Sel)
    2'd0: O = r0;
    2'd1: O = r1;  
	  2'd2: O = r2;
    2'd3: O = r3;   
   endcase
 end
//  initial begin
//  #5
//  $display("Mux A:   Sel %b O %b", Sel, O);
//  #5
//  $display("Mux A:   Sel %b O %b", Sel, O);
//  #5
//  $display("Mux A:   Sel %b O %b", Sel, O);
//  end
endmodule

module mux_3x1_32(output reg [31:0] O, input [1:0] Sel, input [31:0] r0, input [31:0] r1, input [31:0] r2);
 always @ (r0,r1,r2,Sel) begin	
    case(Sel)
    2'd0: O = r0;
    2'd1: O = r1;  
	2'd2: O = r2; 
   endcase
 end
endmodule

module registerclr(output reg [31:0]Q, input ld, input [31:0]D, input Clr, input Clk);	
  always @ (posedge Clk, posedge Clr)         
		if(Clr) Q = 32'h0000_0000;
		else if(ld)
			Q = D;           
endmodule

module signextend(output reg[31:0] O, input [15:0] imm16);
	always @ (imm16) begin
		if(imm16[15] == 1) begin
			O[31:16] = 16'hFFFF;
			O[15:0] = imm16;
		end

		else begin
			O[31:16] = 16'h0000;
			O[15:0] = imm16;
		end
	end
endmodule


module ALU(output reg [31:0] Y, output reg N, Z, C, V, input [5:0] func, input signed [31:0]A, input signed [31:0]B);
integer count1,count2,count3,i;
reg [63:0] temp;
always@(func,A,B)
begin
	case(func)
		6'b100000: begin {C,Y}=A+B; 
		N = (Y[31]==1'b1);
		Z = (Y == 32'b00000000000000000000000000000000) ? 1 : 0 ;
		if(A[31]==B[31] && Y[31]!=A[31]) V=1;
		else V=0;
		end// ADD--

		6'b100001: begin {C,Y}=A+B; 
		N = (Y[31]==1'b1);
		Z = (Y == 32'b00000000000000000000000000000000) ? 1 : 0 ;
		if(A[31]==B[31] && Y[31]!=A[31]) V=1;
		else V=0;
		end // ADDU--

		6'b100011: begin {C,Y}=A-B;
		N = (Y[31]==1'b1);
		Z = (Y == 32'b00000000000000000000000000000000) ? 1 : 0 ;
		if(A[31]==B[31] && Y[31]!=A[31]) V=1;
		else V=0;
		end // SUBU--
		/* 6'b001000: begin {C,Y}=$signed(A)+$signed(B); 
		N=Y[31];
		if(Y==0) Z=1;
		else Z=0;
		if(A[31]==B[31] && Y[31]!=A[31]) V=1;
		else V=0;
		end // ADDI-- */

		6'b001001: begin {C,Y}=$signed(A)+$signed(B); 
		N = (Y[31]==1'b1);
		Z = (Y == 32'b00000000000000000000000000000000) ? 1 : 0 ;
		if(A[31]==B[31] && Y[31]!=A[31]) V=1;
		else V=0;
		end // ADDiU--
		
		6'b011001: begin temp=$signed(A)*$signed(B);
		N = (temp[63]==1'b1);
		Z = (temp == 64'h0000000000000000) ? 1 : 0 ;
		V=0;
		Y = temp[31:0];
		end // MULTU LO
		
		6'b011010: begin temp=$signed(A)*$signed(B);
		N = (temp[63]==1'b1);
		Z = (temp == 64'h0000000000000000) ? 1 : 0 ;
		V=0;
		Y = temp[63:32];
		end // MULTU HI
		
		//SRA
		6'b000011: begin {C,Y} = B>>>32'h00000003;
		N = (Y[31]==1'b1);
		Z = (Y == 32'b00000000000000000000000000000000) ? 1 : 0 ;
		if(A[31]==B[31] && Y[31]!=A[31]) V=1;
		else V=0;
		end 
		
		//SRL
		6'b000010: begin {C,Y} = B>>32'h00000003;
		N = (Y[31]==1'b1);
		Z = (Y == 32'b00000000000000000000000000000000) ? 1 : 0 ;
		if(A[31]==B[31] && Y[31]!=A[31]) V=1;
		else V=0;
		end 
		
		//SLL	
		6'b000001: begin {C,Y} = B<<32'h00000003;
		N = (Y[31]==1'b1);
		Z = (Y == 32'b00000000000000000000000000000000) ? 1 : 0 ;
		if(A[31]==B[31] && Y[31]!=A[31]) V=1;
		else V=0;
		end 

		/* 6'b100100: begin{C,Y}=A&B; 
		N=Y[31];
		if(Y==0) Z=1;
		else Z=0;
		V=0;
		end// AND--

		6'b001100: begin{C,Y}=A&B; 
		N=Y[31];
		if(Y==0) Z=1;
		else Z=0;
		V=0;
		end// ANDI-- */



		// 6'b100111: begin Y=~(A|B); 
		// N=Y[31];
		// if(Y==0) Z=1;
		// else Z=0;
		// end//NOR--

		6'b100101: begin Y=A|B; 
		N=Y[31];
		if(Y==0) Z=1;
		else Z=0;
		end// OR--

		/*6'b001101: begin Y=A|B; 
		N=Y[31];
		if(Y==0) Z=1;
		else Z=0;
		end// ORI--

		6'b100110: begin Y=A^B;
		N=Y[31];
		if(Y==0) Z=1;
		else Z=0;
		end// XOR--

		6'b001110: begin Y=A^B; 
		N=Y[31];
		if(Y==0) Z=1;
		else Z=0;
		end// XORI--

		6'b001111: begin Y[31:16]=A; 
		Y[15:0]=0;
		end// LUI-- */


		/* 6'b100010: begin {C,Y}=$signed(A)-$signed(B);
		N=Y[31];
		if(Y==0) Z=1;
		else Z=0;
		if(A[31]==B[31] && Y[31]!=A[31]) V=1;
		else V=0;
		end//SUB--


		6'b100011: begin {C,Y}=A-B;
		N=Y[31];
		if(Y==0) Z=1;
		else Z=0;
		if(A[31]==B[31] && Y[31]!=A[31]) V=1;
		else V=0;
		end//SUBU--

		6'b101010: begin if($signed(A)<$signed(B)) Y=1;//SLT--
		else Y=0;

		end

		6'b101011: begin if(A<B) Y=1;//SLTU--
		else Y=0;

		end */


		/* 6'b001010: begin if($signed(A)<$signed(B)) Y=1;//SLTI--
		else Y=0;

		end

		6'b001011: begin if(A<B) Y=1;//SLTIU--
		else Y=0;

		end */

		/* 6'b000000: Y=A<<B[4:0];//SLL--
		6'b000100: Y=A<<B[4:0];//SLLV--
		6'b000011: Y=A>>B[4:0]; //SRA--
		6'b000111: Y=A>>B[4:0]; //SRAV--
		6'b000010: Y=A>>B[4:0]; //SRL--
		6'b000110: Y=A>>B[4:0]; //SRLV-- */


	endcase
end
endmodule

module datapath(input Clr, Clk);

//wires to be used on instantiating the modules on datapath
  reg [7:0] state1;
  wire [7:0] Y, state, st;
  wire COND, MOC, IR_Ld, MAR_Ld, MDR_Ld, MuxMAR_Ld, RF_Ld, MuxC_Ld, PC_Ld, nPC_Ld, MuxMDR_Ld, MOV, RW, Hi_Ld, Lo_Ld;
  wire [5:0] opcode;
  wire[1:0] MuxA_Ld, MuxB_Ld, MuxReg_Ld;

  wire[31:0] DataOut;

	wire[31:0] PC_out;

	wire[31:0] se_out;

	wire[31:0] nPC_out;

	wire[31:0] MAR_out, Mux_MAR_out;

	wire[31:0] MDR_out, Mux_MDR_out;

	wire[31:0] ALU_out, Hi_in, Lo_in;
	wire[5:0] func;
  wire N, Z, C, V;

	wire[31:0] IR_out;

	wire[31:0] PA, PB; 
	wire[4:0] Mux_c_out;

	wire[31:0] Mux_a_out, Mux_b_out, Mux_reg_out;
	
  wire[31:0] lo_out, hi_out;
	
// Modules are Instantiated here. Missing ALU and Register File

	Control_Unit CU(state, Y, st, state1, COND, IR_out, MOC, IR_Ld, MAR_Ld, MDR_Ld, MuxMAR_Ld, RF_Ld, MuxC_Ld, PC_Ld, nPC_Ld, MuxMDR_Ld, MOV, RW, Hi_Ld, Lo_Ld, opcode, func, MuxA_Ld, MuxB_Ld, MuxReg_Ld, Clr, Clk);

	mux_4x1_32 MUXA(Mux_a_out, MuxA_Ld, PA, PC_out,  nPC_out, 32'd0);
	
	mux_4x1_32 MUXB(Mux_b_out, MuxB_Ld, PB, se_out, MDR_out, 32'd4);
	
  mux_2x1_5 MUXC(Mux_c_out, MuxC_Ld, IR_out[15:11], IR_out[20:16]);

  mux_3x1_32 MUXREG(Mux_reg_out, MuxReg_Ld, ALU_out, hi_out, lo_out);
			
	signextend SE(se_out, IR_out[15:0]);
  
  registerclr nPC(nPC_out, nPC_Ld, ALU_out, Clr, Clk);
	
	registerclr PC(PC_out, PC_Ld, nPC_out, Clr, Clk);
	
	registerclr IR(IR_out, IR_Ld, DataOut, Clr, Clk);
	
	registerclr LO(lo_out, Lo_Ld, ALU_out, Clr, Clk);
	
	registerclr HI(hi_out, Hi_Ld, ALU_out, Clr, Clk);
	
	mux_2x1_32 MUXMAR(Mux_MAR_out, MuxMAR_Ld, PC_out, ALU_out);
	
	mux_2x1_32 MUXMDR(Mux_MDR_out, MuxMDR_Ld, ALU_out, DataOut);
	
	registerclr MAR(MAR_out, MAR_Ld, Mux_MAR_out, Clr, Clk);
	
	registerclr MDR(MDR_out, MDR_Ld, Mux_MDR_out, Clr, Clk);
	
	ram512x8 RAM(DataOut, MOC, RW, MOV, MAR_out, MDR_out, opcode);

	ALU ALU(ALU_out, N, Z, C, V, func, Mux_a_out, Mux_b_out);

  RegisterFile REGISTERFILE(PA, PB, Mux_reg_out, Mux_c_out, RF_Ld, Clk, IR_out[25:21], IR_out[20:16], Clr);


endmodule

module RegisterFile(PA,PB,PC,C,RF,Clk,A,B,Clr);

 parameter DATA_WIDTH = 32; //32-bit bus for Data input and output
 parameter OpCode_bits = 5; // 2^5 = 32 localizations to select rister 
 
 //Outputs
 output[DATA_WIDTH-1:0]PA,PB; 
 //Inputs
 input [DATA_WIDTH-1:0] PC;
 input [OpCode_bits-1:0]C,A,B;
 input RF,Clk,Clr;
 
 wire [31:0] E; // Enable register from binary decoder 
 // registers output to connect with mux with bus of 32-bits
 wire [31:0] r0,r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12,r13,r14,r15,r16,r17,r18,r19,r20,r21,r22,r23,r24,r25,r26,r27,r28,r29,r30,r31; 

 binarydecoder binarydecoder(E,C,RF); // Output E enables register at desired localization

 register reg0(r0,32'h0000_0000,E[0],RF,Clk,Clr);

 register reg1(r1,PC,E[1],RF,Clk,Clr);

 register reg2(r2,PC,E[2],RF,Clk,Clr);
 register reg3(r3,PC,E[3],RF,Clk,Clr);
 register reg4(r4,PC,E[4],RF,Clk,Clr);
 
 register reg5(r5,PC,E[5],RF,Clk,Clr);
 register reg6(r6,PC,E[6],RF,Clk,Clr);
 register reg7(r7,PC,E[7],RF,Clk,Clr);
 register reg8(r8,PC,E[8],RF,Clk,Clr);
 register reg9(r9,PC,E[9],RF,Clk,Clr);
 
 register reg10(r10,PC,E[10],RF,Clk,Clr);
 register reg11(r11,PC,E[11],RF,Clk,Clr);
 register reg12(r12,PC,E[12],RF,Clk,Clr);
 register reg13(r13,PC,E[13],RF,Clk,Clr);
 register reg14(r14,PC,E[14],RF,Clk,Clr);
 
 register reg15(r15,PC,E[15],RF,Clk,Clr);
 register reg16(r16,PC,E[16],RF,Clk,Clr);
 register reg17(r17,PC,E[17],RF,Clk,Clr);
 register reg18(r18,PC,E[18],RF,Clk,Clr);
 register reg19(r19,PC,E[19],RF,Clk,Clr);
 
 register reg20(r20,PC,E[20],RF,Clk,Clr);
 register reg21(r21,PC,E[21],RF,Clk,Clr);
 register reg22(r22,PC,E[22],RF,Clk,Clr);
 register reg23(r23,PC,E[23],RF,Clk,Clr);
 register reg24(r24,PC,E[24],RF,Clk,Clr);
 
 register reg25(r25,PC,E[25],RF,Clk,Clr);
 register reg26(r26,PC,E[26],RF,Clk,Clr);
 register reg27(r27,PC,E[27],RF,Clk,Clr);
 register reg28(r28,PC,E[28],RF,Clk,Clr);
 register reg29(r29,PC,E[29],RF,Clk,Clr);
 
 register reg30(r30,PC,E[30],RF,Clk,Clr);
 register reg31(r31,PC,E[31],RF,Clk,Clr);  

 mux_32x1 muxA(PA,A,r0,r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12,r13,r14,r15,r16,r17,r18,r19,r20,r21,r22,r23,r24,r25,r26,r27,r28,r29,r30,r31);
 mux_32x1 muxB(PB,B,r0,r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12,r13,r14,r15,r16,r17,r18,r19,r20,r21,r22,r23,r24,r25,r26,r27,r28,r29,r30,r31);
 
 /* always @ (PC)
	$monitor("PC: %h", PC); */
 
endmodule

//Chooses register
module binarydecoder (E,C,RF);
 
  // BUS width
  parameter DATA_WIDTH = 32; 	// 32 registers, user adjustable
  parameter OpCode_bits = 5; 	// 2^5 = 32 possible register selection
  
  // Output
  output reg [DATA_WIDTH-1:0]E;	// Output E, register enable
  
  // Inputs	
  input[OpCode_bits-1:0]C;	    // OpCode selects Register upon to write to
  input RF;                     // Load Enable, write to register

  /* initial begin
	E = 32'h00000000;
  end */
  
  always @ (posedge RF, C)		      // Start writing when enable load
  begin
  //$monitor("INSIDE binary C %h", C);
    case(C)                     // test OpCode
    5'd0: E = 32'b0000_0000_0000_0000_0000_0000_0000_0001;
    5'd1: E = 32'b0000_0000_0000_0000_0000_0000_0000_0010;
    5'd2: E = 32'b0000_0000_0000_0000_0000_0000_0000_0100;
    5'd3: E = 32'b0000_0000_0000_0000_0000_0000_0000_1000;
    5'd4: E = 32'b0000_0000_0000_0000_0000_0000_0001_0000;
    5'd5: E = 32'b0000_0000_0000_0000_0000_0000_0010_0000;
    5'd6: E = 32'b0000_0000_0000_0000_0000_0000_0100_0000;
    5'd7: E = 32'b0000_0000_0000_0000_0000_0000_1000_0000;
    5'd8: E = 32'b0000_0000_0000_0000_0000_0001_0000_0000;
    5'd9: E = 32'b0000_0000_0000_0000_0000_0010_0000_0000;
    5'd10:E = 32'b0000_0000_0000_0000_0000_0100_0000_0000;
    5'd11:E = 32'b0000_0000_0000_0000_0000_1000_0000_0000;
    5'd12:E = 32'b0000_0000_0000_0000_0001_0000_0000_0000;
    5'd13:E = 32'b0000_0000_0000_0000_0010_0000_0000_0000;
    5'd14:E = 32'b0000_0000_0000_0000_0100_0000_0000_0000;
    5'd15:E = 32'b0000_0000_0000_0000_1000_0000_0000_0000;
    5'd16:E = 32'b0000_0000_0000_0001_0000_0000_0000_0000;
    5'd17:E = 32'b0000_0000_0000_0010_0000_0000_0000_0000;
    5'd18:E = 32'b0000_0000_0000_0100_0000_0000_0000_0000;
    5'd19:E = 32'b0000_0000_0000_1000_0000_0000_0000_0000;
    5'd20:E = 32'b0000_0000_0001_0000_0000_0000_0000_0000;
    5'd21:E = 32'b0000_0000_0010_0000_0000_0000_0000_0000;
    5'd22:E = 32'b0000_0000_0100_0000_0000_0000_0000_0000;
    5'd23:E = 32'b0000_0000_1000_0000_0000_0000_0000_0000;
    5'd24:E = 32'b0000_0001_0000_0000_0000_0000_0000_0000;
    5'd25:E = 32'b0000_0010_0000_0000_0000_0000_0000_0000;
    5'd26:E = 32'b0000_0100_0000_0000_0000_0000_0000_0000;
    5'd27:E = 32'b0000_1000_0000_0000_0000_0000_0000_0000;
    5'd28:E = 32'b0001_0000_0000_0000_0000_0000_0000_0000;
    5'd29:E = 32'b0010_0000_0000_0000_0000_0000_0000_0000; 
    5'd30:E = 32'b0100_0000_0000_0000_0000_0000_0000_0000; 
    5'd31:E = 32'b1000_0000_0000_0000_0000_0000_0000_0000; 
   endcase
   //$monitor("INSIDE binary E %h", E);
   end
endmodule

module register(Q,D,Ld,RF,Clk,Clr);
  // BUS width
  parameter DATA_WIDTH = 32;    // Adjust register size as desired

  // Outputs
  output reg [DATA_WIDTH-1:0]Q;	// Output Q, register stored data
  
  // Inputs	
  input[DATA_WIDTH-1:0]D;	    // Input Data
  input Ld,Clk,Clr,RF;				    // Load Enable, Clock

  
  always @ (posedge Clk)        // At positive edge 
  begin
	if(Clr) Q = 32'b00000000;
	else begin
		if (Ld && RF) Q = D;            // Load Enable, then store data
		else Q = Q;
		//$monitor("INSIDE REGISTER Q D E RF %h %h %h %h", Q, D, Ld, RF);
	end
  end
	  
endmodule


// Multiplexer 32 inputs
//Sets PA or PB to the desired register values
module mux_32x1(O,Sel,r0,r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12,r13,r14,r15,r16,r17,r18,r19,r20,r21,r22,r23,r24,r25,r26,r27,r28,r29,r30,r31);
 
 parameter DATA_WIDTH = 32; // 32-bit data bus
 parameter OpCode_bits = 5; // 2^5 = 32 locations

 //Output
 output reg [DATA_WIDTH-1:0]O;
 //Inputs
 input[DATA_WIDTH-1:0]r0,r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12,r13,r14,r15,r16,r17,r18,r19,r20,r21,r22,r23,r24,r25,r26,r27,r28,r29,r30,r31;
 input [OpCode_bits-1:0] Sel;
 
always @ (r0,r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12,r13,r14,r15,r16,r17,r18,r19,r20,r21,r22,r23,r24,r25,r26,r27,r28,r29,r30,r31,Sel) begin	
	//$monitor("Sel: %d", Sel);
    case(Sel)
    5'd0: O = 32'h0000_0000;
    5'd1: O = r1;
    5'd2: O = r2;
    5'd3: O = r3;
    5'd4: O = r4;
    5'd5: O = r5;
    5'd6: O = r6;
    5'd7: O = r7;
    5'd8: O = r8;
    5'd9: O = r9;
    5'd10:O = r10;
    5'd11:O = r11;
    5'd12:O = r12;
    5'd13:O = r13;
    5'd14:O = r14;
    5'd15:O = r15;
    5'd16:O = r16;
    5'd17:O = r17;
    5'd18:O = r18;
    5'd19:O = r19;
    5'd20:O = r20;
    5'd21:O = r21;
    5'd22:O = r22;
    5'd23:O = r23;
    5'd24:O = r24;
    5'd25:O = r25;
    5'd26:O = r26;
    5'd27:O = r27;
    5'd28:O = r28;
    5'd29:O = r29;
    5'd30:O = r30; 
    5'd31:O = r31;    
   endcase
   end
endmodule



















module Control_Unit(output [7:0] state,
                    output [7:0] Y, st,
                    input  [7:0] state1 , 
                    input COND,
                    input [31:0] IR, 
                    input MOC, 
                    output IR_Ld,
                    output MAR_Ld,
                    output MDR_Ld,
                    output MuxMAR_Ld,
                    output RF_Ld,
                    output MuxC_Ld,
                    output PC_Ld,
                    output nPC_Ld,
                    output MuxMDR_Ld,
                    output MOV,
                    output RW,
                    output Hi_Ld,
                    output Lo_Ld,
                    output [5:0] opcode,
                    output[5:0] func,
                    output [1:0] MuxA_Ld, MuxB_Ld, MuxReg_Ld,
                    input Clr, Clk);

// outputs 
wire [5:0] CR;  // esto es para los estados 
wire Sts ,in,Inv;  // aqui los el inverter y NSAddressSelector
wire[1:0] M, S;  // para los moc  y NSASelector
wire [2:0] N;  // para NSASelecto
wire [7:0] D;  // salida del adder  
wire [7:0] Q; // salida del incremento 
wire[50:0] data_out;
// instancias entre moduluos 
IR_Encoder enc( state, IR,Clk);   
MUX_COND mxcnd( in, MOC,COND, S); 
inverter inv( Sts, in, Inv);  
next_state_address_selector nsas( M,  N, Sts); 
state_adder stadd( D, st, 8'd1); 
incrementer_register increg( Q,  D, Clk);
MUX_ROM mxrom( st, state, state1, CR, Q, M, Clr); 
microStore mcStr(st, data_out, Y);  
control_register Creg(N, Inv, S, IR_Ld, MAR_Ld, MDR_Ld, MuxMAR_Ld, RF_Ld, MuxC_Ld, PC_Ld, nPC_Ld, MuxMDR_Ld, MOV, RW, Hi_Ld, Lo_Ld, opcode, func, MuxA_Ld, MuxB_Ld, MuxReg_Ld, CR,Y, data_out, Clr, Clk);

endmodule//////////////////////////////////////////////////////////////////////////////////////////


module IR_Encoder(output reg [7:0] state, input [31:0] IR, input Clk);

  wire [5:0] func;
  wire [5:0] opcode;
  wire [4:0] rs;
  wire [4:0] rt;
  wire [4:0] rd;
  wire [4:0] sa;
  wire [15:0] imm16;
  wire [25:0] address26;


  assign func = IR[5:0];
  assign opcode = IR[31:26];
  assign rs = IR[25:21];
  assign rt = IR[20:16];
  assign rd = IR[15:11];
  assign sa = IR[10:6];
  assign imm16 = IR[15:0];
  assign address26 = IR[25:0];

  always@(IR, Clk)
  
    case(opcode)
      6'b000000: 
        case(func)
          6'b100000: state = 8'd5;
          6'b100001: state = 8'd6;
          6'b100010: state = 8'd9;
          6'b100011: state = 8'd10;
          6'b101010: state = 8'd11;
          6'b101011: state = 8'd13;
          6'b100100: state = 8'd21;
          6'b100101: state = 8'd23;
          6'b100110: state = 8'd25;
          6'b100111: state = 8'd27;
          6'b000000: state = 8'd29;
          6'b000100: state = 8'd30;
          6'b000011: state = 8'd31;
          6'b000111: state = 8'd32;
          6'b000010: state = 8'd33;
          6'b000110: state = 8'd34;
          6'b010000: state = 8'd71;
          6'b010010: state = 8'd72;
          6'b001011: state = 8'd73;
          6'b001010: state = 8'd74;
          6'b010001: state = 8'd75;
          6'b010011: state = 8'd76;
          6'b001001: state = 8'd99;
          6'b001000: state = 8'd101;
          6'b010000: state = 8'd103;
          6'b110100: state = 8'd104;
          6'b110000: state = 8'd106;
          6'b110001: state = 8'd108;
          6'b110010: state = 8'd110;
          6'b110011: state = 8'd112;
          6'b110110: state = 8'd114;
          6'b011001: state = 8'd129;
          6'b011010: state = 8'd131;
        endcase
      6'b010000:
        case(func)
          6'b010000: state = 8'd102;
          6'b011000: state = 8'd128;
        endcase
      6'b001000: state = 8'd7;
      6'b001001: state = 8'd8;
      6'b001010: state = 8'd15;
      6'b001011: state = 8'd17;
      6'b011100: 
        case(func)
          6'b100001: state = 8'd19;
          6'b100000: state = 8'd20;
        endcase
      6'b001100: state = 8'd22;
      6'b001101: state = 8'd24;
      6'b001110: state = 8'd26;
      6'b001111: state = 8'd28;
      6'b100011: state = 8'd35;
      6'b100001: state = 8'd39;
      6'b100101: state = 8'd43;
      6'b100000: state = 8'd47;
      6'b100100: state = 8'd51;
      6'b111111: state = 8'd55;
      6'b101011: state = 8'd59;
      6'b101001: state = 8'd63;
      6'b101000: state = 8'd67;
      6'b000001: 
        case(rt)
          5'b10001: state = 8'd84;
          5'b00000: state = 8'd92;
          5'b01100: state = 8'd116;
          5'b10000: state = 8'd118;
          5'b01001: state = 8'd120;
          5'b01010: state = 8'd122;
          5'b01011: state = 8'd124;
          5'b01110: state = 8'd126;
        endcase
      6'b000100: state = 8'd80;
      6'b000101: state = 8'd94;
      6'b000010: state = 8'd96;
      6'b000011: state = 8'd97;
    endcase
endmodule

module MUX_COND(output reg out, input I0, I1, input [1:0] S);
  always @(S)
    case(S)
    2'b00: out = I0;
    2'b01: out = I1;
    endcase

endmodule

module MUX_ROM(output reg [7:0] Y, input [7:0] I0, I1, input[5:0] I2, input[7:0] I3, input [1:0] M, input Clr);
  always @*

//Clr redirects to state 0 and clears CR
  if(Clr)
    Y = 8'd0;
  else
      case(M)
        2'b00: Y = I0; 
        2'b01: Y = I1;
        2'b10: Y = I2;
        2'b11: Y = I3;
      endcase

endmodule

module next_state_address_selector(output reg [1:0] M, input [2:0] N, input Sts);
  wire [3:0] A;

  assign A[0] = Sts;
  assign A[3:1] = N;
  always @(A)
    case(A)
    
    4'b0000: M = 2'b00;
    4'b0001: M = 2'b00;
    4'b0010: M = 2'b01;
    4'b0011: M = 2'b01;
    4'b0100: M = 2'b10;
    4'b0101: M = 2'b10;
    4'b0110: M = 2'b11;
    4'b0111: M = 2'b11;
    4'b1000: M = 2'b00;
    4'b1001: M = 2'b10;
    4'b1010: M = 2'b11;
    4'b1011: M = 2'b10;
    4'b1100: M = 2'b11;
    4'b1101: M = 2'b00;
    4'b1110: M = 2'b00;
    4'b1111: M = 2'b00;
    endcase

endmodule

module state_adder(output [7:0] Z, input [7:0] A, B);
  assign Z = A + B;

endmodule

module incrementer_register(output reg [7:0] Q, input [7:0] D, input  Clk);
  always @(posedge Clk)
  Q=D;
endmodule

module inverter(output reg out, input in, inv);
  wire [1:0] A;

  assign A[0] = in;
  assign A[1] = inv;

  always @(A)
    case(A)
    2'b00: out = 1'b0;
    2'b01: out = 1'b1;
    2'b10: out = 1'b1;
    2'b11: out = 1'b0;
    endcase

endmodule

module microStore(input [7:0] Y, output reg [50:0] data_out, output reg [7:0] state);

  reg[2:0] N;
  reg Inv;
  reg[1:0] S;
  reg IR_Ld;
  reg MAR_Ld;
  reg MDR_Ld;
  reg MuxMAR_Ld;
  reg RF_Ld;
  reg MuxC_Ld;
  reg PC_Ld;
  reg nPC_Ld;
  reg MuxMDR_Ld;
  reg MOV;
  reg RW;
  reg[5:0] opcode;
  reg[5:0] func;
  reg Hi_Ld, Lo_Ld;
  reg[1:0] MuxA_Ld, MuxB_Ld, MuxReg_Ld;
  reg[5:0] CR;
  reg dummy1;
  reg [1:0] dummy2;
  reg [5:0] dummy3;

  always @(*)
  
    // N = 3'b 000 Para Instruction encoder => 
    // N = 3'b 010 Para usar el CR
    // N = 3'b011 para incrementer

    case(Y)
        8'd0:
        begin
            N = 3'b011;//  32
            Inv = 1'b0;//  29
            S = 2'b00;//  28
            IR_Ld = 1'b1;//  26
            MAR_Ld = 1'b1;//  25
            MDR_Ld = 1'b1;//  24
            MuxMAR_Ld = 1'b0;//  23
            RF_Ld = 1'b0;// 22
            MuxC_Ld = 1'b0;// 21
            PC_Ld = 1'b0;// 20
            nPC_Ld = 1'b0;// 19
            MuxMDR_Ld = 1'b0;// 18
            MOV = 1'b0;// 17
            RW = 1'b0;// 16
            opcode = 6'b000000;// 15
            func = 6'b000000;
            MuxA_Ld = 2'b00;// 9
            MuxB_Ld = 2'b00; // 8
            Hi_Ld = 1'b0;
            Lo_Ld = 1'b0;
            MuxReg_Ld = 2'b00;
            CR = 6'b000001; // 5
            state = Y;
          
          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
          end


         8'd1:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b1;// 19
          nPC_Ld = 1'b1;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b000000;// 14
          func = 6'b100001;
          MuxA_Ld = 2'b10;// 8
          MuxB_Ld = 2'b10; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;
          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 


        end
         8'd2:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b1;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b1;// 19
          nPC_Ld = 1'b1;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b1;// 16
          RW = 1'b1;// 15
          opcode = 6'b100011;// 14
          func = 6'b000000;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000011; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 

        end
         8'd3:
        begin
            N = 3'b011;//  32
            Inv = 1'b0;//  28
            S = 2'b00;//  27
            IR_Ld = 1'b0;//  25
            MAR_Ld = 1'b0;//  24
            MDR_Ld = 1'b0;//  23
            MuxMAR_Ld = 1'b0;//  22
            RF_Ld = 1'b0;// 21
            MuxC_Ld = 1'b0;// 20
            PC_Ld = 1'b0;// 19
            nPC_Ld = 1'b0;// 18
            MuxMDR_Ld = 1'b0;// 17
            MOV = 1'b1;// 16
            RW = 1'b1;// 15
            opcode = 6'b100001;// 14
            func = 6'b100011;
            MuxA_Ld = 2'b00;// 8
            MuxB_Ld = 2'b00; // 6
            Hi_Ld = 1'b0;
            Lo_Ld = 1'b0;
            MuxReg_Ld = 2'b00;
            CR = 6'b000011; //4
            state = Y;
          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 

        end
          8'd254:
        begin
            N = 3'b011;//  32
            Inv = 1'b0;//  28
            S = 2'b00;//  27
            IR_Ld = 1'b0;//  25
            MAR_Ld = 1'b0;//  24
            MDR_Ld = 1'b0;//  23
            MuxMAR_Ld = 1'b0;//  22
            RF_Ld = 1'b0;// 21
            MuxC_Ld = 1'b0;// 20
            PC_Ld = 1'b0;// 19
            nPC_Ld = 1'b0;// 18
            MuxMDR_Ld = 1'b0;// 17
            MOV = 1'b1;// 16
            RW = 1'b1;// 15
            opcode = 6'b100001;// 14
            func = 6'b100011;
            MuxA_Ld = 2'b00;// 8
            MuxB_Ld = 2'b00; // 6
            Hi_Ld = 1'b0;
            Lo_Ld = 1'b0;
            MuxReg_Ld = 2'b00;
            CR = 6'b000011; //4
            state = Y;
          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 

        end
         8'd4:
        begin
          N = 3'b000;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b1;//  25
          MAR_Ld = 1'b1;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b1;// 16
          RW = 1'b1;// 15
          opcode = 6'b000000;// 14
          func = 6'b000000;;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;
          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 

        end
      8'd5:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b1;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b000000;// 14
          func = 6'b100000;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
      8'd6:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b1;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b000000;// 14
          func = 6'b100001;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
      8'd7:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b001000;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
      8'd8:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b001001;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd9:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b1;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b000000;// 14
          func = 6'b100010;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd10:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b1;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b000000;// 14
          func = 6'b100011;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd11:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b1;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b000000;// 14
          func = 6'b101010;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd12:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b1;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b000000;// 14
          func = 6'b101010;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd13:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b1;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b000000;// 14
          func = 6'b101011;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd14:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b1;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b000000;// 14
          func = 6'b101011;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd15:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b1;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b001001;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd16:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b1;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b001010;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd17:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b1;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b001011;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
         8'd18:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b1;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b001011;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd19:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b1;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b011100;// 14
          func = 6'b100001;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd20:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b1;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b011100;// 14
          func = 6'b100000;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd21:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b1;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b000000;// 14
          func = 6'b100100;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd22:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b001100;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd23:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b1;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b000000;// 14
          func = 6'b100101;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd24:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b001101;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd25:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b1;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b000000;// 14
          func = 6'b100110;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd26:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b001110;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd27:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b1;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b000000;// 14
          func = 6'b100111;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd28:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b001111;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd29:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b1;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b000000;// 14
          func = 6'b000000;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd30:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b1;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b000000;// 14
          func = 6'b000100;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd31:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b1;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b000000;// 14
          func = 6'b000011;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd32:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b1;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b000000;// 14
          func = 6'b000111;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd33:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b1;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b000000;// 14
          func = 6'b000010;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd34:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b1;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b000000;// 14
          func = 6'b000110;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd35:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b1;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b100011;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd36:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b1;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b1;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b1;// 17
          MOV = 1'b1;// 16
          RW = 1'b1;// 15
          opcode = 6'b100011;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd37:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b1;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b1;// 17
          MOV = 1'b1;// 16
          RW = 1'b1;// 15
          opcode = 6'b100011;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd38:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b1;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b1;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b100011;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b10; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd39:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b1;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b100001;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd40:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b1;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b1;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b1;// 17
          MOV = 1'b1;// 16
          RW = 1'b1;// 15
          opcode = 6'b100001;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd41:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b1;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b1;// 17
          MOV = 1'b1;// 16
          RW = 1'b1;// 15
          opcode = 6'b100001;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd42:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b1;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b1;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b100001;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b10; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd43:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b1;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b100101;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd44:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b1;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b1;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b1;// 17
          MOV = 1'b1;// 16
          RW = 1'b1;// 15
          opcode = 6'b100101;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd45:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b1;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b1;// 17
          MOV = 1'b1;// 16
          RW = 1'b1;// 15
          opcode = 6'b100101;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
       //Ale End 
        
       //Steph Begin
        8'd46:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b1;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b1;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b101001;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b10; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end      
           8'd47:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b1;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b000001;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
           state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd48:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b1;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b1;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b1;// 17
          MOV = 1'b1;// 16
          RW = 1'b1;// 15
          opcode = 6'b000001;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd49:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b1;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b1;// 17
          MOV = 1'b1;// 16
          RW = 1'b1;// 15
          opcode = 6'b000001;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
         state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd50:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b1;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b1;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b000001;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b10; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
         state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd51:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b1;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b001001;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
         state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd52:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b1;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b1;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b1;// 17
          MOV = 1'b1;// 16
          RW = 1'b1;// 15
          opcode = 6'b001001;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
         state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd53:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b1;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b1;// 17
          MOV = 1'b1;// 16
          RW = 1'b1;// 15
          opcode = 6'b001001;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd54:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b1;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b001001;// 14
          func = opcode;
          MuxA_Ld = 2'b11;// 8
          MuxB_Ld = 2'b01; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd55:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b1;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b1;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b111111;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
         state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd56:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b1;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b1;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b111111;// 14
          func = opcode;
          MuxA_Ld = 2'b10;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd57:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b1;//  23
          MuxMAR_Ld = 1'b1;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b1;// 17
          MOV = 1'b1;// 16
          RW = 1'b0;// 15
          opcode = 6'b111111;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd58:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b1;// 16
          RW = 1'b0;// 15
          opcode = 6'b111111;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
         state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd59:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b1;// 16
          RW = 1'b0;// 15
          opcode = 6'b110101;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
        state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd60:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b1;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b1;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b1;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b110101;// 14
          func = opcode;
          MuxA_Ld = 2'b10;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
        state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd61:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b1;//  23
          MuxMAR_Ld = 1'b1;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b1;// 17
          MOV = 1'b1;// 16
          RW = 1'b0;// 15
          opcode = 6'b110101;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
         state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd62:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b1;// 16
          RW = 1'b0;// 15
          opcode = 6'b110101;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
         state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd63:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b1;// 16
          RW = 1'b0;// 15
          opcode = 6'b100101;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;
   
          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd64:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b1;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b1;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b100101;// 14
          func = opcode;
          MuxA_Ld = 2'b10;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd65:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b1;//  23
          MuxMAR_Ld = 1'b1;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b1;// 17
          MOV = 1'b1;// 16
          RW = 1'b0;// 15
          opcode = 6'b100101;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd66:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b1;// 16
          RW = 1'b0;// 15
          opcode = 6'b100101;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd67:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b1;// 16
          RW = 1'b0;// 15
          opcode = 6'b000101;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
         state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd68:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b1;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b1;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b1;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b000101;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
         state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd69:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b1;//  23
          MuxMAR_Ld = 1'b1;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b1;// 17
          MOV = 1'b1;// 16
          RW = 1'b0;// 15
          opcode = 6'b000101;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd70:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b1;// 16
          RW = 1'b0;// 15
          opcode = 6'b000101;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
     8'd71:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b1;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b000000;// 14
          func = 6'b010000;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd72:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b1;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b000000;// 14
          func = 6'b010010;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd73:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b1;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b000000;// 14
          func = 6'b001011;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
         state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd74:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b1;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b000000;// 14
          func = 6'b001010;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd75:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b1;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b000000;// 14
          func = 6'b010011;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd76:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b1;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b000000;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd77:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b1;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b001000;// 14
          func = opcode;
          MuxA_Ld = 2'b01;// 8
          MuxB_Ld = 2'b10; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
         state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd78:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b100000;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd79:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b1;// 19
          nPC_Ld = 1'b1;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b100000;// 14
          func = opcode;
          MuxA_Ld = 2'b01;// 8
          MuxB_Ld = 2'b10; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
      8'd80:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b1;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b1;// 16
          RW = 1'b0;// 15
          opcode = 6'b000000;// 14
          func = opcode;
          MuxA_Ld = 2'b10;// 8
          MuxB_Ld = 2'b01; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd81:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b1;// 19
          nPC_Ld = 1'b1;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b001000;// 14
          func = opcode;
          MuxA_Ld = 2'b01;// 8
          MuxB_Ld = 2'b10; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
         state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd82:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b100000;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd83:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b1;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b100000;// 14
          func = opcode;
          MuxA_Ld = 2'b01;// 8
          MuxB_Ld = 2'b10; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd84:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b100000;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
         state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd85:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b1;// 19
          nPC_Ld = 1'b1;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b100000;// 14
          func = opcode;
          MuxA_Ld = 2'b01;// 8
          MuxB_Ld = 2'b10; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd86:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b111000;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd87:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b1;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b111000;// 14
          func = opcode;
          MuxA_Ld = 2'b01;// 8
          MuxB_Ld = 2'b10; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd88:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b011000;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd89:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b1;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b011000;// 14
          func = opcode;
          MuxA_Ld = 2'b01;// 8
          MuxB_Ld = 2'b10; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd90:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b0;//  24
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b100000;// 14
          func = opcode;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
      //Steph End
    
      //Jaha Begin
      //BLTZ
		    8'd91:
        begin
		  N = 3'b010;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b1;
          MuxC_Ld = dummy1;
          PC_Ld = 1'b1;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000001;
          func = opcode;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b01; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		//BLTZAL
		    8'd92:
        begin
		
		//NOT YET ****************** Condition
		      N = 3'b011;
          Inv = 1'b0;
          S = 2'b0;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b0;
          MuxC_Ld = dummy1;
          PC_Ld = 1'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000001;
          func = opcode;
          MuxA_Ld = 2'b00;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		//BLTZAL (final state)
		    8'd93:
        begin
		  N = 3'b010;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b1;
          MuxC_Ld = dummy1;
          PC_Ld = 1'b1;
          nPC_Ld = 1'b1;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000001;
          func = opcode;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b01; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
		  CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		//BNE
		    8'd94:
        begin
		

	//NOT YET ****************** Condition
		      N = 3'b011;
          Inv = 1'b0;
          S = 2'b0;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b0;
          MuxC_Ld = dummy1;
          PC_Ld = 1'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000101;
          func = opcode;
          MuxA_Ld = 2'b00;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		
		//BNE Final State
		    8'd95:
        begin
		  N = 3'b010;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b1;
          MuxC_Ld =dummy1;
          PC_Ld = 1'b1;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000101;
          func = opcode;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b01; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		//J
         8'd96:
        begin
		  N = 3'b010;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b0;
          MuxC_Ld =dummy1;
          PC_Ld = 1'b1;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000010;
          func = opcode;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b01; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		//JAL
		    8'd97:
        begin
		  N = 3'b011;;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b0;
          MuxC_Ld = dummy1;
          PC_Ld = 1'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000011;
          func = opcode;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		//JAL (Final state)
		    8'd98:
        begin
		
		  N = 3'b010;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b0;
          MuxC_Ld =dummy1;
          PC_Ld = 1'b1;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000011;
          func = opcode;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b01; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		//JALR
		    8'd99:
        begin
		
		  N = 3'b011;;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b1;
          MuxC_Ld =1;
          PC_Ld = 1'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000000;
          func = 6'b001001;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		//JALR (final state)
		    8'd100:
        begin
	
		  N = 3'b010;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b1;
          MuxC_Ld = dummy1;
          PC_Ld = 1'b1;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000000;
          func = 6'b001001;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;


          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		//JR
		    8'd101:
        begin
		  N = 3'b010;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b1;
          MuxC_Ld =1'b1;
          PC_Ld = 11'b1;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000000;
          func = 6'b001000;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		//MFCO
		    8'd102:
        begin
		  N = 3'b010;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b1;
          MuxC_Ld =1'b1;
          PC_Ld = 11'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b010000;
          func = 6'b010000;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		//MTC0
		    8'd103:
        begin
		  N = 3'b010;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b1;
          MuxC_Ld =1'b1;
          PC_Ld = 11'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000000;
          func = 6'b010000;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		//TEQ (first state)
		    8'd104:
        begin
		  //condition *****************
		      N = 3'b011;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b0;
          MuxC_Ld =1'b0;
          PC_Ld = 11'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000000;
          func = 6'b110100;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		//TEQ (Final State)
		    8'd105:
        begin
		  N = 3'b010;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b1;
          MuxC_Ld =1'b1;
          PC_Ld = 11'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000000;
          func = 6'b110100;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		//TGE (first state)
		    8'd106:
        begin
		   //condition *****************
		  N = 3'b011;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b0;
          MuxC_Ld =1'b0;
          PC_Ld = 11'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000000;
          func = 6'b110000;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		    8'd107:
        begin
	      N = 3'b010;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b1;
          MuxC_Ld =1'b1;
          PC_Ld = 11'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000000;
          func = 6'b110000;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		//TGEU
		    8'd108:
        begin
          //condition *****************
		  N = 3'b011;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b0;
          MuxC_Ld =1'b0;
          PC_Ld = 11'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000000;
          func = 6'b110001;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;
          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		
		    8'd109:
        begin
		 N = 3'b010;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b1;
          MuxC_Ld =1'b1;
          PC_Ld = 11'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000000;
          func = 6'b110001;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		
		    8'd110:
        begin
      //condition *****************
		  N = 3'b011;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b0;
          MuxC_Ld =1'b0;
          PC_Ld = 11'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000000;
          func = 6'b110010;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		    8'd111:
        begin
		  N = 3'b010;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b1;
          MuxC_Ld =1'b1;
          PC_Ld = 11'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000000;
          func = 6'b110010;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		    8'd112:
        begin
		  //condition *****************
		  N = 3'b011;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b0;
          MuxC_Ld =1'b0;
          PC_Ld = 11'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000000;
          func = 6'b110011;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;
          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		    8'd113:
        begin
		  N = 3'b010;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b1;
          MuxC_Ld =1'b1;
          PC_Ld = 11'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000000;
          func = 6'b110011;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		//TNE (first state)
		    8'd114:
        begin
		  //condition *****************
		  N = 3'b011;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b0;
          MuxC_Ld =1'b0;
          PC_Ld = 11'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000000;
          func = 6'b110110;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		
		    8'd115:
        begin
		  N = 3'b011;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b1;
          MuxC_Ld =1'b1;
          PC_Ld = 11'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000000;
          func = 6'b110110;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end

		    8'd116:
        begin
	  //condition *****************
		  N = 3'b011;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b0;
          MuxC_Ld =1'b0;
          PC_Ld = 11'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000001;
          func = opcode;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		
		    8'd117:
          begin
          N = 3'b010;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b1;
          MuxC_Ld =1'b1;
          PC_Ld = 11'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000001;
          func = opcode;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		
		    8'd118:
        begin
		  //condition *****************
		  N = 3'b011;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b0;
          MuxC_Ld =1'b0;
          PC_Ld = 11'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000001;
          func = opcode;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		
		    8'd119:
        begin
		  N = 3'b010;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b1;
          MuxC_Ld =1'b1;
          PC_Ld = 11'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000001;
          func = opcode;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		
		    8'd120:
        begin
			  //condition *****************
		  N = 3'b011;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b0;
          MuxC_Ld =1'b0;
          PC_Ld = 11'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000001;
          func = opcode;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		    8'd121:
        begin
          N = 3'b010;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b1;
          MuxC_Ld =1'b1;
          PC_Ld = 11'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000001;
          func = opcode;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		
		    8'd122:
        begin
				  //condition *****************
		  N = 3'b011;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b0;
          MuxC_Ld =1'b0;
          PC_Ld = 11'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000001;
          func = opcode;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;
		  
          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		    8'd123:
        begin
          N = 3'b010;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b1;
          MuxC_Ld =1'b1;
          PC_Ld = 11'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000001;
          func = opcode;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		
		    8'd124:
        begin
				  //condition *****************
		  N = 3'b011;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b0;
          MuxC_Ld =1'b0;
          PC_Ld = 11'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000001;
          func = opcode;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		
		    8'd125:
        begin
		  N = 3'b010;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b1;
          MuxC_Ld =1'b1;
          PC_Ld = 11'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000001;
          func = opcode;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		
		    8'd126:
        begin
		  //condition *****************
		  N = 3'b011;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b0;
          MuxC_Ld =1'b0;
          PC_Ld = 11'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000001;
          func = opcode;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		
		    8'd127:
        begin
		  N = 3'b010;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b1;
          MuxC_Ld =1'b1;
          PC_Ld = 11'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000001;
          func = opcode;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		
		    8'd128:
        begin
		  N = 3'b010;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b1;
          MuxC_Ld =0;
          PC_Ld = 1'b1;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b010000;
          func = 6'b011000;
          MuxA_Ld = 2'b00;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		
		    8'd129:
        begin
	  //condition *****************
		  N = 3'b011;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b0;
          MuxC_Ld =1'b0;
          PC_Ld = 11'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000000;
          func = 6'b011001;
          MuxA_Ld = 2'b00;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b1;
          Lo_Ld = 1'b1;
          MuxReg_Ld = 2'b01;
          CR = 6'b000001; 
          state = Y;
          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		
		    8'd130:
        begin
	      N = 3'b010;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b1;
          MuxC_Ld =1'b1;
          PC_Ld = 11'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000000;
          func = 6'b011001;
          MuxA_Ld = 2'b00;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b1;
          Lo_Ld = 1'b1;
          MuxReg_Ld = 2'b01;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		
		    8'd131:
        begin
 //condition *****************
		  N = 3'b011;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b0;
          MuxC_Ld =1'b0;
          PC_Ld = 11'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000000;
          func = 6'b011010;
          MuxA_Ld = 2'b00;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b1;
          Lo_Ld = 1'b1;
          MuxReg_Ld = 2'b10;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		
		8'd132:
        begin
	      N = 3'b010;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b1;
          MuxC_Ld =1'b1;
          PC_Ld = 11'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000000;
          func = 6'b011010;
          MuxA_Ld = 2'b00;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b1;
          Lo_Ld = 1'b1;
          MuxReg_Ld = 2'b10;
          CR = 6'b000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
          end
//Jaha End    
	endcase
endmodule


module control_register(output reg[2:0] N,
 output reg Inv,
 output reg[1:0] S,
 output reg IR_Ld,
 output reg MAR_Ld,
 output reg MDR_Ld,
 output reg MuxMAR_Ld,
 output reg RF_Ld,
 output reg MuxC_Ld,
 output reg PC_Ld,
 output reg nPC_Ld,
 output reg MuxMDR_Ld,
 output reg MOV,
 output reg RW,
 output reg Hi_Ld, Lo_Ld,
 output reg[5:0] opcode,
 output reg[5:0] func,
 output reg[1:0] MuxA_Ld, MuxB_Ld, MuxReg_Ld,
 output reg[5:0] CR,
 output reg[7:0] state, 
 input [50:0] Ds,
 input Clr, Clk);
  always @(posedge Clk)

      begin

        state=Ds[7:0];
        CR=Ds[13:8]; 
        MuxReg_Ld=Ds[15:14];
        Lo_Ld=Ds[16];
        Hi_Ld=Ds[17];
        MuxB_Ld=Ds[19:18]; 
        MuxA_Ld=Ds[21:20]; 
        opcode=Ds[27:22];
        func=Ds[33:28];
        RW=Ds[34]; 
        MOV=Ds[35]; 
        MuxMDR_Ld=Ds[36]; 
        nPC_Ld=Ds[37]; 
        PC_Ld=Ds[38]; 
        MuxC_Ld=Ds[39]; 
        RF_Ld=Ds[40]; 
        MuxMAR_Ld=Ds[41]; 
        MDR_Ld=Ds[42]; 
        MAR_Ld=Ds[43]; 
        IR_Ld=Ds[44]; 
        S=Ds[46:45]; 
        Inv=Ds[47]; 
        N=Ds[50:48];  

      end

endmodule

