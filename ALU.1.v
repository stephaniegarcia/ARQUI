
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


		6'b010110: Y <= A + 4 + 4 * B;                       // Operation: Branches and Jumps
		
		
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