 module ram512x8(output reg [31:0] DataOut, output reg MOC, input ReadWrite, MOV,input[31:0] Address,
				 input[31:0] DataIn,
				 input [5:0] OP //gives word type (byte, halfword, word)
				 );
	reg[7:0] Mem[0:511]; //512 bytes of memory total
	always @(posedge MOV, ReadWrite)
		begin
			if(MOV)
			begin
				MOC = 0;
				if(ReadWrite) begin //Read
					case(OP)
					//Word (32bits)
						6'b100011:begin
							DataOut[31:24] <= Mem[Address];
							DataOut[23:16] <= Mem[Address+1];
							DataOut[15:8]  <= Mem[Address+2];
							DataOut[7:0]   <= Mem[Address+3];
							end
					//Half-Word (16 bits)
						6'b100001:begin
							DataOut[31:16]  <= 16'h0000; //16 bits zeroes
							DataOut[15:8]  <= Mem[Address];
							DataOut[7:0]   <= Mem[Address+1];
							end
					//Byte (8 bits)
						6'b100000:begin
							DataOut[31:8]  <= 24'h000000; //24 bits zeroes
							DataOut[7:0]    <= Mem[Address];
						end
					endcase
					MOC = 1;
					end
				else //Write
				begin
					case(OP)
					//Word (32bits)
						6'b101011:begin
							Mem[Address] <= DataIn[31:24];
							Mem[Address+1] <= DataIn[23:16];
							Mem[Address+2] <= DataIn[15:8];
							Mem[Address+3] <= DataIn[7:0];
						end

					//Half-Word	(16bits)
						6'b101001:begin
							Mem[Address] <= DataIn[15:8];
							Mem[Address+1] <= DataIn[7:0];
						end
					//Byte (8 bits)
						6'b101000:begin
							Mem[Address] = DataIn[7:0];
						end
					endcase
					MOC = 1;
				end
			end
		end
endmodule

module mux_2x1_32(output reg [31:0] O, input Sel, input [31:0] r0, input [31:0] r1);
 always @ (r0,r1,Sel) begin
    case(Sel)
    1'd0: O = r0;
    1'd1: O = r1;
   endcase
 end
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
    2'd0: O <= r0;
    2'd1: O <= r1;
	2'd2: O <= r2;
    2'd3: O <= r3;
   endcase
 end
endmodule

module mux_3x1_32(output reg [31:0] O, input [1:0] Sel, input [31:0] r0, input [31:0] r1, input [31:0] r2);
 always @ (r0,r1,r2,Sel) begin
    case(Sel)
    2'd0: O <= r0;
    2'd1: O <= r1;
	2'd2: O <= r2;
   endcase
 end
endmodule

module registerclr(output reg [31:0]Q, input ld, input [31:0]D, input Clr, input Clk);
  always @ (posedge Clk, posedge Clr)        // At positive edge
		if(Clr) Q <= 32'h0000_0000;
		else if(ld)
			Q <= D;           // Load Enable, then store data
endmodule

module shifter(output reg[31:0] extension, input [15:0] instruction);
  reg[31:0] sub_temp;

  always @ ( instruction )
   begin

    sub_temp[15:0] = 2  <<< instruction[15:0];
      extension <= {sub_temp[15],sub_temp[15],sub_temp[15],sub_temp[15],
		  sub_temp[15],sub_temp[15],sub_temp[15],sub_temp[15],
		  sub_temp[15],sub_temp[15],sub_temp[15],sub_temp[15],
		  sub_temp[15],sub_temp[15],sub_temp[15],sub_temp[15],sub_temp[15:0]};
  end



endmodule


module datapath(input Clr, Clk);

//wires to be used on instantiating the modules on datapath
    reg [6:0] state1;
    reg [31:0] IR;
    wire [6:0] Y, state;
    wire COND, MOC, IR_Ld, MAR_Ld, MDR_Ld, MuxMAR_Ld, RF_Ld, MuxC_Ld, PC_Ld, nPC_Ld, MuxMDR_Ld, MOV, RW, Hi_Ld, Lo_Ld;
    wire inputCarry, negativeFlag, zeroFlag, carryFlag, overflowFlag;
    wire [5:0] opcode;
    wire[1:0] MuxA_Ld, MuxB_Ld, MuxReg_Ld;

    wire[31:0] DataOut;

	wire[31:0] PC_out;

	wire[31:0] nPC_out;

	wire[31:0] MAR_out, Mux_MAR_out;

	wire[31:0] MDR_out, Mux_MDR_out;

	wire[31:0] ALU_out;
	// wire[5:0] func;

	wire[31:0] IR_out;

	wire[31:0] PA, PB;
	wire[4:0] Mux_c_out;

	wire[31:0] Mux_a_out, Mux_b_out, Mux_reg_out;

    wire[31:0] lo_out, hi_out;

// Modules are Instantiated here. Missing ALU

	Control_Unit CU(state, Y, state1, COND, IR, MOC, IR_Ld, MAR_Ld, MDR_Ld, MuxMAR_Ld, RF_Ld, MuxC_Ld, PC_Ld, nPC_Ld, MuxMDR_Ld, MOV, RW, Hi_Ld, Lo_Ld, opcode, MuxA_Ld, MuxB_Ld, MuxReg_Ld, Clr, Clk, inputCarry, negativeFlag, zeroFlag, carryFlag, overflowFlag);

	mux_4x1_32 MUXA(Mux_a_out, MuxA_Ld, PA, PC_out,  nPC_out, 32'd0);

	mux_3x1_32 MUXB(Mux_b_out, MuxB_Ld, PB, se_out, MDR_out);

    mux_2x1_5 MUXC(Mux_c_out, MuxC_Ld, IR_out[15:11], IR_out[20:16]);

    mux_3x1_32 MUXREG(Mux_reg_out, MuxReg_Ld, ALU_out, hi_out, lo_out);

	registerclr NPC(nPC_out, nPC_Ld, ALU_out, Clr, Clk);

	registerclr PC(PC_out, PC_Ld, nPC_out, Clr, Clk);

	registerclr IR(IR_out, IR_Ld, DataOut, Clr, Clk);

	registerclr LO(lo_out, Lo_Ld, ALU_out, Clr, Clk);

	registerclr HI(hi_out, Hi_Ld, ALU_out, Clr, Clk);

  shifter Shifter_SignExt(se_out,IR_out);

	mux_2x1_32 MUXMAR(Mux_MAR_out, MuxMAR_Ld, PC_out, ALU_out);

	mux_2x1_32 MUXMDR(Mux_MDR_out, MuxMDR_Ld, ALU_out, DataOut);

	registerclr MAR(MAR_out, MAR_Ld, Mux_MAR_out, Clr, Clk);

	registerclr MDR(MDR_out, MDR_Ld, Mux_MDR_out, Clr, Clk);

	ram512x8 RAM(DataOut, MOC, RW, MOV, MAR_out, MDR_out, opcode);

  registerFile rg(PA, PB, IR_out[25:21], IR_out[20:16], Mux_reg_out, Mux_c_out, RF_Ld, Clk);

  alu ALU(Mux_a_out, Mux_b_out, opcode, ALU_out, inputCarry, negativeFlag, zeroFlag, carryFlag, overflowFlag);



endmodule
