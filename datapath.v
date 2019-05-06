
module datapath(input Clr, Clk);

//wires to be used on instantiating the modules on datapath
  reg [7:0] state1;
  wire [7:0] Y, state, st;
  wire COND, MOC, IR_Ld, MAR_Ld, MDR_Ld, MuxMAR_Ld, RF_Ld, MuxC_Ld, PC_Ld, nPC_Ld, MuxMDR_Ld, MOV, RW, Hi_Ld, Lo_Ld;
	    wire inputCarry, negativeFlag, zeroFlag, carryFlag, overflowFlag;

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

	mux_4x1_32 MUXA(Mux_a_out, MuxA_Ld, PA, PC_out,  nPC_out, 32'b0);

	mux_4x1_32 MUXB(Mux_b_out, MuxB_Ld, PB, se_out, MDR_out, 32'd4);

  mux_2x1_5 MUXC(Mux_c_out, MuxC_Ld, IR_out[15:11], IR_out[20:16]);

  mux_3x1_32 MUXREG(Mux_reg_out, MuxReg_Ld, ALU_out, hi_out, lo_out);

	signextend SE(se_out, IR_out[15:0]);

  registernPC nPC(nPC_out, nPC_Ld, ALU_out, Clr, Clk);

	registerclr PC(PC_out, PC_Ld, nPC_out, Clr, Clk);

	registerIR IR(IR_out, IR_Ld, DataOut, Clr, Clk);

	registerclr LO(lo_out, Lo_Ld, ALU_out, Clr, Clk);

	registerclr HI(hi_out, Hi_Ld, ALU_out, Clr, Clk);

	mux_2x1_32 MUXMAR(Mux_MAR_out, MuxMAR_Ld, PC_out, ALU_out);

	mux_2x1_32 MUXMDR(Mux_MDR_out, MuxMDR_Ld, ALU_out, DataOut);

	registerclr MAR(MAR_out, MAR_Ld, Mux_MAR_out, Clr, Clk);

	registerclr MDR(MDR_out, MDR_Ld, Mux_MDR_out, Clr, Clk);

	ram512x8 RAM(DataOut, MOC, RW, MOV, MAR_out, MDR_out, opcode);

 registerFile rg(PA, PB, Mux_reg_out, IR_out[25:21], IR_out[20:16], Mux_c_out, RF_Ld, Clk);
  // RegisterFile rg(PA, PB, Mux_reg_out, Mux_c_out, RF_Ld, Clk, IR_out[25:21], IR_out[20:16], Clr);

	ALU ALU(ALU_out, N, Z, C, V, func, Mux_a_out, Mux_b_out);


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

module mux_3x1_32(output reg [31:0] O, input [2:0] Sel, input [31:0] r0, input [31:0] r1, input [31:0] r2);
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
module registernPC(output reg [31:0]Q, input ld, input [31:0]D, input Clr, input Clk);
  always @ (posedge Clk, posedge Clr)
		if(Clr) Q = 32'd4;
		else if(ld)
			Q = D;
endmodule


module registerIR(output reg [31:0]Q, input ld, input [31:0]D, input Clr, input Clk);
  always @ (posedge Clk, posedge Clr)

	if(ld)
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
