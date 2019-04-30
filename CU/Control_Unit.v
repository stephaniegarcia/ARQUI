module test;
  reg [6:0] state1;
  reg Clr, Clk;
  reg [5:0] count;
  reg [31:0] IR, clear;
  wire [6:0] Y, state;
  wire COND, MOC, IR_Ld, MAR_Ld,
  MDR_Ld, MuxMAR_Ld, RF_Ld, MuxC_Ld,
  PC_Ld, nPC_Ld, MuxMDR_Ld, MOV, RW, Hi_Ld, Lo_Ld;
  wire [5:0] opcode;
  wire[1:0] MuxA_Ld, MuxB_Ld, MuxReg_Ld;
  reg[43:0] test;
  wire inputCarry, negativeFlag, zeroFlag, carryFlag, overflowFlag;
Control_Unit CU(state, Y, state1, COND, IR, MOC, count, IR_Ld, MAR_Ld, MDR_Ld, MuxMAR_Ld, RF_Ld, MuxC_Ld, PC_Ld, nPC_Ld, MuxMDR_Ld, MOV, RW, Hi_Ld, Lo_Ld, opcode, MuxA_Ld, MuxB_Ld, MuxReg_Ld, Clr, Clk);

initial begin
count = 6'd0;
Clr = 1'b0;
#11 Clr = 1'b1;
end


initial #3000 $finish;
initial begin
IR = 32'b00000000001000010000000000100001;
#60
IR = 32'b10100000001000010000000000100001;
#85
IR = 32'b00010000001000010000000000100001;
#55
IR = clear;

end

assign MOC = 1'b0;
assign COND = 1'b0;
assign initstate = 7'd0;
// esta parte es par el reloj
initial begin
Clk =1'b0;
forever #5 Clk = ~Clk;
end

initial begin
  state1 = 7'd0;
end
initial begin
$display("                     Data                                   IR                  State   ");
#5
$display("\n   Data %b   %b     %d\n", CU.data_out, IR, CU.st);
repeat(20) begin
#10
$display("   Data %b    %b    %d\n", CU.data_out, IR,  CU.st);


end
end
endmodule



module Control_Unit(output [6:0] state,
                    output [6:0] Y,
                    input  [6:0] state1 ,
                    input COND,
                    input [31:0] IR,
                    input MOC,
                    input [5:0] count,
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
                    output [1:0] MuxA_Ld, MuxB_Ld, MuxReg_Ld,
                    input Clr, Clk
                    output inputCarry,
                    input negativeFlag, zeroFlag, carryFlag, overflowFlag
                    );

// outputs
wire [5:0] CR;  // esto es para los estados
wire Sts ,in,Inv;  // aqui los el inverter y NSAddressSelector
wire[1:0] M, S;  // para los moc  y NSASelector
wire [2:0] N;  // para NSASelecto
wire [6:0] D;  // salida del adder
wire [6:0] Q; // salida del incremento
wire[43:0] data_out;
wire[6:0] st;
// instancias entre moduluos
IR_Encoder enc( state, IR,Clk);
MUX_COND mxcnd( in, MOC,COND, S);
inverter inv( Sts, in, Inv);
next_state_address_selector nsas( M,  N, Sts);
state_adder stadd( D, st, 7'd1);
incrementer_register increg( Q,  D, Clk);
MUX_ROM mxrom( st, state, state1, CR, Q, M, Clr);
microStore mcStr(st, data_out, Y);
control_register Creg(N, Inv, S, IR_Ld, MAR_Ld, MDR_Ld, MuxMAR_Ld, RF_Ld, MuxC_Ld, PC_Ld, nPC_Ld, MuxMDR_Ld, MOV, RW, Hi_Ld, Lo_Ld, opcode, MuxA_Ld, MuxB_Ld, MuxReg_Ld, CR,Y, data_out, Clr, Clk, inputCarry, negativeFlag, zeroFlag, carryFlag, overflowFlag);

endmodule//////////////////////////////////////////////////////////////////////////////////////////


module IR_Encoder(output reg [6:0] state, input [31:0] IR, input Clk);

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
          6'b100000: state = 7'd5;
          6'b100001: state = 7'd6;
          6'b100010: state = 7'd9;
          6'b100011: state = 7'd10;
          6'b101010: state = 7'd11;
          6'b101011: state = 7'd13;
          6'b100100: state = 7'd21;
          6'b100101: state = 7'd23;
          6'b100110: state = 7'd25;
          6'b100111: state = 7'd27;
          6'b000000: state = 7'd29;
          6'b000100: state = 7'd30;
          6'b000011: state = 7'd31;
          6'b000111: state = 7'd32;
          6'b000010: state = 7'd33;
          6'b000110: state = 7'd34;
          6'b010000: state = 7'd71;
          6'b010010: state = 7'd72;
          6'b001011: state = 7'd73;
          6'b001010: state = 7'd74;
          6'b010001: state = 7'd75;
          6'b010011: state = 7'd76;
          6'b001001: state = 7'd99;
          6'b001000: state = 7'd101;
          6'b010000: state = 7'd103;
          6'b110100: state = 7'd104;
          6'b110000: state = 7'd106;
          6'b110001: state = 7'd108;
          6'b110010: state = 7'd110;
          6'b110011: state = 7'd112;
          6'b110110: state = 7'd114;
          6'b011001: state = 7'd129;
          6'b011010: state = 7'd131;
        endcase
      6'b010000:
        case(func)
          6'b010000: state = 7'd102;
          6'b011000: state = 7'd128;
        endcase
      6'b001000: state = 7'd7;
      6'b001001: state = 7'd8;
      6'b001010: state = 7'd15;
      6'b001011: state = 7'd17;
      6'b011100:
        case(func)
          6'b100001: state = 7'd19;
          6'b100000: state = 7'd20;
        endcase
      6'b001100: state = 7'd22;
      6'b001101: state = 7'd24;
      6'b001110: state = 7'd26;
      6'b001111: state = 7'd28;
      6'b100011: state = 7'd35;
      6'b100001: state = 7'd39;
      6'b100101: state = 7'd43;
      6'b100000: state = 7'd47;
      6'b100100: state = 7'd51;
      6'b111111: state = 7'd55;
      6'b101011: state = 7'd59;
      6'b101001: state = 7'd63;
      6'b101000: state = 7'd67;
      6'b000001:
        case(rt)
          5'b10001: state = 7'd84;
          5'b00000: state = 7'd92;
        endcase
      6'b000100: state = 7'd80;
      6'b000101: state = 7'd94;
      6'b000010: state = 7'd96;
      6'b000011: state = 7'd97;
      6'b000001:
        case(rt)
          5'b01100: state = 7'd116;
          5'b10000: state = 7'd118;
          5'b01001: state = 7'd120;
          5'b01010: state = 7'd122;
          5'b01011: state = 7'd124;
          5'b01110: state = 7'd126;
        endcase
    endcase
endmodule

module MUX_COND(output reg out, input I0, I1, input [1:0] S);
  always @(S)
    case(S)
    2'b00: out = I0;
    2'b01: out = I1;
    endcase

endmodule

module MUX_ROM(output reg [6:0] Y, input [6:0] I0, I1, input[5:0] I2, input[6:0] I3, input [1:0] M, input Clr);
  always @*

//Clr redirects to state 0 and clears CR
  if(!Clr)
    Y = 7'd0;
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

module state_adder(output [6:0] Z, input [6:0] A, B);
  assign Z = A + B;

endmodule

module incrementer_register(output reg [6:0] Q, input [6:0] D, input  Clk);
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

module microStore(input [6:0] Y, output reg [43:0] data_out, output reg [6:0] state);

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
        7'd0:
        begin
            N = 3'b011;//  32
            Inv = 1'b0;//  29
            S = 2'b00;//  28
            IR_Ld = 1'b0;//  26
            MAR_Ld = 1'b0;//  25
            MDR_Ld = 1'b0;//  24
            MuxMAR_Ld = 1'b0;//  23
            RF_Ld = 1'b0;// 22
            MuxC_Ld = 1'b0;// 21
            PC_Ld = 1'b0;// 20
            nPC_Ld = 1'b0;// 19
            MuxMDR_Ld = 1'b0;// 18
            MOV = 1'b0;// 17
            RW = 1'b0;// 16
            opcode = 6'b000000;// 15
            MuxA_Ld = 2'b00;// 9
            MuxB_Ld = 2'b00; // 7
            Hi_Ld = 1'b0;
            Lo_Ld = 1'b0;
            MuxReg_Ld = 2'b00;
            CR = 6'b000001; // 5
            state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};

          end


         7'd1:
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
          MuxA_Ld = 2'b01;// 8
          MuxB_Ld = 2'b01; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;
          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};


        end
         7'd2:
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
          opcode = 6'b000000;// 14
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000011; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};

        end
         7'd3:
        begin
            N = 3'b101;//  32
            Inv = 1'b0;//  28
            S = 2'b01;//  27
            IR_Ld = 1'b1;//  25
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
            opcode = 6'b000000;// 14
            MuxA_Ld = 2'b00;// 8
            MuxB_Ld = 2'b00; // 6
            Hi_Ld = 1'b0;
            Lo_Ld = 1'b0;
            MuxReg_Ld = 2'b00;
            CR = 6'b000011; //4
            state = Y;
          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};

        end
         7'd4:
        begin
          N = 3'b000;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = dummy1;//  25
          MAR_Ld = dummy1;//  24
          MDR_Ld = dummy1;//  23
          MuxMAR_Ld = dummy1;//  22
          RF_Ld = dummy1;// 21
          MuxC_Ld =dummy1;// 20
          PC_Ld = dummy1;// 19
          nPC_Ld = dummy1;// 18
          MuxMDR_Ld = dummy1;// 17
          MOV = dummy1;// 16
          RW = dummy1;// 15
          opcode = dummy3;// 14
          MuxA_Ld = dummy2;// 8
          MuxB_Ld = dummy2; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;
          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};

        end
      7'd5:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
      7'd6:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
      7'd7:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
      7'd8:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd9:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd10:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd11:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd12:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd13:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd14:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd15:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd16:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd17:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
         7'd18:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd19:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd20:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd21:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd22:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd23:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd24:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd25:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd26:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd27:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd28:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd29:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd30:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd31:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd32:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd33:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd34:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd35:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd36:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd37:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd38:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b10; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd39:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd40:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd41:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd42:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b10; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd43:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd44:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
        7'd45:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
       //Ale End

       //Steph Begin
        7'd46:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b10; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

<<<<<<< HEAD
          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,CR};
        end
=======
          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state}; end      
>>>>>>> c1c52d94aa7414df7a8fb81c17014eccecb05179
           7'd47:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
           state = Y;

           data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};  end
           7'd48:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

    data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};     end
           7'd49:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
         state = Y;

         data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state}; end
           7'd50:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b10; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
         state = Y;

        data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};     end
           7'd51:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
         state = Y;

           data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};      end
           7'd52:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
         state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state}; end
           7'd53:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state}; end
           7'd54:
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
          MuxA_Ld = 2'b11;// 8
          MuxB_Ld = 2'b01; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

           data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state}; end
           7'd55:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
         state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd56:
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
          MuxA_Ld = 2'b10;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd57:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd58:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
         state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd59:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
        state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd60:
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
          MuxA_Ld = 2'b10;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
        state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd61:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
         state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd62:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
         state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd63:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;
<<<<<<< HEAD

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,CR};
        end
=======
   
 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
>>>>>>> c1c52d94aa7414df7a8fb81c17014eccecb05179
           7'd64:
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
          MuxA_Ld = 2'b10;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd65:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd66:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd67:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
         state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd68:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
         state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd69:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd70:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
     7'd71:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd72:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd73:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
         state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd74:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd75:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd76:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd77:
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
          MuxA_Ld = 2'b01;// 8
          MuxB_Ld = 2'b10; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
         state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd78:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          CR = 6'b000001; //4
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd79:
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
          MuxA_Ld = 2'b01;// 8
          MuxB_Ld = 2'b10; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
      7'd80:
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
          MuxA_Ld = 2'b10;// 8
          MuxB_Ld = 2'b01; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd81:
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
          MuxA_Ld = 2'b01;// 8
          MuxB_Ld = 2'b10; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
         state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd82:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd83:
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
          MuxA_Ld = 2'b01;// 8
          MuxB_Ld = 2'b10; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd84:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
         state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd85:
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
          MuxA_Ld = 2'b01;// 8
          MuxB_Ld = 2'b10; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd86:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd87:
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
          MuxA_Ld = 2'b01;// 8
          MuxB_Ld = 2'b10; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd88:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd89:
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
          MuxA_Ld = 2'b01;// 8
          MuxB_Ld = 2'b10; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
           7'd90:
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
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001; //4
          state = Y;

 data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};        end
      //Steph End

      //Jaha Begin
      //BLTZ
		    7'd91:
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
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b01;
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
		//BLTZAL
		    7'd92:
        begin

		//NOT YET ****************** Condition
		//  N = ;
        //  Inv = 1'b;
        //  S = 2'b;
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
          MuxA_Ld = 2'b00;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
		//BLTZAL (final state)
		    7'd93:
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
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b01;
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
		  CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
		//BNE
		    7'd94:
        begin


	//NOT YET ****************** Condition
		//  N = ;
        //  Inv = 1'b;
        //  S = 2'b;
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
          opcode = 6'000101;
          MuxA_Ld = 2'b00;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end

		//BNE Final State
		    7'd95:
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
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b01;
          Hi_Ld = 1'b0;.
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
		//J
         7'd96:
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
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b01;
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
		//JAL
		    7'd97:
        begin
		  N = 3'011;;
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
          opcode = 6'000011;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
		//JAL (Final state)
		    7'd98:
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
          opcode = 6'000011;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b01;
          Hi_Ld = 1'b0;.
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
		//JALR
		    7'd99:
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
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b00;
          Lo_Ld = 1'b00;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
		//JALR (final state)
		    7'd100:
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
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b00;
          Lo_Ld = 1'b00;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;


          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
		//JR
		    7'd101:
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
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b00;
          Lo_Ld = 1'b00;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
		//MFCO
		    7'd102:
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
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b00;
          Lo_Ld = 1'b00;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
		//MTC0
		    7'd103:
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
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b00;
          Lo_Ld = 1'b00;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
		//TEQ (first state)
		    7'd104:
        begin
		  //condition *****************
		  //N = 3'b010;
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
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b00;
          Lo_Ld = 1'b00;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
		//TEQ (Final State)
		    7'd105:
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
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b00;
          Lo_Ld = 1'b00;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
		//TGE (first state)
		    7'd106:
        begin
		   //condition *****************
		  //N = 3'b010;
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
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b00;
          Lo_Ld = 1'b00;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
		    7'd107:
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
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b00;
          Lo_Ld = 1'b00;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
		//TGEU
		    7'd108:
        begin
          //condition *****************
		  //N = 3'b010;
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
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b00;
          Lo_Ld = 1'b00;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;
          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end

		    7'd109:
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
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b00;
          Lo_Ld = 1'b00;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end

		    7'd110:
        begin
      //condition *****************
		  //N = 3'b010;
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
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b00;
          Lo_Ld = 1'b00;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
		    7'd111:
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
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b00;
          Lo_Ld = 1'b00;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
		    7'd112:
        begin
		  //condition *****************
		  //N = 3'b010;
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
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b00;
          Lo_Ld = 1'b00;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;
          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
		    7'd113:
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
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b00;
          Lo_Ld = 1'b00;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
		//TNE (first state)
		    7'd114:
        begin
		  //condition *****************
		  //N = 3'b010;
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
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b00;
          Lo_Ld = 1'b00;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end

		    7'd115:
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
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b00;
          Lo_Ld = 1'b00;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end

		    7'd116:
        begin
	  //condition *****************
		  //N = 3'b010;
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
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b00;
          Lo_Ld = 1'b00;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end

		    7'd117:
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
          opcode = 6'b000001;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b00;
          Lo_Ld = 1'b00;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end

		    7'd118:
        begin
		  //condition *****************
		  //N = 3'b010;
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
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b00;
          Lo_Ld = 1'b00;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end

		    7'd119:
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
          opcode = 6'b000001;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b00;
          Lo_Ld = 1'b00;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end

		    7'd120:
        begin
			  //condition *****************
		  //N = 3'b010;
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
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b00;
          Lo_Ld = 1'b00;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
		    7'd121:
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
          opcode = 6'b000001;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b00;
          Lo_Ld = 1'b00;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end

		    7'd122:
        begin
				  //condition *****************
		  //N = 3'b010;
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
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b00;
          Lo_Ld = 1'b00;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end
		    7'd123:
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
          opcode = 6'b000001;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b00;
          Lo_Ld = 1'b00;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end

		    7'd124:
        begin
				  //condition *****************
		  //N = 3'b010;
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
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b00;
          Lo_Ld = 1'b00;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end

		    7'd125:
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
          opcode = 6'b000001;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b00;
          Lo_Ld = 1'b00;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end

		    7'd126:
        begin
		  //condition *****************
		  //N = 3'b010;
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
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b00;
          Lo_Ld = 1'b00;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end

		    7'd127:
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
          opcode = 6'b000001;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b00;
          Lo_Ld = 1'b00;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end

		    7'd128:
        begin
		  N = 3'b011;
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
          MuxA_Ld = 2'b00;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end

		    7'd129:
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
          opcode = 6'000000;
          MuxA_Ld = 2'b00;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b1;
          Lo_Ld = 1'b1;
          MuxReg_Ld = 2'b01;
          CR = 6'b000001;
          state = Y;
          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end

		    7'd130:
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
          opcode = 6'000000;
          MuxA_Ld = 2'b00;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b1;
          Lo_Ld = 1'b1;
          MuxReg_Ld = 2'b01;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end

		    7'd131:
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
          opcode = 6'000000;
          MuxA_Ld = 2'b00;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b1;
          Lo_Ld = 1'b1;
          MuxReg_Ld = 2'b10;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
        end

		7'd132:
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
          opcode = 6'000000;
          MuxA_Ld = 2'b00;
          MuxB_Ld = 2'b00;
          Hi_Ld = 1'b1;
          Lo_Ld = 1'b1;
          MuxReg_Ld = 2'b10;
          CR = 6'b000001;
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};
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
 output reg[1:0] MuxA_Ld, MuxB_Ld, MuxReg_Ld,
 output reg[5:0] CR,
 output reg[6:0] state,
 input [43:0] Ds,
 input Clr, Clk);
  always @(posedge Clk)

      begin

        state=Ds[6:0];
        CR=Ds[12:7];
        MuxReg_Ld=Ds[14:13];
        Lo_Ld=Ds[15];
        Hi_Ld=Ds[16];
        MuxB_Ld=Ds[18:17];
        MuxA_Ld=Ds[20:19];
        opcode=Ds[26:21];
        RW=Ds[27];
        MOV=Ds[28];
        MuxMDR_Ld=Ds[29];
        nPC_Ld=Ds[30];
        PC_Ld=Ds[31];
        MuxC_Ld=Ds[32];
        RF_Ld=Ds[33];
        MuxMAR_Ld=Ds[34];
        MDR_Ld=Ds[35];
        MAR_Ld=Ds[36];
        IR_Ld=Ds[37];
        S=Ds[39:38];
        Inv=Ds[40];
        N=Ds[43:41];

      end

endmodule
