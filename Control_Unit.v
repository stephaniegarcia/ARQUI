
module Control_Unit(output [7:0] state,
                    output [7:0] Y, st,
                    input  [7:0] state1, 
                    output COND,
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
wire [7:0] CR;  // esto es para los estados 
wire Sts ,in,Inv;  // aqui los el inverter y NSAddressSelector
wire[1:0] M, S;  // para los moc  y NSASelector
wire [2:0] N;  // para NSASelecto
wire [7:0] D;  // salida del adder  
wire [7:0] Q; // salida del incremento 
wire[52:0] data_out;
// instancias entre moduluos 
IR_Encoder enc( state, COND, IR,Clk);   
MUX_COND mxcnd( in, MOC, COND, S); 
inverter inv( Sts, in, Inv);  
next_state_address_selector nsas( M,  N, Sts); 
state_adder stadd( D, st, 8'd1); 
incrementer_register increg( Q,  D, Clk);
MUX_ROM mxrom( st, state, state1, CR, Q, M, Clr); 
microStore mcStr(st, data_out, Y);  
control_register Creg(N, Inv, S, IR_Ld, MAR_Ld, MDR_Ld, MuxMAR_Ld, RF_Ld, MuxC_Ld, PC_Ld, nPC_Ld, MuxMDR_Ld, MOV, RW, Hi_Ld, Lo_Ld, opcode, func, MuxA_Ld, MuxB_Ld, MuxReg_Ld, CR,Y, data_out, Clr, Clk);

endmodule//////////////////////////////////////////////////////////////////////////////////////////


module IR_Encoder(output reg [7:0] state, output reg COND, input [31:0] IR, input Clk);

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
          5'b00001: 
           begin
            COND = (rs>=0);
            state = 8'd82;
           end
          5'b10001: 
          begin
            COND = (rs>=0);
            state = 8'd84;
          end
          5'b00000: 
           begin
            COND = (rs<0);
            state = 8'd92;
          end
          5'b01100: state = 8'd116;
          5'b10000: state = 8'd118;
          5'b01001: state = 8'd120;
          5'b01010: state = 8'd122;
          5'b01011: state = 8'd124;
          5'b01110: state = 8'd126;
        endcase
      6'b000111: 
        case(rt)
          5'b00000: 
          begin
            COND = (rs>0);
            state = 8'd86;
          end
        endcase
      6'b000110: 
        
           begin
            COND = (rs<=0);
            state = 8'd88;
          end
           
      6'b000100: 
        begin
            COND = (rt==rs);
            state = 8'd80;
        end
      6'b000101: 
        begin
            COND = (rt!=rs);
            state = 8'd94;
        end
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

module MUX_ROM(output reg [7:0] Y, input [7:0] I0, I1, input[7:0] I2, input[7:0] I3, input [1:0] M, input Clr);
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

module microStore(input [7:0] Y, output reg [52:0] data_out, output reg [7:0] state);

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
  reg[7:0] CR;
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
            func = 6'b000000;
            MuxA_Ld = 2'b11;// 9
            MuxB_Ld = 2'b11; // 8
            Hi_Ld = 1'b0;
            Lo_Ld = 1'b0;
            MuxReg_Ld = 2'b00;
            CR = 8'b00000001; // 5
            state = Y;
          
          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
          end

         8'd1:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
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
          opcode = 6'b100011;// 14
          func = 6'b000000;
          MuxA_Ld = 2'b01;// 8
          MuxB_Ld = 2'b01; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
          state = Y;
          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 


        end
         8'd2:
        begin
          N = 3'b010;//  32
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
          opcode = 6'b100011;// 14
          func = 6'b000000;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000011; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 

        end
         8'd3:
        begin
            N = 3'b011;//  32
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
            opcode = 6'b100001;// 14
            func = 6'b000000;
            MuxA_Ld = 2'b00;// 8
            MuxB_Ld = 2'b00; // 6
            Hi_Ld = 1'b0;
            Lo_Ld = 1'b0;
            MuxReg_Ld = 2'b00;
            CR = 8'b00000011; //4
            state = Y;
          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 

        end
       
         8'd4:
        begin
          N = 3'b000;//  32
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
            func =6'b100001;
            MuxA_Ld = 2'b01;// 8
            MuxB_Ld = 2'b11; // 6
            Hi_Ld = 1'b0;
            Lo_Ld = 1'b0;
            MuxReg_Ld = 2'b00;
          CR = 8'b11111110; //4
          state = Y;
          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 

        end
        // 8'd254:
        // begin
        //     N = 3'b000;//  32
        //     Inv = 1'b0;//  28
        //     S = 2'b00;//  27
        //     IR_Ld = 1'b0;//  25
        //     MAR_Ld = 1'b0;//  24
        //     MDR_Ld = 1'b0;//  23
        //     MuxMAR_Ld = 1'b0;//  22
        //     RF_Ld = 1'b0;// 21
        //     MuxC_Ld = 1'b0;// 20
        //     PC_Ld = 1'b0;// 19
        //     nPC_Ld = 1'b0;// 18
        //     MuxMDR_Ld = 1'b0;// 17
        //     MOV = 1'b0;// 16
        //     RW = 1'b0;// 15
        //     opcode = 6'b000000;// 14
        //     func =6'b000000;
        //     MuxA_Ld = 2'b00;// 8
        //     MuxB_Ld = 2'b00; // 6
        //     Hi_Ld = 1'b0;
        //     Lo_Ld = 1'b0;
        //     MuxReg_Ld = 2'b00;
        //     CR = 8'b00000011; //4
        //     state = Y;
        //   data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 

        // end
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
          CR = 8'b00000001; //4
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
          CR = 8'b00000001; //4
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
          func = 6'b100011;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          func = 6'b001001;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b01; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          CR = 8'b00000001; //4
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
          CR = 8'b00000001; //4
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
          CR = 8'b00000001; //4
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
          CR = 8'b00000001; //4
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
          CR = 8'b00000001; //4
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
          CR = 8'b00000001; //4
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
          
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          CR = 8'b00000001; //4
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
          CR = 8'b00000001; //4
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
          CR = 8'b00000001; //4
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
          
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          CR = 8'b00000001; //4
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
          
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          CR = 8'b00000001; //4
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
          
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          CR = 8'b00000001; //4
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
          
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          CR = 8'b00000001; //4
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
          CR = 8'b00000001; //4
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
          CR = 8'b00000001; //4
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
          CR = 8'b00000001; //4
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
          CR = 8'b00000001; //4
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
          CR = 8'b00000001; //4
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
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b1;// 16
          RW = 1'b1;// 15
          opcode = 6'b100011;// 14
          func = 6'b100001;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b01; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
         state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd36:
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
          opcode = 6'b100011;// 14
          func = 6'b100001;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b01; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          MDR_Ld = 1'b1;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b1;// 16
          RW = 1'b1;// 15
          opcode = 6'b100011;// 14
          func = 6'b100001;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b01; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
        8'd38:
        begin
          N = 3'b010;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b0;//  25
          MAR_Ld = 1'b1;//  24
          MDR_Ld = 1'b1;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b100011;// 14
          func = 6'b100001;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b01; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b10; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b10; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b10; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
         state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd51:
        begin
          N = 3'b011;//  32
          Inv = 1'b0;//  28
          S = 2'b00;//  27
          IR_Ld = 1'b1;//  25
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
          opcode = 6'b100100;// 14
          func = 6'b100001;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b1;// 16
          RW = 1'b1;// 15
          opcode = 6'b100100;// 14
          func = 6'b100001;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
         state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd53:
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
          MOV = 1'b1;// 16
          RW = 1'b1;// 15
          opcode = 6'b100100;// 14
          func = 6'b100001;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          MDR_Ld = 1'b0;//  23
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b1;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b100100;// 14
          func = 6'b000000;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          MuxMAR_Ld = 1'b0;//  22
          RF_Ld = 1'b0;// 21
          MuxC_Ld = 1'b0;// 20
          PC_Ld = 1'b0;// 19
          nPC_Ld = 1'b0;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b1;// 16
          RW = 1'b0;// 15
          opcode = 6'b111111;// 14
          func = 6'b101000;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          MuxMDR_Ld = 1'b1;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b111111;// 14
          func = 6'b101000;
          MuxA_Ld = 2'b10;// 8
          MuxB_Ld = 2'b01; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          func = 6'b101000;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          func = 6'b101000;
          MuxA_Ld = 2'b11;// 8
          MuxB_Ld = 2'b10; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          func = 6'b101000;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          func = 6'b101000;
          MuxA_Ld = 2'b10;// 8
          MuxB_Ld = 2'b01; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          func = 6'b101000;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          func = 6'b101000;
          MuxA_Ld = 2'b11;// 8
          MuxB_Ld = 2'b10; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          
          MuxA_Ld = 2'b10;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          func = 6'b101000;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          func = 6'b101000;
          MuxA_Ld = 2'b10;// 8
          MuxB_Ld = 2'b01; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          func = 6'b101000;
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          func = 6'b101000;
          MuxA_Ld = 2'b11;// 8
          MuxB_Ld = 2'b10; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          CR = 8'b00000001; //4
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
          CR = 8'b00000001; //4
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
          CR = 8'b00000001; //4
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
          CR = 8'b00000001; //4
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
          CR = 8'b00000001; //4
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
          
          MuxA_Ld = 2'b00;// 8
          MuxB_Ld = 2'b00; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          
          MuxA_Ld = 2'b01;// 8
          MuxB_Ld = 2'b10; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
         state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd78:
        begin
          N = 3'b101;//  32
          Inv = 1'b1;//  28
          S = 2'b01;//  27
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
          func = 6'b010110;
          MuxA_Ld = 2'b01;// 8
          MuxB_Ld = 2'b01; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          func = 6'b010110;
          MuxA_Ld = 2'b01;// 8
          MuxB_Ld = 2'b01; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
      8'd80:
        begin
          N = 3'b101;//  32
          Inv = 1'b1;//  28
          S = 2'b01;//  27
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
          opcode = 6'b000000;// 14
          func = 6'b010110;
          MuxA_Ld = 2'b01;// 8
          MuxB_Ld = 2'b01; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          func = 6'b010110;
          MuxA_Ld = 2'b01;// 8
          MuxB_Ld = 2'b01; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd82:
        begin
          N = 3'b101;//  32
          Inv = 1'b1;//  28
          S = 2'b01;//  27
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
          func = 6'b010110;
          MuxA_Ld = 2'b01;// 8
          MuxB_Ld = 2'b01; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          nPC_Ld = 1'b1;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b100000;// 14
          func = 6'b010110;
          MuxA_Ld = 2'b01;// 8
          MuxB_Ld = 2'b01; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd84:
        begin
          N = 3'b101;//  32
          Inv = 1'b1;//  28
          S = 2'b01;//  27
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
          opcode = 6'b000001;// 14
          func = 6'b010110;
          MuxA_Ld = 2'b01;// 8
          MuxB_Ld = 2'b01; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          opcode = 6'b000001;// 14
          func = 6'b010110;
          MuxA_Ld = 2'b01;// 8
          MuxB_Ld = 2'b01; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd86:
        begin
          N = 3'b101;//  32
          Inv = 1'b1;//  28
          S = 2'b01;//  27
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
          func = 6'b010110;
          MuxA_Ld = 2'b01;// 8
          MuxB_Ld = 2'b01; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          nPC_Ld = 1'b1;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b111000;// 14
          func = 6'b010110;
          MuxA_Ld = 2'b01;// 8
          MuxB_Ld = 2'b01; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd88:
        begin
          N = 3'b101;//  32
          Inv = 1'b1;//  28
          S = 2'b01;//  27
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
          opcode = 6'b000110;// 14
          func = 6'b010110;
          MuxA_Ld = 2'b01;// 8
          MuxB_Ld = 2'b01; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          nPC_Ld = 1'b1;// 18
          MuxMDR_Ld = 1'b0;// 17
          MOV = 1'b0;// 16
          RW = 1'b0;// 15
          opcode = 6'b000110;// 14
          func = 6'b010110;
          MuxA_Ld = 2'b01;// 8
          MuxB_Ld = 2'b01; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
           8'd90:
        begin
          N = 3'b101;//  32
          Inv = 1'b1;//  28
          S = 2'b01;//  27
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
          func = 6'b010110;
          MuxA_Ld = 2'b01;// 8
          MuxB_Ld = 2'b01; // 6
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; //4
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
          MuxC_Ld = 1'b0;
          PC_Ld = 1'b1;
          nPC_Ld = 1'b1;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000001;
          func = 6'b010110;
          MuxA_Ld = 2'b01;
          MuxB_Ld = 2'b01; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		//BLTZAL
		    8'd92:
        begin
		
		//NOT YET ****************** Condition
		 N = 3'b101;//  32
          Inv = 1'b1;//  28
          S = 2'b01;//  27
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b0;
          MuxC_Ld = 1'b0;
          PC_Ld = 1'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000001;
          func = 6'b010110;
          MuxA_Ld = 2'b01;
          MuxB_Ld = 2'b01; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; 
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
          MuxC_Ld = 1'b0;
          PC_Ld = 1'b1;
          nPC_Ld = 1'b1;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000001;
          func = 6'b010110;
          MuxA_Ld = 2'b01;
          MuxB_Ld = 2'b01; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
		  CR = 8'b00000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		//BNE
		    8'd94:
        begin
		

	//NOT YET ****************** Condition
		  N = 3'b101;//  32
          Inv = 1'b1;//  28
          S = 2'b01;//  27
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b0;
          MuxC_Ld = 1'b0;
          PC_Ld = 1'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000101;
          func = 6'b010110;
          MuxA_Ld = 2'b01;
          MuxB_Ld = 2'b01; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; 
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
          MuxC_Ld = 1'b0;
          PC_Ld = 1'b1;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000101;
          func = 6'b010110;
          MuxA_Ld = 2'b01;
          MuxB_Ld = 2'b01; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; 
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
          MuxC_Ld =1'b0;
          PC_Ld = 1'b1;
          nPC_Ld = 1'b1;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000010;
          func = 6'b010110;
          MuxA_Ld = 2'b01;
          MuxB_Ld = 2'b01; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		//JAL
		    8'd97:
        begin
		  N = 3'b011;
          Inv = 1'b0;
          S = 2'b00;
          IR_Ld = 1'b0;
          MAR_Ld = 1'b0;
          MDR_Ld = 1'b0;
          MuxMAR_Ld = 1'b0;
          RF_Ld = 1'b0;
          MuxC_Ld = 1'b0;
          PC_Ld = 1'b0;
          nPC_Ld = 1'b0;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000011;
          func = 6'b010110;
          MuxA_Ld = 2'b01;
          MuxB_Ld = 2'b01; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; 
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
          MuxC_Ld =1'b0;
          PC_Ld = 1'b1;
          nPC_Ld = 1'b1;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000011;
          func = 6'b010110;
          MuxA_Ld = 2'b01;
          MuxB_Ld = 2'b01; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; 
          state = Y;

          data_out = {N,Inv,S,IR_Ld,MAR_Ld,MDR_Ld,MuxMAR_Ld,RF_Ld,MuxC_Ld,PC_Ld,nPC_Ld,MuxMDR_Ld,MOV,RW,opcode,func,MuxA_Ld,MuxB_Ld,Hi_Ld,Lo_Ld,MuxReg_Ld,CR,state};                                 
        end
		//JALR
		    8'd99:
        begin
		
		  N = 3'b011;
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
          func = 6'b010110;
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; 
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
          MuxC_Ld = 1'b0;
          PC_Ld = 1'b1;
          nPC_Ld = 1'b1;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000000;
          func = 6'b010110;
          MuxA_Ld = 2'b01;
          MuxB_Ld = 2'b01; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; 
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
          PC_Ld = 1'b1;
          nPC_Ld = 1'b1;
          MuxMDR_Ld = 1'b0;
          MOV = 1'b0;
          RW = 1'b0;
          opcode = 6'b000000;
          func = 6'b010110;
          MuxA_Ld = 2'b01;
          MuxB_Ld = 2'b01; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; 
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
          CR = 8'b00000001; 
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
          CR = 8'b00000001; 
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
          CR = 8'b00000001; 
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
          CR = 8'b00000001; 
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
          CR = 8'b00000001; 
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
          CR = 8'b00000001; 
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
          CR = 8'b00000001; 
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
          CR = 8'b00000001; 
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
          CR = 8'b00000001; 
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
          CR = 8'b00000001; 
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
          CR = 8'b00000001; 
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
          CR = 8'b00000001; 
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
          CR = 8'b00000001; 
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
          CR = 8'b00000001; 
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
          
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; 
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
          
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; 
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
          
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; 
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
          
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; 
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
          
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; 
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
          
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; 
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
          
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; 
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
          
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; 
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
          
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; 
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
          
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; 
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
          
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; 
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
          
          MuxA_Ld = 2'b10;
          MuxB_Ld = 2'b00; 
          Hi_Ld = 1'b0;
          Lo_Ld = 1'b0;
          MuxReg_Ld = 2'b00;
          CR = 8'b00000001; 
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
          CR = 8'b00000001; 
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
          CR = 8'b00000001; 
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
          CR = 8'b00000001; 
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
          CR = 8'b00000001; 
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
          CR = 8'b00000001; 
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
 output reg[7:0] CR,
 output reg[7:0] state, 
 input [52:0] Ds,
 input Clr, Clk);
  always @(posedge Clk)

      begin

        state=Ds[7:0];
        CR=Ds[15:8]; 
        MuxReg_Ld=Ds[18:16];
        Lo_Ld=Ds[18];
        Hi_Ld=Ds[19];
        MuxB_Ld=Ds[21:20]; 
        MuxA_Ld=Ds[23:22]; 
        func=Ds[29:24];
        opcode=Ds[35:30];
        RW=Ds[36]; 
        MOV=Ds[37]; 
        MuxMDR_Ld=Ds[38]; 
        nPC_Ld=Ds[39]; 
        PC_Ld=Ds[40]; 
        MuxC_Ld=Ds[41]; 
        RF_Ld=Ds[42]; 
        MuxMAR_Ld=Ds[43]; 
        MDR_Ld=Ds[44]; 
        MAR_Ld=Ds[45]; 
        IR_Ld=Ds[46]; 
        S=Ds[48:47]; 
        Inv=Ds[49]; 
        N=Ds[52:50];  

      end

endmodule

