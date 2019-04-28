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