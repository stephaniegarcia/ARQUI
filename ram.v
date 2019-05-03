
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