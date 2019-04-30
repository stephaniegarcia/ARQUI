 module ram512x8(output reg [31:0] DataOut, output reg MOC, input ReadWrite, Enable,input[31:0] Address,
				 input[31:0] DataIn,
				 input [5:0] OP //gives word type (byte, halfword, word)
				 );
	reg[7:0] Mem[0:511]; //512 bytes of memory total
	always @(posedge Enable, ReadWrite)
		begin
			if(Enable)
			begin
				MOC = 0;
				if(ReadWrite) begin //Read
					case(OP)
					//Word (32bits)
						6'b001000:begin
							DataOut[31:24] <= Mem[Address];
							DataOut[23:16] <= Mem[Address+1];
							DataOut[15:8]  <= Mem[Address+2];
							DataOut[7:0]   <= Mem[Address+3];
							end
					//Half-Word (16 bits)
						6'b000010:begin
							DataOut[31:16]  <= 16'h0000; //16 bits zeroes
							DataOut[15:8]  <= Mem[Address];
							DataOut[7:0]   <= Mem[Address+1];
							end
					//Byte (8 bits)
						6'b000001:begin
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
						6'b000100:begin
							Mem[Address] <= DataIn[31:24];
							Mem[Address+1] <= DataIn[23:16];
							Mem[Address+2] <= DataIn[15:8];
							Mem[Address+3] <= DataIn[7:0];
						end

					//Half-Word	(16bits)
						6'b000110:begin
							Mem[Address] <= DataIn[15:8];
							Mem[Address+1] <= DataIn[7:0];
						end
					//Byte (8 bits)
						6'b000101:begin
							Mem[Address] = DataIn[7:0];
						end
					endcase
					MOC = 1;
				end
			end
		end
endmodule
