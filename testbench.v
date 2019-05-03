module datapath_test;

	integer fi, fo, code, i; reg [31:0] data; // variables needed for the file reading and precharging
	reg Enable, ReadWrite; reg [31:0] DataIn;
	reg [7:0] Address; wire [31:0] DataOut; output reg [5:0] OP; wire MOC;


	ram512x8 ram1 (DataOut, MOC, ReadWrite, Enable, Address, DataIn, OP);


	initial begin
		fi = $fopen("PF1_Reyes_Alvarado_Alejandro_ramdata.txt","r");
		ReadWrite = 1'b0;
		OP = 6'b000100;
		Address =  9'b000000000;
		while (!$feof(fi)) begin
		code = $fscanf(fi, "%d", data);
		ram1.Mem[Address] = data;
		Address = Address + 1;

		end
		$fclose(fi);
	end
	initial begin
		$display("        Address    Value                 Time   E  MOC  R/W");
		OP = 6'b001000;
		Enable = 1'b0; ReadWrite = 1'b1;
		Address =  9'b000000000;
		repeat (4) begin
		#5 Enable = 1'b1;
		#5 Enable = 1'b0;
		begin
			case(OP)
				6'b001000:begin
					Address = Address + 4;
				end
				6'b000010:begin
					Address = Address + 2;
				end
				6'b000001:begin
					Address = Address + 1;
				end
			endcase
		end
		end


	end
	always @ (posedge Enable)
	begin
	#1;
	$display(" data en   %h = %h    %b   %b   %b ", Address, DataOut, Enable, MOC, ReadWrite);
	end


initial fork

     #0 reset <= 1;


  join

  initial begin

    $display("\n\n|========================| TESTING... |========================|\n\n");
    $display("MAR       PC            nPC       IR");
    $monitor("%d        %d             %d       %b", datapath.MAR.MAR_out, datapath.PC.PC_out, datapath.NPC.nPC_out, datapath.IR.IR_out);
//

//iniciar el clock
//
      begin
        CLK = 1'b0;
        forever #10 CLK = ~CLK;
      end
end
	
endmodule
