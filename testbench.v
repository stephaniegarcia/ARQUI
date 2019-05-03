module datapath_tb;
	reg Clk = 0;
	reg Clear;
	integer code, count;
	reg [7:0] data;
initial begin
Clear = 1'b1;
#11 Clear = 1'b0;
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
  $monitor("State: %d mar_out %d pc_out %d npc_out %d  IR  %b  ", cpu.st, cpu.MAR_out, cpu.PC_out, cpu.nPC_out, cpu.IR_out);
    //250 for 2
    //60 for 1
	while(count < 60) begin 
      $display("Mem[ %d ]= %b   ", count, cpu.RAM.Mem[count]);
			count = count + 9'd1;

	end
end


initial begin
Clk =1'b0;
forever #5 Clk = ~Clk;
end
	
	 // sim time
parameter sim_time = 1700;
//1700 for test 1
//6000 for test2
initial #sim_time $finish;
	
endmodule
