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
// repeat(200) begin
// #10
		// $display("Y= A + B = %d + %d",  cpu.ALU.A, cpu.ALU.B);
	// $monitor("\nReg0 %d  reg1 %d reg2 %d reg3 %d reg4 %d reg5 %d\n", cpu.rg.r0,cpu.rg.r1,cpu.rg.r2,cpu.rg.r3,cpu.rg.r4,cpu.rg.r5);

	// end
	end

initial begin
// $monitor("State: %d mar_out %d pc_out %d npc_out %d  IR  %b ", cpu.Y, cpu.MAR.Q, cpu.PC.Q, cpu.nPC.Q, cpu.IR.Q);
  $monitor("State: %d mar_out %d pc_out %d npc_out %d  IR  %b  \n\nReg0 %d  reg1 %d reg2 %d reg3 %d reg4 %d reg5 %d\n \n\nReg6 %d  reg7 %d reg8 %d reg9 %d reg10 %d  PA %d PB %d PC %d SA %d SB %d SC %d\n \n \n \nY= A + B = %d + %d\n ALUout %d   in_SA %d", cpu.st, cpu.MAR_out, cpu.PC_out, cpu.nPC_out, cpu.IR_out, cpu.rg.Q0,cpu.rg.Q1,cpu.rg.Q2,cpu.rg.Q3,cpu.rg.Q4,cpu.rg.Q5, cpu.rg.Q6,cpu.rg.Q7,cpu.rg.Q8,cpu.rg.Q9,cpu.rg.Q10, cpu.PA, cpu.PB, cpu.Mux_reg_out, cpu.IR_out[25:21], cpu.IR_out[20:16], cpu.Mux_c_out, cpu.ALU.A, cpu.ALU.B, cpu.ALU.Y, cpu.rg.in_SA);
    //250 for 2
    //60 for 1
	while(count < 250) begin
      $display("Mem[ %d ]= %b   ", count, cpu.RAM.Mem[count]);
			count = count + 9'd1;

	end
end


initial begin
Clk =1'b0;
forever #5 Clk = ~Clk;
end

	 // sim time
parameter sim_time = 1000;
//1700 for test 1
//6000 for test2
initial #sim_time $finish;

endmodule
