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
  $monitor("State: %d mar_out %d pc_out %d npc_out %d  IR  %b  \n\nReg0 %d  reg1 %d reg2 %d reg3 %d reg4 %d reg5 %d\n \n\nReg6 %d  reg7 %d reg8 %d reg9 %d reg10 %d reg11 %d\n \n\nReg12 %d  reg13 %d reg14 %d reg15 %d reg16 %d reg17 %d\n \n\nReg18 %d  reg19 %d reg20 %d reg21 %d reg22 %d reg23 %d\n \n\nReg24 %d  reg25 %d reg26 %d reg27 %d reg28 %d reg29 %d\n \n\nReg30 %d  reg31 %d \n", cpu.st, cpu.MAR_out, cpu.PC_out, cpu.nPC_out, cpu.IR_out, cpu.rg.Q0,cpu.rg.Q1,cpu.rg.Q2,cpu.rg.Q3,cpu.rg.Q4,cpu.rg.Q5, cpu.rg.Q6,cpu.rg.Q7,cpu.rg.Q8,cpu.rg.Q9,cpu.rg.Q10,cpu.rg.Q11, cpu.rg.Q12,cpu.rg.Q13,cpu.rg.Q14,cpu.rg.Q15,cpu.rg.Q16,cpu.rg.Q17, cpu.rg.Q18,cpu.rg.Q19,cpu.rg.Q20,cpu.rg.Q21,cpu.rg.Q22,cpu.rg.Q23, cpu.rg.Q24,cpu.rg.Q25,cpu.rg.Q26,cpu.rg.Q27,cpu.rg.Q28,cpu.rg.Q29, cpu.rg.Q30,cpu.rg.Q31);
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
parameter sim_time = 2020;
//1700 for test 1
//6000 for test2
initial #sim_time $finish;
	
endmodule
