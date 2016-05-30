`include "vJTAG_interface.sv"

parameter N = 8;
parameter NREG = 32;
parameter ADDRFIX = -9;
parameter EXCEPTIONADDR = 128;

`include "alu.sv"
`include "debug.sv"
`include "inst.v"
`include "cache.sv"

module processador (
					input logic 			RESET,
					input logic 			CLK,
					input logic 			IOException,
					input logic [N - 1: 0] 	RD,
					output logic [N - 1: 0] WD,
					output logic 			WE,
					output logic [N - 1: 0] A,
					interface debug
					);

   // ALU Data and Connections
   logic [N - 1: 0] 						SrcA, SrcB;
   logic [3:0] 								ALUControl;

   logic [N - 1: 0] 						ALUResult, ALUResultS;
   logic 									Zero;
   logic 									ALUSrc;
   logic 									ALUSrcA;

   alu #(N)(.*);

   always_comb begin
	  unique case (ALUSrcA)
		0: SrcA <= RD1;
		1: SrcA <= shamt;
	  endcase // unique case (ALUSrcA)

	  unique case (ALUSrc)
		0: SrcB <= RD2;
		1: SrcB <= imm;
	  endcase // unique case (ALUSrc)

	  unique case (ALUResult)
		0: unique case (ALUResultS)
					0: Zero <= 1;
					default: Zero <= 0;
				  endcase // case (ALUResultS)
		default: Zero <= 0;
	  endcase // unique case (ALUResult)
   end // always_comb

   // Instr Decoder
   // I: op rs rt not_used imm
   // R: op rs rt rd shamt funct
   // J: op not_used addr

   logic [31:0] Instr;
   logic [5:0] 	op;
   logic [4:0] 	rs;
   logic [4:0] 	rt;
   logic [N-1:0] imm;
   logic [4:0] 	 rd;
   logic [4:0] 	 shamt;
   logic [5:0] 	 funct;
   logic [N-1:0] addr;
   
   always_comb begin
	  // Both
	  op <= Instr[31:26];	  
	  rs <= Instr[25:21];
	  rt <= Instr[20:16];

	  // I
	  imm <= Instr[N-1:0];

	  // R
	  rd <= Instr[15:11];
	  shamt <= Instr[10:6];
	  funct <= Instr[5:0];

	  // J
	  addr <= Instr[N-1:0] + ADDRFIX;

   end // always_comb

   inst (
		 .clock(CLK),
		 .address(PC_),
		 .q(Instr)
		 );

   // Register File module
   // INPUT
   logic [N-1:0] regi[0:NREG-1];
   logic 		 WE3;
   logic [N-1:0] WD3;
   logic [4:0] 	 A1, A2, A3;

   // OUTPUT
   logic [N-1:0] RD1, RD2;


   always_comb begin
	  RD1 <= regi[A1];
	  RD2 <= regi[A2];
   end

   always_ff @(posedge CLK) begin
	  if (RESET) begin
		 for (int i = 0; i < NREG; i++)
		   regi[i] <= 16*i + i;
	  end else begin
		 if (WE3 && AE)
		   regi[A3] <= WD3;
	  end
   end

   // HILO Registers file module
   logic [N-1:0] HI, LO;

   logic 		 HIWE, LOWE;
   logic [N-1:0] HIWD, LOWD;

   always_ff @(posedge CLK) begin
	  if (RESET) begin
		 HI <= 0;
		 LO <= 0;
	  end else begin
		 if (HIWE) HI <= HIWD;
		 if (LOWE) LO <= LOWD;
	  end
   end

   logic [N-1:0] c0regi[0:NREG-1];
   logic 		 c0WE;
   logic [N-1:0] c0WD;
   logic [N-1:0] c0A;

   logic 		 Exception;
   logic [N-1:0] EPCWD;

   logic [N-1:0] c0RD;


   always_comb begin
	  c0RD <= c0regi[c0A];
   end

   always_ff @(posedge CLK) begin
	  if (RESET) begin
		 for (int i = 0; i < NREG; i++)
		   c0regi[i] <= 0;
	  end else begin
		 if (Exception) begin
			c0regi[14] <= EPCWD;

			if (c0WE && c0A != 14)
			  c0regi[c0A] <= c0WD;
		 end else if (c0WE)
		   c0regi[c0A] <= c0WD;
	  end // else: !if(RESET)
   end // always_ff @

   //  Register file connections
   logic RegWrite;

   logic [4:0] WriteReg;
   logic [N-1:0] Result;

   logic 		 RegDst;
   logic 		 MemtoReg;

   logic [1:0] 	 ResultSrc;

   logic 		 HILOSrc;


   always_comb begin
	  A1 <= rs;
	  A2 <= rt;
	  A3 <= WriteReg;
	  WD3 <= Result;
	  WE3 <= RegWrite;

	  unique case (Link)
		0: begin
		   unique case (RegDst)
			 0: WriteReg <= rt;
			 1: WriteReg <= rd;
		   endcase // unique case (RegDst)
		   unique case (ResultSrc)
			 0: unique case (MemtoReg)
						 0: Result <= ALUResult;
						 1: Result <= CacheRD;
					   endcase // case (MemtoReg)
			 1: unique case (HILOSrc)
						 0: Result <= HI;
						 1: Result <= LO;
					   endcase
			 2: Result <= c0RD;
			 3: Result <= 0;
		   endcase // unique case (ResultSrc)
		end // case: 0
		1: begin
		   WriteReg <= (NREG-1);
		   Result <= PCPlus;
		end
	  endcase // unique case (Link)

	  // HILO File
	  unique case (HILOSrc)
		0: begin
		   HIWD <= RD1;
		   LOWD <= RD1;
		end
		1: begin
		   HIWD <= ALUResultS;
		   LOWD <= ALUResult;
		end
	  endcase // unique case (HILOSrc)

	  c0A <= rd;
	  c0WD <= RD2;
   end // always_comb

   // Data Memory and cache connections
   logic MemWrite;
   logic MemRead;
   logic [N-1:0] CacheRD;

   always_comb begin
	  A <= ALUResult;
	  WE <= MemWrite;
	  WD <= RD2;
   end
   cache  #(N, 8, 5)(
					 .clk(CLK),
					 .reset(RESET),
					 .pAddr(ALUResult),
					 .pDataOut(RD2),
					 .pDatain(CacheRD),
					 .readRequest(MemRead),
					 .writeRequest(MemWrite),
					 
					 .busyClock(BusyClock),
					 
					 .mAddr(A),
					 .mDataIn(RD)
					  );
   
   // PC, Branch and Jump
   logic [N-1:0] PC, PC_, PC__;
   logic [N-1:0] PCPlus;

   logic 		 PCSrc;

   logic 		 Branch;
   logic 		 BranchCondition;
   logic [N-1:0] PCBranch;
   logic 		 Link;

   logic 		 PAUSE;
   logic 		 BusyClock;

   always_comb begin
	  PCPlus <= PC + 1;
	  PCSrc <= Branch & BranchCondition;

	  unique case (RESET)
		0: unique case (PCSrc)
					0: PC__ <= PCPlus;
					1: PC__ <= PCBranch;
				  endcase // case (PCSrc)
		1: PC__ <= 0;
	  endcase // unique case (RESET)

	  PAUSE <= BusyClock & ~RESET;
	  unique case (RESET)
		0: unique case (PAUSE)
					0: unique case (Exception)
								0: PC_ <= PC__;
								1: PC_ <= EXCEPTIONADDR;
							  endcase // case (Exception)
					1: PC_ <= PC;
				  endcase // case (PAUSE)
		1: PC_ <= 0;
	  endcase // unique case (RESET)
   end // always_comb

   always_ff @(posedge CLK) begin
	  PC <= PC_;
   end

   // Exception Handler
   always_comb begin
	  Exception <= IOException;
	  EPCWD <= PC__;
   end

   // Opcode main decoder
   always_comb begin
	  MemRead <= op[5:3] == 3'b100; // lw

	  unique case (op)
		6'b010000: unique case(rs)
							4: c0WE <= 1; // mtc0
							default: c0WE <= 0;
						  endcase // case (rs)
		default: c0WE <= 0;
	  endcase // unique case (op)

	  // ResultSRC
	  unique case (op)
		6'b000000: unique case (funct)
							6'b010000: ResultSrc <= 1; // mfhi
							6'b010010: ResultSrc <= 1; // mfla
							default: ResultSrc <= 0;
						  endcase // case (funct)
		6'b010000: unique case (rs)
							0: ResultSrc <= 2; // mfc0
							default: ResultSrc <= 0;
						  endcase // case (rs)
		default: ResultSrc <= 0;		
	  endcase // unique case (op)

	  // HIWE, LOWE, HILOSrc
	  unique case (op)
		6'b000000: unique case(funct)
							6'b010001: begin
							   HIWE <= 1;
							   LOWE <= 0;
							   HILOSrc <= 0;
							end
							6'b010011: begin
							   HIWE <= 0;
							   LOWE <= 1;
							   HILOSrc <= 0;
							end
							default: begin
							   HIWE <= (funct >= 24 && funct <= 27);
							   LOWE <= HIWE;
							   HILOSrc <= HIWE | (funct == 18);
							end
						  endcase // case (funct)
		default: begin
		   HIWE <= 0;
		   LOWE <= 0;		   
		   HILOSrc <= 0;
		end
	  endcase // unique case (op)

	  // RegDst, ALUSrc, MemtoReg
	  unique case (op)
		6'b000000: begin // R
		   RegDst <= 1;
		   ALUSrc <= 0;
		   MemtoReg <= 0;
		end
		default: begin // I
		   RegDst <= 0;
		   ALUSrc <= ~Branch;
		   MemtoReg <= (op[5:3] == 3'b100);
		end
	  endcase // unique case (op)

	  unique case (op)
		6'b000000: unique case (funct)
							6'b000000: ALUSrcA <= 1;
							6'b000010: ALUSrcA <= 1;
							6'b000011: ALUSrcA <= 1;
							default: ALUSrcA <= 0;
						  endcase // case (funct)
		default: ALUSrcA <= 0;
	  endcase // unique case (op)

	  // MemWrite, RegWrite
	  unique case(op[5:3])
		3'b101: begin
		   MemWrite <= 1;
		   RegWrite <= 0;
		end
		default: begin
		   MemWrite <= 0;
		   unique case (op)
			 6'b000000: unique  case(funct)
								  6'b010001: RegWrite <= 0; //mthi
								  6'b010011: RegWrite <= 0; //mtlo
								  6'b011000: RegWrite <= 0; //mult
								  6'b011001: RegWrite <= 0; //multu
								  6'b011010: RegWrite <= 0; //div
								  6'b011011: RegWrite <= 0; //divu
								  default: RegWrite <= ~Branch | Link;
								endcase // case (funct)
			 6'b010000: RegWrite <= (rs == 0); // mtc0
			 default: RegWrite <= ~Branch | Link;
		   endcase // unique case (op)
		end // case: default
	  endcase // unique case (op[5:3])

	  // PCBranch
	  unique case(op[2:0])
		3'b000: PCBranch <= RD1; // jr, jalr
		3'b010: PCBranch <= addr; // j: {PC[31: 28], addr, 2'b00}
		3'b011: PCBranc <= addr; // jal
		default: PCBranch <= PC+imm;
	  endcase // unique case (op[2:0])

	  // Branch
	  unique case(op)
		6'b000000: Branch <= (funct[5:1] == 5'b00100); //jr, jalr
		default: Branch <= (op[5:3] == 3'b000); // bltz, bltza, bgezal, bgez, j, jal, beq, bne, ble
	  endcase // unique case (op)

	  // Link
	  unique case (op)
		6'b000000: Link <= Branch & funct[0]; // 0 jr, 1 jalr
		6'b000001: Link <= rt[4]; // 0 bltz/bgez, 1 bltzal/bgezal
		6'b000011: Link <= 1; //jal
		default: Link <= 0;
	  endcase // unique case (op)

	  // BranchCondition
	  unique case(op[2:0])
		3'b001: unique case (rt[0])
						 0: BranchCondition <= RD1[N-1]; // bltz, bltzal
						 1: BranchCondition <= ~RD1[N-1];
					   endcase // case (rt[0])
		3'b100: BranchCondition <= Zero; // beq
		3'b101: BranchCondition <= ~Zero; // bne
		3'b110: BranchCondition <= Zero | RD1[N-1]; // blez
		3'b111: BranchCondition <= ~Zero & ~RD1[N-1]; // bgtz
		default: BranchCondition <= Branch; //jr,jalr, j, jal
	  endcase // unique case (op[2:0])

	  // ALUControl
	  unique case (op)
		6'b000000: unique case (funct)
							6'b000000: ALUControl <= 4'b1001; // sll
							6'b000010: ALUControl <= 4'b1010; // srl
							6'b000011: ALUControl <= 4'b1011; // sra
							6'b000100: ALUControl <= 4'b1001; // sllv
							6'b000110: ALUControl <= 4'b1010; // srlv
							6'b000111: ALUControl <= 4'b1011; // srav

							6'b011000: ALUControl <= 4'b1100; //mult
							6'b011001: ALUControl <= 4'b1101; //multu
							6'b011010: ALUControl <= 4'b1110; //div
							6'b011011: ALUControl <= 4'b1111; //divu

							6'b100010: ALUControl <= 4'b0110; //sub
							6'b100011: ALUControl <= 4'b0110; //subu
							6'b100100: ALUControl <= 4'b0000; //and
							6'b100101: ALUControl <= 4'b0001; //or
							6'b100110: ALUControl <= 4'b0100; //xor
							6'b100111: ALUControl <= 4'b0101; //nor
							6'b101010: ALUControl <= 4'b0011; //slt
							6'b101011: ALUControl <= 4'b0111; //sltu
							default: ALUControl <= 4'b0010;
						  endcase // case (funct)
		6'b000100: ALUControl <= 4'b0110; //beq
		6'b000101: ALUControl <= 4'b0110; //bne
		6'b001010: ALUControl <= 4'b0011; //slti
		6'b001011: ALUControl <= 4'b0111; //sltiu
		6'b001100: ALUControl <= 4'b0000; //andi
		6'b001101: ALUControl <= 4'b0001; //ori
		6'b001110: ALUControl <= 4'b0100; //xori
		6'b001111: ALUControl <= 4'b1000; //lui
		default: ALUControl <= 4'b0010;
	  endcase // unique case (op)
   end // always_comb

endmodule // processador

		
		
		
		
		
		
		
		
							
							
							
							
							
							
							
							
							
							 
							
							
 		
		
		
		
						 
		
		 
 		
		
		 
		
			 
								  
								  

	  
   
   
   
   
   

		   
	  
		 
			
		 
   

   
   
	  
   
					
					
   
   
