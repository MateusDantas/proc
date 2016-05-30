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
   logic [6:0] 	op;
   logic [4:0] 	rs;
   logic [4:0] 	rt;
   logic [N-1:0] imm;
   logic [4:0] 	 rd;
   logic [4:0] 	 shamt;
   logic [2:0] 	 funct;
   logic [6:0] 	 funct2;
   logic [N-1:0] imm2;
   logic [N-1:0] addr;
   
   always_comb begin
	  // All
	  op <= Instr[6:0];
	  // R/S/SB
	  rs <= Instr[19:15]; //may swap
	  rt <= Instr[24:20];

	  // I
	  imm <= Instr[(N + 20) - 1:20];

	  // U/UJ
	  imm2 <= Instr[(N + 12) - 1:12];
	  
	  // R/I/S/SB
	  rd <= Instr[11:7];
	  funct <= Instr[14:12];

	  // R/S/SB
	  funct2 <= Instr[31:25];
	  
	  // J
	  addr <= Instr[(N + 12) - 1:12] + ADDRFIX;

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
		   WriteReg <= rd;
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
	  MemRead <= op[6:0] == 7'b0000011; // lw

	  unique case (op)
/*/		6'b010000: unique case(rs)
							4: c0WE <= 1; // mtc0
							default: c0WE <= 0;
						  endcase // case (rs)*/
		default: c0WE <= 0;
	  endcase // unique case (op)

	  // ResultSRC
	  unique case (op)
/*		6'b000000: unique case (funct)
							6'b010000: ResultSrc <= 1; // mfhi
							6'b010010: ResultSrc <= 1; // mfla
							default: ResultSrc <= 0;
						  endcase // case (funct)
		6'b010000: unique case (rs)
							0: ResultSrc <= 2; // mfc0
							default: ResultSrc <= 0;
						  endcase // case (rs)*/
		default: ResultSrc <= 0;		
	  endcase // unique case (op)

	  // HIWE, LOWE, HILOSrc
	  unique case (op)
		7'0110011: unique case(funct)
				/*/			6'b010001: begin
							   HIWE <= 1;
							   LOWE <= 0;
							   HILOSrc <= 0;
							end
							6'b010011: begin
							   HIWE <= 0;
							   LOWE <= 1;
							   HILOSrc <= 0;
							end*/
							default: begin
							   HIWE <= (funct >= 0 && funct <= 5 && funct2 == 7'b0000001); //mult,multu,div,divu
							   LOWE <= HIWE;
							   HILOSrc <= HIWE; //removing mflo
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
		7'b0110011: begin // R
		   RegDst <= 1;
		   ALUSrc <= 0;
		   MemtoReg <= 0;
		end
		default: begin // I
		   RegDst <= 0;
		   ALUSrc <= ~Branch;
		   MemtoReg <= (op[6:0] == 7'b0000011); //all 0 except for lw
		end
	  endcase // unique case (op)

	  unique case (op)
		7'b0110011: unique case (funct)
							 3'b001: unique case (funct2)
											  7'b0000000: ALUSrcA <= 1; //sll
											  default: ALUSrcA <= 0;
											endcase
							 3'b101: unique case (funct2)
											  7'b0000001: ALUSrcA <= 0; //divu
											  default: ALUSrcA <= 1; //srl/sra
											endcase
							 default: ALUSrcA <= 0;
						   endcase // case (funct)
		default: ALUSrcA <= 0;
	  endcase // unique case (op)
	  
	  // MemWrite, RegWrite
	  unique case(op[6:0])
		7'b0100011: begin
		   MemWrite <= 1;
		   RegWrite <= 0;
		end
		default: begin
		   MemWrite <= 0;
		   unique case (op)
			 7'b0110011: unique  case(funct2)
					//			  6'b010001: RegWrite <= 0; //mthi
								   //			  6'b010011: RegWrite <= 0; //mtlo
								   7'b0000001: RegWrite <= 0; //mult,multu,div,divu							
//								   3'b000: RegWrite <= 0; //mult
//								   3'b001: RegWrite <= 0; //multu
//								   3'b011010: RegWrite <= 0; //div
//								   3'b011011: RegWrite <= 0; //divu
								  default: RegWrite <= ~Branch | Link;
								endcase // case (funct)
//			 6'b010000: RegWrite <= (rs == 0); // mtc0
			 default: RegWrite <= ~Branch | Link;
		   endcase // unique case (op)
		end // case: default
	  endcase // unique case (op[5:3])

	  // PCBranch
	  unique case(op)
		7'b1100111: PCBranch <= (RD1 + imm) & ~1; // jr, jalr
		7'b1101111: PCBranch <= addr; // jal
		default: PCBranch <= PC+imm;
	  endcase // unique case (op[2:0])

	  // Branch
	  unique case(op)
		7'b1100111: Branch <= 1; //jr, jalr
		7'b1100011: Branch <= 1; //bltz, bltza, beq, bne, ble
		7'b1101111: Branch <= 1; //jal		
		default: Branch <= 0;		
	  endcase // unique case (op)

	  // Link
	  unique case (op)
		7'b1100111: Link <= Branch; // 0 jr, 1 jalr
		7'b1100011: Link <= (funct == 3'b110 | funct == 3'b111); // 0 bltz/bgez, 1 bltzal/bgezal
		7'b1101111: Link <= 1; //jal
		default: Link <= 0;
	  endcase // unique case (op)

	  // BranchCondition
	  unique case(op)
		7'b1100011: unique case (funct)
							 3'b100: BranchCondition <= RD1[N-1]; // bltz, bltzal
							 3'b110: BranchCondition <= RD1[N-1];						
							 3'b000: BranchCondition <= Zero; // beq
							 3'b001: BranchCondition <= ~Zero; // bne
							 1: BranchCondition <= ~RD1[N-1];
					   endcase // case (rt[0])
/*		3'b100: BranchCondition <= Zero; // beq
		3'b101: BranchCondition <= ~Zero; // bne
		3'b110: BranchCondition <= Zero | RD1[N-1]; // blez
		3'b111: BranchCondition <= ~Zero & ~RD1[N-1]; // bgtz*/
		default: BranchCondition <= Branch; //jr,jalr, j, jal
	  endcase // unique case (op[2:0])

	  // ALUControl
	  unique case (op)
		7'b0110011: unique case (funct2)
							 7'b0000001: unique case (funct)
												  3'b000: ALUControl <= 4'b1100; //mult
												  3'b001: ALUControl <= 4'b1101; //multu
												  3'b100: ALUControl <= 4'b1110; //div
												  3'b101: ALUControl <= 4'b1111; //divu
												endcase // case (funct)
							 7'b0000000: unique case (funct)
												  3'b001: ALUControl <= 4'b1001; // sll
												  3'b101: ALUControl <= 4'b1010; // srl
												  3'b111: ALUControl <= 4'b0000; // and
												  3'b110: ALUControl <= 4'b0001; // or
												  3'b100: ALUControl <= 4'b0100; // xor
												  3'b010: ALUControl <= 4'b0011; // slt
												  3'b011: ALUControl <= 4'b0111; // sltu
												  default: ALUControl <= 4'b0010; 					  
												endcase // case (funct)
							 7'b0100000: unique case (funct)
												  3'b000: ALUControl <= 4'b0110; //sub
												  3'b101: ALUControl <= 4'b1011; // sra
												  default: ALUControl <= 4'b0010;							
												endcase
							 default: ALUControl <= 4'b0010;
						  endcase // case (funct)
		7'b1100011: unique case (funct)
							 3'b000: ALUControl <= 4'b0110; //beq
							 3'b001: ALUControl <= 4'b0110; //bne
							 default: ALUControl <= 4'b0010;
						   endcase
		7'b0010011: unique case (funct)
							 3'b010: ALUControl <= 4'b0011; //slti
							 3'b011: ALUControl <= 4'b0111; //sltiu
							 3'b111: ALUControl <= 4'b0000; //andi
							 3'b110: ALUControl <= 4'b0001; //ori
							 3'b100: ALUControl <= 4'b0100; //xori
							 default: ALUControl <= 4'b0010;
						   endcase
		7'b0110111: ALUControl <= 4'b1000; //lui
		default: ALUControl <= 4'b0010;
	  endcase // unique case (op)
   end // always_comb

endmodule // processador

		
		
		
		
		
		
		
		
							
							
							
							
							
							
							
							
							
							 
							
							
 		
		
		
		
						 
		
		 
 		
		
		 
		
			 
								  
								  

	  
   
   
   
   
   

		   
	  
		 
			
		 
   

   
   
	  
   
					
					
   
   
