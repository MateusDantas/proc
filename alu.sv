module alu #(parameter N)(
						  input logic [N-1:0] SrcA, SrcB,
						  input logic [3:0] ALUControl,
						  output logic [N-1:0] ALUResult, ALUResultS
						  );
   logic [(N<<1)-1:0] 						   Result, ExtSrcA, ExtSrcB, SigSrcA, SigSrcB;
   
   always_comb begin
	  ExtSrcA <= {8'b0, SrcA};
	  ExtSrcB <= {8'b0, SrcB};

	  SigSrcA <= {{8{SrcA[N-1]}}, SrcA};
	  SigSrcB <= {{8{SrcB[N-1]}}, SrcB};

	  unique case (ALUControl)
		4'b1100: Result <= SigSrcA * SigSrcB;
		4'b1101: Result <= ExtSrcA * ExtSrcB;
		4'b1110: Result <= {$signed(SrcA) % $signed(SrcB), $signed(SrcA) / $signed(SrcB)};
		4'b1111: Result <= {SrcA % SrcB, SrcA / SrcB};
	  endcase // unique case (ALUControl)

	  unique case (ALUControl[3:2])
		2'b11: ALUResultS <= Result[(N<<1)-1:N];
		default: ALUResultS <= 0;
	  endcase // unique case (ALUControl[3:2])

	  unique case(ALUControl)
		4'b0000: ALUResult <= SrcA & SrcB;
		4'b0001: ALUResult <= SrcA | SrcB;
		4'b0010: ALUResult <= SrcA + SrcB;
		4'b0011: ALUResult <= $signed(SrcA) < $signed(SrcB);
		4'b0100: ALUResult <= SrcA ^ SrcB;
		4'b0101: ALUResult <= ~(SrcA | SrcB);
		4'b0110: ALUResult <= SrcA - SrcB;
		4'b0111: ALUResult <= SrcA < SrcB;
		4'b1000: ALUResult <= 0;
		4'b1001: ALUResult <= SrcB << SrcA[4:0];
		4'b1010: ALUResult <= SrcB >> SrcA[4:0];
		4'b1011: ALUResult <= $signed(SrcB) >> SrcA[4:0];
		default: ALUResult <= Result[N-1:0];
	  endcase // unique case (ALUControl)
   end // always_comb
endmodule // alu

		
		
		
	  
   
