`include "processador.sv"
`include "memo.v"

module cpu (
			input logic REST,
			input logic CLK,
			input logic [N-1:0] E,
			output logic [N-1:0] S
			);
   processador(.*);
   logic [N-1:0] 				 RD, WD, MRD;
   logic 						 WE;
   logic [N-1:0] 				 A;

   logic [N-1:0] 				 lastE;
   logic 						 IOException;

   logic 						 EN;

   always_comb begin
	  unique case(A)
		255: begin
		   EN <= WE;
		   RD <= E;
		end
		default: begin
		   EN <= 0;
		   RD <= MRD;
		end
	  endcase // unique case (A)

	  IOEXception <= lastE != E;
   end // always_comb

   always_ff @ (posedge CLK) begin
	  if (RESET) begin
		 S <= 0;
		 lastE <= E;
	  end else begin
		 if (EN)
		   S <= WD;
		 if (lastE != E)
		   lastE <= E;
	  end
   end // always_ff @

   memo (
		 .address(A),
		 .clock(CLK),
		 .data(WD),
		 .wren(WE),
		 .q(MRD)
		 );
endmodule
		 

