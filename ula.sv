module ula (
			input logic [7:0] ina,
			input logic [7:0] inb,
			input logic [15:0] incon,
			output logic [7:0] result
			);
   always_comb begin
	  unique case (incon)
		3'b000: result <= ina & inb;
		3'b001: result <= ina | inb;
		3'b010: result <= ina + inb;
		3'b100: result <= ina & (~inb);
		3'b101: result <= ina | (~inb);
		3'b110: result <= ina - inb;
		3'b111: result <= ina > inb;
	  endcase // unique case (incon)
   end // always_comb
endmodule // ula

		
