module cache #(parameter DWS=16, // data word siz
			   MAS=10, // memory addr size
			   CLAS=5) // cache line adr size
   (input logic clk,
	input logic 		   reset,
	input logic [MAS-1:0]  pAddr,
	input logic [DWS-1:0]  pDataOut,
	output logic [DWS-1:0] pDataIn,
	input 				   readRequest,
	input 				   writeRequest,
	output logic 		   busyClock,
	output logic [MAS-1:0] mAddr,
	input logic [DWS-1:0]  mDataIn);

   logic [DWS-1:0] 		   data [(1 << CLAS) - 1:0];
   logic [MAS-CLAS-1:0]    tag [(1 << CLAS) - 1:0];
   logic 				   valid [(1 << CLAS) - 1:0];

   logic 				   hit, miss;

   logic [MAS-CLAS-1:0]    pTag;
   logic [MAS-CLAS-1:0]    mTag;

   logic [CLAS-1:0] 	   pLine;
   logic [CLAS-1:0] 	   mLine;

   always_comb begin
	  {pTag, pLine} = pAddr;
	  {mTag, mLine} = mAddr;
   end

   always_comb begin
	  pDataIn <= data[pLine];
	  hit <= valid[pLine] == 1 && tag[pLine] == pTag;
	  miss <= !hit && readRequest;

	  busyClock <= miss;
   end

   enum logic [2:0] {clean, wwait} state;
   always_ff @(posedge clk)
	 if (reset) begin
		for (int i = 0; i < (1<<CLAS); i++)
		  valid[i] <= 0;
		state <= clean;		
	 end
	 else case (state)
			clean:
			  if (miss) begin
				 valid[mLine] <= 1;
				 tag[mLine] <= mTag;
				 state <= wwait;
			  end else if (writeRequest) begin
				 data[pLine] <= pDataOut;
				 tag[pLine] <= pTag;
				 valid[pLine] <= 1;
			  end
			wwait: begin
			   data[mLine] <= mDataIn;
			   state <= clean;
			end
		  endcase
endmodule
		   
	  
	  
   end
   
