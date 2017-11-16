
`timescale 1 ns / 1 ps

module			coinc_checker	(
	input			reset		,	//Reset, active high
	input			entry_clock	,	//Entry clock
	input			req_coinc	,	//1: Require coincidence (activates this module)
	input			other_valid	,	//Another channel currently has a valid sequence
	//input			coinc_state	,
	//input	reg		coinc_state	,
	output	reg		coinc_met	)	//The result of this module's check
	;
	
	//Declaration of state registers for the coincidence-checking state machine
	reg	[1:0]	coinc_state; /* synthesis syn_encoding = "sequential" */
	
	//State definitions for the coincidence-checking state machine
	parameter COINC_IDLE	= 2'b00; //Currently no need to check for coincidence
	parameter COINC_TBD		= 2'b01; //To Be Determined: coincidence checking is underway
	parameter COINC_DONE	= 2'b10; //Coincidence has been observed so check is complete
	
	always @(posedge entry_clock or posedge reset) begin
		if (reset) begin
			coinc_met <= #1 1'b0;
			coinc_state <= #1 COINC_IDLE; //Permanent state until reset
		end
		else begin
			case (coinc_state)
				COINC_IDLE : begin
					coinc_met <= #1 1'b0;
					if (req_coinc)	coinc_state <= #1 COINC_TBD;
					else			coinc_state <= #1 COINC_IDLE;
				end
				COINC_TBD : begin
					coinc_met <= #1 other_valid;
					if (other_valid)	coinc_state <= #1 COINC_DONE;
					else				coinc_state <= #1 COINC_TBD;
				end
				COINC_DONE : begin
					coinc_met <= #1 1'b1;
					coinc_state <= #1 COINC_DONE; //Permanent state until reset
				end
				default : begin //Here in case of error: returns state machine to IDLE
					coinc_met <= #1 1'b0;
					coinc_state <= #1 COINC_IDLE;
				end
			endcase
		end
	end
	
endmodule


// ******************************* NO LONGER USED OR NO LONGER APPLICABLE: *******************************

	//Declaration of state registers for the coincidence-checking state machine
	// reg	[1:0]	coinc_state, coinc_next; /* synthesis syn_encoding = "sequential" */;
	
	// always @(posedge entry_clock or posedge reset) begin
		// if (reset)	coinc_state	<= #1 COINC_IDLE;
		// else 		coinc_state	<= #1 coinc_next;
	// end
	
	// always @(coinc_state or other_valid) begin
		// case (coinc_state)
			// COINC_IDLE : begin
				// coinc_met <= #1 1'b0;
				// if (req_coinc)	coinc_next <= #1 COINC_TBD;
				// else			coinc_next <= #1 COINC_IDLE;
			// end
			// COINC_TBD : begin
				// coinc_met <= #1 other_valid;
				// if (other_valid)	coinc_next <= #1 COINC_DONE;
				// else				coinc_next <= #1 COINC_TBD;
			// end
			// COINC_DONE : begin
				// coinc_met <= #1 1'b1;
				// coinc_next <= #1 COINC_DONE; //Permanent state until reset
			// end
		// endcase
	// end
	