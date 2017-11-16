
//	This module is used by channel_monitor to operate its four sequence_savers and control when saving and copying
//	functions move from one saver to the next, ensuring that only one is involved in each task at a given time.

`timescale 1 ns / 1 ps

module			saver_selector			(
	input				reset			,	//Reset, active high
	input				entry_clock		,	//Entry clock
	input				A_saved			,	//Indicates that a sequence has been saved by A
	input				B_saved			,	//Indicates that a sequence has been saved by B
	input				C_saved			,	//Indicates that a sequence has been saved by C
	input				D_saved			,	//Indicates that a sequence has been saved by D
	input				A_copied		,	//Indicates that the sequence has been copied from A
	input				B_copied		,	//Indicates that the sequence has been copied from B
	input				C_copied		,	//Indicates that the sequence has been copied from C
	input				D_copied		,	//Indicates that the sequence has been copied from D
	input		[3:0]	copy_job		,	//Indicates which saver is being copied from now
	output		[3:0]	next_copy_from	,	//Tells channel_monitor which saver to copy from next
	output	reg			A_reset			,	//Resets A_sequence_saver
	output	reg			B_reset			,	//Resets B_sequence_saver
	output	reg			C_reset			,	//Resets C_sequence_saver
	output	reg			D_reset			,	//Resets D_sequence_saver
	output	reg			A_saving		,	//Instructs channel_monitor to use A to save a sequence
	output	reg			B_saving		,	//Instructs channel_monitor to use B to save a sequence
	output	reg			C_saving		,	//Instructs channel_monitor to use C to save a sequence
	output	reg			D_saving		)	//Instructs channel_monitor to use D to save a sequence
	;
	
	//STATE MACHINE TO SELECT WHICH SEQUENCE_SAVER WILL CAPTURE SAMPLE SEQUENCES NEXT
	reg	[1:0]	next_save_with; /* synthesis syn_encoding = "sequential" */
	parameter NEXT_USE_A = 2'b00; //Use sequence_saver_A next
	parameter NEXT_USE_B = 2'b01; //Use sequence_saver_B next
	parameter NEXT_USE_C = 2'b10; //Use sequence_saver_C next
	parameter NEXT_USE_D = 2'b11; //Use sequence_saver_D next
	
	reg	next_save_A;	//Tells the state machine operating A_sequence_saver to deassert A_reset
	reg	next_save_B;	//Tells the state machine operating B_sequence_saver to deassert B_reset
	reg	next_save_C;	//Tells the state machine operating C_sequence_saver to deassert C_reset
	reg	next_save_D;	//Tells the state machine operating D_sequence_saver to deassert D_reset
	
	always @(posedge entry_clock or posedge reset) begin
		if (reset) begin
			next_save_A		<= #1 1'b0;
			next_save_B		<= #1 1'b0;
			next_save_C		<= #1 1'b0;
			next_save_D		<= #1 1'b0;
			next_save_with	<= #1 NEXT_USE_A;
		end
		else begin
			case (next_save_with)
				NEXT_USE_A : begin
					next_save_A		<= #1 ~A_saving;
					next_save_B		<= #1 A_saving;
					next_save_C		<= #1 1'b0;
					next_save_D		<= #1 1'b0;
					if (A_saving)	next_save_with <= #1 NEXT_USE_B;	//Proceed, or
					else			next_save_with <= #1 NEXT_USE_A;	// ...stay.
				end
				NEXT_USE_B : begin
					next_save_A		<= #1 1'b0;
					next_save_B		<= #1 ~B_saving;
					next_save_C		<= #1 B_saving;
					next_save_D		<= #1 1'b0;
					if (B_saving)	next_save_with <= #1 NEXT_USE_C;	//Proceed, or
					else			next_save_with <= #1 NEXT_USE_B;	// ...stay.
				end
				NEXT_USE_C : begin
					next_save_A		<= #1 1'b0;
					next_save_B		<= #1 1'b0;
					next_save_C		<= #1 ~C_saving;
					next_save_D		<= #1 C_saving;
					if (C_saving)	next_save_with <= #1 NEXT_USE_D;	//Proceed, or
					else			next_save_with <= #1 NEXT_USE_C;	// ...stay.
				end
				NEXT_USE_D : begin
					next_save_A		<= #1 D_saving;
					next_save_B		<= #1 1'b0;
					next_save_C		<= #1 1'b0;
					next_save_D		<= #1 ~D_saving;
					if (D_saving)	next_save_with <= #1 NEXT_USE_A;	//Loop back to first, or
					else			next_save_with <= #1 NEXT_USE_D;	// ...stay.
				end
				default : begin
					next_save_A		<= #1 1'b0;
					next_save_B		<= #1 1'b0;
					next_save_C		<= #1 1'b0;
					next_save_D		<= #1 1'b0;
					next_save_with	<= #1 NEXT_USE_A;	//Go to first state
				end
			endcase
		end
	end
	
	
	//STATE MACHINE TO OPERATE A_SEQUENCE_SAVER
	reg	[1:0]	A_saver_state; /* synthesis syn_encoding = "sequential" */
	parameter RESET		= 2'b00; //Assert reset for this sequence saver
	parameter ENABLE	= 2'b01; //Deassert reset in preparation for saving
	parameter SAVE		= 2'b10; //This saver is the one working to capture a sequence
	parameter COPY		= 2'b11; //This saver is waiting for its sequence to be copied
	
	always @(posedge entry_clock or posedge reset) begin
		if (reset) begin
			A_reset			<= #1 1'b1;
			A_saving	 	<= #1 1'b0;
			A_saver_state	<= #1 RESET;
		end
		else begin
			case (A_saver_state)
				RESET : begin		//Assert reset for this sequence saver
					A_reset			<= #1 1'b1;
					A_saving	 	<= #1 1'b0;
					if (next_save_A)	A_saver_state <= #1 ENABLE;	//Proceed, or
					else				A_saver_state <= #1 RESET;	// ...stay.
				end
				ENABLE : begin		//Deassert reset in preparation for saving
					A_reset			<= #1 1'b0;
					A_saving	 	<= #1 1'b0;
					if (D_saved)	A_saver_state <= #1 SAVE;	//Proceed, or
					else			A_saver_state <= #1 ENABLE;	// ...stay.
				end
				SAVE : begin		//This saver is the one working to capture a sequence
					A_reset			<= #1 1'b0;
					A_saving	 	<= #1 1'b1;
					if (A_saved)	A_saver_state <= #1 COPY;	//Proceed, or
					else			A_saver_state <= #1 SAVE;	// ...stay.
				end
				COPY : begin		//This saver is waiting for its sequence to be copied
					A_reset			<= #1 1'b0;
					A_saving	 	<= #1 1'b0;
					if (A_copied)	A_saver_state <= #1 RESET;	//Loop back to first, or
					else			A_saver_state <= #1 COPY;	// ...stay.
				end
				default : begin
					A_reset			<= #1 1'b0;
					A_saving	 	<= #1 1'b0;
					A_saver_state	<= #1 RESET;	//Go to first state
				end
			endcase
		end
	end
	
	
	//STATE MACHINE TO OPERATE B_SEQUENCE_SAVER
	reg	[1:0]	B_saver_state; /* synthesis syn_encoding = "sequential" */
	//(Uses same declared states as A_saver_state above)
	
	always @(posedge entry_clock or posedge reset) begin
		if (reset) begin
			B_reset			<= #1 1'b1;
			B_saving	 	<= #1 1'b0;
			B_saver_state	<= #1 RESET;
		end
		else begin
			case (B_saver_state)
				RESET : begin		//Assert reset for this sequence saver
					B_reset			<= #1 1'b1;
					B_saving	 	<= #1 1'b0;
					if (next_save_B)	B_saver_state <= #1 ENABLE;	//Proceed, or
					else				B_saver_state <= #1 RESET;	// ...stay.
				end
				ENABLE : begin		//Deassert reset in preparation for saving
					B_reset			<= #1 1'b0;
					B_saving	 	<= #1 1'b0;
					if (A_saved)	B_saver_state <= #1 SAVE;	//Proceed, or
					else			B_saver_state <= #1 ENABLE;	// ...stay.
				end
				SAVE : begin		//This saver is the one working to capture a sequence
					B_reset			<= #1 1'b0;
					B_saving	 	<= #1 1'b1;
					if (B_saved)	B_saver_state <= #1 COPY;	//Proceed, or
					else			B_saver_state <= #1 SAVE;	// ...stay.
				end
				COPY : begin		//This saver is waiting for its sequence to be copied
					B_reset			<= #1 1'b0;
					B_saving	 	<= #1 1'b0;
					if (B_copied)	B_saver_state <= #1 RESET;	//Loop back to first, or
					else			B_saver_state <= #1 COPY;	// ...stay.
				end
				default : begin
					B_reset			<= #1 1'b0;
					B_saving	 	<= #1 1'b0;
					B_saver_state	<= #1 RESET;	//Go to first state
				end
			endcase
		end
	end
	
	
	//STATE MACHINE TO OPERATE C_SEQUENCE_SAVER
	reg	[1:0]	C_saver_state; /* synthesis syn_encoding = "sequential" */
	//(Uses same declared states as A_saver_state above)
	
	always @(posedge entry_clock or posedge reset) begin
		if (reset) begin
			C_reset			<= #1 1'b1;
			C_saving	 	<= #1 1'b0;
			C_saver_state	<= #1 RESET;
		end
		else begin
			case (C_saver_state)
				RESET : begin		//Assert reset for this sequence saver
					C_reset			<= #1 1'b1;
					C_saving	 	<= #1 1'b0;
					if (next_save_C)	C_saver_state	<= #1 ENABLE;	//Proceed, or
					else				C_saver_state	<= #1 RESET;	// ...stay.
				end
				ENABLE : begin		//Deassert reset in preparation for saving
					C_reset			<= #1 1'b0;
					C_saving	 	<= #1 1'b0;
					if (B_saved)	C_saver_state	<= #1 SAVE;	//Proceed, or
					else			C_saver_state	<= #1 ENABLE;	// ...stay.
				end
				SAVE : begin		//This saver is the one working to capture a sequence
					C_reset			<= #1 1'b0;
					C_saving	 	<= #1 1'b1;
					if (C_saved)	C_saver_state	<= #1 COPY;	//Proceed, or
					else			C_saver_state	<= #1 SAVE;	// ...stay.
				end
				COPY : begin		//This saver is waiting for its sequence to be copied
					C_reset			<= #1 1'b0;
					C_saving	 	<= #1 1'b0;
					if (C_copied)	C_saver_state	<= #1 RESET;	//Loop back to first, or
					else			C_saver_state	<= #1 COPY;	// ...stay.
				end
				default : begin
					C_reset			<= #1 1'b0;
					C_saving	 	<= #1 1'b0;
					C_saver_state	<= #1 RESET;	//Go to first state
				end
			endcase
		end
	end
	
	
	//STATE MACHINE TO OPERATE D_SEQUENCE_SAVER
	reg	[1:0]	D_saver_state; /* synthesis syn_encoding = "sequential" */
	//(Uses same declared states as A_saver_state above)
	
	always @(posedge entry_clock or posedge reset) begin
		if (reset) begin
			D_reset			<= #1 1'b1;
			D_saving	 	<= #1 1'b0;
			D_saver_state	<= #1 RESET;
		end
		else begin
			case (D_saver_state)
				RESET : begin		//Assert reset for this sequence saver
					D_reset			<= #1 1'b1;
					D_saving	 	<= #1 1'b0;
					if (next_save_D)	D_saver_state	<= #1 ENABLE;	//Proceed, or
					else				D_saver_state	<= #1 RESET;	// ...stay.
				end
				ENABLE : begin		//Deassert reset in preparation for saving
					D_reset			<= #1 1'b0;
					D_saving	 	<= #1 1'b0;
					if (C_saved)	D_saver_state	<= #1 SAVE;	//Proceed, or
					else			D_saver_state	<= #1 ENABLE;	// ...stay.
				end
				SAVE : begin		//This saver is the one working to capture a sequence
					D_reset			<= #1 1'b0;
					D_saving	 	<= #1 1'b1;
					if (D_saved)	D_saver_state	<= #1 COPY;	//Proceed, or
					else			D_saver_state	<= #1 SAVE;	// ...stay.
				end
				COPY : begin		//This saver is waiting for its sequence to be copied
					D_reset			<= #1 1'b0;
					D_saving	 	<= #1 1'b0;
					if (D_copied)	D_saver_state	<= #1 RESET;	//Loop back to first, or
					else			D_saver_state	<= #1 COPY;	// ...stay.
				end
				default : begin
					D_reset			<= #1 1'b0;
					D_saving	 	<= #1 1'b0;
					D_saver_state	<= #1 RESET;	//Go to first state
				end
			endcase
		end
	end
	
	
	//STATE MACHINE TO SELECT WHICH SEQUENCE_SAVER WILL HAVE ITS SEQUENCE COPIED NEXT
	reg	[3:0]	next_copy; /* synthesis syn_encoding = "onehot" */
	assign 		next_copy_from = next_copy;	//Output to the copy_job state machine in channel_monitor
	parameter FROM_A	= 4'b0001; //Copy the sequence from saver A next
	parameter FROM_B	= 4'b0010; //Copy the sequence from saver B next
	parameter FROM_C	= 4'b0100; //Copy the sequence from saver C next
	parameter FROM_D	= 4'b1000; //Copy the sequence from saver D next
	
	always @(posedge entry_clock or posedge reset) begin
		if (reset) next_copy	<= #1 FROM_A;
		else begin
			case (next_copy)	//Note: copy_job's bit order corresponds to ["D, C, B, A"]
				FROM_A : begin
					if (copy_job[0])	next_copy <= #1 FROM_B;	//Proceed, or
					else				next_copy <= #1 FROM_A;	// ...stay.
				end
				FROM_B : begin
					if (copy_job[1])	next_copy <= #1 FROM_C;	//Proceed, or
					else				next_copy <= #1 FROM_B;	// ...stay.
				end
				FROM_C : begin
					if (copy_job[2])	next_copy <= #1 FROM_D;	//Proceed, or
					else				next_copy <= #1 FROM_C;	// ...stay.
				end
				FROM_D : begin
					if (copy_job[3])	next_copy <= #1 FROM_A;	//Proceed, or
					else				next_copy <= #1 FROM_D;	// ...stay.
				end
				default : next_copy <= #1 FROM_A;
			endcase
		end
	end

endmodule


// ******************************* NO LONGER USED OR NO LONGER APPLICABLE: *******************************

	// reg	next_save_A;	//Tells 
	// reg	next_save_B;	//Tells 
	// reg	next_save_C;	//Tells 
	// reg	next_save_D;	//Tells 
	
	// always @(posedge entry_clock or posedge reset) begin
		// if (reset) begin
			// next_copy_A		<= #1 1'b1;
			// next_copy_B		<= #1 1'b0;
			// next_copy_C		<= #1 1'b0;
			// next_copy_D		<= #1 1'b0;
			// next_copy_from	<= #1 NEXT_USE_A;
		// end
		// else begin
			// case (next_copy_from)
				// NEXT_USE_A : begin
					// next_copy_A		<= #1 ~A_saving;
					// next_copy_B		<= #1 A_saving;
					// next_copy_C		<= #1 1'b0;
					// next_copy_D		<= #1 1'b0;
					// next_copy_from	<= #1 A_saving ? NEXT_USE_B :
											// NEXT_USE_A;
					// end
				// NEXT_USE_B : begin
					// next_copy_A		<= #1 1'b0;
					// next_copy_B		<= #1 ~B_saving;
					// next_copy_C		<= #1 B_saving;
					// next_copy_D		<= #1 1'b0;
					// next_copy_from	<= #1 B_saving ? NEXT_USE_C :
											// NEXT_USE_B;
					// end
				// NEXT_USE_C : begin
					// next_copy_A		<= #1 1'b0;
					// next_copy_B		<= #1 1'b0;
					// next_copy_C		<= #1 ~C_saving;
					// next_copy_D		<= #1 C_saving;
					// next_copy_from	<= #1 C_saving ? NEXT_USE_D :
											// NEXT_USE_C;
					// end
				// NEXT_USE_D : begin
					// next_copy_A		<= #1 D_saving;
					// next_copy_B		<= #1 1'b0;
					// next_copy_C		<= #1 1'b0;
					// next_copy_D		<= #1 ~D_saving;
					// next_copy_from	<= #1 D_saving ? NEXT_USE_A :
											// NEXT_USE_D;
					// end
				// default : begin
					// next_copy_A		<= #1 1'b0;
					// next_copy_B		<= #1 1'b0;
					// next_copy_C		<= #1 1'b0;
					// next_copy_D		<= #1 1'b0;
					// next_copy_from	<= #1 NEXT_USE_A;
				// end
			// endcase
		// end
			
