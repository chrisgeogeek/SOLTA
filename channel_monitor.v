
//This module monitors samples from one channel of the analog-to-digital converter, manages the capture of sample 
//sequences, and outputs those sequences and associated metadata to entry_maker. The sequences are actually captured
//by four local instances of the sequence_saver module. But channel_monitor coordinates which sequence_saver is being 
//saved to and which is being copied from at any given time with the assistance of a local instance of the 
//saver_selector module. The metadata which channel_monitor outputs to entry_maker get recorded in entry headers.
//These are DOR_flag, max_dur_flag, seq_saved, seq_sum and seq_dur, and these are described below.

`timescale 1 ns / 1 ps

module				channel_monitor		(
	input				reset			,	//Reset, active high
	input				entry_clock		,	//Entry clock
	input				enable_saving	,	//Enable the examination and acquisition of ADC sample sequences
	input		[28:0]	criteria		,	//{req_coinc, min_dur, max_dur, min_amp, DOR_limit}
	input				update_crit		,	//Signal to make this module update its criteria
	output	reg			crit_updated	,	//Indicator that sequence criteria changes have been carried out
	input				DOR				,	//Data-Out-of-Range indicator from the ADC
	input		[7:0]	sample			,	//The 8-bit sample sent by the ADC
	output				DOR_flag		,	//Entry header flag
	//output				max_dur_flag	,	//Entry header flag, 1: disables hit checks until new_second = 1
	output	reg			seq_saved		,	//Indicator that a sequence has been saved
	output	reg	[15:0]	seq_sum			,	//The sum of all samples in the saved sequence
	output	reg	[5:0]	seq_dur			,	//The duration of the saved sequence
	input				fifo_AmFull		,	//"Almost Full" Indicator from an external channel_fifo instance
	output	reg			fifo_WrEn		,	//Signal to enable writing to the external channel_fifo instance
	output	reg	[7:0]	fifo_Data		,	//Data word to write to the external channel_fifo instance
	input				new_second		,	//Resets DOR_flag and max_dur_flag
	//Multi-channel coincidence signals, used to determine whether pulses on different channels have any time overlap
	output				seq_valid		,	//Indicator that a satisfactory sample sequence is being saved on this channel
	input				other_valid		)	//Indicates that seq_valid is true for the other channel_monitor
	;
	
	//COUNTER AND COMPARATOR FOR CONTROLLING THE DATA-OUT-OF-RANGE METADATA FLAG, DOR_flag
	wire	[7:0]	DOR_limit, DOR_count;
	up_counter_8		DOR_COUNTER			(	//Counter for Data Over Range indications sent by the ADC
		.Aclr 				(new_second		),	//Asynchronous signal to clear the counter
		.Clock				(entry_clock	),	//Clock to increment the counter
		.Clk_En 			(DOR			),	//Signal to enable the counter's clock
		.Q					(DOR_count		))	//Output. 8-bit output with the present count value
	;
	com_ge_8_reg		DOR_COUNT_COM		(	//Comparator to indicate when DOR_count >= DOR_limit
		.DataA				(DOR_count		),	//8-bit count value from DOR_COUNTER
		.DataB				(DOR_limit		),	//The parameter controlling the sensitivity of this flag
		.Clock				(entry_clock	),	//Clock to update the output register
		.ClockEn			(~reset			),	//Clock enable
		.Aclr				(new_second		),	//Asynchronous signal to clear the output register
		.AGEB				(DOR_flag		))	//Output. Flag to be recorded in an entry header
	;
	
	
	//COMPARATOR TO INDICATE WHEN A SAMPLE VALUE IS >= min_amp, A REQUIRED CONDITION FOR SEQUENCE CAPTURE
	wire	[7:0]	min_amp;	//Minimum sample amplitude to trigger sequence capture
	com_ge_8_reg		AMPLITUDE_COM		(
		.DataA				(sample			),	//Sample
		.DataB				(min_amp		),	//Sample amplitude threshold
		.Clock				(entry_clock	),	//Clock to update the output register
		.ClockEn			(~reset			),	//Clock enable
		.Aclr				(reset			),	//Asynchronous signal to clear the output register
		.AGEB				(good_sample	))	//Output. Indicates that sample >= min_amp
	;
	
	//STATE MACHINE TO CONTROL THE save_sample SIGNAL
	reg	[4:0]	save_sample_sm; /* synthesis syn_encoding = "onehot" */
	parameter	SAVE_IDLE	= 5'b00001; //Don't assert save_sample
	parameter	SAVING		= 5'b00010; //Assert save_sample
	parameter	STOP_IN_3	= 5'b00100; //Deassert save_sample in 3 cycles
	parameter	STOP_IN_2	= 5'b01000; //Deassert save_sample in 2 cycles
	parameter	STOP_IN_1	= 5'b10000; //Deassert save_sample in 1 cycles
	reg		save_sample;
	
	always @(posedge entry_clock or posedge reset) begin
		if (reset) begin
			save_sample		<= #1 1'b0;
			save_sample_sm	<= #1 SAVE_IDLE;
		end
		else begin
			case (save_sample_sm)
				SAVE_IDLE : begin
					save_sample		<= #1 enable_saving & good_sample;
					if (enable_saving & good_sample)	save_sample_sm	<= #1 SAVING;
					else								save_sample_sm	<= #1 SAVE_IDLE;
				end
				SAVING : begin
					save_sample		<= #1 1'b1;
					if (enable_saving & good_sample)	save_sample_sm	<= #1 SAVING;
					else								save_sample_sm	<= #1 STOP_IN_3;
				end
				STOP_IN_3 : begin
					save_sample		<= #1 1'b1;
					if (enable_saving & good_sample)	save_sample_sm	<= #1 SAVING;
					else								save_sample_sm	<= #1 STOP_IN_2;
				end
				STOP_IN_2 : begin
					save_sample		<= #1 1'b1;
					if (enable_saving & good_sample)	save_sample_sm	<= #1 STOP_IN_3;
					else								save_sample_sm	<= #1 STOP_IN_1;
				end
				STOP_IN_1 : begin
					save_sample		<= #1 1'b0;
					save_sample_sm	<= #1 SAVE_IDLE;
				end
				default :  begin
					save_sample		<= #1 1'b0;
					save_sample_sm	<= #1 SAVE_IDLE;
				end
			endcase
		end
	end
	
	
	//THE SAMPLE PIPELINE FOR SEQUENCE SAVERS
	reg		[7:0]	sample_buf1, sample_buf2, sample_buf3;	//Pipeline registers
	always @(posedge entry_clock or posedge reset) begin
		if (reset) begin
			sample_buf1		<= #1 8'd0;
			sample_buf2		<= #1 8'd0;
			sample_buf3		<= #1 8'd0;
		end
		else begin
			sample_buf1	<= #1 sample;		//Single-cycle delay
			sample_buf2	<= #1 sample_buf1;	//2-cycle delay
			sample_buf3	<= #1 sample_buf2;	//3-cycle delay
		end
	end
	//It's sample_buf3 that is input to the sequence savers.
	//This ensures that captured sequences start with two samples below the min_amp threshold.
	
	//SEQUENCE SAVERS AND THE SAVER SELECTOR TO COORDINATE THEM
	wire	[5:0]	A_seq_dur, B_seq_dur, C_seq_dur, D_seq_dur, min_dur, max_dur;
	wire	[7:0]	A_Q, B_Q, C_Q, D_Q;
	wire	[15:0]	A_seq_sum, B_seq_sum, C_seq_sum, D_seq_sum;
	
	sequence_saver		A_SEQUENCE_SAVER	(
		.reset				(A_reset		),	//Input. Reset, active high
		.entry_clock		(entry_clock	),	//Input. Entry clock
		.sample				(sample_buf3 	),	//Input. ADC sample from 3 cycles ago
		.save_sample		(A_save_sample	),	//Input. Instruction to save samples. Must be held high to record a sequence
		.seq_saved			(A_saved		),	//Output. Indicator that a sequence has been saved
		.seq_sum			(A_seq_sum		),	//Output. The sum of all samples in the saved sequence
		.seq_dur			(A_seq_dur		),	//Output. The duration of the saved sequence
		.min_dur			(min_dur		),	//Input. The minimum required sequence duration/length
		.max_dur			(max_dur		),	//Input. The maximum allowed sequence duration/length
		.dur_maxed			(A_dur_maxed	),	//Output. Indicator that a sequence has lasted max_dur
		.RdEn				(A_RdEn			),	//Input. Read enable for internal memory
		.Q					(A_Q			),	//Output. Output data word from memory
		.Empty				(A_Empty		),	//Output. Indicates 0 of 64 words are stored in memory
		.copied				(A_copied		),	//Output. Indicates that no uncopied sequence is waiting
		.seq_valid			(A_seq_valid	),	//Output. Indicates that this instance is saving a satisfactory sample sequence
		.req_coinc			(req_coinc		),	//Input. 1: other_valid must be asserted some time during a sequence for it to be saved
		.other_valid		(other_valid	))	//Input. Indicates that seq_valid is true for the other channel_monitor
	;
	sequence_saver		B_SEQUENCE_SAVER	(
		.reset				(B_reset		),	//Input. Reset, active high
		.entry_clock		(entry_clock	),	//Input. Entry clock
		.sample				(sample_buf3 	),	//Input. ADC sample from 3 cycles ago
		.save_sample		(B_save_sample	),	//Input. Instruction to save samples. Must be held high to record a sequence
		.seq_saved			(B_saved		),	//Output. Indicator that a sequence has been saved
		.seq_sum			(B_seq_sum		),	//Output. The sum of all samples in the saved sequence
		.seq_dur			(B_seq_dur		),	//Output. The duration of the saved sequence
		.min_dur			(min_dur		),	//Input. The minimum required sequence duration/length
		.max_dur			(max_dur		),	//Input. The maximum allowed sequence duration/length
		.dur_maxed			(B_dur_maxed	),	//Output. Indicator that a sequence has lasted max_dur
		.RdEn				(B_RdEn			),	//Input. Read enable for internal memory
		.Q					(B_Q			),	//Output. Output data word from memory
		.Empty				(B_Empty		),	//Output. Indicates 0 of 64 words are stored in memory
		.copied				(B_copied		),	//Output. Indicates that no uncopied sequence is waiting
		.seq_valid			(B_seq_valid	),	//Output. Indicates that this instance is saving a satisfactory sample sequence
		.req_coinc			(req_coinc		),	//Input. 1: other_valid must be asserted some time during a sequence for it to be saved
		.other_valid		(other_valid	))	//Input. Indicates that seq_valid is true for the other channel_monitor
	;
	sequence_saver		C_SEQUENCE_SAVER	(
		.reset				(C_reset		),	//Input. Reset, active high
		.entry_clock		(entry_clock	),	//Input. Entry clock
		.sample				(sample_buf3 	),	//Input. ADC sample from 3 cycles ago
		.save_sample		(C_save_sample	),	//Input. Instruction to save samples. Must be held high to record a sequence
		.seq_saved			(C_saved		),	//Output. Indicator that a sequence has been saved
		.seq_sum			(C_seq_sum		),	//Output. The sum of all samples in the saved sequence
		.seq_dur			(C_seq_dur		),	//Output. The duration of the saved sequence
		.min_dur			(min_dur		),	//Input. The minimum required sequence duration/length
		.max_dur			(max_dur		),	//Input. The maximum allowed sequence duration/length
		.dur_maxed			(C_dur_maxed	),	//Output. Indicator that a sequence has lasted max_dur
		.RdEn				(C_RdEn			),	//Input. Read enable for internal memory
		.Q					(C_Q			),	//Output. Output data word from memory
		.Empty				(C_Empty		),	//Output. Indicates 0 of 64 words are stored in memory
		.copied				(C_copied		),	//Output. Indicates that no uncopied sequence is waiting
		.seq_valid			(C_seq_valid	),	//Output. Indicates that this instance is saving a satisfactory sample sequence
		.req_coinc			(req_coinc		),	//Input. 1: other_valid must be asserted some time during a sequence for it to be saved
		.other_valid		(other_valid	))	//Input. Indicates that seq_valid is true for the other channel_monitor
	;
	sequence_saver		D_SEQUENCE_SAVER	(
		.reset				(D_reset		),	//Input. Reset, active high
		.entry_clock		(entry_clock	),	//Input. Entry clock
		.sample				(sample_buf3 	),	//Input. ADC sample from 3 cycles ago
		.save_sample		(D_save_sample	),	//Input. Instruction to save samples. Must be held high to record a sequence
		.seq_saved			(D_saved		),	//Output. Indicator that a sequence has been saved
		.seq_sum			(D_seq_sum		),	//Output. The sum of all samples in the saved sequence
		.seq_dur			(D_seq_dur		),	//Output. The duration of the saved sequence
		.min_dur			(min_dur		),	//Input. The minimum required sequence duration/length
		.max_dur			(max_dur		),	//Input. The maximum allowed sequence duration/length
		.dur_maxed			(D_dur_maxed	),	//Output. Indicator that a sequence has lasted max_dur
		.RdEn				(D_RdEn			),	//Input. Read enable for internal memory
		.Q					(D_Q			),	//Output. Output data word from memory
		.Empty				(D_Empty		),	//Output. Indicates 0 of 64 words are stored in memory
		.copied				(D_copied		),	//Output. Indicates that no uncopied sequence is waiting
		.seq_valid			(D_seq_valid	),	//Output. Indicates that this instance is saving a satisfactory sample sequence
		.req_coinc			(req_coinc		),	//Input. 1: other_valid must be asserted some time during a sequence for it to be saved
		.other_valid		(other_valid	))	//Input. Indicates that seq_valid is true for the other channel_monitor
	;
	assign	seq_valid	= A_seq_valid | B_seq_valid | C_seq_valid | D_seq_valid;
	assign	A_save_sample	= save_sample & A_saving;
	assign	B_save_sample	= save_sample & B_saving;
	assign	C_save_sample	= save_sample & C_saving;
	assign	D_save_sample	= save_sample & D_saving;
	
	wire	[3:0]	copy_job, next_copy_from;
	saver_selector		SAVER_SELECTOR_INST	(
		.reset				(reset			),	//Input. Reset, active high
		.entry_clock		(entry_clock	),	//Input. Entry clock
		.A_saved			(A_saved		),	//Input. Indicates that a sequence has been saved by A
		.B_saved			(B_saved		),	//Input. Indicates that a sequence has been saved by B
		.C_saved			(C_saved		),	//Input. Indicates that a sequence has been saved by C
		.D_saved			(D_saved		),	//Input. Indicates that a sequence has been saved by D
		.A_copied			(A_copied		),	//Input. Indicates that the sequence has been copied from A
		.B_copied			(B_copied		),	//Input. Indicates that the sequence has been copied from B
		.C_copied			(C_copied		),	//Input. Indicates that the sequence has been copied from C
		.D_copied			(D_copied		),	//Input. Indicates that the sequence has been copied from D
		.copy_job			(copy_job		),	//Input. Indicates which saver is being copied from now
		.next_copy_from		(next_copy_from	),	//Output. Tells channel_monitor which saver to copy from next
		.A_reset			(A_reset		),	//Output. Resets A_sequence_saver
		.B_reset			(B_reset		),	//Output. Resets B_sequence_saver
		.C_reset			(C_reset		),	//Output. Resets C_sequence_saver
		.D_reset			(D_reset		),	//Output. Resets D_sequence_saver
		.A_saving			(A_saving		),	//Output. Instructs channel_monitor to use A to save a sequence
		.B_saving			(B_saving		),	//Output. Instructs channel_monitor to use B to save a sequence
		.C_saving			(C_saving		),	//Output. Instructs channel_monitor to use C to save a sequence
		.D_saving			(D_saving		))	//Output. Instructs channel_monitor to use D to save a sequence
	;
	
	reg		[7:0]	next_Data;
	reg		[3:0]	copied_bufs, RdEn_bufs;
	reg				next_WrEn;
	
	//STATE MACHINE TO COPY SEQUENCES INTO CHANNEL_FIFO_INST
	reg	[4:0]	copy_job_sm; /* synthesis syn_encoding = "onehot" */
	parameter	IDLE	= 5'b00001; //Don't copy any sequence
	parameter	FROM_A	= 5'b00010; //Copy the sequence from A_sequence_saver
	parameter	FROM_B	= 5'b00100; //Copy the sequence from B_sequence_saver
	parameter	FROM_C	= 5'b01000; //Copy the sequence from C_sequence_saver
	parameter	FROM_D	= 5'b10000; //Copy the sequence from D_sequence_saver
	//wire [4:0]	next_job;
	//assign		next_job = {next_copy_from, 1'b0};
	assign		copy_job = copy_job_sm[4:1];
	
	always @(posedge entry_clock or posedge reset) begin
		if (reset) begin
			RdEn_bufs	<= #1 4'b0000;
			copied_bufs	<= #1 4'b0000;
			next_WrEn	<= #1 1'd0;
			next_Data	<= #1 8'd0;
			copy_job_sm	<= #1 IDLE;
			fifo_WrEn	<= #1 1'd0;
			fifo_Data	<= #1 8'd0;
		end
		else begin
			RdEn_bufs	<= #1 {D_RdEn, C_RdEn, B_RdEn, A_RdEn};			//Pipeline register
			copied_bufs	<= #1 {D_copied, C_copied, B_copied, A_copied};	//Pipeline register
			case (copy_job_sm)
				IDLE : begin	//Check for space in entry_maker's channel_fifo memory for this channel
					next_WrEn	<= #1 1'b0;
					next_Data	<= #1 8'd0;
					if (fifo_AmFull)	copy_job_sm	<= #1 IDLE;		//Wait until channel_fifo has room
					else if (next_copy_from[3])	copy_job_sm	<= #1 FROM_D;
					else if (next_copy_from[2])	copy_job_sm	<= #1 FROM_C;
					else if (next_copy_from[1])	copy_job_sm	<= #1 FROM_B;
					else						copy_job_sm	<= #1 FROM_A;
				end
				FROM_A : begin	//Copying from A_SEQUENCE_SAVER
					next_WrEn	<= #1 RdEn_bufs[0];
					next_Data	<= #1 A_Q;
					if (copied_bufs[0])	copy_job_sm	<= #1 IDLE;		//Loop back to first, or
					else				copy_job_sm	<= #1 FROM_A;	// ...stay.
				end
				FROM_B : begin	//Copying from B_SEQUENCE_SAVER
					next_WrEn	<= #1 RdEn_bufs[1];
					next_Data	<= #1 B_Q;
					if (copied_bufs[1])	copy_job_sm	<= #1 IDLE;	//Loop back to first, or
					else				copy_job_sm	<= #1 	FROM_B;	// ...stay.
				end
				FROM_C : begin	//Copying from C_SEQUENCE_SAVER
					next_WrEn	<= #1 RdEn_bufs[2];
					next_Data	<= #1 C_Q;
					if (copied_bufs[2])	copy_job_sm	<= #1 IDLE;	//Loop back to first, or
					else				copy_job_sm	<= #1 FROM_C;	// ...stay.
				end
				FROM_D : begin	//Copying from D_SEQUENCE_SAVER
					next_WrEn	<= #1 RdEn_bufs[3];
					next_Data	<= #1 D_Q;
					if (copied_bufs[3])	copy_job_sm	<= #1 IDLE;	//Loop back to first, or
					else				copy_job_sm	<= #1 FROM_D;	// ...stay.
				end
				default : begin	//Here in case of error
					next_WrEn	<= #1 1'b0;
					next_Data	<= #1 8'd0;
					copy_job_sm	<= #1 IDLE;	//Go to initial state
				end
			endcase
			fifo_WrEn	<= #1 next_WrEn;	//Pipeline register
			fifo_Data	<= #1 next_Data;	//Pipeline register
		end
	end
	
	assign	A_RdEn	= copy_job_sm[1] & A_saved & ~A_Empty;
	assign	B_RdEn	= copy_job_sm[2] & B_saved & ~B_Empty;
	assign	C_RdEn	= copy_job_sm[3] & C_saved & ~C_Empty;
	assign	D_RdEn	= copy_job_sm[4] & D_saved & ~D_Empty;
	
	
	//SEQUENCE INFO IS ASSIGNED VALUES FROM THE SEQUENCE_SAVER SELECTED BY THIS STATE MACHINE
	reg		[1:0]	set_seq_info_to; /* synthesis syn_encoding = "sequential" */
	parameter	INFO_A = 2'b00;	//Use the metadata from A_sequence_saver
	parameter	INFO_B = 2'b01;	//Use the metadata from B_sequence_saver
	parameter	INFO_C = 2'b10;	//Use the metadata from C_sequence_saver
	parameter	INFO_D = 2'b11;	//Use the metadata from D_sequence_saver
	
	always @(posedge entry_clock or posedge reset) begin
		if (reset) begin
			seq_saved		<= #1 1'd0;
			seq_sum			<= #1 16'd0;
			seq_dur			<= #1 6'd0;
			set_seq_info_to	<= #1 INFO_A;
		end
		else begin
			case (set_seq_info_to)	//Note: this register's bit order corresponds to ["A, B, C, D"]
				INFO_A : begin
					seq_saved	<= #1 A_saved;
					seq_sum		<= #1 A_seq_sum;
					seq_dur		<= #1 A_seq_dur;
					if (A_saved) set_seq_info_to <= #1 INFO_B;
					else		 set_seq_info_to <= #1 INFO_A;
				end
				INFO_B : begin
					{seq_saved, seq_sum, seq_dur}	<= #1 {B_saved, B_seq_sum, B_seq_dur};
					set_seq_info_to					<= #1 B_saved ? 2'd2 : 2'd1;
				end
				INFO_C : begin
					{seq_saved, seq_sum, seq_dur}	<= #1 {C_saved, C_seq_sum, C_seq_dur};
					set_seq_info_to					<= #1 C_saved ? 2'd3 : 2'd2;
				end
				INFO_D : begin
					{seq_saved, seq_sum, seq_dur}	<= #1 {D_saved, D_seq_sum, D_seq_dur};
					set_seq_info_to					<= #1 D_saved ? 2'd0 : 2'd3;
				end
			endcase
		end
	end


	reg	[28:0]	criteria_buf, new_criteria, set_criteria;
	reg			update_crit_buf;
	
	//STATE MACHINE TO DELAY CHANGES IN CRITERIA IF A SEQUENCE IS CURRENTLY BEING SAVED
	reg	[1:0]	crit_state; /* synthesis syn_encoding = "sequential" */
	parameter	CRIT_IDLE	= 2'b00; //No criteria change needed
	parameter	UPDATE		= 2'b01; //Loading new criteria
	parameter	UPDATING	= 2'b10; //Assigning new criteria
	parameter	UPDATED		= 2'b11; //Waiting for deassertion of update_crit
	
	always @(posedge entry_clock or posedge reset) begin
		if (reset) begin
			update_crit_buf	<= #1 1'b0;
			criteria_buf	<= #1 29'd0;
			new_criteria	<= #1 29'd0;
			//Default values of { req_coinc,	min_dur,	max_dur,	min_amp,	DOR_limit	} are:
			set_criteria	<= #1 { 1'd0,		6'd61,		6'd62, 		8'd255,		8'd127		};
			crit_updated	<= #1 1'd0;
			crit_state		<= #1 CRIT_IDLE;
		end
		else begin
			update_crit_buf	<= #1 update_crit;	//Pipeline register
			criteria_buf	<= #1 criteria;		//Pipeline register
			case (crit_state)
				CRIT_IDLE : begin
					new_criteria	<= #1 29'd0;		//Hold_constant
					set_criteria	<= #1 set_criteria;	//Hold_constant
					crit_updated	<= #1 1'd0;
					if (update_crit_buf)	crit_state	<= #1 UPDATE;	//Proceed, or
					else					crit_state	<= #1 CRIT_IDLE;		// ...stay.
				end
				UPDATE : begin
					new_criteria	<= #1 criteria_buf;	//Update from buffer
					set_criteria	<= #1 set_criteria;	//Hold_constant
					crit_updated	<= #1 1'd0;
					if (save_sample)	crit_state	<= #1 UPDATE;		//Stay, or
					else				crit_state	<= #1 UPDATING;		// ...proceed.
				end
				UPDATING : begin
					new_criteria	<= #1 new_criteria;	//Hold_constant
					set_criteria	<= #1 new_criteria;	//Update from new_criteria
					crit_updated	<= #1 1'd0;
					crit_state		<= #1 UPDATED;		//Proceed
				end
				UPDATED : begin
					new_criteria	<= #1 new_criteria;	//Hold_constant
					set_criteria	<= #1 set_criteria;	//Update real criteria registers
					crit_updated	<= #1 1'd1;			//Indicate completion
					if (update_crit_buf)	crit_state	<= #1 UPDATED;	//Stay, or
					else					crit_state	<= #1 CRIT_IDLE;		// ...loop back to first.
				end
				default : begin //Here in case of error
					new_criteria	<= #1 new_criteria;	//Hold_constant
					set_criteria	<= #1 set_criteria;	//Hold_constant
					crit_updated	<= #1 1'd0;
					crit_state		<= #1 IDLE;	//Go to first state
				end
			endcase
			
		end
	end
	assign	req_coinc	=  set_criteria[28];
	assign	min_dur		=  set_criteria[27:22];
	assign	max_dur		=  set_criteria[21:16];
	assign	min_amp		=  set_criteria[15:8];
	assign	DOR_limit	= set_criteria[7:0];
	

endmodule


/* ******************************* NO LONGER USED OR NO LONGER APPLICABLE: *******************************
	
	wire	[7:0]	max_dur_limit, max_dur_count;
	
	//CODE FOR CONTROLLING THE MAXIMUM DURATION LIMIT METADATA FLAG, max_dur_flag
	reg		[3:0]	dur_maxed_bufs;
	reg				A_new_maxed, B_new_maxed, C_new_maxed, D_new_maxed;
	always @(posedge entry_clock or posedge reset) begin
		if (reset) begin
			dur_maxed_bufs	<= #1 4'b0000;
			A_new_maxed		<= #1 1'd0;
			B_new_maxed		<= #1 1'd0;
			C_new_maxed		<= #1 1'd0;
			D_new_maxed		<= #1 1'd0;
		end
		else begin
			dur_maxed_bufs	<= #1 {A_dur_maxed, B_dur_maxed, C_dur_maxed, D_dur_maxed};
			A_new_maxed		<= #1  A_dur_maxed & ~dur_maxed_bufs[3] ;	//Deasserts after one cycle
			B_new_maxed		<= #1  B_dur_maxed & ~dur_maxed_bufs[2] ;	//Deasserts after one cycle
			C_new_maxed		<= #1  C_dur_maxed & ~dur_maxed_bufs[1] ;	//Deasserts after one cycle
			D_new_maxed		<= #1  D_dur_maxed & ~dur_maxed_bufs[0] ;	//Deasserts after one cycle
		end
	end
	assign	new_dur_maxed	= A_new_maxed | B_new_maxed | C_new_maxed | D_new_maxed; //Clk_En for MAX_DUR_COUNTER
	
	up_counter_8		MAX_DUR_COUNTER		(	//8-bit counter for how many times Hit Checks last until max_dur
		.Aclr 				(new_second		),	//Asynchronous signal to clear the counter
		.Clock				(entry_clock	),	//Clock to increment the counter
		.Clk_En 			(new_dur_maxed	),	//Signal to enable the counter's clock
		.Q					(max_dur_count	))	//8-bit output with the present count value
	;
	com_ge_8_reg		MAX_DUR_COUNT_COM	(	//8-bit comparator to indicate when max_dur_count >= max_dur_limit
		.DataA				(max_dur_count	),	//8-bit count value from MAX_DUR_COUNTER
		.DataB				(max_dur_limit	),	//The parameter controlling the sensitivity of this interrupt
		.Clock				(entry_clock	),	//Clock to update the output register
		.ClockEn			(~reset			),	//Clock enable
		.Aclr				(new_second		),	//Asynchronous signal to clear the output register
		.AGEB				(max_dur_flag	))	//Entry header flag that disables hit checks until new_second
	;
	assign	save_sample = enable_saving	& high_sample & ~max_dur_flag;
	
	
*/