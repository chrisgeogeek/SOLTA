
`timescale 1ns/ 1ps

module entry_maker						(
	input				reset			,	//Reset, active high
	input				system_clock	,	//System clock
	output	reg	[20:0]	maker_flags		,	//Various status indicators
	//Signals to configure and manage entries
	input				entry_clock		,	//Entry clock
	input				enable_auto		,	//Enable the automatic creation of new data entries
	input				enable_saving	,	//Enable the acquisition of ADC sample sequences
	input		[21:0]	adc_q			,	//Channel A data bus {MSB, ..., LSB}
	input				t_pulse			,	//External timepulse signal from the GNSS receiver
	input				make_entry		,	//Command to manually trigger the creation of a new entry
	input		[7:0]	seq_limit		,	//A limit on the number of sequence-induced entries created each second
	input		[28:0]	A_criteria		,	//Channel A data collection criteria
	input		[28:0]	B_criteria		,	//Channel B data collection criteria
	input				update_A		,	//Signal to make A_CHANNEL_MONITOR update its criteria
	input				update_B		,	//Signal to make B_CHANNEL_MONITOR update its criteria
	//Signals for writing entry bytes to ENTRY_FIFO_INST at top level
	input				entry_AmFull	,
	output	reg			entry_WrEn		,
	output	reg	[7:0]	entry_Data		)
	;
	
	reg		[111:0]	meta_Data;	//Control Register with a word of Entry metadata
	reg		[103:0]	extended_header;
	reg		[47:0]	seq_info_buf;
	reg		[38:0]	timestamp;
	reg		[28:0]	A_criteria_buf, B_criteria_buf;
	reg		[7:0]	header_details, last_status;
	reg		[2:0]	ready_latency;
	reg				manual_entry, system_ready, WrEn_trigger, meta_WrEn, next_meta_WrEn, update_A_buf, update_B_buf;
	
	wire			A_updated, A_seq_valid, A_saved;
	wire			B_updated, B_seq_valid, B_saved;
	wire	[3:0]	channel_flags;
	//wire	[6:0]	entry_status;
	wire	[7:0]	A_sample, B_sample, A_Data, B_Data, A_Q, B_Q, A_seq_dur_8, B_seq_dur_8, duration_sum, entry_size, entry_status;
	wire	[7:0]	seq_entries;
	wire	[11:0]	whole_seconds;
	wire	[15:0]	A_seq_sum, B_seq_sum;
	wire	[26:0]	sub_seconds;
	wire	[111:0]	meta_Q;
	assign	A_DOR		= adc_q[21];	//Indicates Data-Out-of-Range for ADC Channel A (outside of its analog input range)
	assign	A_sample	= adc_q[20:13];	//Digital sample of ADC Channel A
	assign	B_DOR		= adc_q[10];	//Indicates Data-Out-of-Range for ADC Channel B (outside of its analog input range)
	assign	B_sample	= adc_q[9:2];	//Digital sample of ADC Channel B
	wire	[5:0]	A_seq_dur, B_seq_dur, A_RdEn_count, B_RdEn_count;
	assign	A_seq_dur_8 = {2'b00, A_seq_dur};
	assign	B_seq_dur_8 = {2'b00, B_seq_dur};
	
	
	
	//CODE TO CREATE AND SAVE ENTRY HEADERS
	assign	entry_status = {t_pulse, subsecs_oFlow, seq_limit_flag, enable_saving, channel_flags};
	com_ne_8_noreg		ENTRY_STATUS_COM	(	//Comparator to indicate when the 8-bit entry_status has changed
		.DataA				(entry_status	),	//Input, status indicators during the present entry_clock cycle
		.DataB				(last_status	),	//Input, status indicators during the previous entry_clock cycle
		.ANEB				(status_changed	))	//Output, assertion triggers the creation of an entry and gets recorded
	;
	always @(posedge entry_clock or posedge reset) begin
		if (reset) begin
			ready_latency	<= #1 3'd0;
			system_ready	<= #1 1'd0;
			A_criteria_buf	<= #1 37'd0;
			B_criteria_buf	<= #1 37'd0;
			update_A_buf	<= #1 1'd0;
			update_B_buf	<= #1 1'd0;
			last_status		<= #1 8'd0;
			timestamp		<= #1 39'd0;
			header_details	<= #1 8'd0;
			WrEn_trigger	<= #1 1'd0;
			meta_WrEn		<= #1 1'd0;
			next_meta_WrEn	<= #1 1'd0;
			meta_Data		<= #1 112'd0;
			extended_header	<= #1 104'd0;
			seq_info_buf	<= #1 48'd0;
			//new_hour		<= #1 1'd0;
		end
		else begin
			ready_latency	<= #1 {ready_latency[1:0], 1'b1}; //=001, then 011, then 111
			system_ready	<= #1 ready_latency[2]; //This piplining ensures time for "Empty" flags to be asserted
			A_criteria_buf	<= #1 A_criteria;
			B_criteria_buf	<= #1 B_criteria;
			update_A_buf	<= #1 update_A;
			update_B_buf	<= #1 update_B;
			
			//DETERMINING METADATA ON PIPELINE CLOCK CYCLE N
			//duration_sum <= (A_seq_dur_8 + B_seq_dur_8) inside the SEQ_DUR_ADDER module instance declared below this loop
			header_details	<= #1 {manual_entry, status_changed, A_updated, A_seq_valid, A_saved, B_updated, B_seq_valid, B_saved}; //8*1'
			timestamp		<= #1 {whole_seconds, sub_seconds}; //12'+27'=39'
			last_status		<= #1 entry_status; //8'
			seq_info_buf	<= #1 {A_seq_sum, A_seq_dur_8, B_seq_sum, B_seq_dur_8}; //16'+8'+16'+8'=48'
			WrEn_trigger	<= #1  manual_entry | status_changed | A_updated | A_saved | B_updated | B_saved;
			
			//COMBINING METADATA ON PIPELINE CLOCK CYCLE N+1
			//Effectively, entry_size	<= (duration_sum + num_meta_bytes) inside the SIZE_ADDER module instance declared below this loop
			extended_header	<= #1 {timestamp, 1'b0, header_details, last_status, seq_info_buf}; //39'+1'+8'+8'+48'=104'
			next_meta_WrEn	<= #1  WrEn_trigger & enable_auto;
			
			//ATTACHING entry_size TO METADATA ON PIPELINE CLOCK CYCLE N+2
			meta_Data		<= #1 {entry_size, extended_header}; //8'+104'=112'
			meta_WrEn		<= #1 next_meta_WrEn;
			
		end	
	end
		
		
	//STATE MACHINE TO RESPOND TO make_entry COMMANDS AND INDICATE COMPLETION
	reg	[1:0]	make_entry_sm; /* synthesis syn_encoding = "sequential" */
	parameter	NOT_MAKING	= 2'b00; //No sequence-specific metadata
	parameter	MAKING		= 2'b01; //Just sequence-specific metadata for one channel (A or B)
	parameter	DONE_MAKING	= 2'b10; //Sequence-specific metadata for both channels (A and B)
	
	reg		entry_made;
	always @(posedge entry_clock or posedge reset) begin
		if (reset) begin
			manual_entry	<= #1 1'b0;
			entry_made		<= #1 1'b0;
			make_entry_sm	<= #1 NOT_MAKING;
		end
		else begin
			case (make_entry_sm)
				NOT_MAKING	: begin
					manual_entry	<= #1 1'b0;
					entry_made		<= #1 1'b0;
					if (make_entry)	make_entry_sm	<= #1 MAKING;
					else			make_entry_sm	<= #1 NOT_MAKING;
				end
				MAKING	: begin
					manual_entry	<= #1 1'b1;
					entry_made		<= #1 1'b0;
					make_entry_sm	<= #1 DONE_MAKING;
				end
				DONE_MAKING	: begin
					manual_entry	<= #1 1'b0;
					entry_made		<= #1 1'b1;
					if (make_entry)	make_entry_sm	<= #1 DONE_MAKING;
					else			make_entry_sm	<= #1 NOT_MAKING;
				end
				default	: begin
					manual_entry	<= #1 1'b0;
					entry_made		<= #1 1'b0;
					make_entry_sm	<= #1 NOT_MAKING;
				end
			endcase
		end
	end
			
	//STATE MACHINE TO CONTROL TRANSITIONS FROM EACH SECOND TO THE FOLLOWING SECOND
	reg	[1:0]	new_second_sm; /* synthesis syn_encoding = "sequential" */
	parameter	IDLE_NEW_SEC	= 2'b00; //No sequence-specific metadata
	parameter	START_NEW_SEC	= 2'b01; //Just sequence-specific metadata for one channel (A or B)
	parameter	DONE_NEW_SEC	= 2'b10; //Sequence-specific metadata for both channels (A and B)
	
	reg	new_second;
	
	always @(posedge entry_clock or posedge reset) begin
		if (reset) begin
			new_second		<= #1 1'b0;
			new_second_sm	<= #1 IDLE_NEW_SEC;
		end
		else begin
			case (new_second_sm)
				IDLE_NEW_SEC : begin
					if (t_pulse | subsecs_oFlow) begin
						new_second		<= #1 1'b1;	//Assert
						new_second_sm	<= #1 START_NEW_SEC;
					end
					else begin
						new_second		<= #1 1'b0;
						new_second_sm	<= #1 IDLE_NEW_SEC;
					end
				end
				START_NEW_SEC : begin
					new_second		<= #1 1'b0;	//Deassert
					new_second_sm	<= #1 DONE_NEW_SEC;
				end
				DONE_NEW_SEC : begin
					new_second		<= #1 1'b0;
					if (last_status[7] | t_pulse)	new_second_sm	<= #1 DONE_NEW_SEC;
					else							new_second_sm	<= #1 IDLE_NEW_SEC;
				end
			endcase
		end
	end
	
	
	//STATE MACHINE TO CONTROL THE num_meta_bytes REGISTER
	wire	[1:0]	num_meta_sm; /* synthesis syn_encoding = "sequential" */
	parameter	NO_SEQ		= 2'b00; //No sequence-specific metadata
	parameter	B_SEQ		= 2'b01; //Just sequence-specific metadata for just B
	parameter	A_SEQ		= 2'b10; //Just sequence-specific metadata for just A
	parameter	BOTH_SEQ	= 2'b11; //Sequence-specific metadata for both channels
	assign		num_meta_sm = {A_saved, B_saved};
	
	reg	[7:0]	num_meta_bytes;
	
	always @(posedge entry_clock or posedge reset) begin
		if (reset) begin
			num_meta_bytes	<= #1 8'd7;
		end
		else begin
			case (num_meta_sm)
				NO_SEQ :	num_meta_bytes	<= #1 8'd7;
				B_SEQ :		num_meta_bytes	<= #1 8'd10;
				A_SEQ :		num_meta_bytes	<= #1 8'd10;
				BOTH_SEQ :	num_meta_bytes	<= #1 8'd13;
				default : 	num_meta_bytes	<= #1 8'd7;
			endcase
		end
	end
	
	
	//CODE TO OUTPUT STATUS INFORMATION
	reg		all_LSBs, new_all_LSBs;
	wire	A_WrEn, A_Empty, A_Full, A_AmEmpty, A_AmFull;
	wire	B_WrEn, B_Empty, B_Full, B_AmEmpty, B_AmFull;
	
	always @(posedge system_clock or posedge reset) begin
		if (reset) begin
			all_LSBs		<= #1 1'd0;
			new_all_LSBs	<= #1 1'd0;
			maker_flags		<= #1 21'd0;
		end
		else begin
			all_LSBs		<= #1 adc_q[12] & adc_q[11] & adc_q[1] & adc_q[0];	 //I want these adc_q bits included in synthesis but they...
			new_all_LSBs	<= #1 (all_LSBs | new_all_LSBs) & ~maker_flags[18]; // ...aren't used in entry creation so I involve them here.
			
			maker_flags[20]		<= #1 1'd0; //avoids issue with the new_hour signal. May be used for other purpose in the future.
			maker_flags[19:16]	<= #1 {new_all_LSBs, entry_made, A_updated, B_updated}; //4*1' = 4'
			maker_flags[15:8]	<= #1 {meta_Full, meta_AmFull, meta_AmEmpty, meta_Empty, channel_flags}; //4*1' + 4' = 8'
			maker_flags[7:0]	<= #1 {A_Full, A_AmFull, A_AmEmpty, A_Empty, B_Full, B_AmFull, B_AmEmpty, B_Empty}; //8*1' = 8'
		end
	end
	reg		A_RdEn, B_RdEn;
	channel_monitor		A_CHANNEL_MONITOR		(	//Monitors Channel A samples, selecting and storing sequences
		.reset				(reset				),	//Output.Reset, active high
		.entry_clock		(entry_clock		),	//Input. Entry clock
		.enable_saving		(enable_seq_entries	),	//Input. Enable acquisition of ADC sample sequences
		.criteria			(A_criteria_buf		),	//Input. {req_coincidence, min_amp, min_dur, max_dur, max_dur_limit, DOR_limit}
		.update_crit		(update_A_buf		),	//Input. Signal to make this module update its criteria
		.crit_updated		(A_updated			),	//Output. 
		.DOR				(A_DOR				),	//Input. 
		.sample				(A_sample			),	//Input. 
		.DOR_flag			(channel_flags[3]	),	//Output. 
		//.max_dur_flag		(channel_flags[2]	),	//Output. 
		.seq_saved			(A_saved			),	//Output. 
		.seq_sum			(A_seq_sum			),	//Output. 
		.seq_dur			(A_seq_dur			),	//Output. 
		.fifo_AmFull		(A_AmFull			),	//Output. 
		.fifo_WrEn			(A_WrEn				),	//Output. Signal to enable writing fifo_Data
		.fifo_Data			(A_Data				),	//Output. 8-bit output data word from the read port of fifo_queue_seq
		.new_second			(new_second			),	//Input. Resets DOR_flag and max_dur_flag
		.seq_valid			(A_seq_valid		),	//Output. Indicates that this instance is saving a satisfactory sample sequence
		.other_valid		(B_seq_valid		))	//Input. Indicates that seq_valid is true for other channel(s)
	;
	assign enable_seq_entries	= enable_saving & ~seq_limit_flag;
	assign channel_flags[2] = 1'b0;	//This is because max_dur_flag is not used in this version
	channel_fifo		A_CHANNEL_FIFO		(	//1024 x 8 EBR-based FIFO to store the samples in selected sequences
		.Reset				(reset			),	//Input. Reset signal
		.RPReset			(reset			),	//Input. Read pointer reset signal
		.WrClock			(entry_clock	),	//Input. Write port clock
		.RdClock			(system_clock	),	//Input. Read port clock
		.WrEn				(A_WrEn			),	//Input. Signal to enable writing fifo_Data
		.Data				(A_Data			),	//Input. 8-bit write data word
		.RdEn				(A_RdEn			),	//Input. Signal to enable reading A_Q
		.Q					(A_Q			),	//Output. 8-bit read data word
		.Empty				(A_Empty		),	//Output. Indicates 0 of 1024 words are stored in memory
		.Full				(A_Full			),	//Output. Indicates 1024 of 1024 words are stored in memory
		.AlmostEmpty		(A_AmEmpty		),	//Output. Indicates that at most 511 of 1024 words are stored in memory
		.AlmostFull			(A_AmFull		))	//Output. Indicates that at least 956 of 1024 words are stored in memory
	;
	
	channel_monitor		B_CHANNEL_MONITOR		(	//Monitors Channel B samples, selecting and storing sequences
		.reset				(reset				),	//Input. Reset, active high
		.entry_clock		(entry_clock		),	//Input. Entry clock
		.enable_saving		(enable_seq_entries	),	//Input. Enable acquisition of ADC sample sequences
		.criteria			(B_criteria_buf		),	//Input. {change_crit, min_amp, min_dur, max_dur, max_dur_limit, DOR_limit}
		.update_crit		(update_B_buf		),	//Input. Signal to make this module update its criteria
		.crit_updated		(B_updated			),	//Output. 
		.DOR				(B_DOR				),	//Input. 
		.sample				(B_sample			),	//Input. 
		.DOR_flag			(channel_flags[1]	),	//Output. 
		//.max_dur_flag		(channel_flags[0]	),	//Output. 
		.seq_saved			(B_saved			),	//Output. 
		.seq_sum			(B_seq_sum			),	//Output. 
		.seq_dur			(B_seq_dur			),	//Output. 
		.fifo_AmFull		(B_AmFull			),	//Output. 
		.fifo_WrEn			(B_WrEn				),	//Output. Signal to enable writing fifo_Data
		.fifo_Data			(B_Data				),	//Output. 8-bit output data word from the read port of fifo_queue_seq
		.new_second			(new_second			),	//Input. Resets DOR_flag and max_dur_flag
		.seq_valid			(B_seq_valid		),	//Output. Indicates that this instance is saving a satisfactory sample sequence
		.other_valid		(A_seq_valid		))	//Input. Indicates that seq_valid is true for other channel(s)
	;
	assign channel_flags[0] = 1'b0;	//This is because max_dur_flag is not used in this version
	channel_fifo		B_CHANNEL_FIFO		(	//1024 x 8 EBR-based FIFO to store the samples in selected sequences
		.Reset				(reset			),	//Input. Reset signal
		.RPReset			(reset			),	//Input. Read pointer reset signal
		.WrClock			(entry_clock	),	//Input. Write port clock
		.RdClock			(system_clock	),	//Input. Read port clock
		.WrEn				(B_WrEn			),	//Input. Signal to enable writing fifo_Data
		.Data				(B_Data			),	//Input. 8-bit write data word
		.RdEn				(B_RdEn			),	//Input. Signal to enable reading B_Q
		.Q					(B_Q			),	//Output. 8-bit read data word
		.Empty				(B_Empty		),	//Output. Indicates 0 of 1024 words are stored in memory
		.Full				(B_Full			),	//Output. Indicates 1024 of 1024 words are stored in memory
		.AlmostEmpty		(B_AmEmpty		),	//Output. Indicates that at most 511 of 1024 words are stored in memory
		.AlmostFull			(B_AmFull		))	//Output. Indicates that at least 956 of 1024 words are stored in memory
	;

	adder8_reg			SEQ_DUR_ADDER		(	//
		.DataA				(A_seq_dur_8	),	//Input. 
		.DataB				(B_seq_dur_8	),	//Input. 
		.Clock				(entry_clock	),	//Input. 
		.Reset				(reset			),	//Input. 
		.ClockEn			(~reset			),	//Input. 
		.Result				(duration_sum	))	//Output. 
	;
	adder8_reg			SIZE_ADDER			(
		.DataA				(duration_sum	),	//Input. 
		.DataB				(num_meta_bytes	),	//Input. 
		.Clock				(entry_clock	),	//Input. 
		.Reset				(reset			),	//Input. 
		.ClockEn			(~reset			),	//Input. 
		.Result				(entry_size		))	//Output. 
	;
	
	//COUNTERS AND COMPARATORS 
	wire whole_hour;
	up_counter_12 		SECONDS_COUNTER		(	//12-bit counter of seconds since reset
		.Aclr 				(whole_hour		),	//Input. Asynchronous signal to clear the counter
		.Clock				(entry_clock	),	//Input. Clock to increment the counter
		.Clk_En 			(new_second		),	//Input. Signal to enable the counter's clock
		.Q					(whole_seconds	))	//Output. 12-bit output with the present count value
	;
	com_ge_12_reg		SECONDS_COM			(	//Comparator to indicate when SECONDS_COUNTER reaches 1 hour
		.DataA				(whole_seconds	),	//Input. 12-bit count output from SECONDS_COUNTER
		.DataB				(12'd3600		),	//Input. The number of seconds in one hour
		.Clock				(entry_clock	),	//Input. Clock to update the output register
		.ClockEn			(~reset			),	//Input. Clock enable
		.Aclr				(reset			),	//Input. Asynchronous signal to clear the output register
		.AGEB				(whole_hour		))	//Output. Returns SECONDS_COUNTER value to 0
	;
	
	up_counter_27 		SUB_SECONDS_COUNTER	(	//27-bit GNSS time counter for making and calibrating timestamps
		.Aclr 				(new_second		),	//Input. Asynchronous signal to clear the counter
		.Clock				(entry_clock	),	//Input. Clock to increment the counter
		.Clk_En 			(~reset			),	//Input. Signal to enable the counter's clock
		.Q					(sub_seconds	))	//Output. 27-bit output with the present count value
	;
	com_ge_8_reg		SUB_SECONDS_COM			(	//Comparator to indicate when SUB_SECONDS_COUNTER will soon overflow
		.DataA				(sub_seconds[26:19]	),	//Input. The 8 MSBs of the count output from SUB_SECONDS_COUNTER
		.DataB				(8'd255				),	//Input. 2^8 - 1, expected after ~1.337 seconds
		.Clock				(entry_clock		),	//Input. Clock to update the output register
		.ClockEn			(~reset				),	//Input. Clock enable
		.Aclr				(t_pulse			),	//Input. Asynchronous signal to clear the output register
		.AGEB				(subsecs_oFlow		))	//Output. Overflow signal, gets recorded and causes assertion of new_second
	;
	
	assign new_seq_entry	= A_saved | B_saved;
	up_counter_8		SEQ_ENTRY_COUNTER	(	//8-bit counter for the number of entries that have been created by sequences
		.Aclr				(new_second		),	//Input. Asynchronous signal to clear the counter
		.Clock				(entry_clock	),	//Input. Clock to increment the counter
		.Clk_En 			(new_seq_entry	),	//Input. Enables the counter's clock
		.Q					(seq_entries	))	//Output. 8-bit output with the present count value
	;
	com_ge_8_reg		SEC_ENTRY_COM		(	//8-bit comparator to indicate if there have been "seq_limit" many entries this second
		.DataA				(seq_entries	),	//Input. 8-bit count value from SEQ_ENTRY_COUNTER
		.DataB				(seq_limit		),	//Input. A limit on the number of sequence-induced entries created each second
		.Clock				(entry_clock	),	//Input. Clock to update the output register
		.ClockEn			(~reset			),	//Input. Clock enable
		.Aclr				(new_second		),	//Input. Asynchronous signal to clear the output register
		.AGEB				(seq_limit_flag	))	//Output. Overflow signal, gets recorded and prevents the capture of sequences until next second
	;
	
	
	//STATE MACHINE TO PREPARE ENTRY DATA FOR MULTIPLEXING BY THE STATE MACHINE BELOW THIS ONE
	reg	[1:0]	prep_mux_sm; /* synthesis syn_encoding = "sequential" */
	parameter	PREP_MUX_IDLE	= 2'b00;
	parameter	DONE_META_READ	= 2'b01;
	parameter	READING_A		= 2'b10;
	parameter	READING_B		= 2'b11;
	
	reg			Rd_count_Aclr, meta_RdEn, meta_ready, sample_RdEn_buf, meta_mux_done;
	reg	[111:0]	meta_Q_buf;
	reg	[7:0]	entry_sample;
	
	always @(posedge system_clock or posedge reset) begin
		if (reset) begin
			Rd_count_Aclr	<= #1 1'd0;
			meta_RdEn		<= #1 1'b0;
			A_RdEn			<= #1 1'b0;
			B_RdEn			<= #1 1'b0;
			meta_ready		<= #1 1'b0;
			prep_mux_sm 	<= #1 PREP_MUX_IDLE;
			meta_Q_buf		<= #1 112'd0;
			entry_sample	<= #1 8'd0;
			sample_RdEn_buf	<= #1 1'd0;
		end
		else begin
			case (prep_mux_sm)
				PREP_MUX_IDLE : begin
					Rd_count_Aclr	<= #1 1'b1;
					meta_RdEn		<= #1 system_ready & ~meta_Empty;
					meta_ready		<= #1 1'b0;
					A_RdEn			<= #1 1'b0;
					B_RdEn			<= #1 1'b0;
					if (system_ready & ~meta_Empty)	prep_mux_sm <= #1 DONE_META_READ;
					else							prep_mux_sm <= #1 PREP_MUX_IDLE;
				end
				DONE_META_READ : begin
					Rd_count_Aclr	<= #1 1'b0;
					meta_RdEn		<= #1 1'b0;
					meta_ready		<= #1 1'b1;
					A_RdEn			<= #1 1'b0;
					B_RdEn			<= #1 1'b0;
					if (~meta_mux_done)		prep_mux_sm <= #1 DONE_META_READ;
					else if (~A_Rd_done)	prep_mux_sm <= #1 READING_A;
					else if (~B_Rd_done)	prep_mux_sm <= #1 READING_B;
					else					prep_mux_sm <= #1 PREP_MUX_IDLE;
				end
				READING_A : begin
					Rd_count_Aclr	<= #1 1'b0;
					meta_RdEn		<= #1 1'b0;
					meta_ready		<= #1 1'b1;
					A_RdEn			<= #1 1'b1;
					B_RdEn			<= #1 1'b0;
					if (~A_Rd_done)			prep_mux_sm <= #1 READING_A;
					else if (~B_Rd_done)	prep_mux_sm <= #1 READING_B;
					else					prep_mux_sm <= #1 PREP_MUX_IDLE;
				end
				READING_B : begin
					Rd_count_Aclr	<= #1 1'b0;
					meta_RdEn		<= #1 1'b0;
					meta_ready		<= #1 1'b1;
					A_RdEn			<= #1 1'b0;
					B_RdEn			<= #1 1'b1;
					if (~B_Rd_done)	prep_mux_sm <= #1 READING_B;
					else			prep_mux_sm <= #1 PREP_MUX_IDLE;
				end
				default : begin
					Rd_count_Aclr	<= #1 1'b1;
					meta_RdEn		<= #1 1'b0;
					meta_ready		<= #1 1'b0;
					A_RdEn			<= #1 1'b0;
					B_RdEn			<= #1 1'b0;
					prep_mux_sm 	<= #1 PREP_MUX_IDLE;
				end
			endcase
			if (meta_RdEn)	meta_Q_buf	<= #1 meta_Q;
			else 			meta_Q_buf	<= #1 meta_Q_buf;
			if (B_RdEn)	entry_sample <= #1 B_Q;
			else 		entry_sample <= #1 A_Q;
			sample_RdEn_buf	<= #1	A_RdEn | B_RdEn;
		end
	end
		
	metadata_fifo		METADATA_FIFO_INST	(	//LUT-based First-In First-Out (FIFO) memory instance of size 16 x 112
		.Reset				(reset			),	//Input. Reset signal
		.RPReset			(reset			),	//Input. Read pointer reset signal
		.WrClock			(entry_clock	),	//Input. Write port clock
		.RdClock			(system_clock	),	//Input. Read port clock
		.WrEn				(meta_WrEn		),	//Input. Write port enable
		.Data				(meta_Data		),	//Input. 112-bit write data word (a new entry meta)
		.RdEn				(meta_RdEn		),	//Input. Read enable
		.Q					(meta_Q			),	//Output. 112-bit read data word (the oldest entry meta in memory)
		.Empty				(meta_Empty		),	//Output. Indicates 0 of 16 words are stored in memory
		.Full				(meta_Full		),	//Output. Indicates 16 of 16 words are stored in memory
		.AlmostEmpty		(meta_AmEmpty	),	//Output. Indicates at most 5 of 16 words are stored in memory
		.AlmostFull			(meta_AmFull	))	//Output. Indicates at least 11 of 16 words are stored in memory
	;
	
	up_cntr_6_lower_1	A_RD_COUNTER		(	//6-bit counter that counts up from 1 (not 0)
		.Aclr 				(Rd_count_Aclr	),	//Input. Asynchronous signal to clear the counter
		.Clock				(system_clock	),	//Input. Clock to increment the counter
		.Clk_En 			(A_RdEn			),	//Input. Reading samples from A's channel_fifo enables the counter's clock
		.Q					(A_RdEn_count	))	//Output. 6-bit output with the present count value
	;
	com_ge_6_noreg		A_RD_COUNT_COM			(	//6-bit comparator to indicate if 
		.DataA				(A_RdEn_count		),	//Input. 6-bit count value from A_RD_COUNTER
		.DataB				(meta_Q_buf[29:24]	),	//Input. 
		.AGEB				(A_Rd_done			))	//Output. Indicates A_RdEn_count >= the sequence duration
	;
	
	up_cntr_6_lower_1	B_RD_COUNTER		(	//6-bit counter that counts up from 1 (not 0)
		.Aclr 				(Rd_count_Aclr	),	//Input. Asynchronous signal to clear the counter
		.Clock				(system_clock	),	//Input. Clock to increment the counter
		.Clk_En 			(B_RdEn			),	//Input. Signal to enable the counter's clock
		.Q					(B_RdEn_count	))	//Output. 6-bit output with the present count value
	;
	com_ge_6_noreg		B_RD_COUNT_COM			(	//6-bit comparator to 
		.DataA				(B_RdEn_count		),	//Input. 6-bit count value from A_RD_COUNTER
		.DataB				(meta_Q_buf[5:0]	),	//Input. 
		.AGEB				(B_Rd_done			))	//Output. Indicates B_RdEn_count >= the sequence duration
	;
	
	
	//STATE MACHINE TO MULTIPLEX BETWEEN SOURCES OF DATA TO WRITE TO THE ENTRY_FIFO
	reg	[15:0]	entry_Wr_mux_sm; /* synthesis syn_encoding = "onehot" */
	parameter	MUX_ENTRY_SIZE	= 16'b0000000000000001; //The entry header's most-significant byte
	parameter	MUX_HEADER_7	= 16'b0000000000000010; //The entry header's 2nd-most-significant byte
	parameter	MUX_HEADER_6	= 16'b0000000000000100; //The entry header's 3rd-most-significant byte
	parameter	MUX_HEADER_5	= 16'b0000000000001000; //The entry header's 4th-most-significant byte
	parameter	MUX_HEADER_4	= 16'b0000000000010000; //The entry header's 5th-most-significant byte
	parameter	MUX_HEADER_3	= 16'b0000000000100000; //The entry header's 6th-most-significant byte
	parameter	MUX_HEADER_2	= 16'b0000000001000000; //The entry header's 7th-most-significant byte
	parameter	MUX_HEADER_1	= 16'b0000000010000000; //The entry header's 8th-most-significant byte
	parameter	MUX_A_INFO_3	= 16'b0000000100000000; //The entry header's A_seq_sum[15:8] byte
	parameter	MUX_A_INFO_2	= 16'b0000001000000000; //The entry header's A_seq_sum[7:0] byte
	parameter	MUX_A_INFO_1	= 16'b0000010000000000; //The entry header's A_seq_dur_8 byte
	parameter	MUX_B_INFO_3	= 16'b0000100000000000; //The entry header's B_seq_sum[15:8] byte
	parameter	MUX_B_INFO_2	= 16'b0001000000000000; //The entry header's B_seq_sum[7:0] byte
	parameter	MUX_B_INFO_1	= 16'b0010000000000000; //The entry header's B_seq_dur_8 byte
	parameter	MUX_SAMPLE		= 16'b0100000000000000; //Mux the sample selected by prep_mux_sm above
	parameter	MUX_DONE		= 16'b1000000000000000; //Multiplexing is complete
	
	reg			next_entry_WrEn;
	reg	[7:0]	next_entry_Data;
	
	always @(posedge system_clock or posedge reset) begin
		if (reset) begin
			next_entry_WrEn	<= #1 1'b0;
			next_entry_Data	<= #1 8'd0;
			meta_mux_done	<= #1 1'b0;
			entry_Wr_mux_sm	<= #1 MUX_ENTRY_SIZE;
			entry_WrEn		<= #1 1'd0;
			entry_Data		<= #1 8'd0;
		end
		else begin
			case (entry_Wr_mux_sm)
				MUX_ENTRY_SIZE : begin
					next_entry_Data	<= #1 meta_Q_buf[111:104];
					meta_mux_done	<= #1 1'b0;
					if (meta_ready & ~entry_AmFull) begin
						next_entry_WrEn	<= #1 1'b1;
						entry_Wr_mux_sm	<= #1 MUX_HEADER_7;
					end
					else begin
						next_entry_WrEn	<= #1 1'b0;
						entry_Wr_mux_sm	<= #1 MUX_ENTRY_SIZE;
					end
				end
				MUX_HEADER_7 : begin
					next_entry_WrEn	<= #1 1'b1;
					next_entry_Data	<= #1 meta_Q_buf[103:96];
					meta_mux_done	<= #1 1'b0;
					entry_Wr_mux_sm	<= #1 MUX_HEADER_6;
				end
				MUX_HEADER_6 : begin
					next_entry_WrEn	<= #1 1'b1;
					next_entry_Data	<= #1 meta_Q_buf[95:88];
					meta_mux_done	<= #1 1'b0;
					entry_Wr_mux_sm	<= #1 MUX_HEADER_5;
				end
				MUX_HEADER_5 : begin
					next_entry_WrEn	<= #1 1'b1;
					next_entry_Data	<= #1 meta_Q_buf[87:80];
					meta_mux_done	<= #1 1'b0;
					entry_Wr_mux_sm	<= #1 MUX_HEADER_4;
				end
				MUX_HEADER_4 : begin
					next_entry_WrEn	<= #1 1'b1;
					next_entry_Data	<= #1 meta_Q_buf[79:72];
					meta_mux_done	<= #1 1'b0;
					entry_Wr_mux_sm	<= #1 MUX_HEADER_3;
				end
				MUX_HEADER_3 : begin
					next_entry_WrEn	<= #1 1'b1;
					next_entry_Data	<= #1 meta_Q_buf[71:64];
					meta_mux_done	<= #1 1'b0;
					entry_Wr_mux_sm	<= #1 MUX_HEADER_2;
				end
				MUX_HEADER_2 : begin
					next_entry_WrEn	<= #1 1'b1;
					next_entry_Data	<= #1 meta_Q_buf[63:56];
					meta_mux_done	<= #1 1'b0;
					entry_Wr_mux_sm	<= #1 MUX_HEADER_1;
				end
				MUX_HEADER_1 : begin
					next_entry_WrEn	<= #1 1'b1;
					next_entry_Data	<= #1 meta_Q_buf[55:48];
					if (meta_Q_buf[61]) begin	//This means A_saved was true
						meta_mux_done	<= #1 1'b0;
						entry_Wr_mux_sm	<= #1 MUX_A_INFO_3;
					end
					else if (meta_Q_buf[59]) begin	//This means B_saved was true
						meta_mux_done	<= #1 1'b0;
						entry_Wr_mux_sm	<= #1 MUX_B_INFO_3;
					end
					else begin	//This means neither A_saved nor B_saved were true
						meta_mux_done	<= #1 1'b1;	//Note: meta_mux_done gets asserted
						entry_Wr_mux_sm	<= #1 MUX_DONE;
					end
				end
				MUX_A_INFO_3 : begin
					next_entry_WrEn	<= #1 1'b1;
					next_entry_Data	<= #1 meta_Q_buf[47:40];
					meta_mux_done	<= #1 1'b0;
					entry_Wr_mux_sm	<= #1 MUX_A_INFO_2;
				end
				MUX_A_INFO_2 : begin
					next_entry_WrEn	<= #1 1'b1;
					next_entry_Data	<= #1 meta_Q_buf[39:32];
					meta_mux_done	<= #1 1'b0;
					entry_Wr_mux_sm	<= #1 MUX_A_INFO_1;
				end
				MUX_A_INFO_1 : begin
					next_entry_WrEn	<= #1 1'b1;
					next_entry_Data	<= #1 meta_Q_buf[31:24];
					if (meta_Q_buf[59]) begin	//This means B_saved was true
						meta_mux_done	<= #1 1'b0;
						entry_Wr_mux_sm	<= #1 MUX_B_INFO_3;
					end
					else begin	//This means B_saved was not true
						meta_mux_done	<= #1 1'b1;	//Note: meta_mux_done gets asserted
						entry_Wr_mux_sm	<= #1 MUX_SAMPLE;
					end
				end
				MUX_B_INFO_3 : begin
					next_entry_WrEn	<= #1 1'b1;
					next_entry_Data	<= #1 meta_Q_buf[23:16];
					meta_mux_done	<= #1 1'b0;
					entry_Wr_mux_sm	<= #1 MUX_B_INFO_2;
				end
				MUX_B_INFO_2 : begin
					next_entry_WrEn	<= #1 1'b1;
					next_entry_Data	<= #1 meta_Q_buf[15:8];
					meta_mux_done	<= #1 1'b0;
					entry_Wr_mux_sm	<= #1 MUX_B_INFO_1;
				end
				MUX_B_INFO_1 : begin
					next_entry_WrEn	<= #1 1'b1;
					next_entry_Data	<= #1 meta_Q_buf[7:0];
					meta_mux_done	<= #1 1'b1;	//Note: meta_mux_done gets asserted
					entry_Wr_mux_sm	<= #1 MUX_SAMPLE;
				end
				MUX_SAMPLE : begin
					next_entry_WrEn	<= #1 sample_RdEn_buf;
					next_entry_Data	<= #1 entry_sample;
					meta_mux_done	<= #1 1'b1;
					if (A_Rd_done & B_Rd_done)	entry_Wr_mux_sm	<= #1 MUX_DONE;
					else						entry_Wr_mux_sm	<= #1 MUX_SAMPLE;
				end
				MUX_DONE : begin
					next_entry_WrEn	<= #1 1'b0;
					next_entry_Data	<= #1 meta_Q_buf[111:104];
					meta_mux_done	<= #1 1'b0;
					entry_Wr_mux_sm	<= #1 MUX_ENTRY_SIZE;
				end
			endcase
			entry_WrEn	<= #1	next_entry_WrEn;
			entry_Data	<= #1	next_entry_Data;
			
		end
	end
	
	
endmodule
