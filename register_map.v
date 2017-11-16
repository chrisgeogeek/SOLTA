
//This module contains registers to store system settings and status indicators. It is controlled by the spi_slave 
//module, which can read and write the registers here according to instructions it receives over the SPI interface.

`timescale 1ns/ 1ps

module				register_map			(
	input				reset				,
	input				t_pulse				,	//TIMEPULSE signal from the GNSS receiver
	input				system_clock		,	//System clock
	input		[25:0]	top_status			,	//Status information sent by other modules
	output	reg			enable_ACLK			,	//Enable the PLL's ACLK clock output, serving as sample clock to the ADC
	output	reg			enable_auto			,	//Enable the automatic creation of new data entries
	output	reg			enable_saving		,	//Enable examination of samples & acquisition of sequences
	output	reg	[7:0]	seq_limit			,	//A limit on the number of sequence-induced entries created each second
	output	reg	[28:0]	A_criteria			,	//Channel A data collection criteria
	output	reg	[28:0]	B_criteria			,	//Channel B data collection criteria
	output	reg			update_A			,	//Signal to make A_CHANNEL_MONITOR update its criteria
	output	reg			update_B			,	//Signal to make B_CHANNEL_MONITOR update its criteria
	
	//Signals for handling commands
	input				write_byte			,	//Instruction to write a byte in the register map
	input		[7:0]	byte_to_write		,	//The byte of data that is to be written there
	output	reg			done_write_byte		,	//Indication from the register map that the write is complete
	input		[3:0]	address				,	//Byte address in the register map, either for reading or writing
	input				read_byte			,	//Instruction to read a byte in the register map
	output	reg	[7:0]	read_result			,	//The byte read from the address in the register map
	output	reg			done_read_byte		)	//Indication from the register map that the read is complete
	;
	
	
	//STATE MACHINE TO DECODE THE address SENT BY SPI_SLAVE AND ASSIGN write_target AND read_target
	reg	[3:0]	address_sm; /* synthesis syn_encoding = "sequential" */
	parameter	BYTE_1	= 4'h0;
	parameter	BYTE_2	= 4'h1;
	parameter	BYTE_3	= 4'h2;
	parameter	BYTE_4	= 4'h3;
	parameter	BYTE_5	= 4'h4;
	parameter	BYTE_6	= 4'h5;
	parameter	BYTE_7	= 4'h6;
	parameter	BYTE_8	= 4'h7;
	parameter	BYTE_9	= 4'h8;
	parameter	BYTE_10	= 4'h9;
	parameter	BYTE_11	= 4'hA;
	parameter	BYTE_12	= 4'hB;
	parameter	BYTE_13	= 4'hC;
	parameter	BYTE_14	= 4'hD;
	parameter	BYTE_15	= 4'hE;
	
	wire	[7:0]	LSBs_count;		//"Least significant bits" count. Just here for synthesis reasons
	reg		[9:0]	write_target;	//A "one-hot" pointer for which byte will be overwritten
	reg		[7:0]	read_target;	//For holding data that will be assigned to read_result
	reg		[39:0]	status_buf;		//For concatenating status information
	reg				new_A_crit, new_B_crit;		//For indicating a change in the Channel A or Channel B criteria
	reg		[27:0]	A_crit_regs, B_crit_regs;	//Registers for Channel A and Channel B criteria
	reg		[4:0]	misc_settings;	//Source for {enable_ACLK, enable_auto, enable_saving, A req_coinc, B req_coinc}
	reg		[7:0]	seq_limit_reg;	//A limit on the number of permitted sequence enteries created per second
	
	always @(posedge system_clock or posedge reset) begin
		if (reset) begin
			address_sm		<= #1 BYTE_1;
			write_target	<= #1 10'd0;
			read_target		<= #1 8'd0;
			status_buf		<= #1 40'd0;
		end
		else begin
			address_sm	<= address;
			case (address_sm)
				BYTE_1	: begin
					write_target	<= #1 10'b0000000001;
					read_target		<= #1 B_crit_regs[7:0];
				end
				BYTE_2	: begin
					write_target	<= #1 10'b0000000010;
					read_target		<= #1 B_crit_regs[15:8];
				end
				BYTE_3	: begin
					write_target	<= #1 10'b0000000100;
					read_target		<= #1 {2'b00, B_crit_regs[21:16]};
				end
				BYTE_4	: begin
					write_target	<= #1 10'b0000001000;
					read_target		<= #1 {2'b00, B_crit_regs[27:22]};
				end
				BYTE_5	: begin
					write_target	<= #1 10'b0000010000;
					read_target		<= #1 A_crit_regs[7:0];
				end
				BYTE_6	: begin
					write_target	<= #1 10'b0000100000;
					read_target		<= #1 A_crit_regs[15:8];
				end
				BYTE_7	: begin
					write_target	<= #1 10'b0001000000;
					read_target		<= #1 {2'b00, A_crit_regs[21:16]};
				end
				BYTE_8	: begin
					write_target	<= #1 10'b0010000000;
					read_target		<= #1 {2'b00, A_crit_regs[27:22]};
				end
				BYTE_9	: begin
					write_target	<= #1 10'b0100000000;
					read_target		<= #1 {3'b000, misc_settings};
				end
				BYTE_10 : begin
					write_target	<= #1 10'b1000000000;
					read_target		<= #1 seq_limit_reg;
				end
				BYTE_11	: begin
					write_target	<= #1 10'b0000000001;
					read_target		<= #1 status_buf[39:32];
				end
				BYTE_12	: begin
					write_target	<= #1 10'b0000000001;
					read_target		<= #1 status_buf[31:24];
				end
				BYTE_13	: begin
					write_target	<= #1 10'b0000000001;
					read_target		<= #1 status_buf[23:16];
				end
				BYTE_14	: begin
					write_target	<= #1 10'b0000000001;
					read_target		<= #1 status_buf[15:8];
				end
				BYTE_15	: begin
					write_target	<= #1 10'b0000000001;
					read_target		<= #1 status_buf[7:0];
				end
				default	: begin
					write_target	<= #1 10'b0000000001;
					read_target		<= #1 8'd0;
				end
			endcase
			
			status_buf[39:32]	<= #1 LSBs_count;
			status_buf[31:24]	<= #1 {5'b00000, t_pulse, top_status[25:24]}; //5'+1'+2'=8'
			status_buf[23:16]	<= #1 top_status[23:16];
			status_buf[15:8]	<= #1 top_status[15:8];
			status_buf[7:0]		<= #1 top_status[7:0];
		end
	end
	
	
	//STATE MACHINE TO RESPOND TO write_byte COMMANDS
	reg	[1:0]	write_byte_sm; /* synthesis syn_encoding = "sequential" */
	parameter	WRITE_IDLE	= 2'b00; //Don't overwrite any byte of the settings
	parameter	WRITE_START	= 2'b01; //It's time to start a write operation
	parameter	WRITE_BEGUN	= 2'b10; //The write operation has begun
	parameter	WRITE_DONE	= 2'b11; //The write operation is complete
	
	reg			write_byte_buf;
	reg	[9:0]	set_byte;
	
	always @(posedge system_clock or posedge reset) begin
		if (reset) begin
			write_byte_buf	<= #1 1'b0;
			set_byte		<= #1 10'd0;
			done_write_byte	<= #1 1'b0;
			write_byte_sm	<= #1 WRITE_IDLE;
			
			new_B_crit		<= #1 1'b0;
			new_A_crit		<= #1 1'b0;
			update_A		<= #1 1'b0;
			update_B		<= #1 1'b0;
			
			//The following default settings must be in matched pairs (having equal values)
			seq_limit			<= #1 8'd64;
			seq_limit_reg		<= #1 8'd64;
			//Defaults for			{enable_ACLK,	enable_auto,	enable_saving,	A req_coinc,	B req_coinc}:
			misc_settings		<= #1  {1'b0,		1'b0,			1'b0,			1'b0,			1'b0};
			enable_ACLK			<= #1	1'b0;
			enable_auto			<= #1				1'b0;
			enable_saving		<= #1								1'b0;
			A_criteria[28]		<= #1												1'b0;
			B_criteria[28]		<= #1																1'b0;
			//Channel A defaults for  {min_dur,	max_dur,	min_amp,	DOR_limit}:
			A_criteria[27:0]	<= #1 {6'd61,	6'd62,		8'd255,		8'd127	};
			A_crit_regs			<= #1 {6'd61, 	6'd62,		8'd255,		8'd127	};
			//Channel B defaults for  {min_dur,	max_dur,	min_amp,	DOR_limit}:
			B_criteria[27:0]	<= #1 {6'd61, 	6'd62,		8'd255,		8'd127	};
			B_crit_regs			<= #1 {6'd61, 	6'd62,		8'd255,		8'd127	};
		end
		else begin
			write_byte_buf	<= write_byte;
			case (write_byte_sm) //State machine to decode the command and assign set_byte
				WRITE_IDLE : begin
					set_byte 		<= #1 10'b0000000000;
					done_write_byte	<= #1 1'b0;
					if (write_byte_buf) write_byte_sm	<= #1 WRITE_START;
					else				write_byte_sm	<= #1 WRITE_IDLE;
				end
				WRITE_START : begin
					set_byte 		<= #1 write_target;	//Load write_target into set_byte
					done_write_byte	<= #1 1'b0;
					write_byte_sm	<= #1 WRITE_BEGUN;
				end
				WRITE_BEGUN : begin
					set_byte 		<= #1 set_byte;		//Hold constant while set_byte takes effect
					done_write_byte	<= #1 1'b0;
					write_byte_sm	<= #1 WRITE_DONE;
				end
				WRITE_DONE : begin
					set_byte 		<= #1 10'b0000000000;	//Deassert whichever bit was high
					done_write_byte	<= #1 1'b1;		//Assert
					if (write_byte_buf) write_byte_sm	<= #1 WRITE_DONE;
					else				write_byte_sm	<= #1 WRITE_IDLE;
				end
				default : begin	//Here in case of error
					set_byte 		<= #1 10'b0000000000;
					done_write_byte	<= #1 1'b0;
					write_byte_sm	<= #1 WRITE_IDLE;
				end
			endcase
			
			new_B_crit		<= #1 set_byte[0] | set_byte[1] | set_byte[2] | set_byte[3] | set_byte[8];
			new_A_crit		<= #1 set_byte[4] | set_byte[5] | set_byte[6] | set_byte[7] | set_byte[8];
			B_crit_regs[7:0]	<= #1 set_byte[0] ? byte_to_write		: B_crit_regs[7:0];		//Channel B DOR_limit
			B_crit_regs[15:8]	<= #1 set_byte[1] ? byte_to_write		: B_crit_regs[15:8];	//Channel B min_amp
			B_crit_regs[21:16]	<= #1 set_byte[2] ? byte_to_write[5:0]	: B_crit_regs[21:16];	//Channel B max_dur
			B_crit_regs[27:22]	<= #1 set_byte[3] ? byte_to_write[5:0]	: B_crit_regs[27:22];	//Channel B min_dur
			A_crit_regs[7:0]	<= #1 set_byte[4] ? byte_to_write 		: A_crit_regs[7:0];		//Channel A DOR_limit
			A_crit_regs[15:8]	<= #1 set_byte[5] ? byte_to_write		: A_crit_regs[15:8];	//Channel A min_amp
			A_crit_regs[21:16]	<= #1 set_byte[6] ? byte_to_write[5:0]	: A_crit_regs[21:16];	//Channel A max_dur
			A_crit_regs[27:22]	<= #1 set_byte[7] ? byte_to_write[5:0]	: A_crit_regs[27:22];	//Channel A min_dur
			misc_settings		<= #1 set_byte[8] ? byte_to_write[4:0]	: misc_settings;	//Miscellaneous control bits
			seq_limit_reg		<= #1 set_byte[9] ? byte_to_write		: seq_limit_reg;	//Limit on sequence entries
			
			seq_limit		<= #1 seq_limit_reg;	//Pipeline registers
			enable_ACLK		<= #1 misc_settings[4];	//Pipeline registers
			enable_auto		<= #1 misc_settings[3];	//Pipeline registers
			enable_saving	<= #1 misc_settings[2];	//Pipeline registers
			A_criteria		<= #1 {misc_settings[1], A_crit_regs};	//Pipeline registers
			B_criteria		<= #1 {misc_settings[0], B_crit_regs};	//Pipeline registers
			update_A		<= #1 (new_A_crit | update_A) & ~top_status[17];	//Stays asserted until A_updated is true
			update_B		<= #1 (new_B_crit | update_B) & ~top_status[16];	//Stays asserted until B_updated is true
			
		end
	end
	
	
	//STATE MACHINE TO RESPOND TO read_byte COMMANDS OR THE ASSERTION OF done_write_byte
	reg	[1:0]	read_byte_sm; /* synthesis syn_encoding = "sequential" */
	parameter	READ_IDLE	= 2'b00; //Don't overwrite any byte of the settings
	parameter	READ_START	= 2'b01; //It's time to start a write operation
	parameter	READ_DONE	= 2'b10; //The write operation is complete
	
	reg			read_byte_buf;
	
	always @(posedge system_clock or posedge reset) begin
		if (reset) begin
			read_byte_buf	<= #1 1'b0;
			read_result		<= #1 8'd0;
			done_read_byte	<= #1 1'b0;
			read_byte_sm	<= #1 READ_IDLE;
		end
		else begin
			read_byte_buf	<= read_byte;
			case (read_byte_sm) //State machine to decode the command and assign read_byte
				READ_IDLE : begin
					read_result		<= #1 8'd0;
					done_read_byte	<= #1 1'b0;
					if (read_byte_buf | done_write_byte) read_byte_sm	<= #1 READ_START;
					else								 read_byte_sm	<= #1 READ_IDLE;
				end
				READ_START : begin
					read_result		<= #1 read_target;	//Load read_target into read_byte
					done_read_byte	<= #1 1'b0;
					read_byte_sm	<= #1 READ_DONE;
				end
				READ_DONE : begin
					read_result		<= #1 read_result;	//Hold constant
					done_read_byte	<= #1 1'b1;			//Assert
					if (read_byte_buf)	read_byte_sm	<= #1 READ_DONE;
					else				read_byte_sm	<= #1 READ_IDLE;
				end
				default : begin	//Here in case of error
					read_result		<= #1 8'd0;
					done_read_byte	<= #1 1'b0;
					read_byte_sm	<= #1 READ_IDLE;
				end
			endcase
			
		
		end
	end
		
	
	//CODE DEALING THE THE LSBs SIGNAL (see entry_maker for more information)
	reg				LSBs_flag_buf, LSBs_posedge;
	
	always @(posedge system_clock or posedge reset) begin
		if (reset) begin
			LSBs_flag_buf		<= #1 1'b0;
			LSBs_posedge		<= #1 1'b0;
		end
		else begin
			LSBs_flag_buf		<= #1 top_status[19];
			LSBs_posedge		<= #1 top_status[19] & ~LSBs_flag_buf;
		end
	end
	
	up_counter_8	LSBs_POSEDGE_COUNTER	(	//
		.Aclr 			(LSBs_Aclr			),	//Input. Clear the counter
		.Clock			(system_clock		),	//Input. Clock to increment the counter
		.Clk_En 		(LSBs_posedge		),	//Input. Clock enable
		.Q				(LSBs_count			))	//Output. 8-bit output with the present count value
	;
	com_ge_8_reg	LSBs_COUNTER_COM		(	//Comparator indicating that the above counter needs a reset
		.DataA			(LSBs_count			),	//Input. Counter output
		.DataB			(8'd255				),	//Input. Counter's max value
		.Clock			(system_clock		),	//Input. 
		.ClockEn		(~reset				),	//Input. 
		.Aclr			(reset				),	//Input. This clears the comparator's output register
		.AGEB			(LSBs_count_maxed	))	//Output. The output register
	;
	assign	LSBs_Aclr	= reset | LSBs_count_maxed;
	
endmodule



// ******************************* NO LONGER USED OR NO LONGER APPLICABLE: *******************************

	//input				READ_STATUS			,	//
	//output	reg	[7:0]	status_byte			,	//
	//output	reg			done_read_status	)	//
	
	
	// always @(posedge system_clock or posedge reset) begin
		// if (reset) write_target	<= 10'd0;
		// else case (address)
			// BYTE_1	: write_target <= 10'b0000000001;
			// BYTE_2	: write_target <= 10'b0000000010;
			// BYTE_3	: write_target <= 10'b0000000100;
			// BYTE_4	: write_target <= 10'b0000001000;
			// BYTE_5	: write_target <= 10'b0000010000;
			// BYTE_6	: write_target <= 10'b0000100000;
			// BYTE_7	: write_target <= 10'b0001000000;
			// BYTE_8	: write_target <= 10'b0010000000;
			// BYTE_9	: write_target <= 10'b0100000000;
			// BYTE_10 : write_target <= 10'b1000000000;
			// default : write_target <= 10'b0000000001;	//Here in case of error
		// endcase
	// end
	
	//======================================= Code To Resolve write_byte Commands =========================================
	//*********************************************************************************************************************
	
	// //STATE MACHINE TO DECODE write_byte COMMANDS
	// reg	[10:0]	write_byte_sm; /* synthesis syn_encoding = "onehot" */
	// parameter	WRITE_IDLE	= 11'b00000000001; //Don't overwrite any byte of the settings
	// parameter	WRITE_01	= 11'b00000000010; //Overwrite byte 1 of the settings
	// parameter	WRITE_02	= 11'b00000000100; //Overwrite byte 1 of the settings
	// parameter	WRITE_03	= 11'b00000001000; //Overwrite byte 1 of the settings
	// parameter	WRITE_04	= 11'b00000010000; //Overwrite byte 1 of the settings
	// parameter	WRITE_05	= 11'b00000100000; //Overwrite byte 1 of the settings
	// parameter	WRITE_06	= 11'b00001000000; //Overwrite byte 1 of the settings
	// parameter	WRITE_07	= 11'b00010000000; //Overwrite byte 1 of the settings
	// parameter	WRITE_08	= 11'b00100000000; //Overwrite byte 1 of the settings
	// parameter	WRITE_09	= 11'b01000000000; //Overwrite byte 1 of the settings
	// parameter	WRITE_10	= 11'b10000000000; //Overwrite byte 1 of the settings
	
			// status_buf[47:40]	<= #1 LSBs_count;
			// status_buf[39:32]	<= #1 hours_count[11:4];
			// status_buf[31:24]	<= #1 {hours_count[3:0], t_pulse, top_status[26:24]}; //4'+1'+3'=8'
	
	// up_counter_12	HOUR_COUNTER			(	//Counts the hours elapsed since the design (or this counter) was reset
		// .Aclr 			(hours_Aclr			),	//Input. Clear the counter
		// .Clock			(system_clock		),	//Input. Clock to increment the counter
		// .Clk_En 		(hour_posedge		),	//Input. Clock enable
		// .Q				(hours_count		))	//Output. 12-bit output with the present count value
	// ;
	// com_ge_12_reg	HOUR_COUNTER_COM		(	//Comparator indicating if HOUR_COUNTER should be cleared
		// .DataA			(hours_count		),	//Input. Timer output
		// .DataB			(12'd4081			),	//Input. There are 4080 hours in 170 days. 4095 Would give a partial day
		// .Clock			(system_clock		),	//Input. 
		// .ClockEn		(~reset				),	//Input. 
		// .Aclr			(reset				),	//Input. This clears the comparator's output register
		// .AGEB			(hours_count_maxed	))	//Output. The output register
	// ;
	// assign	hours_Aclr	= reset | hours_count_maxed;

	// reg		[47:0]	status_buf;
	// reg				start_READ_STATUS, status_byte_ready;
	
	// wire	[11:0]	hours_count;
	
	// always @(posedge system_clock or posedge reset) begin
		// if (reset) begin
			// status_buf			<= #1 48'd0;
			// start_READ_STATUS	<= #1 1'b0;
			// status_byte_ready	<= #1 1'b0;
			// status_byte			<= #1 8'd0;
			// done_read_status	<= #1 1'b0;
			
			// LSBs_flag_buf		<= #1 1'b0;
			// LSBs_posedge		<= #1 1'b0;
			// hour_reached_buf	<= #1 1'b0;
			// hour_posedge		<= #1 1'b0;
		// end
		// else begin
			
			// //======================================= Code To Resolve READ_STATUS Commands ========================================
			// //*********************************************************************************************************************
			// status_buf[39:32]	<= #1 LSBs_count;
			// status_buf[31:24]	<= #1 {4'b0000, t_pulse, top_status[26:24]}; //4'+1'+3'=8'
			// status_buf[23:16]	<= #1 top_status[23:16];
			// status_buf[15:8]	<= #1 top_status[15:8];
			// status_buf[7:0]		<= #1 top_status[7:0];
			
			// start_READ_STATUS	<= #1 READ_STATUS & ~(start_READ_STATUS | status_byte_ready | done_read_status);
			// case ({start_READ_STATUS, address}) //State machine to decode the command and assign status_byte
				// //{1'b1, 4'h6} : {status_byte_ready, status_byte} <= #1 {1'b1, status_buf[55:48]};
				// {1'b1, 4'h5} : {status_byte_ready, status_byte} <= #1 {1'b1, status_buf[47:40]};
				// {1'b1, 4'h4} : {status_byte_ready, status_byte} <= #1 {1'b1, status_buf[39:32]};
				// {1'b1, 4'h3} : {status_byte_ready, status_byte} <= #1 {1'b1, status_buf[31:24]};
				// {1'b1, 4'h2} : {status_byte_ready, status_byte} <= #1 {1'b1, status_buf[23:16]};
				// {1'b1, 4'h1} : {status_byte_ready, status_byte} <= #1 {1'b1, status_buf[15:8]};
				// {1'b1, 4'h0} : {status_byte_ready, status_byte} <= #1 {1'b1, status_buf[7:0]};
				// default		 : {status_byte_ready, status_byte} <= #1 {1'b0, status_byte};
			// endcase
			// done_read_status	<= #1 READ_STATUS & (status_byte_ready | done_read_status); //Stays asserted until READ_STATUS = 0
			// //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
			