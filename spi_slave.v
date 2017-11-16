
`timescale 1ns/ 1ps

module				spi_slave				(
	input				reset				,
	input				system_clock		,	//System clock
	input				spi_csn				,	//Hard SPI chip-select (active low)
	input				entry_Empty			,	//The Empty indicator from ENTRY_FIFO_INST
	output	reg			entry_RdEn			,	//Read enable for ENTRY_FIFO_INST
	input		[7:0]	entry_Q				,	//A byte read out from ENTRY_FIFO_INST
	output				make_entry			,	//Signal to manually have ENTRY_MAKER_INST create a new entry
	
	//Signals for delegating commands to the design's register_map module instance
	output				write_byte			,	//Instruction to write a byte in the register map
	output	reg	[7:0]	byte_to_write		,	//The byte of data that is to be written there
	input				done_write_byte		,	//Indication from the register map that the write is complete
	output	reg	[3:0]	address				,	//Byte address in the register map, either for reading or writing
	output				read_byte			,	//Instruction to read a byte in the register map
	input		[7:0]	read_result			,	//The byte read from the address in the register map
	input				done_read_byte		,	//Indication from the register map that the read is complete
	
	//WISHBONE bus signals
    output	reg			wb_cyc				,	//WISHBONE bus cycle (also used for the strobe signal in this design)
    output	reg			wb_wr_enable		,	//WISHBONE read/write control. 0: read, 1: write
    output	reg	[7:0]	wb_address			,	//WISHBONE address
    output	reg	[7:0]	wb_data_mosi		,	//WISHBONE data sent to the EFB
    input		[7:0]	wb_data_miso		,	//WISHBONE data sent from the EFB
    input				wb_ack				)	//WISHBONE transfer acknowledge
	;
	
	
	wire	[7:0]	wb_rst_tmr_Q;	//Wishbone reset timer/counter value
	
	up_counter_8	WB_RESET_TIMER			(	//To prevent new WB transactions for 1us after de-assertion of wb_reset (top level reset)
		.Aclr 			(reset				),	//Input. Clear the counter
		.Clock			(system_clock		),	//Input. Clock to increment the counter
		.Clk_En 		(wb_rst_tmr_Clk_En	),	//Input. Clock enable
		.Q				(wb_rst_tmr_Q		))	//Output. 8-bit output with the present count value
	;
	com_ge_8_reg	WB_RESET_TIMER_COM		(	//Comparator indicating if it's been 1us since deasserting of wb_reset (top level reset)
		.DataA			(wb_rst_tmr_Q		),	//Input. Timer output
		.DataB			(8'd110				),	//Input. 110 cycles gives a +10% margin for error, assuming a 100MHz system_clock
		.Clock			(system_clock		),	//Input. 
		.ClockEn		(wb_rst_tmr_Clk_En	),	//Input. 
		.Aclr			(reset				),	//Input. This clears the comparator's output register
		.AGEB			(wb_ready			))	//Output. The output register
	;
	assign	wb_rst_tmr_Clk_En = ~reset & ~wb_ready;	//Timer is disabled once comparator output is true (time reached)
	

	//STATE MACHINE ONE, TO ACT AS MASTER FOR THE WISHBONE BUS, READING AND WRITING REGISTERS IN THE EFB'S REGISTER MAP
	reg [1:0]	wb_sm;	/* synthesis syn_encoding = "sequential" */
	parameter	WB_IDLE		= 2'b00; //The state machine is idle
	parameter	WB_SET		= 2'b01; //The state machine is asserting signals to the WB bus for this cycle
	parameter	WB_BUSY		= 2'b10; //The state machine is waiting for the EFB to assert wb_ack, before ending this cycle
	
	reg [3:0]	wb_op_type;		//WB operation type in "one-hot" style: {write {cmnd_code, address}, write result_byte, read RXDR, or read SR}
	reg	[3:0]	cmnd_code;		//For storing the command code, the first 4 bits of the first byte received in an SPI transfer
	reg	[7:0]	result_byte;	//The result of/answer to the latest command, to transmit during a SPI transfer
	reg	[7:0]	RXDR_byte;		//The byte received most recently over an SPI transfer
	wire		lead_TXDR;		//1: Indicates a reply is prepared for transmission and its first byte should be sent
	wire		result_TXDR;	//1: Indicates a reply is prepared for transmission and its next byte should be sent
	reg			TXDR; 		//The EFB's "SPI Transmit Ready". 1: Indicates the SPI transmit data register (SPITXDR) is empty
	reg			RXDR;		//Its "SPI Receive Ready". 1: Indicates the receive data register (SPIRXDR) contains valid receive data
	reg			lead_done;		//1: Indicates lead_byte has been written to the EFB so it will be transmitted next
	reg			result_done;	//1: Indicates result_byte has been written to the EFB so it will be transmitted next
	reg			RXDR_done;		//1: Indicates RXDR_byte has been read from the EFB
	
	always @(posedge system_clock or posedge reset) begin
		if (reset) begin	//States/values upon powering up
			wb_cyc			<= 1'b0;
			wb_wr_enable	<= 1'b0;
			wb_data_mosi	<= 8'd0;
			wb_address		<= 8'd0;
			wb_op_type		<= 4'b0000;
			wb_sm			<= WB_IDLE;
			lead_done		<= 1'b0;
			result_done		<= 1'b0;
			RXDR_done		<= 1'b0;
			RXDR_byte		<= 8'd0;
			TXDR			<= 1'b0;
			RXDR			<= 1'b0;
		end
		else begin
			
			case (wb_sm)
				WB_IDLE : begin //The state machine is idle
					wb_cyc			<= 1'b0;
					wb_wr_enable	<= 1'b0;
					wb_data_mosi	<= 8'd0;
					wb_address		<= 8'h00;
					wb_op_type		<= 4'b0000;
					if (~wb_ready)	wb_sm <= WB_IDLE; //Wait after a bus reset (see WB_RESET_TIMER_COM),
					else			wb_sm <= WB_SET; //  or the WB bus is ready so go prepare for a cycle.
				end
				WB_SET : begin //The state machine is asserting signals to the WB bus for this cycle
					wb_cyc			<= 1'b1;
					wb_wr_enable	<= (lead_TXDR & spi_csn) | TXDR;
					if (lead_TXDR & spi_csn) begin 	//Write the first byte to SPITXDR because a reply is ready and no transfer is underway
						wb_data_mosi	<= {cmnd_code, address};
						wb_address		<= 8'h59;	//The SPITXDR register address
						wb_op_type		<= 4'b0001;
					end
					else if (result_TXDR & TXDR) begin 	//Write result_byte to SPITXDR because a reply is ready and the first byte has been sent
						wb_data_mosi	<= result_byte;
						wb_address		<= 8'h59;	//The SPITXDR register address
						wb_op_type		<= 4'b0010;
					end
					else if (TXDR) begin //Write 8'd0 to SPITXDR,
						wb_data_mosi	<= 8'd0;
						wb_address		<= 8'h59;	//The SPITXDR register address
						wb_op_type		<= 4'b0000;
					end
					else if (RXDR) begin //Read out SPIRXDR,
						wb_data_mosi	<= 8'd0;
						wb_address		<= 8'h5B;	//The SPIRXDR register address
						wb_op_type		<= 4'b0100;
					end
					else begin			//Read out SPISR.
						wb_data_mosi	<= 8'd0;
						wb_address		<= 8'h5A;	//The SPISR register address
						wb_op_type		<= 4'b1000;
					end
					wb_sm	<= WB_BUSY;
				end
				WB_BUSY : begin	//Waiting for the transaction to be acknowledged by the SPI slave
					if (~wb_ack) begin	//Continue waiting (hold control signals and wb_op_type constant)
						wb_cyc			<= 1'b1;
						wb_wr_enable	<= wb_wr_enable;
						wb_data_mosi	<= wb_data_mosi;
						wb_address		<= wb_address;
						wb_op_type		<= wb_op_type;
						wb_sm			<= WB_BUSY;
					end
					else begin			//Acknowledged, so reset WB control signals and wb_op_type
						wb_cyc			<= 1'b0;
						wb_wr_enable	<= 1'b0;
						wb_data_mosi	<= 8'd0;
						wb_address		<= 8'h00;
						wb_op_type		<= 4'b0000;
						wb_sm			<= WB_IDLE;
					end
				end
				default : begin	//Here in case of error, returns to initial state
					wb_cyc			<= 1'b0;
					wb_wr_enable	<= 1'b0;
					wb_data_mosi	<= 8'd0;
					wb_address		<= 8'h00;
					wb_op_type		<= 4'b0000;
					wb_sm			<= WB_IDLE;
				end
			endcase
			lead_done	<= ((wb_op_type[0] & wb_ack) | lead_done) & ~spi_csn;	//Once asserted it stays that way for rest of transfer
			result_done	<= wb_op_type[1] & wb_ack & ~result_done;	//The ~result_done ensures deassertion after single clock cycle
			RXDR_done	<= wb_op_type[2] & wb_ack & ~RXDR_done;		//The ~RXDR_done ensures deassertion after single clock cycle
			RXDR_byte	<= (wb_op_type[2] & wb_ack) ? wb_data_miso : RXDR_byte;	//Copies and holds the data from RXDR
			TXDR		<= (wb_op_type[3] & wb_ack) ? wb_data_miso[4] : 1'b0;	//[4] is the TRDY bit in SPISR
			RXDR		<= (wb_op_type[3] & wb_ack) ? wb_data_miso[3] : 1'b0;	//[3] is the RRDY bit in SPISR
		end
	end
	
	
	//STATE MACHINE TWO, FOR RECEIVING AND RESPONDING TO COMMANDS FROM THE SPI MASTER (THE RASPBERRY PI)
	reg [9:0]	cmnd_sm;	/* synthesis syn_encoding = "onehot" */
	parameter	RX_IDLE		= 10'b0000000001;	//The state machine is idle, checking that WB is ready and that the SPI bus is not in use
	parameter	RX_READY	= 10'b0000000010;	//Ready state, waiting for the first byte received in a new SPI transfer
	parameter	RX_BYTE_1	= 10'b0000000100;	//First byte received, waiting for second byte
	parameter	RX_BYTE_2	= 10'b0000001000;	//Second byte also received, so machine is decoding the command code
	parameter	CMND_READ	= 10'b0000010000;	//It was a "read_byte" command that was received
	parameter	CMND_WRITE	= 10'b0000100000;	//It was a "write_byte" command that was received
	parameter	CMND_MAKE	= 10'b0001000000;	//It was a "make_entry" command that was received
	parameter	CMND_SIZE	= 10'b0010000000;	//It was a "send_size" command that was received
	parameter	TX_SHORT	= 10'b0100000000;	//A simple reply with just two bytes will be sent
	parameter	TX_ENTRY	= 10'b1000000000;	//A reply containing an entire data entry will be sent, making it at least 8 bytes long
	
	reg	[7:0]	entry_size;	//The transfer size (# of bytes) needed to transmit the whole data entry, if one is ready (otherwise this is 0)
	reg	[7:0]	entry_byte;	//A byte from the data entry, which changes to the next byte after each time it is sent
	reg			have_entry_byte;	//Indicates that entry_byte is valid
	reg			csn_buf1, csn_buf2;	//Buffer registers for spi_csn signal. 0: SPI transfer is underway or on the verge of starting
	reg	[6:0]	cmnd_flags;			//Controls the various signals listed below:
	assign	read_byte		= cmnd_flags[0];	//Is output to the register_map module
	assign	write_byte		= cmnd_flags[1];	//Is output to the register_map module
	assign	make_entry		= cmnd_flags[2];	//Is handled in this state machine
	assign	check_size		= cmnd_flags[3];	//Affects STATE MACHINE THREE, causing the retrieval of the size of a waiting entry
	assign	get_entry_byte	= cmnd_flags[4];	//Affects STATE MACHINE THREE, causing the retrieval of the next byte of a waiting entry
	assign	lead_TXDR		= cmnd_flags[5];	//Affects STATE MACHINE ONE, causing the first byte of a prepared reply to be sent
	assign	result_TXDR		= cmnd_flags[6];	//Affects STATE MACHINE ONE, causing the next byte of a prepared reply to be sent
	
	always @(posedge system_clock or posedge reset) begin
		if (reset) begin
			csn_buf1		<= 1'b0;
			csn_buf2		<= 1'b0;
			cmnd_code		<= 4'd0;
			address			<= 4'd0;
			byte_to_write	<= 8'd0;
			cmnd_flags		<= 7'b0000000;
			result_byte		<= 8'd0;
			cmnd_sm			<= RX_IDLE;
		end
		else begin
			csn_buf1	<= spi_csn;		//Updating buffer register (active low)
			csn_buf2	<= csn_buf1;	//Updating buffer register (active low)
			case (cmnd_sm)
				RX_IDLE : begin	//The state machine is idle, checking that WB is ready and that the SPI bus is not in use
					cmnd_code		<= 4'd0;
					address			<= 4'd0;
					byte_to_write	<= 8'd0;
					cmnd_flags		<= 7'b0000000;
					result_byte		<= 8'd0;
					if (wb_ready & csn_buf2)	cmnd_sm	<= RX_READY;	//Recall that csn_buf2 is 0 when SPI is in use
					else						cmnd_sm	<= RX_IDLE;
				end
				
				RX_READY : begin	//Ready state, waiting for the first byte received in a new SPI transfer
					byte_to_write	<= 8'd0;
					cmnd_flags		<= 7'b0000000;
					result_byte		<= 8'd0;
					if (~csn_buf2 & RXDR_done)	begin
						cmnd_code	<= RXDR_byte[7:4];
						address		<= RXDR_byte[3:0];
						cmnd_sm		<= RX_BYTE_1;
					end
					else begin
						cmnd_code	<= 4'd0;
						address		<= 4'd0;
						cmnd_sm		<= RX_READY;	//Staying in this state
					end
				end
				
				RX_BYTE_1 : begin	//First byte received, waiting for second byte
					cmnd_code		<= cmnd_code;
					address			<= address;
					cmnd_flags		<= 7'b0000000;
					result_byte		<= 8'd0;
					if (~csn_buf2 & RXDR_done)	begin
						byte_to_write	<= RXDR_byte;
						cmnd_sm			<= RX_BYTE_2;
					end
					else begin
						byte_to_write	<= 8'd0;
						cmnd_sm			<= RX_BYTE_1;	//Staying in this state
					end
				end
				
				RX_BYTE_2 : begin	//Second byte also received, so machine is decoding the command code
					cmnd_code		<= cmnd_code;
					address			<= address;
					byte_to_write	<= byte_to_write;
					result_byte		<= 8'd0;
					if (cmnd_code == 4'h1) begin
						cmnd_flags	<= 7'b0000001;	//Note: the wire "read_byte" is assigned = cmnd_flags[0], so it is asserted here
						cmnd_sm		<= CMND_READ;
					end
					else if (cmnd_code == 4'h2) begin
						cmnd_flags	<= 7'b0000010;	//Note: the wire "write_byte" is assigned = cmnd_flags[1], so it is asserted here
						cmnd_sm		<= CMND_WRITE;
					end
					else if (cmnd_code == 4'h3) begin
						cmnd_flags	<= 7'b0000100;	//Note: the wire "make_entry" is assigned = cmnd_flags[2], so it is asserted here
						cmnd_sm		<= CMND_MAKE;
					end
					else if (cmnd_code == 4'h4) begin
						cmnd_flags	<= 7'b0001000;	//Note: the wire "check_size" is assigned = cmnd_flags[3], so it is asserted here
						cmnd_sm		<= CMND_SIZE;
					end
					else if (cmnd_code == 4'h5) begin
						cmnd_flags	<= 7'b0110000;	//Note: the wire "get_entry_byte" is assigned = cmnd_flags[4], so it is asserted here
						cmnd_sm		<= TX_ENTRY;
					end
					else begin
						cmnd_flags	<= 7'b0000000;
						cmnd_sm		<= RX_IDLE;		//It was a "Null" command so return to idle state
					end
				end
				
				CMND_READ : begin	//It was a "read_byte" command that was received
					cmnd_code		<= cmnd_code;
					address			<= address;
					byte_to_write	<= byte_to_write;
					result_byte		<= read_result;
					if (~done_read_byte) begin	//done_read_byte is sent by the external register_map module
						cmnd_flags	<= 7'b0000001;	//Note: the wire "read_byte" is assigned = cmnd_flags[0], so it is asserted here
						cmnd_sm		<= CMND_READ;	//Staying in this state
					end
					else begin
						cmnd_flags	<= 7'b0100000;	//Note: the wire "reply" is assigned = cmnd_flags[5], so it is asserted here
						cmnd_sm		<= TX_SHORT;	//A simple reply with just two bytes will be sent
					end
				end
				
				CMND_WRITE : begin	//It was a "write_byte" command that was received
					cmnd_code		<= cmnd_code;
					address			<= address;
					byte_to_write	<= byte_to_write;
					result_byte		<= read_result;	//Note: this is deliberate, because every write is followed by a read of the same register
					if (~done_read_byte) begin
						cmnd_flags	<= 7'b0000010;	//Note: the wire "write_byte" is assigned = cmnd_flags[1], so it is asserted here
						cmnd_sm		<= CMND_WRITE;	//Staying in this state
					end
					else begin
						cmnd_flags	<= 7'b0100000;	//Note: the wire "lead_TXDR" is assigned = cmnd_flags[5], so it is asserted here
						cmnd_sm		<= TX_SHORT;	//A simple reply with just two bytes will be sent
					end
				end
				
				CMND_MAKE : begin	//It was a "MAKE_ENTRY" command that was received
					cmnd_code		<= cmnd_code;
					address			<= address;
					byte_to_write	<= byte_to_write;
					result_byte		<= 8'd1;	//Acknowledgement that make_entry state was reached
					if (~make_entry) begin
						cmnd_flags	<= 7'b0000100;	//Note: the wire "make_entry" is assigned = cmnd_flags[2], so it is asserted here
						cmnd_sm		<= CMND_MAKE;	//Staying in this state
					end
					else begin
						cmnd_flags	<= 7'b0100000;	//Note: the wire "lead_TXDR" is assigned = cmnd_flags[5], so it is asserted here
						cmnd_sm		<= TX_SHORT;	//A simple reply with just two bytes will be sent
					end
				end
				
				CMND_SIZE : begin	//It was a "SEND_SIZE" command that was received
					cmnd_code		<= cmnd_code;
					address			<= address;
					byte_to_write	<= byte_to_write;
					result_byte		<= entry_size;	//Note: is 8'd0 if no entry is ready to be transmitted
					if (~check_size) begin
						cmnd_flags	<= 7'b0001000;	//Note: the wire "check_size" is assigned = cmnd_flags[3], so it is asserted here
						cmnd_sm		<= CMND_SIZE;	//Staying in this state
					end
					else begin
						cmnd_flags	<= 7'b0100000;	//Note: the wire "lead_TXDR" is assigned = cmnd_flags[5], so it is asserted here
						cmnd_sm		<= TX_SHORT;	//A simple reply with just two bytes will be sent
					end
				end
				
				TX_SHORT : begin	//A simple reply with just two bytes is being sent
					cmnd_code		<= cmnd_code;
					address			<= address;
					byte_to_write	<= byte_to_write;
					result_byte		<= result_byte;
					if (~lead_done) begin	//Waiting for first byte to be sent
						cmnd_flags	<= 7'b0100000;	//Note: the wire "lead_TXDR" is assigned = cmnd_flags[5], so it STAYS asserted here
						cmnd_sm		<= TX_SHORT;	//Staying in this state
					end
					else if (~result_done) begin	//Waiting for second byte to be sent
						cmnd_flags	<= 7'b1000000;	//Note: the wire "result_TXDR" is assigned = cmnd_flags[6], so it is asserted here
						cmnd_sm		<= TX_SHORT;	//Staying in this state
					end
					else begin	//Both bytes have been sent so this state machine's job is complete for this command
						cmnd_flags	<= 7'b0000000;
						cmnd_sm		<= RX_IDLE;
					end
				end
				
				TX_ENTRY : begin	//A reply containing an entire data entry will be sent, making it at least 8 bytes long
					cmnd_code		<= cmnd_code;
					address			<= address;
					byte_to_write	<= byte_to_write;
					result_byte		<= entry_byte;	//Note: is 8'd0 unless (an entry is ready AND the entry's size was already transmitted)
					if (~lead_done) begin
						cmnd_flags	<= 7'b0100000;	//Note: the wire "lead_TXDR" is assigned = cmnd_flags[5], so it STAYS asserted here
						cmnd_sm		<= TX_ENTRY;
					end
					else if (have_entry_byte & ~csn_buf2) begin
						cmnd_flags	<= 7'b1000000;	//Note: the wire "result_TXDR" is assigned = cmnd_flags[6], so it is asserted here
						cmnd_sm		<= TX_ENTRY;
					end
					else if (~csn_buf2) begin
						cmnd_flags	<= 7'b0010000;	//Note: the wire "get_entry_byte" is assigned = cmnd_flags[4], so it is asserted here
						cmnd_sm		<= TX_ENTRY;
					end
					else begin	//The SPI Master ended the transfer
						cmnd_flags	<= 7'b0000000;
						cmnd_sm		<= RX_IDLE;
					end
				end
				
				default : begin		//Here in case of error. Same as IDLE state and goes to that state
					cmnd_code		<= 4'd0;
					address			<= 4'd0;
					byte_to_write	<= 8'd0;
					cmnd_flags		<= 7'b0000000;
					result_byte		<= 8'd0;
					cmnd_sm			<= RX_IDLE;
				end
			endcase
		end
	end
	
	
	reg			entry_rd_Aclr;
	wire [7:0]	entry_RdEn_count;
	up_counter_8	ENTRY_RD_COUNTER		(	//8-bit counter for the number of bytes read out from ENTRY_FIFO_INST
		.Aclr 			(entry_rd_Aclr		),	//Input. Asynchronous signal to clear the counter
		.Clock			(system_clock		),	//Input. Clock to increment the counter
		.Clk_En 		(entry_RdEn			),	//Input. Reading samples from A's channel_fifo enables the counter's clock
		.Q				(entry_RdEn_count	))	//Output. 8-bit output with the present count value
	;
	com_ne_8_noreg	ENTRY_RD_COM			(	//8-bit comparator to indicate if bytes still need to read from ENTRY_FIFO_INST
		.DataA			(entry_RdEn_count	),	//Input. From counter right above this
		.DataB			(entry_size			),	//Input. Determined in "The Size Retrieval Phase" of state machine below
		.ANEB			(keep_reading		))	//Output. Controls "The Byte Retrieval Phase" of state machine below
	;
	
	//STATE MACHINE THREE, TO SUPPLY ENTRY SIZE AND ENTRY BYTES FOR TRANSMISSION
	reg	[5:0]	entry_tx_sm;	/* synthesis syn_encoding = "onehot" */
	parameter	ENTRY_IDLE		= 6'b000001;
	parameter	ENTRY_GET_SIZE	= 6'b000010;
	parameter	ENTRY_HAVE_SIZE	= 6'b000100;
	parameter	ENTRY_BYTE_MODE	= 6'b001000;
	parameter	ENTRY_GET_BYTE	= 6'b010000;
	parameter	ENTRY_HAVE_BYTE	= 6'b100000;
	
	reg		entry_Empty_buf, entry_waiting;
	
	always @(posedge system_clock or posedge reset) begin
		if (reset) begin
			entry_Empty_buf	<= 1'b0;
			entry_waiting	<= 1'b0;
			entry_rd_Aclr	<= 1'b1;
			entry_RdEn		<= 1'b0;
			entry_size		<= 8'd0;
			entry_byte		<= 8'd0;
			have_entry_byte	<= 1'b0;
			entry_tx_sm		<= ENTRY_IDLE;
		end
		else begin
			entry_Empty_buf	<= entry_Empty;
			entry_waiting	<= wb_ready & ~entry_Empty_buf;
			entry_rd_Aclr	<= entry_tx_sm[5] & ~keep_reading;
			
			case (entry_tx_sm)
				//---------------------------The Size Retrieval Phase--------------------------
				ENTRY_IDLE : begin		//Checking if an entry is waiting in ENTRY_FIFO_INST
					entry_size		<= 8'd0;
					entry_byte		<= 8'd0;
					have_entry_byte	<= 1'b0;
					if (entry_waiting) begin	//Assert RdEn to get the entry size and go to next step
						entry_RdEn	<= 1'b1;
						entry_tx_sm <= ENTRY_GET_SIZE;
					end
					else begin
						entry_RdEn	<= 1'b0;
						entry_tx_sm	<= ENTRY_IDLE;
					end
				end
				
				ENTRY_GET_SIZE : begin	//entry_Q should now have the size of the waiting entry
					entry_RdEn		<= 1'b0;	//Deassert
					entry_size		<= entry_Q;	//Store the size of the entry
					entry_byte		<= 8'd0;
					have_entry_byte	<= 1'b0;
					entry_tx_sm		<= ENTRY_HAVE_SIZE;
				end
				
				ENTRY_HAVE_SIZE : begin	//entry_size is now accurate
					entry_RdEn		<= 1'b0;
					entry_size		<= entry_size; //Holding constant for the rest of the state machine's steps
					entry_byte		<= 8'd0;
					have_entry_byte	<= 1'b0;
					if (check_size) entry_tx_sm	<= ENTRY_BYTE_MODE;
					else			entry_tx_sm	<= ENTRY_HAVE_SIZE;
				end
				
				//---------------------------The Byte Retrieval Phase--------------------------
				ENTRY_BYTE_MODE : begin	//Watch for get_entry_byte requests
					entry_size		<= entry_size;
					entry_byte		<= 8'd0;
					have_entry_byte	<= 1'b0;
					if (get_entry_byte & keep_reading) begin	//Assert RdEn to read the entry byte
						entry_RdEn	<= 1'b1;
						entry_tx_sm <= ENTRY_GET_BYTE;
					end
					else begin
						entry_RdEn	<= 1'b0;
						entry_tx_sm	<= ENTRY_BYTE_MODE;
					end
				end
				
				ENTRY_GET_BYTE : begin	//entry_Q should now have a retrieved byte of the waiting entry
					entry_RdEn		<= 1'b0;	//Deassert
					entry_size		<= entry_size;
					entry_byte		<= entry_Q;	//Latch the entry byte
					have_entry_byte	<= 1'b1;	//Assert
					entry_tx_sm 	<= ENTRY_HAVE_BYTE;
				end
				
				ENTRY_HAVE_BYTE : begin			//entry_byte is now accurate
					entry_RdEn		<= 1'b0;
					entry_byte		<= entry_byte;
					have_entry_byte	<= 1'b1;
					if (keep_reading & result_done) begin	//result_done means the current entry_byte was copied for transfer
						entry_size	<= entry_size;
						entry_tx_sm	<= ENTRY_BYTE_MODE;		// ...so now this state machine is going back two steps
					end
					else if (keep_reading) begin
						entry_size	<= entry_size;
						entry_tx_sm	<= ENTRY_HAVE_BYTE;	//Staying in current state
					end
					else begin					//The whole entry has been read out
						entry_size		<= 8'd0;	//Wipe because no longer needed
						entry_tx_sm		<= ENTRY_IDLE;
					end
				end
			endcase
		end
	end
			
	
endmodule
		
			
			