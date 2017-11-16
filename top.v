
`timescale 1ns/ 1ps

module top			(
	input	RESETN	,	//Reset design, active low, asserted by a Raspberry Pi GPIO
	input	CCLK	,	//Hard SPI serial clock
	input	SCSN	,	//Hard SPI chip-select (active low)
	input	SI		,	//Hard SPI serial input data
	output	SO		,	//Hard SPI serial output data
	input	SCL		,	//Hard Primary I2C clock
	inout	SDA		,	//Hard Primary I2C data
	output	ACLK	,	//Clock output, provides sample clock signal to the ADC
	input	DA9		,	//ADC Channel A sample bit 9, most significant bit (MSB)
	input	DA8		,	//ADC Channel A sample bit 8
	input	DA7		,	//ADC Channel A sample bit 7
	input	DA6		,	//ADC Channel A sample bit 6
	input	DA5		,	//ADC Channel A sample bit 5
	input	DA4		,	//ADC Channel A sample bit 4
	input	DA3		,	//ADC Channel A sample bit 3
	input	DA2		,	//ADC Channel A sample bit 2
	input	DA1		,	//ADC Channel A sample bit 1
	input	DA0		,	//ADC Channel A sample bit 0, least significant bit (LSB)
	input 	DCLKA	,	//ADC Channel A sample data clock
	input	DORA	,	//ADC Channel A sample Data Over Range indicator
	input	DB9		,	//ADC Channel B sample bit 9, most significant bit (MSB)
	input	DB8		,	//ADC Channel B sample bit 8
	input	DB7		,	//ADC Channel B sample bit 7
	input	DB6		,	//ADC Channel B sample bit 6
	input	DB5		,	//ADC Channel B sample bit 5
	input	DB4		,	//ADC Channel B sample bit 4
	input	DB3		,	//ADC Channel B sample bit 3
	input	DB2		,	//ADC Channel B sample bit 2
	input	DB1		,	//ADC Channel B sample bit 1
	input	DB0		,	//ADC Channel B sample bit 0, least significant bit (LSB)
	input 	DCLKB	,	//ADC Channel B sample data clock
	input	DORB	,	//ADC Channel B sample Data Over Range indicator
	input	TPULSE	,	//TIMEPULSE signal from the GNSS receiver
	output	ENTRY	)	//Indicator that at least one entry is waiting to be transmitted
	;
	wire	entry_Empty	;	//The Empty indicator from ENTRY_FIFO_INST at the bottom of this module
	assign	ENTRY		= ~entry_Empty;
	wire	reset		;
	assign	reset		= ~RESETN;
	wire	t_pulse		;
	assign	t_pulse		= TPULSE;
	//THE XO3'S INTERNAL OSCILLATOR PROVIDES THE INPUT CLOCK FOR DAQ_PLL_INST
	wire		osc_clock		;	//The oscillator's clock output
	defparam	OSCH_INST.NOM_FREQ = "66.50";	// "2.08" is the default frequency
	OSCH			OSCH_INST			(
		.STDBY			(1'b0			),	//Input. 0=Enabled, 1=Disabled. Also Disabled with Bandgap=OFF
		.OSC			(osc_clock		),	//Output. 
		.SEDSTDBY		(				))	//This signal is not required if not using SED
	;
	
	//PHASE-LOCKED-LOOP TO PROVIDE THE PRINCIPAL CLOCK SIGNALS FOR THE DESIGN
	daq_pll			DAQ_PLL_INST		(
		.RST			(reset			),	//Input. Reset the PLL
		.CLKI			(osc_clock		),	//Input. Input clock
		.ENCLKOP		(1'b1			),	//Input. Enables CLKOP output
		.ENCLKOS		(1'b1			),	//Input. Enables CLKOS output
		.ENCLKOS2		(enable_ACLK	),	//Input. Enables CLKOS2 output
		.CLKOP			(system_clock	),	//Output. 
		.CLKOS			(entry_clock	),	//Output. 
		.CLKOS2			(ACLK			))	//Output. 
	;
	
	
	wire			make_entry	;	//Control signal to manually create a new entry
	wire			entry_RdEn	;	//Read enable for ENTRY_FIFO_INST
	wire	[7:0]	entry_Q		;	//A byte read from ENTRY_FIFO_INST
	wire	[3:0]	address		;
	wire	[7:0]	byte_to_write, read_result;
	
	//WISHBONE (WB) Master Interface signals
	//wire			wb_reset		;	//WISHBONE bus reset
	wire			wb_cyc			;	//WISHBONE bus cycle (also used for the strobe signal in this design)
	wire			wb_wr_enable	;	//WISHBONE read/write control. 0: read, 1: write
	wire	[7:0]	wb_address		;	//WISHBONE address to select an EFB register to read or write
	wire	[7:0]	wb_data_mosi	;	//WISHBONE data; Master-Out, Slave-In
	wire	[7:0]	wb_data_miso	;	//WISHBONE data; Master-In, Slave-Out
	wire			wb_ack			;	//WISHBONE acknowledge; data written is accepted/data read is valid
	
	//THE KEY CONTROL MODULE IN THE DESIGN, SLAVE TO THE EXTERNAL SPI MASTER (THE RASPBERRY PI)
	spi_slave			SPI_SLAVE_INST			(
		.reset				(reset				),
		.system_clock		(system_clock		),	//Input. System clock
		.spi_csn			(SCSN				),	//Input. Hard SPI chip-select (active low)
		.entry_Empty		(entry_Empty		),	//The Empty indicator from ENTRY_FIFO_INST
		.entry_RdEn			(entry_RdEn			),	//Read enable for ENTRY_FIFO_INST
		.entry_Q			(entry_Q			),	//A byte read out from ENTRY_FIFO_INST
		.make_entry			(make_entry			),	//Signal to manually have ENTRY_MAKER_INST create a new entry
		//Signals for delegating commands to the design's register_map module instance, REGISTER_MAP_INST
		.address			(address			),	//Output. The map address of the register to be read/written
		.write_byte			(write_byte			),	//Output. The signal to make REGISTER_MAP_INST write to a register
		.byte_to_write		(byte_to_write		),	//Output. The byte to write to the addressed register
		.done_write_byte	(done_write_byte	),	//Input. The indicator that the write operation is complete
		.read_byte			(read_byte			),	//Output. The signal to make REGISTER_MAP_INST read to a register
		.read_result		(read_result		),	//Input. The byte read from the addressed register
		.done_read_byte		(done_read_byte		),	//Input. The indicator that the read operation is complete
		//WISHBONE Bus signals
		.wb_cyc				(wb_cyc				),	//WISHBONE bus cycle (also used for the strobe signal in this design)
		.wb_wr_enable		(wb_wr_enable		),	//WISHBONE read/write control. 0: read, 1: write
		.wb_address			(wb_address			),	//WISHBONE address
		.wb_data_mosi		(wb_data_mosi		),	//WISHBONE data sent to the EFB
		.wb_data_miso		(wb_data_miso		),	//WISHBONE data sent from the EFB
		.wb_ack				(wb_ack				))	//WISHBONE transfer acknowledge
	;
	
	
	//wire			e_maker_reset	;	//Reset the ENTRY_MAKER_INST
	wire			enable_saving	;	//Enable examination of samples & acquisition of sequences
	wire	[28:0]	A_criteria		;	//Criteria for the selection of sample sequences on Channel A
	wire	[28:0]	B_criteria		;	//Criteria for the selection of sample sequences on Channel B
	wire	[25:0]	status			;	//A concatenation of the status information sent by other modules
	
	//MODULE CONTAINING GENERAL REGISTERS FOR CONFIGURING AND MONITORING THE DESIGN
	wire	[7:0]	seq_limit;	//A limit on the number of sequence-induced entries created each second
	register_map		REGISTER_MAP_INST		(
		.reset				(reset				),	//Input. 
		.t_pulse			(t_pulse			),	//Input. 
		.system_clock		(system_clock		),	//Input. 
		.top_status			(status				),	//Input. 
		.enable_ACLK		(enable_ACLK		),	//Output. 
		.enable_auto		(enable_auto		),	//Output. Enable the automatic creation of new data entries
		.enable_saving		(enable_saving		),	//Output. 
		.seq_limit			(seq_limit			),	//Output. A limit on the number of sequence-induced entries created each second
		.A_criteria			(A_criteria			),	//Output. 
		.B_criteria			(B_criteria			),	//Output. 
		.update_A			(update_A			),	//Output. 
		.update_B			(update_B			),	//Output. 
		//Signals for handling commands delegated by SPI_SLAVE_INST
		.address			(address			),	//Input. 
		.write_byte			(write_byte			),	//Input. 
		.byte_to_write		(byte_to_write		),	//Input. 
		.done_write_byte	(done_write_byte	),	//Output. 
		.read_byte			(read_byte			),	//Input. 
		.read_result		(read_result		),	//Output. 
		.done_read_byte		(done_read_byte		))	//Output. 
	;
	
	
	//MACHXO3 EMBEDDED FUNCTION BLOCK (EFB) WITH BUILT-IN, HARDENED CONTROL FUNCTIONS
	daq_efb				DAQ_EFB_INST		(
		//WISHBONE signals for interacting with SPI_SLAVE_INST
		.wb_rst_i			(reset			),	//Input. Synchronous reset signal for WB interface logic
		.wb_clk_i			(system_clock	),	//Input. Positive edge clock used by WB interface registers
		.wb_cyc_i			(wb_cyc			),	//Input. 
		.wb_stb_i			(wb_cyc			),	//Input. Strobe signal to target this WB slave (the only slave)
		.wb_we_i			(wb_wr_enable	),	//Input. 
		.wb_adr_i			(wb_address		),	//Input. 
		.wb_dat_i			(wb_data_mosi	),	//Input. Master-Out, Slave-In
		.wb_dat_o			(wb_data_miso	),	//Output. Master-In, Slave-Out
		.wb_ack_o			(wb_ack			),	//Output. 
		//Signals for the Hardened communication IP Cores
		.i2c1_scl			(SCL			),	//Input. 
		.i2c1_sda			(SDA			),	//Input/output. 
		.i2c1_irqo			(i2c1_irqo		),	//Output. Hard Primary I2C interrupt output
		.spi_clk			(CCLK			),	//Input. 
		.spi_miso			(SO				),	//Output. 
		.spi_mosi			(SI				),	//Input. 
		.spi_scsn			(SCSN			))	//Input. 
	;
	
	//SINGLE-DATA-RATE I/O INTERFACES FOR RECEIVING ADC DATA
	wire	[10:0]	A_bus_datain, B_bus_datain, A_bus_q, B_bus_q	;
	assign			A_bus_datain	= {DORA, DA9, DA8, DA7, DA6, DA5, DA4, DA3, DA2, DA1, DA0}	;
	assign			B_bus_datain	= {DORB, DB9, DB8, DB7, DB6, DB5, DB4, DB3, DB2, DB1, DB0}	;
	sdr_rx_11_100		A_BUS_RECEIVER		(
		.reset				(reset			),	//Input. Reset
		.clk				(DCLKA			),	//Input. Data clock from the Channel A bus
		.datain				(A_bus_datain	),	//Input. Concatenation of Bank 2 input signals for ADC data from Channel A
		.sclk				(A_bus_sclk		),	//Output. Data clock for the A_bus_q output
		.q					(A_bus_q		))	//Output. The registered data received from Channel A
	;
	sdr_rx_11_100		B_BUS_RECEIVER		(
		.reset				(reset			),	//Input. Reset
		.clk				(DCLKB			),	//Input. Data clock from the Channel B bus
		.datain				(B_bus_datain	),	//Input. Concatenation of Bank 1 input signals for ADC data from Channel B
		.sclk				(B_bus_sclk		),	//Output. Data clock for the B_bus_q output
		.q					(B_bus_q		))	//Output. The registered data received from Channel B
	;
	
	//FIFO (FIRST-IN, FIRST-OUT) MEMORY BLOCKS TO SYNCHRONIZE THE UPDATE OF A_fifo_Q and B_fifo_Q
	wire			A_AlmostEmpty, B_AlmostEmpty ;
	wire	[10:0]	A_fifo_Q, B_fifo_Q ;
	data_bus_fifo		A_BUS_FIFO			(	//PFU-based FIFO memory instance of size 8 x 11
		.Reset				(reset			),	//Input. Reset signal
		.RPReset			(reset			),	//Input. Read pointer reset signal
		.WrClock			(A_bus_sclk		),	//Input. Write port clock
		.RdClock			(entry_clock	),	//Input. Read port clock
		.WrEn				(enable_ACLK	),	//Input. Write port enable
		.Data				(A_bus_q		),	//Input. 8-bit write data word
		.RdEn				(A_RdEn			),	//Input. Read enable
		.Q					(A_fifo_Q		),	//Output. 8-bit read word
		.Empty				(A_Empty		),	//Output. Indicates 0 of 8 words are stored in memory
		.AlmostEmpty		(A_AlmostEmpty	),	//Output. Indicates at most 2 of 8 words are stored in memory
		.Full				(A_Full			))	//Output. Indicates 8 of 8 words are stored in memory
	;
	data_bus_fifo		B_BUS_FIFO			(	//PFU-based FIFO memory instance of size 8 x 11
		.Reset				(reset			),	//Input. Reset signal
		.RPReset			(reset			),	//Input. Read pointer reset signal
		.WrClock			(B_bus_sclk		),	//Input. Write port clock
		.RdClock			(entry_clock	),	//Input. Read port clock
		.WrEn				(enable_ACLK	),	//Input. Write port enable
		.Data				(B_bus_q		),	//Input. 8-bit write data word
		.RdEn				(B_RdEn			),	//Input. Read enable
		.Q					(B_fifo_Q		),	//Output. 8-bit read word
		.Empty				(B_Empty		),	//Output. Indicates 0 of 8 words are stored in memory
		.AlmostEmpty		(B_AlmostEmpty	),	//Output. Indicates at most 2 of 8 words are stored in memory
		.Full				(B_Full			))	//Output. Indicates 8 of 8 words are stored in memory
	;
	assign			A_RdEn	= ~(A_AlmostEmpty | B_AlmostEmpty)	;
	assign			B_RdEn	= ~(A_AlmostEmpty | B_AlmostEmpty)	;
	
	//MODULE THAT MONITORS ADC SAMPLES, SELECTIVELY CAPTURES SAMPLE SEQUENCES AND LABELS THEM WITH A METADATA "HEADER"
	wire	[20:0]	maker_flags	;
	wire	[21:0]	adc_q		;
	assign			adc_q		= {A_fifo_Q, B_fifo_Q}	;
	wire			entry_AmFull;	//Indicates the ENTRY_FIFO_INST memory below is almost full
	wire			entry_WrEn	;	//Enables the write port of the ENTRY_FIFO_INST below
	wire	[7:0]	entry_Data	;	//The data word to store in the ENTRY_FIFO_INST below
	entry_maker			ENTRY_MAKER_INST	(
		.reset				(reset			),	//Input. Reset
		.system_clock		(system_clock	),	//Input. 
		.entry_clock		(entry_clock	),	//Input. 
		.maker_flags		(maker_flags	),	//Output. Fifo memory flags and other entry-related status signals
		.enable_auto		(enable_auto	),	//Input. Enable the automatic creation of new data entries
		.enable_saving		(enable_saving	),	//Input. Enable examination of samples & acquisition of sequences
		.adc_q				(adc_q			),	//Input. 
		.t_pulse			(t_pulse		),	//Input. 
		.make_entry			(make_entry		),	//Input. Control signal to manually create a new entry
		.seq_limit			(seq_limit		),	//Input. A limit on the number of sequence-induced entries created each second
		.A_criteria			(A_criteria		),	//Input. Criteria for the selection of sample sequences on Channel A
		.B_criteria			(B_criteria		),	//Input. Criteria for the selection of sample sequences on Channel B
		.update_A			(update_A		),	//Input. Signal to make A_CHANNEL_MONITOR update its criteria
		.update_B			(update_B		),	//Input. Signal to make B_CHANNEL_MONITOR update its criteria
		.entry_AmFull		(entry_AmFull	),	//Input. 
		.entry_WrEn			(entry_WrEn		),	//Output. 
		.entry_Data			(entry_Data		))	//Output. 
	;
	
	//FIFO (FIRST-IN, FIRST-OUT) MEMORY FOR STORING DATA ENTRIES ASSEMBLED BY ENTRY_MAKER_INST
	wire	entry_Full;		//Indicates the memory is full
	wire	entry_AmEmpty;	//Indicates the memory is almost empty
	entry_fifo			ENTRY_FIFO_INST		(	//EBR-based FIFO memory instance of size 16384 x 8
		.Reset				(reset			),	//Input. Reset signal
		.RPReset			(reset			),	//Input. Read pointer reset signal
		.WrClock			(system_clock	),	//Input. Write port clock
		.RdClock			(system_clock	),	//Input. Read port clock
		.WrEn				(entry_WrEn		),	//Input. Write port enable
		.Data				(entry_Data		),	//Input. 8-bit write data word
		.RdEn				(entry_RdEn		),	//Input. Read enable
		.Q					(entry_Q		),	//Output. 8-bit read word
		.Empty				(entry_Empty	),	//Output. Indicates 0 of 16384 words are stored in memory
		.Full				(entry_Full		),	//Output. Indicates 16384 of 16384 words are stored in memory
		.AlmostEmpty		(entry_AmEmpty	),	//Output. Indicates at most 8192 of 16384 words are stored in memory
		.AlmostFull			(entry_AmFull	))	//Output. Indicates at least 16240 of 16384 words are stored in memory
	;
	assign status[20:0]		= maker_flags;
	assign status[24:21]	= {entry_Empty, entry_Full, entry_AmEmpty, entry_AmFull};
	assign status[25]		= i2c1_irqo;
	
endmodule

/* ******************************* NO LONGER USED OR NO LONGER APPLICABLE: *******************************

	wire	[3:0]	sample_LSBs		;	//ADC output bits not involved in data acquisition in this design version
	wire	[17:0]	DOR_and_MSBs	;	//ADC output bits involved in data acquisition in this design version
	assign			sample_LSBs		= {adc_q[12:11], adc_q[1:0]};	//Each channel's 2 least significant sample bits
	assign			DOR_and_MSBs	= {adc_q[21:13], adc_q[10:2]};	//Each channel's DOR and 8 most significant bits
	
		.sample_LSBs		(sample_LSBs	),
		
		.adc_q				(DOR_and_MSBs	),
		
	input	[10:0]	DATAIN			,
	//input			D9				,	//ADC sample data bit 9, most significant bit (MSB)
	//input			D8				,	//ADC sample data bit 8
	//input			D7				,	//ADC sample data bit 7
	//input			D6				,	//ADC sample data bit 6
	//input			D5				,	//ADC sample data bit 5
	//input			D4				,	//ADC sample data bit 4
	//input			D3				,	//ADC sample data bit 3
	//input			D2				,	//ADC sample data bit 2
	//input			D1				,	//ADC sample data bit 1
	//input			D0				,	//ADC sample data bit 0, least significant bit (LSB)
	
	//input			DOR				,	//ADC sample Data Over Range indicator
	
		//.freeze				(adc_freeze		),	//
		//.uddcntln			(adc_uddcntln	),	//
		//.lock				(status[23]		),	//
		
		.adc_freeze			(adc_freeze		),
		.adc_uddcntln		(adc_uddcntln	),
		
		*/