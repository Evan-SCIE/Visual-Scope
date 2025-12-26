//Purpose: Implement a high-speed digital storage oscilloscope using a FPGA 
//Receive analog signals via ADCs, deserializes and processes the data, and transfers to DDR3 memory / PC 
//Support configuration and control of the analog front-end through I2C / SPI -> High-speed data transfer via PCIe + Provide flash memory access via QSPI
//Goal: Provide flexible, reliable and high-performance signal acquisition, storage and transfer for measurement and diagnostics applications 
`timescale 1 ps / 1 ps //Set simulation time unit and precision 

module dso_top
   (
    //DDR3 memory interface (for external RAM)
    output [14:0] DDR3_addr,  //DDR3 address bus (column/row address)
    output [2:0] DDR3_ba,     //DDR3 bank address (select memory bank)
    output DDR3_cas_n,        //DDR3 column address strobe (active low)
    output [0:0] DDR3_ck_n,   //DDR3 differential clock negative 
    output [0:0] DDR3_ck_p,   //DDR3 differential clock positive 
    output [0:0] DDR3_cke,    //DDR3 clock enable (enables clock to DDR3)
    output [0:0] DDR3_cs_n,   //DDR3 chip select (active low)
    output [3:0] DDR3_dm,     //DDR3 data mask (byte-wise write mask)
    inout [31:0] DDR3_dq,     //DDR3 data bus (bidirectional, 32 bits)
    inout [3:0] DDR3_dqs_n,   //DDR3 data strobe negative (bidirectional)
    inout [3:0] DDR3_dqs_p,   //DDR3 data strobe positive (bidirectional)
    output [0:0] DDR3_odt,    //DDR3 on-die termination enable 
    output DDR3_ras_n,        //DDR3 row address strobe (active low)
    output DDR3_reset_n,      //DDR3 reset (active low)
    output DDR3_we_n,         //DDR3 write enable (active low)

    //PCIe interface (for high-speed data transfer to host PC)
    input [0:0] pcie_clk_n,    //PCIe differential clock negative 
    input [0:0] pcie_clk_p,    //PCIe differential clock positive 
    input [3:0] pcie_mgt_rxn,  //PCIe receive differential negative 
    input [3:0] pcie_mgt_rxp,  //PCIe receive differential positive 
    output [3:0] pcie_mgt_txn, //PCIe transmit differential negative 
    output [3:0] pcie_mgt_txp, //PCIe transmit differential positive 
    input pcie_perstn,         //PCIe reset (active low)

    //Front panel and control signals 
    output led,           //Status LED (shows system ready/fault indicator)
    output sync,          //Synchronization output (10 kHz square wave)
    output [3:0] atten,   //Attenuator control (for analog front-end)
    output [3:0] dc_cpl,  //DC counpling control (for analog front-end)
    output [3:0] term,    //Termination control (for analog front-end)

    //I2C and SPI interfaces
    output i2c_sda,	     //I2C data (open-drain emulation)				
    output i2c_scl,			 //I2C clock (open-drain emulation)
    output spi_sdio,		 //SPI data 
    output spi_sclk,		 //SPI clock
    output [3:0] pga_cs, //Programmable gain amplifier chip select 
    output adc_cs,       //ADC chip select 

    //Acquisition and oscilloscope controls 
    output acq_en,       //Acquisition mode 
    output osc_oe,       //Oscillator output enable 
    output fe_en,        //Front-end enable (analog selection)
    output probe_comp,   //Probe compensation output (10 kHz square waqve)

    //ADC differential clock and data 
    input adc_lclk_p,      //ADC LVDS clock positive
    input adc_lclk_n,      //ADC LVDS clock negative 
    input adc_fclk_p,      //ADC frame clock positive 
    input adc_fclk_n,      //ADC frame clock negative 
    input[7:0] adc_data_p, //ADC data positive (8 LVDS channels)
    input[7:0] adc_data_n, //ADC data negative (8 LVDS channels)

    //Analog probe input 
    input Vp_Vn_0_v_n,   //Analog probe negative 
    input Vp_Vn_0_v_p,   //Analog probe positive 

    //QSPI flash interface
    inout qspi_d0,   //QSPI data 0 (bidirectional)
    inout qspi_d1,   //QSPI data 1 (bidirectional)
    inout qspi_d2,   //QSPI data 2 (bidirectional)
    inout qspi_d3,   //QSPI data 3 (bidirectional)
    output qspi_cs,  //QSPI chip select (open-drain emulation)

    //Ethernet reset  
    output eth_rst_n, //Ethernet PHY reset (active low)

    //External clock input 
    input ext_clk_p,  //External clock positive 
    input ext_clk_n   //External clock negative
    );
  
  //Internal signal declaration 
  //AXI Stream for data transfer to host 
  wire [31:0]AXI_STR_TXD_0_tdata;  //AXI stream data (32 bits)
  wire AXI_STR_TXD_0_tlast;        //AXI stream last signal (end of packet)
  wire AXI_STR_TXD_0_tready;       //AXI stream ready (slave ready to accept data)
  wire AXI_STR_TXD_0_tvalid;       //AXI stream valid (data is valid)
  
  //AXI and DMA control signals
  wire S01_ARESETN;                 //AXI reset (active low)
  wire [71:0]S_AXIS_S2MM_CMD_tdata; //DMA command data (write address, length, etc.)
  wire S_AXIS_S2MM_CMD_tready;      //DMA command ready (DMA ready to accept command)
  wire S_AXIS_S2MM_CMD_tvalid;      //DMA command valid(command is valid)
  wire [127:0]S_AXIS_S2MM_tdata;    //DMA data (128 bits)
  wire S_AXIS_S2MM_tready;          //DMA data ready (DMA ready to accept data)
  wire S_AXIS_S2MM_tvalid;          //DMA data valid (data is valid)
  wire S_AXIS_S2MM_tlast;           //DMA data last (end of burst)
  
  wire axi_aclk;              //AXI clock (system clock)
  wire axi_aresetn;           //AXI reset (active low)
 
  wire [31:0]gpio2_io_i;      //GPIO input （32 bits）
  wire [31:0]gpio2_io_o_0;    //GPIO output 2 (32 bits)
  wire [31:0]gpio_io_o_0;     //GPIO output (32 bits)

  wire s2mm_err;              //DMA error flag 
  wire s2mm_halt;             //DMA halt request
  wire s2mm_halt_cmplt;       //DMA halt complete 
  wire s2mm_wr_xfer_cmplt;    //DMA write transfer complete
  
  wire fe_sda_buf;            //I2C SDA buffer (internal)
  wire fe_scl_buf;            //I2C SCL buffer (internal)
  
  wire divclk;            //Divided clock (from SERDES, 125 MHz)  
  wire[63:0] data_deser;  //Deserialized ADC data (from SERDES, 8 channels x 8 bits)
  reg[63:0] adc_data;	    //Processed ADC data (to DMA)
  wire serdes_ready;      //SERDES ready flag (data valid)
  
  //SPI/QSPI IO buffer signals (for bidirectional QSPI lines)
  //Declare signals (wires) used for the QSPI (Quad SPI) flash interface 
  //QSPI is a high-speed serial interface commonly used for connecting flash memory to FPGAs 
  //IOBUF primitives later in the code - Handle the bidirectional nature of QSPI data lines
  //IOBUF allows teh FPGA to switch each pin between input and output dynamically - Reading from or writing to the flash memory 
  wire spi_rtl_0_io0_i;  //Input data from the QSPI pin to the FPGA logic
  wire spi_rtl_0_io0_io; //Bidirectional IO pins 
  wire spi_rtl_0_io0_o;  //Output data from the FPGA logic to the QSPI pin
  wire spi_rtl_0_io0_t;  //Tristate control signal - Asserted - High-impedance mode input // Deasserted - Pin drives output 
  wire spi_rtl_0_io1_i;
  wire spi_rtl_0_io1_io;
  wire spi_rtl_0_io1_o;
  wire spi_rtl_0_io1_t;
  wire spi_rtl_0_io2_i;
  wire spi_rtl_0_io2_io;
  wire spi_rtl_0_io2_o;
  wire spi_rtl_0_io2_t;
  wire spi_rtl_0_io3_i;
  wire spi_rtl_0_io3_io;
  wire spi_rtl_0_io3_o;
  wire spi_rtl_0_io3_t;
  wire ss_o_0;          //QSPI chip select output (internal)
  
  wire init_calib_complete_0;  //DDR3 initialization complete flag
  
  //Output assignments for control signals 
  assign eth_rst_n = 0; //Hold Ethernet in reset (always low, disables Ethernet)
  
  //Assign GPIO outputs to analog front-end controls 
  assign term = gpio_io_o_0[15:12];    //Termination control from GPIO (General Purpose input output) 15:12
  assign atten = gpio_io_o_0[19:16];   //Attenuator control from GPIO 19:16
  assign dc_cpl = ~gpio_io_o_0[23:20]; //DC counpling control (active low, so invert GPIO bits 23:20)
  
  //LED is ON (low) if either SERDES or DDR3 is not ready
  assign led = ~(serdes_ready & init_calib_complete_0); 
  
  //Assign GPIO outputs to acquisition and front-end enables 
  assign acq_en = gpio_io_o_0[24];  //Acquisition enable (GPIO bit 24)
  assign osc_oe = gpio_io_o_0[25];  //Oscillator output enable (GPIO bit 25)
  assign fe_en = gpio_io_o_0[26];   //Front-end enable (GPIO bit 26)
  
  //I2C open-drain emulation: drive low for '0', high-Z for '1' 
  assign i2c_sda = (i2c_sda_buf) ? (1'bz) : (1'b0); //If buffer is 1, float; else drive 0
  assign i2c_scl = (i2c_scl_buf) ? (1'bz) : (1'b0);

  //QSPI chip select open-drain emulation 
  assign qspi_cs = (ss_o_0) ? (1'bz) : (1'b0);

  //Generate a 10 kHz sync signal from divclk (125 MHz)
  reg[15:0] sync_counter;      //Counter for sync clock divider 
  reg sync_clk = 1'b0;         //Sync clock output 
  always @ (posedge divclk) begin
      if (sync_counter==16'h1869) begin //Divide 125MHz clk by 6250 per each period of 10kHz clock
          sync_counter <= 16'h0000;
          sync_clk <= ~sync_clk;        //Toggle sync clock 
      end
      else 
          sync_counter <= sync_counter + 1'b1;
  end
  assign sync = sync_clk;      //Output sync clock
  
  //Generate a 10 kHz probe compensation signal from axi_aclk (125 MHz)
  reg[15:0] probe_div_counter;      //Counter for probe compensation divider
  reg probe_div_clk = 1'b0;         //Probe compensation clock output
  always @ (posedge axi_aclk) begin
      if (probe_div_counter==16'h1869) begin //Divide 125MHz clk by 6250 per each period of 10kHz clock
          probe_div_counter <= 16'h0000;
          probe_div_clk <= ~probe_div_clk;   //Toggle probe compensation 
      end
      else 
          probe_div_counter <= probe_div_counter + 1'b1;
  end
  assign probe_comp = probe_div_clk;   //Output probe compensation clock 
  
  //Two's complement conversion for ADC data 
  reg[63:0] twos_comp;                 //Register for two's complement data 
  always @(*) begin 
    //For each byte, invert all bits except the MSB (sign bit)
    //Transformation on the deserialized ADC data to convert it into a two's complement format 
    //Data_deser - 64-bit bus - 8 bytes of ADC data 
    twos_comp[63:56] <= {data_deser[63],~data_deser[62:56]};   //For the highest byte, keep MSB(data_deser[63]), invert bits 62:56 
    twos_comp[55:48] <= {data_deser[55],~data_deser[54:48]};
    twos_comp[47:40] <= {data_deser[47],~data_deser[46:40]};
    twos_comp[39:32] <= {data_deser[39],~data_deser[38:32]};
    twos_comp[31:24] <= {data_deser[31],~data_deser[30:24]};
    twos_comp[23:16] <= {~data_deser[23],data_deser[22:16]};   //Invert MSB and keep the lower bits as-is - Intentional for a specific channel 
    twos_comp[15:8] <= {data_deser[15],~data_deser[14:8]};
    twos_comp[7:0] <= {data_deser[7],~data_deser[6:0]};
  end
  
  //Channel multiplexer for ADC data selection (with clock domain crossing)
  wire[1:0] channel_mux;        //Channel select (2 bits)
  reg [2:0] channel_mux_cdc_0;  //Synchronizer for channel_mux[0]
  reg [2:0] channel_mux_cdc_1;  //Synchronizer for channel_mux[1]
  
  always @(posedge divclk) begin
    //Synchronize channel select bits from GPIO to divclk domain
    //Clock domain crossing synchronizer for channel selection signals - Ensure channel selection bits from the GPIO safely transferred into divclk clock domain  
    channel_mux_cdc_0 <= { channel_mux_cdc_0[1:0], gpio_io_o_0[4] };
    channel_mux_cdc_1 <= { channel_mux_cdc_1[1:0], gpio_io_o_0[5] };
  end
  
  assign channel_mux = {channel_mux_cdc_1[2],channel_mux_cdc_0[2]};  //Form 2-bit mux
  
  //Select ADC data arrangement based on chennel_mux
  //Select how processed ADC data is arranged before sent to the DMA 
  always @(*)
    begin
        case(channel_mux) //2-bit signal - Order of tyhe ADC channels in the output data
            2'b00: adc_data <= {twos_comp[63:0]};  //twos_comp - 64-bit register - Two's complement converted ADC data -> 8Bytes (one for each channe)  
            2'b01: adc_data <= {twos_comp[63:56],twos_comp[31:24],twos_comp[55:48],twos_comp[23:16],twos_comp[47:40],twos_comp[15:8],twos_comp[39:32],twos_comp[7:0]};
            2'b10: adc_data <= {twos_comp[63:56],twos_comp[47:40],twos_comp[31:24],twos_comp[15:8],twos_comp[55:48],twos_comp[39:32],twos_comp[23:16],twos_comp[7:0]};		
            2'b11: adc_data <= {twos_comp[63:56],twos_comp[47:40],twos_comp[31:24],twos_comp[15:8],twos_comp[55:48],twos_comp[39:32],twos_comp[23:16],twos_comp[7:0]};
    endcase
  end
  
  /*
  //Directly assign the two's complement converted ADC data to adc_data
  assign adc_data = data_deser_twos_hand sizecomp;

  //Assign a fixed test pattern to adc_data for debugging or simulation  
  assign adc_data = {8'h77,8'h66,8'h55,8'h44,8'h33,8'h22,8'h11,8'h00};
  
  //Create a ramp counter for generating a test pattern
  reg[7:0] adc_ramp_counter;
  always @(posedge axi_aclk) begin
    //Reset the counter when S01_ARESETN is low, otherwise increment it every clock cycle
    if (!S01_ARESETN) 
        adc_ramp_counter <= 0;
    else
        adc_ramp_counter <= adc_ramp_counter + 1;
  end

  //Replicate the ramp counter value across all 8 bytes of adc_data for a ramp test pattern
  always @(*) begin
    adc_data <= {8{adc_ramp_counter}};
  end
  */
  
  //SERDES reset synchronizer (clock domain crossing)
  wire serdes_rst;                    //SERDES reset (active high)
  reg [2:0] serdes_rst_cdc = 3'b111;  //Synchronizer register
  always @(posedge divclk)
    serdes_rst_cdc <= { serdes_rst_cdc[1:0], ~gpio_io_o_0[2] }; //Synchronize SERDES RSTn from GPIO
  assign serdes_rst = serdes_rst_cdc[2];   //Output synchronized reset 
  
  //DDR3 ready synchronizer (clock domain crossing)
  wire ddr_ready;                   //DDR3 ready (synchronized)
  reg [2:0] ddr_ready_cdc = 3'b000; //Synchronizer register 
  always @(posedge divclk)
    ddr_ready_cdc <= { ddr_ready_cdc[1:0], init_calib_complete_0};
  assign ddr_ready = ddr_ready_cdc[2]; //Output synchronized ready 
  
  //SERDES module: Deserilaized ADC data from LVDS to parallel format 
  //Function: Converts high-speed serial ADC data to parallel for further processing
  serdes serdes (
	.rst            (serdes_rst),    //Reset input 
	.adc_lclk_p		(adc_lclk_p),      //ADC LVDS clock positive 
	.adc_lclk_n		(adc_lclk_n),      //ADC LVDS clock negative
	.adc_fclk_p		(adc_fclk_p),      //ADC frame clock positive 
	.adc_fclk_n		(adc_fclk_n),      //ADC frame clock negative
	.adc_data_p		(adc_data_p),      //ADC data positive (8 channels)
	.adc_data_n		(adc_data_n),      //ADC data negative (8 channels)
	.axi_aclk       (axi_aclk),      //AXI clock 
	.divclk			(divclk),            //Divided clock input 
	.data_deser		(data_deser),      //Deserialized data output 
	.ready          (serdes_ready)   //Ready flag output 
	);
  
  //ADC to DMA data mover: Moves ADC data to memory via AXI DMA
  //Function: Bridges ADC data to system memory using AXI DMA, manages data transfer and status
  adc_to_datamover adc_to_datamover (
    .axi_aclk(axi_aclk),                        //AXI clock 
    .axi_aresetn(axi_aresetn),                  //AXI reset (active low)
    .S01_ARESETN(S01_ARESETN),                  //AXI reset (active low)
    .axis_cmd_tready(S_AXIS_S2MM_CMD_tready),   //DMA command ready 
    .axis_cmd_tdata(S_AXIS_S2MM_CMD_tdata),     //DMA command data 
    .axis_cmd_tvalid(S_AXIS_S2MM_CMD_tvalid),   //DMA command valid 
    .axis_data_tready(S_AXIS_S2MM_tready),      //DMA data ready 
    .axis_data_tdata(S_AXIS_S2MM_tdata),        //DMA data 
    .axis_data_tvalid(S_AXIS_S2MM_tvalid),      //DMA data valid
    .axis_data_tlast(S_AXIS_S2MM_tlast),        //DMA data last 
    .adc_data(adc_data),                        //DMA data input
    .adc_divclk(divclk),                        //DMA divided clock 
    .s2mm_err(s2mm_err),                        //DMA error
    .s2mm_halt(s2mm_halt),                      //DMA halt 
    .s2mm_halt_cmplt(s2mm_halt_cmplt),          //DMA halt complete 
    .s2mm_wr_xfer_cmplt(s2mm_wr_xfer_cmplt),    //DMA write transfer complete 
    .gpio_io_o_0(gpio_io_o_0),                  //GPIO output 
    .gpio2_io_i(gpio2_io_i),                    //GPIO input 
    .gpio2_io_o_0(gpio2_io_o_0),                //GPIO output 2
    .serdes_ready (serdes_ready),               //SERDES ready 
    .ddr_ready(ddr_ready)                       //DDR3 ready 
  );
  
  //Serial controller: Handles I2C, SPI, and other serial comms
  //Function: Manages configuration/comtrol of analog front-end and periperals via serial protocols 
  serial_controller  serial_controller(
    .clk(axi_aclk),                              //System clock 
    .rst(axi_aresetn),                           //System reset (active low)
    .AXI_STR_TXD_0_tdata(AXI_STR_TXD_0_tdata),   //AXI stream data
    .AXI_STR_TXD_0_tready(AXI_STR_TXD_0_tready), //AXI stream ready 
    .AXI_STR_TXD_0_tvalid(AXI_STR_TXD_0_tvalid), //AXI stream valid 
    .i2c_sda(i2c_sda_buf),                       //I2C SDA buffer 
    .i2c_scl(i2c_scl_buf),                       //I2C SCL buffer 
    .spi_sdio(spi_sdio),                         //SPI data 
    .spi_sclk(spi_sclk),                         //SPI clock 
    .pga_cs(pga_cs),                             //PGA chip select 
    .adc_cs(adc_cs)                              //ADC chip select 
    );
  

  //Top-level design wrapper: Connects all major blocks and external interfaces
  //Function: Integrates all subsystems and external interfeaces, acting as the main system backbone
  design_1 design_1_i
   (
    .AXI_STR_TXD_0_tdata(AXI_STR_TXD_0_tdata),       //AXI stream data 
    .AXI_STR_TXD_0_tlast(AXI_STR_TXD_0_tlast),       //AXI stream last 
    .AXI_STR_TXD_0_tready(AXI_STR_TXD_0_tready),     //AXI stream ready 
    .AXI_STR_TXD_0_tvalid(AXI_STR_TXD_0_tvalid),     //AXI stream valid
    .DDR3_addr(DDR3_addr),                           //DDR3 address 
    .DDR3_ba(DDR3_ba),                               //DDR3 bank address 
    .DDR3_cas_n(DDR3_cas_n),                         //DDR3 CAS 
    .DDR3_ck_n(DDR3_ck_n),                           //DDR3 clock negative
    .DDR3_ck_p(DDR3_ck_p),                           //DDR3 clock positive
    .DDR3_cke(DDR3_cke),                             //DDR3 clock enable 
    .DDR3_cs_n(DDR3_cs_n),                           //DDR3 chip select 
    .DDR3_dm(DDR3_dm),                               //DDR3 data mask 
    .DDR3_dq(DDR3_dq),                               //DDR3 data 
    .DDR3_dqs_n(DDR3_dqs_n),                         //DDR3 data strobe negative 
    .DDR3_dqs_p(DDR3_dqs_p),                         //DDR3 data stribe positive 
    .DDR3_odt(DDR3_odt),                             //DDR3 data on-die termination 
    .DDR3_ras_n(DDR3_ras_n),                         //DDR3 RAS 
    .DDR3_reset_n(DDR3_reset_n),                     //DDR3 reset 
    .DDR3_we_n(DDR3_we_n),                           //DDR3 write enable 
    .S01_ARESETN(S01_ARESETN),                       //AXI reset 
    .S_AXIS_S2MM_CMD_tdata(S_AXIS_S2MM_CMD_tdata),   //DMA coommand data 
    .S_AXIS_S2MM_CMD_tready(S_AXIS_S2MM_CMD_tready), //DMA command ready
    .S_AXIS_S2MM_CMD_tvalid(S_AXIS_S2MM_CMD_tvalid), //DMA command valid
    .S_AXIS_S2MM_tdata(S_AXIS_S2MM_tdata),           //DMA data 
    .S_AXIS_S2MM_tkeep(16'hFFFF),                    //DMA data keep (all bytes valid) 
    .S_AXIS_S2MM_tready(S_AXIS_S2MM_tready),         //DMA data ready 
    .S_AXIS_S2MM_tvalid(S_AXIS_S2MM_tvalid),         //DMA data valid 
    .S_AXIS_S2MM_tlast(S_AXIS_S2MM_tlast),           //DMA data last 
    .axi_aclk(axi_aclk),                             //AXI clock 
    .axi_aresetn(axi_aresetn),                       //AXI reset 
    .gpio2_io_i(gpio2_io_i),                         //GPIO input
    .gpio2_io_o_0(gpio2_io_o_0),                     //GPIO output 2
    .gpio_io_o_0(gpio_io_o_0),                       //GPIO output 
    .pcie_clk_n(pcie_clk_n),                         //PCIe clock negative 
    .pcie_clk_p(pcie_clk_p),                         //PCIe clock positive 
    .pcie_mgt_rxn(pcie_mgt_rxn),                     //PCIe RX negative 
    .pcie_mgt_rxp(pcie_mgt_rxp),                     //PCIe RX positive 
    .pcie_mgt_txn(pcie_mgt_txn),                     //PCIe TX negative 
    .pcie_mgt_txp(pcie_mgt_txp),                     //PCIe TX positive 
    .pcie_perstn(pcie_perstn),                       //PCIe reset 
    .s2mm_err(s2mm_err),                             //DMA error 
    .s2mm_halt(s2mm_halt),                           //DMA halt 
    .s2mm_halt_cmplt(s2mm_halt_cmplt),               //DMA halt complete 
    .s2mm_wr_xfer_cmplt(s2mm_wr_xfer_cmplt),         //DMA write transfer complete
    .Vp_Vn_0_v_n(Vp_Vn_0_v_n),                       //Analog probe negative 
    .Vp_Vn_0_v_p(Vp_Vn_0_v_p),                       //Analog probe positive
    .SPI_0_0_io0_i(spi_rtl_0_io0_i),                 //QSPI IO0 input 
    .SPI_0_0_io0_o(spi_rtl_0_io0_o),                 //QSPI IO0 output
    .SPI_0_0_io0_t(spi_rtl_0_io0_t),                 //QSPI IO0 tristate 
    .SPI_0_0_io1_i(spi_rtl_0_io1_i),                 //QSPI IO1 input 
    .SPI_0_0_io1_o(spi_rtl_0_io1_o),                 //QSPI IO1 output 
    .SPI_0_0_io1_t(spi_rtl_0_io1_t),                 //QSPI IO1 tristate 
    .SPI_0_0_io2_i(spi_rtl_0_io2_i),                 //QSPI IO2 input 
    .SPI_0_0_io2_o(spi_rtl_0_io2_o),                 //QSPI IO2 output 
    .SPI_0_0_io2_t(spi_rtl_0_io2_t),                 //QSPI IO2 tristate
    .SPI_0_0_io3_i(spi_rtl_0_io3_i),                 //QSPI IO3 input 
    .SPI_0_0_io3_o(spi_rtl_0_io3_o),                 //QSPI IO3 output 
    .SPI_0_0_io3_t(spi_rtl_0_io3_t),                 //QSPI IO3 tristate 
    .SPI_0_0_ss_t(),                                 //QSPI slave select tristate (unused)
    .ss_o_0(ss_o_0),                                 //QSPI chip select output 
    .init_calib_complete_0(init_calib_complete_0),   //DDR3 init complete 
    .ext_clk_p(ext_clk_p),                           //External clock positive 
    .ext_clk_n(ext_clk_n)                            //External clock negative
    );

//QSPI IO buffers for bidirectional data lines
//Function: Allow QSPI lines to be used for both input and output, supporting flash memory access
IOBUF spi_rtl_0_io0_iobuf    
    (.I(spi_rtl_0_io0_o),    //Output data to QSPI IO0 
    .IO(qspi_d0),            //Bidirectional QSPI IO0 pin 
    .O(spi_rtl_0_io0_i),     //Input data from QSPI IO0
    .T(spi_rtl_0_io0_t));    //Tristate control for IO0 

IOBUF spi_rtl_0_io1_iobuf   
    (.I(spi_rtl_0_io1_o),    //Output data to QSPI IO1 
    .IO(qspi_d1),            //Bidirectional QSPI IO1 pin 
    .O(spi_rtl_0_io1_i),     //Input data from QSPI IO1 
    .T(spi_rtl_0_io1_t));    //Tristate control for IO1

IOBUF spi_rtl_0_io2_iobuf    
    (.I(spi_rtl_0_io2_o),    //Output data to QSPI IO2  
    .IO(qspi_d2),            //Bidirectional QSPI IO2 pin
    .O(spi_rtl_0_io2_i),     //Input data from QSPI IO2 
    .T(spi_rtl_0_io2_t));    //Tristate control for IO2 

IOBUF spi_rtl_0_io3_iobuf   
    (.I(spi_rtl_0_io3_o),    //Ouput data to QSPI IO3 
    .IO(qspi_d3),            //Bidrectional QSPI IO3 
    .O(spi_rtl_0_io3_i),     //Input data from QSPI IO3 
    .T(spi_rtl_0_io3_t));    //Tristate control for IO3
endmodule