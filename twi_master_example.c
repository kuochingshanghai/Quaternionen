//-----------------------IMU------------------------------------------///


#include "defines.h"


//------------------  C O N F I G U R A T I O N S IMU3000 -------------------

#define EEPROM_ADDRESS        0x68        // EEPROM's TWI address
#define EEPROM_ADDR_LGT       1           // Address length of the EEPROM memory
#define VIRTUALMEM_ADDR_START 0x68    // Address of the virtual mem in the EEPROM
#define TWI_SPEED             50000       // Speed of TWI

//------------------  D E F I N I T I O N S IMU3000 -------------------




 // USART Fix Parameters
 #define USART_0		                 	(&AVR32_USART0) ////!!!!!!!!!!!USART0 bei interrupt und USART1 bei anderer ausgabe!!!!!
 #define USART_0_IRQ             			AVR32_USART0_IRQ
 #define USART_BAUDRATE      		  		57600

 // TC Fix Parameters
 #define RC									(FOSC0/ 8 / 1000)
 #define TC_CHANNEL   						0
 volatile avr32_tc_t *tc = &AVR32_TC;
 int tc_tick;					// Time in ms

 //gyro Werte in °/s
volatile float gx =0;
volatile float gy= 0;
volatile float gz = 0;
//gyro Werte in ° absolut
volatile float gxDeg = 0;
volatile float gyDeg = 0;
volatile float gzDeg = 0;
//gyro Wert in d°(veränderung des Winkel)
volatile float dGx = 0;
volatile float dGy = 0;
volatile float dGz = 0;



volatile bool exe = false;


 // ISR, USART INTC: Speichere hier in den Ringbuffer
 __attribute__((__interrupt__))	// Folgende Funktion ist Interrupt
 static void usart_int_handler(void)
 {
 	int c;	// Empfangenes Datum

 	usart_read_char(USART_0, &c);
 	//INTERRUPT CODE USART


 }

 // ISR, TC: Zähle hier den Counter hoch
  // Attribute nicht vergessen!
__attribute__((__interrupt__))
static void tc_irq(void)
{
	// Increment the ms seconds counter
	tc_tick++;
	if(tc_tick % 30 == 0){
		exe = true;

	}


	// Clear the interrupt flag. This is a side effect of reading the TC SR.
	tc_read_sr(tc, TC_CHANNEL);
}





int main(void)
{



	//-------------------------USART INTERRUPT REGISTRATION.------------//
	// Set Clock: Oscillator needs to initialized once: First
		 pcl_switch_to_osc(PCL_OSC0, FOSC0, OSC0_STARTUP);

		 // --------------		USART INIT		-----------------------------------------------
		 static const gpio_map_t USART_GPIO_MAP =
		  {
			{AVR32_USART0_RXD_0_0_PIN, AVR32_USART0_RXD_0_0_FUNCTION},
			{AVR32_USART0_TXD_0_0_PIN, AVR32_USART0_TXD_0_0_FUNCTION}
		  };

		  // USART options.
		  static const usart_options_t USART_OPTIONS =
		  {
			.baudrate     = USART_BAUDRATE,
			.charlength   = 8,
			.paritytype   = USART_NO_PARITY,
			.stopbits     = USART_1_STOPBIT,
			.channelmode  = USART_NORMAL_CHMODE
		  };

		// Assign GPIO to USART
		gpio_enable_module(USART_GPIO_MAP, sizeof(USART_GPIO_MAP) / sizeof(USART_GPIO_MAP[0]));

		// Init USART
		usart_init_rs232(USART_0, &USART_OPTIONS, FOSC0);

		Disable_global_interrupt();
		INTC_init_interrupts();			// Init Interrupt Table: Once at first

		// Register USART Interrupt (hinzufügen)
		INTC_register_interrupt(&usart_int_handler, AVR32_USART0_IRQ, AVR32_INTC_INT0);

		USART_0->ier = AVR32_USART_IER_RXRDY_MASK; // Activate ISR on RX Line
		Enable_global_interrupt();
	// -----------------------------------------------------------------------------------

		// --------------------------		Display INIT		----------------------------------
			// Map SPI Pins
			static const gpio_map_t DIP204_SPI_GPIO_MAP =
			  {
				{DIP204_SPI_SCK_PIN,  DIP204_SPI_SCK_FUNCTION },  // SPI Clock.
				{DIP204_SPI_MISO_PIN, DIP204_SPI_MISO_FUNCTION},  // MISO.
				{DIP204_SPI_MOSI_PIN, DIP204_SPI_MOSI_FUNCTION},  // MOSI.
				{DIP204_SPI_NPCS_PIN, DIP204_SPI_NPCS_FUNCTION}   // Chip Select NPCS.
			  };

			// add the spi options driver structure for the LCD DIP204
			  spi_options_t spiOptions =
			  {
				.reg          = DIP204_SPI_NPCS,
				.baudrate     = 1000000,
				.bits         = 8,
				.spck_delay   = 0,
				.trans_delay  = 0,
				.stay_act     = 1,
				.spi_mode     = 0,
				.modfdis      = 1
			  };


			// SPI Inits: Assign I/Os to SPI
			gpio_enable_module(DIP204_SPI_GPIO_MAP,
			                     sizeof(DIP204_SPI_GPIO_MAP) / sizeof(DIP204_SPI_GPIO_MAP[0]));

			// Initialize as master
			spi_initMaster(DIP204_SPI, &spiOptions);

			// Set selection mode: variable_ps, pcs_decode, delay
			spi_selectionMode(DIP204_SPI, 0, 0, 0);

			// Enable SPI
			spi_enable(DIP204_SPI);

			// setup chip registers
			spi_setupChipReg(DIP204_SPI, &spiOptions, FOSC0);

			// initialize delay driver: Muss vor dip204_init() ausgeführt werden
			delay_init( FOSC0 );

			// initialize LCD
			dip204_init(backlight_PWM, TRUE);
			// ---------------------------------------------------------------------------------------

			// -----------------			Timer Counter Init		---------------------------------
				// Timer Configs:  Options for waveform generation.
				static const tc_waveform_opt_t WAVEFORM_OPT =
				{
				.channel  = TC_CHANNEL,                        // Channel selection.

				.bswtrg   = TC_EVT_EFFECT_NOOP,                // Software trigger effect on TIOB.
				.beevt    = TC_EVT_EFFECT_NOOP,                // External event effect on TIOB.
				.bcpc     = TC_EVT_EFFECT_NOOP,                // RC compare effect on TIOB.
				.bcpb     = TC_EVT_EFFECT_NOOP,                // RB compare effect on TIOB.

				.aswtrg   = TC_EVT_EFFECT_NOOP,                // Software trigger effect on TIOA.
				.aeevt    = TC_EVT_EFFECT_NOOP,                // External event effect on TIOA.
				.acpc     = TC_EVT_EFFECT_NOOP,                // RC compare effect on TIOA: toggle.
				.acpa     = TC_EVT_EFFECT_NOOP,                // RA compare effect on TIOA: toggle

				.wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,//  Count till RC and reset (S. 649): Waveform selection
				.enetrg   = FALSE,                             // External event trigger enable.
				.eevt     = 0,                                 // External event selection.
				.eevtedg  = TC_SEL_NO_EDGE,                    // External event edge selection.
				.cpcdis   = FALSE,                             // Counter disable when RC compare.
				.cpcstop  = FALSE,                             // Counter clock stopped with RC compare.

				.burst    = FALSE,                             // Burst signal selection.
				.clki     = FALSE,                             // Clock inversion.
				.tcclks   = TC_CLOCK_SOURCE_TC3                // Internal source clock 3, connected to fPBA / 8.
				};


				// TC Interrupt Enable Register
				static const tc_interrupt_t TC_INTERRUPT =
				{ .etrgs = 0, .ldrbs = 0, .ldras = 0, .cpcs  = 1, .cpbs  = 0, .cpas  = 0, .lovrs = 0, .covfs = 0
				};
				// 0 = No Effect | 1 = Enable ( CPCS = 1 enables the RC Compare Interrupt )

				// *****************   Timer Setup ***********************************************
				// Initialize the timer/counter.
				tc_init_waveform(tc, &WAVEFORM_OPT);         // Initialize the timer/counter waveform.

				// Set the compare triggers.
				tc_write_rc(tc, TC_CHANNEL, RC); // Set RC value.

				tc_configure_interrupts(tc, TC_CHANNEL, &TC_INTERRUPT);

				// Start the timer/counter.
				tc_start(tc, TC_CHANNEL);                    // And start the timer/counter.
				// *******************************************************************************

				Disable_global_interrupt();
				// Register TC Interrupt
				INTC_register_interrupt(&tc_irq, AVR32_TC_IRQ0, AVR32_INTC_INT3);

				Enable_global_interrupt();
				// ---------------------------------------------------------------------------------------



				imu_init();

//-------------------------------TWI R/W ---------------------------------------------------



  sensorDaten imu_data = {0};
  char disp1[30], disp2[30], disp3[30], disp4[30];
  short GX,GY,GZ, AX, AY, AZ;			//shifted comlete Data
  RPY currMoveRPY;
  Quaternion currQuat;
  currQuat.q0 = 1.0;
  currQuat.q1 = 0;
  currQuat.q2 = 0;
  currQuat.q3 = 0;
  Quaternion deltaQuat;
  RotMatrix rot = {0};
  RPY reconverted;


  calibrate_all(&imu_data);


  while(1){
	  if(exe){
		  exe = false;
		  read_sensor(&imu_data);




		  AX = imu_data.acc_x + imu_data.acc_x_bias;
		  AY = imu_data.acc_y + imu_data.acc_y_bias;
		  AZ = imu_data.acc_z + imu_data.acc_z_bias;

		  GX = imu_data.gyro_x + imu_data.gyro_x_bias;
		  GY = imu_data.gyro_y + imu_data.gyro_y_bias;
		  GZ = imu_data.gyro_z + imu_data.gyro_z_bias;




		  //convert to 1G
		  float ax = (float)AX * (-4.0);
		  float ay = (float)AY * (-4.0); //wegen 2^11= 2048, /2 = 1024 entspricht 4G -> 1G = (1024/4)
		  float az = (float)AZ * (-4.0);


		  //convert to 1°/s
		  gx = ((float)GX/ 14.375); // in °/s
		  gy = ((float)GY/ 14.375);
		  gz = ((float)GZ/ 14.375);

		  //Integration over time
		  dGx = (gx*0.03);
		  dGy = (gy*0.03);
		  dGz = (gz*0.03);

		  currMoveRPY.pitch = -dGx;
		  currMoveRPY.roll = dGy;
		  currMoveRPY.yaw = dGz;


		  //aufaddieren auf den aktuellen Winkel IN GRAD
			gxDeg += dGx;
			gyDeg += dGy;
			gzDeg += dGz;


			//RPY in Quaternion umwandeln
			RPYtoQuat(&deltaQuat, &currMoveRPY);


			//normieren
			normQuat(&deltaQuat);


			//aufmultiplizeiren
			quatMultiplication(&deltaQuat, &currQuat, &currQuat);



			//nochmal normieren
			normQuat(&currQuat);

			//rücktransformation nicht nötig!!


			char send[80];
			sprintf(send,"$,%f,%f,%f,%f,#", currQuat.q0, currQuat.q1, currQuat.q2, currQuat.q3);
		 	usart_write_line(USART_0,send);



		   sprintf(disp1,"q0:%.3f, GX:%3.0f",currQuat.q0,gxDeg);
		   sprintf(disp2,"q1:%.3f, GY:%3.0f",currQuat.q1, gyDeg);
		   sprintf(disp3,"q2:%.3f, GZ:%3.0f",currQuat.q2, gzDeg);
		   sprintf(disp4,"q3:%.3f",currQuat.q3);



		   dip204_clear_display();

		   dip204_set_cursor_position(1,1);
		   dip204_write_string(disp1);
		   dip204_set_cursor_position(1,2);
		   dip204_write_string(disp2);
		   dip204_set_cursor_position(1,3);
		   dip204_write_string(disp3);
		   dip204_set_cursor_position(1,4);
		   dip204_write_string(disp4);











			//sprintf(data,"TEST:%s",high);
		   //print_dbg(data);
	  }

  }
}
