//*****************************************************************************
// Program:	 ROHM I2C Emulator Application
//		 ROHM Semiconductor USA, LLC
//		 US Design Center
// Date:  July 21st, 2015
// Purpose:	 Firmware for Q112 for I2C Emulator
// Updated:	 July 8th, 2014
/*
	Copyright (C) 2015 ROHM USDC Applications Engineering Team

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
//*****************************************************************************
#define DebugOn

//***** PREPROCESSOR DIRECTIVES ***********************************************
// INCLUDED FILES...
// Include Path: common;main;irq;timer;clock;tbc;pwm;uart;

	#include	<ML610112.H>	// Lapis Micro ML610Q112 on LaPi Development Board
	#include	<stdlib.h>		// General-purpose utilities
	#include 	<uart.h>		// UART Function Prototypes
	#include 	<common.h>		// Common Definitions
	#include 	<irq.h>			// IRQ Definitions
	#include 	<mcu.h>			// MCU Definition
	#include	<i2c.h>			// I2C Definition
	//#include 	<clock.h>		// Set System Clock API
	#include 	<tbc.h>			// Set TBC (Timer Based Clock) API
	#include 	<timer.h>		// Timer Macros & APIs
	//#include 	<main.h>		// Clear WDT API
	//#include	<ctype.h>		// Character classification and conversion 
	//#include	<errno.h>		// Error identifiers Library
	//#include	<float.h>		// Numerical limits for floating-point numbers
	//#include	<limits.h>		// Numerical limits for integers
	//#include	<math.h>		// Mathematical functions
	//#include	<muldivu8.h>	// Multiplication and Division accelerator
	//#include	<setjmp.h>		// Global jump (longjmp)
	//#include	<signal.h>		// Signal handling functions
	//#include	<stdarg.h>		// Variable numbers of arguments
	//#include	<stddef.h>		// Standard types and macros 
	#include	<stdio.h>		// I/O-related processing
	//#include	<string.h>		// Character string manipulation routines
	//#include	<yfuns.h>		// 
	//#include	<yvals.h>		// Called for by most Header Files

//===========================================================================
//   MACROS: 
//===========================================================================

#define WelcomeString	("\033[2J\033[1;1H"\
	"*********************************************\n\r"\
	"** Q112 Firmware - Sensor Platform EVK\n\r"\
	"** Revision    : REV00\n\r"\
	"** Release date: " __DATE__ " " __TIME__ "\n\r"\
	"** By          : ROHM Semiconductor USA, LLC\n\r"\
	"*********************************************\n\r"\
)
#define PRINTF(msg)		write(0, msg, sizeof(msg))

// ===== Peripheral setting.=====
#define HSCLK_KHZ	( 8000u )	// 8MHz = 8000kHz (will be multiplied by 1024 to give 8,192,000Hz)
#define FLG_SET		( 0x01u )

// SET DESIRED UART SETTINGS HERE! (Options in UART.h)
//#define UART_BAUDRATE		( UART_BR_115200BPS) 	// Data Bits Per Second - Tested at rates from 2400bps to 512000bps!
//#define UART_BAUDRATE		( UART_BR_9600BPS) 	// Data Bits Per Second - Tested at rates from 2400bps to 512000bps!
#define UART_BAUDRATE		( UART_BR_9600BPS) 	// Data Bits Per Second - Tested at rates from 2400bps to 512000bps!
#define UART_DATA_LENGTH	( UART_LG_8BIT )		// x-Bit Data
#define UART_PARITY_BIT		( UART_PT_NON )		// Parity
#define UART_STOP_BIT		( UART_STP_1BIT )		// x Stop-Bits
#define UART_LOGIC			( UART_NEG_POS )		// Desired Logic
#define UART_DIRECTION		( UART_DIR_LSB )		// LSB or MSB First
//#define _TBC_H_

/**
 * Sensor Interface Header 1
 */
#define SENINTF_HDR1_GPIO0(reg)		PB2##reg
#define SENINTF_HDR1_GPIO1(reg)		PB3##reg
#define SENINTF_HDR1_GPIO2(reg)		PB4##reg
#define SENINTF_HDR1_GPIO3(reg)		PB7##reg

/**
 * LED[7-0]
 */
#define LEDOUT(x)	PCD=x

//===========================================================================
//   STRUCTURES:    
//===========================================================================
static const tUartSetParam  _uartSetParam = {		// UART Parameters
	UART_BAUDRATE,								// Members of Structure...
	UART_DATA_LENGTH,							// Members of Structure...
	UART_PARITY_BIT,							// Members of Structure...
	UART_STOP_BIT,								// Members of Structure...
	UART_LOGIC,									// Members of Structure...
	UART_DIRECTION								// Members of Structure...
};

//===========================================================================
//   FUNCTION PROTOTYPES: 
//	Establishes the name and return type of a function and may specify the 
// 	types, formal parameter names and number of arguments to the function                                 
//===========================================================================
void main_clrWDT( void );			// no return value and no arguments
void Initialization( void );		// no return value and no arguments
void SetOSC( void );				// no return value and no arguments
void PortA_Low( void );				// no return value and no arguments
void PortB_Low( void );				// no return value and no arguments
void PortC_Low( void );				// no return value and no arguments
void PortD_Low( void );				// no return value and no arguments

//RTLU8: Low-level function
int write(int handle, unsigned char *buffer, unsigned int len);
int ADC_Read(unsigned char idx);
void I2C_Read(unsigned char slave_address, unsigned char *reg_address, unsigned char reg_address_size, unsigned char *buffer, unsigned char size);
void I2C_Write(unsigned char slave_address, unsigned char *reg_address, unsigned char reg_address_size, unsigned char *buffer, unsigned char size);

//UART and I2C Functions
void _funcUartFin( unsigned int size, unsigned char errStat );
void _funcI2CFin( unsigned int size, unsigned char errStat );
void main_reqNotHalt( void );
void _intUart( void );
void _intI2c( void );
void _intADC( void );
void _intPB2( void );
void NOPms( unsigned int ms );

void Init_EEPROM(void);

unsigned char ReverseBits(unsigned char data);
unsigned char FlashLEDs(void);

// I2C device address of EEPROM
const unsigned char BR24_I2C_ADDR		= 0x50u;
// Register addresses of KX022
const unsigned char BR24_REG00[2]				= {0x00u, 0x00u};
const unsigned char BR24_REG10				= 0x10u;

const unsigned char BR24_REG00_Contents[256]	= { 0x1F,0xA0,0x00,0x07,0x00,0x00,0x00,0x0F,0x4E,0x07,
													0xC0,0xC2,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,
													0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
													0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
													0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x00,
													0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x00,0x00,0x00,
													0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x26,0x00,
													0x51,0x00,0x28,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
													0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
													0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
													0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
													0x00,0x00,0x00 };


//const unsigned char BR24_REG00_Contents[13]	= { 0x1F, 0xA0, 0x00, 0x07, 0x00, 0x00, 0x00, 0x0F, 0x4E, 0x07, 0xC0, 0xC2, 0x00 };
/*
const unsigned char BR24_REG0F				= 0x0Au;
const unsigned char BR24_REG0F_Contents		= 0x01;
const unsigned char BR24_REG30				= 0x30u;
const unsigned char BR24_REG30_Contents[9]	= { 0x18, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x08 };
const unsigned char BR24_REG40				= 0x40u;
const unsigned char BR24_REG40_Contents[13]	= { 0x00, 0x00, 0x00, 0x00, 0x26, 0x00, 0x51, 0x00, 0x28, 0x00, 0x00, 0x00, 0x00 };
const unsigned char BR24_REG50				= 0x50u;
const unsigned char BR24_REG50_Contents[12]	= { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
const unsigned char BR24_REG60				= 0x60u;
const unsigned char BR24_REG60_Contents[3]	= { 0x00, 0x00, 0x00 };
const unsigned char BR24_REG70				= 0x70u;
const unsigned char BR24_REG70_Contents		= 0x00;
*/

unsigned char	Test00_Return[256];

//*****************************************************************************
// GLOBALS...
// ADC, UART and I2C Variables
unsigned char	_flgUartFin;
unsigned char 	_flgI2CFin;
unsigned char	_flgADCFin;
unsigned char	_flgPB2Int = 0;
unsigned char	_reqNotHalt;

union {
	unsigned char	_uchar;
	unsigned char	_ucharArr[6];
	unsigned int	_uint;
	unsigned int	_uintArr[3];
	int				_intArr[3];
	float			_float;
} uniRawSensorOut;

float flSensorOut[3];

/**
 * ANSI Escape Code
 */
#define ESC_SOL			"\r"
#define ESC_NEWLINE		"\n\r"
#define ESC_PREVLINE	"\033[F"
#define ESC_ERASE2END	"\033[J"

//===========================================================================
//  	Start of MAIN FUNCTION
//===========================================================================
int main(void) 
{ 	
	Initialization(); //Ports, UART, Timers, Oscillator, Comparators, etc.
	#ifdef DebugOn
	PRINTF("Start Program");
	#endif
	I2C_Write(BR24_I2C_ADDR, &BR24_REG00, 2, &BR24_REG00_Contents, 255);
	
MainLoop:
	main_clrWDT();
	
	I2C_Read(BR24_I2C_ADDR, &BR24_REG00, 2, &Test00_Return, 255);
	
	HLT = 1;	//Wait time here depends on the WDT timing
	__asm("nop\n"); 
	__asm("nop\n");
	
	goto MainLoop;
}
//===========================================================================
//  	End of MAIN FUNCTION
//===========================================================================


//===========================================================================
//  	Start of Other Functions...
//===========================================================================
//==========================================================================
//	Initialize Micro to Desired State...
//===========================================================================
static void Initialization(void){

	//Initialize Peripherals	
	//BLKCON2 Control Bits...Manually Set 4/12/2013
	DSIO0 = 1; // 0=> Enables Synchronous Serial Port 0 (initial value).
	#ifdef DebugOn
	DUA0  = 0; // 0=> Enables the operation of UART0 (initial value).
	#endif
	#ifndef DebugOn
	DUA0  = 1; // 0=> Enables the operation of UART0 (initial value).
	#endif
	DUA1  = 1; // 0=> Enables Uart1 (initial value). 
	DI2C1 = 1; // 0=> Enables I2C bus Interface (Slave) (initial value).
	DI2C0 = 0; // 0=> Enables I2C bus Interface (Master) (initial value).	
	
	BLKCON4 = 0x01; // 0=> Enables SA-ADC
	BLKCON6 = 0xC3; // (1=disables; 0=enables) the operation of Timers 8, 9, A, E, F.
					// only timer AB are enabled
	BLKCON7 = 0x0F; // (1=disables; 0=enables) the operation of PWM (PWMC, PWMD, PWME, PWMF

	// Port Initialize
	PortA_Low();	//Initialize all 3 Ports of Port A to GPIO-Low
	PortB_Low();	//Initialize all 8 Ports of Port B to GPIO-Low
	PortC_Low();	//Initialize all 8 Ports of Port C to GPIO-Low
	PortD_Low();	//Initialize all 6 Ports of Port D to input GPIO
	
	// Set Oscillator Rate
    SetOSC();
	
	// Settings for the ADC input (A0, A1)
	PA0DIR = 1;
	PA1DIR = 1;		//GPIO Input
	//SACH0 = 1;		//This enables the ADC Channel 0 from A0 Pin
	//SACH1 = 1;		//This enables the ADC Channel 1 from A1 Pin
	//SALP = 0;		//Single Read or Continuous Read... Single = 0, Consecutive = 1
	
	// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
	// INTERRUPT SETUP...
	irq_di();	// Disable Interrupts
	irq_init();	// Initialize Interrupts (All Off and NO Requests)

	// INTERRUPT ENABLE REGISTERS...
	IE0 = IE1 = IE2 = IE3 = IE4 = IE5 = IE6 = IE7 = 0;
	// INTERRUPT REQUEST REGISTERS...
	IRQ0 = IRQ1 = IRQ2 = IRQ3 = IRQ4 = IRQ5 = IRQ6 = IRQ7 = 0;

	E2H = 0;	// E2H is the Enable flag for 2Hz TBC Interrupt (1=ENABLED)
		
	#ifdef DebugOn		
	irq_setHdr((unsigned char)IRQ_NO_UA0INT, _intUart);
	EUA0 = 1; 	// EUA0 is the enable flag for the UART0 interrupt (1=ENABLED)
	#endif
	
	irq_setHdr((unsigned char)IRQ_NO_I2CMINT, _intI2c);
	EI2CM = 1;
	QI2CM = 0;
	
	//Enable ADC Interrupts Handler
	/*
	irq_setHdr((unsigned char)IRQ_NO_SADINT, _intADC);
	ESAD = 1;
	QSAD = 0;
	*/
	
	/*
	//Need to setup PB2 as external interrupt pin
	PB2MD0 = 0;
	PB2MD1 = 0;
	PB2DIR = 1;
	PB2C0 = 0;
	PB2C1 = 1;
	PB2E0 = 1;
	PB2E1 = 0;
	
	//Setup the Callback Function for External interrupt on PB2 for Accel
	irq_setHdr((unsigned char)IRQ_NO_PB2INT, _intPB2);
	EPB2 = 1;
	QPB2 = 0;
	*/	
	
	/*
	//Set up xHz TBC Interrupt (Options: 128Hz, 32Hz, 16Hz, 2Hz)
	//(void)irq_setHdr( (unsigned char)IRQ_NO_T2HINT, TBC_ISR );  //Clear interrupt request flag
	
	// TBC...Set Ratio: : 1:1 => 1_1
	(void)tb_setHtbdiv( (unsigned char)TB_HTD_1_1 ); //Set the ratio of dividing frequency of the time base counter
		E2H = 0;	  // Enable x Hz TBC Interrupt (1=ENABLED)
		Q2H = 0;	  // Request flag for the time base counter x Hz interrupt	
	*/
	
	irq_ei(); // Enable Interrupts
	// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

	// WDT... This will be the triggering condition to return from halt mode
	// We will need to calibrate based on the timing of our loop
	// 0x00 = 125ms
	// 0x01 = 500ms
	// 0x02 = 2sec
	// 0x03 = 8sec
	// 0x04 = 23.4ms
	// 0x05 = 31.25ms
	// 0x06	= 62.5ms
	WDTMOD = 0x01; 	
	main_clrWDT(); 	// Clear WDT
	
	//I2C Initialization...
	//P20C0 = 1;	/* CMOS output */
	//P20C1 = 1;	
	//P20D = 1;		/* write protect enable */
	i2c_init(I2C_MOD_FST, (unsigned short)HSCLK_KHZ, I2C_SYN_OFF);
	
	//UART Initialization...
	uart_init((unsigned char)UART_CS_HSCLK,		/* Generator       */
			  (unsigned short)HSCLK_KHZ,		/* HSCLK frequency */
			   &_uartSetParam );				/* Param... 	 */
	uart_PortSet();
	
	//Initialize GPIO3 for Garage Controller
	PB7DIR = 0;		// PortB Bit7 set to Output Mode...
	PB7C1  = 1;		// PortB Bit7 set to CMOS Output...
	PB7C0  = 1;	
	PB7MD1  = 0;	// PortB Bit7 set to General Purpose Output...
	PB7MD0  = 0;
	PB7D = 0;		// B.7 Output OFF....
	
}//End Initialization
//===========================================================================

/*******************************************************************************
	Routine Name	: write
	Form			: int write(int handle, unsigned char *buffer, unsigned int len)
	Parameters		: int handle
					  unsigned char *buffer
					  unsigned int len
	Return value	: int
	Initialization	: None.
	Description		: The write function writes len bytes of data from the area specified by buffer to UART0.
******************************************************************************/
int write(int handle, unsigned char *buffer, unsigned int len)
{
	_flgUartFin = 0; 
	uart_stop();
	uart_startSend(buffer, len, _funcUartFin); 
	while(_flgUartFin != 1)
	{
		main_clrWDT();
	}
	return len;
}

/*******************************************************************************
	Routine Name	: ADC_Read
	Form			: int ADC_Read()
	Parameters		: unsigned char idx
	Return value	: int
	Initialization	: None.
	Description		: Read ADC(idx) value
******************************************************************************/
int ADC_Read(unsigned char idx)
{
	_flgADCFin = 0;
	SADMOD0 = (unsigned char)(1<<idx);
	SARUN = 1;
	while(_flgADCFin == 0)
	{
		main_clrWDT();
	}
	switch(idx)
	{
		case 0:		return (SADR0H<<2|SADR0L>>6);
		case 1:		return (SADR1H<<2|SADR1L>>6);
		case 2:		return (SADR2H<<2|SADR2L>>6);
		case 3:		return (SADR3H<<2|SADR3L>>6);
		case 4:		return (SADR4H<<2|SADR4L>>6);
		case 5:		return (SADR5H<<2|SADR5L>>6);
		case 6:		return (SADR6H<<2|SADR6L>>6);
		case 7:		return (SADR7H<<2|SADR7L>>6);
		default:	return 0;
	}
}

/*******************************************************************************
	Routine Name	: I2C_Read
	Form			: void I2C_Read(unsigned char slave_address, unsigned char *address, unsigned char address_size, unsigned char *buffer, unsigned char size)
	Parameters		: unsigned char slave_address
					  unsigned char *address
					  unsigned char address_size
					  unsigned char *buffer
					  unsigned char size
	Return value	: void
	Initialization	: None.
	Description		: 
******************************************************************************/
void I2C_Read(unsigned char slave_address, unsigned char *reg_address, unsigned char reg_address_size, unsigned char *buffer, unsigned char size)
{
	_flgI2CFin = 0;
	i2c_stop();	
	i2c_startReceive(slave_address, reg_address, reg_address_size, buffer, size, (cbfI2c)_funcI2CFin);
	while(_flgI2CFin != 1)
	{
		main_clrWDT();
	}
}

/*******************************************************************************
	Routine Name	: I2C_Write
	Form			: void I2C_Write(unsigned char slave_address, unsigned char *address, unsigned char address_size, unsigned char *buffer, unsigned char size)
	Parameters		: unsigned char slave_address
					  unsigned char *address
					  unsigned char address_size
					  unsigned char *buffer
					  unsigned char size
	Return value	: void
	Initialization	: None.
	Description		: 
******************************************************************************/
void I2C_Write(unsigned char slave_address, unsigned char *reg_address, unsigned char reg_address_size, unsigned char *buffer, unsigned char size)
{
	_flgI2CFin = 0;
	i2c_stop();	
	i2c_startSend(slave_address, reg_address, reg_address_size, buffer, size, (cbfI2c)_funcI2CFin);
	while(_flgI2CFin != 1)
	{
		main_clrWDT();
	}
}

/*******************************************************************************
	Routine Name:	main_clrWDT
	Form:			void main_clrWDT( void )
	Parameters:		void
	Return value:	void
	Description:	clear WDT.
******************************************************************************/

void main_clrWDT( void )
{
	//How to clear the Watch Dog Timer:
	// => Write alternately 0x5A and 0xA5 into WDTCON register
	do {
		WDTCON = 0x5Au;
	} while (WDP != 1);
	WDTCON = 0xA5u;
}

/*******************************************************************************
	Routine Name:	_funcUartFin
	Form:			static void _funcUartFin( unsigned int size, unsigned char errStat )
	Parameters:		unsigned int size		 : 
				unsigned char errStat	 : 
	Return value:	void
	Description:	UART transmission completion callback function.
******************************************************************************/
static void _funcUartFin( unsigned int size, unsigned char errStat )
{
	uart_continue();					// Function in UART.c: process to continue send and receive...
	_flgUartFin = (unsigned char)FLG_SET;
	main_reqNotHalt();				// uncommented 5/2/2013
}

/*******************************************************************************
	Routine Name:	_funcI2CFin
	Form:			static void _funcUartFin( unsigned int size, unsigned char errStat )
	Parameters:		unsigned int size		 : 
				unsigned char errStat	 : 
	Return value:	void
	Description:	UART transmission completion callback function.
******************************************************************************/
static void _funcI2CFin( unsigned int size, unsigned char errStat )
{
	i2c_continue();					// Function in UART.c: process to continue send and receive...
	_flgI2CFin = (unsigned char)FLG_SET;
	main_reqNotHalt();				// uncommented 5/2/2013
}

/*******************************************************************************
	Routine Name:	_intI2c
	Form:			static void _intI2c( void )
	Parameters:		void
	Return value:	void
	Description:	I2C handler.
******************************************************************************/
static void _intI2c( void )
{
	i2c_continue();
	main_reqNotHalt();
}

/*******************************************************************************
	Routine Name:	_intADC
	Form:			static void _intADC( void )
	Parameters:		void
	Return value:	void
	Description:	I2C handler.
******************************************************************************/
static void _intADC( void )
{
	_flgADCFin = 1;
}

/*******************************************************************************
	Routine Name:	_intPB2
	Form:			static void _intADC( void )
	Parameters:		void
	Return value:	void
	Description:	I2C handler.
******************************************************************************/
static void _intPB2( void )
{
	_flgPB2Int = 1;
	//PRINTF("PB2 Int Works!");
}

/*******************************************************************************
	Routine Name:	main_reqNotHalt
	Form:			void reqNotHalt( void )
	Parameters:		void
	Return value:	void
	Description:	request not halt.
******************************************************************************/
void main_reqNotHalt( void )
{
	_reqNotHalt = (unsigned char)FLG_SET;
}

/*******************************************************************************
	Routine Name:	_intUart
	Form:			static void _intUart( void )
	Parameters:		void
	Return value:	void
	Description:	UART handler.
******************************************************************************/
static void _intUart( void )
{
	uart_continue(); 	//in UART.c: process to continue send and receive...
}

/*******************************************************************************
	Routine Name:	_intUart
	Form:			static void _intUart( void )
	Parameters:		void
	Return value:	void
	Description:	UART handler.
******************************************************************************/
void Init_EEPROM(void)
{	
	I2C_Write(BR24_I2C_ADDR, &BR24_REG00, 1, &BR24_REG00_Contents, 256);
	/*
	//NOPms(1);
	I2C_Write(BR24_I2C_ADDR, &BR24_REG0F, 1, &BR24_REG0F_Contents, 1);
	//NOPms(1);
	I2C_Write(BR24_I2C_ADDR, &BR24_REG30, 1, &BR24_REG30_Contents, 9);
	//NOPms(1);
	I2C_Write(BR24_I2C_ADDR, &BR24_REG40, 1, &BR24_REG40_Contents, 13);
	//NOPms(1);
	I2C_Write(BR24_I2C_ADDR, &BR24_REG50, 1, &BR24_REG50_Contents, 12);
	//NOPms(1);
	I2C_Write(BR24_I2C_ADDR, &BR24_REG60, 1, &BR24_REG60_Contents, 3);
	//NOPms(1);
	I2C_Write(BR24_I2C_ADDR, &BR24_REG70, 1, &BR24_REG70_Contents, 1);
	*/
}

//===========================================================================
//	OSC set
//===========================================================================
static void SetOSC(void){

	//FCON0: 			// xMHz PLL (3=1MHz; 2=2MHz; 1=4MHz; 0=8MHz)...
	SYSC0 = 0;			// Used to select the frequency of the HSCLK => 00=8.192MHz.
	SYSC1 = 0;			// setting HS Clock to 1/8 aka 1.024MHz

	OSCM1 = 1;			// 10 => Built-in PLL oscillation mode
	OSCM0 = 0;
   	
	ENOSC = 1;			//1=Enable High Speed Oscillator...
	SYSCLK = 1;			//1=HSCLK; 0=LSCLK 

	LPLL = 1;			//1=Enables the use of PLL oscillation - ADDED 4/30/2013

	__EI();			//INT enable
}
//===========================================================================

//===========================================================================
//	Clear All 3 Bits of Port A
//===========================================================================
void PortA_Low(void){

//Carl's Notes...

//Step 1: Set Pin Direction...
//Step 2: Set Pin I/O Type...
//Step 3: Set Pin Purpose...
//Step 4: Set Pin Data...

	//Direction...	
	PA0DIR = 0;		// PortA Bit0 set to Output Mode...
	PA1DIR = 0;		// PortA Bit1 set to Output Mode...
	PA2DIR = 0;		// PortA Bit2 set to Output Mode...

	//I/O Type...
	PA0C1  = 1;		// PortA Bit0 set to CMOS Output...
	PA0C0  = 1;		
	PA1C1  = 1;		// PortA Bit1 set to CMOS Output...
	PA1C0  = 1;	
	PA2C1  = 1;		// PortA Bit2 set to CMOS Output...
	PA2C0  = 1;	

	//Purpose...
	PA0MD1  = 0;	// PortA Bit0 set to General Purpose Output...
	PA0MD0  = 0;	
	PA1MD1  = 0;	// PortA Bit1 set to General Purpose Output...
	PA1MD0  = 0;	
	PA2MD1  = 0;	// PortA Bit2 set to General Purpose Output...
	PA2MD0  = 0;	

	//Data...
	PA0D = 0;		// A.0 Output OFF....
	PA1D = 0;		// A.1 Output OFF....
	PA2D = 0;		// A.2 Output OFF....

	main_clrWDT(); 	// Clear WDT
}
//===========================================================================

//===========================================================================
//	Clear All 8 Bits of Port B
//===========================================================================
void PortB_Low(void){

//Carl's Notes...

//Step 1: Set Pin Direction...
//Step 2: Set Pin I/O Type...
//Step 3: Set Pin Purpose...
//Step 4: Set Pin Data...

	//Direction...	
	PB0DIR = 0;		// PortB Bit0 set to Output Mode...
	PB1DIR = 0;		// PortB Bit1 set to Output Mode...
	PB2DIR = 0;		// PortB Bit2 set to Output Mode...
	PB3DIR = 0;		// PortB Bit3 set to Output Mode...
	PB4DIR = 0;		// PortB Bit4 set to Output Mode...
	PB5DIR = 0;		// PortB Bit5 set to Output Mode...
	PB6DIR = 0;		// PortB Bit6 set to Output Mode...
	PB7DIR = 0;		// PortB Bit7 set to Output Mode...

	//I/O Type...
	PB0C1  = 1;		// PortB Bit0 set to CMOS Output...
	PB0C0  = 1;		
	PB1C1  = 1;		// PortB Bit1 set to CMOS Output...
	PB1C0  = 1;	
	PB2C1  = 1;		// PortB Bit2 set to CMOS Output...
	PB2C0  = 1;	
	PB3C1  = 1;		// PortB Bit3 set to CMOS Output...
	PB3C0  = 1;		
	PB4C1  = 1;		// PortB Bit4 set to CMOS Output...
	PB4C0  = 1;	
	PB5C1  = 1;		// PortB Bit5 set to CMOS Output...
	PB5C0  = 1;	
	PB6C1  = 1;		// PortB Bit6 set to CMOS Output...
	PB6C0  = 1;	
	PB7C1  = 1;		// PortB Bit7 set to CMOS Output...
	PB7C0  = 1;	

	//Purpose...
	PB0MD1  = 0;	// PortB Bit0 set to General Purpose Output...
	PB0MD0  = 0;	
	PB1MD1  = 0;	// PortB Bit1 set to General Purpose Output...
	PB1MD0  = 0;	
	PB2MD1  = 0;	// PortB Bit2 set to General Purpose Output...
	PB2MD0  = 0;	
	PB3MD1  = 0;	// PortB Bit3 set to General Purpose Output...
	PB3MD0  = 0;	
	PB4MD1  = 0;	// PortB Bit4 set to General Purpose Output...
	PB4MD0  = 0;	
	PB5MD1  = 0;	// PortB Bit5 set to General Purpose Output...
	PB5MD0  = 0;
	PB6MD1  = 0;	// PortB Bit6 set to General Purpose Output...
	PB6MD0  = 0;	
	PB7MD1  = 0;	// PortB Bit7 set to General Purpose Output...
	PB7MD0  = 0;

	//Data...
	PB0D = 0;		// B.0 Output OFF....
	PB1D = 0;		// B.1 Output OFF....
	PB2D = 0;		// B.2 Output OFF....
	PB3D = 0;		// B.3 Output OFF....
	PB4D = 0;		// B.4 Output OFF....
	PB5D = 0;		// B.5 Output OFF....
	PB6D = 0;		// B.6 Output OFF....
	PB7D = 0;		// B.7 Output OFF....

	main_clrWDT(); 	// Clear WDT
}
//===========================================================================

//===========================================================================
//	Clear All 8 Bits of Port C
//===========================================================================
void PortC_Low(void){

//Carl's Notes...

//Step 1: Set Pin Direction...
//Step 2: Set Pin I/O Type...
//Step 3: Set Pin Purpose...
//Step 4: Set Pin Data...

	//Direction...	
	PC0DIR = 0;		// PortC Bit0 set to Output Mode...
	PC1DIR = 0;		// PortC Bit1 set to Output Mode...
	PC2DIR = 0;		// PortC Bit2 set to Output Mode...
	PC3DIR = 0;		// PortC Bit3 set to Output Mode...
	PC4DIR = 0;		// PortC Bit4 set to Output Mode...
	PC5DIR = 0;		// PortC Bit5 set to Output Mode...
	PC6DIR = 0;		// PortC Bit6 set to Output Mode...
	PC7DIR = 0;		// PortC Bit7 set to Output Mode...

	//I/O Type...
	PC0C1  = 1;		// PortC Bit0 set to High-Impedance Output...
	PC0C0  = 1;		
	PC1C1  = 1;		// PortC Bit1 set to High-Impedance Output...
	PC1C0  = 1;	
	PC2C1  = 1;		// PortC Bit2 set to High-Impedance Output...
	PC2C0  = 1;	
	PC3C1  = 1;		// PortC Bit3 set to High-Impedance Output...
	PC3C0  = 1;		
	PC4C1  = 1;		// PortC Bit4 set to High-Impedance Output...
	PC4C0  = 1;	
	PC5C1  = 1;		// PortC Bit5 set to High-Impedance Output...
	PC5C0  = 1;	
	PC6C1  = 1;		// PortC Bit6 set to High-Impedance Output...
	PC6C0  = 1;	
	PC7C1  = 1;		// PortC Bit7 set to High-Impedance Output...
	PC7C0  = 1;	

	//Purpose...
	PC0MD1  = 0;	// PortC Bit0 set to General Purpose Output...
	PC0MD0  = 0;	
	PC1MD1  = 0;	// PortC Bit1 set to General Purpose Output...
	PC1MD0  = 0;	
	PC2MD1  = 0;	// PortC Bit2 set to General Purpose Output...
	PC2MD0  = 0;	
	PC3MD1  = 0;	// PortC Bit3 set to General Purpose Output...
	PC3MD0  = 0;	
	PC4MD1  = 0;	// PortC Bit4 set to General Purpose Output...
	PC4MD0  = 0;	
	PC5MD1  = 0;	// PortC Bit5 set to General Purpose Output...
	PC5MD0  = 0;
	PC6MD1  = 0;	// PortC Bit6 set to General Purpose Output...
	PC6MD0  = 0;	
	PC7MD1  = 0;	// PortC Bit7 set to General Purpose Output...
	PC7MD0  = 0;

	//Data...
	PC0D = 0;		// C.0 Output OFF....
	PC1D = 0;		// C.1 Output OFF....
	PC2D = 0;		// C.2 Output OFF....
	PC3D = 0;		// C.3 Output OFF....
	PC4D = 0;		// C.4 Output OFF....
	PC5D = 0;		// C.5 Output OFF....
	PC6D = 0;		// C.6 Output OFF....
	PC7D = 0;		// C.7 Output OFF....

	main_clrWDT(); 	// Clear WDT

}
//===========================================================================

//===========================================================================
//	Clear All 6 Bits of Port D
//===========================================================================
void PortD_Low(void){

	//Carl's Notes...

	//Step 1: Set Pin Direction...
	//Step 2: Set Pin I/O Type...
	//Step 3: Set Pin Data...

	//Direction...	
	PD0DIR = 1;		// PortD Bit0 set to Input Mode...
	PD1DIR = 1;		// PortD Bit1 set to Input Mode...
	PD2DIR = 1;		// PortD Bit2 set to Input Mode...
	PD3DIR = 1;		// PortD Bit3 set to Input Mode...
	PD4DIR = 1;		// PortD Bit4 set to Input Mode...
	PD5DIR = 1;		// PortD Bit5 set to Input Mode...

	//I/O Type...
	PD0C1= 1;		// PortD Bit0 set to High-impedance input...
	PD0C0= 1;		
	PD1C1= 1;		// PortD Bit1 set to High-impedance input...
	PD1C0= 1;	
	PD2C1= 1;		// PortD Bit2 set to High-impedance input...
	PD2C0= 1;	
	PD3C1= 1;		// PortD Bit3 set to High-impedance input...
	PD3C0= 1;		
	PD4C1= 1;		// PortD Bit4 set to High-impedance input...
	PD4C0= 1;	
	PD5C1= 1;		// PortD Bit5 set to High-impedance input...
	PD5C0= 1;	

	//Data...
	PD0D = 0;		// D.0 Input OFF....
	PD1D = 0;		// D.1 Input OFF....
	PD2D = 0;		// D.2 Input OFF....
	PD3D = 0;		// D.3 Input OFF....
	PD4D = 0;		// D.4 Input OFF....
	PD5D = 0;		// D.5 Input OFF....

	main_clrWDT(); 	// Clear WDT
}
//===========================================================================

/*******************************************************************************
	Routine Name:	NOPms
	Form:			void NOP1000( unsigned int ms )
	Parameters:		unsigned int sec = "Number of seconds where the device is not doing anything"
	Return value:	void
	Description:	NOP for x seconds. Uses HTB* clock (512kHz) and timer 8+9 (max 0xFFFF)
					*(HTBCLK = 1/16 * HSCLK = (1/16)*8192kHz = 512kHz, see HTBDR to change if we need an even smaller increment timer...)
					1/(512kHz) * 0xFFFF = 127ms
					
					(HTBCLK = 1/16 * HSCLK = (1/16)*1024kHz = 64kHz, see HTBDR to change if we need an even smaller increment timer...)
					1/(512kHz) * 0xFFFF = 1.02secs
					
******************************************************************************/
void NOPms( unsigned int ms )
{
	unsigned int timerThres;
	unsigned char TimeFlag;
	unsigned int TempSec;
	unsigned int timer;
	unsigned int timertest;

	TempSec = ms;
	TimeFlag = 0;

	tm_init(TM_CH_NO_AB);
	tm_setABSource(TM_CS_HTBCLK);
	tm_setABData(0xffff);
	
	/*
	if(ms < 128){
		timerThres = 0x1FF * ms;
		TimeFlag = 0;
	}
	if(ms == 128){
		timerThres = 0xFFFF;
		TimeFlag = 0;
	}
	if(ms > 128){
		while(TempSec > 128){
			TempSec -= 128;
			TimeFlag++;
		}
		if(TempSec != 0){
			timerThres = 0x1FF * TempSec;
		}
		else{
			timerThres = 0xFFFF;
			TimeFlag--;
		}
	}
	*/
	TimeFlag = ms-1;
	timerThres = 0xFFFF;
	
TimerRestart:
	main_clrWDT();	
	//tm_restart89();	//using LSCLK, the maximum delay time we have is ~2 secs
	tm_startAB();
	timer = tm_getABCounter();
	while(timer < timerThres){
		timer = tm_getABCounter();
		//timertest = timer;
	}
	if(TimeFlag !=0){
		tm_stopAB();
		TimeFlag--;
		timerThres = 0xFFFF;
		goto TimerRestart;
	}
}
	
/*******************************************************************************
	Routine Name:	ReverseBits
	Form:			unsigned char ReverseBits(unsigned char data)
	Parameters:		unsigned char data
	Return value:	unsigned char
	Description:	Reverse bits order of data
******************************************************************************/	
unsigned char ReverseBits(unsigned char data)
{
__asm("\n\
	MOV r1,r0\n\
	MOV r0,#0\n\
	MOV r2,#8\n\
_ReverseBits_loop:\n\
	SLL r0,#1\n\
	SRL r1,#1\n\
	BGE _ReverseBits_next\n\
	OR r0,#1\n\
_ReverseBits_next:\n\
	ADD	r2,	#0ffh\n\
	CMP	r2,	#00h\n\
	BGT _ReverseBits_loop\n\
");
}