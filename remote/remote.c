#include <XC.h>
#include <sys/attribs.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lcd.h"
#include "UART.h"

// Configuration Bits (somehow XC32 takes care of this)
#pragma config FNOSC = FRCPLL       // Internal Fast RC oscillator (8 MHz) w/ PLL
#pragma config FPLLIDIV = DIV_2     // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL = MUL_20     // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV = DIV_2     // Divide After PLL (now 40 MHz)
#pragma config FWDTEN = OFF         // Watchdog Timer Disabled
#pragma config FPBDIV = DIV_1       // PBCLK = SYCLK
#pragma config FSOSCEN = OFF        // Secondary Oscillator Enable (Disabled)

// Defines
#define Baud2BRG(desired_baud)( (SYSCLK / (16*desired_baud))-1)
#define Baud1BRG(desired_baud)( (SYSCLK / (16*desired_baud))-1)
#define DEF_FREQ 1800L


/* Pinout for DIP28 PIC32MX130:
										  --------
								   MCLR -|1     28|- AVDD
  VREF+/CVREF+/AN0/C3INC/RPA0/CTED1/RA0 -|2     27|- AVSS
		VREF-/CVREF-/AN1/RPA1/CTED2/RA1 -|3     26|- AN9/C3INA/RPB15/SCK2/CTED6/PMCS1/RB15
   PGED1/AN2/C1IND/C2INB/C3IND/RPB0/RB0 -|4     25|- CVREFOUT/AN10/C3INB/RPB14/SCK1/CTED5/PMWR/RB14
  PGEC1/AN3/C1INC/C2INA/RPB1/CTED12/RB1 -|5     24|- AN11/RPB13/CTPLS/PMRD/RB13
   AN4/C1INB/C2IND/RPB2/SDA2/CTED13/RB2 -|6     23|- AN12/PMD0/RB12
	 AN5/C1INA/C2INC/RTCC/RPB3/SCL2/RB3 -|7     22|- PGEC2/TMS/RPB11/PMD1/RB11
									VSS -|8     21|- PGED2/RPB10/CTED11/PMD2/RB10
					 OSC1/CLKI/RPA2/RA2 -|9     20|- VCAP
				OSC2/CLKO/RPA3/PMA0/RA3 -|10    19|- VSS
						 SOSCI/RPB4/RB4 -|11    18|- TDO/RPB9/SDA1/CTED4/PMD3/RB9
		 SOSCO/RPA4/T1CK/CTED9/PMA1/RA4 -|12    17|- TCK/RPB8/SCL1/CTED10/PMD4/RB8
									VDD -|13    16|- TDI/RPB7/CTED3/PMD5/INT0/RB7
					PGED3/RPB5/PMD7/RB5 -|14    15|- PGEC3/RPB6/PMD6/RB6
										  --------
*/


void __ISR(_TIMER_1_VECTOR, IPL5SOFT) Timer1_Handler(void)
{
	LATBbits.LATB10 = !LATBbits.LATB10; // Desired frequency on RB6
	IFS0CLR = _IFS0_T1IF_MASK; // Clear timer 1 interrupt flag, bit 4 of IFS0
}


void SetupTimer1(void)
{
	// Explanation here:
	// https://www.youtube.com/watch?v=bu6TTZHnMPY
	__builtin_disable_interrupts();
	PR1 = (SYSCLK / (DEF_FREQ * 2L)) - 1; // since SYSCLK/FREQ = PS*(PR1+1)
	TMR1 = 0;
	T1CONbits.TCKPS = 0; // Pre-scaler is 1:1 (runs at SYSCLK) (0 = 1:1, 1 = 1:8, 2 = 1:64, 3 = 1:256)
	T1CONbits.TCS = 0; // Use internal peripheral clock (runs at SYSCLK)
	T1CONbits.ON = 1; // Enable timer 1
	IPC1bits.T1IP = 5; // Interrupt priority is set to 5 (0-7, 3-bit field)
	IPC1bits.T1IS = 0; // Interrupt sub-priority is set to 0 (0-3, 2-bit field)
	IFS0bits.T1IF = 0; // Interrupt flag is cleared
	IEC0bits.T1IE = 1; // Enable interrupts for timer 1

	INTCONbits.MVEC = 1; // Interrupts are multi-vectored (0 = single, 1 = multi)
	__builtin_enable_interrupts();
}


void UART2Configure(int baud_rate)
{
	// Peripheral Pin Select
	U2RXRbits.U2RXR = 4;    //SET RX to RB8
	RPB9Rbits.RPB9R = 2;    //SET RB9 to TX

	U2MODE = 0;         // disable autobaud, TX and RX enabled only, 8N1, idle=HIGH
	U2STA = 0x1400;     // enable TX and RX
	U2BRG = Baud2BRG(baud_rate); // U2BRG = (FPb / (16*baud)) - 1

	U2MODESET = 0x8000;     // enable UART2
}


void ADCConf(void)
{
	AD1CON1CLR = 0x8000;    // disable ADC before configuration
	AD1CON1 = 0x00E0;       // internal counter ends sampling and starts conversion (auto-convert), manual sample
	AD1CON2 = 0;            // AD1CON2<15:13> set voltage reference to pins AVSS/AVDD
	AD1CON3 = 0x0f01;       // TAD = 4*TPB, acquisition time = 15*TAD 
	AD1CON1SET = 0x8000;      // Enable ADC
}

int ADCRead(char analogPIN)
{
	AD1CHS = analogPIN << 16;    // AD1CHS<16:19> controls which analog pin goes to the ADC
	AD1CON1bits.SAMP = 1;        // Begin sampling
	while (AD1CON1bits.SAMP);     // wait until acquisition is done
	while (!AD1CON1bits.DONE);    // wait until conversion done

	return ADC1BUF0;             // result stored in ADC1BUF0
}

int UART1Configure(int desired_baud)
{
	int actual_baud;

	// Peripheral Pin Select for UART1.  These are the pins that can be used for U1RX from TABLE 11-1 of '60001168J.pdf':
	// 0000 = RPA2
	// 0001 = RPB6
	// 0010 = RPA4
	// 0011 = RPB13
	// 0100 = RPB2

	// Do what the caption of FIGURE 11-2 in '60001168J.pdf' says: "For input only, PPS functionality does not have
	// priority over TRISx settings. Therefore, when configuring RPn pin for input, the corresponding bit in the
	// TRISx register must also be configured for input (set to �1�)."

	ANSELB &= ~(1 << 13); // Set RB13 as a digital I/O
	TRISB |= (1 << 13);   // configure pin RB13 as input
	CNPUB |= (1 << 13);   // Enable pull-up resistor for RB13
	U1RXRbits.U1RXR = 3; // SET U1RX to RB13

	// These are the pins that can be used for U1TX. Check table TABLE 11-2 of '60001168J.pdf':
	// RPA0
	// RPB3
	// RPB4
	// RPB15
	// RPB7

	ANSELB &= ~(1 << 15); // Set RB15 as a digital I/O
	RPB15Rbits.RPB15R = 1; // SET RB15 to U1TX

	U1MODE = 0;         // disable autobaud, TX and RX enabled only, 8N1, idle=HIGH
	U1STA = 0x1400;     // enable TX and RX
	U1BRG = Baud1BRG(desired_baud); // U1BRG = (FPb / (16*baud)) - 1
	// Calculate actual baud rate
	actual_baud = SYSCLK / (16 * (U1BRG + 1));

	U1MODESET = 0x8000;     // enable UART1

	return actual_baud;
}



// Information here:
// http://umassamherstm5.org/tech-tutorials/pic32-tutorials/pic32mx220-tutorials/1-basic-digital-io-220
void main(void)
{
	long int count = 0;
	float T, f;
	char line[20];
	char cut[14];
	char buff[80];
	int timeout_cnt = 0;
	int cont1 = 0, cont2 = 100;
	int adcval, adcval2;
	int pickup_button,auto_button;
	int cnt=0;

	DDPCON = 0;
	CFGCON = 0;

	UART2Configure(115200);  // Configure UART2 for a baud rate of 115200
	UART1Configure(9600);  // Configure UART1 to communicate with JDY40 with a baud rate of 9600
	LCD_4BIT();

	waitms(500);
	printf("remote control testint code:\r\n");

	TRISBbits.TRISB10 = 0;
	LATBbits.LATB10 = 0;
	INTCONbits.MVEC = 1;
	SetupTimer1();

	U2RXRbits.U2RXR = 4; //SET RX to RB8
	RPB9Rbits.RPB9R = 2; //SET RB9 to TX

	ANSELBbits.ANSB2 = 1;   // set RB2 (AN4, pin 6 of DIP28) as analog pin
	TRISBbits.TRISB2 = 1;   // set RB2 as an input

	ANSELBbits.ANSB1 = 1;   // set RB1 as analog pin
	TRISBbits.TRISB1 = 1;   // set RB1 as an input

	ANSELBbits.ANSB0 = 0;   // set RB0 as digital pin
	TRISBbits.TRISB0 = 1;   // set RB0 as an input

	ANSELB &= ~(1<<6); // Set RB15 as a digital I/O
    TRISB |= (1<<6);   // configure pin RB15 as input
    CNPUB |= (1<<6);   // Enable pull-up resistor for RB15

	ADCConf(); // Configure ADC

	// RB14 is connected to the 'SET' pin of the JDY40.  Configure as output:
	ANSELB &= ~(1 << 14); // Set RB14 as a digital I/O
	TRISB &= ~(1 << 14);  // configure pin RB14 as output
	LATB |= (1 << 14);    // 'SET' pin of JDY40 to 1 is normal operation mode

	ReceptionOff();

	// To check configuration
	SendATCommand("AT+VER\r\n");
	SendATCommand("AT+BAUD\r\n");
	SendATCommand("AT+RFID\r\n");
	SendATCommand("AT+DVID\r\n");
	SendATCommand("AT+RFC\r\n");
	SendATCommand("AT+POWE\r\n");
	SendATCommand("AT+CLSS\r\n");

	SendATCommand("AT+DVID7788\r\n"); // Set device ID to 7788 (0x7788, from 0x0000 to 0xFFFF)
	SendATCommand("AT+RFC052\r\n"); // Set RF frequency to 052 (from 001 to 128) 

	LCDprint("JDY-40 test", 1, 1);


	while (1)
	{

		// Potential Changes:
		// ***1. Parse the message from the robot and set flags for auto mode/pickup, and set values for frequency difference and coins collected (use sscanf?)
		// ***2. Set beeper frequency based on the frequency difference (which is based on the distance to the coin)
		// ***3. Add pushbuttons for frequency calibration, and for setting the robot to auto mode
		// 4. Display useful information on the LCD (like frequency difference, coins collected, auto mode status, connection, etc.)
		// 5. Maybe show on LCD if perimeter is detected/nearby? (send ADC value from robot for this, extra feature?)
		// 6. If flag is set for pickup, ignore lack of response from the robot and send only (512, 512) as joystick values to the robot (keeps it from moving while picking up the coin)
		// 7. Maybe add a timeout for the pickup so it doesn't get stuck in pickup mode forever? (might not be needed but simple and prevents issues) (might also be able to just implement timeout without considering pickup since data is still sent, just inconsistently)
		// 8. Add debouncing logic for buttons (use ISR to do this)
		// 9. 



		adcval = ADCRead(4); // note that we call pin AN4 (RB2) by it's analog number
		adcval2 = ADCRead(3); // note that we call pin AN3 (RB1) by it's analog number
		pickup_button = PORTBbits.RB0; // Read the state of the button (RB0)
		auto_button=(int)PORTB&(1<<6);
		if (auto_button==64)
			auto_button=0;
		else 
			auto_button=1;

		sprintf(buff, "%04d %04d %d %d\n", adcval, adcval2, pickup_button,auto_button); // Construct a message with the ADC values and button state
		if (cnt++>5)
		{
			printf(buff);
			cnt=0;
		}
		LCDprint(buff, 1, 1);

		//sprintf(buff, "%03d,%03d\n", cont1, cont2); // Construct a test message
		putc1('!'); // Send a message to the slave. First send the 'attention' character which is '!'
		// Wait a bit so the slave has a chance to get ready
		waitms(5); // This may need adjustment depending on how busy is the slave
		SerialTransmit1(buff); // Send the test message

		if (++cont1 > 200) cont1 = 0; // Increment test counters for next message
		if (++cont2 > 200) cont2 = 0;
		waitms(5); // This may need adjustment depending on how busy is the slave

		// Clear the receive 8-level FIFO of the PIC32MX, so we get a fresh reply from the slave
		if (U1STAbits.URXDA) SerialReceive1_timeout(buff, sizeof(buff) - 1);

		putc1('@'); // Request a message from the slave

		timeout_cnt = 0;
		while (1)
		{
			if (U1STAbits.URXDA) break; // Something has arrived
			if (++timeout_cnt > 250) break; // Wait up to 25ms for the repply
			delayus(100); // 100us*250=25ms
		}

		if (U1STAbits.URXDA) // Something has arrived from the slave
		{
			SerialReceive1_timeout(buff, sizeof(buff) - 1); // Get the message from the slave
			if (strlen(buff) == 5) // Check for valid message size
			{
				printf("Slave says: %s\r\n", buff);
			}
			else
			{
				printf("*** BAD MESSAGE ***: %s\r\n", buff);
			}
			//strcpy(cut, &buff[17]);
			LCDprint(buff, 2, 1);
		}
		else // Timed out waiting for reply
		{
			printf("NO RESPONSE\r\n", buff);
			LCDprint("No response", 2, 1);
		}

		waitms(50);  // Set the information interchange pace: communicate about every 50ms
		count++;
		if (count >= 10)
		{
			T1CONbits.ON = !T1CONbits.ON;
			count = 0;
		}
	}
}
