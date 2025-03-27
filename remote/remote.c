#include <XC.h>
#include <sys/attribs.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lcd.h"

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

// Needed to by scanf() and gets()
int _mon_getc(int canblock)
{
	char c;

	if (canblock)
	{
		while (!U2STAbits.URXDA); // wait (block) until data available in RX buffer
		c = U2RXREG;
		if (c == '\r') c = '\n'; // When using PUTTY, pressing <Enter> sends '\r'.  Ctrl-J sends '\n'
		return (int)c;
	}
	else
	{
		if (U2STAbits.URXDA) // if data available in RX buffer
		{
			c = U2RXREG;
			if (c == '\r') c = '\n';
			return (int)c;
		}
		else
		{
			return -1; // no characters to return
		}
	}
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

void putc1(char c)
{
	while (U1STAbits.UTXBF);   // wait while TX buffer full
	U1TXREG = c;               // send single character to transmit buffer
}

int SerialTransmit1(const char* buffer)
{
	unsigned int size = strlen(buffer);
	while (size)
	{
		while (U1STAbits.UTXBF);    // wait while TX buffer full
		U1TXREG = *buffer;          // send single character to transmit buffer
		buffer++;                   // transmit next character on following loop
		size--;                     // loop until all characters sent (when size = 0)
	}

	while (!U1STAbits.TRMT);        // wait for last transmission to finish

	return 0;
}

unsigned int SerialReceive1(char* buffer, unsigned int max_size)
{
	unsigned int num_char = 0;

	while (num_char < max_size)
	{
		while (!U1STAbits.URXDA);   // wait until data available in RX buffer
		*buffer = U1RXREG;          // empty contents of RX buffer into *buffer pointer

		// insert nul character to indicate end of string
		if (*buffer == '\n')
		{
			*buffer = '\0';
			break;
		}

		buffer++;
		num_char++;
	}

	return num_char;
}

unsigned int SerialReceive1_timeout(char* buffer, unsigned int max_size)
{
	unsigned int num_char = 0;
	int timeout_cnt;

	while (num_char < max_size)
	{
		timeout_cnt = 0;
		while (1)
		{
			if (U1STAbits.URXDA) // check if data is available in RX buffer
			{
				*buffer = U1RXREG; // copy RX buffer into *buffer pointer
				break;
			}
			if (++timeout_cnt == 100) // 100 * 100us = 10 ms
			{
				*buffer = '\n';
				break;
			}
			delayus(100);
		}

		// insert nul character to indicate end of string
		if (*buffer == '\n')
		{
			*buffer = '\0';
			break;
		}

		buffer++;
		num_char++;
	}

	return num_char;
}

void ClearFIFO(void)
{
	unsigned char c;
	U1STA = 0x1400;     // enable TX and RX, clear FIFO
	while (U1STAbits.URXDA) c = U1RXREG;
}

void SendATCommand(char* s)
{
	char buff[40];
	printf("Command: %s", s);
	LATB &= ~(1 << 14); // 'SET' pin of JDY40 to 0 is 'AT' mode.
	waitms(10);
	SerialTransmit1(s);
	U1STA = 0x1400;     // enable TX and RX, clear FIFO
	SerialReceive1(buff, sizeof(buff) - 1);
	LATB |= 1 << 14; // 'SET' pin of JDY40 to 1 is normal operation mode.
	waitms(10);
	printf("Response: %s\n", buff);
}

void ReceptionOff(void)
{
	LATB &= ~(1 << 14); // 'SET' pin of JDY40 to 0 is 'AT' mode.
	waitms(10);
	SerialTransmit1("AT+DVID0000\r\n"); // Some unused id, so that we get nothing.
	waitms(10);
	ClearFIFO();
	LATB |= 1 << 14; // 'SET' pin of JDY40 to 1 is normal operation mode.
}

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
	float voltage, v2, but;
	int but_pressed;

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

	ANSELBbits.ANSB0 = 1;   // set RB0 as analog pin
	TRISBbits.TRISB0 = 1;   // set RB0 as an input

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

	// We should select an unique device ID.  The device ID can be a hex
	// number from 0x0000 to 0xFFFF.  In this case is set to 0xABBA
	SendATCommand("AT+DVID7788\r\n");
	SendATCommand("AT+RFC529\r\n");

	LCDprint("JDY-40 test", 1, 1);


	while (1)
	{
		adcval = ADCRead(4); // note that we call pin AN4 (RB2) by it's analog number
		voltage = adcval * 3.3 / 1023.0;
		adcval2 = ADCRead(3); // note that we call pin AN4 (RB2) by it's analog number
		v2 = adcval2 * 3.3 / 1023.0;
		but = ADCRead(2) / 1023.0;
		if (but == 0.0)
			but_pressed = 1;
		else
			but_pressed = 0;

		sprintf(buff, "%.3fV %.3fV %d\n", voltage, v2, but_pressed); // Construct a test message
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
