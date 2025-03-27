#include <XC.h>
#include <sys/attribs.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lcd.h"


void delayus(int us) // Use the core timer to wait for a specified number of microseconds
{
	uint32_t start = _CP0_GET_COUNT(); // Get the current core timer count
	uint32_t ticks = (SYSCLK / 2000000) * us; // Calculate the number of ticks for the desired delay

	while ((_CP0_GET_COUNT() - start) < ticks); // Wait until the desired number of ticks has passed
	// Note: The core timer runs at SYSCLK/2, so we divide by 2 to get the correct number of ticks for the desired delay
}


void waitms(int ms) // Use the core timer to wait for a specified number of ms
{
	uint32_t start = _CP0_GET_COUNT(); // Get the current core timer count
	uint32_t ticks = (SYSCLK / 2000) * ms; // Calculate the number of ticks for the desired delay

	while ((_CP0_GET_COUNT() - start) < ticks); // Wait until the desired number of ticks has passed
	// Note: The core timer runs at SYSCLK/2, so we divide by 2 to get the correct number of ticks for the desired delay
}


void LCD_pulse(void)
{
	LCD_E = 1;
	delayus(5);
	LCD_E = 0;
}

void LCD_byte(unsigned char x)
{
	LCD_D7 = (x & 0x80) ? 1 : 0;
	LCD_D6 = (x & 0x40) ? 1 : 0;
	LCD_D5 = (x & 0x20) ? 1 : 0;
	LCD_D4 = (x & 0x10) ? 1 : 0;
	LCD_pulse();
	delayus(5);
	LCD_D7 = (x & 0x08) ? 1 : 0;
	LCD_D6 = (x & 0x04) ? 1 : 0;
	LCD_D5 = (x & 0x02) ? 1 : 0;
	LCD_D4 = (x & 0x01) ? 1 : 0;
	LCD_pulse();
}

void WriteData(unsigned char x)
{
	LCD_RS = 1;
	LCD_byte(x);
	delayus(37);
}

void WriteCommand(unsigned char x)
{
	LCD_RS = 0;
	LCD_byte(x);
	delayus(37);
}

void LCD_4BIT(void)
{
	// Configure the pins used to communicate with the LCD as outputs
	LCD_RS_ENABLE = 0;
	LCD_E_ENABLE = 0;
	LCD_D4_ENABLE = 0;
	LCD_D5_ENABLE = 0;
	LCD_D6_ENABLE = 0;
	LCD_D7_ENABLE = 0;

	LCD_E = 0; // Resting state of LCD's enable is zero
	// LCD_RW = 0; Not used in this code.  Connect to ground.
	waitms(15);
	// First make sure the LCD is in 8-bit mdode, then change to 4-bit mode
	WriteCommand(0x33);
	WriteCommand(0x33);
	WriteCommand(0x32); // Change to 4-bit mode

	// Configure the LCD
	WriteCommand(0x28);
	WriteCommand(0x0c);
	WriteCommand(0x01); // Clear screen command (takes some time)
	waitms(15); // Wait for clear screen command to finish
	LATBbits.LATB0 = !LATBbits.LATB0;
}

void LCDprint(char* string, unsigned char line, unsigned char clear)
{
	int j;

	WriteCommand(line == 2 ? 0xc0 : 0x80);
	delayus(37);
	for (j = 0;string[j] != 0;j++)
		WriteData(string[j]); //Write the message character by character
	if (clear)
		for (;j < CHARS_PER_LINE;j++)
			WriteData(' '); //Clear the rest of the line if clear is 1
}