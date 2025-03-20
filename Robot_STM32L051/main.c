#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

#include "Common/Include/stm32l051xx.h"
#include "Common/Include/stm32l0xx_ll_gpio.h"
#include "Common/Include/stm32l0xx_ll_tim.h"
#include "Common/Include/stm32l0xx_ll_bus.h"
#include "Common/Include/stm32l0xx_ll_adc.h"
#include "Common/Include/stm32l0xx_ll_system.h"
#include "Common/Include/stm32l0xx_ll_utils.h"

#include "Common/Include/serial.h"
#include "UART2.h"
#include "lcd.h"


// LQFP32 pinout
//              ----------
//        VDD -|1       32|- VSS
//       PC14 -|2       31|- BOOT0
//       PC15 -|3       30|- PB7
//       NRST -|4       29|- PB6  LCD_D7
//       VDDA -|5       28|- PB5  LCD_D6
//        PA0 -|6       27|- PB4  LCD_D5
//        PA1 -|7       26|- PB3  LCD_D4
//        PA2 -|8       25|- PA15
//        PA3 -|9       24|- PA14
//        PA4 -|10      23|- PA13
//        PA5 -|11      22|- PA12
//        PA6 -|12      21|- PA11  LCD_E
//        PA7 -|13      20|- PA10 (Reserved for RXD)
//        PB0 -|14      19|- PA9  (Reserved for TXD)
//        PB1 -|15      18|- PA8   LCD_RS
//        VSS -|16      17|- VDD
//              ----------


// Use the volatile keyword for all global variables, prevents compiler from optimizing them out
volatile int frequency = 0;
volatile float capacitance = 0;
volatile int print_frequency = 0;
volatile static int ms_counter2 = 0;
volatile static int ms_counter4 = 0;

// Bit-field struct to hold flags, add more as needed
typedef struct {
	bool printFlag : 1;
	bool disconnectedFlag : 1;
	bool microFlag : 1;
	bool periodFlag : 1;
} flags_struct;

volatile flags_struct flag;

void Configure_Pins(void)
{
	// Enable GPIOA and GPIOB clocks
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA); // Enables clock for GPIOA
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB); // Enables clock for GPIOB

	// Set display pins to be outputs in push-pull mode
	LL_GPIO_SetPinMode(GPIOB, BIT3, LL_GPIO_MODE_OUTPUT); // Set PB3 to output mode
	LL_GPIO_SetPinMode(GPIOB, BIT4, LL_GPIO_MODE_OUTPUT); // Set PB4 to output mode
	LL_GPIO_SetPinMode(GPIOB, BIT5, LL_GPIO_MODE_OUTPUT); // Set PB5 to output mode
	LL_GPIO_SetPinMode(GPIOB, BIT6, LL_GPIO_MODE_OUTPUT); // Set PB6 to output mode
	LL_GPIO_SetPinMode(GPIOA, BIT8, LL_GPIO_MODE_OUTPUT); // Set PA8 to output mode
	LL_GPIO_SetPinMode(GPIOA, BIT11, LL_GPIO_MODE_OUTPUT); // Set PA11 to output mode

	LL_GPIO_SetPinOutputType(GPIOB, BIT3 | BIT4 | BIT5 | BIT6, LL_GPIO_OUTPUT_PUSHPULL); // Set PB3-PB6 to push-pull mode    
	LL_GPIO_SetPinOutputType(GPIOA, BIT8 | BIT11, LL_GPIO_OUTPUT_PUSHPULL); // Set PA8 and PA11 to push-pull mode

	// Allow PA0 to be used as an input capture
	LL_GPIO_SetPinMode(GPIOA, BIT0, LL_GPIO_MODE_ALTERNATE); // Set PA0 to alternate function mode
	LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_0, LL_GPIO_AF_2); // Set PA0 to work as an input capture for TIM2_CH1 (channel is arbitrary here)
	LL_GPIO_SetPinPull(GPIOA, BIT0, LL_GPIO_PULL_DOWN); // Enables the pull-down resistor for pin PA0, might reduce noise

	// Allow PA5 and PA7 to be used as inputs
	LL_GPIO_SetPinMode(GPIOA, BIT5, LL_GPIO_MODE_INPUT); // Set PA5 to input mode
	LL_GPIO_SetPinPull(GPIOA, BIT5, LL_GPIO_PULL_UP); // Enables the pull-up resistor for pin PA5, allows you to read active-low pushbutton (connected to ground like we usually do)

	LL_GPIO_SetPinMode(GPIOA, BIT7, LL_GPIO_MODE_INPUT); // Set PA7 to input mode
	LL_GPIO_SetPinPull(GPIOA, BIT7, LL_GPIO_PULL_UP); // Enables the pull-up resistor for pin PA7, allows you to read active-low pushbutton (connected to ground like we usually do)


	// Configure pins for UART2
	LL_GPIO_SetPinSpeed(GPIOA, BIT13, LL_GPIO_SPEED_FREQ_VERY_HIGH); // Set PA13 to high speed
	LL_GPIO_SetPinSpeed(GPIOA, BIT14, LL_GPIO_SPEED_FREQ_VERY_HIGH); // Set PA14 to high speed
	LL_GPIO_SetPinSpeed(GPIOA, BIT15, LL_GPIO_SPEED_FREQ_VERY_HIGH); // Set PA15 to high speed
	LL_GPIO_SetPinMode(GPIOA, BIT13, LL_GPIO_MODE_OUTPUT); // Set PA13 to output mode
	LL_GPIO_SetOutputPin(GPIOA, BIT13); // Set PA13 to high by default (required for JDY-40 to work)


}

void init_timers(void)
{
	// Configure TIM2 for input capture
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2); // Enables clock for TIM2
	LL_TIM_SetAutoReload(TIM2, 1000 - 1); // 1000-tick preload value (arbitrary, we aren't using the time for this timer)
	LL_TIM_SetPrescaler(TIM2, 31); // Sets the prescaler to 31, so the counter ticks at 1MHz (Divides clock by 31+1 = 32, so 1Mhz)
	LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI); // Sets the active input for channel 1 to direct input (input comes from pin PA0)
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1); // Enables channel 1
	LL_TIM_EnableIT_CC1(TIM2); // Enables interrupt on channel 1
	LL_TIM_EnableCounter(TIM2); // Enables the counter
	NVIC_EnableIRQ(TIM2_IRQn); // Enables interrupts for TIM2

	// Configure TIM6 for periodic interrupts every 1ms
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6); // Enables clock for TIM6
	LL_TIM_SetPrescaler(TIM6, 31); // Sets the prescaler to 31, so the counter ticks at 1MHz (Divides clock by 31+1 = 32, so 1Mhz)
	LL_TIM_SetAutoReload(TIM6, 1000 - 1); // 1000-tick preload value (includes 0, so 999 means 1000 ticks)
	LL_TIM_ClearFlag_UPDATE(TIM6); // Clears the update event flag
	LL_TIM_EnableIT_UPDATE(TIM6); // Enables interrupt on update event
	LL_TIM_EnableCounter(TIM6); // Enables the counter
	NVIC_EnableIRQ(TIM6_IRQn); // Enables interrupts for TIM6


	__enable_irq(); // Enables global interrupts
}

void TIM2_Handler(void) // This function is called when a rising edge is detected on the input capture pin
{
	if ((LL_TIM_IsActiveFlag_CC1(TIM2))) { // Flag at bit zero is true only if a capture of a rising edge has occured
		LL_TIM_ClearFlag_CC1(TIM2); // Clears the capture flag

		frequency++; // Increments the number of rising edges detected, cleared after every second in main
	}
}

float calculate_capacitance(void) // Calculates the capacitance based on the frequency
{
	float cap = 0;
	cap = (1440000) / (float)(5000 * frequency);
	return cap;
}

void TIM6_Handler(void) // This function is called every 1ms
{
	if (LL_TIM_IsActiveFlag_UPDATE(TIM6)) { // Flag at bit zero is true only if an update event has occured
		LL_TIM_ClearFlag_UPDATE(TIM6); // Clears the update flag

		volatile static int ms_counter1 = 0;
		volatile static int ms_counter3 = 0;
		ms_counter1++;
		ms_counter3++;

		if (ms_counter1 >= 1000) {
			ms_counter1 = 0;

			if (frequency > 205000) flag.disconnectedFlag = true; // If the frequency is very high (around 205 kHz), the capacitor is disconnected
			else flag.disconnectedFlag = false;

			capacitance = calculate_capacitance(); // Calculates the capacitance based on the frequency
			print_frequency = frequency; // Stores the frequency to be printed every second since we print values every 250ms
			frequency = 0; // Resets the frequency counter
		}

		if (ms_counter3 >= 250) { // Every 250ms
			flag.printFlag = true; // Sets the print flag to true every 250ms
			ms_counter3 = 0;
		}

		if (LL_GPIO_IsInputPinSet(GPIOA, BIT5)) { // Button on PA5 not pressed
			if (ms_counter2 >= 30) {
				flag.microFlag ^= 1; // Toggles the microFarad flag
				flag.printFlag = true; // Sets the print flag to true, causing the LCD to update
				ms_counter2 = 0; // Resets the counter for the button on PA5
			}
			else ms_counter2 = 0; // Resets the counter for the button on PA5
		}
		else if (!(LL_GPIO_IsInputPinSet(GPIOA, BIT5))) { // Button on PA5 pressed
			ms_counter2++; // Increments the counter for the button on PA5 while pressed
		}
		if (LL_GPIO_IsInputPinSet(GPIOA, BIT7)) { // Button on PA7 not pressed
			if (ms_counter4 >= 30) {
				flag.periodFlag ^= 1; // Toggles the period flag
				flag.printFlag = true; // Sets the print flag to true, causing the LCD to update
				ms_counter4 = 0; // Resets the counter for the button on PA7
			}
			else ms_counter4 = 0; // Resets the counter for the button on PA7
		}
		else if (!(LL_GPIO_IsInputPinSet(GPIOA, BIT7))) { // Button on PA7 pressed
			ms_counter4++; // Increments the counter for the button on PA7 while pressed
		}
	}
}

void SendATCommand(char* s)
{
	char buff[40];
	printf("Command: %s", s);
	LL_GPIO_ResetOutputPin(GPIOA, BIT13); // 'set' pin to 0 is 'AT' mode.
	waitms(10);
	eputs2(s);
	egets2(buff, sizeof(buff) - 1);
	LL_GPIO_SetOutputPin(GPIOA, BIT13); // 'set' pin to 1 is normal operation mode.
	waitms(10);
	printf("Response: %s", buff);
}

void ReceptionOff(void)
{
	LL_GPIO_ResetOutputPin(GPIOA, BIT13); // 'set' pin to 0 is 'AT' mode.
	waitms(10);
	eputs2("AT+DVID0000\r\n"); // Some unused id, so that we get nothing in RXD1.
	waitms(10);
	LL_GPIO_SetOutputPin(GPIOA, BIT13); // 'set' pin to 1 is normal operation mode.
	while (ReceivedBytes2() > 0) egetc2(); // Clear FIFO
}

void main(void)
{
	char cap_string[16];
	char freq_string[16];

	char buff[80];
	int cnt = 0;
	char c;
	int timeout_cnt = 0;
	int cont1 = 0, cont2 = 100;

	Configure_Pins();
	LCD_4BIT();
	init_timers();

	printf("Testing!!!\r\n");
	LCDprint("Loading...", 1, 1);
	LCDprint("Loading...", 2, 1);

	flag.periodFlag = false;
	flag.microFlag = false;


	initUART2(9600);

	waitms(1000); // Give putty some time to start.
	printf("\r\nJDY-40 Slave test for the STM32L051\r\n");

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
	// number from 0x0000 to 0xFFFF.  In this case is set to 0xSICK

	SendATCommand("AT+DVID7788\r\n");
	SendATCommand("AT+RFC529\r\n");
	
	while (1) // Loop indefinitely
	{
	
		if (flag.printFlag == true) {
			if (flag.disconnectedFlag == true) {
				printf("Capacitor Disconnected, Frequency = %d\r\n\n", print_frequency);
				LCDprint("Capacitor", 1, 1);
				LCDprint("not detected", 2, 1);
				flag.printFlag = false;
			}
			else {
				printf("Capacitance = %3.3f nF\r\n", capacitance * 1000); // **DO NOT FORGET TO TO \r\n**; IT FREEZES THE ENTIRE PROGRAM IF YOU DONT BECAUSE OF THE GCC COMPILER
				printf("Frequency = %d Hz\r\n\n", print_frequency);

				if (flag.microFlag == 1) {
					sprintf(cap_string, "Cap: %1.5fuF", capacitance);
				}
				else {
					sprintf(cap_string, "Cap: %3.3fnF", capacitance * 1000);
				}

				if (flag.periodFlag == 1) {
					sprintf(freq_string, "Pd: %1.5fms", (1000 / (float)print_frequency));
				}
				else {
					sprintf(freq_string, "Freq: %dHz", print_frequency);
				}

				LCDprint(freq_string, 1, 1);
				LCDprint(cap_string, 2, 1);

				flag.printFlag = false;
			}
		}

		if (ReceivedBytes2() > 0) // Something has arrived
		{
			c = egetc2();

			if (c == '!') // Master is sending message
			{
				egets2(buff, sizeof(buff) - 1);
				if (strlen(buff) == 8)
				{
					printf("Master says: %s\r", buff);
				}
				else
				{
					printf("*** BAD MESSAGE ***: %s\r", buff);
				}
			}
			else if (c == '@') // Master wants slave data
			{
				sprintf(buff, "%05u\n", cnt);
				cnt++;
				waitms(5); // The radio seems to need this delay...
				eputs2(buff); // Can only send one message at a time				
			}
		}

		
	}
}
