#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

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
volatile int second_counter = 0;
volatile uint8_t motorCommand;
volatile int motorPWM_x;
volatile int motorPWM_y;
volatile char joyStick[10];
volatile char buff[80];

// Bit-field struct to hold flags, add more as needed
typedef struct {
	bool printFlag : 1;
} flags_struct;

volatile flags_struct flag;

void Configure_Pins(void)
{
	// Enable GPIOA and GPIOB clocks
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA); // Enables clock for GPIOA
	// LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB); // Enables clock for GPIOB

	// Configure pins for UART2
	LL_GPIO_SetPinSpeed(GPIOA, BIT2, LL_GPIO_SPEED_FREQ_VERY_HIGH); // Set PA2 to high speed
	LL_GPIO_SetPinSpeed(GPIOA, BIT3, LL_GPIO_SPEED_FREQ_VERY_HIGH); // Set PA3 to high speed
	LL_GPIO_SetPinSpeed(GPIOA, BIT4, LL_GPIO_SPEED_FREQ_VERY_HIGH); // Set PA4 to high speed
	//LL_GPIO_SetPinPull(GPIOA, BIT1, LL_GPIO_PULL_DOWN); // No pull-up or pull-down for PA2
	LL_GPIO_SetPinPull(GPIOA, BIT2, LL_GPIO_PULL_DOWN); // No pull-up or pull-down for PA2
	LL_GPIO_SetPinPull(GPIOA, BIT3, LL_GPIO_PULL_DOWN);
	LL_GPIO_SetPinMode(GPIOA, BIT4, LL_GPIO_MODE_OUTPUT); // Set PA4 to output mode for SET pin
	LL_GPIO_SetOutputPin(GPIOA, BIT4); // Set PA4 to high by default (required for JDY-40 to work)

	LL_GPIO_SetPinMode(GPIOA, BIT15, LL_GPIO_MODE_ALTERNATE); // Set PA15 to alternate function mode (TIM2_CH1)
	LL_GPIO_SetPinSpeed(GPIOA, BIT15, LL_GPIO_SPEED_FREQ_VERY_HIGH); // Set PA15 to high speed
	LL_GPIO_SetPinOutputType(GPIOA, BIT15, LL_GPIO_OUTPUT_PUSHPULL); // Set PA15 to push-pull mode
	LL_GPIO_SetAFPin_8_15(GPIOA, BIT15, LL_GPIO_AF_5); // Set PA15 to AF1 (TIM2_CH1)

	LL_GPIO_SetPinMode(GPIOA, BIT1, LL_GPIO_MODE_ALTERNATE); // Set PA1 to alternate function mode (TIM2_CH1)
	LL_GPIO_SetPinSpeed(GPIOA, BIT1, LL_GPIO_SPEED_FREQ_VERY_HIGH); // Set PA1 to high speed
	LL_GPIO_SetPinOutputType(GPIOA, BIT1, LL_GPIO_OUTPUT_PUSHPULL); // Set PA1 to push-pull mode
	LL_GPIO_SetAFPin_0_7(GPIOA, BIT1, LL_GPIO_AF_2); // Set PA1 to AF2 (TIM2_CH1)


}

void init_timers(void)
{
	/*
	// Configure TIM2 for input capture
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2); // Enables clock for TIM2
	LL_TIM_SetAutoReload(TIM2, 1000 - 1); // 1000-tick preload value (arbitrary, we aren't using the time for this timer)
	LL_TIM_SetPrescaler(TIM2, 31); // Sets the prescaler to 31, so the counter ticks at 1MHz (Divides clock by 31+1 = 32, so 1Mhz)
	LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI); // Sets the active input for channel 1 to direct input (input comes from pin PA0)
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1); // Enables channel 1
	LL_TIM_EnableIT_CC1(TIM2); // Enables interrupt on channel 1
	LL_TIM_EnableCounter(TIM2); // Enables the counter
	NVIC_EnableIRQ(TIM2_IRQn); // Enables interrupts for TIM2
	*/
	
	// Configure TIM2 for PWM
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2); // Enables clock for TIM2
	LL_TIM_SetPrescaler(TIM2, 31); // Sets the prescaler to 31, so the counter ticks at 1MHz (Divides clock by 31+1 = 32, so 1Mhz)
	LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_DOWN); // Sets the counter mode to downcounting
	LL_TIM_EnableARRPreload(TIM2); // Enables auto-reload preload (ARPE)
	LL_TIM_EnableIT_UPDATE(TIM2); // Enables interrupt on update event
	LL_TIM_EnableCounter(TIM2); // Enables the counter
	LL_TIM_SetAutoReload(TIM2, 20000 - 1); // 20000-tick auto-reload value, causes 50Hz PWM frequency (1MHz/20000 = 50Hz)

<<<<<<< HEAD:robot/main.c
	LL_TIM_OC_SetCompareCH1(TIM2, 1000); // Sets the compare value for channel 1 to 1000 (10% duty cycle, (20000/100)*100% = 10%)
=======
	LL_TIM_OC_SetCompareCH1(TIM2, 1000); // Sets the compare value for channel 1 to 1000 (5% duty cycle, (1000/20000)*100% = 5%)
>>>>>>> 180b7819379b45659c2fb5243716a7e278bf8e1e:robot/main_clint.c
	LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1); // Sets the output mode for channel 1 to PWM mode 1
	LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH1); // Enables preload for channel 1
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1); // Enables channel 1
	LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH); // Sets the output polarity for channel 1 to high
	
	LL_TIM_OC_SetCompareCH2(TIM2, 2000); // Sets the compare value for channel 2 to 2000 (10% duty cycle, (2000/20000)*100% = 10%)
	LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1); // Sets the output mode for channel 2 to PWM mode 1
	LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH2); // Enables preload for channel 2
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH2); // Enables channel 2
	LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH); // Sets the output polarity for channel 2 to high

	LL_TIM_OC_SetCompareCH3(TIM2, 3000); // Sets the compare value for channel 2 to 3000 (15% duty cycle, (3000/20000)*100% = 15%)
	LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1); // Sets the output mode for channel 2 to PWM mode 1
	LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH3); // Enables preload for channel 2
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH3); // Enables channel 2
	LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH3, LL_TIM_OCPOLARITY_HIGH); // Sets the output polarity for channel 2 to high

	LL_TIM_OC_SetCompareCH4(TIM2, 4000); // Sets the compare value for channel 2 to 4000 (20% duty cycle, (4000/20000)*100% = 20%)
	LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM1); // Sets the output mode for channel 2 to PWM mode 1
	LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH4); // Enables preload for channel 2
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH4); // Enables channel 2
	LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH4, LL_TIM_OCPOLARITY_HIGH); // Sets the output polarity for channel 2 to high

	LL_TIM_GenerateEvent_UPDATE(TIM2); // Generates an update event to load the new values into the registers
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

int map_value(int x, int in_min, int in_max, int out_min, int out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void set_servo(int position, int channel) {
	int duty_cycle = map_value(position, 0, 180, 500, 2500); // Maps the position to a duty cycle value between 1000 and 2000
	if (channel == 1) {
		LL_TIM_OC_SetCompareCH1(TIM2, duty_cycle); // Sets the duty cycle for channel 1
		printf("Compare value is %d\r\n", duty_cycle);
	}
	else if (channel == 2) {
		LL_TIM_OC_SetCompareCH2(TIM2, duty_cycle); // Sets the duty cycle for channel 2
		printf("Compare value is %d\r\n", duty_cycle);
	}
}

void TIM2_Handler(void) // This function is called when a rising edge is detected on the input capture pin
{
	if(LL_TIM_IsActiveFlag_UPDATE(TIM2)){ // Check if Timer2 caused an interrupt at 20ms
		LL_TIM_ClearFlag_UPDATE(TIM2);    // Clear interrupt flag
		
		//grab values from remote controller
		strncpy(joyStick, buff+0, 4);
		joyStick[4] = '\0';
		motorPWM_y = atoi(joyStick);

		strncpy(joyStick, buff+4, 4);
		motorPWM_x = atoi(joyStick);
	}
}

float mapToRange(int x, int minInput, int maxInput) {
    return round((float)(x - minInput) * 100 / (maxInput - minInput));  // Rounded result
}

void motorControl(void)
{
	//use mapped values
	float x_PWM, y_PWM;
	x_PWM = (int)((mapToRange(motorPWM_x, 512, 1023) / 100.0) * 20000.0);
	y_PWM = (int)((mapToRange(motorPWM_x, 512, 1023) / 100.0) * 20000.0);

	;
	
		
}





void TIM6_Handler(void) // This function is called every 1ms
{
	if (LL_TIM_IsActiveFlag_UPDATE(TIM6)) { // Flag at bit zero is true only if an update event has occured
		LL_TIM_ClearFlag_UPDATE(TIM6); // Clears the update flag

		volatile static int ms_counter1 = 0;
		volatile static int duty_cycle = 0;
		ms_counter1++; // Increments the millisecond counter
		if (ms_counter1 >= 500) {
			second_counter++;
			ms_counter1 = 0;

			if (flag.pickupFlag == true) {

				switch (pickup_state) {

				case 0:
					duty_cycle  = 1960;
					if (duty_cycle <= 160) {
						duty_cycle = 1960;
					}
					break;
					
				}


			}


		}
	}
}

void SendATCommand(char* s)
{
	char buff[40];
	printf("Command: %s", s);
	LL_GPIO_ResetOutputPin(GPIOA, BIT4); // 'set' pin to 0 is 'AT' mode.
	waitms(10);
	eputs2(s);
	egets2(buff, sizeof(buff) - 1);
	LL_GPIO_SetOutputPin(GPIOA, BIT4); // 'set' pin to 1 is normal operation mode.
	waitms(10);
	printf("Response: %s", buff);
}

void ReceptionOff(void)
{
	LL_GPIO_ResetOutputPin(GPIOA, BIT4); // 'set' pin to 0 is 'AT' mode.
	waitms(10);
	eputs2("AT+DVID0000\r\n"); // Some unused id, so that we get nothing in RXD1.
	waitms(10);
	LL_GPIO_SetOutputPin(GPIOA, BIT4); // 'set' pin to 1 is normal operation mode.
	while (ReceivedBytes2() > 0) egetc2(); // Clear FIFO
}



void main(void)
{

	char number[5];
	int cnt = 0, but;
	char c;
	int timeout_cnt = 0;
	int cont1 = 0, cont2 = 100;
	float x, y;
	

	Configure_Pins();
	LCD_4BIT();
	init_timers();

	printf("Testing!!!\r\n");
	LCDprint("Loading...", 1, 1);
	LCDprint("Loading...", 2, 1);


	waitms(2000);
	/*
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

		if (ReceivedBytes2() > 0) // Something has arrived
		{
			c = egetc2();

			if (c == '!') // Master is sending message
			{
				egets2(buff, sizeof(buff) - 1);
				if (strlen(buff) != 0)
				{
					printf("Master says: %s\n\r", buff);
					//x=atof(buff[:5]);
					//y=atof(buff[7:12]);
					//but=atoi(buff[14]);
				}
				else
				{
					printf("*** BAD MESSAGE ***: %s\n\r", buff);
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
*/
		//For testing purposes, we can set the duty cycle of the PWM output based on user input

		printf("Enter a duty cycle for channel 1: ");
		fflush(stdout); // GCC peculiarities: need to flush stdout to get string out without a '\n'
		egets_echo(buff, sizeof(buff));
		printf("\r\n");
		for (int i = 0; i < sizeof(buff); i++)
		{
			if (buff[i] == '\n') buff[i] = 0;
			if (buff[i] == '\r') buff[i] = 0;
		}
		int duty_cycle1 = atoi(buff); // Convert the string to an integer
		set_servo(duty_cycle1, 1); // Set the duty cycle for channel 1 based on the first character of the input

		printf("Enter a duty cycle for channel 2: ");
		fflush(stdout); // GCC peculiarities: need to flush stdout to get string out without a '\n'
		egets_echo(buff, sizeof(buff));
		printf("\r\n");
		for (int i = 0; i < sizeof(buff); i++)
		{
			if (buff[i] == '\n') buff[i] = 0;
			if (buff[i] == '\r') buff[i] = 0;
		}
		int duty_cycle2 = atoi(buff); // Convert the string to an integer
		set_servo(duty_cycle2, 2); // Set the duty cycle for channel 2 based on the first character of the input


	}
}
