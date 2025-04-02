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
#include "wait.h"
#include "md.h"


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


/*
Servo Motors:
Position is CH1, Magnet is CH2

Starting Position:
	CH1: 150 degrees
	CH2: 0 degrees

Pickup Position:
	CH1: 150 degrees
	CH2: 150 degrees

Picked-Up Position:
	CH1: 150 degrees
	CH2: 70 degrees

Drop Position
	CH1: 35 degrees
	CH2: 70 degrees

*/


// Use the volatile keyword for all global variables, prevents compiler from optimizing them out
volatile int x_joystick;
volatile int y_joystick;

// Bit-field struct to hold flags, add more as needed
typedef struct {
	bool printFlag : 1;
	bool pickupFlag : 1;
	bool freqFlag : 1;
	bool getPeriodFlag : 1;
	bool pickBackFlag : 1;
	bool perimeterFlag : 1;
	bool autoFlag : 1;
} flags_struct;

volatile flags_struct flag = { 0 };

void init_pins(void)
{
	// Enable GPIOA and GPIOB clocks
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA); // Enables clock for GPIOA
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB); // Enables clock for GPIOB

	// Configure pins for UART2
	LL_GPIO_SetPinSpeed(GPIOA, BIT15, LL_GPIO_SPEED_FREQ_VERY_HIGH); // Set PA2 to high speed
	LL_GPIO_SetPinSpeed(GPIOA, BIT14, LL_GPIO_SPEED_FREQ_VERY_HIGH); // Set PA3 to high speed
	LL_GPIO_SetPinSpeed(GPIOA, BIT13, LL_GPIO_SPEED_FREQ_VERY_HIGH); // Set PA4 to high speed

	LL_GPIO_SetPinMode(GPIOA, BIT13, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetOutputPin(GPIOA, BIT13); // Set PA4 to high by default (required for JDY-40 to work)

	// Configure pins for TIM2 PWM (DC Motors)
	LL_GPIO_SetPinMode(GPIOA, BIT0, LL_GPIO_MODE_ALTERNATE); // Set PA15 to alternate function mode (TIM2_CH1)
	LL_GPIO_SetPinSpeed(GPIOA, BIT0, LL_GPIO_SPEED_FREQ_VERY_HIGH); // Set PA15 to high speed
	LL_GPIO_SetPinOutputType(GPIOA, BIT0, LL_GPIO_OUTPUT_PUSHPULL); // Set PA15 to push-pull mode
	LL_GPIO_SetAFPin_0_7(GPIOA, BIT0, LL_GPIO_AF_2); // Set PA15 to AF1 (TIM2_CH1)

	LL_GPIO_SetPinMode(GPIOA, BIT1, LL_GPIO_MODE_ALTERNATE); // Set PA1 to alternate function mode (TIM2_CH1)
	LL_GPIO_SetPinSpeed(GPIOA, BIT1, LL_GPIO_SPEED_FREQ_VERY_HIGH); // Set PA1 to high speed
	LL_GPIO_SetPinOutputType(GPIOA, BIT1, LL_GPIO_OUTPUT_PUSHPULL); // Set PA1 to push-pull mode
	LL_GPIO_SetAFPin_0_7(GPIOA, BIT1, LL_GPIO_AF_2); // Set PA1 to AF2 (TIM2_CH1)

	LL_GPIO_SetPinMode(GPIOA, BIT2, LL_GPIO_MODE_ALTERNATE); // Set PA1 to alternate function mode (TIM2_CH3)
	LL_GPIO_SetPinSpeed(GPIOA, BIT2, LL_GPIO_SPEED_FREQ_VERY_HIGH); // Set PA1 to high speed
	LL_GPIO_SetPinOutputType(GPIOA, BIT2, LL_GPIO_OUTPUT_PUSHPULL); // Set PA1 to push-pull mode
	LL_GPIO_SetAFPin_0_7(GPIOA, BIT2, LL_GPIO_AF_2); // Set PA1 to AF2 (TIM2_CH3)

	LL_GPIO_SetPinMode(GPIOA, BIT3, LL_GPIO_MODE_ALTERNATE); // Set PA3 to alternate function mode (TIM2_CH4)
	LL_GPIO_SetPinSpeed(GPIOA, BIT3, LL_GPIO_SPEED_FREQ_VERY_HIGH); // Set PA3 to high speed
	LL_GPIO_SetPinOutputType(GPIOA, BIT3, LL_GPIO_OUTPUT_PUSHPULL); // Set PA3 to push-pull mode
	LL_GPIO_SetAFPin_0_7(GPIOA, BIT3, LL_GPIO_AF_2); // Set PA3 to AF2 (TIM2_CH4)

	// Configure pins for TIM22 PWM (Servo Motors)
	LL_GPIO_SetPinMode(GPIOB, BIT4, LL_GPIO_MODE_ALTERNATE); // Set PB4 to alternate function mode (TIM22_CH1)
	LL_GPIO_SetPinSpeed(GPIOB, BIT4, LL_GPIO_SPEED_FREQ_VERY_HIGH); // Set PB4 to high speed
	LL_GPIO_SetPinOutputType(GPIOB, BIT4, LL_GPIO_OUTPUT_PUSHPULL); // Set PB4 to push-pull mode
	LL_GPIO_SetAFPin_0_7(GPIOB, BIT4, LL_GPIO_AF_4); // Set PB4 to AF2 (TIM22_CH1)

	LL_GPIO_SetPinMode(GPIOB, BIT5, LL_GPIO_MODE_ALTERNATE); // Set PB5 to alternate function mode (TIM22_CH1)
	LL_GPIO_SetPinSpeed(GPIOB, BIT5, LL_GPIO_SPEED_FREQ_VERY_HIGH); // Set PB5 to high speed
	LL_GPIO_SetPinOutputType(GPIOB, BIT5, LL_GPIO_OUTPUT_PUSHPULL); // Set PB5 to push-pull mode
	LL_GPIO_SetAFPin_0_7(GPIOB, BIT5, LL_GPIO_AF_4); // Set PB5 to AF2 (TIM22_CH1)

	// Configure pin for frequency/period measurement (for metal detector)
	LL_GPIO_SetPinMode(GPIOA, BIT8, LL_GPIO_MODE_INPUT); // Set PA8 to input mode (TIM6)
	LL_GPIO_SetPinPull(GPIOA, BIT8, LL_GPIO_PULL_UP); //Set PA8 to pull-up

	// Configure pin for ADC (for perimeter detection)
	LL_GPIO_SetPinMode(GPIOA, BIT5, LL_GPIO_MODE_ANALOG); // Set PA5 to analog mode (ADC1_IN0)

	// Configure pin for electromagnet
	LL_GPIO_SetPinMode(GPIOB, BIT6, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(GPIOB, BIT6, LL_GPIO_OUTPUT_PUSHPULL);
}

void init_timers(void)
{

	// Configure TIM2 for PWM
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2); // Enables clock for TIM2
	LL_TIM_SetPrescaler(TIM2, 31); // Sets the prescaler to 31, so the counter ticks at 1MHz (Divides clock by 31+1 = 32, so 1Mhz)
	LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_DOWN); // Sets the counter mode to downcounting
	LL_TIM_EnableARRPreload(TIM2); // Enables auto-reload preload (ARPE)
	LL_TIM_EnableIT_UPDATE(TIM2); // Enables interrupt on update event
	LL_TIM_EnableCounter(TIM2); // Enables the counter
	LL_TIM_SetAutoReload(TIM2, 20000 - 1); // 20000-tick auto-reload value, causes 50Hz PWM frequency (1MHz/20000 = 50Hz)

	LL_TIM_OC_SetCompareCH1(TIM2, 0); // Sets the compare value for channel 1 to 1000 (10% duty cycle, (20000/100)*100% = 10%)
	LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1); // Sets the output mode for channel 1 to PWM mode 1
	LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH1); // Enables preload for channel 1
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1); // Enables channel 1
	LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH); // Sets the output polarity for channel 1 to high

	LL_TIM_OC_SetCompareCH2(TIM2, 0); // Sets the compare value for channel 2 to 2000 (10% duty cycle, (2000/20000)*100% = 10%)
	LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1); // Sets the output mode for channel 2 to PWM mode 1
	LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH2); // Enables preload for channel 2
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH2); // Enables channel 2
	LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH); // Sets the output polarity for channel 2 to high

	LL_TIM_OC_SetCompareCH3(TIM2, 0); // Sets the compare value for channel 2 to 3000 (15% duty cycle, (3000/20000)*100% = 15%)
	LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1); // Sets the output mode for channel 2 to PWM mode 1
	LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH3); // Enables preload for channel 2
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH3); // Enables channel 2
	LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH3, LL_TIM_OCPOLARITY_HIGH); // Sets the output polarity for channel 2 to high

	LL_TIM_OC_SetCompareCH4(TIM2, 0); // Sets the compare value for channel 2 to 4000 (20% duty cycle, (4000/20000)*100% = 20%)
	LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM1); // Sets the output mode for channel 2 to PWM mode 1
	LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH4); // Enables preload for channel 2
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH4); // Enables channel 2
	LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH4, LL_TIM_OCPOLARITY_HIGH); // Sets the output polarity for channel 2 to high

	LL_TIM_GenerateEvent_UPDATE(TIM2); // Generates an update event to load the new values into the registers
	NVIC_EnableIRQ(TIM2_IRQn); // Enables interrupts for TIM2

	// Configure TIM22 for Servo PWM
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM22); // Enables clock for TIM22
	LL_TIM_SetPrescaler(TIM22, 31); // Sets the prescaler to 31, so the counter ticks at 1MHz (Divides clock by 31+1 = 32, so 1Mhz)
	LL_TIM_SetCounterMode(TIM22, LL_TIM_COUNTERMODE_DOWN); // Sets the counter mode to downcounting
	LL_TIM_EnableARRPreload(TIM22); // Enables auto-reload preload (ARPE)
	LL_TIM_EnableIT_UPDATE(TIM22); // Enables interrupt on update event
	LL_TIM_EnableCounter(TIM22); // Enables the counter
	LL_TIM_SetAutoReload(TIM22, 20000 - 1); // 20000-tick auto-reload value, causes 50Hz PWM frequency (1MHz/20000 = 50Hz)

	LL_TIM_OC_SetCompareCH2(TIM22, 0); // Sets the compare value for channel 2 to 2000 (10% duty cycle, (2000/20000)*100% = 10%)
	LL_TIM_OC_SetMode(TIM22, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1); // Sets the output mode for channel 2 to PWM mode 1
	LL_TIM_OC_EnablePreload(TIM22, LL_TIM_CHANNEL_CH2); // Enables preload for channel 2
	LL_TIM_CC_EnableChannel(TIM22, LL_TIM_CHANNEL_CH2); // Enables channel 2
	LL_TIM_OC_SetPolarity(TIM22, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH); // Sets the output polarity for channel 2 to high

	LL_TIM_OC_SetCompareCH1(TIM22, 0); // Sets the compare value for channel 1 to 1000 (10% duty cycle, (20000/100)*100% = 10%)
	LL_TIM_OC_SetMode(TIM22, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1); // Sets the output mode for channel 1 to PWM mode 1
	LL_TIM_OC_EnablePreload(TIM22, LL_TIM_CHANNEL_CH1); // Enables preload for channel 1
	LL_TIM_CC_EnableChannel(TIM22, LL_TIM_CHANNEL_CH1); // Enables channel 1
	LL_TIM_OC_SetPolarity(TIM22, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH); // Sets the output polarity for channel 1 to high

	LL_TIM_GenerateEvent_UPDATE(TIM22); // Generates an update event to load the new values into the registers
	NVIC_EnableIRQ(TIM22_IRQn); // Enables interrupts for TIM22

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


void init_ADC(void) { // Initialize ADC for perimeter detection



	// LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1); // Enables clock for ADC1
	// LL_ADC_SetClock(ADC1, LL_ADC_CLOCK_SYNC_PCLK_DIV1); // Sets the ADC clock to PCLK/1 (assuming PCLK is 32MHz, this gives 32MHz for ADC)

	// LL_ADC_ClearFlag_ADRDY(ADC1); // Clears the ADC ready flag

	// LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_10B); // Sets the resolution to 10 bits (we don't need more than 10 bits for this, we just need a rough voltage measurement)
	// LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_1CYCLE_5); // Sets the sampling time to 1.5 cycles

	// LL_ADC_Enable(ADC1); // Enables ADC


	RCC->APB2ENR |= BIT9; // peripheral clock enable for ADC (page 175 or RM0451)

	// ADC clock selection procedure (page 746 of RM0451)
	/* (1) Select PCLK by writing 11 in CKMODE */
	ADC1->CFGR2 |= ADC_CFGR2_CKMODE; /* (1) */

	// ADC enable sequence procedure (page 745 of RM0451)
	/* (1) Clear the ADRDY bit */
	/* (2) Enable the ADC */
	/* (3) Wait until ADC ready */
	ADC1->ISR |= ADC_ISR_ADRDY; /* (1) */
	ADC1->CR |= ADC_CR_ADEN; /* (2) */
	if ((ADC1->CFGR1 & ADC_CFGR1_AUTOFF) == 0)
	{
		while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* (3) */
		{
			/* For robust implementation, add here time-out management */
		}
	}

	// Calibration code procedure (page 745 of RM0451)
	/* (1) Ensure that ADEN = 0 */
	/* (2) Clear ADEN */
	/* (3) Set ADCAL=1 */
	/* (4) Wait until EOCAL=1 */
	/* (5) Clear EOCAL */
	if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */
	{
		ADC1->CR |= ADC_CR_ADDIS; /* (2) */
	}
	ADC1->CR |= ADC_CR_ADCAL; /* (3) */
	while ((ADC1->ISR & ADC_ISR_EOCAL) == 0) /* (4) */
	{
		/* For robust implementation, add here time-out management */
	}
	ADC1->ISR |= ADC_ISR_EOCAL; /* (5) */

}


int read_ADC(unsigned int channel)
{

	//READ CHANNEL 5 FOR PA5 (ADC_CHSELR_CHSEL5)

	// Single conversion sequence code example - Software trigger (page 746 of RM0451)
	/* (1) Select HSI16 by writing 00 in CKMODE (reset value) */
	/* (2) Select the auto off mode */
	/* (3) Select channel */
	/* (4) Select a sampling mode of 111 i.e. 239.5 ADC clk to be greater than17.1us */
	/* (5) Wake-up the VREFINT (only for VRefInt) */
	//ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE; /* (1) */
	ADC1->CFGR1 |= ADC_CFGR1_AUTOFF; /* (2) */
	ADC1->CHSELR = channel; /* (3) */
	ADC1->SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2; /* (4) */
	if (channel == ADC_CHSELR_CHSEL17)
	{
		ADC->CCR |= ADC_CCR_VREFEN; /* (5) */
	}

	/* Performs the AD conversion */
	ADC1->CR |= ADC_CR_ADSTART; /* start the ADC conversion */
	while ((ADC1->ISR & ADC_ISR_EOC) == 0) /* wait end of conversion */
	{
		/* For robust implementation, add here time-out management */
	}

	return ADC1->DR; // ADC_DR has the 12 bits out of the ADC
}

int map_value(int x, int in_min, int in_max, int out_min, int out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void set_servo(int position, int channel) {
	int duty_cycle = map_value(position, 0, 180, 500, 2500); // Maps the position to a duty cycle value between 1000 and 2000
	if (channel == 1) {
		LL_TIM_OC_SetCompareCH1(TIM22, duty_cycle); // Sets the duty cycle for channel 1
		//printf("Compare value is %d\r\n", duty_cycle);
	}
	else if (channel == 2) {
		LL_TIM_OC_SetCompareCH2(TIM22, duty_cycle); // Sets the duty cycle for channel 2
		//printf("Compare value is %d\r\n", duty_cycle);
	}
	else printf("INVALID SERVO CHANNEL\r\n"); // Invalid channel
}

int get_servo(int channel) {
	int compare_value;
	if (channel == 1) {
		compare_value = LL_TIM_OC_GetCompareCH1(TIM22); // Gets the duty cycle for channel 1
		return map_value(compare_value, 500, 2500, 0, 180); // Maps the duty cycle back to a position value between 0 and 180 degrees
	}

	else if (channel == 2) {
		compare_value = LL_TIM_OC_GetCompareCH2(TIM22); // Gets the duty cycle for channel 2
		return map_value(compare_value, 500, 2500, 0, 180); // Maps the duty cycle back to a position value between 0 and 180 degrees
	}

	return -1; // Invalid channel
}


float mapToRange(int x, int minInput, int maxInput) {
	return round((float)(x - minInput) * 100 / (maxInput - minInput));  // Rounded result
}

void TIM2_Handler(void) // This function is called when a rising edge is detected on the input capture pin
{
	if (LL_TIM_IsActiveFlag_UPDATE(TIM2)) { // Check if Timer2 caused an interrupt at 20ms
		LL_TIM_ClearFlag_UPDATE(TIM2);    // Clear interrupt flag
		static int counter = 0;

		counter++;

		if (counter >= 10) {

			flag.printFlag = true; // Set the print flag to true every 200ms
			counter = 0;
		}


	}
}

void motor_control(int x, int y)
{
	//use mapped values
	int x_PWM, y_PWM;
	x_PWM = (int)((mapToRange(x, 512, 1023) / 100.0) * 20000.0);
	y_PWM = (int)((mapToRange(y, 512, 1023) / 100.0) * 20000.0);

	//printf("%d \n", x_PWM);
	//printf("%d", y_PWM);
	if (y_PWM<1000 && y_PWM>-1000)
	{
		if (x_PWM > 0)
		{
			LL_TIM_OC_SetCompareCH1(TIM2, x_PWM);
			LL_TIM_OC_SetCompareCH2(TIM2, y_PWM);
			LL_TIM_OC_SetCompareCH3(TIM2, y_PWM);
			LL_TIM_OC_SetCompareCH4(TIM2, x_PWM);
		}
		else
		{
			x_PWM = -1 * x_PWM;
			LL_TIM_OC_SetCompareCH1(TIM2, y_PWM);
			LL_TIM_OC_SetCompareCH2(TIM2, x_PWM);
			LL_TIM_OC_SetCompareCH3(TIM2, x_PWM);
			LL_TIM_OC_SetCompareCH4(TIM2, y_PWM);
		}
	}
	else if (y_PWM > 0) {
		LL_TIM_OC_SetCompareCH1(TIM2, x_PWM);
		LL_TIM_OC_SetCompareCH2(TIM2, y_PWM);
		LL_TIM_OC_SetCompareCH3(TIM2, x_PWM);
		LL_TIM_OC_SetCompareCH4(TIM2, y_PWM);
	}
	else {
		y_PWM = -1 * y_PWM;
		LL_TIM_OC_SetCompareCH1(TIM2, y_PWM);
		LL_TIM_OC_SetCompareCH2(TIM2, x_PWM);
		LL_TIM_OC_SetCompareCH3(TIM2, y_PWM);
		LL_TIM_OC_SetCompareCH4(TIM2, x_PWM);

	}


	// if(y_PWM < 0){ //for spinning backwards
	// 	y_PWM = -1*y_PWM;
	// 	LL_TIM_OC_SetCompareCH1(TIM2, y_PWM); 
	// 	LL_TIM_OC_SetCompareCH2(TIM2, x_PWM); 
	// }

}

void motor_control_smooth(int x, int y) // TESTING NEW FUNCTION, MAY NOT WORK AT ALL
{
	int x_PWM, y_PWM;
	x_PWM = (int)((mapToRange(x, 512, 1023) / 100.0) * 20000.0);
	y_PWM = (int)((mapToRange(y, 512, 1023) / 100.0) * 20000.0);

	// Compute blending factor, allows for smooth turns and purely arithmetic operations for PWM values, while still allowing for turns at full speed
	float alpha = 1.0f - ((float)abs(y_PWM) / 20000.00);  // Normalize with max speed, change to 1000.00 if changing frequency to 1kHz

	// Turn sensitivity factor, higher values cause sharper turns
	float turn_factor = 1.5f;  // Adjust this value as needed  (1.5 causes one motor to max out and one to turn at 62.5% speed when going diagonally)

	// Calculate final PWM values
	int left_PWM = y_PWM + (int)(alpha * turn_factor * x_PWM);
	int right_PWM = y_PWM - (int)(alpha * turn_factor * x_PWM);

	// Set PWM outputs for left Motor (CH1 and CH2)
	if (left_PWM > 0) {
		LL_TIM_OC_SetCompareCH1(TIM2, 0);
		LL_TIM_OC_SetCompareCH2(TIM2, left_PWM); // Goes forward
	}
	else {
		LL_TIM_OC_SetCompareCH1(TIM2, -left_PWM); // Goes backward
		LL_TIM_OC_SetCompareCH2(TIM2, 0);
	}

	// Set PWM outputs for right Motor (CH3 and CH4)
	if (right_PWM > 0) {
		LL_TIM_OC_SetCompareCH3(TIM2, 0);
		LL_TIM_OC_SetCompareCH4(TIM2, right_PWM); // Goes forward
	}
	else {
		LL_TIM_OC_SetCompareCH3(TIM2, -right_PWM); // Goes backward
		LL_TIM_OC_SetCompareCH4(TIM2, 0);
	}
}

void TIM22_Handler(void) {
	if (LL_TIM_IsActiveFlag_UPDATE(TIM22)) { // Check if TIM22 caused an interrupt
		LL_TIM_ClearFlag_UPDATE(TIM22); // Clear interrupt flag
	}
	static int state = 10;
	static int count = 0;

	if (flag.pickupFlag)
	{
		switch (state) {

		case 10:
			if (count < 20)
			{
				LL_GPIO_ResetOutputPin(GPIOB, BIT6);
				count++;
				flag.pickBackFlag = true;
			}

			else
			{
				LL_GPIO_ResetOutputPin(GPIOB, BIT6);
				count = 0;
				state = 0;
				flag.pickBackFlag = false;
			}

			break;

		case 0:
			if (get_servo(2) < 150)
			{
				LL_GPIO_SetOutputPin(GPIOB, BIT6);
				set_servo(get_servo(2) + 2, 2);
			}

			else
			{
				LL_GPIO_SetOutputPin(GPIOB, BIT6);
				state = 1;
			}

			break;

		case 1:
			if (get_servo(2) > 70)
			{
				LL_GPIO_SetOutputPin(GPIOB, BIT6);
				set_servo(get_servo(2) - 2, 2);
			}
			
			else
			{
				LL_GPIO_SetOutputPin(GPIOB, BIT6);
				state = 2;
			}

			break;

		case 2:
			if (get_servo(1) > 25)
			{
				LL_GPIO_SetOutputPin(GPIOB, BIT6);
				set_servo(get_servo(1) - 2, 1);
			}

			else
			{
				LL_GPIO_SetOutputPin(GPIOB, BIT6);
				state = 20;
			}

			break;

		case 20:
			if (count <= 10) {
				LL_GPIO_SetOutputPin(GPIOB, BIT6);
				count++;
			}

			else {
				LL_GPIO_ResetOutputPin(GPIOB, BIT6);
				state = 3;
			}

			break;

		case 3:
			if (get_servo(1) < 150)
			{
				LL_GPIO_ResetOutputPin(GPIOB, BIT6);
				set_servo(get_servo(1) + 2, 1);
			}

			else
			{
				LL_GPIO_ResetOutputPin(GPIOB, BIT6);
				state = 4;
			}

			break;

		case 4:
			if (get_servo(2) > 0)
			{
				LL_GPIO_ResetOutputPin(GPIOB, BIT6);
				set_servo(get_servo(2) - 2, 2);
			}

			else
			{
				LL_GPIO_ResetOutputPin(GPIOB, BIT6);
				flag.pickupFlag = 0;
				state = 10;
			}

			break;

		default:
			break;
		}
	}
}



void TIM6_Handler(void) // This function is called every 1ms
{
	if (LL_TIM_IsActiveFlag_UPDATE(TIM6)) { // Flag at bit zero is true only if an update event has occured
		LL_TIM_ClearFlag_UPDATE(TIM6); // Clears the update flag
		static int count = 0;

		if (count++ >= 100)
		{
			count = 0;
			flag.getPeriodFlag = true;
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



void detect_perimeter(void) { // This function sets the perimeter flag to true if the perimeter is detected

	int perimeter_adc;
	perimeter_adc = read_ADC(ADC_CHSELR_CHSEL5); // Read the ADC value from channel 5 (PA5)	
	if (perimeter_adc < 1000) // If the ADC value is less than 1000, we have detected the perimeter
	{
		flag.perimeterFlag = true; // Set the perimeter flag to true
	}
	else
	{
		flag.perimeterFlag = false; // Set the perimeter flag to false
	}

	printf("Perimeter ADC Value: %d\r\n", perimeter_adc); // Print the ADC value for debugging purposes
	printf("Perimeter Status: %d\r\n", flag.perimeterFlag); // Print the perimeter status for debugging purposes

}

void servo_debug(char* buff) {
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

void parse_buffer(char* buff) { // Parses the "buff" string containing data from the JDY-40, extracting the joystick values and button states

	char temp_joystick_string[10];
	static int temp_x = 0;
	static int temp_y = 0;

	if (buff[3] != ' ' && buff[8] != ' ') // Checks if the last number for the joystick X and Y values is not a space, and does not parse the buffer if it is a space
	{
		strncpy(temp_joystick_string, buff + 0, 4); // Extract joystick Y value from the buffer
		temp_joystick_string[4] = '\0';
		temp_y = atoi(temp_joystick_string);

		strncpy(temp_joystick_string, buff + 4, 5); // Extract joystick X value from the buffer
		temp_joystick_string[5] = '\0';
		temp_x = atoi(temp_joystick_string);

		if ((temp_x > 500) && (temp_x < 524)) { // Deadzone for joystick X
			x_joystick = 512; // Center position, no movement
		}
		else x_joystick = temp_x; // Use the value from the remote controller if outside the deadzone

		if ((temp_y > 500) && (temp_y < 524)) { // Deadzone for joystick Y
			y_joystick = 512; // Center position, no movement
		}
		else y_joystick = temp_y; // Use the value from the remote controller if outside the deadzone


		if (buff[10] == '1') flag.pickupFlag = true; // If the joystick button is pushed, set the pickup flag to true (DO NOT ADD ELSE STATEMENT, WE WANT TO KEEP THE FLAG TRUE UNTIL WE PICK UP THE COIN)

		//Add more buttons here if needed
	}

	else // If the joystick values are not valid, set the motor PWM values to center position to keep it still
	{
		x_joystick = 512;
		y_joystick = 512;
	}

}

void main(void)
{

	int cnt = 0;
	char c;
	char output_buff[20];
	char buff[80];
	int freq_diff;
	int const_freq;


	printf("Reset triggered...\r\n");
	waitms(100);

	initUART2(9600);
	ReceptionOff();
	printf("UART2 initialized...\r\n");
	waitms(100);

	// To check configuration, will only print out the first command if module is not working correctly.
	SendATCommand("AT+VER\r\n");
	SendATCommand("AT+BAUD\r\n");
	SendATCommand("AT+RFID\r\n");
	SendATCommand("AT+DVID\r\n");
	SendATCommand("AT+RFC\r\n");
	SendATCommand("AT+POWE\r\n");
	SendATCommand("AT+CLSS\r\n");

	SendATCommand("AT+DVID7788\r\n"); // Set device ID to 7788 (0x7788, from 0x0000 to 0xFFFF)
	SendATCommand("AT+RFC052\r\n"); // Set RF frequency to 052 (from 001 to 128) 
	printf("AT test commands sent...\r\n");
	waitms(100);

	init_pins();
	init_timers();
	init_ADC();
	printf("Peripherals configured...\r\n");
	waitms(100);

	set_servo(150, 1);
	set_servo(0, 2);
	LL_GPIO_SetOutputPin(GPIOB, BIT6);
	flag.freqFlag = true;
	printf("System initialized...\r\n");
	waitms(100);

	printf("ENTERING MAIN LOOP:\r\n");
	waitms(500);

	while (1) // Loop indefinitely
	{

		//Potential Changes:
		// 1. Declare a variable that gets the difference of the current and constant frequency, and reuse it for the remote speaker frequency
		// 2. Add a volatile int counter for the number of coins we have picked up in auto mode (based on detected frequency), and send it to the remote controller (maybe also for manual mode, extra feature?)
		// 3. Send flag.autoFlag to the remote, so it can see what the status is, the remote should display the status of the robot based on what it RECIEVES (not what it sends, ensures that we see the actual status and not the intended status)
		//***4. Add diagonals for the DC motors
		//***5. Create the state machine for auto mode
		// 6. Change parse_buffer to use sscanf to parse entire buffer at once, and check for errors by checking the return value of sscanf (should be 3 for 3 values)
		// 7. Add a check if the remote is not connected? (Extra feature? Could make it do something if not connected)
		// 8. Change DC PWM to 1kHz instead of 50Hz to make the speed smoother (smoother average voltage, so more accurate speed control and smoother movement), but this will require changing the prescaler and auto-reload value (20000 to 1000) for TIM2 
		// ***9. Change makefile to remove printf for floats (halves size of file being flashed), and enable verification to make sure flashing never corrupts (-v flag, makes flashing super slow for tests though) 
		// 10. If changing DC PWM to 1kHz, try using map_value instead of mapToRange and see if it works better (should be faster, entirely integer math but I'm unsure if it will work correctly, output range should be from 0-1000 instead of 0-20000)
		// 11. Center deadzone to 508 and 513 instead of 512 (joysticks are not perfectly centered, should let us reduce deadzone to +-6 while still removing most noise)


		if (flag.getPeriodFlag == true) {

			if (flag.freqFlag == true) {
				const_freq = (int)(4096000000.00 / (float)GetPeriod(128)); // 4096000000 is the clock frequency (32MHz) multiplied by the number of periods (128)
				flag.freqFlag = false;
			}

			freq_diff = (int)(2048000000.00 / (float)GetPeriod(64)) - const_freq; // 2048000000 is the clock frequency (32MHz) multiplied by the number of periods (64)

			if (freq_diff > 300) { // If the frequency difference is greater than 300, we have detected a coin) 
				// compare set frequency and measured frequency (should probably check how much it fluctuates)
				//if they're not the same, set pickupFlag to 1 to activate FSM for picking up coin
				flag.pickupFlag = true;
				printf("Moving Servo! Frequency Difference: %d\r\n", freq_diff);
			}

			flag.getPeriodFlag = false;

		}

		if (ReceivedBytes2() > 0) // Something has arrived from the remote/JDY-40 module
		{
			c = egetc2(); // Get the first character from the buffer

			if (c == '!') // If the first character is "!", the remote is sending a message
			{
				int err_check = egets2(buff, sizeof(buff) - 1); // egets2 will return -1 if an error has occurred, will also store the message in buff as a side-effect

				if ((strlen(buff) != 11) && (err_check != -1)) // Only parse the message if it is the correct length (11 characters) and if egets2 does not return an error (-1)
				{
					parse_buffer(buff); // Parse the buffer to extract joystick values and button states

					if (flag.printFlag == true) // If the print flag is set, print the joystick values and button states (for debugging only, prints every 500ms)
					{
						printf("Master says: %s\r\n\n", buff);
						printf("Pickup Flag: %d\r\n", flag.pickupFlag);
						printf("Y Position (unmapped): %d\r\n", y_joystick);
						printf("X Position (unmapped): %d\r\n", x_joystick);
						printf("X Position (mapped): %d\r\n", (int)((mapToRange(x_joystick, 512, 1023) / 100.0) * 20000.0));
						printf("Y Position (mapped): %d\r\n", (int)((mapToRange(y_joystick, 512, 1023) / 100.0) * 20000.0));
						printf("Frequency: %d\r\n", freq_diff + const_freq);
						printf("Constant Frequency: %d\r\n", const_freq);
						printf("Frequency Difference: %d\r\n", freq_diff);
						flag.printFlag = false; // Reset the print flag
					}

				}

				else
				{
					printf("*** BAD MESSAGE, NOT PARSED ***: %s\r\n", buff);
					// Not a valid message, do not parse it
				}
			}

			else if (c == '@') // If the first character is "@", the remote wants data
			{
				sprintf(output_buff, "%04d %d %d\n", freq_diff, flag.pickupFlag, flag.autoFlag); // Format the output string with the frequency difference, pickup flag, and auto flag
				waitms(5); // The radio seems to need this delay...
				eputs2(output_buff); // Send the formatted output buffer as the response, can only send one message at a time				
			}

		}


		if (flag.pickBackFlag == true) // If the pick back flag is set, move the robot back
		{
			motor_control(512, 1023); // Move the robot back at full speed
			waitms(50); // Keep the robot moving back for 50ms
		}
		else if (flag.pickupFlag == true) // If the pickup flag is set, keep the robot still
		{
			motor_control(512, 512); // Stop the robot
			waitms(50); // Keep the robot still for 50ms
		}
		else motor_control(x_joystick, y_joystick); // If no relevant flags are set, move the robot based on joystick input


	}
}
