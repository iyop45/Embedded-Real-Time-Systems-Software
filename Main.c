/**************************************************************************//**
 *
 * @file        Main.c
 * @brief       Entry point for the program
 * @author      Geoffrey Daniels, Dimitris Agrafiotis
 * @author		Sam Walder, Jeremy Dalton 12/2012
 * @version     1.0
 * @date        19 July. 2012
 *
 * Copyright(C) 2012, University of Bristol
 * All rights reserved.
 *
******************************************************************************/


/******************************************************************************
 * Includes
 *****************************************************************************/
// Standard C library
#include <stdio.h>
#include <string.h>

// LPC17xx definitions for CMSIS
#include "LPC17xx.h"

// LPC17xx drivers (that use CMSIS)
#include "LPC17xx_Types.h"
#include "LPC17xx_PinSelect.h"
#include "LPC17xx_GPIO.h"
#include "LPC17xx_SSP.h"
#include "LPC17xx_I2C.h"
#include "LPC17xx_ADC.h"
#include "LPC17xx_UART.h"
#include "LPC17xx_Timer.h"
#include "LPC17xx_SysTick.h"
#include "LPC17xx_LED2.h"

// Baseboard drivers (that use LPC17xx drivers)
#include "dfrobot.h"
#include "OLED.h"
#include "Buttons.h"
#include "RotarySwitch.h"
#include "SevenSegment.h"
#include "Tune.h"
#include "pca9532.h"
#include "joystick.h"
#include "new_string.h"


/******************************************************************************
 * Defines and typedefs
 *****************************************************************************/
// PCADC / PCAD
#define ADC_POWERON (1 << 12)
#define PCLK_ADC 24
#define PCLK_ADC_MASK (3 << 24)

// AD0.0 - P0.23, PINSEL1 [15:14] = 01
#define SELECT_ADC0 (0x1<<14)

// ADOCR constants
#define START_ADC (1<<24)
#define OPERATIONAL_ADC (1 << 21)
#define SEL_AD0 (1 <<0)
#define ADC_DONE_BIT	(1 << 31)

/******************************************************************************
 * Global variables
 *****************************************************************************/
uint8_t Gear = 1;

uint8_t Stop = 0;

uint16_t ADCval;


/******************************************************************************
 * Local Functions
 *****************************************************************************/
 
 /******************************************************************************
 * Description:
 *    Simple delaying function. Not good. Blocking code.
 *****************************************************************************/
static void DelayMS(int Length)
{
   volatile int Delay;
   volatile int D;
   for (Delay=0; Delay<Length*3000; Delay++)
   {
	   D = Delay;
   }
}

/******************************************************************************
 * Description:
 *    Flash the LED on the LPC1769 as a heart beat
 *****************************************************************************/
void SysTick_Handler(void)    {
	LED2_Invert();
}

void Init_SysTick1000ms(void) { if (SysTick_Config(SystemCoreClock / 1))    { while(1) { ; } } }
void Init_SysTick100ms(void)  { if (SysTick_Config(SystemCoreClock / 10))   { while(1) { ; } } }
void Init_SysTick10ms(void)   { if (SysTick_Config(SystemCoreClock / 100))  { while(1) { ; } } }
void Init_SysTick1ms(void)    { if (SysTick_Config(SystemCoreClock / 1000)) { while(1) { ; } } }

/******************************************************************************
 * Description: Initialise the SPI hardware
 *    
 *****************************************************************************/
static void Init_SSP(void)
{
	SSP_CFG_Type SSP_Config;
	PINSEL_CFG_Type PinConfig;

	// Initialize SPI pin connect
	// P0.7 - SCK; P0.8 - MISO; P0.9 - MOSI; P2.2 - SSEL - used as GPIO
	PinConfig.Funcnum = 2;
	PinConfig.OpenDrain = 0;
	PinConfig.Pinmode = 0;
	PinConfig.Portnum = 0;
	PinConfig.Pinnum = 7;
	PINSEL_ConfigPin(&PinConfig);
	PinConfig.Pinnum = 8;
	PINSEL_ConfigPin(&PinConfig);
	PinConfig.Pinnum = 9;
	PINSEL_ConfigPin(&PinConfig);
	PinConfig.Funcnum = 0;
	PinConfig.Portnum = 2;
	PinConfig.Pinnum = 2;
	PINSEL_ConfigPin(&PinConfig);
	SSP_ConfigStructInit(&SSP_Config);

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP1, &SSP_Config);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP1, ENABLE);
}

/******************************************************************************
 * Description: Initialise the I2C hardware
 *    
 *****************************************************************************/
static void Init_I2C(void)
{
	PINSEL_CFG_Type PinConfig;

	/* Initialize I2C2 pin connect */
	PinConfig.Funcnum = 2;
	PinConfig.Pinnum = 10;
	PinConfig.Portnum = 0;
	PINSEL_ConfigPin(&PinConfig);
	PinConfig.Pinnum = 11;
	PINSEL_ConfigPin(&PinConfig);

	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	// Enable I2C1 operation
 	I2C_Cmd(LPC_I2C2, I2C_MASTER_MODE, ENABLE);
}

/******************************************************************************
 * Description: Configure an ADC channel
 *    
 *****************************************************************************/
static void Init_ADC(void)
{
	/*
	PINSEL_CFG_Type PinConfig;
	PinConfig.Funcnum = 1;
	PinConfig.OpenDrain = 0;
	PinConfig.Pinmode = 0;
	PinConfig.Pinnum = 23;			// Channel AD0.0
	PinConfig.Portnum = 0;
    PINSEL_ConfigPin(&PinConfig);


	ADC_Init(LPC_ADC, 200000);
	ADC_ChannelCmd(LPC_ADC,0,ENABLE);
	ADC_IntConfig(LPC_ADC,ADC_ADINTEN0,SET);
	ADC_BurstCmd(LPC_ADC,1);
	ADC_StartCmd(LPC_ADC,ADC_START_CONTINUOUS);
	*/
}

/******************************************************************************
 * Description: Run some init functions
 *    
 *****************************************************************************/
void Init(void)
{
	// LPC1769
	LED2_Init();
	Init_SysTick100ms();
	Init_SSP();
	Init_I2C();
	//Init_ADC();
	LED2_On();

	// Baseboard
	Tune_Init();
	OLED_Init();
	RotarySwitch_Init();
	SevenSegment_Init();
	pca9532_init();
	joystick_init();

	// Extra Hardware
	DFR_RobotInit();

	// GPIO Interrupts Enable
	// SW3
	GPIO_IntCmd(0,1 << 4, 0);
	// Joystick
	GPIO_IntCmd(2,1 << 3, 0);
	
	GPIO_IntCmd(2,1 << 3, 0);

	// ADD OTHET INTERRUPTS ROY

	// Enable GPIO Interrupts
	NVIC_EnableIRQ(EINT3_IRQn);
}

enum movement {LEFT, RIGHT, FORWARDS, BACKWARDS, IDLE};
enum movement currentMovement;
uint8_t movementSpeed;

/******************************************************************************
 * Description: Main program entry point
 *    
 *****************************************************************************/
int main (void)
{
	// Globally disable interrupts
	__disable_irq();

	// Initialise
	Init();

	// Globally enable interrupts
	__enable_irq();

  	// Initialise OLED contents
	OLED_ClearScreen(OLED_COLOR_WHITE);

	// Set the seven segment to the current 'gear'
	Gear = DFR_GetGear();
	SevenSegment_SetCharacter('0' + Gear, FALSE);

	TIM_COUNTERCFG_Type timer_config;
	timer_config.CounterOption = TIM_COUNTER_INCAP0;

	TIM_Init(LPC_TIM0, TIM_COUNTER_RISING_MODE, &timer_config);

	RotarySwitch_Init();

	movementSpeed = 100;

	//changeGear(2);
	DFR_IncGear();
	DFR_IncGear();

	//enum movement {LEFT, RIGHT, FORWARDS, BACKWARDS, IDLE};
	//enum movement currentMovement;

	// Main program loop
	while (1)
	{
		DFR_ClearWheelCounts();

		// Poll for the joystick
		switch(joystick_read()){
			case JOYSTICK_CENTER:
					// m
				break;
			case JOYSTICK_DOWN:
				// Moved to interrupt
				break;
			case JOYSTICK_RIGHT:
					// m
				break;
			case JOYSTICK_UP:
				// Moved into interrupt
				break;
			case JOYSTICK_LEFT:
					//m
				break;
		}

		/*switch(currentMovement){
			case BACKWARDS:
				DFR_DecRightWheelCount();
				DFR_DecLeftWheelCount();
				break;
			case FORWARDS:
				DFR_IncRightWheelCount();
				DFR_IncLeftWheelCount();
				break;
			case LEFT:
				DFR_IncLeftWheelCount();
				break;
			case RIGHT:
				DFR_IncRightWheelCount();
				break;
		}*/



		// Poll for switch
		//WriteOLEDString((uint8_t*)RotarySwitch_Read(), 7, 0);

		// Set the seven segment to the current 'gear'
		Gear = DFR_GetGear();
		SevenSegment_SetCharacter('0' + Gear, FALSE);

		if(Buttons_Read1() == 0){
			WriteOLEDString((uint8_t*)"Button1 has been pressed", 1, 0);
		}

		if(Buttons_Read2() == 0){
			WriteOLEDString((uint8_t*)"Button2 has been pressed", 2, 0);
		}

		/*TIM_ResetCounter(LPC_TIM0);
		TIM_Cmd(LPC_TIM0, ENABLE);

		// Hmmm...maybe the ADC could trigger an interrupt when it finishes a conversion
		// instead of doing this.
		// ADCval = ADC_ChannelGetData(LPC_ADC, 0);

		// Could this button be active low?
		if (Buttons_Read2() == 0)
		{
			if(DFR_GetGear() != 3)
			{
				DelayMS(500);
				DFR_IncGear();
				Gear = DFR_GetGear();
				SevenSegment_SetCharacter('0' + Gear, FALSE);

				DelayMS(500);
				DFR_IncGear();
				Gear = DFR_GetGear();
				SevenSegment_SetCharacter('0' + Gear, FALSE);
			}

			while (1)
			{
				// Wheel destinations won't actually work. But Interrupts might
				// be able help us here with this sort of a problem. dfrobot.c
				// might also gives us some clues about how we could use
				// destinations and wheel encoder counts.
				DelayMS(500);
				DFR_ClearWheelCounts();
				DFR_SetRightWheelDestination(10);
				DFR_SetLeftWheelDestination(10);
				DFR_DriveForward(100);

				while (Stop == 0)
				{
					// Spin here until JOY UP is pressed
				}

				DFR_ClearWheelCounts();
				DFR_DriveStop();
				break;
			}

			// Play a song
			Tune_PlaySong(Tune_SampleSongs[0]);

			// Break out ready to drive forward on SW4 again.
			Stop = 0;
		}

		//WriteOLEDString((uint8_t*)converPtrToTimeNum(LPC_TIM0), 5, 0);*/

	}
}

void changeGear(uint8_t gear){
	uint8_t currentGear = DFR_GetGear();
	while(gear != currentGear){
		if(gear > currentGear){
			DFR_IncGear();
		}else if(gear < currentGear){
			DFR_DecGear();
		}
	}
}

/******************************************************************************
 * Interrupt Service Routines
 *****************************************************************************/
void EINT3_IRQHandler (void)
{

	// Encoder input 1 (Left)
	if ((((LPC_GPIOINT->IO2IntStatR) >> 11)& 0x1) == ENABLE)
	{

	}

	// Encoder input 2 (Right)
	else if ((((LPC_GPIOINT->IO2IntStatR) >> 12)& 0x1) == ENABLE)
	{

	}


	// Joystick UP P2[3]
	if ((((LPC_GPIOINT->IO2IntStatR) >> 3)& 0x1) == ENABLE)
	{
		WriteOLEDString((uint8_t*)"JOYSTICK: UP     ", 0, 0);
		currentMovement = FORWARDS;
		WriteOLEDString((uint8_t*)"FORWARDS      ", 3, 0);

		DFR_DriveForward(movementSpeed);

		// I wonder how this function works.
		/*pca9532_setLeds(0b0000000110000000, 0xffff);
		Stop = 1;

		if (Buttons_Read2() == 0)
		{
			// Could be useful for adding second functions to things.
		}
		else
		{

		}*/
	}
	// Joystick DOWN P0[15]
	else if ((((LPC_GPIOINT->IO0IntStatR) >> 15)& 0x1) == ENABLE){
		WriteOLEDString((uint8_t*)"JOYSTICK: DOWN     ", 0, 0);
		currentMovement = BACKWARDS;
		WriteOLEDString((uint8_t*)"BACKWARDS      ", 3, 0);

		DFR_DriveBackward(movementSpeed);
	}
	// Joystick LEFT P2[4]
	else if ((((LPC_GPIOINT->IO2IntStatR) >> 4)& 0x1) == ENABLE){
		WriteOLEDString((uint8_t*)"JOYSTICK: LEFT     ", 0, 0);
		currentMovement = LEFT;
		WriteOLEDString((uint8_t*)"LEFT      ", 3, 0);

		DFR_DriveLeft(movementSpeed);
	}
	// Joystick RIGHT P0[16]
	else if ((((LPC_GPIOINT->IO0IntStatR) >> 16)& 0x1) == ENABLE){
		WriteOLEDString((uint8_t*)"JOYSTICK: RIGHT     ", 0, 0);
		currentMovement = RIGHT;
		WriteOLEDString((uint8_t*)"RIGHT       ", 3, 0);

		DFR_DriveRight(movementSpeed);
	}
	// Joystick CENTER P0[17]
	else if ((((LPC_GPIOINT->IO0IntStatR) >> 17)& 0x1) == ENABLE){
		WriteOLEDString((uint8_t*)"JOYSTICK: CENTER     ", 0, 0);
		currentMovement = IDLE;
		WriteOLEDString((uint8_t*)"IDLE      ", 3, 0);

		DFR_SetRightWheelCount(0);
		DFR_SetLeftWheelCount(0);

		DFR_DriveStop();
	}
	// Left Button
	else if ((((LPC_GPIOINT->IO0IntStatR) >> 4)& 0x1) == ENABLE)
	{

	}

	// Clear GPIO Interrupt Flags
	// SW3
    GPIO_ClearInt(0,1 << 4);
    // Joystick UP | Encoder | Encoder
    GPIO_ClearInt(2,1 << 3 | 1 << 11 | 1 << 12);

    // Joystick DOWN
    //GPIO_ClearInt(0,1 << 15);
}
