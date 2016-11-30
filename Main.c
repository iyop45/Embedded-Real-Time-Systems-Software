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
/*static void DelayMS(int Length)
{
   volatile int Delay;
   volatile int D;
   for (Delay=0; Delay<Length*3000; Delay++)
   {
	   D = Delay;
   }
}*/

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


	/*
	PINSEL_CFG_Type PinCfg; //code from example


	 * Init ADC pin connect
	 * AD0.0 on P0.23

	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Pinnum = 23;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);

	/* Configuration for ADC :
	 * 	Frequency at 0.2Mhz
	 *  ADC channel 0, no Interrupt

	ADC_Init(LPC_ADC, 200000);
	ADC_IntConfig(LPC_ADC,ADC_CHANNEL_0,DISABLE);
	ADC_ChannelCmd(LPC_ADC,ADC_CHANNEL_0,ENABLE);
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
	Init_ADC();
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

	// Port 0
	// SW3 | Joystick DOWN P0[15] | Joystick RIGHT P0[16] | Joystick CENTER P0[17] | Quadrature Rotary Switch P0[24] | Quadrature Rotary Switch P0[25]
	GPIO_IntCmd(0,1 << 4 | 1 << 15 | 1 << 16 | 1 << 17 | 1 << 24 | 1 << 25, 0);

	// Port 2
	// Joystick UP P2[3] | Joystick LEFT P2[4] | Encoder(Left) | Encoder(Right)
	GPIO_IntCmd(2,1 << 3 | 1 << 4 |  1 << 11 | 1 << 12, 0 );

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

	//OLED_FillCircle(20, 20, 50, OLED_COLOR_BLACK);
	OLED_LineRect(3, 5, OLED_DISPLAY_WIDTH-3, OLED_DISPLAY_HEIGHT-5, OLED_COLOR_BLACK);
	OLED_FillCircle(OLED_DISPLAY_WIDTH-10, 10, 2, OLED_COLOR_BLACK);

	// Set the seven segment to the current 'gear'
	Gear = DFR_GetGear();
	SevenSegment_SetCharacter('0' + Gear, FALSE);


	TIM_COUNTERCFG_Type timer_config;
	timer_config.CounterOption = TIM_COUNTER_INCAP0;

	TIM_Init(LPC_TIM0, TIM_COUNTER_RISING_MODE, &timer_config);

	RotarySwitch_Init();

	movementSpeed = 100;

	DFR_IncGear();
	DFR_IncGear();

	//enum movement {LEFT, RIGHT, FORWARDS, BACKWARDS, IDLE};
	//enum movement currentMovement;

	DFR_ClearWheelCounts();

	// Main program loop
	while (1)
	{
		// Should be empty
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
uint8_t isPlayingSong = 0;
uint8_t isPausedSong  = 0;

enum configState {GEAR, TEMPO, PITCH};
enum configState currentConfigState = GEAR;

enum gearState {INC, DEC};
enum gearState currentGearState = INC;

uint8_t gearMutex = 0;
uint8_t LeftWheelCount;
uint8_t RightWheelCount;

uint8_t LeftDestination;
uint8_t RightDestination;

void EINT3_IRQHandler (void)
{

	// Encoder input 1 (Left)
	if ((((LPC_GPIOINT->IO2IntStatR) >> 11)& 0x1) == ENABLE)
	{
		LeftDestination = DFR_GetLeftWheelDestination();
		switch(currentMovement){
			case FORWARDS:
				// Check if its finished moving forwards
				if(DFR_GetLeftWheelCount() > LeftDestination){
					// Reached destination
					DFR_DriveStop();
					currentMovement = IDLE;
					DFR_SetLeftWheelCount(0);
					DFR_SetRightWheelCount(0);

					pca9532_setLeds(0x0000,0xFFFF); // reset LEDs

					WriteOLEDString((uint8_t*)"Stopped     ", 1, 1);
				}else{
					// Not there yet, so increment movement count
					DFR_IncLeftWheelCount();
				}
				break;
			case BACKWARDS:
				if(DFR_GetLeftWheelCount() > LeftDestination){//<
					DFR_DriveStop();
					currentMovement = IDLE;
					DFR_SetLeftWheelCount(0);
					DFR_SetRightWheelCount(0);

					pca9532_setLeds(0x0000,0xFFFF); // reset LEDs

					WriteOLEDString((uint8_t*)"Stopped     ", 1, 1);
				}else{
					DFR_IncLeftWheelCount();
				}
				break;
			case LEFT:
				if(DFR_GetLeftWheelCount() > LeftDestination){//<
					DFR_DriveStop();
					currentMovement = IDLE;
					DFR_SetLeftWheelCount(0);
					DFR_SetRightWheelCount(0);

					pca9532_setLeds(0x0000,0xFFFF); // reset LEDs

					WriteOLEDString((uint8_t*)"Stopped     ", 1, 1);
				}else{
					DFR_IncLeftWheelCount();
				}
				break;
			case RIGHT:
				if(DFR_GetLeftWheelCount() > LeftDestination){
					DFR_DriveStop();
					currentMovement = IDLE;
					DFR_SetLeftWheelCount(0);
					DFR_SetRightWheelCount(0);

					pca9532_setLeds(0x0000,0xFFFF); // reset LEDs

					WriteOLEDString((uint8_t*)"Stopped     ", 1, 1);
				}else{
					DFR_IncLeftWheelCount();
				}
				break;
		}

		LeftWheelCount = DFR_GetLeftWheelCount();
	}
	// Encoder input 2 (Right)
	else if ((((LPC_GPIOINT->IO2IntStatR) >> 12)& 0x1) == ENABLE)
	{
		RightDestination = DFR_GetRightWheelDestination();
		switch(currentMovement){
			case FORWARDS:
				// Check if its finished moving forwards
				if(DFR_GetRightWheelCount() > RightDestination){
					// Reached destination
					DFR_DriveStop();
					currentMovement = IDLE;
					DFR_SetLeftWheelCount(0);
					DFR_SetRightWheelCount(0);

					pca9532_setLeds(0x0000,0xFFFF); // reset LEDs

					WriteOLEDString((uint8_t*)"Stopped     ", 1, 1);
				}else{
					// Not there yet, so increment movement count
					DFR_IncRightWheelCount();
				}
				break;
			case BACKWARDS:
				if(DFR_GetRightWheelCount() > RightDestination){
					DFR_DriveStop();
					currentMovement = IDLE;
					DFR_SetLeftWheelCount(0);
					DFR_SetRightWheelCount(0);

					pca9532_setLeds(0x0000,0xFFFF); // reset LEDs

					WriteOLEDString((uint8_t*)"Stopped     ", 1, 1);
				}else{
					DFR_IncRightWheelCount();
				}
				break;
			case LEFT:
				if(DFR_GetRightWheelCount() > RightDestination){
					DFR_DriveStop();
					currentMovement = IDLE;
					DFR_SetLeftWheelCount(0);
					DFR_SetRightWheelCount(0);

					pca9532_setLeds(0x0000,0xFFFF); // reset LEDs

					WriteOLEDString((uint8_t*)"Stopped     ", 1, 1);
				}else{
					DFR_IncRightWheelCount();
				}
				break;
			case RIGHT:
				if(DFR_GetRightWheelCount() > RightDestination){
					DFR_DriveStop();
					currentMovement = IDLE;
					DFR_SetLeftWheelCount(0);
					DFR_SetRightWheelCount(0);

					pca9532_setLeds(0x0000,0xFFFF); // reset LEDs

					WriteOLEDString((uint8_t*)"Stopped     ", 1, 1);
				}else{
					DFR_IncRightWheelCount();
				}
				break;
		}

		RightWheelCount = DFR_GetRightWheelCount();
	}

	// Quadrature Rotary Switch (Forward)
	if ((((LPC_GPIOINT->IO0IntStatR) >> 24)& 0x1) == ENABLE)
	{
		Tune_IncTempo();
		WriteOLEDString((uint8_t*)"Tempo: ", 3, 1);
		WriteOLEDString((uint32_t*)Tune_GetTempo(), 3, 7);
	}
	// Quadrature Rotary Switch (Backward)
	else if((((LPC_GPIOINT->IO0IntStatR) >> 25)& 0x1) == ENABLE){
		Tune_DecTempo();
		WriteOLEDString((uint8_t*)"Tempo: ", 3, 1);
		WriteOLEDString((uint32_t*)Tune_GetTempo(), 3, 7);
	}

	// Left Button 1 (Start / Pause) P0[4]
	if ((((LPC_GPIOINT->IO0IntStatR) >> 4)& 0x1) == ENABLE)
	{
		// Toggle between pause and play
		if(Tune_IsPlaying()){
			// Pause
			Tune_PauseSong();

			WriteOLEDString((uint8_t*)"Paused      ", 2, 1);
		}else{
			// Play
			Tune_PlaySong(Tune_SampleSongs[0]);

			WriteOLEDString((uint8_t*)"Playing     ", 2, 1);
		}
	}
	// Right Button 2 (Stop) P1[31] is done through polling...


	// Joystick UP P2[3]
	if ((((LPC_GPIOINT->IO2IntStatR) >> 3)& 0x1) == ENABLE)
	{
		//DFR_ClearWheelCounts();
		DFR_SetRightWheelDestination(10);
		DFR_SetLeftWheelDestination(10);

		// Reset wheel counters
		DFR_SetLeftWheelCount(0);
		DFR_SetRightWheelCount(0);

		pca9532_setLeds(0x0000,0xFFFF); // reset LEDs
		pca9532_setLeds(0xFF00,0x0000); // only green LEDs

		WriteOLEDString((uint8_t*)"Moving FWD  ", 1, 1);
		currentMovement = FORWARDS;

		DFR_DriveForward(movementSpeed);
	}
	// Joystick DOWN P0[15]
	else if ((((LPC_GPIOINT->IO0IntStatR) >> 15)& 0x1) == ENABLE){
		//DFR_ClearWheelCounts();
		DFR_SetRightWheelDestination(10);
		DFR_SetLeftWheelDestination(10);

		// Reset wheel counters
		DFR_SetLeftWheelCount(0);
		DFR_SetRightWheelCount(0);

		pca9532_setLeds(0x0000,0xFFFF); // reset LEDs
		pca9532_setLeds(0x00FF,0x0000); // only red LEDs

		WriteOLEDString((uint8_t*)"Moving BCKWD", 1, 1);
		currentMovement = BACKWARDS;

		DFR_DriveBackward(movementSpeed);
	}
	// Joystick LEFT P2[4]
	else if ((((LPC_GPIOINT->IO2IntStatR) >> 4)& 0x1) == ENABLE){
		//DFR_ClearWheelCounts();
		DFR_SetRightWheelDestination(5);
		DFR_SetLeftWheelDestination(5);

		// Reset wheel counters
		DFR_SetLeftWheelCount(0);
		DFR_SetRightWheelCount(0);

		pca9532_setLeds(0x0000,0xFFFF); // reset LEDs
		pca9532_setLeds(0xF0F0,0x0000); //  green LEDs bottom, red LEDs top

		WriteOLEDString((uint8_t*)"Turning LFT ", 1, 1);
		currentMovement = LEFT;

		DFR_DriveLeft(movementSpeed);
	}
	// Joystick RIGHT P0[16]
	else if ((((LPC_GPIOINT->IO0IntStatR) >> 16)& 0x1) == ENABLE){
		//DFR_ClearWheelCounts();
		DFR_SetRightWheelDestination(5);
		DFR_SetLeftWheelDestination(5);

		// Reset wheel counters
		DFR_SetLeftWheelCount(0);
		DFR_SetRightWheelCount(0);

		pca9532_setLeds(0x0000,0xFFFF); // reset LEDs
		pca9532_setLeds(0x0F0F,0x0000); // green LEDs top, red LEDs bottom

		WriteOLEDString((uint8_t*)"Turning RT  ", 1, 1);
		currentMovement = RIGHT;

		DFR_DriveRight(movementSpeed);
	}
	// Joystick CENTER P0[17]
	else if ((((LPC_GPIOINT->IO0IntStatR) >> 17)& 0x1) == ENABLE){
		// Cycle through the gear values

		if(currentGearState == INC){
			// Increment the gear
			DFR_IncGear();
		}else{
			// Decrement the gear
			DFR_DecGear();
		}

		// Set the seven segment to the current 'gear'
		Gear = DFR_GetGear();

		if(Gear == 4){
			currentGearState = DEC;
		}else if(Gear == 1){
			currentGearState = INC;
		}

		SevenSegment_SetCharacter('0' + Gear, FALSE);
	}

	// Clear GPIO Interrupt Flags

	// Port 0
	// SW3 (Left button) | Joystick DOWN | Joystick RIGHT | Joystick CENTER | Quadrature Rotary Switch P0[24] | Quadrature Rotary Switch P0[25]
    GPIO_ClearInt(0,1 << 4 | 1 << 15 | 1 << 16 | 1 << 17 | 1 << 24 | 1 << 25);

    // Port 2
    // Joystick UP | Encoder(Left) | Encoder(Right) | Joystick LEFT
    GPIO_ClearInt(2,1 << 3 | 1 << 11 | 1 << 12 | 1 << 4 );
}
