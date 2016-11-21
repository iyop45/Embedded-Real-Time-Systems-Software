/***************************************************************************//**
 *
 * @file		Tune.c
 * @brief		Source file for the speaker driver
 * @author		Geoffrey Daniels, Dimitris Agrafiotis
 * @version		1.0
 * @date		19 July. 2012
 * @warning		Initialize GPIO before calling any functions in this file.
 *
 * Copyright(C) 2012, University of Bristol
 * All rights reserved
 *
*******************************************************************************/

// Includes
#include "LPC17xx_GPIO.h"

#include "Tune.h"

//------------------------------------------------------------------------------

// Defines and typedefs
#define SPEAKER_PIN_HIGH() GPIO_SetValue(0, 1<<26);
#define SPEAKER_PIN_LOW()  GPIO_ClearValue(0, 1<<26);

// To calculate: ( 1 Second / Note Frequency (Hz) ) * 1000000 Seconds / Microsecond
// Note A4, B4, a5 and b5 are out of order to enable simpler array lookups
static uint32_t Notes[] = {
    2272, // A4 - 440 Hz
    2024, // B4 - 494 Hz
    3816, // C4 - 262 Hz  <- Middle C
    3401, // D4 - 294 Hz
    3030, // E4 - 330 Hz
    2865, // F4 - 349 Hz
    2551, // G4 - 392 Hz
    1136, // a5 - 880 Hz
    1012, // b5 - 988 Hz
    1912, // c5 - 523 Hz
    1703, // d5 - 587 Hz
    1517, // e5 - 659 Hz
    1432, // f5 - 698 Hz
    1275, // g5 - 784 Hz
};

//------------------------------------------------------------------------------

// External global variables
//...

//------------------------------------------------------------------------------

// Local variables
static char* SongStringPointer = NULL;
static uint32_t CurrentNote = 0;
static uint32_t CurrentDuration = 0;
static uint32_t CurrentPause = 0;

//------------------------------------------------------------------------------

// Local Functions
static uint32_t GetNote(uint8_t Character)
{
    if ((Character >= 'A') && (Character <= 'G'))
        return Notes[Character - 'A'];

    if ((Character >= 'a') && (Character <= 'g'))
        return Notes[Character - 'a' + 7];

    return 0;
}

static uint32_t GetDuration(uint8_t Character)
{
    if (Character < '0' || Character > '9')
        return 400;

    // Number of ms
    return (Character - '0') * (200);
}

static uint32_t GetPause(uint8_t Character)
{
    switch (Character)
    {
    case '+':
        return 0;
    case ',':
        return 5;
    case '.':
        return 20;
    case '_':
        return 30;
    default:
        return 5;
    }
}

static void DelayUS(int Length)
{
   volatile int Delay;
   volatile int D;
   for (Delay=0; Delay<Length*3; Delay++)
   {
	   D = Delay;
   }
}

static void DelayMS(int Length)
{
   volatile int Delay;
   volatile int D;
   for (Delay=0; Delay<Length*3000; Delay++)
   {
	   D = Delay;
   }
}

static void PlayNote()
{
	uint32_t Time = 0;
	if (CurrentNote > 0) {
		while (Time < (CurrentDuration * 1000)) {
			SPEAKER_PIN_HIGH();
			DelayUS(CurrentNote / 2);

			SPEAKER_PIN_LOW();
			DelayUS(CurrentNote / 2);

			Time += CurrentNote;
		}
	} /*else {
		DelayMS(CurrentDuration);
	}*/
}


static void PlaySong(void)
{
	//isPlaying = 1;
	/*while (isPaused == 0 && isPlaying == 1)
	{
		if (*SongStringPointer == 0) { Tune_StopSong(); return; }
		CurrentNote = GetNote(*SongStringPointer++);

		if (*SongStringPointer == 0) { Tune_StopSong(); return; }
		CurrentDuration = GetDuration(*SongStringPointer++);

		if (*SongStringPointer == 0) { Tune_StopSong(); return; }
		CurrentPause = GetPause(*SongStringPointer++);

		// Play note
		PlayNote();

		// Wait delay
		DelayMS(CurrentPause);
	}
	isPlaying = 0;*/
}

//------------------------------------------------------------------------------

#define SBIT_TIMER0  1
#define SBIT_TIMER1  2

#define SBIT_MR0I    0
#define SBIT_MR0R    1

#define SBIT_CNTEN   0

#define PCLK_TIMER0  2

#define MiliToMicroSec(x)  (x*1000)  /* ms is multiplied by 1000 to get us*/

unsigned int getPrescalarForUs(uint8_t timerPclkBit);

uint8_t isPlaying;
uint8_t isPaused = 0;

// Public Functions
void Tune_Init(void)
{
    GPIO_SetDir(2, 1<<0, 1);
    GPIO_SetDir(2, 1<<1, 1);

    GPIO_SetDir(0, 1<<27, 1);
    GPIO_SetDir(0, 1<<28, 1);
    GPIO_SetDir(2, 1<<13, 1);
    GPIO_SetDir(0, 1<<26, 1);

    GPIO_ClearValue(0, 1<<27); //LM4811-clk
    GPIO_ClearValue(0, 1<<28); //LM4811-up/dn
    GPIO_ClearValue(2, 1<<13); //LM4811-shutdn

    // Timer interrupts
    LPC_TIM0->MCR  = (1<<SBIT_MR0I) | (1<<SBIT_MR0R);     /* Clear TC on MR0 match and Generate Interrupt*/
    LPC_TIM0->PR   = 100;      							  /* Prescalar for 1us */
    LPC_TIM0->MR0  = MiliToMicroSec(5);                   /* Load timer value to generate 5ms delay*/
    LPC_TIM0->TCR  = (1 <<SBIT_CNTEN);                    /* Start timer by setting the Counter Enable*/
    NVIC_EnableIRQ(TIMER0_IRQn);                          /* Enable Timer0 Interrupt */

    LPC_TIM1->MCR  = (1<<SBIT_MR0I) | (1<<SBIT_MR0R);     /* Clear TC on MR0 match and Generate Interrupt*/
    LPC_TIM1->PR   = 200;      						      /* Prescalar for 1us */
    LPC_TIM1->MR0  = MiliToMicroSec(5);                   /* Load timer value to generate 500us delay*/
    LPC_TIM1->TCR  = (1 <<SBIT_CNTEN);                    /* Start timer by setting the Counter Enable*/
    NVIC_EnableIRQ(TIMER1_IRQn);                          /* Enable Timer1 Interrupt */
}

uint8_t Tune_IsPlaying(void)
{
	if(isPlaying == 1){
		return 1;
	}else{
		return 0;
	}
}

void Tune_PlaySong(char* SongString)
{
	if (!SongString) return;

	if(isPaused == 0){
		SongStringPointer = SongString;
	}

	isPaused = 0;
	isPlaying = 1;
	//PlaySong();
}

uint8_t Tune_IsPaused(void)
{
	if(isPlaying == 0){
		return 1;
	}else{
		return 0;
	}
}

void Tune_PauseSong(void)
{
	isPlaying = 0;
	isPaused = 1;
}

void Tune_StopSong(void)
{
	isPlaying = 0;
	isPaused = 0;
	SongStringPointer = NULL;
}

void Tune_SetTempo(int8_t Tempo)
{
	//...
}

void Tune_SetPitch(int8_t Pitch)
{
	//...
}




// Added
uint8_t timerCounter = 0;
enum characterType {NOTE_AND_DURATION, PAUSE};
enum characterType currentCharacterType = NOTE_AND_DURATION;
uint8_t currentDuration = 0;
uint8_t isPlayingNote = 0;

void TIMER0_IRQHandler(void)
{
    unsigned int isrMask;

    isrMask = LPC_TIM0->IR;
    LPC_TIM0->IR = isrMask;         /* Clear the Interrupt Bit */

    //WriteOLEDString((uint8_t*)"timer interrupt", 5, 0);

    if(isPlaying == 1){
    	// Passed previous duration
    	if(timerCounter >= CurrentDuration){

    		switch(currentCharacterType){
    			case NOTE_AND_DURATION:
    				// Current character is a note, so get the note and increment the song pointer
    				if (*SongStringPointer == 0) { Tune_StopSong(); return; }
    				CurrentNote = GetNote(*SongStringPointer++);

    				if (*SongStringPointer == 0) { Tune_StopSong(); return; }
    				CurrentDuration = GetDuration(*SongStringPointer++);

    				//currentDuration = 0;
    				currentCharacterType = PAUSE;

    				// Play note
    				//PlayNote();
    				isPlayingNote = 1; // Start playing the note with the timer1 interrupt
    				break;
    			case PAUSE:
    				if(isPlayingNote == 0){
						if (*SongStringPointer == 0) { Tune_StopSong(); return; }

						CurrentDuration = GetPause(*SongStringPointer++);
						currentCharacterType = NOTE_AND_DURATION;
						break;
    				}
    		}

    		timerCounter = 0;

    	}

    	timerCounter += 5;
    }
}

static void PlayNote()
{
	uint32_t Time = 0;
	if (CurrentNote > 0) {
		while (Time < (CurrentDuration * 1000)) {
			SPEAKER_PIN_HIGH();
			DelayUS(CurrentNote / 2);

			SPEAKER_PIN_LOW();
			DelayUS(CurrentNote / 2);

			Time += CurrentNote;
		}
	} /*else {
		DelayMS(CurrentDuration);
	}*/
}


//uint8_t timer1Counter = 0;
uint8_t isHigh = 1;
uint8_t frequencyTimer = 0;
uint8_t durationTimer = 0;
void TIMER1_IRQHandler(void)
{
	if(isPlayingNote == 1){
		if(durationTimer >= CurrentDuration * 1000){
			// Finished playing note
			isPlayingNote = 0;
		}else{
			if(timer1Counter <= (CurrentNote / 2)){
				if(isHigh == 1){
					isHigh = 0;
					SPEAKER_PIN_HIGH();
					//DelayUS(CurrentNote / 2);
				}else{
					isHigh = 1;
					SPEAKER_PIN_LOW();
					//DelayUS(CurrentNote / 2);

					timer1Counter += CurrentNote;

					durationTimer += Cur
				}
			}else{

			}
		}
	}else{
		timer1Counter = 0;
	}
}
