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
        return 50;
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
	} else {
		DelayMS(CurrentDuration);
	}
}


static void PlaySong(void) // *Removed*
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
uint8_t timer0InterruptLength = 5;
uint8_t timer1InterruptLength = 5;

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

    // Slower interrupt to push forward the note pointer and setPlayNote to true
    LPC_TIM0->MCR  = (1<<SBIT_MR0I) | (1<<SBIT_MR0R);       /* Clear TC on MR0 match and Generate Interrupt*/
    LPC_TIM0->PR   = 100;      							    /* Prescalar for 1us */
    LPC_TIM0->MR0  = MiliToMicroSec(timer0InterruptLength); /* Load timer value to generate 5ms delay*/
    LPC_TIM0->TCR  = (1 <<SBIT_CNTEN);                      /* Start timer by setting the Counter Enable*/
    //NVIC_EnableIRQ(TIMER0_IRQn);                          /* Enable Timer0 Interrupt */

    // Faster interrupt, to play the note
    LPC_TIM1->MCR  = (1<<SBIT_MR0I) | (1<<SBIT_MR0R);     /* Clear TC on MR0 match and Generate Interrupt*/
    LPC_TIM1->PR   = 200;      						      /* Prescalar for 1us */
    LPC_TIM1->MR0  = MiliToMicroSec(timer1InterruptLength); /* Load timer value to generate 500us delay*/
    LPC_TIM1->TCR  = (1 <<SBIT_CNTEN);                    /* Start timer by setting the Counter Enable*/
    //NVIC_EnableIRQ(TIMER1_IRQn);                          /* Enable Timer1 Interrupt */
}

uint8_t Tune_IsPlaying(void)
{
	return isPlaying;
}

void Tune_PlaySong(char* SongString)
{
	if (!SongString) return;

	if(isPaused == 1){
		isPaused = 0; // Maintained previous position in the song
	}else{
		NVIC_EnableIRQ(TIMER0_IRQn); //Enable Timer0 Interrupt
		SongStringPointer = SongString; // Reset SongStringPointer
	}

	//PlaySong(); *Removed*
	isPlaying = 1;
}

uint8_t Tune_IsPaused(void)
{
	return isPaused;
}

void Tune_PauseSong(void)
{
	isPaused = 1;
	isPlaying = 0; // Stop song immediately
}

void Tune_StopSong(void)
{
	isPlaying = 0;
	isPaused = 0;

	NVIC_DisableIRQ(TIMER0_IRQn); //Disable Timer0 Interrupt
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
//uint8_t timerCounter = 0;
//enum characterType {NOTE_AND_DURATION, PAUSE};
//enum characterType currentCharacterType = NOTE_AND_DURATION;
//uint8_t currentDuration = 0;
//uint8_t isPlayingNote = 0;

uint8_t pauseTimer = 0; // Timer for between notes
uint8_t isPlayingNote = 0;

// Slower interrupt to push forward the note pointer and setPlayNote to true
void TIMER0_IRQHandler(void)
{
    unsigned int isrMask;

    isrMask = LPC_TIM0->IR;
    LPC_TIM0->IR = isrMask; // Clear the Interrupt Bit

    // This interrupt will be called every timer0InterruptLength ms = 5ms
    if(isPlaying == 1){ // isPlaying gets set to 0 in Tune_StopSong();
		if(pauseTimer >= CurrentPause){ // When the timer has exceeded CurrentPause enter this if statement
			pauseTimer = 0; // Reset timer

			if (*SongStringPointer == 0) { Tune_StopSong(); return; } // If last character in the SongStringPointer, exit the while loop
			CurrentNote = GetNote(*SongStringPointer++);

			if (*SongStringPointer == 0) { Tune_StopSong(); return; }
			CurrentDuration = GetDuration(*SongStringPointer++);

			if (*SongStringPointer == 0) { Tune_StopSong(); return; }
			CurrentPause = GetPause(*SongStringPointer++);

			NVIC_EnableIRQ(TIMER1_IRQn); // Enable Timer1 Interrupt
			NVIC_DisableIRQ(TIMER0_IRQn); // Disable (current) Timer0 Interrupt
			isPlayingNote = 1;

			// Wait delay
			// DelayMS(CurrentPause); *Removed*
		}

		pauseTimer += timer0InterruptLength; // Add 5ms to the current timer
    }



    /*if(isPlaying == 1){
    	// Passed previous duration. This is either the duration of the note or the duration between notes
    	if(timerCounter >= CurrentDuration){

    		switch(currentCharacterType){
    			case NOTE_AND_DURATION:
    				// Current character is a note, so get the note and increment the song pointer
    				if (*SongStringPointer == 0) { Tune_StopSong(); return; }
    				CurrentNote = GetNote(*SongStringPointer++);

    				if (*SongStringPointer == 0) { Tune_StopSong(); return; }
    				CurrentDuration = GetDuration(*SongStringPointer++); // Will enter PAUSE after this duration has passed

    				//currentDuration = 0;
    				currentCharacterType = PAUSE;

    				// Play note
    				uint32_t Time = 0;
    				if (CurrentNote > 0) {
    					while (Time < (CurrentDuration * 1000)) {
    						SPEAKER_PIN_HIGH();
    						DelayUS(CurrentNote / 2);

    						SPEAKER_PIN_LOW();
    						DelayUS(CurrentNote / 2);

    						Time += CurrentNote;
    					}
    				} else {
    					DelayMS(CurrentDuration);
    				}

    				//isPlayingNote = 1; // Start playing the note with the timer1 interrupt
    				break;
    			case PAUSE:
    				//if(isPlayingNote == 0){
						if (*SongStringPointer == 0) { Tune_StopSong(); return; }

						CurrentDuration = GetPause(*SongStringPointer++);
						currentCharacterType = NOTE_AND_DURATION;
						break;
    				//}
    		}

    		timerCounter = 0;

    	}

    	timerCounter += 5;
    }*/
}

/*static void PlayNote()
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
	} else {
		DelayMS(CurrentDuration);
	}
}*/


//uint8_t isHigh = 1;
//uint8_t frequencyTimer = 0;
//uint8_t durationTimer = 0;

enum pinState {HIGH, LOW};
enum pinState currentPinState = HIGH;

uint32_t noteTimer = 0;
// Interrupt to play the note
void TIMER1_IRQHandler(void)
{
	if(isPlayingNote == 1){
		if (CurrentNote > 0) {
			// Valid note, so play it
			if(noteTimer < CurrentDuration * 1000){
				SPEAKER_PIN_HIGH();
				DelayUS(CurrentNote / 2);

				SPEAKER_PIN_LOW();
				DelayUS(CurrentNote / 2);

				noteTimer += CurrentNote;
			}else{
				// Finished playing note
				noteTimer = 0;

				NVIC_EnableIRQ(TIMER0_IRQn); // Enable Timer0 Interrupt
				NVIC_DisableIRQ(TIMER1_IRQn); // Disable (current)Timer1 Interrupt
				isPlayingNote = 0;
			}
		}else{
			// Invalid Note, so wait CurrentDuration
			if(noteTimer > CurrentDuration){
				// Finished waiting
				noteTimer = 0;
				NVIC_EnableIRQ(TIMER0_IRQn); // Enable Timer0 Interrupt
				NVIC_DisableIRQ(TIMER1_IRQn); // Disable (current)Timer1 Interrupt
				isPlayingNote = 0;
			}else{
				noteTimer += timer1InterruptLength;
			}
		}
	}
}
