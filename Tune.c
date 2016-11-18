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
	} else {
		DelayMS(CurrentDuration);
	}
}

/*void Tune_PlaySong(char* SongString){
	uint8_t size;
	size = songStrong;
}*/

static void PlaySong(void)
{
	while (1)
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
}

//------------------------------------------------------------------------------

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
}

uint8_t Tune_IsPlaying(void)
{
	//...
	return 0;
}

void Tune_PlaySong(char* SongString)
{
	if (!SongString) return;
	SongStringPointer = SongString;
	PlaySong();
}

uint8_t Tune_IsPaused(void)
{
	//...
	return 0;
}

void Tune_PauseSong(void)
{
	//...
}

void Tune_StopSong(void)
{
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
