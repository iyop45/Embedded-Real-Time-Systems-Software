/***************************************************************************//**
 *
 * @file		Tune.h
 * @brief		Header file for a timer interrupt speaker driver
 * @author		Geoffrey Daniels, Dimitris Agrafiotis
 * @version		1.0
 * @date		06 August. 2012
 * @warning		Initialize GPIO before calling any functions in this file.
 *
 * Copyright(C) 2012, University of Bristol
 * All rights reserved.
 *
*******************************************************************************/

#ifndef TUNE_H
#define TUNE_H

#define TUNE_INCLUDE_SAMPLESONGS

/// @brief 		Sample songs, structure: {NDP}{NDP}... Note, Duration, Pause
#ifdef TUNE_INCLUDE_SAMPLESONGS
static char *Tune_SampleSongs[] = {
	"B2,E2,E4,E2,E2,E4,E2,G2,C2,D2,E8,F2,F2,F2,F2,F2,E2,E2,E2,E2,D2,D2,E2,D4,G4,E2,E2,E4,E2,E2,E4,E2,G2,C2,D2,E8,F2,F2,F2,F2,F2,E2,E2,E2,G2,G2,F2,D2,C8.",
	"D4,B4,B4,A4,A4,G4,E4,D4.D2,E4,E4,A4,F4,D8.D4,d4,d4,c4,c4,B4,G4,E4.E2,F4,F4,A4,A4,G8.",
};
static char *SampleSong2[] = {
	"C2.C2,D4,C4,F4,E8,C2.C2,D4,C4,G4,F8,C2.C2,c4,A4,F4,E4,D4,A2.A2,H4,F4,G4,F8,",
	"D4,B4,B4,A4,A4,G4,E4,D4.D2,E4,E4,A4,F4,D8.D4,d4,d4,c4,c4,B4,G4,E4.E2,F4,F4,A4,A4,G8,"
};

static char *SongPlayList = {Tune_SampleSongs, SampleSong2};
static const char Tune_SampleSongCount = 2;
#endif

/// @brief 		Initialize the tune driver
/// @warning	Initialize GPIO before calling any functions in this file.
void Tune_Init(void);

/// @brief 		Check if the tune is playing
///	@returns	1 if the tune is playing otherwise 0
/// @warning	Initialize the tune driver before running this function
uint8_t Tune_IsPlaying(void);

/// @brief      Play a song
/// @param[in]  A null-terminated string of ascii notes, ensure it is static
/// @warning	Initialize the tune driver before running this function
void Tune_PlaySong(char* SongString);

/// @brief 		Check if the tune is paused
///	@returns	1 if the tune is paused otherwise 0
/// @warning	Initialize the tune driver before running this function
uint8_t Tune_IsPaused(void);

/// @brief      Pause playing
/// @warning	Initialize the tune driver before running this function
void Tune_PauseSong(void);

/// @brief      Stop playing
/// @warning	Initialize the tune driver before running this function
void Tune_StopSong(void);

/// @brief      Set song tempo
/// @param[in]  A positive number to speed up, negative to slow down
/// @warning	Initialize the tune driver before running this function
void Tune_SetTempo(int8_t Tempo);

/// @brief      Set song pitch
/// @param[in]  A positive number to go up, negative to go down
/// @warning	Initialize the tune driver before running this function
void Tune_SetPitch(int8_t Pitch);

void TIMER0_IRQHandler(void);

#endif // TUNE_H
