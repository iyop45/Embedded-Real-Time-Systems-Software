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
 	/* Add timer2 reset*/
      LPC_TIM2->MR0  = MiliToMicroSec(CurrentNote); /* Reload timer2 value to generate interrupt after current note delay */

			if (*SongStringPointer == 0) { Tune_StopSong(); return; }
			CurrentDuration = GetDuration(*SongStringPointer++);
	/* Add timer1 reset*/
      LPC_TIM1->MR0  = MiliToMicroSec(CurrentDuration); /* Reload timer1 value to generate interrupt after current duration delay */

			if (*SongStringPointer == 0) { Tune_StopSong(); return; }
			CurrentPause = GetPause(*SongStringPointer++);
	/* Add timer0 reset*/
      LPC_TIM0->MR0  = MiliToMicroSec(CurrentPause); /* Reload timer0 value to generate interrupt after current pause delay */

			NVIC_EnableIRQ(TIMER1_IRQn); // Enable Timer1 Interrupt
			NVIC_DisableIRQ(TIMER0_IRQn); // Disable (current) Timer0 Interrupt
			isPlayingNote = 1;

			// Wait delay
			// DelayMS(CurrentPause); *Removed*
		}

		pauseTimer += timer0InterruptLength; // Add 5ms to the current timer
    }
