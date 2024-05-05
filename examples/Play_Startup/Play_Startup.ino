#include <Nexgen_Rover.h>
#include "pitches.h"

NXG_Rover rover = NXG_Rover(&Serial, true);

void setup() {
    rover.init(false);
    playCustomStartupSound();
    
}

// the loop function runs over and over again until power down or reset
void loop() {
    // NO CODE HERE!
}

void playCustomStartupSound() {
		rover.playTone(NOTE_C4, 0.25);	// NOTE_C4 is Middle-C on the Piano
    rover.playTone(NOTE_D4, 0.25);
    rover.playTone(NOTE_E4, 0.25);
    rover.playTone(NOTE_F4, 0.25);
    rover.playTone(NOTE_G4, 0.25);
    rover.playTone(NOTE_A4, 0.25);
    rover.playTone(NOTE_B4, 0.25);
    rover.playTone(NOTE_C5, 0.25);
}


