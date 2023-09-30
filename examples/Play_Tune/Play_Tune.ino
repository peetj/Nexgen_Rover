/*
 Name:		Rover_Examples_Play_Tune.ino
 Created:	9/25/2023 3:37:27 PM
 Author:	Peter Januarius
*/

// the setup function runs once when you press reset or power the board
#include <Nexgen_Rover_v4.h>


NXG_Rover nxg = NXG_Rover(&Serial, true);

const int NUM_NOTES_AND_DURATIONS = 32;
int melody[] = { 
    NOTE_G4,2,NOTE_D5,2,NOTE_C5,8,NOTE_B4,8,NOTE_A4,8,
    NOTE_G5,2,NOTE_D5,4,NOTE_C5,8,NOTE_B4,8,NOTE_A4,8,
    NOTE_G5,2,NOTE_D5,4,NOTE_C5,8,NOTE_B4,8,NOTE_C5,8,
    NOTE_A4,2 
};

// const int NUM_NOTES_AND_DURATIONS = 24;
// int melody[] = {NOTE_C4,2,NOTE_E4,4,NOTE_FS4,4,NOTE_A4,8,NOTE_G4,2,NOTE_E4,4,NOTE_C4,4,NOTE_G3,4,NOTE_FS3,8,NOTE_FS3,8,NOTE_FS3,8,NOTE_G3,4 };

void setup() {

    Serial.begin(9600);
    nxg.init(true);

    // Delay start for 2 seconds
    delay(2000);

    nxg.playMelody(melody, NUM_NOTES_AND_DURATIONS);
}

// the loop function runs over and over again until power down or reset
void loop() {
    // NO CODE HERE!
}
