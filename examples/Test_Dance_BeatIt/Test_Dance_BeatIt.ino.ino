#include <Nexgen_Rover.h>

// Initialize the rover
NXG_Rover rover = NXG_Rover(&Serial, false);
// Ensure we only perform the Arduino loop() function once
bool exitLoop = false;

void setup() {
    rover.setServo(90);      // Initialize servo to center position
    rover.init(false);       // Initialize the rover
    rover.setBPM(139);       // Set the BPM of the song
}

void loop() {
  if(exitLoop) return;

  // First 16 beats are metronome
  rover.metronome(16);

  intro();       // 8 bars


  exitLoop = true;
}

// Function for the intro (4 bars = 16 beats)
void intro() {
  for(int i=0; i<2; i++){                 // First 4 bars
    rover.shuffleForward(1, 0.8);
    rover.shuffleBackward(1, 0.8);      
    rover.head180(1, 1.2);
  }

  rover.theLook(8, 1.2);                  // 2 bars
  rover.lookCenter();
  rover.sideToSide(2, 0.7f);              // 2 bars
}