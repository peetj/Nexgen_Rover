#include <Nexgen_Rover.h>

#define DISTANCE_TO_STOP_AT 3

NXG_Rover rover = NXG_Rover(&Serial, true);
// Array of note frequencies for a C major scale (in Hz)
int scale[] = {261, 294, 329, 349, 392, 440, 494, 523}; // C4, D4, E4, F4, G4, A4, B4, C5

/******************************************************************
  START THIS TEST WITH THE ROVER ON THE FLOOR WITH SPACE AROUND IT.
*******************************************************************/
void setup() {
    rover.setServo(90);    
    rover.init(true);
}

// The loop function runs over and over again until power down or reset
void loop() {
  // Use the Touch sensor to start the test
  int isTouched = digitalRead(rover.getTouchSensor());  // Read touch sensor. If touched, its value will be 1

  if (isTouched == 1) {     
    
    // Test LEDs
    flashLED_01(10);
    flashLED_02(10);

    // Test the Buzzer
    playScale();

    // Test the Servo
    rover.scanLeft();
    rover.scanRight();

     // Use the Ultrasonic sensor to test the motors  
    while (rover.scanForward() > DISTANCE_TO_STOP_AT) {               
      // DO NOTHING
      delay(5);      
    }

    // User approaches the Rover
    rover.forward(60, 60, 0.75);                  // Move forward
  }    
}

void flashLED_01(int numFlashes){
  for (int i=0; i<10; i++) {
    digitalWrite(A1, HIGH);
    delay(200);
    digitalWrite(A1, LOW);
    delay(200);
  }
}

void flashLED_02(int numFlashes){
  for (int i=0; i<10; i++) {
    digitalWrite(A2, HIGH);
    delay(200);
    digitalWrite(A2, LOW);
    delay(200);
  }
}

void playScale() {
    // Loop through each note in the scale array
    for (int i = 0; i < sizeof(scale) / sizeof(scale[0]); i++) {
        rover.playTone(scale[i], 0.25); // Play the current note
    }
}

