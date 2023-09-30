/*
 Name:		Test_Drive.ino
 Created:	9/24/2023 3:37:16 PM
 Author:	Peter Januarius
*/

// the setup function runs once when you press reset or power the board
#include <Nexgen_Rover.h>

NXG_Rover nxg = NXG_Rover(&Serial, true);
int num_times_to_repeat = 1;

void setup() {
	
	Serial.begin(9600);
	nxg.init(false);

	// Delay start for 2 seconds
	delay(2000);
}

// the loop function runs over and over again until power down or reset
void loop() {
    if (num_times_to_repeat < 2) {             // When num_times_to_repeat reaches 2, the code inside the 'if' won't run

        nxg.forward(150, 150);
        delay(1000);
        nxg.setSpeed(150, 0);
        delay(1500);
        nxg.setSpeed(200, 200);
        delay(500);
        nxg.stop();

        num_times_to_repeat++;                  // Add 1 to num_times_to_repeat
    }
}
