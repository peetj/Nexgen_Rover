/*
 Name:		Rover_Examples_Drive_Around_Desk.ino
 Created:	9/25/2023 5:20:52 PM
 Author:	Peter Januarius
*/

// the setup function runs once when you press reset or power the board
#include <Nexgen_Rover_v4.h>

#define DEFAULT_SPEED 150

NXG_Rover nxg = NXG_Rover(&Serial, true);
boolean hasStopped = false;     

void setup() {

    Serial.begin(9600);
    nxg.init(true);

    // Delay start for 2 seconds
    delay(2000);
}

// The loop function runs over and over again until power down or reset
void loop() {
    if (hasStopped == true) return;
    /*** YOUR CODE STARTS HERE ***/

    nxg.forward(DEFAULT_SPEED, DEFAULT_SPEED);    // Moves forward for 1 second and stops
    delay(1000);
    nxg.stop();
    delay(500);                                   // We wait half a second for the wheels to grind to a halt

    nxg.forward(DEFAULT_SPEED, 0);                // Turns right (you will most probably have to modify this) for 1 second and stops
    delay(1000);
    nxg.stop();
    delay(500);                                   // We wait half a second for the wheels to grind to a halt

    /*** YOUR CODE ENDS HERE ***/
    hasStopped = true;
}
