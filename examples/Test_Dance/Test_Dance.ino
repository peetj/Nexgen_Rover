#include <Nexgen_Rover.h>

NXG_Rover rover = NXG_Rover(&Serial, false);
//NXG_Rover rover = NXG_Rover(nullptr, false);
int num_times_to_repeat = 1;


void setup() {
    rover.setServo(90);
    rover.init(false);    
    rover.setBPM(139);
}

// The loop function runs over and over again until power down or reset
void loop() {
    if (num_times_to_repeat > 1) return;  // This line ensures that the code below will ONLY run ONCE

    /** TEST MOVEMENT **/
    rover.forward(0.5f, true);
    rover.delayFor(1.5, true);
    rover.forward(0.5f, true);
    rover.delayFor(1.5, true);
    rover.backward(0.5f, true);
    rover.delayFor(1.5, true);
    rover.backward(0.5f, true);
    rover.delayFor(1.5, true);
    // rover.backward(1, true);
    // rover.delayFor(2, true);
    // rover.turnLeft(2, true);
    // rover.delayFor(2, true);
    // rover.turnRight(2, true);
    // rover.delayFor(2, true);

    /** TEST SERVO MOVEMENT **/
    // rover.lookLeft(2);
    // rover.lookRight(2);
    // rover.lookCenter(1);
    // rover.theLook(8,1);
    // rover.lookCenter(1);

    num_times_to_repeat++;
}

/** DANCE MOVE FUNCTIONS - WE HAVE INCLUDED 9 DISTINCT DANCE MOVES BELOW TO GET YOU STARTED **/


/*************************************************************************************
  CODE EXPLANATION
  ----------------

  Putting all this into English.....,
  The Rover travels forward for 1 second at 80% speed on both motors and then stops
**************************************************************************************/