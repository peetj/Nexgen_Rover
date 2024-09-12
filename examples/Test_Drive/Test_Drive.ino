#include <Nexgen_Rover.h>

NXG_Rover rover = NXG_Rover(&Serial, false);
int num_times_to_repeat = 1;

void setup() {
	rover.init(true);
  rover.setServo(90);
}

// The loop function runs over and over again until power down or reset
void loop() {
  if (num_times_to_repeat > 1) return;  // This line ensures that the code below will ONLY run ONCE

  rover.forward(50, 50, 1);
  rover.backward(50, 50, 2);

  num_times_to_repeat++;
}


  /*************************************************************************************
    CODE EXPLANATION
    ----------------

    "rover" is the representation of the actual rover robot (represented in code).
    "rover.forward" tells the rover to move forward

    What are the 3 numbers inside the brackets?  (80, 80, 1)

    (LeftMotorSpeed, RightMotorSpeed, TimeInSeconds)

    So....

    LeftMotorSpeed = 80
    RightMotorSpeed = 80
    TimeInSeconds = 1

    Putting all this into English....., 
    The Rover travels forward for 1 second at 80% speed on both motors and then stops
  **************************************************************************************/