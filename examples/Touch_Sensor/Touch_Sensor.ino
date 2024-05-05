#include <Nexgen_Rover.h>

NXG_Rover rover = NXG_Rover(&Serial, true);
int num_times_to_repeat = 1;

void setup() {
    rover.init(true);
}

// The loop function runs over and over again until power down or reset
void loop() {
    if (num_times_to_repeat > 1) return;                  // When num_times_to_repeat reaches 2, immediately exit loop()
       
    int isTouched = digitalRead(rover.getTouchSensor());  // Read touch sensor. If touched, its value will be 1

    if (isTouched == 1) {                                 // When touched, drive forward, turn right and drive forward again
      rover.forward(90, 90, 1);
      rover.turnRight(90, 90, 1);
      rover.forward(90, 90, 1);
      
      num_times_to_repeat++;                              // Add 1 to num_times_to_repeat
    }
}


  /*************************************************************************************
    CODE EXPLANATION
    ----------------

    "rover" is the representation of the actual rover robot (represented in code).
    "rover.forward" tells the rover to move forward
    "rover.turnRight" tells the rover to turn right

    What are the 3 numbers inside the brackets?  (80, 80, 1)

    (LeftMotorSpeed, RightMotorSpeed, TimeInSeconds)

    So....

    LeftMotorSpeed = 90
    RightMotorSpeed = 90
    TimeInSeconds = 1

    Putting all this into English....., 
    The Rover travels forward for 1 second at 90% speed on both motors
    The Rover then turns right for 1 second at 90% speed on both motors
    The Rover finally travels forward for 1 second again at 90% speed on both motors

  **************************************************************************************/