#include <Nexgen_Rover.h>

#define DISTANCE_TO_STOP_AT 30
#define FORWARD_SPEED 80
#define BACKWARD_SPEED 70
#define TURN_SPEED 90

NXG_Rover rover = NXG_Rover(&Serial, true);

void setup() {
    rover.setServo(90);
    rover.init(true);
}

void loop() {
    if (rover.scanForward() > DISTANCE_TO_STOP_AT) {                // Check distance
      rover.forward(FORWARD_SPEED, FORWARD_SPEED);                  // Move forward
    }
    else {
      rover.stop();                                                 // Stop
      rover.backward(BACKWARD_SPEED, BACKWARD_SPEED, 0.5);          // Back up

      if (rover.scanLeft() > rover.scanRight()) {                   // Which is better? Left or Right?
        rover.turnLeft(TURN_SPEED, TURN_SPEED, 0.5);                // Turn left
      }
      else {
        rover.turnRight(TURN_SPEED, TURN_SPEED, 0.5);               // Turn right        
      }
    }
}