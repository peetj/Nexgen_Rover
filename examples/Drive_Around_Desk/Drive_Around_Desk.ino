#include <Nexgen_Rover.h>

NXG_Rover rover = NXG_Rover(&Serial, true);

bool hasStopped = false;

void setup() {
    rover.setBatteryType(NXG_Rover::ALKALINE_9V);
    rover.setVoltageReference(NXG_Rover::VREF_ARDUINO_NANO_V3);
    
    rover.init(false);
    
    /*** IF YOU WANT TO PLAY YOUR STARTUP SOUND, COPY IT FROM YOUR PREVIOUS SKETCH/CODE ***/

    delay(2000);
}

// The loop function runs over and over again until power down or reset
void loop() {
  if (hasStopped) return;   // EXIT if the rover has travelled around the desk

    // FORWARD
    rover.forward(90, 90, 1); 
    
    // TURN RIGHT
    rover.turnRight(80, 80, 0.5);
    
    // FORWARD
    
    // TURN RIGHT
    
    // FORWARD
    
    // TURN RIGHT
    
    // FORWARD

    hasStopped = true;
}
