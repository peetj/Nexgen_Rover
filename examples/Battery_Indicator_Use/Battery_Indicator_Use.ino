/*
Rover Battery Indicator - How to use Example
Just include the 3 lines in the setup() function if you want to use the LED bar
to indicate battery level. Most useful for LiPo batteries.
*/

#include <Nexgen_Rover.h>

NXG_Rover rover = NXG_Rover(&Serial, true);

void setup() {
    // Initialize Rover
    rover.setBatteryType(NXG_Rover::ALKALINE_9V);
    rover.setVoltageReference(NXG_Rover::VREF_ARDUINO_NANO_V3);
    rover.setUseBatteryIndicator(true);

    rover.init(false);
}

void loop() {
    // NO CODE HERE !!!
}
