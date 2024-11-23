/*
Rover Battery Indicator Example

*/

#include <Nexgen_Rover.h>

NXG_Rover rover = NXG_Rover(&Serial, true);

void setup() {
    // Initialize Rover
    rover.setBatteryType(NXG_Rover::ALKALINE_9V);
    rover.setVoltageReference(NXG_Rover::VREF_ARDUINO_NANO_V3);

    rover.init(false);
}

void loop() {

    float batteryVoltage = rover.getBatteryVoltage(0);
    rover.setBatteryIndicator(batteryVoltage);

    delay(50);
}