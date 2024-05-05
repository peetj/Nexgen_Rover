#include <Nexgen_Rover.h>

NXG_Rover rover = NXG_Rover(&Serial, true);

void setup() {

    rover.setBatteryType(NXG_Rover::ALKALINE_9V);
    rover.setVoltageReference(NXG_Rover::VREF_ARDUINO_NANO_V3);
    rover.init(true);
}

// The loop function runs over and over again until power down or reset
void loop() {
    // No code !!
}