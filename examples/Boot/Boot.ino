#include <Nexgen_Rover.h>

NXG_Rover rover = NXG_Rover(&Serial, true);

void setup() {

    rover.setBatteryType(NXG_Rover::LIP0_2S);
    rover.setVoltageReference(NXG_Rover::VREF_ARDUINO_NANO_V3);

    rover.setServo(90);    
    rover.init(true);
}

// The loop function runs over and over again until power down or reset
void loop() {
    // No code !!
    if(digitalRead(rover.getTouchSensor()) == 1){
      if (rover.scanLeft() > rover.scanRight()) {                   // Which is better? Left or Right?
        Serial.println("Scanning...");
      }

      delay(2000);
    }
}