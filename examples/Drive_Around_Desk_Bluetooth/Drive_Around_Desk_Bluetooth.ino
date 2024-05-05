#include <Nexgen_Rover.h>

NXG_Rover rover = NXG_Rover(&Serial, true);

void setup() {
    // Set battery
    rover.setBatteryType(NXG_Rover::LIP0_2S);
    rover.setVoltageReference(NXG_Rover::VREF_ARDUINO_NANO_V3);
    
    rover.init(true);


}

void loop() {                                           /* The loop function runs over and over again until power down or reset */                                                       
  if(rover.getBluetooth().available()){                 /* Read bluetooth control signals */
    rover.readBluetooth();
    
    if(!rover.is_ARMED) return;  
    
    char cmd = rover.getCommand();
    int value = rover.getValue();
    
    if(cmd == rover.CMD_THROTTLE){
      if(value > rover.ZERO){
        rover.forward(value, value);                    /* Moves forward at speed of 'value' */
      }
      else {
        rover.backward(abs(value), abs(value));         /* Moves backward at speed of 'value' */
      }      
    }
    else if (cmd == rover.CMD_DIRECTION) {
      if(value < 0){
        rover.turnLeft(100);               
      }
      else if(value > 0) {
        rover.turnRight(100);          
      }
      else {
        rover.stop();
      }      
    }    
  }
}
