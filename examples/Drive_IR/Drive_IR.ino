#define DECODE_NEC

#define LEFT_BUTTON 0x00
#define MIDDLE_BUTTON 0x00
#define RIGHT_BUTTON 0x00

#include "PinDefinitionsAndMore.h"
#include <IRremote.hpp> // include the library
#include <Nexgen_Rover.h>

NXG_Rover rover = NXG_Rover(&Serial, true);

void setup() {
    rover.setBatteryType(NXG_Rover::ALKALINE_9V);
    rover.setVoltageReference(NXG_Rover::VREF_ARDUINO_NANO_V3);    rover.init(true);
    IrReceiver.begin(9, ENABLE_LED_FEEDBACK);
}

// The loop function runs over and over again until power down or reset
void loop() {
    if(IrReceiver.decode()){
      IrReceiver.resume();
      if (IrReceiver.decodedIRData.command == LEFT_BUTTON) {        
        rover.turnLeft(100, 100, 0.1);
      } 
      else if (IrReceiver.decodedIRData.command == MIDDLE_BUTTON) {
        rover.forward(100, 100, 1);
      } 
      else if (IrReceiver.decodedIRData.command == RIGHT_BUTTON) {
        rover.turnRight(100, 100, 0.1);
      } 
    }
}

/*** REPLACE THE DEFINES ABOVE WITH YOUR GROUP NUMBER ***/
/*** YOUR GROUP NUMBER REFERS TO THE ROW# ON YOUR REMOTE THAT YOU SHOULD USE TO CONTROL YOUR ROBOT ***/
/*** EG. GROUP 3 SHOULD USE ROW 3 ON THE REMOTE, buttons '-', '+', 'EQ' ***/

// Group 1
// #define LEFT_BUTTON 0x45
// #define MIDDLE_BUTTON 0x46
// #define RIGHT_BUTTON 0x47

// Group 2
// #define LEFT_BUTTON 0x44
// #define MIDDLE_BUTTON 0x40
// #define RIGHT_BUTTON 0x43

// Group 3
// #define LEFT_BUTTON 0x07
// #define MIDDLE_BUTTON 0x15
// #define RIGHT_BUTTON 0x09  

// Group 4
// #define LEFT_BUTTON 0x16
// #define MIDDLE_BUTTON 0x19
// #define RIGHT_BUTTON 0x0D

// Group 5
// #define LEFT_BUTTON 0x0C
// #define MIDDLE_BUTTON 0x18
// #define RIGHT_BUTTON 0x5E

// Group 6
// #define LEFT_BUTTON 0x08
// #define MIDDLE_BUTTON 0x1C
// #define RIGHT_BUTTON 0x5A

// Group 7
// #define LEFT_BUTTON 0x42
// #define MIDDLE_BUTTON 0x52
// #define RIGHT_BUTTON 0x4A
