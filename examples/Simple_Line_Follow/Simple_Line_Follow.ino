/*
 Name:		Rover_Examples_Simple_Line_Follow.ino
 Created:	9/25/2023 5:59:56 PM
 Author:	Peter Januarius
*/

#include <Nexgen_Rover.h>
#include <QTRSensors.h>

#define DEFAULT_SPEED_LEFT 160
#define DEFAULT_SPEED_RIGHT 160
#define OPTIMUM_L_MOTOR_LEFT_TURN_SPEED 110
#define OPTIMUM_R_MOTOR_LEFT_TURN_SPEED 255
#define OPTIMUM_L_MOTOR_RIGHT_TURN_SPEED 255
#define OPTIMUM_R_MOTOR_RIGHT_TURN_SPEED 110

NXG_Rover nxg = NXG_Rover(&Serial, false);

void setup() {
    Serial.begin(9600);
    
    // Initialize the sensors & calibrate the line sensors
    nxg.init(true);
    nxg.initLineSensors();
    nxg.calibrate();
}

void loop() {
    uint16_t sensors[3];

    // Get calibrated sensor values and return position. 1000=CENTRE
    int position = nxg.qtr.readLineBlack(sensors);

    // Categorise the position
    boolean isOnLine = position > 600 && position < 1400;
    boolean isLeftOfLine = position >= 0 && position <= 600;
    boolean isRightOfLine = position >= 1400 && position <= 2000;

    nxg.sensorLog(sensors, isOnLine, isLeftOfLine, isRightOfLine, position, false);

    if (isOnLine) {
        nxg.forward(DEFAULT_SPEED_LEFT, DEFAULT_SPEED_RIGHT);
    }
    else if (isLeftOfLine) {
        nxg.turnRight(OPTIMUM_L_MOTOR_RIGHT_TURN_SPEED, OPTIMUM_R_MOTOR_RIGHT_TURN_SPEED);
        delay(100);
        nxg.stop();
    }
    else if (isRightOfLine) {
        nxg.turnLeft(OPTIMUM_L_MOTOR_LEFT_TURN_SPEED, OPTIMUM_R_MOTOR_LEFT_TURN_SPEED);
        delay(100);
        nxg.stop();
    }
    else {
        nxg.stop();
    }
}
























