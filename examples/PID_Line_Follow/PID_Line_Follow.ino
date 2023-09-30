/*
 Name:		Rover_Examples_PID_Line_Follow.ino
 Created:	9/29/2023 3:50:21 PM
 Author:	Peter Januarius
*/

#include <Nexgen_Rover_v4.h>
#include <QTRSensors.h>

#define SETPOINT    1000    // The goal for readLineBlack (center)
#define KP          0.8     // The P value in PID
#define KI          0.001   // The I value in PID
#define KD          0.9     // The D value in PID
#define MAX_SPEED   125     // The max speed to set motors to
#define SET_SPEED   125     // The goal speed to set motors to
#define MIN_SPEED   0       // The min speed to set motors to
#define NUM_SENSORS 3       // The number of QTR sensors

// PID **************************************
int lastError = 0;  // For storing PID error
float integral = 0;

NXG_Rover nxg = NXG_Rover(&Serial, false);

void setup() {
    Serial.begin(9600);

    // Initialize the sensors & calibrate the line sensors
    nxg.init(true);
    nxg.initLineSensors();
    nxg.calibrate();

    nxg.forward(SET_SPEED, SET_SPEED);
}

void loop() {
    // Take a reading
    uint16_t sensors[NUM_SENSORS];
    unsigned int linePos = nxg.qtr.readLineBlack(sensors);

    // Uncomment to log line position and sensor values
    //log(sensors, linePos, true);

    // Compute the error
    int error = SETPOINT - linePos;
    integral += KI * error;
    // Compute the motor adjustment
    int adjust = error * KP + integral + KD * (error - lastError);

    // Adjust motors, one negatively and one positively
    nxg.setSpeed(constrain(SET_SPEED + adjust, MIN_SPEED, MAX_SPEED), constrain(SET_SPEED - adjust, MIN_SPEED, MAX_SPEED));

    // Record the current error for the next iteration
    lastError = error;
}

void log(unsigned int sensors[], int position, boolean doLog) {
    if (doLog) {
        Serial.print(sensors[0]);Serial.print(", ");Serial.print(sensors[1]);Serial.print(", ");Serial.println(sensors[2]);
        Serial.print("Position: ");Serial.println(position);
        Serial.print("----------------------------------------------------------");
    }
}


























