/*
 Name:		Rover_Examples_Obstacle_Avoidance.ino
 Created:	9/25/2023 5:59:56 PM
 Author:	Peter Januarius
*/

#include <Ultrasonic.h>
#include <Nexgen_Rover_v4.h>
#include "pitches.h"

#define DISTANCE_TO_STOP_AT 40
#define ULTRASONIC_PIN 7
#define FORWARD_SPEED 90
#define BACKWARD_SPEED 90
#define TURN_SPEED 150
#define SERVO_90_DEGREE_POSITION 90

NXG_Rover nxg = NXG_Rover(&Serial, true);
Ultrasonic ultrasonic(ULTRASONIC_PIN);
boolean isMoving = false;

void setup() {

    Serial.begin(9600);
    nxg.init(true);

    // Delay start for 2 seconds
    delay(2000);

    nxg.servo.write(SERVO_90_DEGREE_POSITION);
    nxg.playTone(262);
    delay(1000);

    Serial.println("Rover v4 setup complete...");
}

// The loop function runs over and over again until power down or reset
void loop() {
    // Point 'eyes' forward and check distance
    if (scanForward() > DISTANCE_TO_STOP_AT) {
        if (!isMoving) {
            Serial.println("moving forward");
            // Move forward if not already moving and set moving to TRUE
            nxg.forward(FORWARD_SPEED, FORWARD_SPEED);
            isMoving = true;
        }
    }
    else {
        // distance is less then 30cm so brake, backup and turn left OR right
        nxg.stop();
        delay(500);
        isMoving = false;
        nxg.backward(BACKWARD_SPEED);
        delay(500);
        nxg.stop();

        int leftDistance = scanLeft();
        int rightDistance = scanRight();

        // Turn whichever direction has a greater distance to travel
        if (leftDistance > rightDistance) {
            Serial.println("turning left");
            nxg.turnLeft(TURN_SPEED);
            delay(400);
            nxg.stop();
        }
        else {
            Serial.println("turning right");
            nxg.turnRight(TURN_SPEED);
            delay(400);
            nxg.stop();
        }
    }
    delay(500);
}

long getDistance() {
    long rangeInCentimeters;

    rangeInCentimeters = ultrasonic.MeasureInCentimeters(); // two measurements should keep an interval
    Serial.print(rangeInCentimeters);//0~400cm
    Serial.println(" cm");
    return rangeInCentimeters;
}

long scanForward() {
    nxg.servo.write(90);
    return getDistance();
}

long scanLeft() {
    nxg.servo.write(180);
    Serial.print("left distance: ");
    delay(2000);
    long distance = getDistance();
    scanForward();
    return distance;
}

long scanRight() {
    nxg.servo.write(0);
    Serial.print("right distance: ");
    delay(2000);
    long distance = getDistance();
    scanForward();
    return distance;
}