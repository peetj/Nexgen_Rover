/*
 Name:		Nexgen_Rover.h
 Created:	9/24/2023 3:35:45 PM
 Author:	Peter Januarius
 Editor:	http://www.visualmicro.com
*/

#ifndef _Nexgen_Rover_h
#define _Nexgen_Rover_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
#include "pitches.h"
#include <Servo.h>
#include <QTRSensors.h>
#include "Ultrasonic.h"

class NXG_Rover {

// ############## METHODS ###############
public:
    /* Constructor */
    NXG_Rover(bool debug = false);
    NXG_Rover(HardwareSerial* serial, bool debug = false);

    /* General Methods */
    void init(boolean playStartupSound);

    /* Test methods */
    long getRandomNumber();
    float getPi();

    /* Drive methods */
    void forward(int speedL, int speedR);
    void forward(int speed);
    void backward(int speedL, int speedR);
    void backward(int speed);
    void setSpeed(int sp);
    void setSpeed(int sp_left, int sp_right);
    void setSpeedLeft(int sp_left);
    void setSpeedRight(int sp_right);
    void setDirection(int dLeft, int dRight);
    void turnLeft(int sp);
    void turnRight(int sp);
    void turnLeft(int sp_left, int sp_right);
    void turnRight(int sp_left, int sp_right);
    void veerLeft(int sp_left, int sp_right);
    void veerRight(int sp_left, int sp_right);
    void stop();

    /* Buzzer methods */
    void playMelody(int melody[], int num_notes);
    void playTone(int freq);

    /* Servo and Ultrasonic Methods */
    // For now, I will leave them in the client

    /* Line Sensor Methods*/
    void initLineSensors();
    void calibrate();
    void sensorLog(uint16_t sensors[], boolean isOnLine, boolean isLeftOfLine, boolean isRightOfLine, int position, boolean doLog);

protected:
    //... <protected methods> ...

private:
    //... <private methods> ...
    


// ############## DATA ###############
public:
    //... <public data> ...
    int buzzerPin = 8;
    int touchSensorPin = 9;
    Servo servo;
    QTRSensors qtr;

protected:
    //... <protected data> ...

private:
    #define MOTOR_LEFT_PIN 3
    #define MOTOR_RIGHT_PIN 6
    #define DIR_LEFT_PIN 2
    #define DIR_RIGHT_PIN 5
    #define SERVO_PIN 4
    #define NUMBER_LINE_SENSORS 3

    bool _debug;
    HardwareSerial* _serial = nullptr;    
};


#endif

