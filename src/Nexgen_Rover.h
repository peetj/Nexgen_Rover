/*
 Name:		Nexgen_Rover.h
 Created:	9/24/2023 3:35:45 PM
 Author:	Peter Januarius
 Editor:	http://www.visualmicro.com
*/

#ifndef _Nexgen_Rover_h
#define _Nexgen_Rover_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif
#include <avr/io.h>
#include <avr/interrupt.h>
#include "pitches.h"

#include <QTRSensors.h>
#include "Ultrasonic.h"
#include <SoftwareSerial.h>
#include <Grove_LED_Bar.h>
#include <FastLED.h>
#include <Servo.h>


class NXG_Rover {

// ############## METHODS ###############
public:
    static NXG_Rover* rover;

    static const float VREF_ARDUINO_NANO_V3; // Voltage reference for Arduino Nano V3
    static const float VREF_ARDUINO_NANO_33; // Voltage reference for Arduino Nano 33 IoT
    static const int LIP0_2S = 1;
    static const int LIP0_3S = 2;
    static const int ALKALINE_9V = 3;
    static const int ROVER_BATTERY_TYPE;

    /* Constructor */
    NXG_Rover(bool debug = false);
    NXG_Rover(HardwareSerial* serial, bool debug = false);

    /* Static Methods */
    static void timerCallback();     // Static callback for the timer

    /* General Methods */
    void init(boolean playStartupSound);

    /* Test methods */
    long getRandomNumber();
    float getPi();

    /* Setup methods */
    void setBatteryType(int type);
    void setVoltageReference(float vRef);
    static float getBatteryVoltage(int batteryType);

    /* Bluetooth Methods */
    SoftwareSerial& getBluetooth();
    void readBluetooth();
    void processIncomingChar(char incomingChar);
    char getCommand();
    int getValue();    

    /* Drive methods */
    void accelerate(int to_speed = 255);
    void forward(int speedL, int speedR, float s=0, bool isDanceMode=false);
    void forward(int duration, bool isDanceMode=false);
    void forward(float duration, bool isDanceMode = false);
    void backward(int speedL, int speedR, float s=0, bool isDanceMode = false);
    void backward(int duration, bool isDanceMode=false);
    void backward(float duration, bool isDanceMode = false);
    void setSpeed(int sp);
    void setSpeed(int sp_left, int sp_right);
    void setSpeedLeft(int sp_left);
    void setSpeedRight(int sp_right);
    void setSpeedBoth(int sp_left, int sp_right);
    void setDirection(int dLeft, int dRight);
    void turnLeft();
    void turnLeft(int sp);
    void turnLeft(int sp_left, int sp_right);
    void turnLeft(int sp_left, int sp_right, float s, bool isDanceMode = false);
    void turnLeft(int duration, bool isDanceMode = false, float scalingFactor=1);
    void turnLeft(float duration, bool isDanceMode = false, float scalingFactor = 1);
    void turnRight();
    void turnRight(int sp);
    void turnRight(int sp_left, int sp_right);
    void turnRight(int sp_left, int sp_right, float s, bool isDanceMode = false);
    void turnRight(int duration, bool isDanceMode = false, float scalingFactor = 1);
    void turnRight(float duration, bool isDanceMode = false, float scalingFactor = 1);
    void veerLeft(int sp_left, int sp_right);
    void veerRight(int sp_left, int sp_right);
    void stop();
    void delayFor(float s = 0, bool isDanceMode = false);

    /* Battery methods */
    void setBatteryIndicator(float voltage);
    void turnOffBatteryCheck(bool turnOff);
    void setLEDIndicator(int level);

    /* LED Methods */
    void ledsOn(const CRGB& color, uint8_t leftOn=1, uint8_t rightOn=1);
    void ledsOff();
    void ledsWarning();
    void flashLED_2(int toneFreq, float scalingFactor = 1);
    void flashLED_3(int toneFreq, float scalingFactor = 1);

    /* Buzzer methods */
    void playTone(int freq);
    void playTone(int freq, float durationInSeconds);
    void playMelody(int melody[], int num_notes);
    void playStartupSound();
    void playLowBatteryWarning(int freq);
    void playArmedSound(int freq);
    void playDisarmedSound(int freq);

    /* Touch sensor methods */
    int getTouchSensor();

    /* Infrared remote methods */
    int getInfraredSensor();
    long scanForward();
    long scanLeft();
    long scanRight();

    /* Servo and Ultrasonic Methods */
    void setServo(int degrees);
    long getDistance();

    /* Line Sensor Methods*/
    void initLineSensors();
    void calibrate();
    void sensorLog(uint16_t sensors[], boolean isOnLine, boolean isLeftOfLine, boolean isRightOfLine, int position, boolean doLog);

    /* Dance Methods */
    int getBPM();
    void setBPM(int bpm);
    void NXG_Rover::metronome(int numBeats);
    void lookLeft(float duration=0);
    void lookRight(float duration=0);
    void lookCenter(float duration=0);
    void theLook(int numLooks, float duration=0, float scalingFactor=1);
    void shuffleForward(int numberOfTimes, float scalingFactor = 1);
    void shuffleBackward(int numberOfTimes, float scalingFactor = 1);
    void head180(int numberOfTimes, float scalingFactor = 1);
    void NXG_Rover::sideToSide(int numberOfTimes, float scalingFactor);

    /* Utility Methods */
    void printMultiple(const char* first, ...);

protected:
    //... <protected methods> ...

private:
    //... <private methods> ...
    int sanitiseSpeed(int sp); 
    float getMinVoltage();
    float getMaxVoltage();
    void setupBatteryCheckTimer();
    void handleBatteryMonitoring();  // Handles automatic battery monitoring setup
    void setLEDColor(const CRGB& color, uint8_t leftOn, uint8_t rightOn);
    
// ############## DATA ###############
public:
    //... <public data> ...        
    const int ZERO = 0;
    const int SERVO_90_DEGREES = 90;
    const char* CMD_THROTTLE = 'T';
    const char* CMD_DIRECTION = 'D';
    const char* CMD_ARMED = 'A';
    const char* CMD_DISARMED = 'Z';
    const char* CMD_MOTOR_RATIO = 'M';
    bool is_ARMED = false;
    int currentSpeedLeft = 0;
    int currentSpeedRight = 0;
    Servo servo;
    QTRSensors qtr;    

protected:
    //... <protected data> ...
    static volatile bool __timeToUpdateBatteryStatus;

private:
    #define MOTOR_LEFT_PIN 3
    #define MOTOR_RIGHT_PIN 5
    #define DIR_LEFT_PIN 2
    #define DIR_RIGHT_PIN 4
    #define SERVO_PIN 6
    #define ULTRASONIC_PIN 19
    #define BUZZER_PIN 10
    #define TOUCH_PIN 7
    #define RX 11
    #define TX 13
    #define NUMBER_LINE_SENSORS 3
    #define MAX_SPEED 255
    #define BRAKING_RATE 10
    #define MAX_INPUT 10
    #define BATTERY_PIN A0
    #define LEDBAR_CLOCK_PIN A4
    #define LEDBAR_DATA_PIN A5
    #define INFRARED_PIN 8
    #define RESISTOR_UP 10000
    #define RESISTOR_DOWN 3300
    #define LIPO_2S_MIN_VOLTAGE 6.4
    #define LIPO_3S_MIN_VOLTAGE 10.5
    #define ALKALINE_9V_MIN_VOLTAGE 7.0
    #define LIPO_2S_MAX_VOLTAGE 8.4
    #define LIPO_3S_MAX_VOLTAGE 12.6
    #define ALKALINE_9V_MAX_VOLTAGE 9.0
    #define NUM_LEDS 2
    #define DATA_PIN 12
    #define INITIAL_DELAY_IN_MICROSECONDS 1000
    #define BAUD_RATE 9600
    #define BPM_SCALING_FACTOR 1.15
    
    bool _debug = true;    
    bool _isRoverEnabled = false;
    HardwareSerial* _serial = nullptr;    
    SoftwareSerial _btSerial;
    char _inputCharArray[MAX_INPUT];
    char _command = '\0';
    int _value = 0;
    int _inputIndex = 0;
    int _motorRatio = 0;
    int _minVoltage = 0;
    int _batteryType = -1;
    float _voltageReference = 0;
    volatile bool _timeToUpdateBatteryStatus = false;
    Grove_LED_Bar _bar;
    CRGB _leds[NUM_LEDS];
    bool _blink_on = false;
    bool _leds_state_on = false;
    bool _servoInUse = false;
    bool _turnOffBatteryCheck = false;
    int _bpm = 0;
    int _ms_per_beat = 0;
};


#endif

