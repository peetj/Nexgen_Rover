/*
 Name:		Nexgen_Rover.cpp
 Created:	9/24/2023 3:35:45 PM
 Author:	Peter Januarius
 Editor:	http://www.visualmicro.com
*/

#include "Nexgen_Rover.h"
#include <Servo.h>
#include <QTRSensors.h>
#include "Ultrasonic.h"

NXG_Rover::NXG_Rover(bool debug) {
	_debug = debug;
}

NXG_Rover::NXG_Rover(HardwareSerial* serial, bool debug) {
	_serial = serial;
	_debug = debug;

	_serial->println("Constructor...");
}

void NXG_Rover::init(boolean playStartupSound) {
	if(_serial) _serial->println("init()...");
	// Initialize random seed
	randomSeed(analogRead(0));

	// Setup pins
	pinMode(MOTOR_LEFT_PIN, OUTPUT);
	pinMode(MOTOR_RIGHT_PIN, OUTPUT);
	pinMode(DIR_LEFT_PIN, OUTPUT);
	pinMode(DIR_RIGHT_PIN, OUTPUT);

	servo.attach(SERVO_PIN);

	if (playStartupSound) {
		playTone(NOTE_C5);
	}
}

float NXG_Rover::getPi() {
	return 3.14;
}

long NXG_Rover::getRandomNumber() {
	/* generate secret number between 1 and 10: */
	int num = random(100);
	return num;
}

void NXG_Rover::forward(int speedL, int speedR) {
	if (_serial) _serial->println("forward...");
	
	setDirection(HIGH, HIGH);
	setSpeedLeft(speedL);
	setSpeedRight(speedR);
}

void NXG_Rover::forward(int speed) {
	if (_serial) _serial->println("forward...");
	setDirection(HIGH, HIGH);
	setSpeed(speed);
}

void NXG_Rover::backward(int speedL, int speedR) {
	if (_serial) _serial->println("backward...");

	setDirection(LOW, LOW);
	setSpeedLeft(speedL);
	setSpeedRight(speedR);
}

void NXG_Rover::backward(int speed) {
	if (_serial) _serial->println("backward...");

	setDirection(LOW, LOW);
	setSpeed(speed);
}

void NXG_Rover::setSpeed(int sp) {
	// set the motor speed to a value between 0 and 255
	analogWrite(MOTOR_LEFT_PIN, sp);
	analogWrite(MOTOR_RIGHT_PIN, sp);
}

void NXG_Rover::setSpeed(int sp_left, int sp_right) {
	// set the motor speed to a value between 0 and 255
	analogWrite(MOTOR_LEFT_PIN, sp_left);
	analogWrite(MOTOR_RIGHT_PIN, sp_right);
}

void NXG_Rover::setSpeedLeft(int sp_left) {
	// set the motor speed to a value between 0 and 255
	analogWrite(MOTOR_LEFT_PIN, sp_left);
}

void NXG_Rover::setSpeedRight(int sp_right) {
	// set the motor speed to a value between 0 and 255
	analogWrite(MOTOR_RIGHT_PIN, sp_right);
}

void NXG_Rover::setDirection(int dLeft, int dRight) {
	digitalWrite(DIR_LEFT_PIN, dLeft);
	digitalWrite(DIR_RIGHT_PIN, dRight);
}

void NXG_Rover::turnLeft(int sp) {
	if (_serial) _serial->println("turning left...");
	setDirection(LOW, HIGH);

	// Left motor much slower than right
	setSpeed(sp);
}

void NXG_Rover::turnRight(int sp) {
	if (_serial) _serial->println("turning right...");
	setDirection(HIGH, LOW);

	// Right motor much slower than right
	setSpeed(sp);
}

void NXG_Rover::turnLeft(int sp_left, int sp_right) {
	if (_serial) _serial->println("turning left...");
	setDirection(LOW, HIGH);

	// Left motor much slower than right
	setSpeed(sp_left, sp_right);
}

void NXG_Rover::turnRight(int sp_left, int sp_right) {
	if (_serial) _serial->println("turning right...");
	setDirection(HIGH, LOW);

	// Right motor much slower than right
	setSpeed(sp_left, sp_right);
}

void NXG_Rover::veerLeft(int sp_left, int sp_right) {
	if (_serial) _serial->println("veering left...");
	setDirection(LOW, HIGH);

	// Left motor much slower than right
	setSpeed(sp_left, sp_right);
	stop();
}

void NXG_Rover::veerRight(int sp_left, int sp_right) {
	if (_serial) _serial->println("veering right...");
	setDirection(HIGH, LOW);

	// Right motor much slower than left
	setSpeed(sp_left, sp_right);
	stop();
}

void NXG_Rover::stop() {
	delay(5);
	setSpeed(0);
}

void NXG_Rover::playMelody(int melody[], int num_notes) {

	for (int i = 0; i < num_notes; i++) {
		// to calculate the note duration, take one second divided by the note type.
		//e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
		int noteDuration = 1000 / melody[i + 1];
		tone(buzzerPin, melody[i], noteDuration);

		// to distinguish the notes, set a minimum time between them.
		// the note's duration + 30% seems to work well:
		int pauseBetweenNotes = noteDuration * 1.30;
		delay(pauseBetweenNotes);
		noTone(buzzerPin);
	}
}

void NXG_Rover::playTone(int freq) {
	tone(buzzerPin, freq);
	delay(1000); // Wait for 1 second
	noTone(buzzerPin);
}

void NXG_Rover::sensorLog(uint16_t sensors[], boolean isOnLine, boolean isLeftOfLine, boolean isRightOfLine, int position, boolean doLog) {
	if (doLog) {
		if (_serial) {
			_serial->print(sensors[0]);_serial->print(", ");_serial->print(sensors[1]);_serial->print(", ");_serial->println(sensors[2]);
			_serial->print(isOnLine);_serial->print(", ");_serial->print(isLeftOfLine);_serial->print(", ");_serial->println(isRightOfLine);
			_serial->print("Position: ");_serial->println(position);
		}
	}
}

void NXG_Rover::initLineSensors() {
	qtr.setTypeAnalog();
	qtr.setSensorPins((const uint8_t[]) { A1, A2, A3 }, NUMBER_LINE_SENSORS);
}

void NXG_Rover::calibrate() {

	playTone(2000);
	if (_serial) {
		_serial->println("###");
		_serial->println("### STARTED CALIBRATION ###");
		_serial->println("### CALIBRATION IS NECESSARY BEFORE USING readLineBlack/White OR IT WILL NOT RETURN CORRECT VALUES ###");
		_serial->println("###");
	}

	for (int i = 0; i < 250; i++) {
		qtr.calibrate();
		delay(20);
	}

	if (_serial) {
		_serial->println("###");
		_serial->println("### FINISHED CALIBRATION ###");
		_serial->println("###");
	}

	// Play a tone at 1000 Hz (1 kHz) for 1000 milliseconds (1 second)
	playTone(1000);
}
