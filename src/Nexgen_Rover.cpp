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

// Initialize the static member
NXG_Rover* NXG_Rover::rover = nullptr;

Servo servo;
Ultrasonic ultrasonic(ULTRASONIC_PIN);

// Initialize static constants
const float NXG_Rover::VREF_ARDUINO_NANO_V3 = 5.1;//4.68 when USB is plugged in;
const float NXG_Rover::VREF_ARDUINO_NANO_33 = 3.3;

ISR(TIMER1_COMPB_vect) {
	NXG_Rover::timerCallback();
}

NXG_Rover::NXG_Rover(bool debug) : _btSerial(RX, TX), _bar(LEDBAR_CLOCK_PIN, LEDBAR_DATA_PIN, 0) {
	rover = this;
	_debug = debug;
}

NXG_Rover::NXG_Rover(HardwareSerial* serial, bool debug) : _serial(serial), _debug(debug), _btSerial(RX, TX), _bar(LEDBAR_CLOCK_PIN, LEDBAR_DATA_PIN, 0) {	
	rover = this;
}

void NXG_Rover::init(boolean playSound) {
	if(_serial) _serial->begin(BAUD_RATE);
	if(_serial) _serial->println("init()...");

	// Initialize random seed
	randomSeed(analogRead(0));

	// Setup pins
	pinMode(MOTOR_LEFT_PIN, OUTPUT);
	pinMode(MOTOR_RIGHT_PIN, OUTPUT);
	pinMode(DIR_LEFT_PIN, OUTPUT);
	pinMode(DIR_RIGHT_PIN, OUTPUT);

	pinMode(A1, OUTPUT);
	pinMode(A2, OUTPUT);

	setSpeed(0);

	servo.attach(SERVO_PIN);	

	if (playSound) {
		playStartupSound();
	}

	if (!_servoInUse && !_turnOffBatteryCheck) {
		setupBatteryCheckTimer();
		_bar.begin();
	}

	FastLED.addLeds<WS2812B, DATA_PIN, GRB>(_leds, NUM_LEDS);

	_btSerial.begin(9600);
	if (_serial)_serial->println("Started Bluetooth...");

	delay(INITIAL_DELAY_IN_MICROSECONDS);
}


/*** TEST METHODS ***/
float NXG_Rover::getPi() {
	return 3.14;
}

long NXG_Rover::getRandomNumber() {
	/* generate secret number between 1 and 10: */
	int num = random(100);
	return num;
}
/*** END TEST METHODS ***/

/*** SETUP METHODS ***/
void NXG_Rover::setBatteryType(int type) {
	if (_serial)_serial->print("Setting battery type to: ");
	if (_serial)_serial->println(type);
	_batteryType = type;
}

void NXG_Rover::setVoltageReference(float vRef) {
	_voltageReference = vRef;
}

SoftwareSerial& NXG_Rover::getBluetooth() {
	return _btSerial;
}

void NXG_Rover::readBluetooth() {
	while (getBluetooth().available()) {
		char incomingChar = getBluetooth().read();
		processIncomingChar(incomingChar);
	}

	// Handle motor ration commands internally
	if (_command == CMD_MOTOR_RATIO) {
		_motorRatio = _value;
		if (_serial) _serial->print("_motorRatio: ");
		if (_serial) _serial->print(_motorRatio);
	}
}

char NXG_Rover::getCommand() {
	return _command;
}

int NXG_Rover::getValue() {
	return _value;
}

/** DANCE METHODS **/
int NXG_Rover::getBPM() {
	return _bpm;
}

void NXG_Rover::setBPM(int bpm) {
	_bpm = bpm * BPM_SCALING_FACTOR;
	_ms_per_beat = (float(60) / float(_bpm)) * 1000;
}

void NXG_Rover::lookLeft(float duration) {
	servo.write(180);
	delayFor(duration, true);	
}

void NXG_Rover::lookRight(float duration) {
	servo.write(0);
	delayFor(duration, true);	
}

void NXG_Rover::lookCenter(float duration) {
	servo.write(90);
	delayFor(duration, true);
}

void NXG_Rover::theLook(int numLooks, float duration, float scalingFactor) {
	int numDegrees = 180 / numLooks;
	for (int8_t i = 0; i < numLooks; i++) {
		servo.write(i * numDegrees);
		delayFor(duration * scalingFactor, true);
	}
}

void NXG_Rover::shuffleForward(int numberOfTimes, float scalingFactor) {
	for (int i = 0; i < numberOfTimes; i++) {
		forward(0.25f * scalingFactor, true);
		delayFor(0.75f * scalingFactor, true);
		forward(0.25f * scalingFactor, true);
		delayFor(0.75f * scalingFactor, true);
	}
}

void NXG_Rover::shuffleBackward(int numberOfTimes, float scalingFactor) {
	for (int i = 0; i < numberOfTimes; i++) {
		backward(0.25f * scalingFactor, true);
		delayFor(0.75f * scalingFactor, true);
		backward(0.25f * scalingFactor, true);
		delayFor(0.75f * scalingFactor, true);
	}
}

void NXG_Rover::head180(int numberOfTimes, float scalingFactor) {
	for (int i = 0; i < numberOfTimes; i++) {
		lookLeft(1 * scalingFactor);
		lookRight(1 * scalingFactor);
		lookCenter(1 * scalingFactor);
		delayFor(1, true);
	}
}

void NXG_Rover::sideToSide(int numberOfTimes, float scalingFactor) {
	for (int i = 0; i < numberOfTimes; i++) {         
		turnLeft(1, true, scalingFactor);
		turnRight(1, true, scalingFactor);
		turnRight(1, true, scalingFactor);
		turnLeft(1, true, scalingFactor);
	}
}

/** END DANCE METHODS **/

void NXG_Rover::processIncomingChar(char incomingChar) {
	if (incomingChar == CMD_ARMED) {
		is_ARMED = true;
		playArmedSound(NOTE_C5);
		ledsOn(CRGB::Green);
		if (_serial) _serial->println("Setting ARMED to true...");
		return;
	}
	else if (incomingChar == CMD_DISARMED) {
		is_ARMED = false;
		playDisarmedSound(NOTE_C4);
		ledsOff();
		return;
	}

	if (_inputIndex < MAX_INPUT - 1) {
		_inputCharArray[_inputIndex] = incomingChar;
		_inputIndex++;
	}

	if (incomingChar == '\n' || incomingChar == '\r') {
		_inputCharArray[_inputIndex] = '\0'; // Null terminate the string
		_command = _inputCharArray[0];
		if (_serial) _serial->print("_command = ");
		if (_serial) _serial->println(_command);

		for (int i = 1; i < strlen(_inputCharArray); i++) {
			_inputCharArray[i - 1] = _inputCharArray[i];
		}

		_value = atoi(_inputCharArray);
		if (_serial) _serial->print("_value = ");
		if (_serial) _serial->println(_value);
		// Reset index for next reading and clear array
		_inputIndex = 0;
		memset(_inputCharArray, 0, MAX_INPUT);
		return;
	}
}

void NXG_Rover::accelerate(int to_speed) {
	for (int i = 0; i < to_speed; i++) {
		analogWrite(MOTOR_LEFT_PIN, i);
		analogWrite(MOTOR_RIGHT_PIN, i);
		delay(10);
	}
}
/**
 * @brief Sets the speed of the left motor.
 *
 * This function controls the speed of the left motor.
 * The speed can be adjusted between -255 (full reverse) and 255 (full forward).
 *
 * @param speedL The speed of the left motor. Percentage value from 0-100
 * @param speedR The speed of the right motor. Percentage value from 0-100
 * @param s The number of seconds to move for
 */
void NXG_Rover::forward(int speedL, int speedR, float s, bool isDanceMode) {
	//if (_serial) _serial->println("forward...");
	
	setDirection(HIGH, HIGH);
	setSpeed(speedL, speedR);
	//ledsOn(CRGB::Green);
	delayFor(s, isDanceMode);

	if(s>0)
		stop();
}

void NXG_Rover::forward(int duration, bool isDanceMode) {
	forward(float(duration), isDanceMode);
}

void NXG_Rover::forward(float duration, bool isDanceMode) {
	forward(100, 100, duration, isDanceMode);
}

void NXG_Rover::backward(int speedL, int speedR, float s, bool isDanceMode) {
	//if (_serial) _serial->println("backward...");

	setDirection(LOW, LOW);
	setSpeed(speedL, speedR);
	//ledsOn(CRGB::Red);
	delayFor(s, isDanceMode);

	if(s>0)
		stop();	
}

void NXG_Rover::backward(int duration, bool isDanceMode) {
	//if (_serial) _serial->println("backward...");
	backward(float(duration), isDanceMode);
}

void NXG_Rover::backward(float duration, bool isDanceMode) {
	//if (_serial) _serial->println("backward...");
	backward(100, 100, duration, isDanceMode);
}

void NXG_Rover::setSpeed(int sp) {
	// Apply the motor ratio if applicable
	float ratioFactor;
	int sp_right;

	if (_motorRatio != 0) {
		// Calculate the ratio factor as a floating point value
		ratioFactor = abs(_motorRatio) / 10.0f;

		if (_motorRatio > 0)
			sp_right = static_cast<int>(sp * (1.0f + ratioFactor));
		else
			sp_right = static_cast<int>(sp * (1.0f - ratioFactor));
	}

	int mapped_speedL = min(sanitiseSpeed(sp), MAX_SPEED);
	int mapped_speedR = min(sanitiseSpeed(sp), MAX_SPEED);

	setSpeedLeft(mapped_speedL);
	setSpeedRight(mapped_speedR);
}

void NXG_Rover::setSpeed(int sp_left, int sp_right) {
	// Apply the motor ratio if applicable
	float ratioFactor;

	if (_motorRatio != 0) {
		// Calculate the ratio factor as a floating point value
		ratioFactor = abs(_motorRatio) / 10.0f;

		if (_motorRatio > 0)
			sp_right = static_cast<int>(sp_left * (1.0f + ratioFactor));
		else
			sp_right = static_cast<int>(sp_left * (1.0f - ratioFactor));
	}

	// Set the motor speed to a value between 0 and 100
	int mapped_speedL = min(sanitiseSpeed(sp_left), MAX_SPEED);
	int mapped_speedR = min(sanitiseSpeed(sp_right), MAX_SPEED);

	//setSpeedLeft(mapped_speedL);
	//setSpeedRight(mapped_speedR);
	setSpeedBoth(mapped_speedL, mapped_speedR);
}

void NXG_Rover::setSpeedLeft(int mapped_speedL) {
	//if (_serial) _serial->print("setting speedLeft...");
	//if (_serial) _serial->println(mapped_speedL);

	analogWrite(MOTOR_LEFT_PIN, mapped_speedL);
	currentSpeedLeft = mapped_speedL;
}

void NXG_Rover::setSpeedRight(int mapped_speedR) {
	//if (_serial) _serial->print("setting speedRight....");
	//if (_serial) _serial->println(mapped_speedR);

	analogWrite(MOTOR_RIGHT_PIN, mapped_speedR);
	currentSpeedRight = mapped_speedR;
}

void NXG_Rover::setSpeedBoth(int sp_left, int sp_right) {
	_serial->print("setting setSpeedBoth....");
	for (int i = 0; i < max(sp_left,sp_right); i+=2) {
		analogWrite(MOTOR_LEFT_PIN, min(i,sp_left));
		analogWrite(MOTOR_RIGHT_PIN, min(i,sp_right));
		delay(1);
	}

	currentSpeedRight = sp_right;
	currentSpeedLeft = sp_left;
}

void NXG_Rover::setDirection(int dLeft, int dRight) {
	digitalWrite(DIR_LEFT_PIN, dLeft);
	digitalWrite(DIR_RIGHT_PIN, dRight);
}

void NXG_Rover::turnLeft() {
	//if (_serial) _serial->println("turning left...");
	
	setDirection(LOW, HIGH);
	setSpeedLeft(currentSpeedLeft);
}

void NXG_Rover::turnRight() {
	//if (_serial) _serial->println("turning right...");

	setDirection(HIGH, LOW);
	setSpeedRight(currentSpeedRight);
}

void NXG_Rover::turnLeft(int sp) {
	//if (_serial) _serial->println("turning left...");
	setDirection(LOW, HIGH);

	// Turn left with passed in speed or current_speed
	setSpeed(sp);

	setLEDColor(CRGB::Blue, 1, 0);
}

void NXG_Rover::turnRight(int sp) {
	//if (_serial) _serial->println("turning right...");
	setDirection(HIGH, LOW);

	// Right motor much slower than right
	setSpeed(sp);

	setLEDColor(CRGB::Blue, 0, 1);
}

void NXG_Rover::turnLeft(int sp_left, int sp_right) {
	//if (_serial) _serial->println("turning left...");
	setDirection(LOW, HIGH);

	// Left motor much slower than right
	setSpeed(sp_left, sp_right);
	setLEDColor(CRGB::Blue, 1, 0);
}

void NXG_Rover::turnRight(int sp_left, int sp_right) {
	//if (_serial) _serial->println("turning right...");
	setDirection(HIGH, LOW);
	
	setSpeed(sp_left, sp_right);
	setLEDColor(CRGB::Blue, 0, 1);
}

void NXG_Rover::turnLeft(int sp_left, int sp_right, float s, bool isDanceMode) {
	//if (_serial) _serial->println("turning left...");
	setDirection(LOW, HIGH);

	setSpeed(sp_left, sp_right);
	setLEDColor(CRGB::Blue, 1, 0);

	delayFor(s, isDanceMode);

	if(s>0)
		stop();
}

void NXG_Rover::turnRight(int sp_left, int sp_right, float s, bool isDanceMode) {
	//if (_serial) _serial->println("turning right...");
	setDirection(HIGH, LOW);

	// Right motor much slower than right
	setSpeed(sp_left, sp_right);
	setLEDColor(CRGB::Blue, 0, 1);

	delayFor(s, isDanceMode);

	if(s>0)
		stop();
}

void NXG_Rover::turnLeft(int duration, bool isDanceMode = false, float scalingFactor = 1) {
	turnLeft(float(duration * scalingFactor), isDanceMode);
}

void NXG_Rover::turnLeft(float duration, bool isDanceMode = false, float scalingFactor = 1) {
	turnLeft(50, 50, duration * scalingFactor, isDanceMode);
}

void NXG_Rover::turnRight(int duration, bool isDanceMode = false, float scalingFactor = 1) {
	turnRight(float(duration * scalingFactor), isDanceMode);
}

void NXG_Rover::turnRight(float duration, bool isDanceMode = false, float scalingFactor = 1) {
	turnRight(50, 50, duration * scalingFactor, isDanceMode);
}

void NXG_Rover::veerLeft(int sp_left, int sp_right) {
	//if (_serial) _serial->println("veering left...");
	setDirection(LOW, HIGH);

	// Left motor much slower than right
	setSpeed(sp_left, sp_right);
	stop();
}

void NXG_Rover::veerRight(int sp_left, int sp_right) {
	//if (_serial) _serial->println("veering right...");
	setDirection(HIGH, LOW);

	// Right motor much slower than left
	setSpeed(sp_left, sp_right);
	stop();
}

void NXG_Rover::stop() {
	
	setSpeed(0);

	// Turn LEDs OFF
	//	ledsOn(CRGB::Black);
}

void NXG_Rover::delayFor(float s=0, bool isDanceMode=false) {
	//if (_serial) _serial->println(_ms_per_beat);
	isDanceMode ? delay(s*_ms_per_beat) : delay(s * 1000);
}

/* LED Bar Methods */
void NXG_Rover::setBatteryIndicator(float voltage) {
	//if (_serial) _serial->print("setBatteryIndicator... ");_serial->println(voltage);
	float max_battery_value = getMaxVoltage();
	float min_battery_value = getMinVoltage();
	float indicator_level = voltage - min_battery_value;
	int ledLevelAsPercentage = (indicator_level / (max_battery_value - min_battery_value)) * 100;
	if (rover){
		rover->_bar.setLevel(floor(ledLevelAsPercentage / 10));
	}
}

void NXG_Rover::turnOffBatteryCheck(bool turnOff) {
	if (turnOff) {
		_turnOffBatteryCheck = true;
	}
}

void NXG_Rover::setLEDIndicator(int level) {
	_bar.setLevel(level);
}

/* LED Methods */
void NXG_Rover::ledsOn(const CRGB& color, uint8_t leftOn=1, uint8_t rightOn=1) {
	if(leftOn) _leds[0] = color;
	if(rightOn) _leds[1] = color;
	FastLED.show();
	_leds_state_on = true;
}

void NXG_Rover::ledsOff() {
	_leds[0] = CRGB::Black;
	_leds[1] = CRGB::Black;
	bool _leds_state_on = false;
	FastLED.show();
	_leds_state_on = false;
}

void NXG_Rover::ledsWarning() {
	_leds[0] = CRGB::Red;
	_leds[1] = CRGB::Red;
	FastLED.show();
	_leds_state_on = true;
}

void NXG_Rover::metronome(int numBeats) {
	for (int i = 0; i < numBeats; i++) {
		if (i % 4 == 0) {
			flashLED_3(1000);
		}
		else {
			flashLED_2(500);
		}
	}
}

void NXG_Rover::flashLED_2(int toneFreq, float scalingFactor) {
	tone(BUZZER_PIN, toneFreq);
	digitalWrite(A1, HIGH);
	delayFor(0.2 * BPM_SCALING_FACTOR * scalingFactor, true);
	noTone(BUZZER_PIN);
	digitalWrite(A1, LOW);
	delayFor(0.8 * BPM_SCALING_FACTOR * scalingFactor, true);
}

void NXG_Rover::flashLED_3(int toneFreq, float scalingFactor) {
	tone(BUZZER_PIN, toneFreq);
	digitalWrite(A2, HIGH);
	delayFor(0.2 * BPM_SCALING_FACTOR * scalingFactor, true);
	noTone(BUZZER_PIN);
	digitalWrite(A2, LOW);
	delayFor(0.8 * BPM_SCALING_FACTOR * scalingFactor, true);
}

/* Buzzer Methods */
void NXG_Rover::playTone(int freq) {
	tone(BUZZER_PIN, freq);
	delay(1000); // Wait for 1 second
	noTone(BUZZER_PIN);
}

void NXG_Rover::playTone(int freq, float durationInSeconds) {
	tone(BUZZER_PIN, freq);
	delay(durationInSeconds * 1000); // Wait for duration seconds
	noTone(BUZZER_PIN);
}

void NXG_Rover::playStartupSound() {
	// Array of notes in the arpeggio (C major)
	const int notes[] = { 262, 330, 392, 523 }; // Frequencies for C4, E4, G4, C5
	const int noteDuration = 150; // Duration of each note in milliseconds
	const int numNotes = sizeof(notes) / sizeof(notes[0]); // Number of notes

	for (int i = 0; i < numNotes; i++) {
		tone(BUZZER_PIN, notes[i]);
		delay(noteDuration);
		noTone(BUZZER_PIN);
		delay(50);
	}
}

void NXG_Rover::playArmedSound(int freq) {
	// Array of notes in the arpeggio (C major)
	const int notes[] = { freq, freq }; // Frequency for C5
	const int noteDuration = 75; // Duration of each note in milliseconds
	const int numNotes = sizeof(notes) / sizeof(notes[0]); // Number of notes

	for (int i = 0; i < numNotes; i++) {
		tone(BUZZER_PIN, notes[i]);
		delay(noteDuration);
		noTone(BUZZER_PIN);
		delay(50);
	}
}

void NXG_Rover::playDisarmedSound(int freq) {
	// Array of notes in the arpeggio (C major)
	const int notes[] = { freq, freq, freq }; // Frequency for C5
	const int noteDuration = 75; // Duration of each note in milliseconds
	const int numNotes = sizeof(notes) / sizeof(notes[0]); // Number of notes

	for (int i = 0; i < numNotes; i++) {
		tone(BUZZER_PIN, notes[i]);
		delay(noteDuration);
		noTone(BUZZER_PIN);
		delay(50);
	}
}

void NXG_Rover::playLowBatteryWarning(int freq) {
	for (uint8_t i = 0; i < 3; i++) {
		tone(BUZZER_PIN, freq);
		delay(250);
		noTone(BUZZER_PIN);
	}

	delay(750);
	_isRoverEnabled = false;
}

void NXG_Rover::playMelody(int melody[], int num_notes) {

	for (int i = 0; i < num_notes; i++) {
		// to calculate the note duration, take one second divided by the note type.
		//e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
		int noteDuration = 1000 / melody[i + 1];
		tone(BUZZER_PIN, melody[i], noteDuration);

		// to distinguish the notes, set a minimum time between them.
		// the note's duration + 30% seems to work well:
		int pauseBetweenNotes = noteDuration * 1.30;
		delay(pauseBetweenNotes);
		noTone(BUZZER_PIN);
	}
}

/* Touch Sensor Methods */
int NXG_Rover::getTouchSensor() {
	return TOUCH_PIN;
}

/* Infrared Sensor Methods */
int NXG_Rover::getInfraredSensor() {
	return INFRARED_PIN;
}

/* Servo and Ultrasonic Methods */
void NXG_Rover::setServo(int degrees) {
	_servoInUse = true;
	servo.write(degrees);
}

long NXG_Rover::getDistance() {
	long rangeInCentimeters;

	rangeInCentimeters = ultrasonic.MeasureInCentimeters(); // two measurements should keep an interval
	Serial.print(rangeInCentimeters);//0~400cm
	Serial.println(" cm");
	return rangeInCentimeters;
}

long NXG_Rover::scanForward() {
	servo.write(90);
	long distance = getDistance();
	if (_serial) _serial->print("Distance forward: ");
	if (_serial) _serial->println(distance);
	return distance;
}

long NXG_Rover::scanLeft() {
	servo.write(165);
	delay(2000);
	long distance = getDistance();
	if (_serial) _serial->print("Distance left: ");
	if (_serial) _serial->println(distance);
	scanForward();
	return distance;
}

long NXG_Rover::scanRight() {
	servo.write(15);
	delay(2000);
	long distance = getDistance();
	if (_serial) _serial->print("Distance right: ");
	if (_serial) _serial->println(distance);
	scanForward();
	return distance;
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

// Private Methods
int NXG_Rover::sanitiseSpeed(int sp) {
	// Ensures that speed is mapped to a percentage (0-100) and that it does not exceed MAX_SPEED	
	long mapped_speed = map(sp, 0, 100, 0, 255);
	return min(mapped_speed, MAX_SPEED);
}

float NXG_Rover::getMaxVoltage() {
	float max_voltage;

	switch (_batteryType) {
	case 1:
		max_voltage = LIPO_2S_MAX_VOLTAGE;
		break;
	case 2:
		max_voltage = LIPO_3S_MAX_VOLTAGE;
		break;
	case 3:
		max_voltage = ALKALINE_9V_MAX_VOLTAGE;
		break;
	}
	return max_voltage;
}

float NXG_Rover::getMinVoltage() {
	float min_voltage = 0;
	//if (_serial) _serial->println(_batteryType);
	switch (_batteryType) {
	case 1:
		min_voltage = LIPO_2S_MIN_VOLTAGE;
		break;
	case 2:
		min_voltage = LIPO_3S_MIN_VOLTAGE;
		break;
	case 3:
		min_voltage = ALKALINE_9V_MIN_VOLTAGE;
		break;
	}

	if (min_voltage == 0) {
		Serial.println("### ERROR: No battery selected");
	}

	return min_voltage;
}

void NXG_Rover::setupBatteryCheckTimer() {
	_serial->println("Setting up battery check timer...");
	// Clear Timer on Compare Match (CTC) mode for Timer1
	TCCR1A = 0;
	TCCR1B = 0;
	TCCR1B |= (1 << WGM12); // CTC mode
	TCCR1B |= (1 << CS12) | (1 << CS10); // Prescaler 1024

	// Set OCR1B value for desired interval, e.g., 1 second interval
	OCR1B = 15624; // As used before for OCR1A

	// Enable Timer1 compare interrupt B
	TIMSK1 |= (1 << OCIE1B);

	// Enable global interrupts
	sei();
}

static float NXG_Rover::getBatteryVoltage(int batteryType) {
	//if (_serial) {_serial->println("getBatteryVoltage()...");}
	float analog_proportional_resolution_reading = analogRead(BATTERY_PIN);
	Serial.println(analog_proportional_resolution_reading);
	Serial.println("********************");

	/* We divide the reference voltage into 1024 equal parts and then multiply
	it by the number taken from analog_proportional_voltage_reading, giving
	us the voltage between A0 and the voltage divider mid-point. */
	float proportionalVoltage = analog_proportional_resolution_reading * VREF_ARDUINO_NANO_V3/ 1024.0;

	/* We then get the total voltage of the battery by the following equation. This is a
	rearrangement of Vin x Rdown/Rup+RDown - the proportion across Rdown */
	float totalBatteryVoltage = proportionalVoltage * ((RESISTOR_UP / RESISTOR_DOWN) + 1);

	/*	Using a 2S LiPo we need a calibration factor because the values we are getting are different to the actual measured values
		VREF = 5.1v
		A0 = 2.045v but the analogRead gives us 475 (which is the difference)
		Thus we will introduce a calibration factor for the LiPo only
	*/
	float calibrationFactor = batteryType == 1 ? 0.875 : 1;
	
	return totalBatteryVoltage * calibrationFactor;
}

void NXG_Rover::handleBatteryMonitoring() {
	// Set up a timer or interrupt that triggers `timerCallback` every second
	// Depending on the platform, this might use hardware timers or a software timer library
}

void NXG_Rover::timerCallback() {
	// This method is called by the timer interrupt
	// Here, perform the battery check and update any indicators or alarms
	if (rover) {
		float battery_voltage = rover->getBatteryVoltage(rover->_batteryType);  // Assume getBatteryVoltage() is a method that fetches the current battery voltage

		if (battery_voltage < rover->getMinVoltage()) {
			rover->playLowBatteryWarning(NOTE_G5);
			rover->ledsWarning();
			rover->_isRoverEnabled = false;
		}

		rover->setBatteryIndicator(battery_voltage);         // Assume setBatteryIndicator(float) updates the battery level display
	}
}

void NXG_Rover::setLEDColor(const CRGB& color, uint8_t leftOn = 0, uint8_t rightOn = 0) {
	ledsOn(color, leftOn, rightOn);
}

// Method to print multiple strings in one line
void NXG_Rover::printMultiple(const char* first, ...) {
	va_list args;
	va_start(args, first); // Initialize the argument list

	Serial.print(first); // Print the first argument

	const char* next;
	while ((next = va_arg(args, const char*)) != nullptr) {
		Serial.print(next);
	}

	va_end(args); // Clean up the argument list
	Serial.println(); // Move to the next line
}
