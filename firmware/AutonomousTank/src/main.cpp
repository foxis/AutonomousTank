#include <Arduino.h>
#include <Servo.h>
#include <CmdBuffer.hpp>
#include <CmdCallback.hpp>
#include <CmdParser.hpp>

#define VERSION "1.0"

#define PIN_SERVOS_ENABLE A2
#define PIN_MOTORS_ENABLE A1
#define PIN_MA_PWM 5
#define PIN_MB_PWM 6
#define PIN_MA_A 4
#define PIN_MA_B 7
#define PIN_MB_A 8
#define PIN_MB_B 9

#define PIN_MA_ENCODER 2
#define PIN_MB_ENCODER 3

#define PIN_SERVO_0 10
#define PIN_SERVO_1 11
#define PIN_SERVO_2 12
#define PIN_SERVO_3 13

// =======================================================
Servo servo[4];
CmdCallback<3> cmdCallback;
CmdBuffer<32>  cmdBuffer;
CmdParser      cmdParser;
unsigned long last_now = 0;

// =======================================================
void motors_enable(bool);
void servos_enable(bool);

void handle_motor(CmdParser *myParser);
void handle_servo(CmdParser *myParser);
void handle_status(CmdParser *myParser);

// =======================================================
void setup() {
	Serial.begin(115200);
	Serial.print("Autonomous Robot Firmware Version ");
	Serial.println(VERSION);

	Serial.println("Initializing pins...");
	pinMode(PIN_SERVOS_ENABLE, OUTPUT);
	pinMode(PIN_MOTORS_ENABLE, OUTPUT);
	motors_enable(false);
	servos_enable(false);

	pinMode(PIN_MA_PWM, OUTPUT);
	pinMode(PIN_MB_PWM, OUTPUT);

	pinMode(PIN_MA_A, OUTPUT);
	pinMode(PIN_MA_B, OUTPUT);
	pinMode(PIN_MB_A, OUTPUT);
	pinMode(PIN_MB_B, OUTPUT);

	servo[0].attach(PIN_SERVO_0);
	servo[1].attach(PIN_SERVO_1);
	servo[2].attach(PIN_SERVO_2);
	servo[3].attach(PIN_SERVO_3);

	Serial.println("Setting up encoder...");
	// TODO

	Serial.println("Setting up lidar...");
  //TODO

	Serial.println("Setting up cli...");
	Serial.println("Commands:");
	Serial.println("  MOTOR {enable: 0/1} {left speed: 0..1} {right speed: 0..1} -> OK/ERR");
	cmdBuffer.add(PSTR("MOTOR"), &handle_motors);
	Serial.println("  SERVO {enable: 0/1} {0: 0..180} {1} {2} {3} -> OK/ERR");
	cmdBuffer.add(PSTR("SERVO"), &handle_servos);
	Serial.println("  STATUS -> {left speed} {right speed} {distance 0} .. {distance N}");
	cmdBuffer.add(PSTR("STATUS"), &handle_status);

	Serial.println("Done");
	Serial.println("");

	last_now = micros();
}

void loop() {
	unsigned long ms = micros();

	if (cmdBuffer.readFromSerial(&Serial, 10)) {
    Serial.println("Line have readed:");
    Serial.println(cmdBuffer.getStringFromBuffer());
		if (cmdParser.parseCmd(&myBuffer) != CMDPARSER_ERROR) {
			cmdCallback.processCmd(&cmdParser);
		}
		else {
			Serial.println("ERR");
		}
  }

	handle_motor_pid(now);

	last_now = now;
}

// =================================================================
void motors_enable(bool enable)
{
		digitalWrite(PIN_MOTORS_ENABLE, enable);
}

void motors_set(float left, float right)
{

}

void handle_motor_pid(unsigned long now)
{

}

void servos_enable(bool enable)
{
	digitalWrite(PIN_SERVOS_ENABLE, enable);
}

// ----------------------------------------------------------------
void handle_motor(CmdParser *myParser)
{
	Serial.println("Motor");
	bool enable = strcmp("1", myParser->getCmdParam(1)) == 0;
	float left = float(myParser->getCmdParam(2));
	float right = float(myParser->getCmdParam(3));

	Serial.println("OK");
}
void handle_servo(CmdParser *myParser)
{
	Serial.println("Servo");
}
void handle_status(CmdParser *myParser)
{
	Serial.println("Status");
}
