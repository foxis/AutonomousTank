#include <Arduino.h>
#include <Servo.h>
#include <CmdBuffer.hpp>
#include <CmdCallback.hpp>
#include <CmdParser.hpp>

#include "effectors/HBridge_TB724.h"
#include "sensors/SWFrequencyCounter.h"
#include "PID/PID.h"

#define VERSION "1.0"
#define SUPPORTS "motors,servos"

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
CmdCallback_P<5> cmdCallback;
CmdBuffer<128>  cmdBuffer;
CmdParser      cmdParser;
unsigned long last_now = 0;
unsigned long pid_delta = 0;

Locomotion::SoftwareFrequencyCounter encoder_a(100);
Locomotion::SoftwareFrequencyCounter encoder_b(100);
Locomotion::HBridge_TB724 motors(PIN_MA_A, PIN_MA_B, PIN_MA_PWM,
						            				 PIN_MB_A, PIN_MB_B, PIN_MB_PWM,
										 					 	 PIN_MOTORS_ENABLE, 255);

Locomotion::real_t pid_a_in, pid_a_out, pid_a_set, pid_a_acc = 0;
Locomotion::real_t pid_b_in, pid_b_out, pid_b_set, pid_b_acc = 0;
Locomotion::PID pid_a(&pid_a_in, &pid_a_out, &pid_a_set, 0.0005, 0.0002, 0.00007, DIRECT);
Locomotion::PID pid_b(&pid_b_in, &pid_b_out, &pid_b_set, 0.0005, 0.0002, 0.00007, DIRECT);


// =======================================================
void motors_enable(bool);
void servos_enable(bool);

void handle_hello(CmdParser *myParser);
void handle_motors(CmdParser *myParser);
void handle_servos(CmdParser *myParser);
void handle_status(CmdParser *myParser);
void handle_debug(CmdParser *myParser);
void handle_encoder_a();
void handle_encoder_b();
void handle_motor_pid(unsigned long now);

// =======================================================
void setup() {
	Serial.begin(115200);
	handle_hello(NULL);

	Serial.println("Initializing pins...");
	pinMode(PIN_SERVOS_ENABLE, OUTPUT);
	servos_enable(false);

	motors.begin();

	servo[0].attach(PIN_SERVO_0);
	servo[1].attach(PIN_SERVO_1);
	servo[2].attach(PIN_SERVO_2);
	servo[3].attach(PIN_SERVO_3);

	Serial.println("Setting up speed controllers...");
	pid_a.SetOutputLimits(-1.0, 1.0);
	pid_a.SetSampleTime(10000);
	pid_a.SetMode(AUTOMATIC);
	pid_b.SetOutputLimits(-1.0, 1.0);
	pid_b.SetSampleTime(10000);
	pid_b.SetMode(AUTOMATIC);

	Serial.println("Setting up encoder...");
	attachInterrupt(0, handle_encoder_a, RISING);
  attachInterrupt(1, handle_encoder_b, RISING);
		// TODO

	Serial.println("Setting up lidar...");
  //TODO

	Serial.println("Setting up cli...");
	Serial.println("Commands:");
	Serial.println("  HELLO");
	cmdCallback.addCmd(PSTR("HELLO"), &handle_hello);
	Serial.println("  MOTOR {enable: 0/1} {left speed: 0..1} {right speed: 0..1} -> OK/ERR");
	cmdCallback.addCmd(PSTR("MOTOR"), &handle_motors);
	Serial.println("  SERVO {enable: 0/1} {0: 0..180} {1} {2} {3} -> OK/ERR");
	cmdCallback.addCmd(PSTR("SERVO"), &handle_servos);
	Serial.println("  STATUS -> {left speed} {right speed} {distance 0} .. {distance N}");
	cmdCallback.addCmd(PSTR("STATUS"), &handle_status);
	Serial.println("  DEBUG -> {speed control values}");
	cmdCallback.addCmd(PSTR("DEBUG"), &handle_debug);

	Serial.println("Done");
	Serial.println("");

	last_now = micros();
}

void loop() {
	unsigned long now = micros();

	if (cmdBuffer.readFromSerial(&Serial, 10)) {
		if (cmdParser.parseCmd(&cmdBuffer) != CMDPARSER_ERROR) {
			cmdCallback.processCmd(&cmdParser);
		}
		else {
			Serial.println("ERR Failed parsing arguments");
		}
  }

	handle_motor_pid(now);

	last_now = now;
}

// =================================================================
void handle_encoder_a()
{
	encoder_a.count(micros());
}
void handle_encoder_b()
{
	encoder_b.count(micros());
}

void handle_motor_pid(unsigned long now)
{
	static unsigned long last_now;
	encoder_a.update(now);
	encoder_b.update(now);

	pid_a_in = encoder_a.lastFrequency(now);
	pid_b_in = encoder_b.lastFrequency(now);
	if (pid_a_acc < 0)
		pid_a_in *= -1;
	if (pid_b_acc < 0)
		pid_b_in *= -1;

	pid_a.Compute(now);
	pid_b.Compute(now);
	pid_a_acc += pid_a_out;
	pid_b_acc += pid_b_out;
	pid_a_acc = min(1, max(-1, pid_a_acc));
	pid_b_acc = min(1, max(-1, pid_b_acc));
	motors.setMotorsSpeed(pid_a_acc, pid_b_acc);
	pid_delta = now - last_now;
	last_now = now;
}

void servos_enable(bool enable)
{
	digitalWrite(PIN_SERVOS_ENABLE, enable);
}

// ----------------------------------------------------------------
void handle_hello(CmdParser *myParser)
{
	Serial.println("Autonomous Robot Controller");
	Serial.print("Firmware Version: ");
	Serial.println(VERSION);
	Serial.print("Firmware supports: ");
	Serial.println(SUPPORTS);
	Serial.println("");
}
void handle_motors(CmdParser *myParser)
{
	bool enable = strcmp("1", myParser->getCmdParam(1)) == 0;
	pid_a_set = atof(myParser->getCmdParam(2));
	pid_b_set = atof(myParser->getCmdParam(3));
	motors.enable(enable);
	Serial.println("OK");
}
void handle_servos(CmdParser *myParser)
{
	bool enable = strcmp("1", myParser->getCmdParam(1)) == 0;

	servos_enable(enable);
	for (int i = 0; i < 4; i ++) {
		servo[i].write(atoi(myParser->getCmdParam(2 + i)));
	}

	Serial.println("OK");
}
void handle_status(CmdParser *myParser)
{
	unsigned long now = micros();
	float left = encoder_a.lastFrequency(now);
	float right = encoder_b.lastFrequency(now);
	Serial.print(left); Serial.print(" ");
	Serial.print(right); Serial.print(" ");
	Serial.println("");
}

void handle_debug(CmdParser *myParser)
{
	Serial.print(pid_delta); Serial.print(" ");
	Serial.print(pid_a_set); Serial.print(" ");
	Serial.print(pid_a_in); Serial.print(" ");
	Serial.print(pid_a_acc); Serial.print(" ");
	Serial.print(pid_b_set); Serial.print(" ");
	Serial.print(pid_b_in); Serial.print(" ");
	Serial.print(pid_b_acc); Serial.print(" ");
	Serial.println("");
}
