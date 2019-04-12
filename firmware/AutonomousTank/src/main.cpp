#include <Arduino.h>
#include <Servo.h>
#include <CmdBuffer.hpp>
#include <CmdCallback.hpp>
#include <CmdParser.hpp>

#include "effectors/HBridge_TB724.h"
#include "sensors/SWFrequencyCounter.h"
#include "PID/PID.h"
#include "sensors/TCA9548A_VL53L0X_array.h"

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
#define NUM_SERVOS 2

#define LOOP_DELTA 100

#define _PSTR(str) ((const __FlashStringHelper *)PSTR(str))

// =======================================================
Servo servo[NUM_SERVOS];
CmdCallback_P<6> cmdCallback;
CmdBuffer<128>  cmdBuffer;
CmdParser      cmdParser;
unsigned long last_now = 0;
unsigned long pid_delta = 0;
long last_ticks_a = 0;
long last_ticks_b = 0;
long max_ticks_a = 0;
long max_ticks_b = 0;
bool test_result = true;

#define LIDAR_NUM_SENSORS 8
#define LIDAR_YAW(index) ((index - 3) * 3.14156 / LIDAR_NUM_SENSORS)
Locomotion::RangeSensorBase::Reading_t lidar_readings[LIDAR_NUM_SENSORS] = {
	{0, LIDAR_YAW(0), 0},
	{0, LIDAR_YAW(1), 0},
	{0, LIDAR_YAW(2), 0},
	{0, LIDAR_YAW(3), 0},
	{0, LIDAR_YAW(4), 0},
	{0, LIDAR_YAW(5), 0},
	{0, LIDAR_YAW(6), 0},
	{0, -3.14156, 0},	// back facing lidar
};
Locomotion::RangeSensorBase::Measurement_t lidar_measurement = {
	false,
	0,
	LIDAR_NUM_SENSORS,
	lidar_readings,
};

Locomotion::SoftwareFrequencyCounter<long> encoder_a(true);
Locomotion::SoftwareFrequencyCounter<long> encoder_b(true);
Locomotion::HBridge_TB724 motors(PIN_MA_A, PIN_MA_B, PIN_MA_PWM,
						            				 PIN_MB_A, PIN_MB_B, PIN_MB_PWM,
										 					 	 PIN_MOTORS_ENABLE, 255);

Locomotion::real_t pid_a_in, pid_a_out, pid_a_set, pid_a_acc = 0;
Locomotion::real_t pid_b_in, pid_b_out, pid_b_set, pid_b_acc = 0;
Locomotion::PID pid_a(&pid_a_in, &pid_a_out, &pid_a_set, 0.0008, 0.0004, 0.00000, DIRECT);
Locomotion::PID pid_b(&pid_b_in, &pid_b_out, &pid_b_set, 0.0008, 0.0004, 0.00000, DIRECT);

Locomotion::TCA9548A lidar_mux(&Wire);
Locomotion::VL53L0X lidar_sensors[LIDAR_NUM_SENSORS] = {
	Locomotion::VL53L0X(&Wire),
	Locomotion::VL53L0X(&Wire),
	Locomotion::VL53L0X(&Wire),
	Locomotion::VL53L0X(&Wire),
	Locomotion::VL53L0X(&Wire),
	Locomotion::VL53L0X(&Wire),
	Locomotion::VL53L0X(&Wire),
	Locomotion::VL53L0X(&Wire),
};
Locomotion::TCA9548A_VL53L0X_Array lidar(&lidar_mux, 1, lidar_sensors, LIDAR_NUM_SENSORS);

// =======================================================
void motors_enable(bool);
void servos_enable(bool);

void handle_hello(CmdParser *myParser);
void handle_motors(CmdParser *myParser);
void handle_servos(CmdParser *myParser);
void handle_status(CmdParser *myParser);
void handle_debug(CmdParser *myParser);
void handle_reboot(CmdParser *myParser);
void handle_encoder_a();
void handle_encoder_b();
void handle_motor_pid(unsigned long now);

// =======================================================
void setup() {
	Serial.begin(115200);
	handle_hello(NULL);

	Serial.print(_PSTR("Initializing pins..."));
	pinMode(PIN_SERVOS_ENABLE, OUTPUT);
	servos_enable(false);

	motors.begin();

	servo[0].attach(PIN_SERVO_0);
	servo[1].attach(PIN_SERVO_1);
	//servo[2].attach(PIN_SERVO_2);
	//servo[3].attach(PIN_SERVO_3);
	Serial.println(_PSTR("OK"));

	Serial.print(_PSTR("Setting up speed controllers..."));
	pid_a.SetOutputLimits(-1.0, 1.0);
	pid_a.SetSampleTime(LOOP_DELTA * 1000L);
	pid_a.SetMode(AUTOMATIC);
	pid_b.SetOutputLimits(-1.0, 1.0);
	pid_b.SetSampleTime(LOOP_DELTA * 1000L);
	pid_b.SetMode(AUTOMATIC);

	Serial.println(_PSTR("OK"));

	Serial.print(_PSTR("Setting up encoder..."));
	attachInterrupt(0, handle_encoder_a, RISING);
  attachInterrupt(1, handle_encoder_b, RISING);
	Serial.println(_PSTR("OK"));

	Serial.print(_PSTR("Setting up lidar... "));
	lidar.begin(false);
	Wire.begin();
//	lidar.setIOTimeout(50000);

	bool test_results[1 + LIDAR_NUM_SENSORS] = {false,};
	lidar.test(test_results);
	for (size_t i = 0; i < (1 + LIDAR_NUM_SENSORS); i++) {
		Serial.print(i);
		if (!test_results[i]) {
			test_result = false;
			Serial.print(_PSTR("!"));
		}
	}
	if (test_result) {
		if (lidar.reset())
			Serial.println(_PSTR(" OK"));
		else
			Serial.println(_PSTR(" ERROR"));
	}
	else
		Serial.println(_PSTR(" ERROR"));

	// TODO: Add these commands:
	// DIFF {forward speed} {angular speed} {max tics}
	// OBSTACLE {0/1 for off/on} # default 1
	Serial.println(_PSTR("Setting up cli..."));
	Serial.println(_PSTR("Commands:"));
	Serial.println(_PSTR("  HELLO"));
	cmdCallback.addCmd(PSTR("HELLO"), &handle_hello);
	Serial.println(_PSTR("  MOTOR {enable: 0/1} {left speed: 0..1} {right speed: 0..1} {max N tics left} {max N ticks right}-> OK/ERR"));
	cmdCallback.addCmd(PSTR("MOTOR"), &handle_motors);
	Serial.println(_PSTR("  SERVO {enable: 0/1} {0: 0..180} {1} {2} {3} -> OK/ERR"));
	cmdCallback.addCmd(PSTR("SERVO"), &handle_servos);
	Serial.println(_PSTR("  STATUS -> {left speed} {right speed} {left N ticks} {right N ticks} {distance 0} .. {distance N}"));
	cmdCallback.addCmd(PSTR("STATUS"), &handle_status);
	Serial.println(_PSTR("  DEBUG -> {speed control values}"));
	cmdCallback.addCmd(PSTR("DEBUG"), &handle_debug);
	Serial.println(_PSTR("  REBOOT"));
	cmdCallback.addCmd(PSTR("REBOOT"), &handle_reboot);

	Serial.println(_PSTR("Done"));
	Serial.println();

	last_now = micros();
}

void loop() {
	unsigned long now = micros();
	encoder_a.update(now);
	encoder_b.update(now);
	handle_motor_pid(now);

	if (test_result) {
		lidar.startSingleSampling();
		lidar.readMeasurement(&lidar_measurement, LIDAR_NUM_SENSORS, 0);
	}

	if (cmdBuffer.readFromSerial(&Serial, LOOP_DELTA - 1 - 57)) {
		if (cmdParser.parseCmd(&cmdBuffer) != CMDPARSER_ERROR) {
			cmdCallback.processCmd(&cmdParser);
		}
		else {
			Serial.println(_PSTR("ERR Failed parsing arguments"));
		}
  }

	unsigned long tmp = micros();
	if (tmp - now < LOOP_DELTA * 1000L) {
		delayMicroseconds(LOOP_DELTA * 1000L - (tmp - now));
	}

//handle_debug(NULL);
	last_now = now;
}

// =================================================================
void handle_encoder_a()
{
	// TODO after fixing hall sensor chip, use this for direction.
	//register byte p = (~PINC) & _BV(0);
	register byte p = 0;
	encoder_a.count(1 - p * 2);
}
void handle_encoder_b()
{
	register byte p = ((~PINC) & _BV(3)) >> 3;  // 1 -> -1; 0 -> +1
	encoder_b.count(1 - p * 2);
}

void handle_motor_pid(unsigned long now)
{
	long ticks_a;
	long ticks_b;

	pid_a_in = encoder_a.lastFrequency();
	pid_b_in = encoder_b.lastFrequency();

	ticks_a = encoder_a.lastCounter();
	ticks_b = encoder_b.lastCounter();

	// FIXME: this will be unneccessary once we use direction info from encoder
	if (pid_a_acc < 0)
		pid_a_in *= -1;

	// TODO: Handle max_ticks

	pid_a.Compute(now);
	pid_b.Compute(now);
	pid_a_acc += pid_a_out;
	pid_b_acc += pid_b_out;
	pid_a_acc = min(1.0, max(-1.0, pid_a_acc));
	pid_b_acc = min(1.0, max(-1.0, pid_b_acc));
	motors.setMotorsSpeed(pid_a_acc, pid_b_acc);
	pid_delta = now - last_now;
}

void servos_enable(bool enable)
{
	digitalWrite(PIN_SERVOS_ENABLE, !enable);
}

// ----------------------------------------------------------------
void handle_hello(CmdParser *myParser)
{
	Serial.println(_PSTR("Autonomous Robot Controller"));
	Serial.print(_PSTR("Firmware Version: "));
	Serial.println(_PSTR(VERSION));
	Serial.print(_PSTR("Firmware supports: "));
	Serial.println(_PSTR(SUPPORTS));
	Serial.println();
}
void handle_motors(CmdParser *myParser)
{
	bool enable = strcmp("1", myParser->getCmdParam(1)) == 0;
	pid_a_set = atof(myParser->getCmdParam(2));
	pid_b_set = atof(myParser->getCmdParam(3));
	max_ticks_a = atof(myParser->getCmdParam(4));
	max_ticks_b = atof(myParser->getCmdParam(5));
	last_ticks_a = encoder_a.lastCounter();
	last_ticks_b = encoder_b.lastCounter();
	motors.enable(enable);
	Serial.println(_PSTR("OK"));
}
void handle_servos(CmdParser *myParser)
{
	bool enable = strcmp("1", myParser->getCmdParam(1)) == 0;

	servos_enable(enable);
	for (int i = 0; i < NUM_SERVOS; i ++) {
		servo[i].write(atoi(myParser->getCmdParam(2 + i)));
	}

	Serial.println(_PSTR("OK"));
}
void handle_status(CmdParser *myParser)
{
	float f_a = encoder_a.lastFrequency();
	float f_b = encoder_b.lastFrequency();
	long cnt_a = encoder_a.lastCounter();
	long cnt_b = encoder_b.lastCounter();
	Serial.print(f_a); Serial.print(_PSTR(" "));
	Serial.print(f_b); Serial.print(_PSTR(" "));
	Serial.print(cnt_a); Serial.print(_PSTR(" "));
	Serial.print(cnt_b); Serial.print(_PSTR(" "));
	for (size_t i = 0; i < LIDAR_NUM_SENSORS; i++) {
		Serial.print(lidar_measurement.readings[i].yaw);
		Serial.print(_PSTR(":"));
		Serial.print(lidar_measurement.readings[i].distance);
		Serial.print(_PSTR(" "));
	}
	Serial.println();
}

void handle_debug(CmdParser *myParser)
{
	Serial.print(pid_delta); Serial.print(_PSTR(" "));
	Serial.print(pid_a_set); Serial.print(_PSTR(" "));
	Serial.print(pid_a_in); Serial.print(_PSTR(" "));
	Serial.print(pid_a_acc); Serial.print(_PSTR(" "));
	Serial.print(pid_b_set); Serial.print(_PSTR(" "));
	Serial.print(pid_b_in); Serial.print(_PSTR(" "));
	Serial.print(pid_b_acc); Serial.print(_PSTR(" "));
	Serial.println();
}

void handle_reboot(CmdParser *myParser) {
	void (*resetFunc)(void) = 0;
	Serial.println(_PSTR("OK"));
	resetFunc();
	Serial.print(_PSTR("ERR"));
}
