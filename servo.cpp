#include "servo.h"
#include "PCA9685.h" //from https://github.com/TeraHz/PCA9685

PCA9685 * servo::_servo_controller;

void servo::Initialise()
{
	_servo_controller = 0;
	_servo_controller = new PCA9685(SERVO_BUS,SERVO_ADDRESS);
	_servo_controller->setPWMFreq(SERVO_DEFAULT_FREQ);
}

void servo::Finalise()
{
	delete _servo_controller;
	_servo_controller = 0;
}

void servo::Open()
{
	_servo_controller->setPWM(SERVO_CHANNEL,SERVO_OPEN_PWM_VALUE);
}

void servo::Close()
{
	_servo_controller->setPWM(SERVO_CHANNEL,SERVO_OPEN_PWM_VALUE);
}

void servo::SetPos(float percentage_open)
{
	float v = (float)SERVO_CLOSED_PWM_VALUE +
			(((float)SERVO_OPEN_PWM_VALUE - (float)SERVO_CLOSED_PWM_VALUE)  * percentage_open);
	_servo_controller->setPWM(SERVO_CHANNEL,v);
}
