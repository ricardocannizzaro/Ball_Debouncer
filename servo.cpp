#include "servo.h"
#include "PCA9685.h" //from https://github.com/TeraHz/PCA9685
#include <mcp3004.h>
#include <wiringPi.h>

PCA9685 * servo::_servo_controller;

void servo::Initialise()
{
	_servo_controller = 0;
	_servo_controller = new PCA9685(SERVO_BUS,SERVO_ADDRESS);
	_servo_controller->setPWMFreq(SERVO_DEFAULT_FREQ);
	mcp3004Setup(ADC_BASE,ADC_SPI_CHANNEL);
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

int servo::GetAnalogInput(int adc_channel)
{
	int result = analogRead(ADC_BASE + adc_channel);
	return result;
}

int servo::GetServoPos()
{
	int result = GetAnalogInput(ADC_POT_CHANNEL);
	return result;
}

bool servo::IsOpen()
{
	bool result = GetServoPos() > SERVO_OPEN_THRESHOLD;
	return result;
}

bool servo::IsClosed()
{
	bool result = GetServoPos() < SERVO_CLOSED_THRESHOLD;
	return result;
}
