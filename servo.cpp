#include "servo.h"
#include "servoI2C.h"
#include <mcp3004.h>
#include <wiringPi.h>

PCA9685 * servo::_servo_controller;

void servo::Initialise()
{
	servoI2C::Initialise();
	servoI2C::SetPWMFrequency(SERVO_DEFAULT_FREQ);
	mcp3004Setup(ADC_BASE,ADC_SPI_CHANNEL);
}

void servo::Finalise()
{
	servoI2C::Reset();
}

void servo::Open()
{
	servoI2C::SetPWM(SERVO_CHANNEL,0,SERVO_OPEN_PWM_VALUE);
}

void servo::Close()
{
	servoI2C::SetPWM(SERVO_CHANNEL,0,SERVO_CLOSED_PWM_VALUE);
}

void servo::SetPos(float percentage_open)
{
	float v = (float)SERVO_CLOSED_PWM_VALUE +
			(((float)SERVO_OPEN_PWM_VALUE - (float)SERVO_CLOSED_PWM_VALUE)  * percentage_open);
	servoI2C::SetPWM(SERVO_CHANNEL,0,(int)v);
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
    int servoPos = GetServoPos();
	bool result = servoPos > SERVO_OPEN_THRESHOLD;
	return result;
}

bool servo::IsClosed()
{
    int servoPos = GetServoPos();
	bool result = servoPos < SERVO_CLOSED_THRESHOLD;
	return result;
}
