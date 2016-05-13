#include <wiringPiI2C.h>
#include <wiringPi.h>
#include "servoI2C.h"

int servoI2C::fd;

void servoI2C::Reset()
{
	fd= wiringPiI2CSetup(0x00);
	wiringPiI2CWrite(fd,0x06);
}

void servoI2C::SetPWMFrequency(int hz)
{
	//    """Set the PWM frequency to the provided value in hertz."""
    float prescaleval = 25000000.0;    //# 25MHz
    prescaleval /= 4096.0;       //# 12-bit
    prescaleval /= float(hz);
    prescaleval -= 1.0;
    prescale = prescaleval;//int(math.floor(prescaleval + 0.5))
    int oldmode = wiringPiI2CReadReg8(fd,MODE1);
    int newmode = (oldmode & 0x7F) | 0x10    ;// sleep
    wiringPiI2CWriteReg8(fd,MODE1, newmode)  ;// go to sleep
    wiringPiI2CWriteReg8(fd,PRESCALE, prescale);
    wiringPiI2CWriteReg8(fd,MODE1, oldmode);
    delay(5);
    wiringPiI2CWriteReg8(fd,MODE1, oldmode | 0x80);
}

void servoI2C::SetPWM(int channel, int on, int off)
{
	wiringPiI2CWriteReg8(fd,LED0_ON_L+4*channel, on & 0xFF);
    wiringPiI2CWriteReg8(fd,LED0_ON_H+4*channel, on >> 8);
    wiringPiI2CWriteReg8(fd,LED0_OFF_L+4*channel, off & 0xFF);
    wiringPiI2CWriteReg8(fd,LED0_OFF_H+4*channel, off >> 8);
}

void servoI2C::SetAllPWM(int on, int off)
{
	wiringPiI2CWriteReg8(fd,ALL_LED_ON_L, on & 0xFF);
    wiringPiI2CWriteReg8(fd,ALL_LED_ON_H, on >> 8);
    wiringPiI2CWriteReg8(fd,ALL_LED_OFF_L, off & 0xFF);
    wiringPiI2CWriteReg8(fd,ALL_LED_OFF_H, off >> 8);
}

int servoI2C::Initialise()
{
	fd = 0;
	if (fd = wiringPiI2CSetup(PWM_ADDRESS) < 0)
	{
		return 0;
	}
	SetAllPWM(0, 0);
    wiringPiI2CWriteReg8(fd,MODE2, OUTDRV);
    wiringPiI2CWriteReg8(fd,MODE1, ALLCALL);
    delay(5);//  # wait for oscillator
    int mode1 = wiringPiI2CReadReg8(fd,MODE1);
    mode1 = mode1 & ~SLEEP  // wake up (reset sleep)
    wiringPiI2CWriteReg8(fd,MODE1, mode1);
    delay(5);
}