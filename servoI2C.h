#ifndef _SERVO_I2C_H
#define _SERVO_I2C_H
//registers
#define PCA9685_ADDRESS     (0x40)
#define MODE1               (0x00)
#define MODE2               (0x01)
#define SUBADR1             (0x02)
#define SUBADR2             (0x03)
#define SUBADR3             (0x04)
#define PRESCALE            (0xFE)
#define LED0_ON_L           (0x06)
#define LED0_ON_H           (0x07)
#define LED0_OFF_L          (0x08)
#define LED0_OFF_H          (0x09)
#define ALL_LED_ON_L        (0xFA)
#define ALL_LED_ON_H        (0xFB)
#define ALL_LED_OFF_L       (0xFC)
#define ALL_LED_OFF_H       (0xFD)

// Bits:
#define RESTART             (0x80)
#define SLEEP               (0x10)
#define ALLCALL             (0x01)
#define INVRT               (0x10)
#define OUTDRV              (0x04)

class servoI2C
{
	static int fd;
	public:
	static void Reset();
	static void SetPWMFrequency(int hz);
	static void SetPWM(int channel, int on, int off);
	static void SetAllPWM(int on, int off);
	static int Initialise();
};

#endif
