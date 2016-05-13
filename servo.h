#ifndef SERVO_H
#define SERVO_H

/* our servo wrapper */

//also recommend testing the servo as outline in https://learn.adafruit.com/adafruit-16-channel-pwm-servo-hat-for-raspberry-pi/attach-and-test-the-hat


/*
ADC NOTES:
run terminal commands below to test if the ADC is wired properly
gpio load spi

Note: not sure which will work, some combination of both of the following lines...
gpio -x mcp3004:100:0 aread 100
gpio -x gpio -x mcp3008:200:0 aread 200
*/

#define ADC_BASE (200)				//TODO: Check over these
#define ADC_SPI_CHANNEL (0)
#define ADC_POT_CHANNEL (0)         //the adc pin/channel the servo pot is connected to?

//#define SERVO_BUS (1)
//#define SERVO_ADDRESS (0x40)
//#define SERVO_CHANNEL (1)
#define SERVO_DEFAULT_FREQ (60)
#define SERVO_CLOSED_PWM_VALUE (3900) // (0)
#define SERVO_OPEN_PWM_VALUE (4095)//(4095)     // TODO check if these tick values are correct for our motor!
#define SERVO_CLOSED_THRESHOLD (100)
#define SERVO_OPEN_THRESHOLD (1024-100)

class PCA9685;

class servo
{
private:
    //static PCA9685 * _servo_controller;
public:

    //intiaialises the servo system
    static void Initialise();

    //destroys the servo system
    static void Finalise();

    //moves the servo to the open position
    static void Open();

    //moves the servo to the closed position
    static void Close();

    //set the percentage the servo is fully open, valid value between [0.0,1.0]
    static void SetPos(float percentage_open);

	//returns the position of the servo [0,1024]
	static int GetServoPos();

	//returns the value of an adc input channel [0,1024]
	static int GetAnalogInput(int adc_channel);

	//returns if the servo is open
	static bool IsOpen();

	//returns if the servo is closed
	static bool IsClosed();
};


#endif //SERVO_H
