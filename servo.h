#ifndef SERVO_H
#define SERVO_H

/* our servo wrapper */

//NOTE: to use this the following files must be added to the project
//PCA9685.h,PCA9685.cpp from https://github.com/TeraHz/PCA9685
//I2C.h, I2C.cpp from https://github.com/TeraHz/I2C/tree/master/src
//also recommend testing the servo as outline in https://learn.adafruit.com/adafruit-16-channel-pwm-servo-hat-for-raspberry-pi/attach-and-test-the-hat

/*
I've got this to compile currently by adding both PCA9685 and I2C files
into this project (as source and header files). This is bad, these files should
be included in the project settings somewhere like you would normally include
a library. But this works in the meantime
*/


#define SERVO_BUS (0)
#define SERVO_ADDRESS (0x40)
#define SERVO_CHANNEL (0)
#define SERVO_DEFAULT_FREQ (60)
#define SERVO_CLOSED_PWM_VALUE (0)
#define SERVO_OPEN_PWM_VALUE (4095)

class PCA9685;

class servo
{
private:
    static PCA9685 * _servo_controller;
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
};


#endif //SERVO_H
