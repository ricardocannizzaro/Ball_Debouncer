#ifndef SERVO_H
#define SERVO_H

/* our servo wrapper */

//NOTE: to use this the following files must be added to the project
//PCA9685.h,PCA9685.c //from https://github.com/TeraHz/PCA9685

#define SERVO_BUS (0)
#define SERVO_ADDRESS (0x40)
#define SERVO_CHANNEL (0)
#define SERVO_DEFAULT_FREQ (60)
#define SERVO_CLOSED_PWM_VALUE (0)
#define SERVO_OPEN_PWM_VALUE (4095)

//intiaialises the servo system
void servoInitialise();

//destroys the servo system
void servoFinalise();

//moves the servo to the open position
void servoOpen();

//moves the servo to the closed position
void servoClose();

//[0.0,1.0]
void servoSetPos(float percentage_open);

#endif //SERVO_H