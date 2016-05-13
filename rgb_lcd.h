#ifndef RGB_LCD_H
#define RGB_LCD_H

/* our LCD wrapper */

/* we are using code from http://wiringpi.com/examples/adafruit-rgb-lcd-plate-and-wiringpi/
using the wiring pi lcd library https://projects.drogon.net/raspberry-pi/wiringpi/lcd-library/ */

/*
TODO: implement color changing https://git.drogon.net/?p=wiringPi;a=blob_plain;f=examples/lcd-adafruit.c;hb=refs/heads/master


NOTE: i only got this working
after adding usr/local/lib/libwiringPi.o & libwiringPiDev.o libs
in the projet settings following
project/build-options/linker-settings/link-libraries
*/

//NOTE: following is a macro to use the LCD screens printf function
#define lcdprintf(msg) lcdPrintf(LCD::_handle,msg)

#define LCD_COLUMNS (16)
#define LCD_ROWS (2)

#define AF_BASE         100
#define AF_RED          (AF_BASE + 6)
#define AF_GREEN        (AF_BASE + 7)
#define AF_BLUE         (AF_BASE + 8)

#define AF_E            (AF_BASE + 13)
#define AF_RW           (AF_BASE + 14)
#define AF_RS           (AF_BASE + 15)

#define AF_DB4          (AF_BASE + 12)
#define AF_DB5          (AF_BASE + 11)
#define AF_DB6          (AF_BASE + 10)
#define AF_DB7          (AF_BASE +  9)

#define AF_SELECT       (AF_BASE +  0)
#define AF_RIGHT        (AF_BASE +  1)
#define AF_DOWN         (AF_BASE +  2)
#define AF_UP           (AF_BASE +  3)
#define AF_LEFT         (AF_BASE +  4)

class LCD
{
	public:
	static int _handle;

	//initialise the screen
	static void Initialise();

	//destroy the screen
	static void Finalise();

	//clears the screen and resets cursor position
	static void Clear();

	//prints text on the screen
	//NOTE: you can also use lcdprintf(args..)
	static void Print(const char * text);

};

#endif //RGB_LCD_H
