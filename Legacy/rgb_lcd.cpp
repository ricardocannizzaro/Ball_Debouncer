#include "rgb_lcd.h"
#include <mcp23017.h>
#include <lcd.h>

int LCD::_handle;

//initialise the screen
void LCD::Initialise()
{
	_handle = 0;
	mcp23017Setup (AF_BASE, 0x20) ;
	_handle = lcdInit (LCD_ROWS, LCD_COLUMNS, 4, AF_RS, AF_E, AF_DB4,AF_DB5,AF_DB6,AF_DB7, 0,0,0,0) ;
	lcdDisplay(_handle,1);
}

//destroy the screen
void LCD::Finalise()
{
	//TODO: more destruction here?
	_handle = 0;
}

//clears the screen and resets cursor position
void LCD::Clear()
{
	lcdHome(_handle);
	lcdClear(_handle);
}

//prints text on the screen
void LCD::Print(const char * text)
{
	lcdPuts(_handle,text);
}
