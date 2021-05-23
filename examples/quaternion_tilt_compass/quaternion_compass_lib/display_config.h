
// ----- configure 16x2 LCD display
/* Comment-out the following line if you are not using a 16x2 LCD */
#define LCD2
#define LCDParallel

#ifdef LCD2
// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
#ifdef LCDParallel

#include <LiquidCrystal.h>
LiquidCrystal lcd(8, 9, 4, 5, 6, 7); //const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;

#else

#include <LiquidCrystal_I2C.h>                                    // YwRobot Arduino LCM1602 IIC V1 library  
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);    // LCD pinouts: addr,en,rw,rs,d4,d5,d6,d7,bl,blpol

#endif

#endif