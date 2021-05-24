/*
  "quaternion_compass_new_v8.ino"
  Code by LINGIB
  https://www.instructables.com/member/lingib/instructables/
  Last update 16 Dec 2019.

  This compass uses:
  MPU9250 Basic Example Code by: Kris Winer,
  Date: April 1, 2014
  License: Beerware - Use this code however you'd like. If you find it useful you can buy me a beer some time.

  The code demonstrates basic MPU-9250 functionality including parameterizing the register addresses,
  initializing the sensor, getting properly scaled accelerometer, gyroscope, and magnetometer data out.
  Addition of 9 DoF sensor fusion using open source Madgwick and Mahony filter algorithms.

  ------
  About:
  ------
  This sketch has been tested on an Arduino Uno R3 and a Sparkfun Feather M4 Express

  The compass uses quaternions and behaves as though it is tilt-stabilized.

  The code uses the open source Madgwick and Mahony filter quaternion algorithms for calculating
  the pitch, roll, and yaw.

  The print_number() function uses two overloaded functions that keep your column-numbers
  steady, regardless of size or sign, by converting each number to a string then
  pre-padding each column-width with spaces. The column-widths, which are currently
  set to 6 may be altered by changing each "6" to "whatever-the-number-will-fit-into".

  ---------------
  Hardware setup:
  ---------------
  MPU9250 Breakout --------- Arduino
  VDD ---------------------- 5V
  SDA ---------------------- A4
  SCL ---------------------- A5
  GND ---------------------- GND

  External pull-ups are not required as the MPU9250 has internal 10K pull-ups to an internal 3.3V supply.

  The MPU9250 is an I2C sensor and uses the Arduino Wire library. Because the sensor is not 5V tolerant,
  the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.

  This may be achieved by editing lines 75,76,77 in your
  "C:\Users\Your_name\Documents\Arduino\libraries\Wire\utility\wire.c" file to read:

  // deactivate internal pullups for twi.
  digitalWrite(SDA, 0);
  digitalWrite(SCL, 0);

  ---------------
  Terms of use:
  ---------------
  The software is provided "AS IS", without any warranty of any kind, express or implied,
  including but not limited to the warranties of mechantability, fitness for a particular
  purpose and noninfringement. In no event shall the authors or copyright holders be liable
  for any claim, damages or other liability, whether in an action of contract, tort or
  otherwise, arising from, out of or in connection with the software or the use or other
  dealings in the software.

  -----------
  Warning:
  -----------
  Do NOT use this compass in situations involving safety to life such as navigation at sea.
*/

#include <Wire.h>
#include "MPU9250.h"


// ----- configure 16x2 LCD display
/* Comment-out the following line if you are not using a 16x2 LCD */
// #define LCD2
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


// ----- Arduino pin definitions
// ----- Select a TASK
/*
  Choose a TASK from the following list:
  #define TASK 0    // Calibrate each time at start-up
  #define TASK 1    // Calibrate once ... using onboard code
  #define TASK 2    // Calibrate once ... using external "compass_cal.pde" software
  #define TASK 3    // View ... compass-heading using external "compass_rose.pde" software
  #define TASK 4    // View ... register contents on "Serial Monitor" (115200 bauds)
  #define TASK 5    // View ... pitch, roll, and compass-heading on "Serial Monitor" (115200 bauds)
  #define TASK 6    // View ... pitch, roll, and compass-heading on 16x2 LCD display
*/

#define TASK 6

// ----- user offsets and scale-factors
/*
  Each of the following values must be overwritten with the offsets and scale - factors for
  YOUR location otherwise you will have to "tumble" your compass every time you switch it on.
  here are two methods for obtaining this data :

  Method 1 :
  ----------
  Set "#define TASK 1". Upload this change to your Arduino.
  You will be asked to "tumble" your  compass for 30 seconds.
  Replace (copy - & -paste) the values below with the offsets and scale - factors that appear on your computer screen.
  Once you have done this select one of  TASKs 3, 4, or 5 and upload these changes to your Arduino
  This method is less accurate than Method 2

  Method 2 :
  ----------
  Set "#define TASK 2". Upload this change to your Arduino.
  Run Processing "compass_cal.pde" and follow the on - screen instructions.
  Replace (copy - & - paste) the values below with the offsets and scale - factors that appear on your computer screen.
  Close Processing "compass_cal.pde"
  Once you have done this select one of  TASKs 3, 4, or 5 and upload these changes to your Arduino
  This method is more accurate, and more consistent, than method 1
*/

// ----------------
//  Max|min values 
// ----------------
// xMax: 895.58	xMin: -183.35
// yMax: 948.83	yMin: -182.74
// zMax: 156.26	zMin: -924.0

// ----- NZ offsets & scale-factors (TASK 2 ... 2nd run)

// float
// Mag_x_offset = 366.695,
// Mag_y_offset = 386.56,
// Mag_z_offset = -380.47,
// Mag_x_scale = 1.0603912,
// Mag_y_scale = 0.9851133,
// Mag_z_scale = 0.9598401;
unsigned long delt_t = 0;                           // used to control display output rate
unsigned long count = 0, sumCount = 0;              // used to control display output rate
float pitch, roll, yaw;
float temperature;                                  // Stores the real internal chip temperature in degrees Celsius
float deltat = 0.0f, sum = 0.0f;                    // integration interval for both filter schemes

float
Mag_x_offset = 366.695,
Mag_y_offset = 376.02,
Mag_z_offset = -397.455,
Mag_x_scale = 1.0184643,
Mag_y_scale = 0.97450244,
Mag_z_scale = 1.0081002;


// ----- Processing variables
char InputChar;                   // incoming characters stored here
bool LinkEstablished = false;     // receive flag
String OutputString = "";         // outgoing data string to Processing

// ----- software timer
unsigned long Timer1 = 500000L;   // 500mS loop ... used when sending data to to Processing
unsigned long Stop1;              // Timer1 stops when micros() exceeds this value

MPU9250 mpu;
uint32_t timestamp{0};
// -----------------
// setup()
// -----------------
bool first_time = true;
void setup()
{
  Wire.begin();
  Wire.setClock(400000);                            // 400 kbit/sec I2C speed
  // while (!Serial);                                  // required for Feather M4 Express
  Serial.begin(115200);

  MPU9250Setting setting;
  setting.accel_fs_sel = ACCEL_FS_SEL::A2G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G250DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M14BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
  setting.gyro_fchoice = 0x00;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  setting.accel_fchoice = 0x00;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;


if (!mpu.setup(0x68, setting)) {  // change to your own address
}

mpu.calibrateAccelGyro();
mpu.setMagBias(Mag_x_offset, Mag_y_offset, Mag_z_offset);
mpu.setMagScale(Mag_x_scale, Mag_y_scale, Mag_z_scale);

mpu.selectFilter(QuatFilterSel::MAHONYEM);

delay(2000);
timestamp = micros();

}

// ----------
// loop()
// ----------
void loop()
{
  // mpu.update(timestamp);
  if(mpu.update(micros() - timestamp))
  {
    printHeading();
  }
  timestamp = micros();
  delay(200);
}


// ------------------------
// view_heading_LCD()
// ------------------------
void printHeading()
{
  
  Serial.print("Heading ");
  Serial.println((int)mpu.getHeading());

  

}

