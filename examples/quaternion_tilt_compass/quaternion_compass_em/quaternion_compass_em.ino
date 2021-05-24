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

#include <SPI.h>
#include <Wire.h>
#include "MPU9250.h"

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
#define Serial SERIAL_PORT_USBVIRTUAL                               // Required for Serial on Zero based boards
#endif

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


// -----------------
// setup()
// -----------------
void setup()
{
  Wire.begin();
  Wire.setClock(400000);                            // 400 kbit/sec I2C speed
  while (!Serial);                                  // required for Feather M4 Express
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



  // ----- Display title
  Serial.println(F("MPU-9250 Quaternion Compass"));
  Serial.println("");
  delay(2000);

  // ----- Level surface message
  Serial.println(F("Place the compass on a level surface"));
  Serial.println("");
  delay(2000);

  

  // ------------------------
  // TASK 1,2,3,4,5 messages
  // ------------------------
#ifdef LCD2
  // ----- Start LCD
  lcd.begin(16, 2);                                 // Configure 16 char x 2 line LCD display

  // ----- Display title
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("   Quaternion"));
  lcd.setCursor(0, 1);
  lcd.print(F("   Compass EM V1"));
  delay(2000);

  // ----- Level surface message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Place compass on"));
  lcd.setCursor(0, 1);
  lcd.print(F("level surface"));
  delay(2000);

  if (TASK == 2) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Run compass_cal"));
    lcd.setCursor(0, 1);
    lcd.print(F("on your PC"));
    delay(2000);
  }

  if (TASK == 3) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Run compass_rose"));
    lcd.setCursor(0, 1);
    lcd.print(F("on your PC"));
    delay(2000);
  }

  if ((TASK == 4) || (TASK == 5)) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Serial Monitor"));
    lcd.setCursor(0, 1);
    lcd.print(F("115200 bauds"));
    delay(2000);
  }
#endif

if (!mpu.setup(0x68, setting)) {  // change to your own address
#ifdef LCD2
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Error I2C Check"));
  lcd.setCursor(0, 1);
  lcd.print(F("   Connection"));
#endif
    
  while (1) {
    Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
    delay(5000);
  }
}

mpu.calibrateAccelGyro();
// mpu.setMagneticDeclination(3.633);
mpu.setMagBias(Mag_x_offset, Mag_y_offset, Mag_z_offset);
mpu.setMagScale(Mag_x_scale, Mag_y_scale, Mag_z_scale);

mpu.selectFilter(QuatFilterSel::MAHONYEM);

print_calibration();
delay(2000);

}

// ----------
// loop()
// ----------
void loop()
{
  mpu.update();

  // Serial.print(" ax: ");
  // Serial.print(mpu.getAcc(0));
  // Serial.print(" ay: ");
  // Serial.print(mpu.getAcc(1));
  // Serial.print(" az: ");
  // Serial.print(mpu.getAcc(2));
  // Serial.print(" gx: ");
  // Serial.print(mpu.getGyro(0));
  // Serial.print(" gy: ");
  // Serial.print(mpu.getGyro(1));
  // Serial.print(" gz: ");
  // Serial.print(mpu.getGyro(2));
  // Serial.print(" mx: ");
  // Serial.print(mpu.getMag(0));
  // Serial.print(" my: ");
  // Serial.print(mpu.getMag(1));
  // Serial.print(" mz: ");
  // Serial.println(mpu.getMag(2));

  // ----- Processing Tasks
  switch (TASK) {
    // case 2:
    //   compass_cal();                          // Get compass offsets and scale-factors using Processing "compass_cal.pde" (circle-method)
    //   break;
    case 3:
      // compass_rose();                         // View compass heading using Processing "compass_rose.pde"
      break;
    default:
      break;
  }

  // ----- Perform these tasks every 500mS
  delt_t = millis() - count;
  if (delt_t > 500)
  {
    switch (TASK)
    {
      case 0:
        view_heading_LCD();                   // Send data to LCD display
        break;
      case 4:
        // view_registers_SM();                  // Send  MPU9250 register contents to Serial Monitor (115200 bauds)
        break;
      case 5:
        // view_heading_SM();                    // Send compass pitch roll & heading to Serial Monitor (115200 bauds)
        break;
      case 6:
        view_heading_LCD();                   // Send data to LCD display
        break;
      default:
        break;
    }

    // ----- Housekeeping
    // digitalWrite(myLed, !digitalRead(myLed));  // Toggle led
    count = millis();                          // Reset timer
  }
}


// ------------------------
// view_heading_LCD()
// ------------------------
void view_heading_LCD()
{
  // mpu.getPitch()
  // mpu.getRoll()
  // mpu.getHeading()
  // mpu.getTemperature()
  // ----- calculate pitch , roll, and yaw (radians)
  
  // pitch = mpu.getPitch();
  // roll  = mpu.getRoll();
  // yaw   = mpu.getYaw();

  float pitch = mpu.getEulerX();
  float roll  = mpu.getEulerY();
  float yaw   = mpu.getEulerZ();


  // float yaw2 = atan2(2.0f * (mpu.getQuaternionX() * mpu.getQuaternionY() + mpu.getQuaternionW() * mpu.getQuaternionZ()), mpu.getQuaternionW() * mpu.getQuaternionW() + mpu.getQuaternionX() * mpu.getQuaternionX() - mpu.getQuaternionY() * mpu.getQuaternionY() - mpu.getQuaternionW() * mpu.getQuaternionW());
  float q[4] = {mpu.getQuaternionW(), mpu.getQuaternionX(), mpu.getQuaternionY(), mpu.getQuaternionZ()};
  float yaw2   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);

  /*
     The yaw and compass heading (after the next two lines) track each other 100%
  */
  yaw2 *= 180/3.14;
  yaw2 += 3.633;
  float heading = yaw2;
  if (heading < 0) heading += 360.0;                        // Yaw goes negative between 180 amd 360 degrees
  if (heading >= 360) heading -= 360.0;

  // ----- send the results to the Serial Monitor
  Serial.print("        Pitch ");
  print_number((short)pitch);
  Serial.print("        Roll ");
  print_number((short)roll);
  Serial.print("        Heading ");
  print_number((short)heading);
  Serial.print("        HeadingM ");
  print_number((short)mpu.getHeading());

  // ----- Print temperature in degrees Centigrade
  temperature = mpu.getTemperature();       // Temp in degrees C
  Serial.print("        Temp(C) ");
  Serial.print(temperature, 1);

#ifdef LCD2
  lcd.clear();

  /*
     The pitch and roll readings are fixed width.
     This code can be shortened if you need more processing time
  */

  // ----- display the pitch
  lcd.setCursor(0, 1);
  (pitch < 0) ? lcd.print("P-") : lcd.print("P+");
  lcd.print((int(abs(pitch * 10)) / 1000));                // 1st digit
  lcd.print((int(abs(pitch * 10)) / 100) % 10);            // 2nd digit
  lcd.print((int(abs(pitch * 10)) / 10) % 10);             // 3rd digit
  lcd.print(".");                                          // decimal point
  lcd.print(int(abs(pitch * 10)) % 10);                    // last digit

  // ----- display the roll
  lcd.setCursor(9, 1);
  (roll < 0) ? lcd.print("R-") : lcd.print("R+");
  lcd.print((int(abs(roll * 10)) / 1000));                 // 1st digit
  lcd.print((int(abs(roll * 10)) / 100) % 10);             // 2nd digit
  lcd.print((int(abs(roll * 10)) / 10) % 10);              // 3rd digit
  lcd.print(".");                                          // decimal point
  lcd.print(int(abs(roll * 10)) % 10);                     // last digit

  // ----- display the heading
  lcd.setCursor(0, 0);
  lcd.print("H ");                                   // Heading
  lcd.print(int(heading));

  lcd.setCursor(6, 0);
  lcd.print("HM ");                                   // Heading
  lcd.print(int(mpu.getHeading()));
#endif

  // ----- housekeeping
  Serial.println("");
  count = millis();
  sumCount = 0;
  sum = 0;
}

// ------------------------
// print_number()
// ------------------------
/* Overloaded routine to stop integer numbers jumping around */
long print_number(short number) {
  String myString = String(number);
  short numberChars = myString.length();
  for (short i = 0; i < 6 - numberChars; i++) {
    Serial.print(" ");
  }
  Serial.print(myString);
}

// ------------------------
// print_number()
// ------------------------
/* Overloaded routine to stop float numbers jumping around */
float print_number(float number) {
  String myString = String(number);
  short numberChars = myString.length();
  for (short i = 0; i < 6 - numberChars; i++) {
    Serial.print(" ");
  }
  Serial.print(myString);
}



void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();

    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() );
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() );
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() );
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}
