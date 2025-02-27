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
byte intPin = 2;                                                    // Interrupt pin (not used) ... 2 and 3 are the Arduinos ext int pins
byte myLed = 13;                                                    // Set up pin 13 led for toggling
const int pin_BL = 10; 
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

float
Mag_x_offset = 366.695,
Mag_y_offset = 376.02,
Mag_z_offset = -397.455,
Mag_x_scale = 1.0184643,
Mag_y_scale = 0.97450244,
Mag_z_scale = 1.0081002;



// float
// Mag_x_offset = 0.0,
// Mag_y_offset = 0.0,
// Mag_z_offset = 0.0,
// Mag_x_scale = 1.0,
// Mag_y_scale = 1.0,
// Mag_z_scale = 1.0;



// Mag_x_offset = 137.41,
// Mag_y_offset = 189.01,
// Mag_z_offset = -890.54,
// Mag_x_scale = 0.73,
// Mag_y_scale = 0.72,
// Mag_z_scale = 4.40;

// ----- Magnetic declination
/*
  The magnetic declination for Lower Hutt, New Zealand is +22.5833 degrees
  Obtain your magnetic declination from http://www.magnetic-declination.com/
  By convention, declination is positive when magnetic north
  is east of true north, and negative when it is to the west.
  Substitute your magnetic declination for the "Declination" shown below.
*/
#define True_North false          // change this to "true" for True North                
float Declination = +3.633;     // substitute your magnetic declination

// ----- Processing variables
char InputChar;                   // incoming characters stored here
bool LinkEstablished = false;     // receive flag
String OutputString = "";         // outgoing data string to Processing

// ----- software timer
unsigned long Timer1 = 500000L;   // 500mS loop ... used when sending data to to Processing
unsigned long Stop1;              // Timer1 stops when micros() exceeds this value

// ----- MPU-9250 addresses
#define MPU9250_ADDRESS     0x68  // Device address when ADO = 0; Use 0x69 when AD0 = 1
#define AK8963_ADDRESS      0x0C  //  Address of magnetometer

/*
  See also MPU-9250 Register Map and Descriptions, Revision 4.0, and
  RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in the above document;
  The MPU9250 and MPU9150 are virtually identical but the latter has a different register map
  The MPU9255 is similar but WHO_AM_I reports 0x73
*/

// ----- MPU-9250 register map
#define AK8963_WHO_AM_I     0x00  // should return 0x48
#define AK8963_INFO         0x01
#define AK8963_ST1          0x02  // data ready status bit 0
#define AK8963_XOUT_L       0x03  // data
#define AK8963_XOUT_H       0x04
#define AK8963_YOUT_L       0x05
#define AK8963_YOUT_H       0x06
#define AK8963_ZOUT_L       0x07
#define AK8963_ZOUT_H       0x08
#define AK8963_ST2          0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL         0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC         0x0C  // Self test control
#define AK8963_I2CDIS       0x0F  // I2C disable
#define AK8963_ASAX         0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY         0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ         0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO    0x00
#define SELF_TEST_Y_GYRO    0x01
#define SELF_TEST_Z_GYRO    0x02

/*
  #define X_FINE_GAIN         0x03   // [7:0] fine gain
  #define Y_FINE_GAIN         0x04
  #define Z_FINE_GAIN         0x05
  #define XA_OFFSET_H         0x06   // User-defined trim values for accelerometer
  #define XA_OFFSET_L_TC      0x07
  #define YA_OFFSET_H         0x08
  #define YA_OFFSET_L_TC      0x09
  #define ZA_OFFSET_H         0x0A
  #define ZA_OFFSET_L_TC      0x0B
*/

#define SELF_TEST_X_ACCEL   0x0D
#define SELF_TEST_Y_ACCEL   0x0E
#define SELF_TEST_Z_ACCEL   0x0F

#define SELF_TEST_A         0x10

#define XG_OFFSET_H         0x13   // User-defined trim values for gyroscope
#define XG_OFFSET_L         0x14
#define YG_OFFSET_H         0x15
#define YG_OFFSET_L         0x16
#define ZG_OFFSET_H         0x17
#define ZG_OFFSET_L         0x18
#define SMPLRT_DIV          0x19
#define CONFIG              0x1A
#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C
#define ACCEL_CONFIG2       0x1D
#define LP_ACCEL_ODR        0x1E
#define WOM_THR             0x1F

#define MOT_DUR             0x20   // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR            0x21   // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR           0x22   // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN             0x23
#define I2C_MST_CTRL        0x24
#define I2C_SLV0_ADDR       0x25
#define I2C_SLV0_REG        0x26
#define I2C_SLV0_CTRL       0x27
#define I2C_SLV1_ADDR       0x28
#define I2C_SLV1_REG        0x29
#define I2C_SLV1_CTRL       0x2A
#define I2C_SLV2_ADDR       0x2B
#define I2C_SLV2_REG        0x2C
#define I2C_SLV2_CTRL       0x2D
#define I2C_SLV3_ADDR       0x2E
#define I2C_SLV3_REG        0x2F
#define I2C_SLV3_CTRL       0x30
#define I2C_SLV4_ADDR       0x31
#define I2C_SLV4_REG        0x32
#define I2C_SLV4_DO         0x33
#define I2C_SLV4_CTRL       0x34
#define I2C_SLV4_DI         0x35
#define I2C_MST_STATUS      0x36
#define INT_PIN_CFG         0x37
#define INT_ENABLE          0x38
#define DMP_INT_STATUS      0x39  // Check DMP interrupt
#define INT_STATUS          0x3A
#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40
#define TEMP_OUT_H          0x41
#define TEMP_OUT_L          0x42
#define GYRO_XOUT_H         0x43
#define GYRO_XOUT_L         0x44
#define GYRO_YOUT_H         0x45
#define GYRO_YOUT_L         0x46
#define GYRO_ZOUT_H         0x47
#define GYRO_ZOUT_L         0x48
#define EXT_SENS_DATA_00    0x49
#define EXT_SENS_DATA_01    0x4A
#define EXT_SENS_DATA_02    0x4B
#define EXT_SENS_DATA_03    0x4C
#define EXT_SENS_DATA_04    0x4D
#define EXT_SENS_DATA_05    0x4E
#define EXT_SENS_DATA_06    0x4F
#define EXT_SENS_DATA_07    0x50
#define EXT_SENS_DATA_08    0x51
#define EXT_SENS_DATA_09    0x52
#define EXT_SENS_DATA_10    0x53
#define EXT_SENS_DATA_11    0x54
#define EXT_SENS_DATA_12    0x55
#define EXT_SENS_DATA_13    0x56
#define EXT_SENS_DATA_14    0x57
#define EXT_SENS_DATA_15    0x58
#define EXT_SENS_DATA_16    0x59
#define EXT_SENS_DATA_17    0x5A
#define EXT_SENS_DATA_18    0x5B
#define EXT_SENS_DATA_19    0x5C
#define EXT_SENS_DATA_20    0x5D
#define EXT_SENS_DATA_21    0x5E
#define EXT_SENS_DATA_22    0x5F
#define EXT_SENS_DATA_23    0x60
#define MOT_DETECT_STATUS   0x61
#define I2C_SLV0_DO         0x63
#define I2C_SLV1_DO         0x64
#define I2C_SLV2_DO         0x65
#define I2C_SLV3_DO         0x66
#define I2C_MST_DELAY_CTRL  0x67
#define SIGNAL_PATH_RESET   0x68
#define MOT_DETECT_CTRL     0x69
#define USER_CTRL           0x6A   // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1          0x6B   // Device defaults to the SLEEP mode
#define PWR_MGMT_2          0x6C
#define DMP_BANK            0x6D   // Activates a specific bank in the DMP
#define DMP_RW_PNT          0x6E   // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG             0x6F   // Register in DMP from which to read or to which to write
#define DMP_REG_1           0x70
#define DMP_REG_2           0x71
#define FIFO_COUNTH         0x72
#define FIFO_COUNTL         0x73
#define FIFO_R_W            0x74
#define WHO_AM_I_MPU9250    0x75   // Should return 0x71; MPU9255 will return 0x73
#define XA_OFFSET_H         0x77
#define XA_OFFSET_L         0x78
#define YA_OFFSET_H         0x7A
#define YA_OFFSET_L         0x7B
#define ZA_OFFSET_H         0x7D
#define ZA_OFFSET_L         0x7E

// ----- Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0,               // 0.6 mG per LSB;
  MFS_16BITS                    // 0.15 mG per LSB
};

enum M_MODE {
  M_8HZ = 0x02,                 // 8 Hz ODR (output data rate) update
  M_100HZ = 0x06                // 100 Hz continuous magnetometer
};

// ----- Specify sensor full scale
byte Gscale = GFS_250DPS;
byte Ascale = AFS_2G;
byte Mscale = MFS_14BITS;                           // Choose either 14-bit or 16-bit magnetometer resolution (AK8963=14-bits)
byte Mmode = 0x02;                                  // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float aRes, gRes, mRes;                             // scale resolutions per LSB for the sensors

short accelCount[3];                                // Stores the 16-bit signed accelerometer sensor output
short gyroCount[3];                                 // Stores the 16-bit signed gyro sensor output
short magCount[3];                                  // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0},
                          magBias[3] = {0, 0, 0},
                                       magScale[3] = {0, 0, 0};    // Factory mag calibration, mag offset , mag scale-factor
float gyroBias[3] = {0, 0, 0},
                    accelBias[3] = {0, 0, 0};        // Bias corrections for gyro and accelerometer
short tempCount;                                    // temperature raw count output
float temperature;                                  // Stores the real internal chip temperature in degrees Celsius
float SelfTest[6];                                  // holds results of gyro and accelerometer self test

/*
  There is a tradeoff in the beta parameter between accuracy and response speed.
  In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
  However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
  Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
  By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
  I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
  the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
  In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
*/

// ----- global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);        // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);        // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)

// ----- Madgwick free parameters
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;     // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;     // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

// ----- Mahony free parameters
//#define Kp 2.0f * 5.0f                            // original Kp proportional feedback parameter in Mahony filter and fusion scheme
#define Kp 40.0f                                    // Kp proportional feedback parameter in Mahony filter and fusion scheme
#define Ki 0.0f                                     // Ki integral parameter in Mahony filter and fusion scheme

unsigned long delt_t = 0;                           // used to control display output rate
unsigned long count = 0, sumCount = 0;              // used to control display output rate
float pitch, roll, yaw;
float deltat = 0.0f, sum = 0.0f;                    // integration interval for both filter schemes
unsigned long lastUpdate = 0, firstUpdate = 0;      // used to calculate integration interval
unsigned long Now = 0;                              // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz;           // variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};              // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};                 // vector to hold integral error for Mahony method

// -----------------
// setup()
// -----------------
void setup()
{
  Wire.begin();
  Wire.setClock(400000);                            // 400 kbit/sec I2C speed
  while (!Serial);                                  // required for Feather M4 Express
  Serial.begin(115200);

  // ----- Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

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
  lcd.print(F("   Compass V8"));
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

  // ----- Read the WHO_AM_I register, this is a good test of communication
  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  delay(1000);

  if ((c == 0x71) || (c == 0x73)) // MPU9250=0x68; MPU9255=0x73
  {
    Serial.println(F("MPU9250 is online..."));
    Serial.println("");

    MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
    Serial.println(F("Self test (14% acceptable)"));
    Serial.print(F("x-axis acceleration trim within : ")); Serial.print(SelfTest[0], 1); Serial.println(F("% of factory value"));
    Serial.print(F("y-axis acceleration trim within : ")); Serial.print(SelfTest[1], 1); Serial.println(F("% of factory value"));
    Serial.print(F("z-axis acceleration trim within : ")); Serial.print(SelfTest[2], 1); Serial.println(F("% of factory value"));
    Serial.print(F("x-axis gyration trim within : ")); Serial.print(SelfTest[3], 1); Serial.println(F("% of factory value"));
    Serial.print(F("y-axis gyration trim within : ")); Serial.print(SelfTest[4], 1); Serial.println(F("% of factory value"));
    Serial.print(F("z-axis gyration trim within : ")); Serial.print(SelfTest[5], 1); Serial.println(F("% of factory value"));
    Serial.println("");

    calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers

    Serial.println(F("MPU9250 accelerometer bias"));
    Serial.print(F("x = ")); Serial.println((int)(1000 * accelBias[0]));
    Serial.print(F("y = ")); Serial.println((int)(1000 * accelBias[1]));
    Serial.print(F("z = ")); Serial.print((int)(1000 * accelBias[2]));
    Serial.println(F(" mg"));
    Serial.println("");

    Serial.println(F("MPU9250 gyro bias"));
    Serial.print(F("x = ")); Serial.println(gyroBias[0], 1);
    Serial.print(F("y = ")); Serial.println(gyroBias[1], 1);
    Serial.print(F("z = ")); Serial.print(gyroBias[2], 1);
    Serial.println(F(" o/s"));
    Serial.println("");
    delay(1000);

    initMPU9250();
    Serial.println(F("MPU9250 initialized for active data mode....")); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    Serial.println("");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    byte d = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);  // Read WHO_AM_I register for AK8963
    if (d == 0x48)
    {
      // ----- AK8963 found
      Serial.println(F("AK8963 is online ..."));

      // Get magnetometer calibration from AK8963 ROM
      initAK8963(magCalibration);
      Serial.println(F("AK8963 initialized for active data mode ...")); // Initialize device for active mode read of magnetometer
      Serial.println("");

      Serial.println(F("Asahi sensitivity adjustment values"));
      Serial.print(F("ASAX = ")); Serial.println(magCalibration[0], 2);
      Serial.print(F("ASAY = ")); Serial.println(magCalibration[1], 2);
      Serial.print(F("ASAZ = ")); Serial.println(magCalibration[2], 2);
      Serial.println("");
      delay(1000);
    }
    else
    {
      // ----- AK8963 not found
      Serial.print(F("AK8963 WHO_AM_I = ")); Serial.println(c, HEX);
      Serial.println(F("I should be 0x48"));
      Serial.print(F("Could not connect to AK8963: 0x"));
      Serial.println(d, HEX);
      Serial.println("");
      while (1) ; // Loop forever if communication doesn't happen
    }
  }
  else
  {
    // ----- MPU9250 not found
    Serial.print(F("MPU9250 WHO_AM_I = ")); Serial.println(c, HEX);
    Serial.println(F("I should be 0x71 or 0x73"));
    Serial.print(F("Could not connect to MPU9250: 0x"));
    Serial.println(c, HEX);
    Serial.println("");

  #ifdef LCD2
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("MPU9250 CHECK I2C"));
    lcd.setCursor(0, 1);
    lcd.print(F("CONNECTION"));
  #endif
    while (1) ; // Loop forever if communication doesn't happen
  }

  // --------------------------
  // TASK 0,1 tasks & messages
  // --------------------------
  if ((TASK == 0) || (TASK == 1))
  {
#ifdef LCD2
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Tumble compass"));
    lcd.setCursor(0, 1);
    lcd.print(F("for 30 seconds"));
#endif

    // ----- Calculate magnetometer offsets & scale-factors
    Serial.println(F("Magnetometer calibration ..."));
    Serial.println(F("Tumble/wave device for 30 seconds in a figure 8"));
    delay(2000);
    magCalMPU9250(magBias, magScale);

#ifdef LCD2
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Stop tumbling"));
#endif

    Serial.println(F("Stop tumbling"));
    Serial.println("");
    delay(4000);
  }

  if (TASK == 1)
  {
#ifdef LCD2
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Record offsets"));
    lcd.setCursor(0, 1);
    lcd.print(F("& scale-factors"));
#endif

    // ----- Message
    Serial.println(F("------------------------------------------"));
    Serial.println(F("Copy-&-paste the following code into your "));
    Serial.println(F("Arduino header then delete the old code."));
    Serial.println(F("------------------------------------------"));
    Serial.println(F(""));
    Serial.println(F("float"));
    Serial.print(F("Mag_x_offset = "));
    Serial.print(magBias[0]);
    Serial.println(",");
    Serial.print(F("Mag_y_offset = "));
    Serial.print(magBias[1]);
    Serial.println(",");
    Serial.print(F("Mag_z_offset = "));
    Serial.print(magBias[2]);
    Serial.println(",");
    Serial.print(F("Mag_x_scale = "));
    Serial.print(magScale[0]);
    Serial.println(",");
    Serial.print(F("Mag_y_scale = "));
    Serial.print(magScale[1]);
    Serial.println(",");
    Serial.print(F("Mag_z_scale = "));
    Serial.print(magScale[2]);
    Serial.println(F(";"));

    // ----- Halt program
    while (true);
  }
}

// ----------
// loop()
// ----------
void loop()
{
  refresh_data();                              // This must be done each time through the loop
  calc_quaternion();                           // This must be done each time through the loop

  // ----- Processing Tasks
  switch (TASK) {
    case 2:
      compass_cal();                          // Get compass offsets and scale-factors using Processing "compass_cal.pde" (circle-method)
      break;
    case 3:
      compass_rose();                         // View compass heading using Processing "compass_rose.pde"
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
        view_registers_SM();                  // Send  MPU9250 register contents to Serial Monitor (115200 bauds)
        break;
      case 5:
        view_heading_SM();                    // Send compass pitch roll & heading to Serial Monitor (115200 bauds)
        break;
      case 6:
        view_heading_LCD();                   // Send data to LCD display
        break;
      default:
        break;
    }

    // ----- Housekeeping
    digitalWrite(myLed, !digitalRead(myLed));  // Toggle led
    count = millis();                          // Reset timer
  }
}

// -------------------
// getMres()
// -------------------
/* Get magnetometer resolution */
void getMres() {
  switch (Mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
      mRes = 10.*4912. / 8190.; // Proper scale to return milliGauss
      break;
    case MFS_16BITS:
      mRes = 10.*4912. / 32760.0; // Proper scale to return milliGauss
      break;
  }
}

// -------------------
// getGres()
// -------------------
/* Get gyro resolution */
void getGres() {
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
      gRes = 250.0 / 32768.0;
      break;
    case GFS_500DPS:
      gRes = 500.0 / 32768.0;
      break;
    case GFS_1000DPS:
      gRes = 1000.0 / 32768.0;
      break;
    case GFS_2000DPS:
      gRes = 2000.0 / 32768.0;
      break;
  }
}

// -------------------
// getAres()
// -------------------
/* Get accelerometer resolution */
void getAres() {
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
      aRes = 2.0 / 32768.0;
      break;
    case AFS_4G:
      aRes = 4.0 / 32768.0;
      break;
    case AFS_8G:
      aRes = 8.0 / 32768.0;
      break;
    case AFS_16G:
      aRes = 16.0 / 32768.0;
      break;
  }
}

// -------------------
// readAccelData()
// -------------------
/* Read accelerometer registers */
void readAccelData(short * destination)
{
  byte rawData[6];  // x/y/z accel register data stored here
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((short)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((short)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((short)rawData[4] << 8) | rawData[5] ;
}

// -------------------
// readGyroData()
// -------------------
/* Read gyro registers */
void readGyroData(short * destination)
{
  byte rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((short)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((short)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((short)rawData[4] << 8) | rawData[5] ;
}

// -------------------
// readMagData()
// -------------------
/* Read magnetometer registers */
void readMagData(short * destination)
{
  byte rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  if (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
    readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
    byte c = rawData[6]; // End data read by reading ST2 register
    if (!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
      destination[0] = ((short)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
      destination[1] = ((short)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
      destination[2] = ((short)rawData[5] << 8) | rawData[4] ;
    }
  }
}

// -------------------
// readTempData()
// -------------------
/* Read temperature */
short readTempData()
{
  byte rawData[2];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  return ((short)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

// -------------------
// initAK8963()
// -------------------
/* Initialize the AK8963 magnetometer */
void initAK8963(float * destination)
{
  // First extract the factory calibration for each magnetometer axis
  byte rawData[3];  // x/y/z gyro calibration data stored here
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128) / 256. + 1.;
  destination[2] =  (float)(rawData[2] - 128) / 256. + 1.;
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  delay(10);
}

// -------------------
// initMPU9250()
// -------------------
/* Initialize the MPU9250|MPU6050 chipset */
void initMPU9250()
{
  // -----wake up device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  delay(100); // Wait for all registers to reset

  // -----get stable time source
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200);

  // ----- Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
  // be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

  // -----Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
  // determined inset in CONFIG above

  // Set gyroscope full scale range
  // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  byte c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x03; // Clear Fchoice bits [1:0]
  c = c & ~0x18; // Clear GFS bits [4:3]
  c = c | Gscale << 3; // Set full scale range for the gyro
  // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

  // ----- Set accelerometer full-scale range configuration
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | Ascale << 3; // Set full scale range for the accelerometer
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

  // ----- Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
  // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // ----- Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
  writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
  delay(100);
}

// -------------------
// calibrateMPU9250()
// -------------------
/*
  Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
  of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
*/
void calibrateMPU9250(float * dest1, float * dest2)
{
  byte data[12]; // data array to hold accelerometer and gyro x, y, z, data
  unsigned short ii, packet_count, fifo_count;
  long gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  // ----- reset device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);

  // ----- get stable time source; Auto select clock source to be PLL gyroscope reference if ready
  // else use the internal oscillator, bits 2:0 = 001
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
  writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
  delay(200);

  // ----- Configure device for bias calculation
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);

  // ----- Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  unsigned short  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  unsigned short  accelsensitivity = 16384;  // = 16384 LSB/g

  // ----- Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

  // ----- At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((unsigned short)data[0] << 8) | data[1];
  packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    short accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (short) (((short)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (short) (((short)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (short) (((short)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (short) (((short)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (short) (((short)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (short) (((short)data[10] << 8) | data[11]) ;

    accel_bias[0] += (long) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (long) accel_temp[1];
    accel_bias[2] += (long) accel_temp[2];
    gyro_bias[0]  += (long) gyro_temp[0];
    gyro_bias[1]  += (long) gyro_temp[1];
    gyro_bias[2]  += (long) gyro_temp[2];

  }
  accel_bias[0] /= (long) packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (long) packet_count;
  accel_bias[2] /= (long) packet_count;
  gyro_bias[0]  /= (long) packet_count;
  gyro_bias[1]  /= (long) packet_count;
  gyro_bias[2]  /= (long) packet_count;

  if (accel_bias[2] > 0L) {
    accel_bias[2] -= (long) accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
  }
  else {
    accel_bias[2] += (long) accelsensitivity;
  }

  // ----- Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0] / 4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1] / 4)       & 0xFF;
  data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2] / 4)       & 0xFF;

  // ----- Push gyro biases to hardware registers
  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

  // ----- Output scaled gyro biases for display in the main program
  dest1[0] = (float) gyro_bias[0] / (float) gyrosensitivity;
  dest1[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

  // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
  // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
  // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  long accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (long) (((short)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (long) (((short)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (long) (((short)data[0] << 8) | data[1]);

  unsigned long mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  byte mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for (ii = 0; ii < 3; ii++) {
    if ((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // ----- Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0] / 8);     // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1] / 8);
  accel_bias_reg[2] -= (accel_bias[2] / 8);

  //    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  //    data[1] = (accel_bias_reg[0])      & 0xFF;
  //    data[1] = data[1] | mask_bit[0];              // preserve temperature compensation bit when writing back to accelerometer bias registers
  //    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  //    data[3] = (accel_bias_reg[1])      & 0xFF;
  //    data[3] = data[3] | mask_bit[1];              // preserve temperature compensation bit when writing back to accelerometer bias registers
  //    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  //    data[5] = (accel_bias_reg[2])      & 0xFF;
  //    data[5] = data[5] | mask_bit[2];              // preserve temperature compensation bit when writing back to accelerometer bias registers
  //    //   Apparently this is not working for the acceleration biases in the MPU-9250
  //    //   Are we handling the temperature correction bit properly?

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFE;
  data[1] = data[1] | mask_bit[0];              // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFE;
  data[3] = data[3] | mask_bit[1];              // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFE;
  data[5] = data[5] | mask_bit[2];              // preserve temperature compensation bit when writing back to accelerometer bias registers
  // see https://github.com/kriswiner/MPU9250/issues/215

  // Push accelerometer biases to hardware registers
  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

  // ----- Output scaled accelerometer biases for display in the main program
  dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
  dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
  dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;
}

// -----------------
// magCalMPU9250()
// -----------------
/*
  Function which accumulates magnetometer data after device initialization.
  It calculates the bias and scale in the x, y, and z axes.
*/
void magCalMPU9250(float * bias_dest, float * scale_dest)
{
  unsigned short ii = 0, sample_count = 0;
  short mag_max[3]  = { -32768, -32768, -32768},
                      mag_min[3]  = {32767, 32767, 32767},
                                    mag_temp[3] = {0, 0, 0};
  long mag_bias[3] = {0, 0, 0};
  float mag_chord[3] = {0, 0, 0};
  float avg_chord;

  // ----- Make sure resolution has been calculated
  getMres();

  // ----- Tumble compass for 30 seconds
  /*
    At 8 Hz ODR (output data rate), new mag data is available every 125 ms
    At 100 Hz ODR, new mag data is available every 10 ms
  */

  if (Mmode == M_8HZ) sample_count = 240;         // 240*125mS=30 seconds
  if (Mmode == M_100HZ) sample_count = 3000;      // 3000*10mS=30 seconds

  for (ii = 0; ii < sample_count; ii++)
  {
    readMagData(mag_temp);  // Read the raw mag data

    for (int jj = 0; jj < 3; jj++)
    {
      if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }

    if (Mmode == M_8HZ) delay(135);               // At 8 Hz ODR, new mag data is available every 125 ms
    if (Mmode == M_100HZ) delay(12);              // At 100 Hz ODR, new mag data is available every 10 ms
  }

  // Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
  // Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
  // Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

  // ----- Get hard iron correction
  /* long data-type  */
  mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2;                       // data-type: long
  mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2;
  mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2;

  // ----- Save mag biases in G for main program
  /* float data-type  */
  bias_dest[0] = (float)mag_bias[0] * magCalibration[0] * mRes;       // rawMagX * ASAX * 0.6
  bias_dest[1] = (float)mag_bias[1] * magCalibration[1] * mRes;       // rawMagY * ASAY * 0.6
  bias_dest[2] = (float)mag_bias[2] * magCalibration[2] * mRes;       // rawMagZ * ASAZ * 0.6

  // ----- Get soft iron correction estimate
  /* float data-type */
  mag_chord[0]  = ((float)(mag_max[0] - mag_min[0])) / 2.0;
  mag_chord[1]  = ((float)(mag_max[1] - mag_min[1])) / 2.0;
  mag_chord[2]  = ((float)(mag_max[2] - mag_min[2])) / 2.0;
  avg_chord = (mag_chord[0] + mag_chord[1] + mag_chord[2]) / 3.0;

  // ----- calculate scale-factors
  /* Destination data-type is float */
  scale_dest[0] = avg_chord / mag_chord[0];
  scale_dest[1] = avg_chord / mag_chord[1];
  scale_dest[2] = avg_chord / mag_chord[2];

  //  Serial.print("bias_dest[0] "); Serial.println(bias_dest[0]);
  //  Serial.print("bias_dest[1] "); Serial.println(bias_dest[1]);
  //  Serial.print("bias_dest[2] "); Serial.println(bias_dest[2]);
  //  Serial.print("");
  //
  //  Serial.print("mag_chord[0]] "); Serial.println(mag_chord[0]);
  //  Serial.print("mag_chord[1]] "); Serial.println(mag_chord[1]);
  //  Serial.print("mag_chord[2]] "); Serial.println(mag_chord[2]);
  //  Serial.print("avg_chord] "); Serial.println(avg_chord);
  //  Serial.print("");
  //
  //  Serial.print("scale_dest[0] "); Serial.println(scale_dest[0]);
  //  Serial.print("scale_dest[1] "); Serial.println(scale_dest[1]);
  //  Serial.print("scale_dest[2] "); Serial.println(scale_dest[2]);
}

// ------------------
// MPU9250SelfTest()
// ------------------
/* Accelerometer and gyroscope self test; check calibration wrt factory settings */
void MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
  byte rawData[6] = {0, 0, 0, 0, 0, 0};
  byte selfTest[6];
  long gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
  float factoryTrim[6];
  byte FS = 0;

  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, FS << 3); // Set full scale range for the gyro to 250 dps
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, FS << 3); // Set full scale range for the accelerometer to 2 g

  for ( int ii = 0; ii < 200; ii++) { // get average current values of gyro and acclerometer

    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
    aAvg[0] += (short)(((short)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (short)(((short)rawData[2] << 8) | rawData[3]) ;
    aAvg[2] += (short)(((short)rawData[4] << 8) | rawData[5]) ;

    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
    gAvg[0] += (short)(((short)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (short)(((short)rawData[2] << 8) | rawData[3]) ;
    gAvg[2] += (short)(((short)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average current readings
    aAvg[ii] /= 200;
    gAvg[ii] /= 200;
  }

  // Configure the accelerometer for self-test
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  delay(25);  // Delay a while to let the device stabilize

  for ( int ii = 0; ii < 200; ii++) { // get average self-test values of gyro and acclerometer

    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
    aSTAvg[0] += (short)(((short)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[1] += (short)(((short)rawData[2] << 8) | rawData[3]) ;
    aSTAvg[2] += (short)(((short)rawData[4] << 8) | rawData[5]) ;

    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    gSTAvg[0] += (short)(((short)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[1] += (short)(((short)rawData[2] << 8) | rawData[3]) ;
    gSTAvg[2] += (short)(((short)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average self-test readings
    aSTAvg[ii] /= 200;
    gSTAvg[ii] /= 200;
  }

  // Configure the gyro and accelerometer for normal operation
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);
  delay(25);  // Delay a while to let the device stabilize

  // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
  selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
  selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
  selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
  selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
  selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
  factoryTrim[0] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
  factoryTrim[1] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
  factoryTrim[2] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
  factoryTrim[3] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
  factoryTrim[4] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
  factoryTrim[5] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
  // To get percent, must multiply by 100
  for (int i = 0; i < 3; i++) {
    destination[i]   = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.; // Report percent differences
    destination[i + 3] = 100.0 * ((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3] - 100.; // Report percent differences
  }
}

// --------------
// writeByte()
// --------------
void writeByte(byte address, byte subAddress, byte data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

byte readByte(byte address, byte subAddress)
{
  byte data; // `data` will store the register data
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (byte) 1);  // Read one byte from slave register address
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

// --------------
// readBytes()
// --------------
void readBytes(byte address, byte subAddress, byte count, byte * dest)
{
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  byte i = 0;
  Wire.requestFrom(address, count);  // Read bytes from slave register address
  while (Wire.available()) {
    dest[i++] = Wire.read();
  }         // Put read results in the Rx buffer
}

// ---------------------------
// MadgwickQuaternionUpdate()
// ---------------------------
/*
  Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
  (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
  which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
  device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
  The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
  but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
*/
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  // ----- Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // ----- Normalise accelerometer measurement
  norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // ----- Normalise magnetometer measurement
  norm = sqrtf(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;
  mx *= norm;
  my *= norm;
  mz *= norm;

  // ----- Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrtf(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // ----- Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
  norm = 1.0f / norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // ----- Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

  // ----- Integrate to yield quaternion
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;
  norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}

// --------------------------
// MahonyQuaternionUpdate()
// --------------------------
/*
  Similar to Madgwick scheme but uses proportional and integral filtering
  on the error between estimated reference vectors and measured ones.
*/
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  Serial.print(" an: "); Serial.print(ax);
  Serial.print(" ae: "); Serial.print(ay);
  Serial.print(" ad: "); Serial.print(az);
  Serial.print(" gn: "); Serial.print(gx);
  Serial.print(" ge: "); Serial.print(gy);
  Serial.print(" gd: "); Serial.print(gz);
  Serial.print(" mn: "); Serial.print(mx);
  Serial.print(" me: "); Serial.print(my);
  Serial.print(" md: "); Serial.print(mz);
  Serial.print(" q[0]: "); Serial.print(q[0]);
  Serial.print(" q[0]: "); Serial.print(q[1]);
  Serial.print(" q[0]: "); Serial.print(q[2]);
  Serial.print(" q[0]: "); Serial.println(q[3]);


  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // ----- Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // ----- Normalise accelerometer measurement
  norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  ax *= norm;
  ay *= norm;
  az *= norm;

  // ----- Normalise magnetometer measurement
  norm = sqrtf(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  mx *= norm;
  my *= norm;
  mz *= norm;

  // ----- Reference direction of Earth's magnetic field
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrtf((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

  // ----- Estimated direction of gravity and magnetic field
  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

  //  ----- Error is cross product between estimated direction and measured direction of gravity
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (Ki > 0.0f)
  {
    eInt[0] += ex;      // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
  }
  else
  {
    eInt[0] = 0.0f;     // prevent integral wind up
    eInt[1] = 0.0f;
    eInt[2] = 0.0f;
  }

  // ----- Apply feedback terms
  gx = gx + Kp * ex + Ki * eInt[0];
  gy = gy + Kp * ey + Ki * eInt[1];
  gz = gz + Kp * ez + Ki * eInt[2];

  // ----- Integrate rate of change of quaternion
  pa = q2;
  pb = q3;
  pc = q4;
  q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
  q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
  q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
  q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

  // ----- Normalise quaternion
  norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}

// ------------------------
// refresh_data()
// ------------------------
/* Get current MPU-9250 register values */
void refresh_data()
{
  // ----- If intPin goes high, all data registers have new data
  if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    readAccelData(accelCount);                          // Read the accelerometer registers
    getAres();                                          // Get accelerometer resolution

    // ----- Accelerometer calculations
    ax = (float)accelCount[0] * aRes;                   // - accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1] * aRes;                   // - accelBias[1];
    az = (float)accelCount[2] * aRes;                   // - accelBias[2];
    // Serial.print(" ax: ");
    // Serial.print(ax);
    // Serial.print(" ay: ");
    // Serial.print(ay);
    // Serial.print(" az: ");
    // Serial.print(az);

    // ----- Gyro calculations
    readGyroData(gyroCount);                            // Read the gyro registers
    getGres();                                          // Get gyro resolution

    // ----- Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0] * gRes; // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1] * gRes;
    gz = (float)gyroCount[2] * gRes;

    // Serial.print(" gx: ");
    // Serial.print(gx);
    // Serial.print(" gy: ");
    // Serial.print(gy);
    // Serial.print(" gz: ");
    // Serial.print(gz);

    // ----- Magnetometer calculations
    readMagData(magCount);                              // Read the magnetometer x|y| registers
    getMres();                                          // Get magnetometer resolution

    //    // ----- Kris Winer hard-iron offsets
    //    magBias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
    //    magBias[1] = +120.;  // User environmental x-axis correction in milliGauss
    //    magBias[2] = +125.;  // User environmental x-axis correction in milliGauss

    // ----- Copy hard-iron offsets
    magBias[0] = Mag_x_offset;                          // Get hard-iron offsets (from compass_cal)
    magBias[1] = Mag_y_offset;
    magBias[2] = Mag_z_offset;

    // ----- Copy the soft-iron scalefactors
    magScale[0] = Mag_x_scale;
    magScale[1] = Mag_y_scale;
    magScale[2] = Mag_z_scale;

    //    // ----- Calculate the magnetometer values in milliGauss
    //    /* Include factory calibration per data sheet and user environmental corrections */
    //    mx = (float)magCount[0] * mRes * magCalibration[0] - magBias[0];    // get actual magnetometer value, this depends on scale being set
    //    my = (float)magCount[1] * mRes * magCalibration[1] - magBias[1];
    //    mz = (float)magCount[2] * mRes * magCalibration[2] - magBias[2];

    // ----- Calculate the magnetometer values in milliGauss
    /* The above formula is not using the soft-iron scale factors */
    mx = ((float)magCount[0] * mRes * magCalibration[0] - magBias[0]) * magScale[0];    // (rawMagX*ASAX*0.6 - magOffsetX)*scalefactor
    my = ((float)magCount[1] * mRes * magCalibration[1] - magBias[1]) * magScale[1];    // (rawMagY*ASAY*0.6 - magOffsetY)*scalefactor
    mz = ((float)magCount[2] * mRes * magCalibration[2] - magBias[2]) * magScale[2];
    
    // Serial.print(" magCount[0]: "); Serial.print(magCount[0]);
    // Serial.print(" magCount[1]: "); Serial.print(magCount[1]);
    // Serial.print(" magCount[2]: "); Serial.print(magCount[2]);

    // Serial.print(" mRes: "); Serial.print(mRes);

    // Serial.print(" magCalibration[0]: "); Serial.print(magCalibration[0]);
    // Serial.print(" magCalibration[1]: "); Serial.print(magCalibration[1]);
    // Serial.print(" magCalibration[2]: "); Serial.print(magCalibration[2]);

    // Serial.print(" magBias[0]: "); Serial.print(magBias[0]);
    // Serial.print(" magBias[1]: "); Serial.print(magBias[1]);
    // Serial.print(" magBias[2]: "); Serial.print(magBias[2]);

    // Serial.print(" magScale[0]: "); Serial.print(magScale[0]);
    // Serial.print(" magScale[1]: "); Serial.print(magScale[1]);
    // Serial.println(" magScale[2]: "); Serial.print(magScale[2]);

    // Serial.print(" mx: ");
    // Serial.print(mx);
    // Serial.print(" my: ");
    // Serial.print(my);
    // Serial.print(" mz: ");
    // Serial.println(mz);  // (rawMagZ*ASAZ*0.6 - magOffsetZ)*scalefactor
    // Serial.print(" mRes[0]: ");
    // Serial.println(mRes);
  }
}

// ------------------------
// calc_quaternion()
// ------------------------
/* Send current MPU-9250 register values to Mahony quaternion filter */
void calc_quaternion()
{
  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;
  sum += deltat; // sum for averaging filter update rate
  sumCount++;

  /*
    Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
    the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
    We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
    For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
    in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
    This is ok by aircraft orientation standards!
    Pass gyro rate as rad/s
  */

  //  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);

  // ----- Apply NEU (north east up)signs when parsing values
  MahonyQuaternionUpdate(
    ax,                -ay,                  az,
    gx * DEG_TO_RAD,   -gy * DEG_TO_RAD,     gz * DEG_TO_RAD,
    my,                -mx,                 -mz);
}

// ------------------------
// compass_cal()
// ------------------------
/* Obtain magnetometer offsets and scale-factors using Processing "compass_cal.pde" */
void compass_cal()
{
  // ----- Locals
  float
  xPos,
  yPos,
  zPos;

  // ----- read input character
  if (Serial.available()) {
    InputChar = Serial.read();
    if ((InputChar == 's') || (InputChar == 'S')) {
      LinkEstablished = true;
    }
  }

  // ----- Read magnetometer registers
  readMagData(magCount);                                  // Read the magnetometer x|y| registers
  getMres();                                              // Get magnetometer resolution

  // ----- calculate the magnetometer values (no offsets)
  xPos = (float)magCount[0] * magCalibration[0] * mRes;   // raw mx * ASAX * 0.6 (mG)
  yPos = (float)magCount[1] * magCalibration[1] * mRes;   // raw my * ASAY * 0.6 (mG)
  zPos = (float)magCount[2] * magCalibration[2] * mRes;   // raw mz * ASAZ * 0.6 (mG)

  // ------ create output data string
  OutputString = String(xPos) + ',' + String(yPos) + ',' + String(zPos);

  // ----- send string if link established
  if (LinkEstablished && ((InputChar == 's') || (InputChar == 'S'))) {
    InputChar = 'x';
    Serial.println(OutputString);
  }

  // ----- send 'S' if link not established
  if (micros() > Stop1) {
    Stop1 += Timer1;
    if (!LinkEstablished) {
      Serial.println('S');
    }
  }
}

// ------------------------
// compass_rose()
// ------------------------
/* View heading using Processing "compass_rose.pde" */
void compass_rose()
{
  // ----- read input character
  if (Serial.available()) {
    InputChar = Serial.read();
    if ((InputChar == 's') || (InputChar == 'S')) {
      LinkEstablished = true;
    }
  }

  // ----- Calculate the yaw
  float yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);

  /*
    The normal range for yaw is  +/- 0..179 degrees. Adding 360 to
    the negative readings converts the range to 0..360 degrees. The yaw
    output is heavily damped but doesn't appear to suffer from pitch or roll.
    For all practical purposes the compass yaw and compass heading are the same
  */

  // ----- Convert yaw to compass heading
  float heading = yaw * RAD_TO_DEG;
  if (heading < 0) heading += 360.0;                  // Yaw goes negative between 180 amd 360 degrees
  if (True_North == true) heading += Declination;     // Calculate True North
  if (heading < 0) heading += 360.0;                  // Allow for under|overflow
  if (heading >= 360) heading -= 360.0;

  // ------ Create output data string
  OutputString = String(heading);

  // ----- Send string if link established
  if (LinkEstablished && ((InputChar == 's') || (InputChar == 'S'))) {
    Serial.println(OutputString);
  }

  // ----- send 'S' if link not established
  if (micros() > Stop1) {
    Stop1 += Timer1;
    if (!LinkEstablished) {
      Serial.println('S');
    }
  }
}

// ------------------------
// view_registers_SM()
// ------------------------
/* View registers on serial monitor */
void view_registers_SM()
{
  // ----- Print accelerometer values
  Serial.print("        Accel(mg)");
  print_number((short)(1000 * ax));
  print_number((short)(1000 * ay));
  print_number((short)(1000 * az));

  // ----- Print gyro values
  Serial.print("        Gyro(dps)");
  print_number((short)(gx));
  print_number((short)(gy));
  print_number((short)(gz));

  // ----- Print magnetometer values
  Serial.print("        Mag(mG)");
  print_number((short)(mx));
  print_number((short)(my));
  print_number((short)(mz));

  // ----- Print temperature in degrees Centigrade
  tempCount = readTempData();  // Read the adc values
  temperature = ((float) tempCount) / 333.87 + 21.0; // Temp in degrees C
  Serial.print("        Temp(C) ");
  Serial.println(temperature, 1);
}

// ------------------------
// view_heading_SM()
// ------------------------
/* View heading on serila monitor */
void view_heading_SM()
{
  //    Serial.print("Accel");
  //    print_number((short)(1000 * ax));
  //    print_number((short)(1000 * ay));
  //    print_number((short)(1000 * az));

  //    Serial.print("        Gyro");
  //    print_number((short)(gx));
  //    print_number((short)(gy));
  //    print_number((short)(gz));

  //    Serial.print("        Mag");
  //    print_number((short)(mx));
  //    print_number((short)(my));
  //    print_number((short)(mz));

  //    Serial.print("        q0|qx|qy|qz");
  //    print_number(q[0]);
  //    print_number(q[1]);
  //    print_number(q[2]);
  //    print_number(q[3]);

  /*
    Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
    In this coordinate system, the positive z-axis is down toward Earth.
    Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
    Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
    Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
    These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
    Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
    applied in the correct order which for this configuration is yaw, pitch, and then roll.
    For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
  */

  pitch = asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
  roll  = -atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
  yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);

  // ----- convert to degrees
  pitch *= RAD_TO_DEG;
  roll *= RAD_TO_DEG;
  yaw *= RAD_TO_DEG;

  // ----- calculate the heading
  /*
     The yaw and compass heading (after the next two lines) track each other 100%
  */
  float heading = yaw;
  if (heading < 0) heading += 360.0;                        // Yaw goes negative between 180 amd 360 degrees
  if (True_North == true) heading += Declination;           // Calculate True North
  if (heading < 0) heading += 360.0;                        // Allow for under|overflow
  if (heading >= 360) heading -= 360.0;

  // ----- send the results to the Serial Monitor
  Serial.print("        Pitch ");
  print_number((short)pitch);
  Serial.print("        Roll ");
  print_number((short)roll);
  Serial.print("        Heading ");
  print_number((short)heading);

  // ----- Print temperature in degrees Centigrade
  tempCount = readTempData();                               // Read the temperature registers
  temperature = ((float) tempCount) / 333.87 + 21.0;        // Temp in degrees C
  Serial.print("        Temp(C) ");
  Serial.print(temperature, 1);
  
  /*
    With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and
    > 200 Hz using the Mahony scheme even though the display refreshes at only 2 Hz.
    The filter update rate is determined mostly by the mathematical steps in the respective algorithms,
    the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR (output data rate) :
    an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
    filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively.
    This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
    This filter update rate should be fast enough to maintain accurate platform orientation for
    stabilization control of a fast - moving robot or quadcopter. Compare to the update rate of 200 Hz
    produced by the on - board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
    The 3.3 V 8 MHz Pro Mini is doing pretty well!
  */

  //      Serial.print("        Rate ");
  //      print_number((float) sumCount / sum, 2);
  //      Serial.print(" Hz");

  Serial.println("");
  count = millis();

  sumCount = 0;
  sum = 0;
}

// ------------------------
// view_heading_LCD()
// ------------------------
void view_heading_LCD()
{
  // ----- calculate pitch , roll, and yaw (radians)
  pitch = asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
  roll  = -atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
  yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);

  // ----- convert to degrees
  pitch *= RAD_TO_DEG;
  roll *= RAD_TO_DEG;
  yaw *= RAD_TO_DEG;
  
  // Serial.print(" eulerx: ");
  // Serial.print(roll);
  // Serial.print(" eulery: ");
  // Serial.print(pitch);
  // Serial.print(" eulerz: ");
  // Serial.println(yaw); 

  // ----- calculate the heading
  /*
     The yaw and compass heading (after the next two lines) track each other 100%
  */
  float heading = yaw;
  if (heading < 0) heading += 360.0;                        // Yaw goes negative between 180 amd 360 degrees
  if (True_North == true) heading += Declination;           // Calculate True North
  if (heading < 0) heading += 360.0;                        // Allow for under|overflow
  if (heading >= 360) heading -= 360.0;

  // ----- send the results to the Serial Monitor
  Serial.print("        Pitch ");
  print_number((short)pitch);
  Serial.print("        Roll ");
  print_number((short)roll);
  Serial.print("        Heading ");
  print_number((short)heading);

  // ----- Print temperature in degrees Centigrade
  tempCount = readTempData();                               // Read the temperature registers
  temperature = ((float) tempCount) / 333.87 + 21.0;        // Temp in degrees C
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
  lcd.setCursor(3, 0);
  lcd.print("Heading ");                                   // Heading
  lcd.print(int(heading));
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
