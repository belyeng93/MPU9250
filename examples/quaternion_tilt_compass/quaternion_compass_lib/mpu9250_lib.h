/**
 * @file mpu9250_lib.h
 * @author Emanuele Belia
 * @date 23/05/2021
 * @brief this class is for quaternion compass implementation
 *
 * Here typically goes a more extensive explanation of the module 
 */

#ifndef MPU9250_LIB_H
#define MPU9250_LIB_H

/*-----------------------------------*
 * INCLUDE FILES
 *-----------------------------------*/
#include <Wire.h>
#include <Arduino.h>
#include "mpu9250_defs.h"
/*-----------------------------------*
 * PUBLIC DEFINES
 *-----------------------------------*/
/*-----------------------------------*
 * PUBLIC MACROS
 *-----------------------------------*/
/* None */

/*-----------------------------------*
 * PUBLIC TYPEDEFS
 *-----------------------------------*/
/* None */

/*-----------------------------------*
 * PUBLIC VARIABLE DECLARATIONS
 *-----------------------------------*/
/* None */

/*-----------------------------------*
 * PUBLIC FUNCTION PROTOTYPES
 *-----------------------------------*/

class MPU9250
{
public:
    /*******************
     * CONSTRUCTOR & DESTRUCTOR METHODS
    *******************/
    /**
     * @name MPU9250
     * @brief MPU9250: constructor
    */
    MPU9250();
    ~MPU9250();

    /*******************
     * PUBLIC METHODS
    *******************/
    /**
     * @name setTarget
     * @brief setTarget: Set latitude and longitude target
     * @param [in] double targetLatitude: set desired latitude target
     * @param [in] double targetLatitude: set desired logitude target
     * @retval None
     */
    void setTarget(double targetLatitude, double targetLongitude);

    /* Get magnetometer resolution */
    void getMres();
    /* Get gyro resolution */
    void getGres();
    /* Get accelerometer resolution */
    void getAres();

    void getQuaternion(float &quaternion[4]);

    void getYawPitchRoll(float &yaw, float &pitch, float &roll);

    void getTemperature(float &temperature);
    void getHeading(float &heading);


    /* refresh data and compute*/
    bool update();

    /*
    Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
    of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
    */
    void calibrateMPU9250(float * dest1, float * dest2);
    /*
    Function which accumulates magnetometer data after device initialization.
    It calculates the bias and scale in the x, y, and z axes.
    */
    void magCalMPU9250(float * bias_dest, float * scale_dest);
    /* Accelerometer and gyroscope self test; check calibration wrt factory settings */
    void MPU9250SelfTest(float * destination); // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
  

    
    


    /*******************
     * PUBLIC VARIABLES
    *******************/
    // None


private:
    /*******************
     * PRIVATE METHODS
    *******************/
    /* Read accelerometer registers */
    void readAccelData(short * destination);
    /* Read gyro registers */
    void readGyroData(short * destination);
    /* Read magnetometer registers */
    void readMagData(short * destination);
    /* Read temperature */
    short readTempData();
    /* Initialize the AK8963 magnetometer */
    void initAK8963(float * destination);
    /* Initialize the MPU9250|MPU6050 chipset */
    void initMPU9250();

    /* Get current MPU-9250 register values */
    void refresh_data();

    /* Send current MPU-9250 register values to Mahony quaternion filter */
    void calc_quaternion();

    void writeByte(byte address, byte subAddress, byte data);
    byte readByte(byte address, byte subAddress);
    void readBytes(byte address, byte subAddress, byte count, byte * dest);

    /*
    Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
    (see http://www.x-io.co.uk/category/open-source/ for examples and more details);
    which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
    device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
    The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
    but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
    */
    void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

    /*
    Similar to Madgwick scheme but uses proportional and integral filtering
    on the error between estimated reference vectors and measured ones.
    */
    void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);



    /*******************
     * PRIVATE VARIABLES
    *******************/
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
    
    // Magnetic calibration offset
    float Mag_x_offset = 366.695;
    float Mag_y_offset = 376.02;
    float Mag_z_offset = -397.455;
    float Mag_x_scale = 1.0184643;
    float Mag_y_scale = 0.97450244;
    float Mag_z_scale = 1.0081002;


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
    float heading;

    float deltat = 0.0f, sum = 0.0f;                    // integration interval for both filter schemes
    unsigned long lastUpdate = 0, firstUpdate = 0;      // used to calculate integration interval
    unsigned long Now = 0;                              // used to calculate integration interval

    float ax, ay, az, gx, gy, gz, mx, my, mz;           // variables to hold latest sensor data values
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};              // vector to hold quaternion
    float eInt[3] = {0.0f, 0.0f, 0.0f};                 // vector to hold integral error for Mahony method

    // ----- Specify sensor full scale
    byte Gscale = GFS_250DPS;
    byte Ascale = AFS_2G;
    byte Mscale = MFS_14BITS;                           // Choose either 14-bit or 16-bit magnetometer resolution (AK8963=14-bits)
    byte Mmode = 0x02;                                  // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
    float aRes, gRes, mRes;                             // scale resolutions per LSB for the sensors

}; /* MPU9250 */


#endif /* MPU9250_LIB_H */

/****************************************************************************
 ****************************************************************************/
