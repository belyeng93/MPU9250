

// // ----------------
// //  Max|min values 
// // ----------------
// // xMax: 895.58	xMin: -183.35
// // yMax: 948.83	yMin: -182.74
// // zMax: 156.26	zMin: -924.0

// // ----- NZ offsets & scale-factors (TASK 2 ... 2nd run)

// // float
// // Mag_x_offset = 366.695,
// // Mag_y_offset = 386.56,
// // Mag_z_offset = -380.47,
// // Mag_x_scale = 1.0603912,
// // Mag_y_scale = 0.9851133,
// // Mag_z_scale = 0.9598401;

// float
// Mag_x_offset = 366.695,
// Mag_y_offset = 376.02,
// Mag_z_offset = -397.455,
// Mag_x_scale = 1.0184643,
// Mag_y_scale = 0.97450244,
// Mag_z_scale = 1.0081002;



// // float
// // Mag_x_offset = 0.0,
// // Mag_y_offset = 0.0,
// // Mag_z_offset = 0.0,
// // Mag_x_scale = 1.0,
// // Mag_y_scale = 1.0,
// // Mag_z_scale = 1.0;



// // Mag_x_offset = 137.41,
// // Mag_y_offset = 189.01,
// // Mag_z_offset = -890.54,
// // Mag_x_scale = 0.73,
// // Mag_y_scale = 0.72,
// // Mag_z_scale = 4.40;


// // ----- Processing variables
// char InputChar;                   // incoming characters stored here
// bool LinkEstablished = false;     // receive flag
// String OutputString = "";         // outgoing data string to Processing

// // ----- software timer
// unsigned long Timer1 = 500000L;   // 500mS loop ... used when sending data to to Processing
// unsigned long Stop1;              // Timer1 stops when micros() exceeds this value


// short accelCount[3];                                // Stores the 16-bit signed accelerometer sensor output
// short gyroCount[3];                                 // Stores the 16-bit signed gyro sensor output
// short magCount[3];                                  // Stores the 16-bit signed magnetometer sensor output
// float magCalibration[3] = {0, 0, 0},
//                           magBias[3] = {0, 0, 0},
//                                        magScale[3] = {0, 0, 0};    // Factory mag calibration, mag offset , mag scale-factor
// float gyroBias[3] = {0, 0, 0},
//                     accelBias[3] = {0, 0, 0};        // Bias corrections for gyro and accelerometer
// short tempCount;                                    // temperature raw count output
// float temperature;                                  // Stores the real internal chip temperature in degrees Celsius
// float SelfTest[6];                                  // holds results of gyro and accelerometer self test

// /*
//   There is a tradeoff in the beta parameter between accuracy and response speed.
//   In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
//   However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
//   Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
//   By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
//   I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
//   the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
//   In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
// */

// // ----- global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
// float GyroMeasError = PI * (40.0f / 180.0f);        // gyroscope measurement error in rads/s (start at 40 deg/s)
// float GyroMeasDrift = PI * (0.0f  / 180.0f);        // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)

// // ----- Madgwick free parameters
// float beta = sqrt(3.0f / 4.0f) * GyroMeasError;     // compute beta
// float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;     // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

// // ----- Mahony free parameters
// //#define Kp 2.0f * 5.0f                            // original Kp proportional feedback parameter in Mahony filter and fusion scheme
// #define Kp 40.0f                                    // Kp proportional feedback parameter in Mahony filter and fusion scheme
// #define Ki 0.0f                                     // Ki integral parameter in Mahony filter and fusion scheme

// unsigned long delt_t = 0;                           // used to control display output rate
// unsigned long count = 0, sumCount = 0;              // used to control display output rate
// float pitch, roll, yaw;
// float deltat = 0.0f, sum = 0.0f;                    // integration interval for both filter schemes
// unsigned long lastUpdate = 0, firstUpdate = 0;      // used to calculate integration interval
// unsigned long Now = 0;                              // used to calculate integration interval

// float ax, ay, az, gx, gy, gz, mx, my, mz;           // variables to hold latest sensor data values
// float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};              // vector to hold quaternion
// float eInt[3] = {0.0f, 0.0f, 0.0f};                 // vector to hold integral error for Mahony method

