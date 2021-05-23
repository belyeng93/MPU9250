/**
 * @file mpu9250_lib.h
 * @author Emanuele Belia
 * @date 23/05/2021
 * @brief this class is for quaternion compass implementation
 *
 * Here typically goes a more extensive explanation of the module 
 */


/*-----------------------------------*
* INCLUDE FILES
*-----------------------------------*/
#include "mpu9250_lib.h"
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

/*******************
 * CONSTRUCTOR & DESTRUCTOR METHODS
*******************/
/**
 * @name MPU9250
 * @brief MPU9250: constructor
*/
MPU9250::MPU9250()
{
    Wire.begin();
    Wire.setClock(400000);                            // 400 kbit/sec I2C speed

    while (!Serial);// required for Feather M4 Express<
    Serial.begin(115200);

    
    // ----- Display title
    Serial.println(F("MPU-9250 Quaternion Compass"));
    Serial.println("");
    delay(2000);

    // ----- Level surface message
    Serial.println(F("Place the compass on a level surface"));
    Serial.println("");
    delay(2000);

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
    

}
~MPU9250::MPU9250(){}

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
/* Get magnetometer resolution */
void MPU9250::getMres()
{
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

/* Get gyro resolution */
void MPU9250::getGres()
{
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

/* Get accelerometer resolution */
void MPU9250::getAres()
{
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

/* refresh data and compute QUATERNION*/
bool MPU9250::update()
{
    refresh_data();                              // This must be done each time through the loop
    calc_quaternion();                           // This must be done each time through the loop

    pitch = asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = -atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);

    // ----- convert to degrees
    //TODO: flag for select if you want radians or degrees
    pitch *= RAD_TO_DEG;
    roll *= RAD_TO_DEG;
    yaw *= RAD_TO_DEG;

    // ----- calculate the heading
    /*
        The yaw and compass heading (after the next two lines) track each other 100%
    */
    this->heading = yaw;
    if (this->heading < 0) this->heading += 360.0;                        // Yaw goes negative between 180 amd 360 degrees
    if (True_North == true) this->heading += Declination;           // Calculate True North
    if (this->heading < 0) this->heading += 360.0;                        // Allow for under|overflow
    if (this->heading >= 360) this->heading -= 360.0;

    // ----- send the results to the Serial Monitor
    Serial.print("        Pitch ");
    print_number((short)pitch);
    Serial.print("        Roll ");
    print_number((short)roll);
    Serial.print("        Heading ");
    print_number((short)this->heading);

    // ----- Print temperature in degrees Centigrade
    tempCount = readTempData();                               // Read the temperature registers
    temperature = ((float) tempCount) / 333.87 + 21.0;        // Temp in degrees C
    Serial.print("        Temp(C) ");
    Serial.print(temperature, 1);

}

/*
Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
*/
void MPU9250::calibrateMPU9250(float * dest1, float * dest2)
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

/*
Function which accumulates magnetometer data after device initialization.
It calculates the bias and scale in the x, y, and z axes.
*/
void MPU9250::magCalMPU9250(float * bias_dest, float * scale_dest)
{
    unsigned short ii = 0, sample_count = 0;
    short mag_max[3]  = { -32768, -32768, -32768}:
    short mag_min[3]  = {32767, 32767, 32767}:
    short mag_temp[3] = {0, 0, 0};

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
}

/* Accelerometer and gyroscope self test check calibration wrt factory settings */
// Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
void MPU9250::MPU9250SelfTest(float * destination)
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

void MPU9250::writeByte(byte address, byte subAddress, byte data)
{
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.write(data);                 // Put data in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer
}

byte MPU9250::readByte(byte address, byte subAddress)
{
    byte data; // `data` will store the register data
    Wire.beginTransmission(address);         // Initialize the Tx buffer
    Wire.write(subAddress);                  // Put slave register address in Tx buffer
    Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(address, (byte) 1);  // Read one byte from slave register address
    data = Wire.read();                      // Fill Rx buffer with result
    return data;     
}

void MPU9250::readBytes(byte address, byte subAddress, byte count, byte * dest)
{
    Wire.beginTransmission(address);   // Initialize the Tx buffer
    Wire.write(subAddress);            // Put slave register address in Tx buffer
    Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
    byte i = 0;
    Wire.requestFrom(address, count);  // Read bytes from slave register address
    while (Wire.available()) 
    {
        dest[i++] = Wire.read();
    } // Put read results in the Rx buffer
}

/*
Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor MPU9250::arrays"
(see http://www.x-io.co.uk/category/open-source/ for examples and more details)
which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
*/
void MPU9250::MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
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

/*
Similar to Madgwick scheme but uses proportional and integral filtering
on the error between estimated reference vectors and measured ones.
*/
void MPU9250::MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
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


/*******************
 * PUBLIC VARIABLES
*******************/

/*******************
 * PRIVATE METHODS
*******************/
/* Read accelerometer registers */
void MPU9250::readAccelData(short * destination)
{
    byte rawData[6];  // x/y/z accel register data stored here
    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
    destination[0] = ((short)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((short)rawData[2] << 8) | rawData[3] ;
    destination[2] = ((short)rawData[4] << 8) | rawData[5] ;

}
/* Read gyro registers */
void MPU9250::readGyroData(short * destination)
{
    byte rawData[6];  // x/y/z gyro register data stored here
    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    destination[0] = ((short)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((short)rawData[2] << 8) | rawData[3] ;
    destination[2] = ((short)rawData[4] << 8) | rawData[5] ;

}
/* Read magnetometer registers */
void MPU9250::readMagData(short * destination)
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

/* Read temperature */
short MPU9250::readTempData()
{
    /* Read temperature */
    short readTempData()
    {
        byte rawData[2];  // x/y/z gyro register data stored here
        readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
        return ((short)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
    }

}

/* Initialize the AK8963 magnetometer */
void MPU9250::initAK8963(float * destination)
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

/* Initialize the MPU9250|MPU6050 chipset */
void MPU9250::initMPU9250()
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

/* Get current MPU-9250 register values */
void MPU9250::refresh_data()
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

        // ----- Gyro calculations
        readGyroData(gyroCount);                            // Read the gyro registers
        getGres();                                          // Get gyro resolution

        // ----- Calculate the gyro value into actual degrees per second
        gx = (float)gyroCount[0] * gRes; // get actual gyro value, this depends on scale being set
        gy = (float)gyroCount[1] * gRes;
        gz = (float)gyroCount[2] * gRes;

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
        mz = ((float)magCount[2] * mRes * magCalibration[2] - magBias[2]) * magScale[2];    // (rawMagZ*ASAZ*0.6 - magOffsetZ)*scalefactor
    }

}

/* Send current MPU-9250 register values to Mahony quaternion filter */
void MPU9250::calc_quaternion()
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

void MPU9250::getQuaternion(float &quaternion[4])
{
    memcpy ( quaternion, &q, sizeof(q) );
}

void MPU9250::getYawPitchRoll(float &yaw, float &pitch, float &roll)
{
    yaw = this->yaw:
    pitch = this->pitch:
    roll = this->roll:
}

void MPU9250::getTemperature(float &temperature)
{
    temperature = this->temperature;
}

void MPU9250::getHeading(float &heading)
{
    heading = this->heading;
}

/*******************
 * PRIVATE VARIABLES
*******************/


/****************************************************************************
 ****************************************************************************/
