#include "MPU9250.h"

MPU9250 mpu;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    MPU9250Setting setting;
    setting.accel_fs_sel = ACCEL_FS_SEL::A2G;
    setting.gyro_fs_sel = GYRO_FS_SEL::G250DPS;
    setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
    setting.gyro_fchoice = 0x00;
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
    setting.accel_fchoice = 0x00;
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

    if (!mpu.setup(0x68, setting)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
    // Set gyro Calibrated Parameters
    // mpu.setGyroBias(0.836, 0.157, 0.923);
    // calibrate anytime you want to
    
    //Serial.println("Accel Gyro calibration will start in 5sec.");
    //Serial.println("Please leave the device still on the flat plane.");
    mpu.calibrateAccelGyro();
    //print_calibration();
    //delay(2000);

    float gx = 0.796582275390625;
    float gy = 0.1499676513671875;
    float gz = 0.948497314453125; //0.93 //92
    mpu.setGyroBias(gx * (float)MPU9250::CALIB_GYRO_SENSITIVITY, gy * (float)MPU9250::CALIB_GYRO_SENSITIVITY, gz * (float)MPU9250::CALIB_GYRO_SENSITIVITY);

    mpu.setMagneticDeclination(3.633);
    mpu.setMagBias(137.41, 189.01, -890.54);
    mpu.setMagScale(0.73, 0.72, 4.40);

    //print_calibration();
    //delay(2000);

    
}

void loop() {
    if(mpu.update())
    {
        print_roll_pitch_yaw();
    }
}
void print_gyro() {
    Serial.print("GyroX, GyroY, GyroZ: ");
    Serial.print(mpu.getGyroX(), 2);
    Serial.print(", ");
    Serial.print(mpu.getGyroY(), 2);
    Serial.print(", ");
    Serial.println(mpu.getGyroZ(), 2);
}

void print_roll_pitch_yaw() {
    //Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu.getYaw() + 180, 0);
    Serial.print(",");
    Serial.print(mpu.getPitch()+ 180, 0);
    Serial.print(",");
    Serial.println(mpu.getRoll()+ 180, 0);
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
