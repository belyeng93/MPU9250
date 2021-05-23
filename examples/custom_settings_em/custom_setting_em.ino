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

    // Set Mag Calibrated Parameters 
    mpu.setMagneticDeclination(3.633);
    mpu.setMagBias(135.41, 188.01, -881.54);
    mpu.setMagScale(0.71, 0.72, 4.65);

    // Set Acc Calibrated Parameters
    // 0.99980378,-0.00374048;0.99725472,-0.0189183;0.9860536,-0.01127061
    mpu.setAccBias(-0.00374048, -0.0189183, -0.01127061);

    // Set gyro Calibrated Parameters
    float gx = 0.796582275390625;
    float gy = 0.1499676513671875;
    float gz = 0.948497314453125; //0.93 //92
    mpu.setGyroBias(gx * (float)MPU9250::CALIB_GYRO_SENSITIVITY, gy * (float)MPU9250::CALIB_GYRO_SENSITIVITY, gz * (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    
}

void loop() {
    if (mpu.update()) {
        print_roll_pitch_yaw();
        delay(100);
    }
}

void print_roll_pitch_yaw() {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu.getYaw(), 0);
    Serial.print(", ");
    Serial.print(mpu.getPitch(), 0);
    Serial.print(", ");
    Serial.println(mpu.getRoll(), 0);
}
