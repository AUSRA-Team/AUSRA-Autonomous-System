#include "MPU9250.h"

MPU9250 mpu;

void setup() {
    Serial.begin(115200);
    Wire.begin(17, 18); // SDA=17, SCL=18
    
    delay(2000);
    
    if (!mpu.setup(0x68)) {  // Default I2C address
        Serial.println("MPU9250 connection failed");
        while (1) {
            delay(1000);
        }
    }
    
    // Calibrate (keep IMU still during this)
    Serial.println("Calibrating... Keep IMU still!");
    delay(2000);
    mpu.calibrateAccelGyro();
    mpu.calibrateMag();
    Serial.println("Calibration complete!");
}

void loop() {
    if (mpu.update()) {
        // Send data in CSV format: ax,ay,az,gx,gy,gz,mx,my,mz
        Serial.print(mpu.getAccX());
        Serial.print(",");
        Serial.print(mpu.getAccY());
        Serial.print(",");
        Serial.print(mpu.getAccZ());
        Serial.print(",");
        Serial.print(mpu.getGyroX());
        Serial.print(",");
        Serial.print(mpu.getGyroY());
        Serial.print(",");
        Serial.print(mpu.getGyroZ());
        Serial.print(",");
        Serial.print(mpu.getMagX());
        Serial.print(",");
        Serial.print(mpu.getMagY());
        Serial.print(",");
        Serial.println(mpu.getMagZ());
    }
    delay(10); // 100 Hz update rate
}
