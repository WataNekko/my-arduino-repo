#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;

// MPU control/status vars
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector



void setup() {
    Wire.begin();
    Wire.setClock(400000);
    Wire.setWireTimeout(3000, true);
    Serial.begin(115200);
    
    mpu.initialize();

    if (mpu.dmpInitialize() != 0) {
      Serial.println("DMP init failed.");
      while (true); // Hangs if fail to initialize dmp
    }
    
    mpu.CalibrateAccel();
    mpu.CalibrateGyro();
    
    mpu.setDMPEnabled(true);
}

void loop() {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        // Get ypr
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        // Get accel
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        
        Serial.print(ypr[0] * RAD_TO_DEG);
        Serial.print("  ");
        Serial.print(ypr[1] * RAD_TO_DEG);
        Serial.print("  ");
        Serial.println(ypr[2] * RAD_TO_DEG);
//        Serial.print("  |  ");
        
//        Serial.print(aaReal.x);
//        Serial.print("  ");
//        Serial.print(aaReal.y);
//        Serial.print("  ");
//        Serial.print(aaReal.z);
//        Serial.print("  |  ");
//        
//        Serial.print(aaWorld.x);
//        Serial.print("  ");
//        Serial.print(aaWorld.y);
//        Serial.print("  ");
//        Serial.println(aaWorld.z);
    }
}
