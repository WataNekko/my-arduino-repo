#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"



// Motor variables
Servo motor[2];
uint16_t throttle = 1000;
int motorVal[2];
#define MOTOR_PIN_1 9
#define MOTOR_PIN_2 10

// Time variables
unsigned long currTime, prevTime;
float elapsedTime;



// PID variables
float setpoint;

float error, prevError;
float PID;
float pid_p, pid_i, pid_d;
#define INTEGRAL_BOUND 3

bool pidState = false;

//////////PID GAINS//////////
float kP = 0.0;
float kI = 0.0;
float kD = 0.0;
/////////////////////////////



///////////MPU6050///////////
MPU6050 mpu;

uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

/////////////////////////////



/////////DATA PACKET/////////

// Receive packet
uint8_t receiveBuffer[5];
uint8_t serialCount = 0; // position when reading serial packet
uint8_t inByte;
uint8_t endBytePos;

unsigned long lastRXTime; // for receive timeout check
#define RECEIVE_TIMEOUT 30000 // (microsecond)

// Transmit packet
uint8_t transmitBuffer[23] = {'<',0,  0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,0,  '>'};

/////////////////////////////





////////// Transmit serial data /////////
void sendError(uint8_t errorCode) {
    // Drone error: <e(1 byte)>
    //     errorCode
    transmitBuffer[1] = 'e';
    transmitBuffer[2] = errorCode;
    transmitBuffer[3] = '>';
    Serial.write(transmitBuffer, 4);
}

void sendSyncDataPacket() {
    // Sync data packet: <S(19 bytes)>
    //     pidState (1) - throttle (2) - setpoint (4) - kP (4) - kI (4) - kD (4)
    transmitBuffer[1] = 'S';
    transmitBuffer[2] = (pidState ? 1 : 0);
    *((uint16_t*)(&transmitBuffer[3])) = throttle;
    *((float*)(&transmitBuffer[5])) = setpoint;
    *((float*)(&transmitBuffer[9])) = kP;
    *((float*)(&transmitBuffer[13])) = kI;
    *((float*)(&transmitBuffer[17])) = kD;
    transmitBuffer[21] = '>';
    Serial.write(transmitBuffer, 22);
}

void sendDroneValues() {
    // Drone values data packet: <V(20 bytes)>
    //     yaw (4) - pitch (4) - roll (4) - motorVal[0] (2) - motorVal[1] (2) - PID (4)s
    transmitBuffer[1] = 'V';
    *((float*)(&transmitBuffer[2])) = ypr[0];  // yaw
    *((float*)(&transmitBuffer[6])) = ypr[1];  // pitch
    *((float*)(&transmitBuffer[10])) = ypr[2]; // roll
    *((uint16_t*)(&transmitBuffer[14])) = (uint16_t) motorVal[0];
    *((uint16_t*)(&transmitBuffer[16])) = (uint16_t) motorVal[1];
    *((float*)(&transmitBuffer[18])) = PID;
    transmitBuffer[22] = '>';
    Serial.write(transmitBuffer, 23);
}
/////////////////////////////////////////



////////// Receive serial data //////////
void processPacketData()  {
    // Process packet data based on code
    
    switch (receiveBuffer[0]) {
        // Request sync data packet
        case 'S':
            sendSyncDataPacket();
            break;
        // Set PID control ON
        case '1':
            pidState = true;
            break;
        // Set PID control OFF
        case '0':
            pidState = false;
            break;
            
        // Set throttle value
        case 't':
            throttle = *((uint16_t*)(&receiveBuffer[1]));
            break;
            
        // Set setpoint value
        case 's':
            setpoint = *((float*)(&receiveBuffer[1]));
            break;
        // Set Proportional gain (kP)
        case 'p':
            kP = *((float*)(&receiveBuffer[1]));
            break;
        // Set Integral gain (kI)
        case 'i':
            kI = *((float*)(&receiveBuffer[1]));
            break;
        // Set Derivative gain (kD)
        case 'd':
            kD = *((float*)(&receiveBuffer[1]));
            break;
    }
    
}

void checkSerialData() {
    // Check for receive timeout
    if ((serialCount > 0) && (currTime - lastRXTime > RECEIVE_TIMEOUT)) {
        // If receiving data packet and timed out
        serialCount = 0; // reset
    }
    
    while (Serial.available() > 0) {
        inByte = Serial.read();

        // Process byte based on serialCount
        // Packet structure: {'<', code, [valueBytes,] '>'}
        if (serialCount == 0) {
            // Start byte
            if (inByte == '<'){
               serialCount = 1;
            }
            
        } else if (serialCount == 1) {
            // Code byte
            // Check for valid code and determine packet length
            switch (inByte) {
                // Single code (value: 0 byte)
                case 'S': case '1': case '0':
                    endBytePos = 2;
                    break;
                // UInt16 value (value: 2 bytes)
                case 't':
                    endBytePos = 4;
                    break;
                // Float value (value: 4 bytes)
                case 's': case 'p': case 'i': case 'd':
                    endBytePos = 6;
                    break;
                // Undefined code
                default:
                    serialCount = 0;
                    continue;
            }

            receiveBuffer[0] = inByte;
            serialCount = 2;
            
        } else if (serialCount == endBytePos) {
            // End byte
            if (inByte == '>') processPacketData();
            serialCount = 0;
            
        } else {
            // Value bytes
            receiveBuffer[(serialCount++) - 1] = inByte;
        }

        // If receiving data packet
        if (serialCount > 0)
            // Update last receive time
            lastRXTime = currTime;
    }
}
/////////////////////////////////////////



//////////// SETUP ////////////
void mpuSetup() {
    Wire.begin();
    Wire.setClock(400000);
    Wire.setWireTimeout(3000, true);
    
    mpu.initialize();
    
    if (mpu.dmpInitialize() != 0) {
        while (true) { // Hang if failed to initialize dmp
            sendError(1); // : DMP failed to initialize
            delay(5000);
        }
    }
    
    mpu.CalibrateAccel();
    mpu.CalibrateGyro();
    
    mpu.setDMPEnabled(true);
}

void setup() {
    Serial.begin(115200);
    
    mpuSetup();

    // Motors setup
    motor[0].attach(MOTOR_PIN_1);
    motor[1].attach(MOTOR_PIN_2);
    
    // Write minimum throttle value for start up (1000)
    motor[0].writeMicroseconds(1000);
    motor[1].writeMicroseconds(1000);
}

///////////////////////////////



void updateFeedback() {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        // Update ypr
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // Convert radians to degrees
        ypr[0] *= RAD_TO_DEG;
        ypr[1] *= RAD_TO_DEG;
        ypr[2] *= RAD_TO_DEG;
    }
}

void updatePIDValue() {
    if (pidState == false) { // PID control is OFF
        // Keep time and error vars up-to-date
        // for the next time PID control is ON
        currTime = micros();
        error = setpoint - ypr[2];
        // -----------------------------------
        
        PID = 0; // PID = 0 if PID control is OFF
        return;
    }
    
    // Update time
    prevTime = currTime;
    currTime = micros();
    delay(1);
    elapsedTime = (currTime - prevTime) / 1000000.0;
  
    ////////////// PID //////////////
    prevError = error;
    error = setpoint - ypr[2]; // feedback is roll (ypr[2])
  
    // Proportional
    pid_p = kP * error;
  
    // Integral (Only if the error is small)
    if ((-INTEGRAL_BOUND <= error) && (error <= INTEGRAL_BOUND))
        pid_i += kI * error;
    else
        pid_i = 0;
  
    // Derivative
    pid_d = kD * ((error - prevError) / elapsedTime);
  
    //-----------------------------//
    PID = pid_p + pid_i + pid_d;
}



////////// MAIN LOOP //////////

void loop() {
    checkSerialData();
    // Check for (and process) incoming serial data
    // at the start of every loop
    
    updateFeedback(); // Update real angles (ypr)
    // Get data from MPU every loop even if PID
    // control is OFF, so that the fifo buffer
    // does not overflow
    
    updatePIDValue();
    // Calculate the PID value based on setpoint,
    // feedback and the PID gains (0 if PID control
    // is OFF)
    
    //////// Write to motors ////////
    // Adjust each motor values accordingly
    motorVal[0] = throttle - (int)PID;
    motorVal[1] = throttle + (int)PID;
    
    // Constrain values between 1000 and 2000
    for (uint8_t i = 0; i < 2; i++) {
        if (motorVal[i] < 1000)
            motorVal[i] = 1000;
        else if (motorVal[i] > 2000)
            motorVal[i] = 2000;
    }
    
    // Write to motors
    motor[0].writeMicroseconds(motorVal[0]);
    motor[1].writeMicroseconds(motorVal[1]);

    // Send drone values data packet to serial
    // at the end of every loop
    sendDroneValues();
}
