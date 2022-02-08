uint8_t receiveBuffer[5];
uint8_t serialCount = 0; // position when reading serial packet
uint8_t inByte;
uint8_t endBytePos;

unsigned long lastRXTime, currTime; // for receive timeout check
#define RECEIVE_TIMEOUT 30000 // (microsecond)

uint8_t transmitBuffer[7] = {'<', 0, 0, 0, 0, 0, '>'};

void processPacketData()  {
    switch (receiveBuffer[0]) {
        case 't':
            transmitBuffer[1] = 't';
            transmitBuffer[2] = receiveBuffer[1];
            transmitBuffer[3] = receiveBuffer[2];
            transmitBuffer[4] = '>';
            Serial.write(transmitBuffer, 5);
            break;
        case 's':
            transmitBuffer[1] = 's';
            transmitBuffer[2] = receiveBuffer[1];
            transmitBuffer[3] = receiveBuffer[2];
            transmitBuffer[4] = receiveBuffer[3];
            transmitBuffer[5] = receiveBuffer[4];
            Serial.write(transmitBuffer, 7);
            break;
        case 'T':
            transmitBuffer[1] = 't';
            *((uint16_t*)(&transmitBuffer[2])) = *((uint16_t*)(receiveBuffer + 1)) + 1;
            transmitBuffer[4] = '>';
            Serial.write(transmitBuffer, 5);
            break;
        case 'S':
            transmitBuffer[1] = 's';
            *((float*)(&transmitBuffer[2])) = *((float*)(receiveBuffer + 1)) + 1;
            Serial.write(transmitBuffer, 7);
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
        // Packet structure: ['<', code, (byte1, byte2, (byte3, byte4,)) '>']
        if (serialCount == 0) {
          
            if (inByte == '<'){
               serialCount = 1;
            }
            
        } else if (serialCount == 1) {
            // Check for valid code and determine packet length
            switch (inByte) {
                case 'E': case '1': case '0':
                    endBytePos = 2;
                    break;
                case 't': case 'T':
                    endBytePos = 4;
                    break;
                case 's': case 'p': case 'i': case 'd': case 'S':
                    endBytePos = 6;
                    break;
                default:
                    serialCount = 0;
                    continue;
            }

            receiveBuffer[0] = inByte;
            serialCount = 2;
            
        } else if (serialCount == endBytePos) {
            
            if (inByte == '>') processPacketData();
            serialCount = 0;
            
        } else {
            receiveBuffer[(serialCount++) - 1] = inByte;
        }

        // If receiving data packet
        if (serialCount > 0)
            // Update last receive time
            lastRXTime = currTime;
    }
}

void setup() {
    Serial.begin(9600);
//    Serial.begin(115200);
}

void loop() {
    checkSerialData();
    
    currTime = micros();
}
