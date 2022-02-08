#include<Servo.h>

//int pin = -1;
Servo esc;
int data;

void setup() {
  Serial.begin(9600);
  esc.attach(10);
}

void loop() {
  if (Serial.available()) {
//    unsigned char data = Serial.read();
//    if (pin != data) {
//      esc.attach(data);
//      pin = data;
//    }
    data = Serial.read();
    esc.write(data);
  }
}
