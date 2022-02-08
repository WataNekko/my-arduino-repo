#define ledPin 9
#define LED_CMD 'l'
#define TEST_CMD 't'

char data;

void setup() {
  pinMode(ledPin,OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    data = Serial.read();
    switch (data) {
      case LED_CMD:
        data = Serial.read();
        analogWrite(ledPin, data);
        break;
      case TEST_CMD:
        Serial.write('$');
    }
  }
}
