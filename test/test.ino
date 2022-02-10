void setup()
{
    Serial.begin(9600);
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
    digitalWrite(LED_BUILTIN, 1);
    Serial.println("ON");
    delay(1000);
    digitalWrite(LED_BUILTIN, 0);
    Serial.println("OFF");
    delay(1000);
}
