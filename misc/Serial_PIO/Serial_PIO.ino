SerialPIO mySerial(D0,D1);

void setup() {
  Serial.begin(115200);
  mySerial.begin(115200);
  }

void loop() {
  if (Serial.available()) {
    char c = (char)Serial.read();
    mySerial.write(c);
  }

  if (mySerial.available()) {
    char c = (char)mySerial.read();
    Serial.write(c);
  }
}
