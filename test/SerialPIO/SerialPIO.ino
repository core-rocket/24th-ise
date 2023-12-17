SerialPIO mySerial(D0, D1, 128); //txpin, rxpin, fifosize

void setup() {
  Serial.begin(115200);
  mySerial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
      mySerial.write(c);
  }

  if (mySerial.available()) {
    char c = mySerial.read();
    Serial.write(c);
  }
}
