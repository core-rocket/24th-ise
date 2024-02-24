#include <SPI.h>
#include <string.h>
#include <CCP_MCP2515.h>
#include <Adafruit_LPS35HW.h>

Adafruit_LPS35HW lps35hw = Adafruit_LPS35HW();

#define CAN_INT D6
#define CAN_CS D7

// CAN
CCP_MCP2515 CCP(CAN_CS, CAN_INT);

void setup() {
  delay(500);
  Serial.begin(115200);
  pinMode(CAN_CS, OUTPUT);
  pinMode(CAN_INT, INPUT);
  digitalWrite(CAN_CS, HIGH);

  // CAN
  CCP.begin();

  while (!lps35hw.begin_I2C(0x5C)) {
  //if (!lps35hw.begin_SPI(LPS_CS)) {
  //if (!lps35hw.begin_SPI(LPS_CS, LPS_SCK, LPS_MISO, LPS_MOSI)) {
    Serial.println("Couldn't find LPS35HW chip");
    delay(10);
  }
  lps35hw.setDataRate(LPS35HW_RATE_75_HZ);
}

void loop() {
  CCP.float_to_device(0x80, lps35hw.readTemperature());
  CCP.float_to_device(0x81, lps35hw.readPressure());
  delay(10);

  if (!digitalRead(CAN_INT))  // データ受信確認
  {
    CCP.read_device();
    // if (CCP.id == ~) {
    // }
  }
}
