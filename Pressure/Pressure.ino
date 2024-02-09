#include <SPI.h>
#include <string.h>
#include <CCP_MCP2515.h>

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
}

void loop() {
  // CCP.float_to_device(CCP_ID, 123.45);

  if (!digitalRead(CAN_INT))  // データ受信確認
  {
    CCP.read_device();
    // if (CCP.id == ~) {
    // }
  }
}
