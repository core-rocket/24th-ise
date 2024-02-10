#define Serial_ES920 Serial2
const pin_size_t ES920_TX = 8;
const pin_size_t ES920_RX = 9;

void setup() {
  // put your setup code here, to run once:
  //Serial.setFIFOSize(64);
  Serial.begin(115200);

  Serial_ES920.setTX(ES920_TX);
  Serial_ES920.setRX(ES920_RX);
  Serial_ES920.setFIFOSize(64);
  Serial_ES920.begin(115200);
  while (Serial_ES920.available()) {
    Serial_ES920.read();
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    char c = (char)Serial.read();
    Serial_ES920.write(c);
  }

  if (Serial_ES920.available()) {
    char c = (char)Serial_ES920.read();
    Serial.write(c);
  }
}

// ROCKET
// configuration setting is below.
//   -------------------------------------
//   Node                        : Coordinator
//   Band Width                  : 125kHz
//   Spreading Factor            : 11
//   Effective Bitrate           : 538bps
//   Channel                     : 2
//   PAN ID                      : 0001
//   Own Node ID                 : 0000
//   Destination ID              : FFFF
//   Acknowledge                 : OFF
//   Retry count                 : 3
//   Transfer Mode               : Payload
//   Receive Node ID information : OFF
//   RSSI information            : OFF
//   Config/Operation            : Operation
//   UART baudrate               : 115200
//   Sleep Mode                  : No Sleep
//   Sleep Time                  : 50
//   Output Power                : 13dBm
//   Format                      : ASCII
//   Send Time                   : 0
//   Send Data                   : 
//   AES Key                     : 00000000000000000000000000000000
