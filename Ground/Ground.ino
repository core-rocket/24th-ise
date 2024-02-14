#define Serial_ES920 Serial1
bool need_uplink = false;
String uplink_string = "";
typedef enum {
  SENDING,
  FREE
} MODE;
MODE mode = FREE;
uint32_t send_millis = millis();
uint32_t timeout_ms = 3000;

void setup() {

  //Serial.setFIFOSize(64);
  Serial.begin(115200);

  Serial_ES920.begin(115200);
  while (Serial_ES920.available()) {
    Serial_ES920.read();
  }
}

void loop() {

  while (Serial.available()) {
    uplink_string = Serial.readStringUntil('\n');
    uplink_string.trim();
    need_uplink = true;
  }

  if ((mode == FREE) && Serial_ES920.available()) {
    String downlink = Serial_ES920.readStringUntil('\n');
    // Serial.print("raw:");
    // Serial.println(downlink);
    downlink.trim();

    char rssi_char[] = "FFFF";
    downlink.substring(0, 4).toCharArray(rssi_char, 5);
    int16_t rssi_long = rssi(rssi_char);

    String downlink_ASCII = downlink.substring(4, downlink.length());

    Serial.print(rssi_long);
    Serial.print(",");
    Serial.println(downlink_ASCII);

    if (need_uplink) {
      delay(50);
      need_uplink = false;
      Serial_ES920.println(uplink_string);
      mode = SENDING;
      send_millis = millis();
      Serial.print("uplink ");
      Serial.println(uplink_string);
    }
  }

  if (mode == SENDING) {
    if (Serial_ES920.available()) {
      String response = Serial_ES920.readStringUntil('\n');
      response.trim();
      mode = FREE;
      Serial.print("response from ES920LR:");
      Serial.println(response);
    }
    if (millis() - send_millis > timeout_ms) {
      mode = FREE;
      Serial.print("timeout");
    }
  }
}

int16_t rssi(char *_rssi_char) {
  char *e;
  return strtol(_rssi_char, &e, 16) - 65536;
}

// GROUND
// configuration setting is below.
// -------------------------------------
// Node                        : EndDevice
// Band Width                  : 125kHz
// Spreading Factor            : 11
// Effective Bitrate           : 537bps
// Channel                     : 2
// PAN ID                      : 0001
// Own Node ID                 : 0001
// Destination ID              : 0000
// Acknowledge                 : OFF
// Retry count                 : 3
// Transfer Mode               : Payload
// Receive Node ID information : OFF
// RSSI information            : ON
// Config/Operation            : Operation
// UART baudrate               : 115200
// Sleep Mode                  : No Sleep
// Sleep Time                  : 50
// Output Power                : 13dBm
// Format                      : ASCII
// Send Time                   : 0
// Send Data                   :
// AES Key                     : 00000000000000000000000000000000
// RF Mode                     : TxRx
// Protocol Type               : Private LoRa
// Rx Boosted                  : ON
