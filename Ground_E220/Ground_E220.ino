#include <E220.h>

#define SEND_PERIOD_MS 1000

E220 e220(Serial1, 0xFF, 0xFF, 0x0A);  //TARGETADRESS=0xFFFF,CHANNEL=0x0A=10ch=ARIB 34-35

/*E220configuration
- UARTbaudrate:115200bps
- bandwith: 250kHz//審査書の値なので運営からの指示以外変更禁止
- channel: 0x0A(ARIB 34-35)//審査書の値なので運営からの指示以外変更禁止
- target address: 0xFFFF(broradcast)
- power: 13dBm
- SF: TBD
*/

/*
------configuration------
Address(HEX):00
UARTBaudrate:115200bps
SF:11
BW:250kHz
SubpacketLength:200bytes
RSSINoise:Not Available
TxPower:0dBm
Channel:10
RSSIByte:Available
TxMethod:Transparent mode
WORCycle:500ms
-------------------------
*/

union unionfloat {
  float f;
  byte b[4];
};
union unionuint32 {
  uint32_t i;
  byte b[4];
};


void setup() {
  Serial1.setFIFOSize(512);  //E220のサブパケ200byteより大きいサイズにする
  Serial.begin(115200);
  Serial1.begin(115200);  //E220のUART
}

void loop() {
  static byte rx_payload[199] = { 0 };
  int rssi = 0;
  int Rxlength = 0;
  Rxlength = e220.ReceiveDataVariebleLength(rx_payload, 47, &rssi);  //surface6_pressureまでの長さ
  unionuint32 mcutime_ms;
  byte nose_adc_raw[6] = { 0x00 };
  unionfloat nose_temperature;
  unionfloat nose_barometic_presure;
  unionfloat nose_voltage;
  unionfloat surface1_pressure;
  unionfloat surface2_pressure;
  unionfloat surface3_pressure;
  unionfloat surface4_pressure;
  unionfloat surface5_pressure;
  unionfloat surface6_pressure;
  byte status = 0x00;
  for (int i = 0; i < 4; i++) {
    mcutime_ms.b[i] = rx_payload[i];
  }
  status = rx_payload[4];
  for (int i = 0; i < 6; i++) {
    nose_adc_raw[i] = rx_payload[i + 5];
  }
  for (int i = 0; i < 4; i++) {
    nose_temperature.b[i] = rx_payload[i + 11];
  }
  for (int i = 0; i < 4; i++) {
    nose_barometic_presure.b[i] = rx_payload[i + 15];
  }
  for (int i = 0; i < 4; i++) {
    nose_voltage.b[i] = rx_payload[i + 19];
  }
  for (int i = 0; i < 4; i++) {
    surface1_pressure.b[i] = rx_payload[i + 23];
  }
  for (int i = 0; i < 4; i++) {
    surface2_pressure.b[i] = rx_payload[i + 27];
  }
  for (int i = 0; i < 4; i++) {
    surface3_pressure.b[i] = rx_payload[i + 31];
  }
  for (int i = 0; i < 4; i++) {
    surface4_pressure.b[i] = rx_payload[i + 35];
  }
  for (int i = 0; i < 4; i++) {
    surface5_pressure.b[i] = rx_payload[i + 39];
  }
  for (int i = 0; i < 4; i++) {
    surface6_pressure.b[i] = rx_payload[i + 43];
  }

  if (Rxlength > 0) {
    Serial.print("RSSI[dBm]:");
    Serial.print(rssi);
    Serial.print(",rocket_time_ms:");
    Serial.print(mcutime_ms.i);
    Serial.print(",status:");
    StatusSerialPrint(status);
    Serial.print(",");
    for (int i = 0; i < 6; i++) {
      Serial.print(nose_adc_raw[i]);
      Serial.print(",");
    }
    Serial.print(nose_temperature.f, 6);
    Serial.print(",");
    Serial.print(nose_barometic_presure.f, 6);
    Serial.print(",");
    Serial.print(nose_voltage.f, 6);
    Serial.print(",");
    Serial.print(surface1_pressure.f, 6);
    Serial.print(",");
    Serial.print(surface2_pressure.f, 6);
    Serial.print(",");
    Serial.print(surface3_pressure.f, 6);
    Serial.print(",");
    Serial.print(surface4_pressure.f, 6);
    Serial.print(",");
    Serial.print(surface5_pressure.f, 6);
    Serial.print(",");
    Serial.println(surface6_pressure.f, 6);
  }
}

void StatusSerialPrint(byte _status) {
  for (int i = 0; i < 8; i++) {
    if ((_status >> 7 - i) & 0x01) {
      Serial.print("1");
    } else {
      Serial.print("0");
    }
  }
}