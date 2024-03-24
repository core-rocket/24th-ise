#include <SPI.h>
#include <string.h>
#include <CCP_MCP2515.h>

#define CAN_CS D0
#define CAN_INT D1
#define FLASH_CS D2
#define MISSION_POWER D3
#define LED_R 17
#define LED_G 16
#define LED_B 25

// E220
#include <E220.h>
#define PAYLOAD_SIZE 55
E220 e220(Serial1, 0xFF, 0xFF, 0x0A);  //TARGETADRESS=0xFFFF,CHANNEL=0x0A
/*E220configuration
- UARTbaudrate:115200bps
- bandwith: 250kHz//審査書の値なので運営からの指示以外変更禁止
- channel: 0x0A(ARIB 34-35)//審査書の値なので運営からの指示以外変更禁止
- target address: 0xFFFF(broradcast)
- power: 13dBm
- SF: 11
*/
union unionfloat {
  float f;
  byte b[4];
};
union unionuint32 {
  uint32_t i;
  byte b[4];
};
unionuint32 mcutime_ms;
unionfloat buf;
uint8_t status_byte = 0x00;
uint8_t tx_payload[199] = { 0 };

// CAN
CCP_MCP2515 CCP(CAN_CS, CAN_INT);

// Flash
#include <W25Q512.h>
#include "Queue.h"
W25Q512 flash(SPI, FLASH_CS);
static uint8_t flash_buf[256];
static size_t flash_index = 0;
static uint32_t flash_addr = 0;

typedef enum {
  SLEEP,
  ACTIVE,
  CLEARING
} Flash_Mode;
Flash_Mode flash_mode = SLEEP;

// Other
char msgString[128];
char str_buf[7];  // 6+\0

float nose_temperature = 0;
float nose_barometic_presure = 0;
float nose_voltage = 0;
float surface1_pressure = 0;
float surface2_pressure = 0;
float surface3_pressure = 0;
float surface4_pressure = 0;
float surface5_pressure = 0;
float surface6_pressure = 0;
float surface7_pressure = 0;
float surface8_pressure = 0;

void setup() {
  delay(500);
  Serial.begin(115200);
  pinMode(CAN_CS, OUTPUT);
  pinMode(CAN_INT, INPUT);
  digitalWrite(CAN_CS, HIGH);

  pinMode(MISSION_POWER, OUTPUT);
  digitalWrite(MISSION_POWER, LOW);

  Serial1.setFIFOSize(512);  //E220のサブパケ200byteより大きいサイズにする
  Serial1.begin(921600);
  while (Serial1.available()) {
    Serial1.read();
  }

  // CAN
  CCP.begin();

  flash.begin();

  reset_tx_payload();

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  digitalWrite(LED_R, HIGH);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, HIGH);
}

void flash_print(char str[]) {

  // Serial.print("flash_addr: ");
  // Serial.println(flash_addr);

  if (flash_mode != ACTIVE) {
    return;
  }

  size_t len = strlen(str);
  for (size_t i = 0; i < len; i++) {
    flash_buf[flash_index] = str[i];
    flash_index++;
    if (flash_index == 256) {
      flash.write(flash_addr, flash_buf, 256, false);
      flash_addr += 256;
      flash_index = 0;
      if (flash_addr == 0x3FFFF00) {
        flash_mode = SLEEP;
      }
    }
  }
}

void flash_dump() {
  uint8_t flash_buf[256];
  size_t flash_index = 0;
  uint32_t flash_addr = 0;

  while (true) {
    flash.read(flash_addr, flash_buf, 256);
    Serial.write(flash_buf, 256);
    flash_addr += 256;
    if (flash_addr == 0x3FFFF00) {
      return;
    }
    while (Serial.available()) {
      String usb_str = Serial.readStringUntil('\n');
      usb_str.trim();

      if (usb_str == "stop") {
        return;
      }
    }
  }
}

void loop() {
  while (Serial.available()) {
    String usb_str = Serial.readStringUntil('\n');
    usb_str.trim();

    if (usb_str == "dump") {
      flash_dump();
    }
  }

  while (Serial1.available()) {
    String data_str = Serial1.readStringUntil('\n');
    static char data_char[256];
    data_str.trim();

    if (data_str == "mif-on") {
      digitalWrite(MISSION_POWER, HIGH);
    } else if (data_str == "mif-off") {
      digitalWrite(MISSION_POWER, LOW);
    } else if (data_str == "flash-start") {
      flash_mode = ACTIVE;
    } else if (data_str == "flash-stop") {
      flash_mode = SLEEP;
    } else if (data_str == "flash-clear") {
      flash.chipErase(false);  // non-blocking
      flash_mode = CLEARING;
      flash_index = 0;
      flash_addr = 0;
    } else {
      data_str += String(nose_temperature, 2);
      data_str += ",";
      data_str += String(nose_barometic_presure, 2);
      data_str += ",";
      data_str += String(nose_voltage, 4);
      data_str += ",";
      data_str += String(surface1_pressure, 2);
      data_str += ",";
      data_str += String(surface2_pressure, 2);
      data_str += ",";
      data_str += String(surface3_pressure, 2);
      data_str += ",";
      data_str += String(surface4_pressure, 2);
      data_str += ",";
      data_str += String(surface5_pressure, 2);
      data_str += ",";
      data_str += String(surface6_pressure, 2);
      data_str += ",";
      data_str += String(surface7_pressure, 2);
      data_str += ",";
      data_str += String(surface8_pressure, 2);
      data_str += ",\n";
      data_str.toCharArray(data_char, 256);
      flash_print(data_char);
    }
  }

  if (flash_mode == ACTIVE) {
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, HIGH);
  }
  if (flash_mode == SLEEP) {
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, HIGH);
  }
  if (flash_mode == CLEARING) {
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, LOW);
    if (!flash.IsBusy()) {
      flash_mode = SLEEP;
    }
  }

  if (!digitalRead(CAN_INT))  // データ受信確認
  {
    CCP.read_device();
    if (CCP.id < 0x40) {
      CCP.string(str_buf, 7);
      snprintf(msgString, sizeof(msgString), "%d,ID,%03x,time,%d000,string,%s,,,,", millis(), CCP.id, CCP.time16(), str_buf);
    } else if (CCP.id < 0x80) {
      snprintf(msgString, sizeof(msgString), "%d,ID,%03x,time,%lu,uint32,%lu,,,,", millis(), CCP.id, CCP.time32(), CCP.data_uint32());
    } else if (CCP.id < 0xC0) {
      snprintf(msgString, sizeof(msgString), "%d,ID,%03x,time,%lu,float,%8.2f,,,,", millis(), CCP.id, CCP.time32(), CCP.data_float());
    } else {
      snprintf(msgString, sizeof(msgString), "%d,ID,%03x,time,%d000,fp16_0,%8.2f,fp16_1,%8.2f,fp16_2,%8.2f", millis(), CCP.id, CCP.time16(), CCP.data_fp16_0(), CCP.data_fp16_1(), CCP.data_fp16_2());
    }
    Serial.println(msgString);


    switch (CCP.id) {
      case CCP_nose_status:
        if (CCP.str_match("OK", 2)) {
          status_byte |= 0b10000000;
        }
        break;
      case CCP_surface_pressure1_status:
        if (CCP.str_match("OK", 2)) {
          status_byte |= 0b01000000;
        }
        break;
      case CCP_surface_pressure2_status:
        if (CCP.str_match("OK", 2)) {
          status_byte |= 0b00100000;
        }
        break;
      case CCP_surface_pressure3_status:
        if (CCP.str_match("OK", 2)) {
          status_byte |= 0b00010000;
        }
        break;
      case CCP_surface_pressure4_status:
        if (CCP.str_match("OK", 2)) {
          status_byte |= 0b00001000;
        }
        break;
      case CCP_surface_pressure5_status:
        if (CCP.str_match("OK", 2)) {
          status_byte |= 0b00000100;
        }
        break;
      case CCP_surface_pressure6_status:
        if (CCP.str_match("OK", 2)) {
          status_byte |= 0b00000010;
        }
        break;
      case CCP_nose_adc:  //自信ない
        //adcの生データを16進数表示の文字列で送信
        for (int i = 0; i < 6; i++) {
          tx_payload[i + 5] = CCP.msg.string_msg.string[i];
        }
        break;
      case CCP_nose_temperature:
        buf.f = CCP.data_float();
        for (int i = 0; i < 4; i++) {
          tx_payload[i + 11] = buf.b[i];
        }
        break;
      case CCP_nose_barometic_pressure:
        buf.f = CCP.data_float();
        for (int i = 0; i < 4; i++) {
          tx_payload[i + 15] = buf.b[i];
        }
        break;
      case CCP_nose_voltage:
        buf.f = CCP.data_float();
        for (int i = 0; i < 4; i++) {
          tx_payload[i + 19] = buf.b[i];
        }
        break;
      case CCP_surface_pressure1_pressure_hPa:
        buf.f = CCP.data_float();
        for (int i = 0; i < 4; i++) {
          tx_payload[i + 23] = buf.b[i];
        }
        break;
      case CCP_surface_pressure2_pressure_hPa:
        buf.f = CCP.data_float();
        for (int i = 0; i < 4; i++) {
          tx_payload[i + 27] = buf.b[i];
        }
        break;
      case CCP_surface_pressure3_pressure_hPa:
        buf.f = CCP.data_float();
        for (int i = 0; i < 4; i++) {
          tx_payload[i + 31] = buf.b[i];
        }
        break;
      case CCP_surface_pressure4_pressure_hPa:
        buf.f = CCP.data_float();
        for (int i = 0; i < 4; i++) {
          tx_payload[i + 35] = buf.b[i];
        }
        break;
      case CCP_surface_pressure5_pressure_hPa:
        buf.f = CCP.data_float();
        for (int i = 0; i < 4; i++) {
          tx_payload[i + 39] = buf.b[i];
        }
        break;
      case CCP_surface_pressure6_pressure_hPa:
        buf.f = CCP.data_float();
        for (int i = 0; i < 4; i++) {
          tx_payload[i + 43] = buf.b[i];
        }
        break;
      case CCP_surface_pressure7_pressure_hPa:
        buf.f = CCP.data_float();
        for (int i = 0; i < 4; i++) {
          tx_payload[i + 47] = buf.b[i];
        }
        break;
      case CCP_surface_pressure8_pressure_hPa:
        buf.f = CCP.data_float();
        for (int i = 0; i < 4; i++) {
          tx_payload[i + 51] = buf.b[i];
        }
        break;
      default:
        break;
    }
  }

  mcutime_ms.i = millis();
  for (int i = 0; i < 4; i++) {
    tx_payload[i] = mcutime_ms.b[i];
  }
  status_byte = 0x00;
  tx_payload[4] = status_byte;

  static uint32_t last_send_time = 0;
  if (millis() - last_send_time >= 1000) {
    last_send_time = millis();
    e220.TransmissionDataVariebleLength(tx_payload, PAYLOAD_SIZE);
  }
}

void reset_tx_payload() {
  for (int i = 0; i < 11; i++) {
    tx_payload[i] = 0;
  }
  buf.f = 0;
  for (int data = 0; data < 11; data++) {
    for (int i = 0; i < 4; i++) {
      tx_payload[i + 11 + (4 * data)] = buf.b[i];
    }
  }
}
