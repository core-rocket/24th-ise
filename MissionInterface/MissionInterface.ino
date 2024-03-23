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

// CAN
CCP_MCP2515 CCP(CAN_CS, CAN_INT);

// Flash
#include <W25Q512.h>
#include "Queue.h"
W25Q512 flash(SPI, FLASH_CS);

typedef enum {
  SLEEP,
  ACTIVE,
  CLEARING
} Flash_Mode;
Flash_Mode flash_mode = SLEEP;

// Other
char msgString[128];
char str_buf[7];  // 6+\0

void setup() {
  delay(500);
  Serial.begin(115200);
  pinMode(CAN_CS, OUTPUT);
  pinMode(CAN_INT, INPUT);
  digitalWrite(CAN_CS, HIGH);

  pinMode(MISSION_POWER, OUTPUT);
  digitalWrite(MISSION_POWER, LOW);

  Serial1.begin(921600);
  while (Serial1.available()) {
    Serial1.read();
  }

  // CAN
  CCP.begin();

  flash.begin();

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  digitalWrite(LED_R, HIGH);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, HIGH);
}

void flash_print(char str[]) {
  static uint8_t flash_buf[256];
  static size_t flash_index = 0;
  static uint32_t flash_addr = 0;

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
  static uint8_t flash_buf[256];
  static size_t flash_index = 0;
  static uint32_t flash_addr = 0;

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
    } else {
      data_str += "\n";
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
  }
}
