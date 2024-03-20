#include <E220.h>
#include <CCP.h>
#include <CCP_MCP2515.h>
#define CAN0_CS 0
#define CAN0_INT 1

#define PAYLOAD_SIZE 47
#define ROCKET_INSIDE_PACKET_LETTER 0x4E

CCP_MCP2515 CCP(CAN0_CS, CAN0_INT);  //CAN

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

void setup() {
  Serial1.setFIFOSize(512);  //E220のサブパケ200byteより大きいサイズにする
  Serial.begin(9600);
  Serial1.begin(115200);  //E220のUART
  CCP.begin();
}

void loop() {
  static byte tx_payload[199] = { 0 };
  static byte rocket_inside_packet[200] = { 0 };
  static uint32_t last_send_time = 0;
  bool send_allowed = false;

  GeneratePayload(tx_payload);
  //tx_payloadをMain基板に送信する処理
  if (millis() - last_send_time > 1000) {
    send_allowed = true;
    last_send_time = millis();
  } else {
    send_allowed = false;
  }
  if (send_allowed == true) {
    SendMainDataPacket(tx_payload);
  }
}

void SendMainDataPacket(byte* _tx_payload) {
  byte rocket_inside_packet[200] = { 0 };
  rocket_inside_packet[0] = ROCKET_INSIDE_PACKET_LETTER;  // Mainマイコンがどっからデータか判断するための識別子
  for (int i = 0; i < PAYLOAD_SIZE; i++) {
    rocket_inside_packet[i + 1] = _tx_payload[i];
  }
  //main基板に送信
  Serial1.write(rocket_inside_packet, 200);  //一個目が識別子，後ろがデータ
}

void GeneratePayload(byte* tx_payload) {
  unionuint32 mcutime_ms;
  unionfloat buf;
  CCP.read_device();
  mcutime_ms.i = millis();
  byte status_byte = 0x00;

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
    case CCP_surface_pressure1_pressure_pa:
      buf.f = CCP.data_float();
      for (int i = 0; i < 4; i++) {
        tx_payload[i + 23] = buf.b[i];
      }
      break;
    case CCP_surface_pressure2_pressure_pa:
      buf.f = CCP.data_float();
      for (int i = 0; i < 4; i++) {
        tx_payload[i + 27] = buf.b[i];
      }
      break;
    case CCP_surface_pressure3_pressure_pa:
      buf.f = CCP.data_float();
      for (int i = 0; i < 4; i++) {
        tx_payload[i + 31] = buf.b[i];
      }
      break;
    case CCP_surface_pressure4_pressure_pa:
      buf.f = CCP.data_float();
      for (int i = 0; i < 4; i++) {
        tx_payload[i + 35] = buf.b[i];
      }
      break;
    case CCP_surface_pressure5_pressure_pa:
      buf.f = CCP.data_float();
      for (int i = 0; i < 4; i++) {
        tx_payload[i + 39] = buf.b[i];
      }
      break;
    case CCP_surface_pressure6_pressure_pa:
      buf.f = CCP.data_float();
      for (int i = 0; i < 4; i++) {
        tx_payload[i + 43] = buf.b[i];
      }
      break;
    default:
      break;
  }
  for (int i = 0; i < 4; i++) {
    tx_payload[i] = mcutime_ms.b[i];
  }
  tx_payload[4] = status_byte;
}
