#include <E220.h>
#include<CCP.h>
#include <CCP_MCP2515.h>
#define CAN0_CS 0
#define CAN0_INT 1

#define SEND_PERIOD_MS 1000

E220 e220(0xFF, 0xFF, 0x00);  //TARGETADRESS=0xFFFF,CHANNEL=0x00
CCP_MCP2515 CCP(CAN0_CS, CAN0_INT);  //CAN

/*E220configuration
- UARTbaudrate:115200bps
- bandwith: 250kHz//審査書の値なので運営からの指示以外変更禁止
- channel: 0x0A(ARIB 34-35)//審査書の値なので運営からの指示以外変更禁止
- target address: 0xFFFF(broradcast)
- power: 13dBm
- SF: TBD
*/  
union uinonfloat{
  float f;
  byte b[4];
};
union unionuint32{
  uint32_t i;
  byte b[4];
};



void setup() {
  Serial1.setFIFOSize(512);  //E220のサブパケ200byteより大きいサイズにする
  Serial.begin(9600);
  Serial1.begin(115200);//E220のUART
  CCP.begin();
}

void loop() {
  static byte tx_payload[199] = { 0 };
  static bool send_allowed = false;//送信許可
  static bool payload_semapho = false;//payloadの生成が終わるまで送信を許可しない
  static uint32_t latest_send_time = 0;//最後に送信した時間
  GeneratePayload(tx_payload,payload_semapho);
  if((millis()-latest_send_time)>SEND_PERIOD_MS){//前の送信から一定時間経過しているか
    send_allowed = true;
  }
  if(send_allowed==true && payload_semapho==false){
    e220.TransmissionData(tx_payload);
    latest_send_time = millis();//送信済みの時間を記録
  }
  
  delay(2000);
}

void GeneratePayload(byte* tx_payload,bool _payload_semapho){
  _payload_semapho = true;
  unionuint32 mcutime_ms;
  CCP.read_device();
  mcutime_ms.i = millis();
  byte status_byte = 0x00;

  switch (CCP.id) {
        case CCP_nose_status:
          if(CCP.str_match("OK",2)){
            status_byte|=0b10000000;
          }        
          break;
        case CCP_surface_pressure1_status:
          if(CCP.str_match("OK",2)){
            status_byte|=0b01000000;
          }
          break;
        case CCP_surface_pressure2_status:
          if(CCP.str_match("OK",2)){
            status_byte|=0b00100000;
          }
          break;
        case CCP_surface_pressure3_status:
          if(CCP.str_match("OK",2)){
            status_byte|=0b00010000;
          }
          break;
        case CCP_surface_pressure4_status:
          if(CCP.str_match("OK",2)){
            status_byte|=0b00001000;
          }
          break;
        case CCP_surface_pressure5_status:
          if(CCP.str_match("OK",2)){
            status_byte|=0b00000100;
          }  
          break;
        case CCP_surface_pressure6_status:
          if(CCP.str_match("OK",2)){
            status_byte|=0b00000010;
          }       
          break;
        case CCP_nose_adc:
           //todo adcの生データを16進数表示の文字列で送信

          break;
        case CCP_nose_temperature:
          uninonfloat temperature;
          temperature.f = CCP.float_from_device();
          tx_payload[11] = temperature.b[0];
          tx_payload[12] = temperature.b[1];
          tx_payload[13] = temperature.b[2];
          tx_payload[14] = temperature.b[3];
          break;
        case CCP_nose_barometic_pressure:
          uninonfloat barometic_pressure;
          barometic_pressure.f = CCP.float_from_device();
          tx_payload[15] = barometic_pressure.b[0];
          tx_payload[16] = barometic_pressure.b[1];
          tx_payload[17] = barometic_pressure.b[2];
          tx_payload[18] = barometic_pressure.b[3];
          break;
        case CCP_nose_voltage:
          uninonfloat voltage;
          voltage.f = CCP.float_from_device();
          tx_payload[19] = voltage.b[0];
          tx_payload[20] = voltage.b[1];
          tx_payload[21] = voltage.b[2];
          tx_payload[22] = voltage.b[3];
          break;
        case CCP_surface_pressure1_pressure_pa:
          uninonfloat surface_pressure1;
          surface_pressure1.f = CCP.float_from_device();
          tx_payload[23] = surface_pressure1.b[0];
          tx_payload[24] = surface_pressure1.b[1];
          tx_payload[25] = surface_pressure1.b[2];
          tx_payload[26] = surface_pressure1.b[3];          
          break;
        case CCP_surface_pressure2_pressure_pa:
          uninonfloat surface_pressure2;
          surface_pressure2.f = CCP.float_from_device();
          tx_payload[27] = surface_pressure2.b[0];
          tx_payload[28] = surface_pressure2.b[1];
          tx_payload[29] = surface_pressure2.b[2];
          tx_payload[30] = surface_pressure2.b[3];
          break;
        case CCP_surface_pressure3_pressure_pa:
          uninonfloat surface_pressure3;
          surface_pressure3.f = CCP.float_from_device();
          tx_payload[31] = surface_pressure3.b[0];
          tx_payload[32] = surface_pressure3.b[1];
          tx_payload[33] = surface_pressure3.b[2];
          tx_payload[34] = surface_pressure3.b[3];
          break;
        case CCP_surface_pressure4_pressure_pa:
          uninonfloat surface_pressure4;
          surface_pressure4.f = CCP.float_from_device();
          tx_payload[35] = surface_pressure4.b[0];
          tx_payload[36] = surface_pressure4.b[1];
          tx_payload[37] = surface_pressure4.b[2];
          tx_payload[38] = surface_pressure4.b[3];
          break;
        case CCP_surface_pressure5_pressure_pa:
          uninonfloat surface_pressure5;
          surface_pressure5.f = CCP.float_from_device();
          tx_payload[39] = surface_pressure5.b[0];
          tx_payload[40] = surface_pressure5.b[1];
          tx_payload[41] = surface_pressure5.b[2];
          tx_payload[42] = surface_pressure5.b[3];
          break;
        case CCP_surface_pressure6_pressure_pa:
          uninonfloat surface_pressure6;
          surface_pressure6.f = CCP.float_from_device();
          tx_payload[43] = surface_pressure6.b[0];
          tx_payload[44] = surface_pressure6.b[1];
          tx_payload[45] = surface_pressure6.b[2];
          tx_payload[46] = surface_pressure6.b[3];       
          break;
        default:
          break;
      }
  tx_payload[0] = mcutime_ms.b[0];
  tx_payload[1] = mcutime_ms.b[1];
  tx_payload[2] = mcutime_ms.b[2];
  tx_payload[3] = mcutime_ms.b[3];
  tx_payload[4] = status_byte;
  _payload_semapho = false;
}
