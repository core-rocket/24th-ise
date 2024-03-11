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
  GeneratePayload(payload_semapho);
  if((millis()-latest_send_time)>SEND_PERIOD_MS){//前の送信から一定時間経過しているか
    send_allowed = true;
  }
  if(send_allowed==true && payload_semapho==false){
    e220.TransmissionData(tx_payload);
    latest_send_time = millis();//送信済みの時間を記録
  }
  
  delay(2000);
}

void GeneratePayload(bool _payload_semapho){
  _payload_semapho = true;
  CCP.read_device();
  //TODO canのプロトコル整理・CCPの変更
  //TODO 送信データのプロトコルの整理
  // switch (CCP.id) {
  //       case :

  //         break;
  //       case CCP_nose_adc:

  //         break;
  //       default:
  //         updated_flag = false;
  //         break;
  //     }
  _payload_semapho = false;
}
