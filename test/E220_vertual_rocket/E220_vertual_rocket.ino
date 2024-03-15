#include <E220.h>
#define CAN0_CS 0
#define CAN0_INT 1

#define SEND_PERIOD_MS 1000

E220 e220(0xFF, 0xFF, 0x00);  //TARGETADRESS=0xFFFF,CHANNEL=0x00

/*E220configuration
- UARTbaudrate:115200bps
- bandwith: 250kHz//審査書の値なので運営からの指示以外変更禁止
- channel: 0x0A(ARIB 34-35)//審査書の値なので運営からの指示以外変更禁止
- target address: 0xFFFF(broradcast)
- power: 13dBm
- SF: TBD
*/  


union unionfloat{
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
  Serial1.begin(9600);//E220のUART
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
    Serial.println("send!");
    e220.TransmissionData(tx_payload);
    latest_send_time = millis();//送信済みの時間を記録
    send_allowed = false;
  }
}

void GeneratePayload(byte* tx_payload,bool _payload_semapho){
  _payload_semapho = true;
  unionuint32 mcutime_ms;
  unionfloat buf;
  mcutime_ms.i = millis();
  byte status_byte = 0x00;

  buf.f = 1.1111;
  tx_payload[11] = buf.b[0];
  tx_payload[12] = buf.b[1];
  tx_payload[13] = buf.b[2];
  tx_payload[14] = buf.b[3];
  
  buf.f =2.2222;
  tx_payload[15] = buf.b[0];
  tx_payload[16] = buf.b[1];
  tx_payload[17] = buf.b[2];
  tx_payload[18] = buf.b[3];

  buf.f = 1.3232;
  tx_payload[19] = buf.b[0];
  tx_payload[20] = buf.b[1];
  tx_payload[21] = buf.b[2];
  tx_payload[22] = buf.b[3];
  
  buf.f = 0.2;
  tx_payload[43] = buf.b[0];
  tx_payload[44] = buf.b[1];
  tx_payload[45] = buf.b[2];
  tx_payload[46] = buf.b[3];       

  tx_payload[0] = mcutime_ms.b[0];
  tx_payload[1] = mcutime_ms.b[1];
  tx_payload[2] = mcutime_ms.b[2];
  tx_payload[3] = mcutime_ms.b[3];
  tx_payload[4] = 0x44;
  _payload_semapho = false;
}
