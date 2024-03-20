#include <E220.h>

#define SEND_PERIOD_MS 1000  //E220のダウンリンクの周期(ms)

#define PAYLOAD_SIZE 47
#define ROCKET_INSIDE_PACKET_LETTER 0x4E

E220 e220(Serial2, 0xFF, 0xFF, 0x0A);  //TARGETADRESS=0xFFFF,CHANNEL=0x0A
CCP_MCP2515 CCP(CAN0_CS, CAN0_INT);    //CAN


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
  Serial1.begin(115200);  //MIFとつながってるUART
  Serial2.begin(115200);  //E220のUART
  CCP.begin();
}

void loop() {
  static byte tx_payload[199] = { 0 };   //MIF基板から送られるbyte配列格納先
  static bool send_allowed = false;      //送信許可
  static uint32_t latest_send_time = 0;  //最後に送信した時間
  static bool comingdata = false;        //MIF基板からデータが来ているか

  //MIF基板からのデータを受け取ってtx_payloadにいれる処理
  if (Serial1.read() == ROCKET_INSIDE_PACKET_LETTER) {
    for (int i = 0; i < PAYLOAD_SIZE; i++) {
      tx_payload[i] = Serial1.read();
    }
    comingdata = true;
  } else {
    comingdata = false;
  }
  //ここから送信処理
  if (((millis() - latest_send_time) > SEND_PERIOD_MS) && comingdata) {  //前の送信から一定時間経過しているか
    send_allowed = true;
  }
  if (send_allowed == true) {
    e220.TransmissionDataVariebleLength(tx_payload, PAYLOAD_SIZE);
    latest_send_time = millis();  //送信済みの時間を記録
    send_allowed = false;
  }
}
