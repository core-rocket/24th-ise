#include <SPI.h>
#include <string.h>
#include <CCP_MCP2515.h>

#define CAN_INT D1
#define CAN_CS D0
#define MISSION_POWER D3

// CAN
CCP_MCP2515 CCP(CAN_CS, CAN_INT);

// Other
char msgString[128];
char str_buf[7]; // 6+\0

void setup()
{
  delay(500);
  Serial.begin(115200);
  pinMode(CAN_CS, OUTPUT);
  pinMode(CAN_INT, INPUT);
  digitalWrite(CAN_CS, HIGH);

  pinMode(MISSION_POWER, OUTPUT);
  digitalWrite(MISSION_POWER, HIGH);

  // CAN
  CCP.begin();
}

void loop()
{
  if (!digitalRead(CAN_INT)) // データ受信確認
  {
    CCP.read_device();
    if (CCP.id < 0x40)
    {
      CCP.string(str_buf, 7);
      sprintf(msgString, "%d,ID,%03x,time,%d000,string,%s,,,,", millis(), CCP.id, CCP.time16(), str_buf);
    }
    else if (CCP.id < 0x80)
    {
      sprintf(msgString, "%d,ID,%03x,time,%lu,uint32,%lu,,,,", millis(), CCP.id, CCP.time32(), CCP.data_uint32());
    }
    else if (CCP.id < 0xC0)
    {
      sprintf(msgString, "%d,ID,%03x,time,%lu,float,%8.2f,,,,", millis(), CCP.id, CCP.time32(), CCP.data_float());
    }
    else
    {
      sprintf(msgString, "%d,ID,%03x,time,%d000,fp16_0,%8.2f,fp16_1,%8.2f,fp16_2,%8.2f", millis(), CCP.id, CCP.time16(), CCP.data_fp16_0(), CCP.data_fp16_1(), CCP.data_fp16_2());
    }
    Serial.println(msgString);
  }
}