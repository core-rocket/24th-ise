#include <E220.h>

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
  Serial1.begin(115200);//E220のUART
}

void loop() {
  byte rx_payload[199] = { 0 };
  int rssi=0;
  int Rxlength=0;
  Rxlength=e220.ReceiveData(rx_payload,&rssi);
  unionuint32 mcutime_ms;
  byte nose_adc_raw[6]={0x00};
  unionfloat nose_temperature;
  unionfloat nose_barometic_presure;
  unionfloat nose_different_pressure;
  unionfloat nose_voltage;
  unionfloat surface1_pressure;
  unionfloat surface2_pressure;
  unionfloat surface3_pressure;
  unionfloat surface4_pressure;
  unionfloat surface5_pressure;
  unionfloat surface6_pressure;
  byte status=0x00;
  mcutime_ms.b[0] = rx_payload[0];
  mcutime_ms.b[1] = rx_payload[1];
  mcutime_ms.b[2] = rx_payload[2];
  mcutime_ms.b[3] = rx_payload[3];
  status = rx_payload[4];
  nose_adc_raw[0] = rx_payload[5];
  nose_adc_raw[1] = rx_payload[6];
  nose_adc_raw[2] = rx_payload[7];
  nose_adc_raw[3] = rx_payload[8];
  nose_adc_raw[4] = rx_payload[9];
  nose_adc_raw[5] = rx_payload[10];
  nose_temperature.b[0] = rx_payload[11];
  nose_temperature.b[1] = rx_payload[12];
  nose_temperature.b[2] = rx_payload[13];
  nose_temperature.b[3] = rx_payload[14];
  nose_barometic_presure.b[0] = rx_payload[15];
  nose_barometic_presure.b[1] = rx_payload[16];
  nose_barometic_presure.b[2] = rx_payload[17];
  nose_barometic_presure.b[3] = rx_payload[18];
  nose_different_pressure.b[0] = rx_payload[19];
  nose_different_pressure.b[1] = rx_payload[20];
  nose_different_pressure.b[2] = rx_payload[21];
  nose_different_pressure.b[3] = rx_payload[22];
  nose_voltage.b[0] = rx_payload[23];
  nose_voltage.b[1] = rx_payload[24];
  nose_voltage.b[2] = rx_payload[25];
  nose_voltage.b[3] = rx_payload[26];
  surface1_pressure.b[0] = rx_payload[27];
  surface1_pressure.b[1] = rx_payload[28];
  surface1_pressure.b[2] = rx_payload[29];
  surface1_pressure.b[3] = rx_payload[30];
  surface2_pressure.b[0] = rx_payload[31];
  surface2_pressure.b[1] = rx_payload[32];
  surface2_pressure.b[2] = rx_payload[33];
  surface2_pressure.b[3] = rx_payload[34];
  surface3_pressure.b[0] = rx_payload[35];
  surface3_pressure.b[1] = rx_payload[36];
  surface3_pressure.b[2] = rx_payload[37];
  surface3_pressure.b[3] = rx_payload[38];
  surface4_pressure.b[0] = rx_payload[39];
  surface4_pressure.b[1] = rx_payload[40];
  surface4_pressure.b[2] = rx_payload[41];
  surface4_pressure.b[3] = rx_payload[42];
  surface5_pressure.b[0] = rx_payload[43];
  surface5_pressure.b[1] = rx_payload[44];
  surface5_pressure.b[2] = rx_payload[45];
  surface5_pressure.b[3] = rx_payload[46];
  surface6_pressure.b[0] = rx_payload[47];
  surface6_pressure.b[1] = rx_payload[48];
  surface6_pressure.b[2] = rx_payload[49];
  surface6_pressure.b[3] = rx_payload[50];
  Serial.print("mcutime_ms:");


}
