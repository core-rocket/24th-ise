
#include <Wire.h>
#include <MCP342X.h>

MCP342X myADC;
double voltage;
void setup() {
  Wire.begin();        // join I2C bus
                       //   TWBR = 12;  // 400 kHz (maximum)
  Serial.begin(9600);  // Open serial connection to send info to the host
  myADC.configure(MCP342X_MODE_CONTINUOUS | MCP342X_CHANNEL_1 | MCP342X_SIZE_18BIT | MCP342X_GAIN_1X);
  delay(300);
}

void loop() {
  static int32_t result;
  byte bytes[4];
  myADC.startConversion();
  myADC.getResult(&result);
  ConvertToVoltage(&result, &voltage);

  Serial.print("voltage:");
  Serial.println(voltage);
}
void ConvertToVoltage(int32_t* _result, double* voltage) {
  byte bytes[4];
  bytes[3] = (char)(*_result & 0xFF);
  bytes[2] = (char)((*_result >> 8) & 0xFF);
  bytes[1] = (char)((*_result >> 16) & 0xFF);
  bytes[0] = (char)((*_result >> 24) & 0xFF);  //ライブラリの関数内で8bit右シフトしたときに発生したものなので無視
  double pga = 1;
  double lsb = 2 * 2.048 / pow(2, 18);

  byte msb = (bytes[1] >> 6) & 0x01;
  uint32_t outputcode = bytes[3] | (bytes[2] << 8) | ((bytes[1] * 0x01) << 16);
  if (msb == 0x00) {  //正の値
    *voltage = (double)(outputcode)*lsb / pga;
  } else {                                        //負の値
    outputcode = ((~outputcode) & 0x01FFFF) + 1;  //2の補数
    *voltage = -(double)(outputcode)*lsb / pga;
  }
}