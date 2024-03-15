#include <Wire.h>
#include <Adafruit_BME280.h>
#include <MCP342X.h>
#include <CCP_MCP2515.h>

#define CAN0_CS D0
#define CAN0_INT D1
#define LED_YELLOW LED_BUILTIN
#define LED_BLUE PIN_LED_RXL

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;
MCP342X myADC;
CCP_MCP2515 CCP(CAN0_CS, CAN0_INT);  //CAN

const int clockFrequency = 400000;  //I2C bus speed
bool timer100Hz = false;
bool timer1Hz=false;
bool can_checkerflag = false;

void setup() {
  Serial.begin(115200);
  Wire.setSDA(6);//rp2040の場合は必要
  Wire.setSCL(7);//rp2040の場合は必要
  Wire.setClock(clockFrequency);
  Wire.begin();
  bme.begin(0x76);
  myADC.configure(MCP342X_MODE_CONTINUOUS | MCP342X_CHANNEL_1 | MCP342X_SIZE_18BIT | MCP342X_GAIN_1X);
  CCP.begin();
}

void loop() {
  static int32_t result = 0x00;
  static float temperature = 0.0;
  static float barometic_pressure = 0.0;
  static char adc_bytes[3] = { 0x00, 0x00, 0x00 };
  static double voltage = 0.0;
  static uint32_t pretime_100Hz=0;
  static uint32_t pretime_1Hz=0;

  //100Hz用
  if(millis()-pretime_100Hz>10){
    timer100Hz=true;
    pretime_100Hz=millis();
  }else{
    timer100Hz=false;
  }
  //1Hz用
  if(millis()-pretime_1Hz>1000){
    timer1Hz=true;
    pretime_1Hz=millis();
  }else{
    timer1Hz=false;
  }

  if (timer100Hz) {
    timer100Hz = false;
    //差圧センサ関連
    myADC.startConversion();
    myADC.getResult(&result);
    DevideBytes(&result, adc_bytes);
    ConvertToVoltage(adc_bytes, &voltage);  //3つのバイトを電圧に変換
    //BME280関連
    GetBME280Data(&temperature, &barometic_pressure);
    //CAN送信
    CCP.float_to_device(CCP_nose_temperature, temperature);
    CCP.float_to_device(CCP_nose_barometic_pressure, barometic_pressure);
    CCP.float_to_device(CCP_nose_voltage, static_cast<float>(voltage));
    CCP.string_to_device(CCP_nose_adc, adc_bytes);
    if(timer1Hz){
        CCP.string_to_device(CCP_nose_status, "OK");
    }
    //シリアル出力
    SerialPrintSensors(adc_bytes, temperature, barometic_pressure, voltage);
  }
}

void GetBME280Data(float* temperature, float* barometic_pressure) {
  *temperature = bme.readTemperature();
  *barometic_pressure = bme.readPressure();
}

void DevideBytes(int32_t* _result, char* bytes) {
  bytes[2] = static_cast<char>(*_result & 0xFF);  //(char)から変更．C-style castを使うとunsafeなコードになるので
  bytes[1] = static_cast<char>((*_result >> 8) & 0xFF);
  bytes[0] = static_cast<char>((*_result >> 16) & 0xFF);
}

void ConvertToVoltage(char* bytes, double* voltage) {
  double pga = 1;
  double lsb = 2 * 2.048 / pow(2, 18);

  byte msb = (bytes[0] >> 6) & 0x01;
  uint32_t outputcode = bytes[2] | (bytes[1] << 8) | ((bytes[0] * 0x01) << 16);
  if (msb == 0x00) {  //正の値
    *voltage = static_cast<double>(outputcode) * lsb / pga;
  } else {                                        //負の値
    outputcode = ((~outputcode) & 0x01FFFF) + 1;  //2の補数
    *voltage = -static_cast<double>(outputcode) * lsb / pga;
  }
}

void SerialPrintSensors(char* adc_bytes, float temperature, float barometic_pressure, double voltage) {
  Serial.print("time:");
  Serial.print(micros());
  Serial.print(",adc_bytes:");
  Serial.print(adc_bytes[0], HEX);
  Serial.print(adc_bytes[1], HEX);
  Serial.print(adc_bytes[2], HEX);
  Serial.print(",temperature:");
  Serial.print(temperature, 10);
  Serial.print(",");
  Serial.print(",barometic_pressure:");
  Serial.print(barometic_pressure, 10);
  Serial.print(",voltage:");
  Serial.println(voltage, 10);
}
