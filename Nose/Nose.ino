#include <Wire.h>
#include <Adafruit_BME280.h>
#include <MCP342X.h>
#include <CCP_MCP2515.h>

#define CAN_AVAIRABLE

#define CAN0_CS 0
#define CAN0_INT 1
#define LED_YELLOW LED_BUILTIN
#define LED_BLUE PIN_LED_RXL

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;
MCP342X myADC;
CCP_MCP2515 CCP(CAN0_CS, CAN0_INT);  //CAN

const int clockFrequency = 400000;  //I2C bus speed
bool timer100Hz = false;
bool sleep_sensors = false;
bool can_checkerflag = false;
struct repeating_timer st_timer;

void setup() {
  Serial.begin(1843200);
  Wire.setSDA(6);
  Wire.setSCL(7);
  Wire.setClock(clockFrequency);
  Wire.begin();
  bme.begin(0x76);
  myADC.configure(MCP342X_MODE_CONTINUOUS | MCP342X_CHANNEL_1 | MCP342X_SIZE_18BIT | MCP342X_GAIN_1X);
  add_repeating_timer_us(10000, TimerIsr, NULL, &st_timer);  //100Hz
#ifdef CAN_AVAIRABLE
  CCP.begin();
#endif
}

void loop() {
  static int32_t result;
  static float temperature;
  static float barometic_pressure;
  static char adc_bytes[3];
  static double voltage;
  if (timer100Hz) {
    timer100Hz = false;
    if (!sleep_sensors) {
      //差圧センサ関連
      myADC.startConversion();
      myADC.getResult(&result);
      DevideBytes(&result, adc_bytes);
      ConvertToVoltage(adc_bytes, &voltage);  //3つのバイトを電圧に変換
      //BME280関連
      GetBME280Data(&temperature, &barometic_pressure);
//CAN送信
#ifdef CAN_AVAIRABLE
      CCP.uint32_to_device(CCP_nose_adc, voltage);
      CCP.float_to_device(CCP_nose_temperature, temperature);
      CCP.float_to_device(CCP_nose_barometic_pressure, barometic_pressure);
      if (can_checkerflag) {
        CCP.string_to_device(CCP_nose_status, "OK");
        can_checkerflag = false;
      }
#endif

      //シリアル出力
      SerialPrintSensors(adc_bytes, temperature, barometic_pressure, voltage);
    }
  }
#ifdef CAN_AVAIRABLE
  CCP.read_device();
  switch (CCP.id) {
    case CCP_EMST_mesure:
      if (CCP.str_match("STOP", 4)) {
        sleep_sensors = true;
      } else if (CCP.str_match("CLEAR", 5)) {
        sleep_sensors = false;
      }
      break;
    case CCP_nose_adc:
      if (CCP.str_match("CHECK", 5)) {
        can_checkerflag = true;
      }
      if (CCP.str_match("KILL", 4)) {
        sleep_sensors = true;
      }
      break;
    default:
      break;
  }
#endif
}

void GetBME280Data(float* temperature, float* barometic_pressure) {
  *temperature = bme.readTemperature();
  *barometic_pressure = bme.readPressure();
}

void DevideBytes(int32_t* _result, char* bytes) {
  bytes[2] = static_cast<char>(*_result & 0xFF);//(char)から変更．C-style castを使うとunsafeなコードになるので
  bytes[1] = static_cast<char>((*_result >> 8) & 0xFF);
  bytes[0] = static_cast<char>((*_result >> 16) & 0xFF);
}

void ConvertToVoltage(char* bytes, double* voltage) {
  double pga = 1;
  double lsb = 2 * 2.048 / pow(2, 18);

  byte msb = (bytes[0] >> 6) & 0x01;
  uint32_t outputcode = bytes[2] | (bytes[1] << 8) | ((bytes[0] * 0x01) << 16);
  if (msb == 0x00) {  //正の値
    *voltage = static_cast<double>(outputcode)*lsb / pga;
  } else {                                        //負の値
    outputcode = ((~outputcode) & 0x01FFFF) + 1;  //2の補数
    *voltage = -static_cast<double>(outputcode)*lsb / pga;
  }
}

void SerialPrintSensors(char* adc_bytes, float temperature, float barometic_pressure, double voltage) {
  // if(timer100Hz) Serial.println("overrun");
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

bool TimerIsr(struct repeating_timer* t) {
  timer100Hz = true;
  return true;
}