#include <Wire.h>
#include <Adafruit_BME280.h>
#include <MCP342X.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;
MCP342X myADC;

const int clockFrequency = 400000;  //I2C bus speed

double voltage;

void setup() {
  Serial.begin(115200);
  unsigned status;
  Wire.setSDA(6);
  Wire.setSCL(7);
  Wire.setClock(clockFrequency);
  Wire.begin();
  status = bme.begin(0x76);
  myADC.configure(MCP342X_MODE_CONTINUOUS | MCP342X_CHANNEL_1 | MCP342X_SIZE_18BIT | MCP342X_GAIN_1X);
}


void loop() {
  printValues();

  static int32_t result;
  byte bytes[4];
  myADC.startConversion();
  myADC.getResult(&result);
  ConvertToVoltage(&result, &voltage);
  Serial.print("voltage:");
  Serial.println(voltage, 10);
  delay(5);
}


void printValues() {
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" °C");

  Serial.print("Pressure = ");

  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();
}

void ConvertToVoltage(int32_t* _result, double* voltage) {
  byte bytes[4];
  bytes[3] = static_cast<char>(*_result & 0xFF);
  bytes[2] = static_cast<char>((*_result >> 8) & 0xFF);
  bytes[1] = static_cast<char>((*_result >> 16) & 0xFF);
  bytes[0] = static_cast<char>((*_result >> 24) & 0xFF);  //ライブラリの関数内で8bit右シフトしたときに発生したものなので無視
  double pga = 1;
  double lsb = 2 * 2.048 / pow(2, 18);

  byte msb = (bytes[1] >> 6) & 0x01;
  uint32_t outputcode = bytes[3] | (bytes[2] << 8) | ((bytes[1] * 0x01) << 16);
  if (msb == 0x00) {  //正の値
    *voltage = static_cast<double>(outputcode)*lsb / pga;
  } else {                                        //負の値
    outputcode = ((~outputcode) & 0x01FFFF) + 1;  //2の補数
    *voltage = -static_cast<double>(outputcode)*lsb / pga;
  }
}
