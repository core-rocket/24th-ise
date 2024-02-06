// global data
float data_bno_accel_x_mss = 0;
float data_bno_accel_y_mss = 0;
float data_bno_accel_z_mss = 0;
float data_bme_pressure_hPa = 0;
float data_bme_temperature_degC = 0;
float data_bme_altitude_m = 0;
uint32_t data_gnss_latitude_udeg = 0;
uint32_t data_gnss_longitude_udeg = 0;

// Servo


// KeySW


// BNO055
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
sensors_event_t  accelerometerData;

// BME280
#include <Adafruit_BME280.h>
Adafruit_BME280 bme;
const float SEALEVELPRESSURE_HPA = 1013.25;

// SAM-M8Q


// ES920LR


// W25Q128



// setup()ではdelay()使用可
void setup() {
  // デバッグ出力
  Serial.begin(115200);

  while (!bno.begin())
  {
    Serial.print("BNO055 ERR");
    delay(100);
  }
  
  Wire1.setSDA(10);
  Wire1.setSCL(11);
  while (!bme.begin(0x76, &Wire1))
  {
    Serial.print("BME280 ERR");
    delay(100);
  }
  bme.setSampling(
    Adafruit_BME280::MODE_NORMAL,
    Adafruit_BME280::SAMPLING_X1,
    Adafruit_BME280::SAMPLING_X2,
    Adafruit_BME280::SAMPLING_NONE,
    Adafruit_BME280::FILTER_X16,
    Adafruit_BME280::STANDBY_MS_0_5
  );

}

// loop()と，ここから呼び出される関数ではdelay()使用禁止
void loop() {
  static uint32_t time_100Hz = 0;
  if (millis() - time_100Hz >= 10) {
    time_100Hz += 10;

    static int count_10Hz = 0;
    count_10Hz++;
    if (count_10Hz > 10) {
      count_10Hz = 0;

      // 10Hzで実行する処理

      // デバッグ出力
      Serial.print("BNO055, ");
      Serial.print(data_bno_accel_x_mss);
      Serial.print(", ");
      Serial.print(data_bno_accel_y_mss);
      Serial.print(", ");
      Serial.print(data_bno_accel_z_mss);
      Serial.print(", ");

      Serial.print("BME280, ");
      Serial.print(data_bme_pressure_hPa);
      Serial.print(", ");
      Serial.print(data_bme_temperature_degC);
      Serial.print(", ");
      Serial.print(data_bme_altitude_m);
      Serial.print(", ");

      Serial.print("GNSS, ");
      Serial.print(data_gnss_latitude_udeg);
      Serial.print(", ");
      Serial.print(data_gnss_latitude_udeg);
      Serial.print(", ");
      Serial.print("\n");
    }

    // 100Hzで実行する処理


    // BNO055から100Hzで測定
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    data_bno_accel_x_mss = accelerometerData.acceleration.x;
    data_bno_accel_y_mss = accelerometerData.acceleration.y;
    data_bno_accel_z_mss = accelerometerData.acceleration.z;

    // BME280から100Hzで測定
    data_bme_pressure_hPa = bme.readPressure() / 100.0F;
    data_bme_temperature_degC = bme.readTemperature();
    data_bme_altitude_m = bme.readAltitude(SEALEVELPRESSURE_HPA);
  }

  // 常に実行する処理

  // SAM-M8QのUARTを常に読み出し
  // ～測定処理～
  // data_gnss_latitude_udeg = ~;
  // data_gnss_longitude_udeg = ~;
}
