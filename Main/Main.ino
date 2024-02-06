// global data
float data_bno_accel_x_mss = 0;
float data_bno_accel_y_mss = 0;
float data_bno_accel_z_mss = 0;
float data_bme_pressure_hPa = 0;
float data_bme_temperature_degC = 0;
float data_bme_altitude_m = 0;
uint32_t data_gnss_latitude_udeg = 0;
uint32_t data_gnss_longitude_udeg = 0;

// pinout
const uint8_t BNO_SDA = 4;
const uint8_t BNO_SCL = 5;
const uint8_t VALVE_TX = 6;
const uint8_t VALVE_RX = 7;
const uint8_t ES920_TX = 8;
const uint8_t ES920_RX = 9;
const uint8_t BME_SDA = 10;
const uint8_t BME_SCL = 11;
const uint8_t E220_TX = 12;
const uint8_t E220_RX = 13;
const uint8_t GNSS_TX = 14;
const uint8_t GNSS_RX = 15;
const uint8_t KEY_SW = 20;
const uint8_t SERVO = 21;
const uint8_t EXT_V = 26;
const uint8_t BAT_V = 27;

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
#include <TinyGPS++.h>
TinyGPSPlus gps;
SerialPIO Serial_GNSS(GNSS_TX, GNSS_RX, 512);
// https://arduino-pico.readthedocs.io/en/latest/piouart.html

// ES920LR
#define Serial_ES920 Serial2
// https://moyoi-memo.hatenablog.com/entry/2022/02/15/112100

// W25Q128



// setup()ではdelay()使用可
void setup() {
  // デバッグ出力
  Serial.begin(115200);

  Wire.setSDA(BNO_SDA);
  Wire.setSCL(BNO_SCL);
  while (!bno.begin())
  {
    Serial.print("BNO055 ERR");
    delay(100);
  }
  
  Wire1.setSDA(BME_SDA);
  Wire1.setSCL(BME_SCL);
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

  Serial_ES920.setTX(ES920_TX);
  Serial_ES920.setRX(ES920_RX);
  Serial_ES920.setFIFOSize(64);
  Serial_ES920.begin(115200);

  Serial_GNSS.begin(9600);
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
  while (Serial_GNSS.available()) {
    gps.encode(Serial_GNSS.read());
    if (gps.location.isUpdated()) {
      data_gnss_latitude_udeg = gps.location.lat() * 1000000;
      data_gnss_longitude_udeg = gps.location.lng() * 1000000;
    }
  }
}
